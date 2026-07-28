/**
 * @module dynamics/World
 *
 * The simulation container: owns the bodies, fixtures, contacts, joints, the
 * broad phase and the solver, and drives one deterministic step at a time.
 *
 * ```ts
 * const world = new World({ gravity: { x: 0, y: -10 } });
 *
 * const ground = world.createBody({ type: BodyType.Static });
 * ground.addFixture({ shape: Polygon.box(50, 1) });
 *
 * const box = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 10 } });
 * box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
 *
 * for (let i = 0; i < 60; i++) world.step();
 * ```
 *
 * ### Fixed time step
 *
 * `step()` takes **no delta time** by default: the step is fixed at
 * construction. This is deliberate — a variable `dt` makes results depend on
 * frame rate, which destroys both determinism and stability. Use
 * {@link World#accumulate} to drive a fixed simulation from a variable render
 * loop.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { AABB } from './../collision/AABB.js';
import { BroadPhase } from './../collision/BroadPhase.js';
import { Body, BodyType } from './Body.js';
import type { BodyDef } from './Body.js';
import { Fixture } from './Fixture.js';
import type { FixtureDef } from './Fixture.js';
import { Contact, ContactFlags } from './Contact.js';
import { shouldCollide } from './Filter.js';
import { Solver } from './Solver.js';
import { solveContinuous } from './Continuous.js';
import type { StepContext } from './Solver.js';
import { Joint } from './joints/Joint.js';
import { RevoluteJoint } from './joints/RevoluteJoint.js';
import type { RevoluteJointDef } from './joints/RevoluteJoint.js';
import { DistanceJoint } from './joints/DistanceJoint.js';
import type { DistanceJointDef } from './joints/DistanceJoint.js';
import { PrismaticJoint } from './joints/PrismaticJoint.js';
import type { PrismaticJointDef } from './joints/PrismaticJoint.js';
import { WeldJoint } from './joints/WeldJoint.js';
import type { WeldJointDef } from './joints/WeldJoint.js';
import { MouseJoint } from './joints/MouseJoint.js';
import type { MouseJointDef } from './joints/MouseJoint.js';
import { MotorJoint } from './joints/MotorJoint.js';
import type { MotorJointDef } from './joints/MotorJoint.js';
import {
  DEFAULT_VELOCITY_ITERATIONS,
  DEFAULT_RELAX_ITERATIONS,
  TIME_TO_SLEEP,
  AABB_MARGIN,
  SPECULATIVE_DISTANCE,
} from './../util/settings.js';
import type { RayCastInput, RayCastOutput } from './../collision/Shape.js';
import { Rng } from './../math/rng.js';

/* ------------------------------------------------------------------ *
 * Events
 * ------------------------------------------------------------------ */

/** Fired when two fixtures start or stop touching. */
export interface ContactEvent {
  fixtureA: Fixture;
  fixtureB: Fixture;
  contact: Contact;
}

/** Fired after the solve, carrying the impulses that were applied. */
export interface ImpactEvent {
  fixtureA: Fixture;
  fixtureB: Fixture;
  contact: Contact;
  /** Largest normal impulse over the manifold points, N·s. */
  maxNormalImpulse: Scalar;
  /** Approach speed before the impact, m/s (negative = closing). */
  approachSpeed: Scalar;
}

/**
 * Listener bundle passed to {@link World#setListener}.
 *
 * ⚠️ **Event objects are pooled and reused.** The record handed to a callback
 * is only valid for the duration of that call — copy anything you need to
 * keep, and never store the event itself:
 *
 * ```ts
 * beginContact(e) {
 *   queue.push(e);                        // ✗ every entry aliases one record
 *   queue.push({ a: e.fixtureA.id });     // ✓ copy what you need
 * }
 * ```
 */
export interface WorldListener {
  /** Two fixtures began touching. */
  beginContact?(e: ContactEvent): void;
  /** Two fixtures stopped touching. */
  endContact?(e: ContactEvent): void;
  /**
   * Called after the manifold is computed but before the solve.
   * Call `contact.setEnabled(false)` to ignore this contact for one step —
   * the standard way to implement one-way platforms.
   */
  preSolve?(e: ContactEvent): void;
  /** Called after the solve with the resulting impulses. */
  postSolve?(e: ImpactEvent): void;
  /** A sensor started overlapping. */
  beginSensor?(e: ContactEvent): void;
  /** A sensor stopped overlapping. */
  endSensor?(e: ContactEvent): void;
}

/** Options for the {@link World} constructor. */
export interface WorldDef {
  /** Gravity, m/s². Default `(0, -10)`. */
  gravity?: { x: number; y: number };
  /** Fixed step duration, seconds. Default `1/60`. */
  timeStep?: number;
  /**
   * Sub-steps per call to {@link World#step}. More sub-steps means stiffer
   * stacks and better fast-motion handling at linear cost. Default `4`.
   */
  subSteps?: number;
  /**
   * Biased velocity iterations per sub-step. Default `2`.
   * Prefer raising {@link WorldDef.subSteps} over this.
   */
  velocityIterations?: number;
  /** Relax iterations per sub-step. Default `1`. */
  relaxIterations?: number;
  /** Allow bodies to sleep. Default `true`. */
  enableSleep?: boolean;
  /** Enable warm starting. Default `true`; turning it off is for debugging. */
  enableWarmStarting?: boolean;
  /** Enable the restitution pass. Default `true`. */
  enableRestitution?: boolean;
  /** Seed for the world's deterministic RNG. */
  seed?: number;
}

/* ------------------------------------------------------------------ *
 * World
 * ------------------------------------------------------------------ */

const _aabb = new AABB();
const _tmpAABB = new AABB();
const _disp = Vec2.zero();
const _tmp = Vec2.zero();

export class World {
  /** Gravity, m/s². Mutable, but must match on every peer. */
  readonly gravity = Vec2.zero();

  /** Fixed step duration. */
  readonly timeStep: Scalar;
  /** Sub-steps per step. */
  subSteps: number;
  velocityIterations: number;
  relaxIterations: number;
  enableSleep: boolean;
  enableWarmStarting: boolean;
  enableRestitution: boolean;

  /**
   * Deterministic RNG bound to this world. Use it instead of `Math.random`
   * for anything that affects the simulation — it is captured in snapshots
   * and rewound correctly during rollback.
   */
  readonly rng: Rng;

  /** Bodies, indexed by id. Holes are `null` after destruction. */
  readonly bodies: (Body | null)[] = [];
  /** Fixtures, indexed by id. */
  readonly fixtures: (Fixture | null)[] = [];
  /** Live contacts, in a canonical order. */
  readonly contacts: Contact[] = [];
  /** Joints, indexed by id. */
  readonly joints: (Joint | null)[] = [];

  /** Broad phase. */
  readonly broadPhase: BroadPhase;
  /** Solver. */
  readonly solver = new Solver();

  /** Monotonically increasing step counter. The lockstep "tick". */
  tick = 0;

  /**
   * `1 / subStepDuration`.
   *
   * Joint impulse accumulators are per **sub-step**, so this — not `1/timeStep`
   * — is the factor that converts them into forces:
   *
   * ```ts
   * joint.getReactionForce(out, world.invSubStep);   // newtons
   * ```
   */
  get invSubStep(): Scalar {
    return this.ctx.invH;
  }
  /** Accumulated simulated time. */
  time: Scalar = S.ZERO;

  /** Scratch vector for user-facing helpers. @internal */
  readonly scratchPoint = Vec2.zero();

  /**
   * Reusable event records.
   *
   * `preSolve` fires once per touching contact per step — with a couple of
   * thousand contacts that is a couple of thousand short-lived objects per
   * frame, and the resulting GC churn showed up as ~5% of total step time.
   * The records are therefore pooled and refilled.
   *
   * The documented contract is that an event object is only valid for the
   * duration of the callback; copy anything you need to keep.
   * @internal
   */
  private readonly _contactEvent: ContactEvent = {
    fixtureA: null as unknown as Fixture,
    fixtureB: null as unknown as Fixture,
    contact: null as unknown as Contact,
  };
  private readonly _impactEvent: ImpactEvent = {
    fixtureA: null as unknown as Fixture,
    fixtureB: null as unknown as Fixture,
    contact: null as unknown as Contact,
    maxNormalImpulse: S.ZERO,
    approachSpeed: S.ZERO,
  };

  /** Fill and return the shared contact-event record. */
  private contactEvent(fixtureA: Fixture, fixtureB: Fixture, contact: Contact): ContactEvent {
    const e = this._contactEvent;
    e.fixtureA = fixtureA;
    e.fixtureB = fixtureB;
    e.contact = contact;
    return e;
  }

  private listener: WorldListener | null = null;
  private freeBodyIds: number[] = [];
  private freeFixtureIds: number[] = [];
  private freeJointIds: number[] = [];
  /** Map from `min(idA,idB) * 2^20 + max` to the contact index. */
  private contactMap = new Map<number, Contact>();
  /** Set when a contact was added, so the canonical order is restored. */
  private contactsDirty = false;
  private ctx: StepContext;
  private awakeBodies: Body[] = [];
  private stepAccumulator: Scalar = S.ZERO;

  /** Per-step profiling, in milliseconds. */
  readonly profile = {
    total: 0,
    broadPhase: 0,
    narrowPhase: 0,
    solve: 0,
    continuous: 0,
    bodyCount: 0,
    contactCount: 0,
    activeContactCount: 0,
  };

  constructor(def: WorldDef = {}) {
    this.gravity.set(S.fromFloat(def.gravity?.x ?? 0), S.fromFloat(def.gravity?.y ?? -10));
    this.timeStep = S.fromFloat(def.timeStep ?? 1 / 60);
    this.subSteps = Math.max(1, def.subSteps ?? 4);
    this.velocityIterations = def.velocityIterations ?? DEFAULT_VELOCITY_ITERATIONS;
    this.relaxIterations = def.relaxIterations ?? DEFAULT_RELAX_ITERATIONS;
    this.enableSleep = def.enableSleep ?? true;
    this.enableWarmStarting = def.enableWarmStarting ?? true;
    this.enableRestitution = def.enableRestitution ?? true;
    this.rng = new Rng(def.seed ?? 0x9e3779b9);
    this.broadPhase = new BroadPhase(256);

    const h = S.divInt(this.timeStep, this.subSteps);
    this.ctx = {
      dt: this.timeStep,
      h,
      invDt: this.timeStep > S.ZERO ? S.inv(this.timeStep) : S.ZERO,
      invH: h > S.ZERO ? S.inv(h) : S.ZERO,
      velocityIterations: this.velocityIterations,
      relaxIterations: this.relaxIterations,
      gravity: this.gravity,
      enableSleep: this.enableSleep,
      enableWarmStarting: this.enableWarmStarting,
      enableRestitution: this.enableRestitution,
    };
  }

  /** Install the event listener. Pass `null` to remove it. */
  setListener(listener: WorldListener | null): void {
    this.listener = listener;
  }

  /* ------------------------- construction ------------------------- */

  /** Create a body. */
  createBody(def: BodyDef = {}): Body {
    const id = this.freeBodyIds.length > 0 ? this.freeBodyIds.pop()! : this.bodies.length;
    const body = new Body(id, this, def);
    if (id === this.bodies.length) this.bodies.push(body);
    else this.bodies[id] = body;
    return body;
  }

  /**
   * Destroy a body along with its fixtures, contacts and joints.
   * The body object must not be used afterwards.
   */
  destroyBody(body: Body): void {
    // Joints first — they hold references to the body.
    for (let i = this.joints.length - 1; i >= 0; i--) {
      const j = this.joints[i];
      if (j && (j.bodyA === body || j.bodyB === body)) this.destroyJoint(j);
    }
    // Then contacts.
    for (let i = this.contacts.length - 1; i >= 0; i--) {
      const c = this.contacts[i]!;
      if (c.fixtureA.body === body || c.fixtureB.body === body) this.removeContactAt(i);
    }
    // Then fixtures.
    for (const f of body.fixtures.slice()) this.unregisterFixture(f);
    body.fixtures.length = 0;

    this.bodies[body.id] = null;
    this.freeBodyIds.push(body.id);
  }

  /** @internal Allocate a fixture id and construct the fixture. */
  registerFixture(body: Body, def: FixtureDef): Fixture {
    const id = this.freeFixtureIds.length > 0 ? this.freeFixtureIds.pop()! : this.fixtures.length;
    const fixture = new Fixture(id, body, def);
    if (id === this.fixtures.length) this.fixtures.push(fixture);
    else this.fixtures[id] = fixture;
    return fixture;
  }

  /** @internal Remove a fixture, its proxy and its contacts. */
  unregisterFixture(fixture: Fixture): void {
    for (let i = this.contacts.length - 1; i >= 0; i--) {
      const c = this.contacts[i]!;
      if (c.fixtureA === fixture || c.fixtureB === fixture) this.removeContactAt(i);
    }
    this.destroyFixtureProxy(fixture);
    this.fixtures[fixture.id] = null;
    this.freeFixtureIds.push(fixture.id);
  }

  /** @internal Insert a fixture's broad-phase proxy. */
  createFixtureProxy(fixture: Fixture): void {
    if (fixture.proxyId >= 0) return;
    fixture.synchronize();
    fixture.proxyId = this.broadPhase.createProxy(fixture.aabb, fixture.id);
  }

  /** @internal Remove a fixture's broad-phase proxy. */
  destroyFixtureProxy(fixture: Fixture): void {
    if (fixture.proxyId < 0) return;
    this.broadPhase.destroyProxy(fixture.proxyId);
    fixture.proxyId = -1;
  }

  /** @internal Refresh every proxy of a body after a teleport. */
  synchronizeFixtures(body: Body): void {
    for (const f of body.fixtures) {
      if (f.proxyId < 0) continue;
      f.synchronize();
      this.broadPhase.moveProxy(f.proxyId, f.aabb, _disp.setZero());
    }
  }

  /**
   * Rebuild the broad phase from the current body transforms.
   *
   * Called by {@link loadSnapshot}. Also useful after teleporting a large
   * number of bodies at once, e.g. when loading a level.
   *
   * @param discardContacts also drop the contact list, so pairs are
   *        rediscovered from the restored geometry. Required after a snapshot
   *        load: the existing contacts belong to the timeline being abandoned,
   *        and any pair that was destroyed since the snapshot would otherwise
   *        never come back.
   */
  rebuildBroadPhase(discardContacts = false): void {
    if (discardContacts) {
      this.contacts.length = 0;
      this.contactMap.clear();
    }
    const live: Fixture[] = [];
    for (const f of this.fixtures) {
      if (f && f.proxyId >= 0) {
        f.synchronize();
        live.push(f);
      }
    }
    this.broadPhase.rebuild(live);

    if (discardContacts) {
      // Rediscover every overlapping pair immediately, so the world is fully
      // consistent before the caller restores impulses onto the contacts.
      this.broadPhase.updatePairs((a, b) => this.onPair(a, b));
      for (const c of this.contacts) c.update();
    }
    this.sortContacts();
  }

  /** @internal Force a fixture's pairs to be re-evaluated. */
  refilterFixture(fixture: Fixture): void {
    for (let i = this.contacts.length - 1; i >= 0; i--) {
      const c = this.contacts[i]!;
      if (c.fixtureA === fixture || c.fixtureB === fixture) this.removeContactAt(i);
    }
    if (fixture.proxyId >= 0) this.broadPhase.touchProxy(fixture.proxyId);
  }

  /** @internal Refilter every fixture of a body. */
  refilterBody(body: Body): void {
    for (const f of body.fixtures) this.refilterFixture(f);
  }

  /* --------------------------- joints ----------------------------- */

  private addJoint<T extends Joint>(joint: T): T {
    if (joint.id === this.joints.length) this.joints.push(joint);
    else this.joints[joint.id] = joint;
    joint.bodyA.joints.push(joint.id);
    joint.bodyB.joints.push(joint.id);
    joint.wake();
    if (!joint.collideConnected) this.removeContactsBetween(joint.bodyA, joint.bodyB);
    return joint;
  }

  private nextJointId(): number {
    return this.freeJointIds.length > 0 ? this.freeJointIds.pop()! : this.joints.length;
  }

  /** Create a hinge. See {@link RevoluteJoint}. */
  createRevoluteJoint(def: RevoluteJointDef): RevoluteJoint {
    return this.addJoint(new RevoluteJoint(this.nextJointId(), def));
  }

  /** Create a hinge at a world point, deriving the local anchors. */
  createRevoluteJointAt(
    bodyA: Body,
    bodyB: Body,
    x: number,
    y: number,
    extra?: Partial<RevoluteJointDef>,
  ): RevoluteJoint {
    return this.addJoint(
      RevoluteJoint.atWorldPoint(this.nextJointId(), bodyA, bodyB, x, y, extra),
    );
  }

  /** Create a distance/spring/rope joint. See {@link DistanceJoint}. */
  createDistanceJoint(def: DistanceJointDef): DistanceJoint {
    return this.addJoint(new DistanceJoint(this.nextJointId(), def));
  }

  /** Create a slider. See {@link PrismaticJoint}. */
  createPrismaticJoint(def: PrismaticJointDef): PrismaticJoint {
    return this.addJoint(new PrismaticJoint(this.nextJointId(), def));
  }

  /** Create a weld. See {@link WeldJoint}. */
  createWeldJoint(def: WeldJointDef): WeldJoint {
    return this.addJoint(new WeldJoint(this.nextJointId(), def));
  }

  /** Create a mouse/target joint. See {@link MouseJoint}. */
  createMouseJoint(def: MouseJointDef): MouseJoint {
    return this.addJoint(new MouseJoint(this.nextJointId(), def));
  }

  /** Create a motor joint. See {@link MotorJoint}. */
  createMotorJoint(def: MotorJointDef): MotorJoint {
    return this.addJoint(new MotorJoint(this.nextJointId(), def));
  }

  /** Destroy a joint. */
  destroyJoint(joint: Joint): void {
    joint.wake();
    const ia = joint.bodyA.joints.indexOf(joint.id);
    if (ia >= 0) joint.bodyA.joints.splice(ia, 1);
    const ib = joint.bodyB.joints.indexOf(joint.id);
    if (ib >= 0) joint.bodyB.joints.splice(ib, 1);
    this.joints[joint.id] = null;
    this.freeJointIds.push(joint.id);
  }

  /* --------------------------- contacts --------------------------- */

  /** Stable key for a fixture pair. */
  private static pairKey(a: number, b: number): number {
    return a < b ? a * 0x100000 + b : b * 0x100000 + a;
  }

  private removeContactAt(index: number): void {
    const c = this.contacts[index]!;
    if (c.isTouching && this.listener) {
      const e = this.contactEvent(c.fixtureA, c.fixtureB, c);
      if (c.isSensor) this.listener.endSensor?.(e);
      else this.listener.endContact?.(e);
    }
    this.contactMap.delete(World.pairKey(c.fixtureA.id, c.fixtureB.id));
    this.contacts.splice(index, 1);
  }

  private removeContactsBetween(a: Body, b: Body): void {
    for (let i = this.contacts.length - 1; i >= 0; i--) {
      const c = this.contacts[i]!;
      const ba = c.fixtureA.body;
      const bb = c.fixtureB.body;
      if ((ba === a && bb === b) || (ba === b && bb === a)) this.removeContactAt(i);
    }
  }

  /** `true` when a joint forbids these two bodies from colliding. */
  private jointsBlockCollision(a: Body, b: Body): boolean {
    for (const jid of a.joints) {
      const j = this.joints[jid];
      if (!j || j.collideConnected) continue;
      if (j.bodyA === b || j.bodyB === b) return true;
    }
    return false;
  }

  /**
   * Create the {@link Contact} for a newly overlapping pair, unless the pair
   * is filtered out.
   */
  private onPair(idA: number, idB: number): void {
    const fA = this.fixtures[idA];
    const fB = this.fixtures[idB];
    if (!fA || !fB) return;
    const bodyA = fA.body;
    const bodyB = fB.body;
    if (bodyA === bodyB) return; // never self-collide
    // At least one must be able to move.
    if (bodyA.type === BodyType.Static && bodyB.type === BodyType.Static) return;
    if (bodyA.type !== BodyType.Dynamic && bodyB.type !== BodyType.Dynamic) return;
    if (!shouldCollide(fA.filter, fB.filter)) return;
    if (this.jointsBlockCollision(bodyA, bodyB)) return;

    const key = World.pairKey(idA, idB);
    if (this.contactMap.has(key)) return;

    // Order the contact by fixture id so the solve order is canonical.
    const [first, second] = idA < idB ? [fA, fB] : [fB, fA];
    const contact = new Contact(this.contacts.length, first, second);
    if (bodyA.type !== BodyType.Static && bodyB.type !== BodyType.Static) {
      contact.flags |= ContactFlags.Simulated;
    }
    this.contacts.push(contact);
    this.contactMap.set(key, contact);
    this.contactsDirty = true;
  }

  /**
   * Sort the contact list into a canonical order.
   *
   * The solver is a *sequential* impulse solver, so the order in which
   * contacts are solved changes the result in the last bits. Left alone, that
   * order is the order contacts happened to be discovered — a function of the
   * broad-phase tree's shape and therefore of the entire history of the
   * simulation.
   *
   * That is fatal for rollback: a peer that reaches a state by rewinding and
   * replaying discovers the same contacts in a different sequence than a peer
   * that arrived there directly, and the two drift apart a few seconds later.
   * Sorting by `(fixtureA.id, fixtureB.id)` makes the solve order a pure
   * function of *state*, which is what determinism actually requires.
   */
  private sortContacts(): void {
    this.contacts.sort((a, b) => {
      const d = a.fixtureA.id - b.fixtureA.id;
      return d !== 0 ? d : a.fixtureB.id - b.fixtureB.id;
    });
    this.contactsDirty = false;
  }

  /**
   * Drop contacts whose fat AABBs no longer overlap, then re-run the narrow
   * phase on the survivors.
   */
  private updateContacts(): void {
    const listener = this.listener;

    for (let i = this.contacts.length - 1; i >= 0; i--) {
      const c = this.contacts[i]!;
      const fA = c.fixtureA;
      const fB = c.fixtureB;
      const bodyA = fA.body;
      const bodyB = fB.body;

      // Both asleep or static: nothing can have changed.
      const activeA = bodyA.awake && bodyA.type !== BodyType.Static;
      const activeB = bodyB.awake && bodyB.type !== BodyType.Static;
      if (!activeA && !activeB) continue;

      if (!fA.body.enabled || !fB.body.enabled) {
        this.removeContactAt(i);
        continue;
      }

      /*
       * Two-stage rejection before the narrow phase.
       *
       * 1. Fat AABBs no longer overlap  -> the pair is genuinely finished, so
       *    drop the contact entirely.
       * 2. Fat boxes still overlap but the *tight* boxes are further apart than
       *    the speculative margin -> keep the contact alive (its impulses are
       *    the warm-start seed) but skip the SAT/clipping work.
       *
       * Stage 2 matters because the broad phase deliberately pads proxies by
       * AABB_MARGIN so slow bodies need no tree surgery; that padding otherwise
       * forces a full narrow-phase test on every near-miss pair every step.
       */
      if (fA.proxyId >= 0 && fB.proxyId >= 0) {
        this.broadPhase.tree.getAABB(_aabb, fA.proxyId);
        this.broadPhase.tree.getAABB(_tmpAABB, fB.proxyId);
        if (!AABB.overlaps(_aabb, _tmpAABB)) {
          this.removeContactAt(i);
          continue;
        }

        const a = fA.aabb;
        const b = fB.aabb;
        if (
          b.lower.x - a.upper.x > SPECULATIVE_DISTANCE ||
          a.lower.x - b.upper.x > SPECULATIVE_DISTANCE ||
          b.lower.y - a.upper.y > SPECULATIVE_DISTANCE ||
          a.lower.y - b.upper.y > SPECULATIVE_DISTANCE
        ) {
          if (c.isTouching) {
            // It was touching and now is not: report the separation.
            c.manifold.pointCount = 0;
            c.flags &= ~ContactFlags.Touching;
            if (listener) {
              const e = this.contactEvent(fA, fB, c);
              if (c.isSensor) listener.endSensor?.(e);
              else listener.endContact?.(e);
            }
          }
          continue;
        }
      }

      const changed = c.update();
      if (changed && listener) {
        const e = this.contactEvent(fA, fB, c);
        if (c.isSensor) {
          if (c.isTouching) listener.beginSensor?.(e);
          else listener.endSensor?.(e);
        } else if (c.isTouching) {
          listener.beginContact?.(e);
        } else {
          listener.endContact?.(e);
        }
      }

      if (c.isTouching && !c.isSensor && listener?.preSolve) {
        listener.preSolve(this.contactEvent(fA, fB, c));
      }

      /*
       * Propagate wakefulness across a touching pair.
       *
       * Only *sleeping* bodies are touched here. Calling `setAwake(true)` on
       * an already-awake body would reset its sleep timer, and doing that
       * every step would keep a perfectly settled stack awake forever.
       */
      if (c.isTouching && !c.isSensor) {
        if (bodyA.type === BodyType.Dynamic && !bodyA.awake && activeB) bodyA.setAwake(true);
        if (bodyB.type === BodyType.Dynamic && !bodyB.awake && activeA) bodyB.setAwake(true);
      }
    }
  }

  /* ---------------------------- stepping -------------------------- */

  /**
   * Advance the simulation by exactly one fixed step.
   *
   * Deterministic: given the same world state and the same inputs applied
   * before the call, every machine produces bit-identical results.
   */
  step(): void {
    const t0 = now();

    /*
     * `gravity` is a public mutable Vec2, so it has no setter to validate.
     * A NaN slipped in there would reach every awake body in the very next
     * integration and poison the whole world, so it is checked once per step
     * — one comparison against thousands of bodies.
     */
    if (!Number.isFinite(this.gravity.x as number) || !Number.isFinite(this.gravity.y as number)) {
      this.gravity.setZero();
    }

    this.tick++;
    this.time += this.timeStep;

    // Keep the context in sync with any settings changed since construction.
    const h = S.divInt(this.timeStep, this.subSteps);
    this.ctx.h = h;
    this.ctx.invH = h > S.ZERO ? S.inv(h) : S.ZERO;
    this.ctx.velocityIterations = this.velocityIterations;
    this.ctx.relaxIterations = this.relaxIterations;
    this.ctx.enableSleep = this.enableSleep;
    this.ctx.enableWarmStarting = this.enableWarmStarting;
    this.ctx.enableRestitution = this.enableRestitution;

    /* --- 1. broad phase --- */
    const t1 = now();
    this.broadPhase.updatePairs((a, b) => this.onPair(a, b));
    if (this.contactsDirty) this.sortContacts();
    this.profile.broadPhase = now() - t1;

    /* --- 2. narrow phase --- */
    const t2 = now();
    this.updateContacts();
    this.profile.narrowPhase = now() - t2;

    /* --- 3. solve --- */
    const t3 = now();
    const solver = this.solver;
    solver.prepareBodies(this.bodies);

    if (solver.bodyCount > 0) {
      /*
       * Soft-step TGS.
       *
       * Constraints are prepared **once** per step: the anchors and effective
       * masses are computed from the pose at the start of the step, and each
       * sub-step then tracks how far the bodies have moved since via the
       * solver's `dp` deltas. That is the whole reason `dp` exists — rebuilding
       * the constraints every sub-step would recompute the same mass matrices
       * `subSteps` times for no benefit.
       *
       * Each sub-step is then:
       *
       *   integrate velocities  ->  solve with bias  ->  integrate positions
       *                         ->  relax without bias
       *
       * One biased iteration per sub-step is enough because the sub-step
       * itself is the convergence mechanism; `velocityIterations` is there for
       * scenes that need extra rigidity, not as the default workhorse.
       */
      solver.prepareContacts(this.ctx, this.contacts, this.bodies);
      solver.prepareJoints(this.ctx, this.joints);

      for (let sub = 0; sub < this.subSteps; sub++) {
        solver.integrateVelocities(this.ctx, this.bodies);

        /*
         * Warm start contacts every sub-step, joints only on the first.
         *
         * A contact's accumulated impulse is clamped to be non-negative (and
         * friction to the Coulomb cone) every iteration, so re-applying it is
         * self-correcting — and doing so is what keeps deep stacks converging.
         *
         * A joint's impulse is unbounded by construction: an equality
         * constraint may push or pull by any amount. Re-applying it once per
         * sub-step feeds it back `subSteps` times, and above four sub-steps
         * that positive feedback diverges — a heavy pendulum would launch
         * itself across the level. Applying it once per step is the correct
         * reading of the accumulated impulse.
         */
        solver.warmStart(this.ctx);
        solver.warmStartJoints(this.ctx);

        for (let i = 0; i < this.ctx.velocityIterations; i++) {
          solver.solveJoints(this.ctx, true);
          solver.solveContacts(this.ctx, true);
        }

        solver.integratePositions(this.ctx);

        for (let i = 0; i < this.ctx.relaxIterations; i++) {
          solver.solveJoints(this.ctx, false);
          solver.solveContacts(this.ctx, false);
        }
      }

      // Restitution is a single pass over the whole step, using the approach
      // speed captured before any of the sub-steps ran.
      solver.applyRestitution(this.ctx);
      solver.storeImpulses();

      solver.finalizeBodies(this.ctx, this.bodies, this.awakeBodies);

      /* --- 3b. continuous collision for bullet bodies --- */
      const tc = now();
      solveContinuous(this, this.awakeBodies);
      this.profile.continuous = now() - tc;

      /* --- 4. sleep --- */
      if (this.enableSleep) this.updateSleep();

      /* --- 5. sync the broad phase --- */
      for (const body of this.awakeBodies) {
        for (const f of body.fixtures) {
          if (f.proxyId < 0) continue;
          f.synchronize();
          _disp.set(
            S.mul(body.linearVelocity.x, this.timeStep),
            S.mul(body.linearVelocity.y, this.timeStep),
          );
          this.broadPhase.moveProxy(f.proxyId, f.aabb, _disp);
        }
      }
    }
    this.profile.solve = now() - t3;

    /* --- 6. post-solve events --- */
    if (this.listener?.postSolve) {
      for (const c of this.contacts) {
        if (!c.isTouching || c.isSensor || c.constraintIndex < 0) continue;
        let maxImpulse = S.ZERO;
        let approach = S.ZERO;
        for (let i = 0; i < c.manifold.pointCount; i++) {
          const mp = c.manifold.points[i]!;
          maxImpulse = S.max(maxImpulse, mp.maxNormalImpulse);
          approach = S.min(approach, mp.relativeVelocity);
        }
        if (maxImpulse > S.ZERO) {
          const e = this._impactEvent;
          e.fixtureA = c.fixtureA;
          e.fixtureB = c.fixtureB;
          e.contact = c;
          e.maxNormalImpulse = maxImpulse;
          e.approachSpeed = approach;
          this.listener.postSolve(e);
        }
      }
    }

    this.profile.bodyCount = solver.bodyCount;
    this.profile.contactCount = this.contacts.length;
    this.profile.activeContactCount = solver.contactConstraintCount;
    this.profile.total = now() - t0;
  }

  /**
   * Drive a fixed simulation from a variable frame time.
   *
   * ```ts
   * const alpha = world.accumulate(frameDeltaSeconds);
   * renderInterpolated(alpha); // alpha in [0,1) between the last two states
   * ```
   *
   * @param dt        real elapsed seconds since the last call
   * @param maxSteps  cap on catch-up steps, to avoid a death spiral
   * @returns the interpolation factor for rendering
   */
  accumulate(dt: number, maxSteps = 5): number {
    /*
     * Reject a non-finite delta outright.
     *
     * `Math.max(0, NaN)` is `NaN`, so a single bad frame time — which browsers
     * do produce, e.g. from a `performance.now()` hiccup or an uninitialised
     * `last` timestamp — would poison the accumulator permanently and the
     * world would silently never step again.
     */
    if (!Number.isFinite(dt) || dt <= 0) {
      return S.toFloat(S.div(this.stepAccumulator, this.timeStep));
    }
    this.stepAccumulator += S.fromFloat(dt);
    let steps = 0;
    while (this.stepAccumulator >= this.timeStep && steps < maxSteps) {
      this.stepAccumulator -= this.timeStep;
      this.step();
      steps++;
    }
    // Drop the backlog rather than spiral.
    if (steps === maxSteps && this.stepAccumulator > this.timeStep) {
      this.stepAccumulator = S.ZERO;
    }
    return S.toFloat(S.div(this.stepAccumulator, this.timeStep));
  }

  /**
   * Put islands of connected, settled bodies to sleep.
   *
   * The rule is per **island**, not per body: a box resting on a moving
   * platform must stay awake even though it is not moving itself. Islands are
   * discovered with a union-find over contacts and joints, which is `O(n·α)`
   * and — being index-ordered — deterministic.
   */
  /** Union-find parent buffer, reused across steps. @internal */
  private _ufParent = new Int32Array(0);
  /** Per-island minimum sleep time, reused across steps. @internal */
  private _ufMinSleep = new Float64Array(0);

  /** Union-find root with path compression, over {@link _ufParent}. */
  private ufFind(x: number): number {
    const parent = this._ufParent;
    let r = x;
    while (parent[r] !== r) r = parent[r]!;
    // Path compression: point every node on the way at the root.
    while (parent[x] !== r) {
      const next = parent[x]!;
      parent[x] = r;
      x = next;
    }
    return r;
  }

  /** Merge two islands, always keeping the lower index as the root. */
  private ufUnion(a: number, b: number): void {
    const ra = this.ufFind(a);
    const rb = this.ufFind(b);
    if (ra === rb) return;
    // Lower index wins, so the result does not depend on merge order.
    if (ra < rb) this._ufParent[rb] = ra;
    else this._ufParent[ra] = rb;
  }

  private updateSleep(): void {
    const solver = this.solver;
    const n = solver.bodyCount;
    if (n === 0) return;

    // Grow the scratch buffers geometrically; in the steady state this never
    // runs, so the whole pass is allocation-free.
    if (this._ufParent.length < n) {
      const cap = 1 << (32 - Math.clz32(Math.max(n, 64) - 1));
      this._ufParent = new Int32Array(cap);
      this._ufMinSleep = new Float64Array(cap);
    }
    const parent = this._ufParent;
    const minSleep = this._ufMinSleep;
    for (let i = 0; i < n; i++) {
      parent[i] = i;
      minSleep[i] = Number.MAX_VALUE;
    }

    for (let i = 0; i < this.contacts.length; i++) {
      const c = this.contacts[i]!;
      if (!c.isTouching || c.isSensor) continue;
      const ia = c.fixtureA.body.solverIndex;
      const ib = c.fixtureB.body.solverIndex;
      if (ia >= 0 && ib >= 0) this.ufUnion(ia, ib);
    }
    for (let i = 0; i < this.joints.length; i++) {
      const j = this.joints[i];
      if (!j) continue;
      const ia = j.bodyA.solverIndex;
      const ib = j.bodyB.solverIndex;
      if (ia >= 0 && ib >= 0) this.ufUnion(ia, ib);
    }

    // Minimum sleep time per island.
    for (let i = 0; i < n; i++) {
      const body = this.bodies[solver.bodies[i]!.bodyIndex]!;
      const t = body.updateSleepTime(this.timeStep);
      const root = this.ufFind(i);
      const value = body.allowSleep && body.type !== BodyType.Kinematic ? (t as number) : 0;
      if (value < minSleep[root]!) minSleep[root] = value;
    }

    const threshold = TIME_TO_SLEEP as number;
    for (let i = 0; i < n; i++) {
      if (minSleep[this.ufFind(i)]! >= threshold) {
        this.bodies[solver.bodies[i]!.bodyIndex]!.setAwake(false);
      }
    }
  }

  /* ---------------------------- queries --------------------------- */

  /**
   * Report every fixture whose AABB overlaps the box.
   * Return `false` from the callback to stop early.
   */
  queryAABB(
    lowerX: number,
    lowerY: number,
    upperX: number,
    upperY: number,
    cb: (fixture: Fixture) => boolean,
  ): void {
    _aabb.set(
      S.fromFloat(lowerX),
      S.fromFloat(lowerY),
      S.fromFloat(upperX),
      S.fromFloat(upperY),
    );
    this.broadPhase.query(_aabb, (fixtureId) => {
      const f = this.fixtures[fixtureId];
      return f ? cb(f) : true;
    });
  }

  /** Report every fixture that actually contains the world point. */
  queryPoint(x: number, y: number, cb: (fixture: Fixture) => boolean): void {
    const p = _tmp.set(S.fromFloat(x), S.fromFloat(y));
    this.broadPhase.queryPoint(p, (fixtureId) => {
      const f = this.fixtures[fixtureId];
      if (!f) return true;
      if (!f.shape.testPoint(f.body.transform, p)) return true;
      return cb(f);
    });
  }

  /**
   * Cast a ray and report the **closest** hit, or `null`.
   *
   * ```ts
   * const hit = world.rayCastClosest(0, 0, 10, 0);
   * if (hit) console.log(hit.fixture.body.userData, hit.point.toFloats());
   * ```
   */
  rayCastClosest(
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    filter?: (f: Fixture) => boolean,
  ): { fixture: Fixture; point: Vec2; normal: Vec2; fraction: Scalar } | null {
    let best: { fixture: Fixture; point: Vec2; normal: Vec2; fraction: Scalar } | null = null;
    this.rayCast(x1, y1, x2, y2, (fixture, point, normal, fraction) => {
      if (filter && !filter(fixture)) return S.NEG_ONE;
      best = { fixture, point: point.clone(), normal: normal.clone(), fraction };
      return fraction; // shrink the search
    });
    return best;
  }

  /**
   * Cast a ray and report every hit.
   *
   * The callback controls the traversal:
   * * return `-1` to ignore this fixture and continue unchanged;
   * * return `0` to stop immediately;
   * * return `fraction` to keep only closer hits from now on;
   * * return `1` to continue with the full range.
   */
  rayCast(
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    cb: (fixture: Fixture, point: Vec2, normal: Vec2, fraction: Scalar) => Scalar,
  ): void {
    const p1 = new Vec2(S.fromFloat(x1), S.fromFloat(y1));
    const p2 = new Vec2(S.fromFloat(x2), S.fromFloat(y2));
    const input: RayCastInput = { p1, p2, maxFraction: S.ONE };
    const output: RayCastOutput = {
      normal: Vec2.zero(),
      point: Vec2.zero(),
      fraction: S.ZERO,
      hit: false,
    };

    this.broadPhase.rayCast(p1, p2, S.ONE, (fixtureId, rp1, rp2, maxFraction) => {
      const f = this.fixtures[fixtureId];
      if (!f) return maxFraction;
      input.p1 = rp1;
      input.p2 = rp2;
      input.maxFraction = maxFraction;
      if (!f.shape.rayCast(output, input, f.body.transform)) return maxFraction;
      return cb(f, output.point, output.normal, output.fraction);
    });
  }

  /* ---------------------------- utility --------------------------- */

  /** Number of live bodies. */
  get bodyCount(): number {
    let n = 0;
    for (const b of this.bodies) if (b) n++;
    return n;
  }

  /** Number of bodies the solver moved last step. */
  get awakeBodyCount(): number {
    return this.solver.bodyCount;
  }

  /** Number of live contacts. */
  get contactCount(): number {
    return this.contacts.length;
  }

  /** Number of live joints. */
  get jointCount(): number {
    let n = 0;
    for (const j of this.joints) if (j) n++;
    return n;
  }

  /** Iterate the live bodies in id order. */
  *eachBody(): IterableIterator<Body> {
    for (const b of this.bodies) if (b) yield b;
  }

  /** Iterate the live joints in id order. */
  *eachJoint(): IterableIterator<Joint> {
    for (const j of this.joints) if (j) yield j;
  }

  /** Wake every body. */
  wakeAll(): void {
    for (const b of this.bodies) if (b) b.setAwake(true);
  }

  /** Remove everything. The world is reusable afterwards. */
  clear(): void {
    this.bodies.length = 0;
    this.fixtures.length = 0;
    this.contacts.length = 0;
    this.joints.length = 0;
    this.contactMap.clear();
    this.freeBodyIds.length = 0;
    this.freeFixtureIds.length = 0;
    this.freeJointIds.length = 0;
    this.broadPhase.clear();
    this.solver.clear();
    this.tick = 0;
    this.time = S.ZERO;
    this.stepAccumulator = S.ZERO;
  }

  /** The fat-AABB padding used by the broad phase. */
  static get aabbMargin(): Scalar {
    return AABB_MARGIN;
  }
}

/** Monotonic clock in ms; falls back to `Date.now` where unavailable. */
const now: () => number =
  typeof performance !== 'undefined' && typeof performance.now === 'function'
    ? () => performance.now()
    : () => Date.now();
