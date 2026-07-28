/**
 * @module dynamics/Body
 *
 * A rigid body: position, orientation, velocity and mass.
 *
 * ### The three body types
 *
 * | type        | moved by      | mass     | collides with        |
 * |-------------|---------------|----------|----------------------|
 * | `static`    | you, directly | infinite | dynamic, kinematic   |
 * | `kinematic` | its velocity  | infinite | dynamic              |
 * | `dynamic`   | forces        | finite   | everything           |
 *
 * Static bodies are free — they never enter the solver. Kinematic bodies move
 * but are not pushed back (moving platforms, elevators). Dynamic bodies are
 * fully simulated.
 *
 * ### Centre of mass
 *
 * Internally the solver integrates the **centre of mass**, not the body
 * origin, because that is the point about which rotation is inertia-free. The
 * public API is expressed in terms of the origin (the transform you set and
 * read); `localCenter` / `worldCenter` bridge the two.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Rot } from './../math/Rot.js';
import { Transform } from './../math/Transform.js';
import { Fixture } from './Fixture.js';
import type { FixtureDef } from './Fixture.js';
import type { MassData } from './../collision/Shape.js';
import type { World } from './World.js';
import {
  SLEEP_LINEAR_TOLERANCE,
  SLEEP_ANGULAR_TOLERANCE,
  TIME_TO_SLEEP,
} from './../util/settings.js';

/** How a body responds to simulation. */
export const enum BodyType {
  Static = 0,
  Kinematic = 1,
  Dynamic = 2,
}

/** Options accepted by {@link World#createBody}. */
export interface BodyDef {
  /** Default `Dynamic`. */
  type?: BodyType;
  /** World position of the body origin. */
  position?: { x: number; y: number };
  /** Rotation in radians. */
  angle?: number;
  /** Initial linear velocity, m/s. */
  linearVelocity?: { x: number; y: number };
  /** Initial angular velocity, rad/s. */
  angularVelocity?: number;
  /** Linear damping, 1/s. Applied as an implicit exponential decay. */
  linearDamping?: number;
  /** Angular damping, 1/s. */
  angularDamping?: number;
  /** Multiplier on the world gravity. `0` makes the body float. Default `1`. */
  gravityScale?: number;
  /** Lock rotation entirely — the standard trick for platformer characters. */
  fixedRotation?: boolean;
  /** Allow this body to sleep. Default `true`. */
  allowSleep?: boolean;
  /** Start awake. Default `true`. */
  awake?: boolean;
  /** Start enabled (in the broad phase). Default `true`. */
  enabled?: boolean;
  /**
   * Enable continuous collision for this body. Costs a sweep test per step;
   * use it for bullets and other small fast objects, not for everything.
   */
  bullet?: boolean;
  /** Arbitrary payload; never touched by the engine. */
  userData?: unknown;
}

const _massData: MassData = { mass: S.ZERO, center: Vec2.zero(), inertia: S.ZERO };

/**
 * Reject a non-finite value coming in through the public API.
 *
 * A single `NaN` velocity is catastrophic and *silent*: it propagates into the
 * body's position, then into every contact it takes part in, and the usual
 * symptom is a body that simply stops responding rather than an error anyone
 * can trace. Game code produces `NaN` easily — `0/0` from a normalise, an
 * un-initialised value, a bad lerp — so the boundary is checked here.
 *
 * These setters are called at most a few times per body per frame, never from
 * the solver's inner loops, so the check costs nothing measurable. The hot
 * path (`Scalar.fromFloat`) is deliberately left unguarded.
 */
function finite(v: number): boolean {
  return Number.isFinite(v);
}

export class Body {
  /** Dense index into the world's body table. Stable for the body's lifetime. */
  readonly id: number;
  /** Owning world. */
  readonly world: World;

  type: BodyType;

  /** Origin transform (position + rotation). Read freely; write via setters. */
  readonly transform = new Transform();
  /** Centre of mass in world space — what the solver actually integrates. */
  readonly worldCenter = Vec2.zero();
  /** Centre of mass in local space. */
  readonly localCenter = Vec2.zero();

  /**
   * Centre of mass at the **start** of the current step.
   * Together with {@link sweepRot0} this defines the motion continuous
   * collision sweeps against.
   * @internal
   */
  readonly sweepCenter0 = Vec2.zero();
  /** Rotation at the start of the current step. @internal */
  readonly sweepRot0 = new Rot();

  /** Linear velocity of the **centre of mass**, m/s. */
  readonly linearVelocity = Vec2.zero();
  /** Angular velocity, rad/s. */
  angularVelocity: Scalar = S.ZERO;

  /** Accumulated force, N. Cleared after every step. */
  readonly force = Vec2.zero();
  /** Accumulated torque, N·m. Cleared after every step. */
  torque: Scalar = S.ZERO;

  /** Mass, kg. `0` for static/kinematic bodies. */
  mass: Scalar = S.ZERO;
  /** `1 / mass`, or `0` when infinite. */
  invMass: Scalar = S.ZERO;
  /** Rotational inertia about the centre of mass. */
  inertia: Scalar = S.ZERO;
  /** `1 / inertia`, or `0` when infinite or rotation is locked. */
  invInertia: Scalar = S.ZERO;

  linearDamping: Scalar;
  angularDamping: Scalar;
  gravityScale: Scalar;

  fixedRotation: boolean;
  allowSleep: boolean;
  bullet: boolean;

  /** `false` while asleep — the solver skips it entirely. */
  awake = true;
  /** Seconds spent below the sleep tolerances. */
  sleepTime: Scalar = S.ZERO;
  /** `false` removes the body from collision without destroying it. */
  enabled = true;

  userData: unknown;

  /** Attached fixtures, in creation order. */
  readonly fixtures: Fixture[] = [];

  /** @internal Solver slot for this step, or `-1` when not solved. */
  solverIndex = -1;
  /** @internal Island id assigned during the current step. */
  islandId = -1;
  /** @internal Contacts referencing this body (edge list heads). */
  readonly contacts: number[] = [];
  /** @internal Joints referencing this body. */
  readonly joints: number[] = [];

  /** @internal — created by {@link World#createBody}. */
  constructor(id: number, world: World, def: BodyDef) {
    this.id = id;
    this.world = world;
    this.type = def.type ?? BodyType.Dynamic;

    if (def.position) this.transform.p.set(S.fromFloat(def.position.x), S.fromFloat(def.position.y));
    if (def.angle) this.transform.q.setAngle(S.fromFloat(def.angle));
    this.worldCenter.copyFrom(this.transform.p);

    if (def.linearVelocity) {
      this.linearVelocity.set(
        S.fromFloat(def.linearVelocity.x),
        S.fromFloat(def.linearVelocity.y),
      );
    }
    this.angularVelocity = S.fromFloat(def.angularVelocity ?? 0);
    this.linearDamping = S.fromFloat(def.linearDamping ?? 0);
    this.angularDamping = S.fromFloat(def.angularDamping ?? 0);
    this.gravityScale = S.fromFloat(def.gravityScale ?? 1);
    this.fixedRotation = def.fixedRotation ?? false;
    this.allowSleep = def.allowSleep ?? true;
    this.awake = def.awake ?? true;
    this.enabled = def.enabled ?? true;
    this.bullet = def.bullet ?? false;
    this.userData = def.userData;
  }

  /* --------------------------- fixtures --------------------------- */

  /**
   * Attach a shape. Mass properties are recomputed automatically unless you
   * have called {@link setMassData} to override them.
   */
  addFixture(def: FixtureDef): Fixture {
    const fixture = this.world.registerFixture(this, def);
    this.fixtures.push(fixture);
    if (this.enabled) this.world.createFixtureProxy(fixture);
    /*
     * Always recompute, even for a zero-density fixture. Skipping the call
     * left a dynamic body at `mass = invMass = 0`, i.e. infinitely heavy — it
     * fell under gravity but no contact impulse could stop it, so it sank
     * straight through the floor. `resetMassData` has the correct fallback
     * (unit mass for a massless dynamic body); it just has to run.
     */
    if (!this.massOverridden) this.resetMassData();
    return fixture;
  }

  /** Detach and destroy a fixture. */
  removeFixture(fixture: Fixture): void {
    const i = this.fixtures.indexOf(fixture);
    if (i < 0) return;
    this.fixtures.splice(i, 1);
    this.world.unregisterFixture(fixture);
    if (!this.massOverridden) this.resetMassData();
  }

  /** `true` when {@link setMassData} was used to pin the mass manually. */
  private massOverridden = false;

  /**
   * Recompute mass, centre of mass and inertia by summing all fixtures.
   *
   * Static and kinematic bodies always end up with infinite mass. A dynamic
   * body with no (or zero-density) fixtures is given `mass = 1` so it still
   * responds to forces instead of silently becoming immovable.
   */
  resetMassData(): void {
    this.mass = S.ZERO;
    this.invMass = S.ZERO;
    this.inertia = S.ZERO;
    this.invInertia = S.ZERO;
    this.localCenter.setZero();

    if (this.type !== BodyType.Dynamic) {
      this.worldCenter.copyFrom(this.transform.p);
      return;
    }

    const center = Vec2.zero();
    let inertiaAboutOrigin = S.ZERO;

    for (const f of this.fixtures) {
      if (f.density === S.ZERO) continue;
      f.shape.computeMass(_massData, f.density);
      this.mass += _massData.mass;
      center.addScaled(_massData.center, _massData.mass);
      inertiaAboutOrigin += _massData.inertia;
    }

    if (this.mass > S.ZERO) {
      this.invMass = S.inv(this.mass);
      center.scale(this.invMass);
    } else {
      // Give shapeless dynamic bodies unit mass so forces still apply.
      this.mass = S.ONE;
      this.invMass = S.ONE;
    }

    if (inertiaAboutOrigin > S.ZERO && !this.fixedRotation) {
      // Shift the inertia from the origin to the centre of mass.
      this.inertia = inertiaAboutOrigin - S.mul(this.mass, center.lengthSq());
      this.invInertia = this.inertia > S.ZERO ? S.inv(this.inertia) : S.ZERO;
    } else {
      this.inertia = S.ZERO;
      this.invInertia = S.ZERO;
    }

    // Keep the linear velocity of the *old* centre so overriding mass mid-air
    // does not teleport momentum.
    const oldCenterX = this.worldCenter.x;
    const oldCenterY = this.worldCenter.y;
    this.localCenter.copyFrom(center);
    Transform.apply(this.worldCenter, this.transform, this.localCenter);
    this.linearVelocity.x += S.mul(-this.angularVelocity, this.worldCenter.y - oldCenterY);
    this.linearVelocity.y += S.mul(this.angularVelocity, this.worldCenter.x - oldCenterX);
  }

  /**
   * Override the computed mass properties.
   * @param mass    kg; `0` makes the body behave as if static
   * @param inertia about the centre of mass; `0` locks rotation dynamically
   * @param centerX local centre of mass
   */
  setMassData(mass: number, inertia: number, centerX = 0, centerY = 0): void {
    if (!finite(mass) || !finite(inertia) || !finite(centerX) || !finite(centerY)) return;
    this.massOverridden = true;
    this.mass = S.fromFloat(mass);
    this.invMass = this.mass > S.ZERO ? S.inv(this.mass) : S.ZERO;
    this.inertia = S.fromFloat(inertia);
    this.invInertia =
      this.inertia > S.ZERO && !this.fixedRotation ? S.inv(this.inertia) : S.ZERO;
    this.localCenter.set(S.fromFloat(centerX), S.fromFloat(centerY));
    Transform.apply(this.worldCenter, this.transform, this.localCenter);
  }

  /** Return to automatically computed mass properties. */
  clearMassOverride(): void {
    this.massOverridden = false;
    this.resetMassData();
  }

  /* -------------------------- transform --------------------------- */

  /** World position of the body **origin**. */
  getPosition(): Vec2 {
    return this.transform.p;
  }

  /** Rotation in radians, in `(-π, π]`. */
  getAngle(): Scalar {
    return this.transform.q.getAngle();
  }

  /**
   * Teleport the body. Velocities are preserved, contacts are re-evaluated
   * next step. Prefer velocity changes for anything that should look physical.
   */
  setTransform(x: number, y: number, angle: number): void {
    if (!finite(x) || !finite(y) || !finite(angle)) return;
    this.transform.p.set(S.fromFloat(x), S.fromFloat(y));
    this.transform.q.setAngle(S.fromFloat(angle));
    Transform.apply(this.worldCenter, this.transform, this.localCenter);
    this.world.synchronizeFixtures(this);
    this.setAwake(true);
  }

  /** Same as {@link setTransform} but takes backend scalars. */
  setTransformScalar(x: Scalar, y: Scalar, angle: Scalar): void {
    this.transform.p.set(x, y);
    this.transform.q.setAngle(angle);
    Transform.apply(this.worldCenter, this.transform, this.localCenter);
    this.world.synchronizeFixtures(this);
    this.setAwake(true);
  }

  /** `out = world point for the given local point`. */
  getWorldPoint(out: Vec2, local: Vec2): Vec2 {
    return Transform.apply(out, this.transform, local);
  }

  /** `out = local point for the given world point`. */
  getLocalPoint(out: Vec2, world: Vec2): Vec2 {
    return Transform.applyT(out, this.transform, world);
  }

  /** `out = world direction for the given local vector` (ignores translation). */
  getWorldVector(out: Vec2, local: Vec2): Vec2 {
    return Rot.rotate(out, this.transform.q, local);
  }

  /** `out = local direction for the given world vector`. */
  getLocalVector(out: Vec2, world: Vec2): Vec2 {
    return Rot.rotateT(out, this.transform.q, world);
  }

  /** Velocity of the world point `p` on this body: `v + ω × (p - c)`. */
  getVelocityAtPoint(out: Vec2, p: Vec2): Vec2 {
    out.x = this.linearVelocity.x - S.mul(this.angularVelocity, p.y - this.worldCenter.y);
    out.y = this.linearVelocity.y + S.mul(this.angularVelocity, p.x - this.worldCenter.x);
    return out;
  }

  /* -------------------------- velocity ---------------------------- */

  /**
   * Set the linear velocity of the centre of mass.
   * Non-finite input is ignored — see {@link finite}.
   */
  setLinearVelocity(vx: number, vy: number): void {
    if (this.type === BodyType.Static) return;
    if (!finite(vx) || !finite(vy)) return;
    this.linearVelocity.set(S.fromFloat(vx), S.fromFloat(vy));
    if (!this.linearVelocity.isZero()) this.setAwake(true);
  }

  /** Set the angular velocity. Non-finite input is ignored. */
  setAngularVelocity(w: number): void {
    if (this.type === BodyType.Static) return;
    if (!finite(w)) return;
    this.angularVelocity = S.fromFloat(w);
    if (this.angularVelocity !== S.ZERO) this.setAwake(true);
  }

  /* --------------------------- forces ----------------------------- */

  /**
   * Apply a force at a world point. Accumulates until the next step; a force
   * applied every frame produces smooth acceleration.
   */
  applyForce(fx: number, fy: number, px?: number, py?: number, wake = true): void {
    if (this.type !== BodyType.Dynamic) return;
    if (!finite(fx) || !finite(fy)) return;
    if (wake) this.setAwake(true);
    else if (!this.awake) return;
    const sfx = S.fromFloat(fx);
    const sfy = S.fromFloat(fy);
    this.force.x += sfx;
    this.force.y += sfy;
    if (px !== undefined && py !== undefined) {
      const rx = S.fromFloat(px) - this.worldCenter.x;
      const ry = S.fromFloat(py) - this.worldCenter.y;
      this.torque += S.mulAdd(rx, sfy, -S.mul(ry, sfx));
    }
  }

  /** Apply a force at the centre of mass (no torque). */
  applyForceToCenter(fx: number, fy: number, wake = true): void {
    this.applyForce(fx, fy, undefined, undefined, wake);
  }

  /** Apply a pure torque, N·m. */
  applyTorque(t: number, wake = true): void {
    if (this.type !== BodyType.Dynamic) return;
    if (!finite(t)) return;
    if (wake) this.setAwake(true);
    else if (!this.awake) return;
    this.torque += S.fromFloat(t);
  }

  /**
   * Apply an instantaneous impulse (N·s) at a world point.
   * Use this for hits, jumps and explosions — anything that should change the
   * velocity *now* rather than over time.
   */
  applyLinearImpulse(ix: number, iy: number, px?: number, py?: number, wake = true): void {
    if (this.type !== BodyType.Dynamic) return;
    if (!finite(ix) || !finite(iy)) return;
    if (wake) this.setAwake(true);
    else if (!this.awake) return;
    const six = S.fromFloat(ix);
    const siy = S.fromFloat(iy);
    this.linearVelocity.x += S.mul(this.invMass, six);
    this.linearVelocity.y += S.mul(this.invMass, siy);
    if (px !== undefined && py !== undefined) {
      const rx = S.fromFloat(px) - this.worldCenter.x;
      const ry = S.fromFloat(py) - this.worldCenter.y;
      this.angularVelocity += S.mul(this.invInertia, S.mulAdd(rx, siy, -S.mul(ry, six)));
    }
  }

  /** Apply an angular impulse, kg·m²/s. */
  applyAngularImpulse(impulse: number, wake = true): void {
    if (this.type !== BodyType.Dynamic) return;
    if (!finite(impulse)) return;
    if (wake) this.setAwake(true);
    else if (!this.awake) return;
    this.angularVelocity += S.mul(this.invInertia, S.fromFloat(impulse));
  }

  /* ---------------------------- state ----------------------------- */

  /**
   * Wake or sleep the body.
   *
   * Waking resets the sleep timer; sleeping zeroes the velocities and pending
   * forces so the body cannot drift.
   */
  setAwake(awake: boolean): void {
    if (this.type === BodyType.Static) return;
    if (awake) {
      this.awake = true;
      this.sleepTime = S.ZERO;
    } else {
      this.awake = false;
      this.sleepTime = S.ZERO;
      this.linearVelocity.setZero();
      this.angularVelocity = S.ZERO;
      this.force.setZero();
      this.torque = S.ZERO;
    }
  }

  /** Change the body type; mass and contacts are rebuilt. */
  setType(type: BodyType): void {
    if (this.type === type) return;
    this.type = type;
    this.resetMassData();
    if (type === BodyType.Static) {
      this.linearVelocity.setZero();
      this.angularVelocity = S.ZERO;
    }
    this.setAwake(true);
    this.world.refilterBody(this);
  }

  /**
   * Remove the body from the simulation entirely, or put it back.
   *
   * A disabled body keeps its state but is **frozen**: it is skipped by the
   * solver (so gravity and forces do not move it), its broad-phase proxies are
   * released (so it collides with nothing) and its contacts are dropped. This
   * is the cheap way to park an off-screen object without losing it.
   *
   * Re-enabling re-inserts the proxies at the body's current transform and
   * wakes it, so set the position *before* re-enabling if you are relocating.
   */
  setEnabled(enabled: boolean): void {
    if (this.enabled === enabled) return;
    this.enabled = enabled;
    if (enabled) {
      for (const f of this.fixtures) this.world.createFixtureProxy(f);
      this.setAwake(true);
    } else {
      for (const f of this.fixtures) this.world.destroyFixtureProxy(f);
    }
  }

  /** Lock or unlock rotation. */
  setFixedRotation(fixed: boolean): void {
    if (this.fixedRotation === fixed) return;
    this.fixedRotation = fixed;
    this.angularVelocity = S.ZERO;
    this.resetMassData();
  }

  /**
   * @internal Advance the sleep timer.
   * @returns the body's sleep-eligible time this step
   */
  updateSleepTime(dt: Scalar): Scalar {
    if (!this.allowSleep || this.type === BodyType.Static) {
      this.sleepTime = S.ZERO;
      return S.ZERO;
    }
    const linTol = S.mul(SLEEP_LINEAR_TOLERANCE, SLEEP_LINEAR_TOLERANCE);
    if (
      S.mul(this.angularVelocity, this.angularVelocity) >
        S.mul(SLEEP_ANGULAR_TOLERANCE, SLEEP_ANGULAR_TOLERANCE) ||
      this.linearVelocity.lengthSq() > linTol
    ) {
      this.sleepTime = S.ZERO;
    } else {
      this.sleepTime += dt;
    }
    return this.sleepTime;
  }

  /** Total kinetic energy, J. Useful for stability assertions in tests. */
  getKineticEnergy(): Scalar {
    const lin = S.mul(this.mass, this.linearVelocity.lengthSq());
    const ang = S.mul(this.inertia, S.mul(this.angularVelocity, this.angularVelocity));
    return S.half(lin + ang);
  }

  /** `true` when the solver will move this body. */
  get isDynamic(): boolean {
    return this.type === BodyType.Dynamic;
  }

  /** Time the body must remain still before it may sleep. */
  static get timeToSleep(): Scalar {
    return TIME_TO_SLEEP;
  }
}
