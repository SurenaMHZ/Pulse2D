/**
 * @module dynamics/Solver
 *
 * The constraint solver — a **soft-step sequential impulse** solver with
 * relaxation, in the spirit of Box2D v3's TGS-Soft.
 *
 * ### The pipeline for one step
 *
 * ```
 * 1. integrate velocities      gravity, forces, damping
 * 2. prepare constraints       mass matrices, bias terms, anchors
 * 3. warm start                re-apply last step's impulses
 * 4. solve velocities × N      with soft position bias  (useBias = true)
 * 5. integrate positions       move the bodies
 * 6. relax × M                 same solve, no bias      (useBias = false)
 * 7. restitution               a final bounce pass
 * 8. store impulses            for next step's warm start
 * ```
 *
 * ### Why "soft"
 *
 * A hard constraint tries to remove all penetration in one step, which
 * injects energy and makes stacks jitter. Instead each contact is treated as a
 * stiff spring-damper characterised by a frequency (`hertz`) and a damping
 * ratio, converted into three coefficients:
 *
 * ```
 * ω     = 2π·hertz
 * a1    = 2ζ + h·ω              (biasRate  denominator)
 * biasRate    = ω / a1
 * massScale   = h·ω·a1 / (1 + h·ω·a1)
 * impulseScale= 1 / (1 + h·ω·a1)
 * ```
 *
 * The push-out is spread over several steps, so the simulation stays calm and
 * — because the coefficients depend only on `h` and the constants — perfectly
 * reproducible.
 *
 * ### Why relaxation
 *
 * The biased pass deliberately overshoots to close the gap. The relax pass
 * re-solves the *same* constraints with the bias switched off, which removes
 * exactly the extra velocity the bias added. Without it, a deep stack would
 * visibly "pop" apart.
 *
 * ### Determinism
 *
 * Constraints are built in a fixed, index-sorted order and solved in that same
 * order every step. No parallelism, no floating-point reductions with
 * unspecified order, no `Math` transcendentals. Two machines running the same
 * inputs produce identical bits.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Rot } from './../math/Rot.js';
import type { Body } from './Body.js';
import { BodyType } from './Body.js';
import type { Contact } from './Contact.js';
import type { Joint } from './joints/Joint.js';
import {
  CONTACT_HERTZ,
  CONTACT_DAMPING_RATIO,
  MAX_BIAS_VELOCITY,
  RESTITUTION_ITERATIONS,
  RESTITUTION_TOLERANCE,
  MAX_TRANSLATION,
  MAX_ROTATION,
  RESTITUTION_THRESHOLD,
  TIME_TO_SLEEP,
} from './../util/settings.js';

/* ------------------------------------------------------------------ *
 * Soft constraint coefficients
 * ------------------------------------------------------------------ */

/** The three numbers that turn a spring spec into solver coefficients. */
export interface SoftConstraint {
  biasRate: Scalar;
  massScale: Scalar;
  impulseScale: Scalar;
}

/**
 * Convert `(hertz, dampingRatio, h)` into solver coefficients.
 *
 * `hertz = 0` yields a rigid constraint (no bias, full mass).
 */
export function makeSoft(hertz: Scalar, zeta: Scalar, h: Scalar, out: SoftConstraint): SoftConstraint {
  if (hertz === S.ZERO) {
    out.biasRate = S.ZERO;
    out.massScale = S.ONE;
    out.impulseScale = S.ZERO;
    return out;
  }
  const omega = S.mul(S.TWO_PI, hertz);
  const a1 = S.mulInt(zeta, 2) + S.mul(h, omega);
  const a2 = S.mul(S.mul(h, omega), a1);
  const a3 = S.inv(S.ONE + a2);
  out.biasRate = S.div(omega, a1);
  out.massScale = S.mul(a2, a3);
  out.impulseScale = a3;
  return out;
}

/* ------------------------------------------------------------------ *
 * Solver body — a compact, cache-friendly mirror of a Body
 * ------------------------------------------------------------------ */

/**
 * The per-step view of a body the solver actually touches.
 *
 * Copying the handful of hot fields out of {@link Body} into a dense array
 * means the inner loops walk contiguous memory instead of chasing pointers
 * through user objects, and it keeps the body class free to grow without
 * slowing the solver down.
 */
export class SolverBody {
  /** Linear velocity of the centre of mass. */
  readonly v = Vec2.zero();
  /** Angular velocity. */
  w: Scalar = S.ZERO;
  /** Accumulated position delta this step (relative to the step start). */
  readonly dp = Vec2.zero();
  /** Accumulated rotation delta this step. */
  readonly dq = new Rot();
  /** Rotation at the start of the step. */
  readonly q0 = new Rot();
  /** Current rotation (q0 · dq). */
  readonly q = new Rot();
  /** Centre of mass at the start of the step. */
  readonly c0 = Vec2.zero();
  /** Current centre of mass. */
  readonly c = Vec2.zero();

  invMass: Scalar = S.ZERO;
  invInertia: Scalar = S.ZERO;
  linearDamping: Scalar = S.ZERO;
  angularDamping: Scalar = S.ZERO;
  gravityScale: Scalar = S.ONE;

  /** Back-reference index into the world's body array. */
  bodyIndex = -1;
  /** `true` for static and kinematic bodies. */
  isKinematic = false;
  /** Sleep bookkeeping for the island this body belongs to. */
  enableSleep = true;
}

/* ------------------------------------------------------------------ *
 * Contact constraint
 * ------------------------------------------------------------------ */

/** Per-point solver state for a contact. */
class ContactPointConstraint {
  /** Anchor on A, relative to A's centre of mass, in world axes. */
  readonly rA = Vec2.zero();
  /** Anchor on B, relative to B's centre of mass. */
  readonly rB = Vec2.zero();
  /** Effective mass along the normal. */
  normalMass: Scalar = S.ZERO;
  /** Effective mass along the tangent. */
  tangentMass: Scalar = S.ZERO;
  /** Accumulated normal impulse. */
  normalImpulse: Scalar = S.ZERO;
  /** Accumulated friction impulse. */
  tangentImpulse: Scalar = S.ZERO;
  /** Largest normal impulse seen — reported to the user as impact strength. */
  maxNormalImpulse: Scalar = S.ZERO;
  /** Separation at the start of the step (negative = overlapping). */
  baseSeparation: Scalar = S.ZERO;
  /** Approach speed captured before solving, used for restitution. */
  relativeVelocity: Scalar = S.ZERO;
  /** `true` when this point existed last step (so warm starting is valid). */
  persisted = false;
}

/** Per-contact solver state. */
class ContactConstraint {
  contact!: Contact;
  indexA = -1;
  indexB = -1;
  readonly normal = Vec2.zero();
  friction: Scalar = S.ZERO;
  restitution: Scalar = S.ZERO;
  tangentSpeed: Scalar = S.ZERO;
  pointCount = 0;
  readonly points: [ContactPointConstraint, ContactPointConstraint] = [
    new ContactPointConstraint(),
    new ContactPointConstraint(),
  ];
  /** Soft coefficients, recomputed each step from `h`. */
  readonly soft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
  /** Coefficients used for *static* contacts, which may be stiffer. */
  readonly staticSoft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
}

/* ------------------------------------------------------------------ *
 * Step context
 * ------------------------------------------------------------------ */

/** Everything the solver needs to know about the current step. */
export interface StepContext {
  /** Full step duration, seconds. */
  dt: Scalar;
  /** Sub-step duration (`dt / subStepCount`). */
  h: Scalar;
  /** `1 / dt`, or `0` when `dt === 0`. */
  invDt: Scalar;
  /** `1 / h`. */
  invH: Scalar;
  /** Velocity iterations. */
  velocityIterations: number;
  /** Relax iterations. */
  relaxIterations: number;
  /** World gravity. */
  gravity: Vec2;
  /** `true` when sleeping is enabled world-wide. */
  enableSleep: boolean;
  /** Warm starting on/off (debug switch). */
  enableWarmStarting: boolean;
  /** `true` to run the restitution pass. */
  enableRestitution: boolean;
}

/* ------------------------------------------------------------------ *
 * Solver
 * ------------------------------------------------------------------ */

const _tmp = Vec2.zero();
const _tangent = Vec2.zero();

/** `0.25`, hoisted so the conversion never runs inside a loop. */
const QUARTER = S.fromFloat(0.25);

/** Per-step soft coefficients, shared by every contact. */
const _dynamicSoft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
const _staticSoft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
const _dv = Vec2.zero();
const _P = Vec2.zero();

/**
 * Owns the per-step arrays and runs the pipeline. One instance per
 * {@link World}; all buffers are reused, so a steady-state step allocates
 * nothing.
 */
export class Solver {
  /** Dense solver bodies for this step. */
  readonly bodies: SolverBody[] = [];
  /** Number of active entries in {@link bodies}. */
  bodyCount = 0;

  private constraints: ContactConstraint[] = [];
  private constraintCount = 0;

  /** Joints participating in this step, in id order. */
  private joints: Joint[] = [];
  private jointCount = 0;

  /** Grow the solver body pool to at least `n` entries. */
  private ensureBodies(n: number): void {
    while (this.bodies.length < n) this.bodies.push(new SolverBody());
  }

  private ensureConstraints(n: number): void {
    while (this.constraints.length < n) this.constraints.push(new ContactConstraint());
  }

  /* ---------------------- 0. gather ---------------------- */

  /**
   * Copy the awake, simulated bodies into the dense solver array and stamp
   * each one with its solver index.
   */
  prepareBodies(bodies: (Body | null)[]): void {
    this.bodyCount = 0;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      if (b === null || b === undefined) continue;
      b.solverIndex = -1;
      if (!b.enabled) continue;
      if (b.type === BodyType.Static) continue;
      if (!b.awake) continue;

      this.ensureBodies(this.bodyCount + 1);
      const sb = this.bodies[this.bodyCount]!;
      sb.bodyIndex = b.id;
      sb.v.copyFrom(b.linearVelocity);
      sb.w = b.angularVelocity;
      sb.dp.setZero();
      sb.dq.setIdentity();
      sb.q0.copyFrom(b.transform.q);
      sb.q.copyFrom(b.transform.q);
      sb.c0.copyFrom(b.worldCenter);
      sb.c.copyFrom(b.worldCenter);
      // Remember the pre-step pose so continuous collision can sweep it.
      b.sweepCenter0.copyFrom(b.worldCenter);
      b.sweepRot0.copyFrom(b.transform.q);
      sb.invMass = b.invMass;
      sb.invInertia = b.invInertia;
      sb.linearDamping = b.linearDamping;
      sb.angularDamping = b.angularDamping;
      sb.gravityScale = b.gravityScale;
      sb.isKinematic = b.type === BodyType.Kinematic;
      sb.enableSleep = b.allowSleep;

      b.solverIndex = this.bodyCount;
      this.bodyCount++;
    }
  }

  /* --------------- 1. integrate velocities --------------- */

  /**
   * Apply gravity, accumulated forces and damping.
   *
   * Damping uses the **implicit** form `v *= 1 / (1 + h·d)` rather than the
   * explicit `v *= (1 - h·d)`. The implicit version is unconditionally stable:
   * no combination of `h` and `d` can ever make the velocity flip sign or
   * blow up, which matters when a game drops frames.
   */
  integrateVelocities(ctx: StepContext, bodies: (Body | null)[]): void {
    const h = ctx.h;
    for (let i = 0; i < this.bodyCount; i++) {
      const sb = this.bodies[i]!;
      if (sb.isKinematic) continue;
      const b = bodies[sb.bodyIndex]!;

      // v += h · (g·gravityScale + F/m)
      const ax = S.mulAdd(ctx.gravity.x, sb.gravityScale, S.mul(sb.invMass, b.force.x));
      const ay = S.mulAdd(ctx.gravity.y, sb.gravityScale, S.mul(sb.invMass, b.force.y));
      sb.v.x = S.mulAdd(ax, h, sb.v.x);
      sb.v.y = S.mulAdd(ay, h, sb.v.y);
      sb.w = S.mulAdd(S.mul(sb.invInertia, b.torque), h, sb.w);

      const linDamp = S.inv(S.ONE + S.mul(h, sb.linearDamping));
      const angDamp = S.inv(S.ONE + S.mul(h, sb.angularDamping));
      sb.v.x = S.mul(sb.v.x, linDamp);
      sb.v.y = S.mul(sb.v.y, linDamp);
      sb.w = S.mul(sb.w, angDamp);
    }
  }

  /* ---------------- 2. prepare contacts ------------------ */

  /**
   * Build the contact constraints: anchors, effective masses and the soft
   * coefficients.
   *
   * The **effective mass** along a direction `n` at anchor `r` is
   *
   * ```
   * 1 / ( mA⁻¹ + mB⁻¹ + IA⁻¹(rA×n)² + IB⁻¹(rB×n)² )
   * ```
   *
   * i.e. how much the pair resists an impulse there. Precomputing it once per
   * step turns each solver iteration into a handful of multiplies.
   */
  prepareContacts(ctx: StepContext, contacts: Contact[], _bodies: (Body | null)[]): void {
    this.constraintCount = 0;
    const contactHertz = S.min(CONTACT_HERTZ, S.mul(QUARTER, ctx.invH));

    /*
     * The soft coefficients depend only on (hertz, damping, h) — all identical
     * across contacts — so they are computed twice per step here rather than
     * twice per contact. With a couple of thousand contacts that removed
     * several thousand redundant transcendental-free but still costly
     * evaluations per frame.
     */
    makeSoft(contactHertz, CONTACT_DAMPING_RATIO, ctx.h, _dynamicSoft);
    makeSoft(S.mulInt(contactHertz, 2), CONTACT_DAMPING_RATIO, ctx.h, _staticSoft);

    for (let ci = 0; ci < contacts.length; ci++) {
      const contact = contacts[ci]!;
      contact.constraintIndex = -1;
      if (!contact.isTouching || contact.isSensor || !contact.isEnabled) continue;
      if (contact.manifold.pointCount === 0) continue;

      const bodyA = contact.fixtureA.body;
      const bodyB = contact.fixtureB.body;
      const iA = bodyA.solverIndex;
      const iB = bodyB.solverIndex;
      if (iA < 0 && iB < 0) continue; // both asleep or static

      this.ensureConstraints(this.constraintCount + 1);
      const c = this.constraints[this.constraintCount]!;
      contact.constraintIndex = this.constraintCount;
      this.constraintCount++;

      c.contact = contact;
      c.indexA = iA;
      c.indexB = iB;
      c.normal.copyFrom(contact.manifold.normal);
      c.friction = contact.friction;
      c.restitution = contact.restitution;
      c.tangentSpeed = contact.tangentSpeed;
      c.pointCount = contact.manifold.pointCount;

      const mA = iA >= 0 ? this.bodies[iA]!.invMass : S.ZERO;
      const iiA = iA >= 0 ? this.bodies[iA]!.invInertia : S.ZERO;
      const mB = iB >= 0 ? this.bodies[iB]!.invMass : S.ZERO;
      const iiB = iB >= 0 ? this.bodies[iB]!.invInertia : S.ZERO;

      const cA = iA >= 0 ? this.bodies[iA]!.c : bodyA.worldCenter;
      const cB = iB >= 0 ? this.bodies[iB]!.c : bodyB.worldCenter;

      // A contact against a static body may be stiffer without instability,
      // which makes ground contacts crisp while stacks stay soft.
      const soft = iA < 0 || iB < 0 ? _staticSoft : _dynamicSoft;
      c.soft.biasRate = soft.biasRate;
      c.soft.massScale = soft.massScale;
      c.soft.impulseScale = soft.impulseScale;

      Vec2.rperpTo(_tangent, c.normal); // tangent = (n.y, -n.x)

      for (let j = 0; j < c.pointCount; j++) {
        const mp = contact.manifold.points[j]!;
        const cp = c.points[j]!;

        cp.rA.set(mp.point.x - cA.x, mp.point.y - cA.y);
        cp.rB.set(mp.point.x - cB.x, mp.point.y - cB.y);
        // Cache the anchors on the manifold too, so the debug view and the
        // events can show where the impulse was applied.
        mp.anchorA.copyFrom(cp.rA);
        mp.anchorB.copyFrom(cp.rB);

        cp.baseSeparation = mp.separation;
        cp.normalImpulse = ctx.enableWarmStarting ? mp.normalImpulse : S.ZERO;
        cp.tangentImpulse = ctx.enableWarmStarting ? mp.tangentImpulse : S.ZERO;
        cp.maxNormalImpulse = S.ZERO;
        cp.persisted = mp.persisted;

        const rnA = Vec2.cross(cp.rA, c.normal);
        const rnB = Vec2.cross(cp.rB, c.normal);
        const kNormal =
          mA + mB + S.mul(iiA, S.mul(rnA, rnA)) + S.mul(iiB, S.mul(rnB, rnB));
        cp.normalMass = kNormal > S.ZERO ? S.inv(kNormal) : S.ZERO;

        const rtA = Vec2.cross(cp.rA, _tangent);
        const rtB = Vec2.cross(cp.rB, _tangent);
        const kTangent =
          mA + mB + S.mul(iiA, S.mul(rtA, rtA)) + S.mul(iiB, S.mul(rtB, rtB));
        cp.tangentMass = kTangent > S.ZERO ? S.inv(kTangent) : S.ZERO;

        // Approach speed, needed later for restitution.
        this.relativeVelocityAt(_dv, c, cp);
        cp.relativeVelocity = Vec2.dot(c.normal, _dv);
        mp.relativeVelocity = cp.relativeVelocity;
      }

    }
  }

  /** `out = vB + ωB×rB − vA − ωA×rA` */
  private relativeVelocityAt(out: Vec2, c: ContactConstraint, cp: ContactPointConstraint): Vec2 {
    const a = c.indexA >= 0 ? this.bodies[c.indexA]! : null;
    const b = c.indexB >= 0 ? this.bodies[c.indexB]! : null;
    const vAx = a ? a.v.x - S.mul(a.w, cp.rA.y) : S.ZERO;
    const vAy = a ? a.v.y + S.mul(a.w, cp.rA.x) : S.ZERO;
    const vBx = b ? b.v.x - S.mul(b.w, cp.rB.y) : S.ZERO;
    const vBy = b ? b.v.y + S.mul(b.w, cp.rB.x) : S.ZERO;
    out.set(vBx - vAx, vBy - vAy);
    return out;
  }

  /* ------------------- 3. warm start --------------------- */

  /**
   * Re-apply the impulses accumulated last step.
   *
   * This is the cheapest possible good initial guess: in a settled stack the
   * correct impulses barely change from step to step, so the first iteration
   * already lands almost exactly on the answer.
   */
  warmStart(ctx: StepContext): void {
    if (!ctx.enableWarmStarting) return;
    for (let i = 0; i < this.constraintCount; i++) {
      const c = this.constraints[i]!;
      const a = c.indexA >= 0 ? this.bodies[c.indexA]! : null;
      const b = c.indexB >= 0 ? this.bodies[c.indexB]! : null;
      Vec2.rperpTo(_tangent, c.normal);

      for (let j = 0; j < c.pointCount; j++) {
        const cp = c.points[j]!;
        // P = normalImpulse·n + tangentImpulse·t
        _P.set(
          S.mulAdd(c.normal.x, cp.normalImpulse, S.mul(_tangent.x, cp.tangentImpulse)),
          S.mulAdd(c.normal.y, cp.normalImpulse, S.mul(_tangent.y, cp.tangentImpulse)),
        );
        if (a) {
          a.v.x -= S.mul(a.invMass, _P.x);
          a.v.y -= S.mul(a.invMass, _P.y);
          a.w -= S.mul(a.invInertia, Vec2.cross(cp.rA, _P));
        }
        if (b) {
          b.v.x += S.mul(b.invMass, _P.x);
          b.v.y += S.mul(b.invMass, _P.y);
          b.w += S.mul(b.invInertia, Vec2.cross(cp.rB, _P));
        }
      }
    }
  }

  /* ------------------ 4/6. solve contacts ---------------- */

  /**
   * One sequential-impulse sweep over every contact.
   *
   * @param useBias `true` during the main iterations (soft push-out active),
   *                `false` during relaxation (velocity-only, removes the
   *                energy the bias injected)
   */
  solveContacts(ctx: StepContext, useBias: boolean): void {
    const invH = ctx.invH;
    const bodies = this.bodies;

    for (let i = 0; i < this.constraintCount; i++) {
      const c = this.constraints[i]!;
      const a = c.indexA >= 0 ? bodies[c.indexA]! : null;
      const b = c.indexB >= 0 ? bodies[c.indexB]! : null;

      /*
       * Everything below is hoisted out of the point loop on purpose. A
       * manifold has one or two points and the loop runs `subSteps ×
       * (velocityIterations + relaxIterations)` times per step, so a property
       * load left inside it is paid thousands of times per frame.
       */
      const mA = a !== null ? a.invMass : S.ZERO;
      const iiA = a !== null ? a.invInertia : S.ZERO;
      const mB = b !== null ? b.invMass : S.ZERO;
      const iiB = b !== null ? b.invInertia : S.ZERO;

      const nx = c.normal.x;
      const ny = c.normal.y;
      // tangent = rperp(normal) = (n.y, -n.x)
      const tx = ny;
      const ty = -nx;

      const friction = c.friction;
      const tangentSpeed = c.tangentSpeed;
      const softBiasRate = c.soft.biasRate;
      const softMassScale = c.soft.massScale;
      const softImpulseScale = c.soft.impulseScale;

      // Relative displacement of the two centres so far this step, projected
      // onto the normal once for the whole manifold.
      let dsBase = S.ZERO;
      if (a !== null && b !== null) {
        dsBase = S.mulAdd(b.dp.x - a.dp.x, nx, S.mul(b.dp.y - a.dp.y, ny));
      } else if (a !== null) {
        dsBase = -S.mulAdd(a.dp.x, nx, S.mul(a.dp.y, ny));
      } else if (b !== null) {
        dsBase = S.mulAdd(b.dp.x, nx, S.mul(b.dp.y, ny));
      }

      // Local copies of the velocity state; written back once at the end.
      let vax = S.ZERO, vay = S.ZERO, wa = S.ZERO;
      let vbx = S.ZERO, vby = S.ZERO, wb = S.ZERO;
      if (a !== null) { vax = a.v.x; vay = a.v.y; wa = a.w; }
      if (b !== null) { vbx = b.v.x; vby = b.v.y; wb = b.w; }

      const count = c.pointCount;

      /* ---- normal constraints ---- */
      for (let j = 0; j < count; j++) {
        const cp = c.points[j]!;
        const rax = cp.rA.x, ray = cp.rA.y;
        const rbx = cp.rB.x, rby = cp.rB.y;

        const separation = cp.baseSeparation + dsBase;

        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;

        if (separation > S.ZERO) {
          /*
           * Speculative contact: the shapes are still apart. Rather than
           * pushing, remove exactly the velocity that would close the gap this
           * step, so the bodies land on the surface instead of sinking in.
           */
          bias = S.mul(separation, invH);
        } else if (useBias) {
          // MAX_BIAS_VELOCITY is a module constant: converting a literal with
          // fromFloat here would run on every point, every iteration.
          bias = S.max(S.mul(softBiasRate, separation), MAX_BIAS_VELOCITY);
          massScale = softMassScale;
          impulseScale = softImpulseScale;
        }

        // Relative normal velocity at the contact point.
        const dvx = vbx - S.mul(wb, rby) - vax + S.mul(wa, ray);
        const dvy = vby + S.mul(wb, rbx) - vay - S.mul(wa, rax);
        const vn = S.mulAdd(dvx, nx, S.mul(dvy, ny));

        // λ = −(m·massScale)·(vn + bias) − impulseScale·λ_acc
        let impulse =
          -S.mul(S.mul(cp.normalMass, massScale), vn + bias) -
          S.mul(impulseScale, cp.normalImpulse);

        // Contacts push but never pull: clamp the accumulated impulse.
        const newImpulse = cp.normalImpulse + impulse > S.ZERO ? cp.normalImpulse + impulse : S.ZERO;
        impulse = newImpulse - cp.normalImpulse;
        cp.normalImpulse = newImpulse;
        if (newImpulse > cp.maxNormalImpulse) cp.maxNormalImpulse = newImpulse;

        const px = S.mul(nx, impulse);
        const py = S.mul(ny, impulse);
        vax -= S.mul(mA, px);
        vay -= S.mul(mA, py);
        wa -= S.mul(iiA, S.mulAdd(rax, py, -S.mul(ray, px)));
        vbx += S.mul(mB, px);
        vby += S.mul(mB, py);
        wb += S.mul(iiB, S.mulAdd(rbx, py, -S.mul(rby, px)));
      }

      /* ---- friction constraints ---- */
      for (let j = 0; j < count; j++) {
        const cp = c.points[j]!;
        const rax = cp.rA.x, ray = cp.rA.y;
        const rbx = cp.rB.x, rby = cp.rB.y;

        const dvx = vbx - S.mul(wb, rby) - vax + S.mul(wa, ray);
        const dvy = vby + S.mul(wb, rbx) - vay - S.mul(wa, rax);
        const vt = S.mulAdd(dvx, tx, S.mul(dvy, ty)) - tangentSpeed;

        let impulse = -S.mul(cp.tangentMass, vt);

        // Coulomb's law: |friction| <= μ · normalImpulse.
        const maxFriction = S.mul(friction, cp.normalImpulse);
        let newImpulse = cp.tangentImpulse + impulse;
        if (newImpulse < -maxFriction) newImpulse = -maxFriction;
        else if (newImpulse > maxFriction) newImpulse = maxFriction;
        impulse = newImpulse - cp.tangentImpulse;
        cp.tangentImpulse = newImpulse;

        const px = S.mul(tx, impulse);
        const py = S.mul(ty, impulse);
        vax -= S.mul(mA, px);
        vay -= S.mul(mA, py);
        wa -= S.mul(iiA, S.mulAdd(rax, py, -S.mul(ray, px)));
        vbx += S.mul(mB, px);
        vby += S.mul(mB, py);
        wb += S.mul(iiB, S.mulAdd(rbx, py, -S.mul(rby, px)));
      }

      if (a !== null) { a.v.x = vax; a.v.y = vay; a.w = wa; }
      if (b !== null) { b.v.x = vbx; b.v.y = vby; b.w = wb; }
    }
  }

  /* ------------------ 5. integrate positions -------------- */

  /**
   * Advance positions by `h`, clamping the per-step motion so a single wild
   * velocity cannot teleport a body across the level.
   */
  integratePositions(ctx: StepContext): void {
    const h = ctx.h;
    for (let i = 0; i < this.bodyCount; i++) {
      const sb = this.bodies[i]!;

      // Translation clamp.
      let dx = S.mul(sb.v.x, h);
      let dy = S.mul(sb.v.y, h);
      const tSq = S.mulAdd(dx, dx, S.mul(dy, dy));
      const maxT = S.mul(MAX_TRANSLATION, MAX_TRANSLATION);
      if (tSq > maxT) {
        const ratio = S.div(MAX_TRANSLATION, S.sqrt(tSq));
        dx = S.mul(dx, ratio);
        dy = S.mul(dy, ratio);
        sb.v.x = S.mul(sb.v.x, ratio);
        sb.v.y = S.mul(sb.v.y, ratio);
      }

      // Rotation clamp.
      let dw = S.mul(sb.w, h);
      if (S.abs(dw) > MAX_ROTATION) {
        const ratio = S.div(MAX_ROTATION, S.abs(dw));
        dw = S.mul(dw, ratio);
        sb.w = S.mul(sb.w, ratio);
      }

      sb.dp.x += dx;
      sb.dp.y += dy;
      sb.c.x += dx;
      sb.c.y += dy;
      sb.q.integrate(dw);
    }
  }

  /* -------------------- 7. restitution -------------------- */

  /**
   * Apply bounce as a separate final pass.
   *
   * Running restitution *after* the main solve (rather than folding it into
   * the bias) means the bounce is computed from the true pre-impact approach
   * speed, so a ball dropped from a fixed height always returns to the same
   * height — no energy creeps in or leaks out.
   */
  applyRestitution(ctx: StepContext): void {
    if (!ctx.enableRestitution) return;
    const threshold = RESTITUTION_THRESHOLD;


    /*
     * Restitution is iterated, not applied once.
     *
     * A single sweep cannot carry a shock through a chain of touching bodies.
     * In a Newton's cradle the strike is transmitted ball-to-ball: the first
     * contact bounces, which only then gives the second contact something to
     * bounce, and so on. Solving each contact once in index order leaves the
     * wave half-propagated, and the classic symptom is every ball drifting
     * off together instead of just the last one flying out.
     *
     * Sweeping repeatedly lets the impulse walk down the chain, one contact
     * per iteration, until it converges. The loop exits as soon as a sweep
     * changes nothing, so the usual single-contact case still costs one pass.
     */
    for (let iter = 0; iter < RESTITUTION_ITERATIONS; iter++) {
      if (!this.restitutionPass(threshold)) break;
    }
  }

  /**
   * One restitution sweep.
   * @returns `true` when any impulse was applied, i.e. another sweep may help
   */
  private restitutionPass(threshold: Scalar): boolean {
    let applied = false;

    for (let i = 0; i < this.constraintCount; i++) {
      const c = this.constraints[i]!;
      if (c.restitution === S.ZERO) continue;

      const a = c.indexA >= 0 ? this.bodies[c.indexA]! : null;
      const b = c.indexB >= 0 ? this.bodies[c.indexB]! : null;
      const mA = a ? a.invMass : S.ZERO;
      const iiA = a ? a.invInertia : S.ZERO;
      const mB = b ? b.invMass : S.ZERO;
      const iiB = b ? b.invInertia : S.ZERO;
      const n = c.normal;

      for (let j = 0; j < c.pointCount; j++) {
        const cp = c.points[j]!;
        // Skip resting contacts and points that never actually pushed.
        if (cp.relativeVelocity > -threshold || cp.maxNormalImpulse === S.ZERO) continue;
        const approach = cp.relativeVelocity;

        const vAx = a ? a.v.x - S.mul(a.w, cp.rA.y) : S.ZERO;
        const vAy = a ? a.v.y + S.mul(a.w, cp.rA.x) : S.ZERO;
        const vBx = b ? b.v.x - S.mul(b.w, cp.rB.y) : S.ZERO;
        const vBy = b ? b.v.y + S.mul(b.w, cp.rB.x) : S.ZERO;
        const vn = S.mulAdd(vBx - vAx, n.x, S.mul(vBy - vAy, n.y));

        // Target: vn' = -e · vn_initial
        let impulse = -S.mul(cp.normalMass, vn + S.mul(c.restitution, approach));
        const newImpulse = S.max(cp.normalImpulse + impulse, S.ZERO);
        impulse = newImpulse - cp.normalImpulse;
        cp.normalImpulse = newImpulse;
        cp.maxNormalImpulse = S.max(cp.maxNormalImpulse, newImpulse);

        // A meaningful correction means the wave has not settled yet.
        if (S.abs(impulse) > RESTITUTION_TOLERANCE) applied = true;

        _P.set(S.mul(n.x, impulse), S.mul(n.y, impulse));
        if (a) {
          a.v.x -= S.mul(mA, _P.x);
          a.v.y -= S.mul(mA, _P.y);
          a.w -= S.mul(iiA, Vec2.cross(cp.rA, _P));
        }
        if (b) {
          b.v.x += S.mul(mB, _P.x);
          b.v.y += S.mul(mB, _P.y);
          b.w += S.mul(iiB, Vec2.cross(cp.rB, _P));
        }
      }
    }
    return applied;
  }

  /* ------------------ 8. store & finalise ----------------- */

  /** Copy the accumulated impulses back onto the manifolds for next step. */
  storeImpulses(): void {
    for (let i = 0; i < this.constraintCount; i++) {
      const c = this.constraints[i]!;
      const m = c.contact.manifold;
      for (let j = 0; j < c.pointCount; j++) {
        const cp = c.points[j]!;
        const mp = m.points[j]!;
        mp.normalImpulse = cp.normalImpulse;
        mp.tangentImpulse = cp.tangentImpulse;
        mp.maxNormalImpulse = cp.maxNormalImpulse;
      }
    }
  }

  /**
   * Write the solver state back onto the bodies and refresh their transforms.
   *
   * The body **origin** is recovered from the centre of mass and the new
   * rotation: `p = c - R·localCenter`.
   */
  finalizeBodies(_ctx: StepContext, bodies: (Body | null)[], out: Body[]): void {
    out.length = 0;
    for (let i = 0; i < this.bodyCount; i++) {
      const sb = this.bodies[i]!;
      const b = bodies[sb.bodyIndex]!;

      b.linearVelocity.copyFrom(sb.v);
      b.angularVelocity = sb.w;
      b.worldCenter.copyFrom(sb.c);
      b.transform.q.copyFrom(sb.q);
      Rot.rotate(_tmp, b.transform.q, b.localCenter);
      b.transform.p.set(b.worldCenter.x - _tmp.x, b.worldCenter.y - _tmp.y);

      b.force.setZero();
      b.torque = S.ZERO;

      out.push(b);
    }
  }

  /* ------------------------- joints ---------------------- */

  /** Collect the joints whose bodies are in the solve set. */
  prepareJoints(ctx: StepContext, joints: (Joint | null)[]): void {
    this.jointCount = 0;
    this.joints.length = 0;
    for (let i = 0; i < joints.length; i++) {
      const j = joints[i];
      if (j === null || j === undefined) continue;
      if (!j.isActive()) continue;
      const iA = j.bodyA.solverIndex;
      const iB = j.bodyB.solverIndex;
      if (iA < 0 && iB < 0) continue;
      j.prepare(ctx, this);
      this.joints.push(j);
      this.jointCount++;
    }
  }

  /** Re-apply last step's joint impulses. */
  warmStartJoints(ctx: StepContext): void {
    if (!ctx.enableWarmStarting) return;
    for (let i = 0; i < this.jointCount; i++) this.joints[i]!.warmStart(this);
  }

  /** One joint solve sweep. */
  solveJoints(ctx: StepContext, useBias: boolean): void {
    for (let i = 0; i < this.jointCount; i++) this.joints[i]!.solve(ctx, this, useBias);
  }

  /** Access a solver body by index; `null` for static/sleeping. */
  getBody(index: number): SolverBody | null {
    return index >= 0 ? this.bodies[index]! : null;
  }

  /** Number of contact constraints solved this step. */
  get contactConstraintCount(): number {
    return this.constraintCount;
  }

  /** Number of joints solved this step. */
  get activeJointCount(): number {
    return this.jointCount;
  }

  /**
   * Sleep bookkeeping: a body may only sleep when *every* body it is
   * connected to may also sleep, otherwise a resting box on a moving platform
   * would freeze in mid-air. The world resolves this per island; here we just
   * compute each body's own timer.
   */
  updateSleep(ctx: StepContext, bodies: (Body | null)[]): Scalar {
    if (!ctx.enableSleep) return S.ZERO;
    let minSleepTime = S.MAX_VALUE;
    for (let i = 0; i < this.bodyCount; i++) {
      const sb = this.bodies[i]!;
      const b = bodies[sb.bodyIndex]!;
      const t = b.updateSleepTime(ctx.dt);
      minSleepTime = S.min(minSleepTime, b.allowSleep ? t : S.ZERO);
    }
    return minSleepTime;
  }

  /** Everything a body needs to be considered settled. */
  static get sleepThreshold(): Scalar {
    return TIME_TO_SLEEP;
  }

  /** Reset all per-step arrays (used when the world is cleared). */
  clear(): void {
    this.bodyCount = 0;
    this.constraintCount = 0;
    this.jointCount = 0;
    this.joints.length = 0;
  }
}

