/**
 * @module dynamics/joints/DistanceJoint
 *
 * Keeps two anchor points a fixed distance apart.
 *
 * With `enableSpring` it becomes a damped spring; with `enableLimit` it
 * behaves like a rope (free below the maximum, taut at it). Combining the two
 * gives suspension, bungees and cloth strands.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Joint, JointType } from './Joint.js';
import type { JointDefBase, LimitDef, MotorDef, SpringDef } from './Joint.js';
import { makeSoft } from './../Solver.js';
import type { SoftConstraint, Solver, StepContext } from './../Solver.js';
import { JOINT_HERTZ, JOINT_DAMPING_RATIO } from './../../util/settings.js';

export interface DistanceJointDef extends JointDefBase, SpringDef, LimitDef, MotorDef {
  localAnchorA?: { x: number; y: number };
  localAnchorB?: { x: number; y: number };
  /** Rest length. Defaults to the distance between the anchors at creation. */
  length?: number;
  /** Minimum length when limits are enabled. */
  minLength?: number;
  /** Maximum length when limits are enabled. */
  maxLength?: number;
  /** Keep the length exactly at `length` (ignores limits). Default `true`. */
  enableRigid?: boolean;
}

const _axis = Vec2.zero();
const _pA = Vec2.zero();
const _pB = Vec2.zero();

export class DistanceJoint extends Joint {
  readonly type = JointType.Distance;

  readonly localAnchorA: Vec2;
  readonly localAnchorB: Vec2;
  length: Scalar;
  minLength: Scalar;
  maxLength: Scalar;

  enableSpring: boolean;
  hertz: Scalar;
  dampingRatio: Scalar;
  enableLimit: boolean;
  enableRigid: boolean;

  enableMotor: boolean;
  motorSpeed: Scalar;
  maxMotorForce: Scalar;

  private impulse: Scalar = S.ZERO;
  private lowerImpulse: Scalar = S.ZERO;
  private upperImpulse: Scalar = S.ZERO;
  private motorImpulse: Scalar = S.ZERO;

  private axialMass: Scalar = S.ZERO;
  private readonly axis = Vec2.zero();
  /** Anchor separation vector captured at prepare time. */
  private readonly baseSeparation = Vec2.zero();
  private readonly soft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
  private readonly springSoft: SoftConstraint = {
    biasRate: S.ZERO,
    massScale: S.ONE,
    impulseScale: S.ZERO,
  };

  constructor(id: number, def: DistanceJointDef) {
    super(id, def);
    this.localAnchorA = def.localAnchorA
      ? Vec2.of(def.localAnchorA.x, def.localAnchorA.y)
      : Vec2.zero();
    this.localAnchorB = def.localAnchorB
      ? Vec2.of(def.localAnchorB.x, def.localAnchorB.y)
      : Vec2.zero();

    if (def.length !== undefined) {
      this.length = S.fromFloat(def.length);
    } else {
      def.bodyA.getWorldPoint(_pA, this.localAnchorA);
      def.bodyB.getWorldPoint(_pB, this.localAnchorB);
      this.length = Vec2.distance(_pA, _pB);
    }
    this.length = S.max(this.length, S.fromFloat(0.001));
    this.minLength = def.minLength !== undefined ? S.fromFloat(def.minLength) : this.length;
    this.maxLength = def.maxLength !== undefined ? S.fromFloat(def.maxLength) : this.length;

    this.enableSpring = def.enableSpring ?? false;
    this.hertz = S.fromFloat(def.hertz ?? 0);
    this.dampingRatio = S.fromFloat(def.dampingRatio ?? 0);
    this.enableLimit = def.enableLimit ?? false;
    this.enableRigid = def.enableRigid ?? !this.enableSpring;
    this.enableMotor = def.enableMotor ?? false;
    this.motorSpeed = S.fromFloat(def.motorSpeed ?? 0);
    this.maxMotorForce = S.fromFloat(def.maxMotorForce ?? 0);
  }

  prepare(ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);

    this.computeAnchor(this.rA, this.bodyA, sa, this.localAnchorA);
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorB);

    // Separation vector between the two anchors.
    _axis.set(
      this.deltaCenter.x + this.rB.x - this.rA.x,
      this.deltaCenter.y + this.rB.y - this.rA.y,
    );
    this.baseSeparation.copyFrom(_axis);
    this.axis.copyFrom(_axis);
    if (this.axis.normalize() === S.ZERO) this.axis.set(S.ZERO, S.ONE);

    // Effective mass along the axis.
    const crA = Vec2.cross(this.rA, this.axis);
    const crB = Vec2.cross(this.rB, this.axis);
    const k =
      this.invMassA +
      this.invMassB +
      S.mul(this.invIA, S.mul(crA, crA)) +
      S.mul(this.invIB, S.mul(crB, crB));
    this.axialMass = k > S.ZERO ? S.inv(k) : S.ZERO;

    makeSoft(JOINT_HERTZ, JOINT_DAMPING_RATIO, ctx.h, this.soft);
    if (this.enableSpring) makeSoft(this.hertz, this.dampingRatio, ctx.h, this.springSoft);

    if (!ctx.enableWarmStarting) {
      this.impulse = S.ZERO;
      this.lowerImpulse = S.ZERO;
      this.upperImpulse = S.ZERO;
      this.motorImpulse = S.ZERO;
    }
  }

  /**
   * Re-apply the accumulated impulse.
   *
   * Called at the start of every sub-step. The accumulators are then reset:
   * the impulse now lives in the bodies' velocities, and leaving it in place
   * would make the next sub-step apply it a second time. Contacts get away
   * without this because their accumulators are re-clamped against the
   * non-negativity and Coulomb conditions each iteration; a joint's impulse is
   * unbounded, so the feedback compounds and a heavy body diverges.
   */
  warmStart(solver: Solver): void {
    const total = this.impulse + this.lowerImpulse - this.upperImpulse + this.motorImpulse;
    this.applyImpulse(solver, S.mul(this.axis.x, total), S.mul(this.axis.y, total));
  }

  solve(ctx: StepContext, solver: Solver, useBias: boolean): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);

    /** Relative velocity along the joint axis. */
    const axialVelocity = (): Scalar => {
      const vA = a ? a.v : Vec2.zero();
      const vB = b ? b.v : Vec2.zero();
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;
      const vx = vB.x - S.mul(wB, this.rB.y) - vA.x + S.mul(wA, this.rA.y);
      const vy = vB.y + S.mul(wB, this.rB.x) - vA.y - S.mul(wA, this.rA.x);
      return S.mulAdd(vx, this.axis.x, S.mul(vy, this.axis.y));
    };

    /**
     * Length including this step's accumulated motion.
     *
     * The separation vector is rebuilt from the sub-step deltas and its true
     * magnitude is taken, rather than projecting the delta onto the axis
     * cached at prepare time. For a pendulum the axis rotates by tens of
     * degrees within a single step, and the projection then badly
     * underestimates the length — the joint reads itself as slack, hauls the
     * body inwards, and a heavy bob collapses onto its anchor. Recomputing
     * costs one sqrt per iteration and is unconditionally stable.
     */
    const length = (): Scalar => {
      const dpA = a ? a.dp : Vec2.zero();
      const dpB = b ? b.dp : Vec2.zero();
      const dx = this.baseSeparation.x + dpB.x - dpA.x;
      const dy = this.baseSeparation.y + dpB.y - dpA.y;
      const len = S.sqrt(S.mulAdd(dx, dx, S.mul(dy, dy)));
      if (len > S.EPSILON) {
        // Keep the constraint axis aligned with the real separation.
        const inv = S.inv(len);
        this.axis.set(S.mul(dx, inv), S.mul(dy, inv));
      }
      return len;
    };

    const push = (impulse: Scalar): void => {
      this.applyImpulse(solver, S.mul(this.axis.x, impulse), S.mul(this.axis.y, impulse));
    };

    /* ---- spring ---- */
    if (this.enableSpring && this.hertz > S.ZERO && !this.enableRigid) {
      const c = length() - this.length;   // refreshes this.axis
      const bias = S.mul(this.springSoft.biasRate, c);
      const cdot = axialVelocity();
      const impulse =
        -S.mul(S.mul(this.axialMass, this.springSoft.massScale), cdot + bias) -
        S.mul(this.springSoft.impulseScale, this.impulse);
      this.impulse += impulse;
      push(impulse);
    }

    /* ---- motor ---- */
    if (this.enableMotor) {
      length();                            // refresh this.axis
      const cdot = axialVelocity() - this.motorSpeed;
      let impulse = -S.mul(this.axialMass, cdot);
      const old = this.motorImpulse;
      const maxImpulse = S.mul(this.maxMotorForce, ctx.h);
      this.motorImpulse = S.clamp(old + impulse, -maxImpulse, maxImpulse);
      impulse = this.motorImpulse - old;
      push(impulse);
    }

    /* ---- rigid length ---- */
    if (this.enableRigid) {
      // length() refreshes `this.axis`; it must run before axialVelocity()
      // and push() so the error and the impulse share one direction.
      const c = length() - this.length;
      let bias = S.ZERO;
      let massScale = S.ONE;
      let impulseScale = S.ZERO;
      if (useBias) {
        bias = S.mul(this.soft.biasRate, c);
        massScale = this.soft.massScale;
        impulseScale = this.soft.impulseScale;
      }
      const cdot = axialVelocity();
      const impulse =
        -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
        S.mul(impulseScale, this.impulse);
      this.impulse += impulse;
      push(impulse);
      return; // limits are meaningless when the length is pinned
    }

    /* ---- limits ---- */
    if (this.enableLimit) {
      const len = length();

      // lower: len - minLength >= 0
      {
        const c = len - this.minLength;
        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;
        if (c > S.ZERO) {
          bias = S.mul(c, ctx.invH);
        } else if (useBias) {
          bias = S.mul(this.soft.biasRate, c);
          massScale = this.soft.massScale;
          impulseScale = this.soft.impulseScale;
        }
        const cdot = axialVelocity();
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.lowerImpulse);
        const newImpulse = S.max(this.lowerImpulse + impulse, S.ZERO);
        impulse = newImpulse - this.lowerImpulse;
        this.lowerImpulse = newImpulse;
        push(impulse);
      }

      // upper: maxLength - len >= 0
      {
        const c = this.maxLength - len;
        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;
        if (c > S.ZERO) {
          bias = S.mul(c, ctx.invH);
        } else if (useBias) {
          bias = S.mul(this.soft.biasRate, c);
          massScale = this.soft.massScale;
          impulseScale = this.soft.impulseScale;
        }
        const cdot = -axialVelocity();
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.upperImpulse);
        const newImpulse = S.max(this.upperImpulse + impulse, S.ZERO);
        impulse = newImpulse - this.upperImpulse;
        this.upperImpulse = newImpulse;
        push(-impulse);
      }
    }
  }

  /** Current distance between the anchors. */
  getCurrentLength(): Scalar {
    this.bodyA.getWorldPoint(_pA, this.localAnchorA);
    this.bodyB.getWorldPoint(_pB, this.localAnchorB);
    return Vec2.distance(_pA, _pB);
  }

  setLength(length: number): void {
    this.length = S.max(S.fromFloat(length), S.fromFloat(0.001));
    this.impulse = S.ZERO;
    this.wake();
  }

  setLengthRange(min: number, max: number): void {
    this.minLength = S.fromFloat(min);
    this.maxLength = S.fromFloat(max);
    this.lowerImpulse = S.ZERO;
    this.upperImpulse = S.ZERO;
    this.wake();
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    const total = this.impulse + this.lowerImpulse - this.upperImpulse + this.motorImpulse;
    return Vec2.scaleTo(out, this.axis, S.mul(total, invDt));
  }

  getReactionTorque(_invDt: Scalar): Scalar {
    return S.ZERO;
  }

  getAnchorA(out: Vec2): Vec2 {
    return this.bodyA.getWorldPoint(out, this.localAnchorA);
  }

  getAnchorB(out: Vec2): Vec2 {
    return this.bodyB.getWorldPoint(out, this.localAnchorB);
  }

  saveState(out: number[]): void {
    out.push(
      this.impulse as number,
      this.lowerImpulse as number,
      this.upperImpulse as number,
      this.motorImpulse as number,
    );
  }

  loadState(data: number[], offset: number): number {
    this.impulse = data[offset]!;
    this.lowerImpulse = data[offset + 1]!;
    this.upperImpulse = data[offset + 2]!;
    this.motorImpulse = data[offset + 3]!;
    return offset + 4;
  }
}
