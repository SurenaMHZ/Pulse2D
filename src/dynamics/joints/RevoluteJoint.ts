/**
 * @module dynamics/joints/RevoluteJoint
 *
 * A hinge: the two bodies share a world point and may rotate freely about it.
 *
 * Optional **limits** clamp the relative angle and an optional **motor**
 * drives it. This is the joint behind wheels, ragdoll elbows, swinging doors
 * and pendulums.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Rot } from './../../math/Rot.js';
import { Mat22 } from './../../math/Mat22.js';
import { Joint, JointType } from './Joint.js';
import type { JointDefBase, LimitDef, MotorDef, SpringDef } from './Joint.js';
import { makeSoft } from './../Solver.js';
import type { SoftConstraint, Solver, StepContext } from './../Solver.js';
import { JOINT_HERTZ, JOINT_DAMPING_RATIO } from './../../util/settings.js';

export interface RevoluteJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  /** Anchor in A's local frame. */
  localAnchorA?: { x: number; y: number };
  /** Anchor in B's local frame. */
  localAnchorB?: { x: number; y: number };
  /**
   * The relative angle considered "zero". Defaults to the angle at creation,
   * so limits are naturally measured from the assembled pose.
   */
  referenceAngle?: number;
}

const _tmp = Vec2.zero();
const _bias = Vec2.zero();
const _impulse = Vec2.zero();
const _cdot = Vec2.zero();

export class RevoluteJoint extends Joint {
  readonly type = JointType.Revolute;

  readonly localAnchorA: Vec2;
  readonly localAnchorB: Vec2;
  referenceAngle: Scalar;

  enableLimit: boolean;
  lowerLimit: Scalar;
  upperLimit: Scalar;

  enableMotor: boolean;
  motorSpeed: Scalar;
  maxMotorTorque: Scalar;

  enableSpring: boolean;
  hertz: Scalar;
  dampingRatio: Scalar;

  /** Accumulated point-constraint impulse. */
  private readonly linearImpulse = Vec2.zero();
  private motorImpulse: Scalar = S.ZERO;
  private lowerImpulse: Scalar = S.ZERO;
  private upperImpulse: Scalar = S.ZERO;
  private springImpulse: Scalar = S.ZERO;

  private readonly K = new Mat22();
  private axialMass: Scalar = S.ZERO;
  private readonly soft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
  private readonly springSoft: SoftConstraint = {
    biasRate: S.ZERO,
    massScale: S.ONE,
    impulseScale: S.ZERO,
  };
  /** Relative rotation captured at prepare time. */
  private readonly deltaQ = new Rot();

  constructor(id: number, def: RevoluteJointDef) {
    super(id, def);
    this.localAnchorA = def.localAnchorA
      ? Vec2.of(def.localAnchorA.x, def.localAnchorA.y)
      : Vec2.zero();
    this.localAnchorB = def.localAnchorB
      ? Vec2.of(def.localAnchorB.x, def.localAnchorB.y)
      : Vec2.zero();
    this.referenceAngle =
      def.referenceAngle !== undefined
        ? S.fromFloat(def.referenceAngle)
        : Rot.relativeAngle(def.bodyA.transform.q, def.bodyB.transform.q);

    this.enableLimit = def.enableLimit ?? false;
    this.lowerLimit = S.fromFloat(def.lowerLimit ?? 0);
    this.upperLimit = S.fromFloat(def.upperLimit ?? 0);
    this.enableMotor = def.enableMotor ?? false;
    this.motorSpeed = S.fromFloat(def.motorSpeed ?? 0);
    this.maxMotorTorque = S.fromFloat(def.maxMotorForce ?? 0);
    this.enableSpring = def.enableSpring ?? false;
    this.hertz = S.fromFloat(def.hertz ?? 0);
    this.dampingRatio = S.fromFloat(def.dampingRatio ?? 0);
  }

  /**
   * Build a hinge at a world point, deriving both local anchors from the
   * bodies' current poses. The usual way to create one.
   */
  static atWorldPoint(
    id: number,
    bodyA: RevoluteJointDef['bodyA'],
    bodyB: RevoluteJointDef['bodyB'],
    wx: number,
    wy: number,
    extra?: Partial<RevoluteJointDef>,
  ): RevoluteJoint {
    const w = Vec2.of(wx, wy);
    const la = Vec2.zero();
    const lb = Vec2.zero();
    bodyA.getLocalPoint(la, w);
    bodyB.getLocalPoint(lb, w);
    return new RevoluteJoint(id, {
      ...extra,
      bodyA,
      bodyB,
      localAnchorA: { x: S.toFloat(la.x), y: S.toFloat(la.y) },
      localAnchorB: { x: S.toFloat(lb.x), y: S.toFloat(lb.y) },
    });
  }

  prepare(ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);

    this.computeAnchor(this.rA, this.bodyA, sa, this.localAnchorA);
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorB);

    const qA = sa ? sa.q : this.bodyA.transform.q;
    const qB = sb ? sb.q : this.bodyB.transform.q;
    Rot.mulTTo(this.deltaQ, qA, qB);

    // 2×2 effective mass for the point constraint.
    const mA = this.invMassA;
    const mB = this.invMassB;
    const iA = this.invIA;
    const iB = this.invIB;
    this.K.set(
      mA + mB + S.mul(iA, S.mul(this.rA.y, this.rA.y)) + S.mul(iB, S.mul(this.rB.y, this.rB.y)),
      -S.mul(iA, S.mul(this.rA.x, this.rA.y)) - S.mul(iB, S.mul(this.rB.x, this.rB.y)),
      -S.mul(iA, S.mul(this.rA.x, this.rA.y)) - S.mul(iB, S.mul(this.rB.x, this.rB.y)),
      mA + mB + S.mul(iA, S.mul(this.rA.x, this.rA.x)) + S.mul(iB, S.mul(this.rB.x, this.rB.x)),
    );

    const k = iA + iB;
    this.axialMass = k > S.ZERO ? S.inv(k) : S.ZERO;

    makeSoft(JOINT_HERTZ, JOINT_DAMPING_RATIO, ctx.h, this.soft);
    if (this.enableSpring) makeSoft(this.hertz, this.dampingRatio, ctx.h, this.springSoft);

    if (!ctx.enableWarmStarting) {
      this.linearImpulse.setZero();
      this.motorImpulse = S.ZERO;
      this.lowerImpulse = S.ZERO;
      this.upperImpulse = S.ZERO;
      this.springImpulse = S.ZERO;
    }
  }

  warmStart(solver: Solver): void {
    const axial = this.motorImpulse + this.lowerImpulse - this.upperImpulse + this.springImpulse;
    this.applyImpulse(solver, this.linearImpulse.x, this.linearImpulse.y);
    this.applyAngularImpulse(solver, axial);
  }

  protected override localAnchorAOf(): Vec2 {
    return this.localAnchorA;
  }

  protected override localAnchorBOf(): Vec2 {
    return this.localAnchorB;
  }

  solve(ctx: StepContext, solver: Solver, useBias: boolean): void {
    // The links of a chain swing far within one step, so the anchor arms must
    // follow the bodies rather than stay frozen at prepare time.
    this.refreshAnchors(solver);
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    const wA0 = a ? a.w : S.ZERO;
    const wB0 = b ? b.w : S.ZERO;

    /* ---- spring (soft rotational drive towards the reference) ---- */
    if (this.enableSpring && this.hertz > S.ZERO) {
      const angle = this.currentAngle(solver);
      const cdot = wB0 - wA0;
      const bias = S.mul(this.springSoft.biasRate, angle);
      const impulse =
        -S.mul(S.mul(this.axialMass, this.springSoft.massScale), cdot + bias) -
        S.mul(this.springSoft.impulseScale, this.springImpulse);
      this.springImpulse += impulse;
      this.applyAngularImpulse(solver, impulse);
    }

    /* ---- motor ---- */
    if (this.enableMotor) {
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;
      const cdot = wB - wA - this.motorSpeed;
      let impulse = -S.mul(this.axialMass, cdot);
      const oldImpulse = this.motorImpulse;
      // Clamp the accumulated torque, not the increment.
      const maxImpulse = S.mul(this.maxMotorTorque, ctx.h);
      this.motorImpulse = S.clamp(oldImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.motorImpulse - oldImpulse;
      this.applyAngularImpulse(solver, impulse);
    }

    /* ---- limits ---- */
    if (this.enableLimit) {
      const angle = this.currentAngle(solver);

      // lower limit: angle - lower >= 0
      {
        const c = angle - this.lowerLimit;
        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;
        if (c > S.ZERO) {
          bias = S.mul(c, ctx.invH); // speculative
        } else if (useBias) {
          bias = S.mul(this.soft.biasRate, c);
          massScale = this.soft.massScale;
          impulseScale = this.soft.impulseScale;
        }
        const wA = a ? a.w : S.ZERO;
        const wB = b ? b.w : S.ZERO;
        const cdot = wB - wA;
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.lowerImpulse);
        const newImpulse = S.max(this.lowerImpulse + impulse, S.ZERO);
        impulse = newImpulse - this.lowerImpulse;
        this.lowerImpulse = newImpulse;
        this.applyAngularImpulse(solver, impulse);
      }

      // upper limit: upper - angle >= 0
      {
        const c = this.upperLimit - angle;
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
        const wA = a ? a.w : S.ZERO;
        const wB = b ? b.w : S.ZERO;
        const cdot = wA - wB;
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.upperImpulse);
        const newImpulse = S.max(this.upperImpulse + impulse, S.ZERO);
        impulse = newImpulse - this.upperImpulse;
        this.upperImpulse = newImpulse;
        this.applyAngularImpulse(solver, -impulse);
      }
    }

    /* ---- point constraint (always active) ---- */
    {
      // K depends on rA/rB, which just changed.
      const mA = this.invMassA;
      const mB = this.invMassB;
      const iA = this.invIA;
      const iB = this.invIB;
      this.K.set(
        mA + mB + S.mul(iA, S.mul(this.rA.y, this.rA.y)) + S.mul(iB, S.mul(this.rB.y, this.rB.y)),
        -S.mul(iA, S.mul(this.rA.x, this.rA.y)) - S.mul(iB, S.mul(this.rB.x, this.rB.y)),
        -S.mul(iA, S.mul(this.rA.x, this.rA.y)) - S.mul(iB, S.mul(this.rB.x, this.rB.y)),
        mA + mB + S.mul(iA, S.mul(this.rA.x, this.rA.x)) + S.mul(iB, S.mul(this.rB.x, this.rB.x)),
      );

      const vA = a ? a.v : Vec2.zero();
      const vB = b ? b.v : Vec2.zero();
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;

      // Cdot = vB + ωB×rB − vA − ωA×rA
      _cdot.set(
        vB.x - S.mul(wB, this.rB.y) - vA.x + S.mul(wA, this.rA.y),
        vB.y + S.mul(wB, this.rB.x) - vA.y - S.mul(wA, this.rA.x),
      );

      _bias.setZero();
      let massScale = S.ONE;
      let impulseScale = S.ZERO;
      if (useBias) {
        // Positional error = (dcB + rB) − (dcA + rA), tracked from the
        // sub-step deltas so no extra transform work is needed.
        /*
         * Anchor separation = (cB + rB) − (cA + rA).
         *
         * `deltaCenter` and the arms are refreshed from the live sub-step
         * state above, so this is the true current error — adding the `dp`
         * deltas on top would count the same motion twice.
         */
        const sepX = this.deltaCenter.x + this.rB.x - this.rA.x;
        const sepY = this.deltaCenter.y + this.rB.y - this.rA.y;
        _bias.set(S.mul(this.soft.biasRate, sepX), S.mul(this.soft.biasRate, sepY));
        massScale = this.soft.massScale;
        impulseScale = this.soft.impulseScale;
      }

      _tmp.set(_cdot.x + _bias.x, _cdot.y + _bias.y);
      this.K.solve(_impulse, _tmp);
      const ix = -S.mul(massScale, _impulse.x) - S.mul(impulseScale, this.linearImpulse.x);
      const iy = -S.mul(massScale, _impulse.y) - S.mul(impulseScale, this.linearImpulse.y);
      this.linearImpulse.x += ix;
      this.linearImpulse.y += iy;
      this.applyImpulse(solver, ix, iy);
    }
  }

  /** Relative angle minus the reference, using the current sub-step rotations. */
  private currentAngle(solver: Solver): Scalar {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    const qA = a ? a.q : this.bodyA.transform.q;
    const qB = b ? b.q : this.bodyB.transform.q;
    return Rot.relativeAngle(qA, qB) - this.referenceAngle;
  }

  /** Current joint angle in radians, relative to the reference. */
  getJointAngle(): Scalar {
    return Rot.relativeAngle(this.bodyA.transform.q, this.bodyB.transform.q) - this.referenceAngle;
  }

  /** Current relative angular velocity, rad/s. */
  getJointSpeed(): Scalar {
    return this.bodyB.angularVelocity - this.bodyA.angularVelocity;
  }

  /** Torque currently applied by the motor, N·m. */
  getMotorTorque(invDt: Scalar): Scalar {
    return S.mul(this.motorImpulse, invDt);
  }

  setLimits(lower: number, upper: number): void {
    this.lowerLimit = S.fromFloat(lower);
    this.upperLimit = S.fromFloat(upper);
    this.lowerImpulse = S.ZERO;
    this.upperImpulse = S.ZERO;
    this.wake();
  }

  setMotorSpeed(speed: number): void {
    this.motorSpeed = S.fromFloat(speed);
    this.wake();
  }

  setMaxMotorTorque(torque: number): void {
    this.maxMotorTorque = S.fromFloat(torque);
    this.wake();
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    return Vec2.scaleTo(out, this.linearImpulse, invDt);
  }

  getReactionTorque(invDt: Scalar): Scalar {
    return S.mul(this.motorImpulse + this.lowerImpulse - this.upperImpulse, invDt);
  }

  getAnchorA(out: Vec2): Vec2 {
    return this.bodyA.getWorldPoint(out, this.localAnchorA);
  }

  getAnchorB(out: Vec2): Vec2 {
    return this.bodyB.getWorldPoint(out, this.localAnchorB);
  }

  saveState(out: number[]): void {
    out.push(
      this.linearImpulse.x as number,
      this.linearImpulse.y as number,
      this.motorImpulse as number,
      this.lowerImpulse as number,
      this.upperImpulse as number,
      this.springImpulse as number,
    );
  }

  loadState(data: number[], offset: number): number {
    this.linearImpulse.set(data[offset]!, data[offset + 1]!);
    this.motorImpulse = data[offset + 2]!;
    this.lowerImpulse = data[offset + 3]!;
    this.upperImpulse = data[offset + 4]!;
    this.springImpulse = data[offset + 5]!;
    return offset + 6;
  }
}
