/**
 * @module dynamics/joints/PrismaticJoint
 *
 * A slider: body B may translate along one axis of body A and nothing else —
 * the perpendicular translation and the relative rotation are both locked.
 *
 * With a motor and limits this is an elevator, a piston, a sliding door or a
 * suspension strut.
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

export interface PrismaticJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  localAnchorA?: { x: number; y: number };
  localAnchorB?: { x: number; y: number };
  /** Slide axis in A's local frame. Normalised automatically. Default `(1,0)`. */
  localAxisA?: { x: number; y: number };
  /** Locked relative angle. Defaults to the angle at creation. */
  referenceAngle?: number;
}

const _axis = Vec2.zero();
const _perp = Vec2.zero();
const _d = Vec2.zero();
const _imp = Vec2.zero();
const _rhs = Vec2.zero();

export class PrismaticJoint extends Joint {
  readonly type = JointType.Prismatic;

  readonly localAnchorA: Vec2;
  readonly localAnchorB: Vec2;
  readonly localAxisA: Vec2;
  referenceAngle: Scalar;

  enableLimit: boolean;
  lowerLimit: Scalar;
  upperLimit: Scalar;
  enableMotor: boolean;
  motorSpeed: Scalar;
  maxMotorForce: Scalar;
  enableSpring: boolean;
  hertz: Scalar;
  dampingRatio: Scalar;

  /** `[perpendicular, angular]` accumulated impulse. */
  private readonly impulse = Vec2.zero();
  private motorImpulse: Scalar = S.ZERO;
  private lowerImpulse: Scalar = S.ZERO;
  private upperImpulse: Scalar = S.ZERO;
  private springImpulse: Scalar = S.ZERO;

  /** World-space slide axis and its perpendicular, refreshed each step. */
  private readonly axis = Vec2.zero();
  private readonly perp = Vec2.zero();
  private translation: Scalar = S.ZERO;
  /** Perpendicular anchor offset at prepare time; the constraint drives it to 0. */
  private perpOffset: Scalar = S.ZERO;
  private a1: Scalar = S.ZERO;
  private a2: Scalar = S.ZERO;
  private s1: Scalar = S.ZERO;
  private s2: Scalar = S.ZERO;
  private axialMass: Scalar = S.ZERO;
  private readonly K = new Mat22();
  private readonly soft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
  private readonly springSoft: SoftConstraint = {
    biasRate: S.ZERO,
    massScale: S.ONE,
    impulseScale: S.ZERO,
  };

  constructor(id: number, def: PrismaticJointDef) {
    super(id, def);
    this.localAnchorA = def.localAnchorA
      ? Vec2.of(def.localAnchorA.x, def.localAnchorA.y)
      : Vec2.zero();
    this.localAnchorB = def.localAnchorB
      ? Vec2.of(def.localAnchorB.x, def.localAnchorB.y)
      : Vec2.zero();
    this.localAxisA = def.localAxisA ? Vec2.of(def.localAxisA.x, def.localAxisA.y) : Vec2.of(1, 0);
    if (this.localAxisA.normalize() === S.ZERO) this.localAxisA.set(S.ONE, S.ZERO);
    this.referenceAngle =
      def.referenceAngle !== undefined
        ? S.fromFloat(def.referenceAngle)
        : Rot.relativeAngle(def.bodyA.transform.q, def.bodyB.transform.q);

    this.enableLimit = def.enableLimit ?? false;
    this.lowerLimit = S.fromFloat(def.lowerLimit ?? 0);
    this.upperLimit = S.fromFloat(def.upperLimit ?? 0);
    this.enableMotor = def.enableMotor ?? false;
    this.motorSpeed = S.fromFloat(def.motorSpeed ?? 0);
    this.maxMotorForce = S.fromFloat(def.maxMotorForce ?? 0);
    this.enableSpring = def.enableSpring ?? false;
    this.hertz = S.fromFloat(def.hertz ?? 0);
    this.dampingRatio = S.fromFloat(def.dampingRatio ?? 0);
  }

  prepare(ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);

    this.computeAnchor(this.rA, this.bodyA, sa, this.localAnchorA);
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorB);

    const qA = sa ? sa.q : this.bodyA.transform.q;
    Rot.rotate(this.axis, qA, this.localAxisA);
    Vec2.perpTo(this.perp, this.axis);

    // d = separation between the anchors
    _d.set(
      this.deltaCenter.x + this.rB.x - this.rA.x,
      this.deltaCenter.y + this.rB.y - this.rA.y,
    );
    this.translation = Vec2.dot(_d, this.axis);
    this.perpOffset = Vec2.dot(_d, this.perp);

    // Jacobian scalars.
    this.a1 = Vec2.cross(_d, this.axis) + Vec2.cross(this.rA, this.axis);
    this.a2 = Vec2.cross(this.rB, this.axis);
    this.s1 = Vec2.cross(_d, this.perp) + Vec2.cross(this.rA, this.perp);
    this.s2 = Vec2.cross(this.rB, this.perp);

    const mA = this.invMassA;
    const mB = this.invMassB;
    const iA = this.invIA;
    const iB = this.invIB;

    const k11 = mA + mB + S.mul(iA, S.mul(this.s1, this.s1)) + S.mul(iB, S.mul(this.s2, this.s2));
    const k12 = S.mul(iA, this.s1) + S.mul(iB, this.s2);
    const k22 = iA + iB > S.ZERO ? iA + iB : S.ONE;
    this.K.set(k11, k12, k12, k22);

    const ka = mA + mB + S.mul(iA, S.mul(this.a1, this.a1)) + S.mul(iB, S.mul(this.a2, this.a2));
    this.axialMass = ka > S.ZERO ? S.inv(ka) : S.ZERO;

    makeSoft(JOINT_HERTZ, JOINT_DAMPING_RATIO, ctx.h, this.soft);
    if (this.enableSpring) makeSoft(this.hertz, this.dampingRatio, ctx.h, this.springSoft);

    if (!ctx.enableWarmStarting) {
      this.impulse.setZero();
      this.motorImpulse = S.ZERO;
      this.lowerImpulse = S.ZERO;
      this.upperImpulse = S.ZERO;
      this.springImpulse = S.ZERO;
    }
  }

  warmStart(solver: Solver): void {
    const axial = this.motorImpulse + this.lowerImpulse - this.upperImpulse + this.springImpulse;
    const px = S.mulAdd(this.axis.x, axial, S.mul(this.perp.x, this.impulse.x));
    const py = S.mulAdd(this.axis.y, axial, S.mul(this.perp.y, this.impulse.y));
    const la = S.mulAdd(axial, this.a1, S.mulAdd(this.impulse.x, this.s1, this.impulse.y));
    const lb = S.mulAdd(axial, this.a2, S.mulAdd(this.impulse.x, this.s2, this.impulse.y));

    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    if (a) {
      a.v.x -= S.mul(a.invMass, px);
      a.v.y -= S.mul(a.invMass, py);
      a.w -= S.mul(a.invInertia, la);
    }
    if (b) {
      b.v.x += S.mul(b.invMass, px);
      b.v.y += S.mul(b.invMass, py);
      b.w += S.mul(b.invInertia, lb);
    }
  }

  /** Apply an impulse along the slide axis. */
  private pushAxial(solver: Solver, impulse: Scalar): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    const px = S.mul(this.axis.x, impulse);
    const py = S.mul(this.axis.y, impulse);
    if (a) {
      a.v.x -= S.mul(a.invMass, px);
      a.v.y -= S.mul(a.invMass, py);
      a.w -= S.mul(a.invInertia, S.mul(impulse, this.a1));
    }
    if (b) {
      b.v.x += S.mul(b.invMass, px);
      b.v.y += S.mul(b.invMass, py);
      b.w += S.mul(b.invInertia, S.mul(impulse, this.a2));
    }
  }

  /** Current translation including this step's motion. */
  private currentTranslation(solver: Solver): Scalar {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    const dpA = a ? a.dp : Vec2.zero();
    const dpB = b ? b.dp : Vec2.zero();
    return this.translation + S.mulAdd(dpB.x - dpA.x, this.axis.x, S.mul(dpB.y - dpA.y, this.axis.y));
  }

  /** Relative velocity along the slide axis. */
  private axialVelocity(solver: Solver): Scalar {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    const vA = a ? a.v : Vec2.zero();
    const vB = b ? b.v : Vec2.zero();
    const wA = a ? a.w : S.ZERO;
    const wB = b ? b.w : S.ZERO;
    return (
      S.mulAdd(vB.x - vA.x, this.axis.x, S.mul(vB.y - vA.y, this.axis.y)) +
      S.mul(this.a2, wB) -
      S.mul(this.a1, wA)
    );
  }

  solve(ctx: StepContext, solver: Solver, useBias: boolean): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);

    /* ---- spring along the axis ---- */
    if (this.enableSpring && this.hertz > S.ZERO) {
      const c = this.currentTranslation(solver);
      const bias = S.mul(this.springSoft.biasRate, c);
      const cdot = this.axialVelocity(solver);
      const impulse =
        -S.mul(S.mul(this.axialMass, this.springSoft.massScale), cdot + bias) -
        S.mul(this.springSoft.impulseScale, this.springImpulse);
      this.springImpulse += impulse;
      this.pushAxial(solver, impulse);
    }

    /* ---- motor ---- */
    if (this.enableMotor) {
      const cdot = this.axialVelocity(solver) - this.motorSpeed;
      let impulse = -S.mul(this.axialMass, cdot);
      const old = this.motorImpulse;
      const maxImpulse = S.mul(this.maxMotorForce, ctx.h);
      this.motorImpulse = S.clamp(old + impulse, -maxImpulse, maxImpulse);
      impulse = this.motorImpulse - old;
      this.pushAxial(solver, impulse);
    }

    /* ---- limits ---- */
    if (this.enableLimit) {
      const t = this.currentTranslation(solver);
      // lower
      {
        const c = t - this.lowerLimit;
        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;
        if (c > S.ZERO) bias = S.mul(c, ctx.invH);
        else if (useBias) {
          bias = S.mul(this.soft.biasRate, c);
          massScale = this.soft.massScale;
          impulseScale = this.soft.impulseScale;
        }
        const cdot = this.axialVelocity(solver);
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.lowerImpulse);
        const ni = S.max(this.lowerImpulse + impulse, S.ZERO);
        impulse = ni - this.lowerImpulse;
        this.lowerImpulse = ni;
        this.pushAxial(solver, impulse);
      }
      // upper
      {
        const c = this.upperLimit - t;
        let bias = S.ZERO;
        let massScale = S.ONE;
        let impulseScale = S.ZERO;
        if (c > S.ZERO) bias = S.mul(c, ctx.invH);
        else if (useBias) {
          bias = S.mul(this.soft.biasRate, c);
          massScale = this.soft.massScale;
          impulseScale = this.soft.impulseScale;
        }
        const cdot = -this.axialVelocity(solver);
        let impulse =
          -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
          S.mul(impulseScale, this.upperImpulse);
        const ni = S.max(this.upperImpulse + impulse, S.ZERO);
        impulse = ni - this.upperImpulse;
        this.upperImpulse = ni;
        this.pushAxial(solver, -impulse);
      }
    }

    /* ---- the two locked DOFs: perpendicular translation + rotation ---- */
    {
      const vA = a ? a.v : Vec2.zero();
      const vB = b ? b.v : Vec2.zero();
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;

      const cdot1 =
        S.mulAdd(vB.x - vA.x, this.perp.x, S.mul(vB.y - vA.y, this.perp.y)) +
        S.mul(this.s2, wB) -
        S.mul(this.s1, wA);
      const cdot2 = wB - wA;

      let bias1 = S.ZERO;
      let bias2 = S.ZERO;
      let massScale = S.ONE;
      let impulseScale = S.ZERO;
      if (useBias) {
        const dpA = a ? a.dp : Vec2.zero();
        const dpB = b ? b.dp : Vec2.zero();
        const dx = dpB.x - dpA.x;
        const dy = dpB.y - dpA.y;
        const c1 = this.perpOffset + S.mulAdd(dx, this.perp.x, S.mul(dy, this.perp.y));
        const qA = a ? a.q : this.bodyA.transform.q;
        const qB = b ? b.q : this.bodyB.transform.q;
        const c2 = Rot.relativeAngle(qA, qB) - this.referenceAngle;
        bias1 = S.mul(this.soft.biasRate, c1);
        bias2 = S.mul(this.soft.biasRate, c2);
        massScale = this.soft.massScale;
        impulseScale = this.soft.impulseScale;
      }

      _rhs.set(cdot1 + bias1, cdot2 + bias2);
      this.K.solve(_imp, _rhs);
      const ix = -S.mul(massScale, _imp.x) - S.mul(impulseScale, this.impulse.x);
      const iy = -S.mul(massScale, _imp.y) - S.mul(impulseScale, this.impulse.y);
      this.impulse.x += ix;
      this.impulse.y += iy;

      const px = S.mul(this.perp.x, ix);
      const py = S.mul(this.perp.y, ix);
      const la = S.mulAdd(ix, this.s1, iy);
      const lb = S.mulAdd(ix, this.s2, iy);
      if (a) {
        a.v.x -= S.mul(a.invMass, px);
        a.v.y -= S.mul(a.invMass, py);
        a.w -= S.mul(a.invInertia, la);
      }
      if (b) {
        b.v.x += S.mul(b.invMass, px);
        b.v.y += S.mul(b.invMass, py);
        b.w += S.mul(b.invInertia, lb);
      }
    }
  }

  /** Current translation along the axis, metres. */
  getJointTranslation(): Scalar {
    const pA = Vec2.zero();
    const pB = Vec2.zero();
    this.bodyA.getWorldPoint(pA, this.localAnchorA);
    this.bodyB.getWorldPoint(pB, this.localAnchorB);
    Vec2.subTo(_d, pB, pA);
    Rot.rotate(_axis, this.bodyA.transform.q, this.localAxisA);
    return Vec2.dot(_d, _axis);
  }

  /** Current slide speed, m/s. */
  getJointSpeed(): Scalar {
    Rot.rotate(_axis, this.bodyA.transform.q, this.localAxisA);
    Vec2.subTo(_perp, this.bodyB.linearVelocity, this.bodyA.linearVelocity);
    return Vec2.dot(_perp, _axis);
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

  getMotorForce(invDt: Scalar): Scalar {
    return S.mul(this.motorImpulse, invDt);
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    const axial = this.motorImpulse + this.lowerImpulse - this.upperImpulse + this.springImpulse;
    out.set(
      S.mul(S.mulAdd(this.perp.x, this.impulse.x, S.mul(this.axis.x, axial)), invDt),
      S.mul(S.mulAdd(this.perp.y, this.impulse.x, S.mul(this.axis.y, axial)), invDt),
    );
    return out;
  }

  getReactionTorque(invDt: Scalar): Scalar {
    return S.mul(this.impulse.y, invDt);
  }

  getAnchorA(out: Vec2): Vec2 {
    return this.bodyA.getWorldPoint(out, this.localAnchorA);
  }

  getAnchorB(out: Vec2): Vec2 {
    return this.bodyB.getWorldPoint(out, this.localAnchorB);
  }

  saveState(out: number[]): void {
    out.push(
      this.impulse.x as number,
      this.impulse.y as number,
      this.motorImpulse as number,
      this.lowerImpulse as number,
      this.upperImpulse as number,
      this.springImpulse as number,
    );
  }

  loadState(data: number[], offset: number): number {
    this.impulse.set(data[offset]!, data[offset + 1]!);
    this.motorImpulse = data[offset + 2]!;
    this.lowerImpulse = data[offset + 3]!;
    this.upperImpulse = data[offset + 4]!;
    this.springImpulse = data[offset + 5]!;
    return offset + 6;
  }
}
