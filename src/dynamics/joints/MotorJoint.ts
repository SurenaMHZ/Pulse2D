/**
 * @module dynamics/joints/MotorJoint
 *
 * Drives body B to a target offset and angle relative to body A, with capped
 * force and torque.
 *
 * This is the joint to use for **kinematic characters and moving platforms
 * that must still respect collisions**: instead of teleporting the body (which
 * lets it pass through walls), you give it a target and the solver pushes it
 * there as hard as it is allowed to, stopping naturally against obstacles.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Rot } from './../../math/Rot.js';
import { Mat22 } from './../../math/Mat22.js';
import { Joint, JointType } from './Joint.js';
import type { JointDefBase } from './Joint.js';
import type { Solver, StepContext } from './../Solver.js';

export interface MotorJointDef extends JointDefBase {
  /** Target position of B relative to A, in A's frame. */
  linearOffset?: { x: number; y: number };
  /** Target angle of B relative to A, radians. */
  angularOffset?: number;
  /** Maximum force, N. Default `1000`. */
  maxForce?: number;
  /** Maximum torque, N·m. Default `1000`. */
  maxTorque?: number;
  /**
   * Fraction of the remaining error corrected per step, `0..1`.
   * `1` snaps immediately (subject to the force caps), lower is smoother.
   * Default `0.3`.
   */
  correctionFactor?: number;
}

const _cdot = Vec2.zero();
const _rhs = Vec2.zero();
const _imp = Vec2.zero();
const _rA = Vec2.zero();
const _rB = Vec2.zero();

export class MotorJoint extends Joint {
  readonly type = JointType.Motor;

  readonly linearOffset = Vec2.zero();
  angularOffset: Scalar;
  maxForce: Scalar;
  maxTorque: Scalar;
  correctionFactor: Scalar;

  private readonly linearImpulse = Vec2.zero();
  private angularImpulse: Scalar = S.ZERO;

  private readonly K = new Mat22();
  private axialMass: Scalar = S.ZERO;
  /** Positional error captured at prepare time. */
  private readonly linearError = Vec2.zero();
  private angularError: Scalar = S.ZERO;

  constructor(id: number, def: MotorJointDef) {
    super(id, def);
    if (def.linearOffset) {
      this.linearOffset.set(S.fromFloat(def.linearOffset.x), S.fromFloat(def.linearOffset.y));
    }
    this.angularOffset = S.fromFloat(def.angularOffset ?? 0);
    this.maxForce = S.fromFloat(def.maxForce ?? 1000);
    this.maxTorque = S.fromFloat(def.maxTorque ?? 1000);
    this.correctionFactor = S.clamp(S.fromFloat(def.correctionFactor ?? 0.3), S.ZERO, S.ONE);
  }

  /** Move the target offset. */
  setLinearOffset(x: number, y: number): void {
    this.linearOffset.set(S.fromFloat(x), S.fromFloat(y));
    this.wake();
  }

  /** Move the target angle. */
  setAngularOffset(angle: number): void {
    this.angularOffset = S.fromFloat(angle);
    this.wake();
  }

  prepare(_ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);

    // Anchors are the centres of mass.
    this.rA.setZero();
    this.rB.setZero();
    _rA.setZero();
    _rB.setZero();

    const qA = sa ? sa.q : this.bodyA.transform.q;
    const qB = sb ? sb.q : this.bodyB.transform.q;

    // Where B's centre should be, in world space.
    Rot.rotate(_rA, qA, this.linearOffset);
    this.linearError.set(
      this.deltaCenter.x - _rA.x,
      this.deltaCenter.y - _rA.y,
    );
    this.angularError = Rot.relativeAngle(qA, qB) - this.angularOffset;

    const mA = this.invMassA;
    const mB = this.invMassB;
    const iA = this.invIA;
    const iB = this.invIB;
    this.K.set(mA + mB, S.ZERO, S.ZERO, mA + mB);
    const k = iA + iB;
    this.axialMass = k > S.ZERO ? S.inv(k) : S.ZERO;
  }

  warmStart(solver: Solver): void {
    this.applyImpulse(solver, this.linearImpulse.x, this.linearImpulse.y);
    this.applyAngularImpulse(solver, this.angularImpulse);
  }

  solve(ctx: StepContext, solver: Solver, _useBias: boolean): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);

    /* ---- angular ---- */
    {
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;
      const c = S.mul(S.mul(this.angularError, this.correctionFactor), ctx.invH);
      let impulse = -S.mul(this.axialMass, wB - wA + c);
      const old = this.angularImpulse;
      const maxImpulse = S.mul(this.maxTorque, ctx.h);
      this.angularImpulse = S.clamp(old + impulse, -maxImpulse, maxImpulse);
      impulse = this.angularImpulse - old;
      this.applyAngularImpulse(solver, impulse);
    }

    /* ---- linear ---- */
    {
      const vA = a ? a.v : Vec2.zero();
      const vB = b ? b.v : Vec2.zero();
      const dpA = a ? a.dp : Vec2.zero();
      const dpB = b ? b.dp : Vec2.zero();

      const ex = this.linearError.x + dpB.x - dpA.x;
      const ey = this.linearError.y + dpB.y - dpA.y;

      _cdot.set(vB.x - vA.x, vB.y - vA.y);
      _rhs.set(
        _cdot.x + S.mul(S.mul(ex, this.correctionFactor), ctx.invH),
        _cdot.y + S.mul(S.mul(ey, this.correctionFactor), ctx.invH),
      );
      this.K.solve(_imp, _rhs);

      const oldX = this.linearImpulse.x;
      const oldY = this.linearImpulse.y;
      this.linearImpulse.x -= _imp.x;
      this.linearImpulse.y -= _imp.y;

      const maxImpulse = S.mul(this.maxForce, ctx.h);
      const lenSq = this.linearImpulse.lengthSq();
      if (lenSq > S.mul(maxImpulse, maxImpulse)) {
        this.linearImpulse.scale(S.div(maxImpulse, S.sqrt(lenSq)));
      }
      this.applyImpulse(solver, this.linearImpulse.x - oldX, this.linearImpulse.y - oldY);
    }
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    return Vec2.scaleTo(out, this.linearImpulse, invDt);
  }

  getReactionTorque(invDt: Scalar): Scalar {
    return S.mul(this.angularImpulse, invDt);
  }

  getAnchorA(out: Vec2): Vec2 {
    return out.copyFrom(this.bodyA.worldCenter);
  }

  getAnchorB(out: Vec2): Vec2 {
    return out.copyFrom(this.bodyB.worldCenter);
  }

  saveState(out: number[]): void {
    out.push(
      this.linearImpulse.x as number,
      this.linearImpulse.y as number,
      this.angularImpulse as number,
    );
  }

  loadState(data: number[], offset: number): number {
    this.linearImpulse.set(data[offset]!, data[offset + 1]!);
    this.angularImpulse = data[offset + 2]!;
    return offset + 3;
  }
}
