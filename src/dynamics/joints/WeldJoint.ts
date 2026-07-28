/**
 * @module dynamics/joints/WeldJoint
 *
 * Rigidly (or softly) fuses two bodies — all three degrees of freedom are
 * constrained.
 *
 * A perfectly rigid weld is usually better expressed as one body with two
 * fixtures; the reason to use this joint is the **soft** mode, where the
 * linear and angular springs let you build breakable structures, wobbling
 * bridges and springy vehicle chassis.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Rot } from './../../math/Rot.js';
import { Mat22 } from './../../math/Mat22.js';
import { Joint, JointType } from './Joint.js';
import type { JointDefBase } from './Joint.js';
import { makeSoft } from './../Solver.js';
import type { SoftConstraint, Solver, StepContext } from './../Solver.js';
import { JOINT_HERTZ, JOINT_DAMPING_RATIO } from './../../util/settings.js';

export interface WeldJointDef extends JointDefBase {
  localAnchorA?: { x: number; y: number };
  localAnchorB?: { x: number; y: number };
  /** Locked relative angle. Defaults to the angle at creation. */
  referenceAngle?: number;
  /** Linear spring frequency, Hz. `0` = rigid. */
  linearHertz?: number;
  /** Linear damping ratio. */
  linearDampingRatio?: number;
  /** Angular spring frequency, Hz. `0` = rigid. */
  angularHertz?: number;
  /** Angular damping ratio. */
  angularDampingRatio?: number;
}

const _cdot = Vec2.zero();
const _rhs = Vec2.zero();
const _imp = Vec2.zero();

export class WeldJoint extends Joint {
  readonly type = JointType.Weld;

  readonly localAnchorA: Vec2;
  readonly localAnchorB: Vec2;
  referenceAngle: Scalar;

  linearHertz: Scalar;
  linearDampingRatio: Scalar;
  angularHertz: Scalar;
  angularDampingRatio: Scalar;

  private readonly linearImpulse = Vec2.zero();
  private angularImpulse: Scalar = S.ZERO;

  private readonly K = new Mat22();
  private axialMass: Scalar = S.ZERO;
  private readonly linearSoft: SoftConstraint = {
    biasRate: S.ZERO,
    massScale: S.ONE,
    impulseScale: S.ZERO,
  };
  private readonly angularSoft: SoftConstraint = {
    biasRate: S.ZERO,
    massScale: S.ONE,
    impulseScale: S.ZERO,
  };

  constructor(id: number, def: WeldJointDef) {
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
    this.linearHertz = S.fromFloat(def.linearHertz ?? 0);
    this.linearDampingRatio = S.fromFloat(def.linearDampingRatio ?? 1);
    this.angularHertz = S.fromFloat(def.angularHertz ?? 0);
    this.angularDampingRatio = S.fromFloat(def.angularDampingRatio ?? 1);
  }

  prepare(ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);

    this.computeAnchor(this.rA, this.bodyA, sa, this.localAnchorA);
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorB);

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

    // hertz = 0 means rigid, so fall back to the stiff default.
    makeSoft(
      this.linearHertz > S.ZERO ? this.linearHertz : JOINT_HERTZ,
      this.linearHertz > S.ZERO ? this.linearDampingRatio : JOINT_DAMPING_RATIO,
      ctx.h,
      this.linearSoft,
    );
    makeSoft(
      this.angularHertz > S.ZERO ? this.angularHertz : JOINT_HERTZ,
      this.angularHertz > S.ZERO ? this.angularDampingRatio : JOINT_DAMPING_RATIO,
      ctx.h,
      this.angularSoft,
    );

    if (!ctx.enableWarmStarting) {
      this.linearImpulse.setZero();
      this.angularImpulse = S.ZERO;
    }
  }

  warmStart(solver: Solver): void {
    this.applyImpulse(solver, this.linearImpulse.x, this.linearImpulse.y);
    this.applyAngularImpulse(solver, this.angularImpulse);
  }

  protected override localAnchorAOf(): Vec2 {
    return this.localAnchorA;
  }

  protected override localAnchorBOf(): Vec2 {
    return this.localAnchorB;
  }

  solve(_ctx: StepContext, solver: Solver, useBias: boolean): void {
    // Anchor arms turn with the bodies; see Joint#refreshAnchors.
    this.refreshAnchors(solver);
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);

    // K depends on the arms that just moved.
    {
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
    }

    /* ---- angular ---- */
    {
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;
      let bias = S.ZERO;
      let massScale = S.ONE;
      let impulseScale = S.ZERO;
      if (useBias || this.angularHertz > S.ZERO) {
        const qA = a ? a.q : this.bodyA.transform.q;
        const qB = b ? b.q : this.bodyB.transform.q;
        const c = Rot.relativeAngle(qA, qB) - this.referenceAngle;
        bias = S.mul(this.angularSoft.biasRate, c);
        massScale = this.angularSoft.massScale;
        impulseScale = this.angularSoft.impulseScale;
      }
      const cdot = wB - wA;
      const impulse =
        -S.mul(S.mul(this.axialMass, massScale), cdot + bias) -
        S.mul(impulseScale, this.angularImpulse);
      this.angularImpulse += impulse;
      this.applyAngularImpulse(solver, impulse);
    }

    /* ---- linear ---- */
    {
      const vA = a ? a.v : Vec2.zero();
      const vB = b ? b.v : Vec2.zero();
      const wA = a ? a.w : S.ZERO;
      const wB = b ? b.w : S.ZERO;

      _cdot.set(
        vB.x - S.mul(wB, this.rB.y) - vA.x + S.mul(wA, this.rA.y),
        vB.y + S.mul(wB, this.rB.x) - vA.y - S.mul(wA, this.rA.x),
      );

      let bx = S.ZERO;
      let by = S.ZERO;
      let massScale = S.ONE;
      let impulseScale = S.ZERO;
      if (useBias || this.linearHertz > S.ZERO) {
        // deltaCenter and the arms are live, so this is the true error.
        const cx = this.deltaCenter.x + this.rB.x - this.rA.x;
        const cy = this.deltaCenter.y + this.rB.y - this.rA.y;
        bx = S.mul(this.linearSoft.biasRate, cx);
        by = S.mul(this.linearSoft.biasRate, cy);
        massScale = this.linearSoft.massScale;
        impulseScale = this.linearSoft.impulseScale;
      }

      _rhs.set(_cdot.x + bx, _cdot.y + by);
      this.K.solve(_imp, _rhs);
      const ix = -S.mul(massScale, _imp.x) - S.mul(impulseScale, this.linearImpulse.x);
      const iy = -S.mul(massScale, _imp.y) - S.mul(impulseScale, this.linearImpulse.y);
      this.linearImpulse.x += ix;
      this.linearImpulse.y += iy;
      this.applyImpulse(solver, ix, iy);
    }
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    return Vec2.scaleTo(out, this.linearImpulse, invDt);
  }

  getReactionTorque(invDt: Scalar): Scalar {
    return S.mul(this.angularImpulse, invDt);
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
      this.angularImpulse as number,
    );
  }

  loadState(data: number[], offset: number): number {
    this.linearImpulse.set(data[offset]!, data[offset + 1]!);
    this.angularImpulse = data[offset + 2]!;
    return offset + 3;
  }
}
