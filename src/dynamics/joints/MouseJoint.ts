/**
 * @module dynamics/joints/MouseJoint
 *
 * Drags a body towards a moving world target with a soft spring.
 *
 * Despite the name this is not just for mice: it is the standard way to
 * implement any "pull an object towards a point" behaviour — tractor beams,
 * grappling hooks, magnet pickups, fish-on-a-line.
 *
 * `bodyA` is ignored (conventionally a static ground body); only `bodyB` is
 * driven.
 *
 * ⚠️ **Networking note.** Because this joint is normally fed by pointer input,
 * remember that the *target position* is part of your input stream and must be
 * transmitted and quantised like any other input, or peers will diverge.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Mat22 } from './../../math/Mat22.js';
import { Joint, JointType } from './Joint.js';
import type { JointDefBase } from './Joint.js';
import { makeSoft } from './../Solver.js';
import type { SoftConstraint, Solver, StepContext } from './../Solver.js';

export interface MouseJointDef extends JointDefBase {
  /** Initial world target. */
  target?: { x: number; y: number };
  /** Spring frequency, Hz. Default `5`. */
  hertz?: number;
  /** Damping ratio. Default `0.7`. */
  dampingRatio?: number;
  /** Maximum force the spring may apply, N. Default `1000`. */
  maxForce?: number;
}

const _cdot = Vec2.zero();
const _rhs = Vec2.zero();
const _imp = Vec2.zero();

export class MouseJoint extends Joint {
  readonly type = JointType.Mouse;

  /** World-space target the body is pulled towards. */
  readonly target = Vec2.zero();
  /** Attachment point in B's local frame. */
  readonly localAnchorB = Vec2.zero();

  hertz: Scalar;
  dampingRatio: Scalar;
  maxForce: Scalar;

  private readonly impulse = Vec2.zero();
  private readonly K = new Mat22();
  private readonly soft: SoftConstraint = { biasRate: S.ZERO, massScale: S.ONE, impulseScale: S.ZERO };
  /** Offset from the target to the anchor at prepare time. */
  private readonly deltaTarget = Vec2.zero();

  constructor(id: number, def: MouseJointDef) {
    super(id, def);
    if (def.target) this.target.set(S.fromFloat(def.target.x), S.fromFloat(def.target.y));
    else this.target.copyFrom(def.bodyB.transform.p);
    def.bodyB.getLocalPoint(this.localAnchorB, this.target);
    this.hertz = S.fromFloat(def.hertz ?? 5);
    this.dampingRatio = S.fromFloat(def.dampingRatio ?? 0.7);
    this.maxForce = S.fromFloat(def.maxForce ?? 1000);
  }

  /** Move the target. Wakes the body. */
  setTarget(x: number, y: number): void {
    const nx = S.fromFloat(x);
    const ny = S.fromFloat(y);
    if (nx !== this.target.x || ny !== this.target.y) {
      this.target.set(nx, ny);
      this.bodyB.setAwake(true);
    }
  }

  /** Move the target using backend scalars (for deterministic input replay). */
  setTargetScalar(x: Scalar, y: Scalar): void {
    if (x !== this.target.x || y !== this.target.y) {
      this.target.set(x, y);
      this.bodyB.setAwake(true);
    }
  }

  prepare(ctx: StepContext, solver: Solver): void {
    this.prepareCommon(solver);
    const sb = solver.getBody(this.indexB);
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorB);
    this.rA.setZero();

    const cB = sb ? sb.c : this.bodyB.worldCenter;
    // Current error: anchor position minus target.
    this.deltaTarget.set(cB.x + this.rB.x - this.target.x, cB.y + this.rB.y - this.target.y);

    const mB = this.invMassB;
    const iB = this.invIB;
    this.K.set(
      mB + S.mul(iB, S.mul(this.rB.y, this.rB.y)),
      -S.mul(iB, S.mul(this.rB.x, this.rB.y)),
      -S.mul(iB, S.mul(this.rB.x, this.rB.y)),
      mB + S.mul(iB, S.mul(this.rB.x, this.rB.x)),
    );

    makeSoft(this.hertz, this.dampingRatio, ctx.h, this.soft);
    if (!ctx.enableWarmStarting) this.impulse.setZero();
  }

  warmStart(solver: Solver): void {
    const b = solver.getBody(this.indexB);
    if (!b) return;
    b.v.x += S.mul(b.invMass, this.impulse.x);
    b.v.y += S.mul(b.invMass, this.impulse.y);
    b.w += S.mul(b.invInertia, Vec2.cross(this.rB, this.impulse));
  }

  solve(ctx: StepContext, solver: Solver, _useBias: boolean): void {
    const b = solver.getBody(this.indexB);
    if (!b) return;

    _cdot.set(b.v.x - S.mul(b.w, this.rB.y), b.v.y + S.mul(b.w, this.rB.x));

    // Positional error, updated with this step's motion.
    const cx = this.deltaTarget.x + b.dp.x;
    const cy = this.deltaTarget.y + b.dp.y;

    _rhs.set(
      S.mulAdd(this.soft.biasRate, cx, _cdot.x),
      S.mulAdd(this.soft.biasRate, cy, _cdot.y),
    );
    this.K.solve(_imp, _rhs);

    const ix = -S.mul(this.soft.massScale, _imp.x) - S.mul(this.soft.impulseScale, this.impulse.x);
    const iy = -S.mul(this.soft.massScale, _imp.y) - S.mul(this.soft.impulseScale, this.impulse.y);

    // Clamp the accumulated impulse to maxForce · h.
    const oldX = this.impulse.x;
    const oldY = this.impulse.y;
    this.impulse.x += ix;
    this.impulse.y += iy;
    const maxImpulse = S.mul(this.maxForce, ctx.h);
    const lenSq = this.impulse.lengthSq();
    if (lenSq > S.mul(maxImpulse, maxImpulse)) {
      const scale = S.div(maxImpulse, S.sqrt(lenSq));
      this.impulse.scale(scale);
    }
    const ax = this.impulse.x - oldX;
    const ay = this.impulse.y - oldY;

    b.v.x += S.mul(b.invMass, ax);
    b.v.y += S.mul(b.invMass, ay);
    b.w += S.mul(b.invInertia, S.mulAdd(this.rB.x, ay, -S.mul(this.rB.y, ax)));
  }

  getReactionForce(out: Vec2, invDt: Scalar): Vec2 {
    return Vec2.scaleTo(out, this.impulse, invDt);
  }

  getReactionTorque(_invDt: Scalar): Scalar {
    return S.ZERO;
  }

  getAnchorA(out: Vec2): Vec2 {
    return out.copyFrom(this.target);
  }

  getAnchorB(out: Vec2): Vec2 {
    return this.bodyB.getWorldPoint(out, this.localAnchorB);
  }

  saveState(out: number[]): void {
    out.push(this.impulse.x as number, this.impulse.y as number);
  }

  loadState(data: number[], offset: number): number {
    this.impulse.set(data[offset]!, data[offset + 1]!);
    return offset + 2;
  }
}
