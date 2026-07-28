/**
 * @module dynamics/joints/Joint
 *
 * Base class for all constraints between two bodies.
 *
 * A joint follows the same three-phase contract as a contact:
 *
 * ```
 * prepare(ctx, solver)      once per step — cache anchors and effective mass
 * warmStart(solver)         re-apply last step's impulses
 * solve(ctx, solver, bias)  called every iteration
 * ```
 *
 * Deriving a custom joint means implementing those three methods; everything
 * else (registration, islands, snapshotting) is handled by the framework.
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Rot } from './../../math/Rot.js';
import type { Body } from './../Body.js';
import { BodyType } from './../Body.js';
import type { Solver, SolverBody, StepContext } from './../Solver.js';

/** Discriminant for serialisation and the debug view. */
export const enum JointType {
  Revolute = 0,
  Prismatic = 1,
  Distance = 2,
  Weld = 3,
  Mouse = 4,
  Motor = 5,
}

/** Fields shared by every joint definition. */
export interface JointDefBase {
  /** First body. */
  bodyA: Body;
  /** Second body. */
  bodyB: Body;
  /**
   * Let the two connected bodies collide with each other.
   * Default `false`, which is almost always what you want.
   */
  collideConnected?: boolean;
  /** Arbitrary payload. */
  userData?: unknown;
}

/** Spring parameters shared by the soft joint variants. */
export interface SpringDef {
  /** Enable the spring; when `false` the constraint is rigid. */
  enableSpring?: boolean;
  /** Oscillation frequency, Hz. */
  hertz?: number;
  /** Damping ratio: `<1` underdamped, `1` critical, `>1` overdamped. */
  dampingRatio?: number;
}

/** Motor parameters shared by the driven joints. */
export interface MotorDef {
  /** Enable the motor. */
  enableMotor?: boolean;
  /** Target speed (rad/s or m/s depending on the joint). */
  motorSpeed?: number;
  /** Maximum force/torque the motor may apply. */
  maxMotorForce?: number;
}

/** Limit parameters shared by the limited joints. */
export interface LimitDef {
  /** Enable the limits. */
  enableLimit?: boolean;
  /** Lower bound (rad or m). */
  lowerLimit?: number;
  /** Upper bound (rad or m). */
  upperLimit?: number;
}

const _tmp = Vec2.zero();

export abstract class Joint {
  /** Dense index in the world's joint table. */
  readonly id: number;
  abstract readonly type: JointType;

  readonly bodyA: Body;
  readonly bodyB: Body;
  readonly collideConnected: boolean;
  userData: unknown;

  /** @internal Solver index of A for this step, or -1. */
  protected indexA = -1;
  /** @internal Solver index of B for this step, or -1. */
  protected indexB = -1;

  /** @internal Anchor on A relative to A's centre of mass, world axes. */
  protected readonly rA = Vec2.zero();
  /** @internal Anchor on B relative to B's centre of mass, world axes. */
  protected readonly rB = Vec2.zero();
  /** @internal Vector from A's centre to B's centre at prepare time. */
  protected readonly deltaCenter = Vec2.zero();

  protected invMassA: Scalar = S.ZERO;
  protected invMassB: Scalar = S.ZERO;
  protected invIA: Scalar = S.ZERO;
  protected invIB: Scalar = S.ZERO;

  constructor(id: number, def: JointDefBase) {
    this.id = id;
    this.bodyA = def.bodyA;
    this.bodyB = def.bodyB;
    this.collideConnected = def.collideConnected ?? false;
    this.userData = def.userData;
  }

  /** `true` when at least one body is awake and simulated. */
  isActive(): boolean {
    const a = this.bodyA;
    const b = this.bodyB;
    if (!a.enabled || !b.enabled) return false;
    const aDyn = a.type !== BodyType.Static && a.awake;
    const bDyn = b.type !== BodyType.Static && b.awake;
    return aDyn || bDyn;
  }

  /** Wake both connected bodies. */
  wake(): void {
    this.bodyA.setAwake(true);
    this.bodyB.setAwake(true);
  }

  /**
   * @internal Cache solver indices and inverse masses.
   * Subclasses call `super.prepareCommon(...)` from their `prepare`.
   */
  protected prepareCommon(solver: Solver): void {
    this.indexA = this.bodyA.solverIndex;
    this.indexB = this.bodyB.solverIndex;
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);
    this.invMassA = sa ? sa.invMass : S.ZERO;
    this.invMassB = sb ? sb.invMass : S.ZERO;
    this.invIA = sa ? sa.invInertia : S.ZERO;
    this.invIB = sb ? sb.invInertia : S.ZERO;

    const cA = sa ? sa.c : this.bodyA.worldCenter;
    const cB = sb ? sb.c : this.bodyB.worldCenter;
    Vec2.subTo(this.deltaCenter, cB, cA);
  }

  /** Rotate a local anchor into world axes relative to the centre of mass. */
  protected computeAnchor(out: Vec2, body: Body, sb: SolverBody | null, localAnchor: Vec2): Vec2 {
    // localAnchor is relative to the body origin; shift to the centre of mass.
    _tmp.set(localAnchor.x - body.localCenter.x, localAnchor.y - body.localCenter.y);
    const q = sb ? sb.q : body.transform.q;
    return Rot.rotate(out, q, _tmp);
  }

  /**
   * Re-rotate both anchors from the bodies' **current** sub-step orientation.
   *
   * A joint's Jacobian depends on the world-space anchor arms `rA` and `rB`,
   * and those turn with the body. Computing them once in `prepare` is fine for
   * a contact — its anchors barely move — but a chain link or a bridge plank
   * can swing tens of degrees within a single step. The stale arm then points
   * the correction impulse in the wrong direction, so the constraint never
   * converges: the joint visibly stretches and the chain jitters, and adding
   * sub-steps does not help because the error is a wrong *direction*, not an
   * unconverged magnitude.
   *
   * Refreshing costs two rotations per joint per iteration and makes long
   * chains behave.
   */
  protected refreshAnchors(solver: Solver): void {
    const sa = solver.getBody(this.indexA);
    const sb = solver.getBody(this.indexB);
    this.computeAnchor(this.rA, this.bodyA, sa, this.localAnchorAOf());
    this.computeAnchor(this.rB, this.bodyB, sb, this.localAnchorBOf());
    // deltaCenter also drifts as the two centres move apart during the step.
    const cA = sa ? sa.c : this.bodyA.worldCenter;
    const cB = sb ? sb.c : this.bodyB.worldCenter;
    Vec2.subTo(this.deltaCenter, cB, cA);
  }

  /** Local anchor on A, for {@link refreshAnchors}. Overridden per joint. */
  protected localAnchorAOf(): Vec2 {
    return Vec2.zero();
  }

  /** Local anchor on B, for {@link refreshAnchors}. */
  protected localAnchorBOf(): Vec2 {
    return Vec2.zero();
  }

  /** Apply an impulse pair to both solver bodies. */
  protected applyImpulse(solver: Solver, px: Scalar, py: Scalar): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    if (a) {
      a.v.x -= S.mul(a.invMass, px);
      a.v.y -= S.mul(a.invMass, py);
      a.w -= S.mul(a.invInertia, S.mulAdd(this.rA.x, py, -S.mul(this.rA.y, px)));
    }
    if (b) {
      b.v.x += S.mul(b.invMass, px);
      b.v.y += S.mul(b.invMass, py);
      b.w += S.mul(b.invInertia, S.mulAdd(this.rB.x, py, -S.mul(this.rB.y, px)));
    }
  }

  /** Apply a pure angular impulse. */
  protected applyAngularImpulse(solver: Solver, impulse: Scalar): void {
    const a = solver.getBody(this.indexA);
    const b = solver.getBody(this.indexB);
    if (a) a.w -= S.mul(a.invInertia, impulse);
    if (b) b.w += S.mul(b.invInertia, impulse);
  }

  /** @internal Called once per step before the iterations. */
  abstract prepare(ctx: StepContext, solver: Solver): void;

  /** @internal Re-apply the accumulated impulses. */
  abstract warmStart(solver: Solver): void;

  /** @internal One solver iteration. */
  abstract solve(ctx: StepContext, solver: Solver, useBias: boolean): void;

  /** Reaction force on body B, N (divide the stored impulse by `dt`). */
  abstract getReactionForce(out: Vec2, invDt: Scalar): Vec2;

  /** Reaction torque on body B, N·m. */
  abstract getReactionTorque(invDt: Scalar): Scalar;

  /** World-space anchor on body A. */
  abstract getAnchorA(out: Vec2): Vec2;

  /** World-space anchor on body B. */
  abstract getAnchorB(out: Vec2): Vec2;

  /** @internal Serialise the accumulated impulses into a snapshot buffer. */
  abstract saveState(out: number[]): void;

  /** @internal Restore impulses from a snapshot buffer. */
  abstract loadState(data: number[], offset: number): number;
}
