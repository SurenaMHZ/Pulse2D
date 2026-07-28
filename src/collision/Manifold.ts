/**
 * @module collision/Manifold
 *
 * Contact manifold: the small set of points describing how two shapes touch.
 *
 * A 2D manifold never needs more than **two** points — that is enough to
 * represent both a vertex touching a face and two faces lying flat against
 * each other, which is why 2D solvers can be so much cheaper than 3D ones.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';

/**
 * Stable identifier for a manifold point.
 *
 * Encodes which feature of each shape produced the point. The solver uses it
 * to match this frame's points with last frame's so accumulated impulses can
 * be **warm-started**, which is what makes stacks converge in a few
 * iterations instead of hundreds.
 */
export type ContactID = number;

/** Pack four feature indices into one integer key. */
export function makeID(a: number, b: number, typeA: number, typeB: number): ContactID {
  return ((a & 0xff) | ((b & 0xff) << 8) | ((typeA & 0xff) << 16) | ((typeB & 0xff) << 24)) >>> 0;
}

/** A single point of contact. */
export class ManifoldPoint {
  /** Contact point in world space (midway between the two surfaces). */
  readonly point: Vec2 = Vec2.zero();
  /** Anchor relative to body A's centre of mass — cached by the solver. */
  readonly anchorA: Vec2 = Vec2.zero();
  /** Anchor relative to body B's centre of mass. */
  readonly anchorB: Vec2 = Vec2.zero();
  /** Penetration depth; negative means the shapes are separated. */
  separation: Scalar = S.ZERO;
  /** Accumulated normal impulse — persisted across steps for warm starting. */
  normalImpulse: Scalar = S.ZERO;
  /** Accumulated tangent (friction) impulse. */
  tangentImpulse: Scalar = S.ZERO;
  /** Impulse applied by the relax/restitution pass. */
  maxNormalImpulse: Scalar = S.ZERO;
  /** Relative normal velocity captured before the solve, for restitution. */
  relativeVelocity: Scalar = S.ZERO;
  /** Feature id used to match points between steps. */
  id: ContactID = 0;
  /** `false` on the first step this point existed. */
  persisted = false;

  reset(): void {
    this.point.setZero();
    this.anchorA.setZero();
    this.anchorB.setZero();
    this.separation = S.ZERO;
    this.normalImpulse = S.ZERO;
    this.tangentImpulse = S.ZERO;
    this.maxNormalImpulse = S.ZERO;
    this.relativeVelocity = S.ZERO;
    this.id = 0;
    this.persisted = false;
  }
}

/**
 * A contact manifold with up to two points sharing one normal.
 *
 * The normal always points **from A towards B**.
 */
export class Manifold {
  readonly points: [ManifoldPoint, ManifoldPoint] = [new ManifoldPoint(), new ManifoldPoint()];
  /** World-space contact normal, from A to B. */
  readonly normal: Vec2 = Vec2.zero();
  /** Number of valid entries in {@link points} (0, 1 or 2). */
  pointCount = 0;

  clear(): void {
    this.pointCount = 0;
    this.normal.setZero();
    this.points[0].reset();
    this.points[1].reset();
  }
}

/** Signature of every narrow-phase routine. */
export type CollideFn = (
  manifold: Manifold,
  shapeA: unknown,
  xfA: unknown,
  shapeB: unknown,
  xfB: unknown,
) => void;
