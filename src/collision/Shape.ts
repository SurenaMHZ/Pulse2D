/**
 * @module collision/Shape
 *
 * Shape interface and mass properties.
 *
 * Pulse2D supports four primitives, all convex:
 *
 * | shape     | id | notes                                          |
 * |-----------|----|------------------------------------------------|
 * | `Circle`  | 0  | cheapest, perfectly smooth rolling             |
 * | `Capsule` | 1  | a segment with a radius; best for characters    |
 * | `Polygon` | 2  | up to 8 vertices, convex, optional skin radius |
 * | `Segment` | 3  | zero-thickness edge, static geometry only      |
 *
 * Concave geometry is expressed as several fixtures on one body.
 */

import type { Scalar } from './../math/scalar.js';
import type { Vec2 } from './../math/Vec2.js';
import type { Transform } from './../math/Transform.js';
import type { AABB } from './AABB.js';

/** Discriminant used for the O(1) collision dispatch table. */
export const enum ShapeType {
  Circle = 0,
  Capsule = 1,
  Polygon = 2,
  Segment = 3,
  /** Number of shape types — sizes the dispatch matrix. */
  Count = 4,
}

/** Mass, centroid and rotational inertia computed from a shape + density. */
export interface MassData {
  /** Mass in kilograms. */
  mass: Scalar;
  /** Local centroid. */
  center: Vec2;
  /** Rotational inertia about the **local origin** (not the centroid). */
  inertia: Scalar;
}

/** Result of a successful ray cast. */
export interface RayCastOutput {
  /** Surface normal at the hit point, pointing out of the shape. */
  normal: Vec2;
  /** Hit point in world space. */
  point: Vec2;
  /** Parametric distance along the ray, in `[0, maxFraction]`. */
  fraction: Scalar;
  /** `true` when the ray actually hit. */
  hit: boolean;
}

/** Input for a ray cast in world space. */
export interface RayCastInput {
  p1: Vec2;
  p2: Vec2;
  maxFraction: Scalar;
}

/**
 * Common interface for every collision primitive.
 *
 * Shapes are **immutable value objects**: they describe geometry in local
 * space only. A {@link Fixture} binds a shape to a body and gives it material
 * properties. The same shape instance may be shared by any number of
 * fixtures, which keeps memory flat when you spawn a thousand identical boxes.
 */
export interface Shape {
  /** Runtime discriminant. */
  readonly type: ShapeType;

  /**
   * Collision skin. Circles and capsules store their true radius here;
   * polygons usually use `0` but may carry a small radius for rounded corners.
   */
  readonly radius: Scalar;

  /** Compute the world-space AABB under `xf`, writing into `out`. */
  computeAABB(out: AABB, xf: Transform): AABB;

  /** Compute mass properties for a uniform `density`, writing into `out`. */
  computeMass(out: MassData, density: Scalar): MassData;

  /** `true` when the world-space point `p` is inside the shape. */
  testPoint(xf: Transform, p: Vec2): boolean;

  /** Cast a ray in world space; returns `false` on a miss. */
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean;

  /**
   * Support function for GJK/SAT: the index of the vertex furthest along
   * the **local** direction `d`. Circles always return `0`.
   */
  supportIndex(d: Vec2): number;

  /** Number of support vertices (1 for a circle, 2 for a capsule/segment). */
  readonly vertexCount: number;

  /** Local-space vertex `i` (the circle/capsule centre points). */
  getVertex(i: number): Vec2;

  /** Structural clone — deep enough that mutating one cannot affect the other. */
  clone(): Shape;
}
