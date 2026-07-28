/**
 * @module collision/ConvexProxy
 *
 * A uniform view of every convex primitive as a **rounded polygon**:
 * a vertex list, an outward normal per edge and a skin radius.
 *
 * | shape     | proxy                                   |
 * |-----------|-----------------------------------------|
 * | `Polygon` | its own vertices/normals, radius 0 or r |
 * | `Capsule` | 2 vertices, 2 opposite normals, radius r|
 * | `Segment` | 2 vertices, 2 opposite normals, radius 0|
 * | `Circle`  | 1 vertex, no normals, radius r          |
 *
 * Collapsing four shape types into one representation means the narrow phase
 * needs a single well-tested SAT routine instead of six bespoke ones — less
 * code, fewer edge cases, and one place to audit for determinism.
 *
 * Proxies are **cached on the shape instance** the first time they are needed
 * and never rebuilt, so the steady-state step allocates nothing.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { ShapeType } from './Shape.js';
import type { Shape } from './Shape.js';
import type { Polygon } from './shapes/Polygon.js';
import type { Capsule } from './shapes/Capsule.js';
import type { Segment } from './shapes/Segment.js';
import type { Circle } from './shapes/Circle.js';

export interface ConvexProxy {
  /** Local-space vertices. */
  readonly vertices: Vec2[];
  /** Outward unit normal of edge `i` (empty for a circle). */
  readonly normals: Vec2[];
  /** Number of vertices. */
  readonly count: number;
  /** Skin radius. */
  readonly radius: Scalar;
  /** Local centroid, used to orient degenerate normals. */
  readonly centroid: Vec2;
}

/** Hidden cache slot on the shape object. */
const CACHE = Symbol('pulse2d.proxy');

interface Cacheable {
  [CACHE]?: ConvexProxy;
}

/**
 * Get (and memoise) the convex proxy for a shape.
 * Safe to call every step — after the first call it is a property read.
 */
export function getProxy(shape: Shape): ConvexProxy {
  const cached = (shape as Cacheable)[CACHE];
  if (cached !== undefined) return cached;
  const proxy = buildProxy(shape);
  (shape as Cacheable)[CACHE] = proxy;
  return proxy;
}

function buildProxy(shape: Shape): ConvexProxy {
  switch (shape.type) {
    case ShapeType.Polygon: {
      const p = shape as Polygon;
      return {
        vertices: p.vertices,
        normals: p.normals,
        count: p.vertexCount,
        radius: p.radius,
        centroid: p.centroid,
      };
    }
    case ShapeType.Capsule: {
      const c = shape as Capsule;
      return segmentProxy(c.p1, c.p2, c.radius);
    }
    case ShapeType.Segment: {
      const s = shape as Segment;
      return segmentProxy(s.p1, s.p2, S.ZERO);
    }
    default: {
      const c = shape as Circle;
      return {
        vertices: [c.center],
        normals: [],
        count: 1,
        radius: c.radius,
        centroid: c.center,
      };
    }
  }
}

/** Two vertices, two opposing normals — a degenerate two-sided polygon. */
function segmentProxy(p1: Vec2, p2: Vec2, radius: Scalar): ConvexProxy {
  const n = new Vec2(p2.y - p1.y, -(p2.x - p1.x));
  if (n.normalize() === S.ZERO) n.set(S.ZERO, S.ONE); // zero-length guard
  return {
    vertices: [p1, p2],
    normals: [n, new Vec2(-n.x, -n.y)],
    count: 2,
    radius,
    centroid: new Vec2(S.half(p1.x + p2.x), S.half(p1.y + p2.y)),
  };
}
