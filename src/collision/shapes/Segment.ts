/**
 * @module collision/shapes/Segment
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Transform } from './../../math/Transform.js';
import { AABB } from './../AABB.js';
import { ShapeType } from './../Shape.js';
import type { MassData, RayCastInput, RayCastOutput, Shape } from './../Shape.js';

const _p1 = Vec2.zero();
const _p2 = Vec2.zero();

/**
 * A zero-thickness line segment — the building block for static level
 * geometry (ground, walls, ramps).
 *
 * Segments have **no mass**; attaching one to a dynamic body without also
 * attaching a solid shape gives an infinite-inertia body. Use
 * {@link ChainShape} to build long, tunnel-free contours out of segments with
 * automatic ghost-vertex handling.
 */
export class Segment implements Shape {
  readonly type = ShapeType.Segment;
  readonly radius: Scalar = S.ZERO;
  readonly p1: Vec2;
  readonly p2: Vec2;
  readonly vertexCount = 2;

  /**
   * Optional neighbour vertices. When set, the collision code suppresses
   * impulses from the "inner" side of the edge, eliminating the classic
   * ghost-collision jolt when a box slides across a tiled floor.
   */
  ghost0: Vec2 | null = null;
  ghost1: Vec2 | null = null;

  constructor(p1: Vec2, p2: Vec2) {
    this.p1 = p1.clone();
    this.p2 = p2.clone();
  }

  static of(x1: number, y1: number, x2: number, y2: number): Segment {
    return new Segment(Vec2.of(x1, y1), Vec2.of(x2, y2));
  }

  /** Attach ghost vertices for one-sided/smooth collision. */
  setGhosts(g0: Vec2 | null, g1: Vec2 | null): this {
    this.ghost0 = g0 ? g0.clone() : null;
    this.ghost1 = g1 ? g1.clone() : null;
    return this;
  }

  computeAABB(out: AABB, xf: Transform): AABB {
    Transform.apply(_p1, xf, this.p1);
    Transform.apply(_p2, xf, this.p2);
    out.lower.set(S.min(_p1.x, _p2.x), S.min(_p1.y, _p2.y));
    out.upper.set(S.max(_p1.x, _p2.x), S.max(_p1.y, _p2.y));
    // A degenerate box breaks the broad-phase tree; give it a hair of volume.
    out.expand(S.fromFloat(0.01));
    return out;
  }

  /** Segments are massless. */
  computeMass(out: MassData, _density: Scalar): MassData {
    out.mass = S.ZERO;
    out.center.set(S.half(this.p1.x + this.p2.x), S.half(this.p1.y + this.p2.y));
    out.inertia = S.ZERO;
    return out;
  }

  /** A zero-area shape contains no points. */
  testPoint(_xf: Transform, _p: Vec2): boolean {
    return false;
  }

  /** Standard segment/segment intersection, solved in local space. */
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean {
    out.hit = false;
    Transform.applyT(_p1, xf, input.p1);
    Transform.applyT(_p2, xf, input.p2);

    const dx = _p2.x - _p1.x;
    const dy = _p2.y - _p1.y;

    const ex = this.p2.x - this.p1.x;
    const ey = this.p2.y - this.p1.y;
    // normal of the edge (right-hand perpendicular), unnormalised for now
    let nx = ey;
    let ny = -ex;
    const nl = S.sqrt(S.mulAdd(nx, nx, S.mul(ny, ny)));
    if (nl < S.EPSILON) return false;
    const inl = S.inv(nl);
    nx = S.mul(nx, inl);
    ny = S.mul(ny, inl);

    const num = S.mulAdd(nx, this.p1.x - _p1.x, S.mul(ny, this.p1.y - _p1.y));
    const den = S.mulAdd(nx, dx, S.mul(ny, dy));
    if (den === S.ZERO) return false;

    const t = S.div(num, den);
    if (t < S.ZERO || t > input.maxFraction) return false;

    const qx = S.mulAdd(dx, t, _p1.x);
    const qy = S.mulAdd(dy, t, _p1.y);

    const ee = S.mulAdd(ex, ex, S.mul(ey, ey));
    if (ee < S.EPSILON_SQ) return false;
    const s = S.div(S.mulAdd(qx - this.p1.x, ex, S.mul(qy - this.p1.y, ey)), ee);
    if (s < S.ZERO || s > S.ONE) return false;

    out.fraction = t;
    out.point.set(
      S.mulAdd(input.p2.x - input.p1.x, t, input.p1.x),
      S.mulAdd(input.p2.y - input.p1.y, t, input.p1.y),
    );
    // face the ray
    const sgn = den > S.ZERO ? S.NEG_ONE : S.ONE;
    out.normal.set(
      S.mul(sgn, S.mulAdd(xf.q.c, nx, -S.mul(xf.q.s, ny))),
      S.mul(sgn, S.mulAdd(xf.q.s, nx, S.mul(xf.q.c, ny))),
    );
    out.hit = true;
    return true;
  }

  supportIndex(d: Vec2): number {
    const d1 = S.mulAdd(this.p1.x, d.x, S.mul(this.p1.y, d.y));
    const d2 = S.mulAdd(this.p2.x, d.x, S.mul(this.p2.y, d.y));
    return d2 > d1 ? 1 : 0;
  }

  getVertex(i: number): Vec2 {
    return i === 0 ? this.p1 : this.p2;
  }

  clone(): Segment {
    const s = new Segment(this.p1, this.p2);
    s.ghost0 = this.ghost0 ? this.ghost0.clone() : null;
    s.ghost1 = this.ghost1 ? this.ghost1.clone() : null;
    return s;
  }
}

/**
 * Helper that turns a polyline into a list of {@link Segment}s with ghost
 * vertices wired up, so a body sliding along it never catches on a seam.
 *
 * ```ts
 * const ground = ChainShape.fromPoints([Vec2.of(-50,0), Vec2.of(0,-2), Vec2.of(50,0)]);
 * ground.forEach(s => body.addFixture({ shape: s }));
 * ```
 */
export const ChainShape = {
  /**
   * @param points  the polyline vertices, in order
   * @param loop    close the chain back to the first point
   */
  fromPoints(points: Vec2[], loop = false): Segment[] {
    const n = points.length;
    if (n < 2) return [];
    const out: Segment[] = [];
    const count = loop ? n : n - 1;
    for (let i = 0; i < count; i++) {
      const a = points[i]!;
      const b = points[(i + 1) % n]!;
      const seg = new Segment(a, b);
      const prev = loop ? points[(i - 1 + n) % n]! : i > 0 ? points[i - 1]! : null;
      const next = loop ? points[(i + 2) % n]! : i + 2 < n ? points[i + 2]! : null;
      seg.setGhosts(prev, next);
      out.push(seg);
    }
    return out;
  },
};
