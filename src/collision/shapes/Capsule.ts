/**
 * @module collision/shapes/Capsule
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
const _d = Vec2.zero();

/**
 * A capsule: the set of points within `radius` of the segment `p1 → p2`.
 *
 * The best character-controller primitive — it climbs steps and slides along
 * walls without the corner-catching a box suffers from, while remaining a
 * single convex shape (no compound needed).
 */
export class Capsule implements Shape {
  readonly type = ShapeType.Capsule;
  readonly radius: Scalar;
  /** First segment endpoint, local space. */
  readonly p1: Vec2;
  /** Second segment endpoint, local space. */
  readonly p2: Vec2;
  readonly vertexCount = 2;

  constructor(p1: Vec2, p2: Vec2, radius: Scalar) {
    this.p1 = p1.clone();
    this.p2 = p2.clone();
    this.radius = radius;
  }

  /** A vertical capsule of total height `height` centred on the origin. */
  static vertical(height: number, radius: number): Capsule {
    const h = Math.max(0, height * 0.5 - radius);
    return new Capsule(Vec2.of(0, -h), Vec2.of(0, h), S.fromFloat(radius));
  }

  /** A horizontal capsule of total width `width` centred on the origin. */
  static horizontal(width: number, radius: number): Capsule {
    const h = Math.max(0, width * 0.5 - radius);
    return new Capsule(Vec2.of(-h, 0), Vec2.of(h, 0), S.fromFloat(radius));
  }

  /** From plain floats. */
  static of(x1: number, y1: number, x2: number, y2: number, radius: number): Capsule {
    return new Capsule(Vec2.of(x1, y1), Vec2.of(x2, y2), S.fromFloat(radius));
  }

  computeAABB(out: AABB, xf: Transform): AABB {
    Transform.apply(_p1, xf, this.p1);
    Transform.apply(_p2, xf, this.p2);
    out.lower.set(S.min(_p1.x, _p2.x) - this.radius, S.min(_p1.y, _p2.y) - this.radius);
    out.upper.set(S.max(_p1.x, _p2.x) + this.radius, S.max(_p1.y, _p2.y) + this.radius);
    return out;
  }

  /**
   * Exact capsule mass: a rectangle of `length × 2r` plus two half discs.
   * The inertia of each part is computed about its own centroid and shifted
   * to the shape centre, then to the body origin.
   */
  computeMass(out: MassData, density: Scalar): MassData {
    const r = this.radius;
    const rr = S.mul(r, r);
    _d.set(this.p2.x - this.p1.x, this.p2.y - this.p1.y);
    const length = _d.length();
    const ll = S.mul(length, length);

    const boxMass = S.mul(density, S.mul(S.mulInt(r, 2), length));
    const circleMass = S.mul(density, S.mul(S.PI, rr));
    out.mass = boxMass + circleMass;

    // Shape centre = segment midpoint.
    const cx = S.half(this.p1.x + this.p2.x);
    const cy = S.half(this.p1.y + this.p2.y);
    out.center.set(cx, cy);

    // Rectangle about its centre: m(w² + h²)/12 with w = 2r, h = length.
    const boxI = S.mul(boxMass, S.div(S.mulInt(rr, 4) + ll, S.fromInt(12)));

    // Two half discs, each offset by length/2 from the centre.
    const h = S.half(length);
    const lc = S.mul(S.fromFloat(4 / (3 * Math.PI)), r); // half-disc centroid offset
    const circleI =
      S.mul(circleMass, S.half(rr) + S.mul(h, h) + S.mulInt(S.mul(h, lc), 2));

    const localI = boxI + circleI;
    // Parallel-axis shift from the shape centre to the body origin.
    out.inertia = localI + S.mul(out.mass, S.mulAdd(cx, cx, S.mul(cy, cy)));
    return out;
  }

  testPoint(xf: Transform, p: Vec2): boolean {
    Transform.applyT(_d, xf, p); // local point
    const dsq = Capsule.distanceSqToSegment(_d, this.p1, this.p2);
    return dsq <= S.mul(this.radius, this.radius);
  }

  /** Squared distance from a point to a segment, clamped at both ends. */
  static distanceSqToSegment(p: Vec2, a: Vec2, b: Vec2): Scalar {
    const ex = b.x - a.x;
    const ey = b.y - a.y;
    const px = p.x - a.x;
    const py = p.y - a.y;
    const ee = S.mulAdd(ex, ex, S.mul(ey, ey));
    if (ee < S.EPSILON_SQ) return S.mulAdd(px, px, S.mul(py, py));
    let t = S.div(S.mulAdd(px, ex, S.mul(py, ey)), ee);
    t = S.clamp(t, S.ZERO, S.ONE);
    const qx = px - S.mul(ex, t);
    const qy = py - S.mul(ey, t);
    return S.mulAdd(qx, qx, S.mul(qy, qy));
  }

  /**
   * Ray cast against the capsule.
   *
   * The cylinder body is solved analytically; the two end caps fall out of the
   * same quadratic with the endpoint substituted for the axis projection, so
   * a single closed-form pass covers all three regions.
   */
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean {
    out.hit = false;
    // Work in local space.
    Transform.applyT(_p1, xf, input.p1);
    Transform.applyT(_p2, xf, input.p2);

    const dx = _p2.x - _p1.x;
    const dy = _p2.y - _p1.y;
    const dd = S.mulAdd(dx, dx, S.mul(dy, dy));
    if (dd < S.EPSILON_SQ) return false;

    let bestT = input.maxFraction;
    let nx = S.ZERO;
    let ny = S.ZERO;
    let found = false;

    // --- infinite cylinder around the axis ---
    const ax = this.p2.x - this.p1.x;
    const ay = this.p2.y - this.p1.y;
    const aa = S.mulAdd(ax, ax, S.mul(ay, ay));
    if (aa > S.EPSILON_SQ) {
      // Perpendicular distance formulation: |(P - p1) × â| = r
      const invLen = S.inv(S.sqrt(aa));
      const ux = S.mul(ax, invLen);
      const uy = S.mul(ay, invLen);
      // component of the ray perpendicular to the axis
      const ocx = _p1.x - this.p1.x;
      const ocy = _p1.y - this.p1.y;
      const perpO = S.mulAdd(ocx, uy, -S.mul(ocy, ux));
      const perpD = S.mulAdd(dx, uy, -S.mul(dy, ux));
      const A = S.mul(perpD, perpD);
      const B = S.mulInt(S.mul(perpO, perpD), 2);
      const C = S.mul(perpO, perpO) - S.mul(this.radius, this.radius);
      if (A > S.EPSILON) {
        const disc = S.mulAdd(B, B, -S.mulInt(S.mul(A, C), 4));
        if (disc >= S.ZERO) {
          const t = S.div(-B - S.sqrt(disc), S.mulInt(A, 2));
          if (t >= S.ZERO && t <= bestT) {
            const hx = S.mulAdd(dx, t, _p1.x) - this.p1.x;
            const hy = S.mulAdd(dy, t, _p1.y) - this.p1.y;
            const along = S.mulAdd(hx, ux, S.mul(hy, uy));
            if (along >= S.ZERO && S.mul(along, along) <= aa) {
              bestT = t;
              const s = S.sign(perpO);
              nx = S.mul(uy, s);
              ny = -S.mul(ux, s);
              found = true;
            }
          }
        }
      }
    }

    // --- the two spherical caps ---
    for (let i = 0; i < 2; i++) {
      const c = i === 0 ? this.p1 : this.p2;
      const sx = _p1.x - c.x;
      const sy = _p1.y - c.y;
      const b = S.mulAdd(sx, sx, S.mul(sy, sy)) - S.mul(this.radius, this.radius);
      const cc = S.mulAdd(sx, dx, S.mul(sy, dy));
      const sigma = S.mulAdd(cc, cc, -S.mul(dd, b));
      if (sigma < S.ZERO) continue;
      const t = S.div(-(cc + S.sqrt(sigma)), dd);
      if (t < S.ZERO || t > bestT) continue;
      bestT = t;
      nx = S.mulAdd(dx, t, sx);
      ny = S.mulAdd(dy, t, sy);
      const l = S.sqrt(S.mulAdd(nx, nx, S.mul(ny, ny)));
      if (l > S.EPSILON) {
        const il = S.inv(l);
        nx = S.mul(nx, il);
        ny = S.mul(ny, il);
      }
      found = true;
    }

    if (!found) return false;
    out.fraction = bestT;
    out.point.set(
      S.mulAdd(input.p2.x - input.p1.x, bestT, input.p1.x),
      S.mulAdd(input.p2.y - input.p1.y, bestT, input.p1.y),
    );
    // rotate the local normal into world space
    out.normal.set(
      S.mulAdd(xf.q.c, nx, -S.mul(xf.q.s, ny)),
      S.mulAdd(xf.q.s, nx, S.mul(xf.q.c, ny)),
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

  clone(): Capsule {
    return new Capsule(this.p1, this.p2, this.radius);
  }
}
