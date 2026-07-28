/**
 * @module collision/shapes/Polygon
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Rot } from './../../math/Rot.js';
import { Transform } from './../../math/Transform.js';
import { AABB } from './../AABB.js';
import { ShapeType } from './../Shape.js';
import type { MassData, RayCastInput, RayCastOutput, Shape } from './../Shape.js';

/** Hard cap on vertices — keeps SAT loops short and cache friendly. */
export const MAX_POLYGON_VERTICES = 8;

const _tmp = Vec2.zero();
const _tmp2 = Vec2.zero();

/**
 * A convex polygon with counter-clockwise winding.
 *
 * The constructor runs a deterministic **gift-wrap convex hull** over the
 * input points, so you may pass an unordered or slightly concave point cloud
 * and still get a valid shape. Duplicate and nearly-collinear points are
 * removed. Both the vertices and the outward edge normals are cached, which
 * is what makes the SAT collision path branch-free.
 *
 * An optional `radius` rounds the corners (a "polygon skin"), useful for
 * fast-moving objects that must not tunnel on sharp edges.
 */
export class Polygon implements Shape {
  readonly type = ShapeType.Polygon;
  readonly radius: Scalar;
  /** Hull vertices, CCW. */
  readonly vertices: Vec2[];
  /** Outward unit normal of edge `i` (from vertex `i` to `i+1`). */
  readonly normals: Vec2[];
  /** Area centroid in local space. */
  readonly centroid: Vec2;
  readonly vertexCount: number;

  /**
   * @param points local-space points; the convex hull is taken
   * @param radius optional corner rounding
   */
  constructor(points: Vec2[], radius: Scalar = S.ZERO) {
    const hull = Polygon.computeHull(points);
    if (hull.length < 3) {
      throw new Error(
        `Polygon needs at least 3 non-collinear points, got ${hull.length} after hulling`,
      );
    }
    this.vertices = hull;
    this.vertexCount = hull.length;
    this.radius = radius;
    this.normals = new Array<Vec2>(this.vertexCount);
    for (let i = 0; i < this.vertexCount; i++) {
      const a = hull[i]!;
      const b = hull[(i + 1) % this.vertexCount]!;
      const n = new Vec2(b.y - a.y, -(b.x - a.x)); // right-hand perpendicular
      n.normalize();
      this.normals[i] = n;
    }
    this.centroid = Polygon.computeCentroid(hull);
  }

  /* ------------------------- factories ------------------------- */

  /**
   * An axis-aligned box centred on the local origin.
   * @param hw half width
   * @param hh half height
   */
  static box(hw: number, hh: number, radius = 0): Polygon {
    return new Polygon(
      [Vec2.of(-hw, -hh), Vec2.of(hw, -hh), Vec2.of(hw, hh), Vec2.of(-hw, hh)],
      S.fromFloat(radius),
    );
  }

  /** A box offset by `(cx, cy)` and rotated by `angle` radians. */
  static offsetBox(hw: number, hh: number, cx: number, cy: number, angle = 0): Polygon {
    const xf = new Transform(Vec2.of(cx, cy), Rot.of(angle));
    const pts = [Vec2.of(-hw, -hh), Vec2.of(hw, -hh), Vec2.of(hw, hh), Vec2.of(-hw, hh)];
    for (const p of pts) Transform.apply(p, xf, p);
    return new Polygon(pts);
  }

  /**
   * A regular n-gon of circumradius `r`.
   * Uses the deterministic trig kernels, so the vertices are identical on
   * every machine.
   */
  static regular(sides: number, r: number, angleOffset = 0): Polygon {
    const n = Math.max(3, Math.min(MAX_POLYGON_VERTICES, sides | 0));
    const pts: Vec2[] = [];
    for (let i = 0; i < n; i++) {
      const a = angleOffset + (2 * Math.PI * i) / n;
      const rot = Rot.of(a);
      pts.push(new Vec2(S.mul(S.fromFloat(r), rot.c), S.mul(S.fromFloat(r), rot.s)));
    }
    return new Polygon(pts);
  }

  /* --------------------------- hull ---------------------------- */

  /**
   * Deterministic convex hull (gift wrapping / Jarvis march).
   *
   * Chosen over quickhull because its comparison order is fully specified and
   * it never sorts by a floating point key — two different machines walk the
   * points in exactly the same sequence.
   */
  static computeHull(points: Vec2[]): Vec2[] {
    const n = Math.min(points.length, MAX_POLYGON_VERTICES * 2);
    if (n < 3) return points.map((p) => p.clone());

    // Merge points that are closer than the linear slop.
    const ps: Vec2[] = [];
    const tolSq = S.fromFloat(0.25 * 0.005 * 0.005);
    outer: for (let i = 0; i < n; i++) {
      const p = points[i]!;
      for (let j = 0; j < ps.length; j++) {
        if (Vec2.distanceSq(p, ps[j]!) < tolSq) continue outer;
      }
      ps.push(p.clone());
    }
    if (ps.length < 3) return ps;

    // Left-most point (ties broken by the lower y) is guaranteed on the hull.
    let start = 0;
    for (let i = 1; i < ps.length; i++) {
      const p = ps[i]!;
      const q = ps[start]!;
      if (p.x < q.x || (p.x === q.x && p.y < q.y)) start = i;
    }

    const hull: Vec2[] = [];
    let cur = start;
    do {
      hull.push(ps[cur]!);
      let next = (cur + 1) % ps.length;
      for (let i = 0; i < ps.length; i++) {
        if (i === next || i === cur) continue;
        // Is `i` to the right of the line cur->next? Then it is more extreme.
        const rx = ps[i]!.x - ps[cur]!.x;
        const ry = ps[i]!.y - ps[cur]!.y;
        const nx = ps[next]!.x - ps[cur]!.x;
        const ny = ps[next]!.y - ps[cur]!.y;
        const cross = S.mulAdd(nx, ry, -S.mul(ny, rx));
        if (cross < S.ZERO) {
          next = i;
        } else if (cross === S.ZERO) {
          // Collinear: keep the furthest point so we drop interior ones.
          if (S.mulAdd(rx, rx, S.mul(ry, ry)) > S.mulAdd(nx, nx, S.mul(ny, ny))) next = i;
        }
      }
      cur = next;
    } while (cur !== start && hull.length < MAX_POLYGON_VERTICES);

    return hull;
  }

  /** Area-weighted centroid via triangle fan decomposition. */
  static computeCentroid(vs: Vec2[]): Vec2 {
    const c = Vec2.zero();
    let area = S.ZERO;
    const ref = vs[0]!;
    for (let i = 1; i < vs.length - 1; i++) {
      const e1x = vs[i]!.x - ref.x;
      const e1y = vs[i]!.y - ref.y;
      const e2x = vs[i + 1]!.x - ref.x;
      const e2y = vs[i + 1]!.y - ref.y;
      const d = S.mulAdd(e1x, e2y, -S.mul(e1y, e2x));
      const triArea = S.half(d);
      area += triArea;
      const k = S.div(triArea, S.fromInt(3));
      c.x += S.mul(k, e1x + e2x);
      c.y += S.mul(k, e1y + e2y);
    }
    if (S.abs(area) > S.EPSILON) {
      const inv = S.inv(area);
      c.x = S.mul(c.x, inv);
      c.y = S.mul(c.y, inv);
    }
    c.x += ref.x;
    c.y += ref.y;
    return c;
  }

  /* -------------------------- Shape API ------------------------- */

  computeAABB(out: AABB, xf: Transform): AABB {
    Transform.apply(_tmp, xf, this.vertices[0]!);
    out.lower.copyFrom(_tmp);
    out.upper.copyFrom(_tmp);
    for (let i = 1; i < this.vertexCount; i++) {
      Transform.apply(_tmp, xf, this.vertices[i]!);
      out.lower.x = S.min(out.lower.x, _tmp.x);
      out.lower.y = S.min(out.lower.y, _tmp.y);
      out.upper.x = S.max(out.upper.x, _tmp.x);
      out.upper.y = S.max(out.upper.y, _tmp.y);
    }
    if (this.radius > S.ZERO) out.expand(this.radius);
    return out;
  }

  /**
   * Exact polygon mass properties by summing signed triangles from the
   * first vertex. See Mirtich, "Fast and Accurate Computation of Polyhedral
   * Mass Properties" (1996) for the derivation of the `1/12` inertia term.
   */
  computeMass(out: MassData, density: Scalar): MassData {
    const vs = this.vertices;
    const n = this.vertexCount;
    let area = S.ZERO;
    let I = S.ZERO;
    const cx0 = vs[0]!.x;
    const cy0 = vs[0]!.y;
    const center = Vec2.zero();
    const inv3 = S.div(S.ONE, S.fromInt(3));
    const inv12 = S.div(S.ONE, S.fromInt(12));

    for (let i = 1; i < n - 1; i++) {
      const e1x = vs[i]!.x - cx0;
      const e1y = vs[i]!.y - cy0;
      const e2x = vs[i + 1]!.x - cx0;
      const e2y = vs[i + 1]!.y - cy0;

      const d = S.mulAdd(e1x, e2y, -S.mul(e1y, e2x));
      const triArea = S.half(d);
      area += triArea;

      const k = S.mul(triArea, inv3);
      center.x += S.mul(k, e1x + e2x);
      center.y += S.mul(k, e1y + e2y);

      const intx2 = S.mulAdd(e1x, e1x, S.mulAdd(e2x, e1x, S.mul(e2x, e2x)));
      const inty2 = S.mulAdd(e1y, e1y, S.mulAdd(e2y, e1y, S.mul(e2y, e2y)));
      I += S.mul(S.mul(inv12, d), intx2 + inty2);
    }

    out.mass = S.mul(density, area);

    if (S.abs(area) > S.EPSILON) {
      const inv = S.inv(area);
      center.x = S.mul(center.x, inv);
      center.y = S.mul(center.y, inv);
    }
    // Move from the reference vertex back to the local origin.
    out.center.set(center.x + cx0, center.y + cy0);

    // I was computed about vs[0]; shift to the origin (parallel axis).
    I = S.mul(density, I);
    const dOrigin = S.mulAdd(out.center.x, out.center.x, S.mul(out.center.y, out.center.y));
    const dRef = S.mulAdd(center.x, center.x, S.mul(center.y, center.y));
    out.inertia = I + S.mul(out.mass, dOrigin - dRef);
    return out;
  }

  testPoint(xf: Transform, p: Vec2): boolean {
    Transform.applyT(_tmp, xf, p);
    for (let i = 0; i < this.vertexCount; i++) {
      const n = this.normals[i]!;
      const v = this.vertices[i]!;
      const d = S.mulAdd(n.x, _tmp.x - v.x, S.mul(n.y, _tmp.y - v.y));
      if (d > this.radius) return false;
    }
    return true;
  }

  /**
   * Ray cast by clipping the ray against every edge half-plane
   * (the classic "slab" method generalised to arbitrary planes).
   * The polygon skin radius is ignored here — cast against a rounded polygon
   * by inflating the ray instead.
   */
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean {
    out.hit = false;
    Transform.applyT(_tmp, xf, input.p1); // local p1
    Transform.applyT(_tmp2, xf, input.p2); // local p2
    const dx = _tmp2.x - _tmp.x;
    const dy = _tmp2.y - _tmp.y;

    let lower = S.ZERO;
    let upper = input.maxFraction;
    let index = -1;

    for (let i = 0; i < this.vertexCount; i++) {
      const n = this.normals[i]!;
      const v = this.vertices[i]!;
      const num = S.mulAdd(n.x, v.x - _tmp.x, S.mul(n.y, v.y - _tmp.y));
      const den = S.mulAdd(n.x, dx, S.mul(n.y, dy));

      if (den === S.ZERO) {
        if (num < S.ZERO) return false; // parallel and outside
      } else if (den < S.ZERO && num < S.mul(lower, den)) {
        lower = S.div(num, den);
        index = i;
      } else if (den > S.ZERO && num < S.mul(upper, den)) {
        upper = S.div(num, den);
      }
      if (upper < lower) return false;
    }

    if (index < 0) return false; // started inside

    out.fraction = lower;
    out.point.set(S.mulAdd(input.p2.x - input.p1.x, lower, input.p1.x),
                  S.mulAdd(input.p2.y - input.p1.y, lower, input.p1.y));
    Rot.rotate(out.normal, xf.q, this.normals[index]!);
    out.hit = true;
    return true;
  }

  supportIndex(d: Vec2): number {
    let best = 0;
    let bestVal = S.mulAdd(this.vertices[0]!.x, d.x, S.mul(this.vertices[0]!.y, d.y));
    for (let i = 1; i < this.vertexCount; i++) {
      const v = this.vertices[i]!;
      const val = S.mulAdd(v.x, d.x, S.mul(v.y, d.y));
      if (val > bestVal) {
        bestVal = val;
        best = i;
      }
    }
    return best;
  }

  getVertex(i: number): Vec2 {
    return this.vertices[i]!;
  }

  clone(): Polygon {
    return new Polygon(this.vertices, this.radius);
  }
}
