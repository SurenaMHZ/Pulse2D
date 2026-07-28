/**
 * @module collision/shapes/Circle
 */

import * as S from './../../math/scalar.js';
import type { Scalar } from './../../math/scalar.js';
import { Vec2 } from './../../math/Vec2.js';
import { Transform } from './../../math/Transform.js';
import { AABB } from './../AABB.js';
import { ShapeType } from './../Shape.js';
import type { MassData, RayCastInput, RayCastOutput, Shape } from './../Shape.js';

const _tmp = Vec2.zero();
const _tmp2 = Vec2.zero();

/**
 * A circle, defined by a local centre and a radius.
 *
 * The cheapest primitive in the engine: collision against another circle is
 * a single distance comparison, and mass properties are closed-form.
 */
export class Circle implements Shape {
  readonly type = ShapeType.Circle;
  readonly radius: Scalar;
  /** Local centre. */
  readonly center: Vec2;
  readonly vertexCount = 1;

  /**
   * @param radius radius in metres (backend scalar)
   * @param center local centre, defaults to the origin
   */
  constructor(radius: Scalar, center?: Vec2) {
    this.radius = radius;
    this.center = center ? center.clone() : Vec2.zero();
  }

  /** Convenience constructor taking plain JS floats. */
  static of(radius: number, cx = 0, cy = 0): Circle {
    return new Circle(S.fromFloat(radius), Vec2.of(cx, cy));
  }

  computeAABB(out: AABB, xf: Transform): AABB {
    Transform.apply(_tmp, xf, this.center);
    out.lower.set(_tmp.x - this.radius, _tmp.y - this.radius);
    out.upper.set(_tmp.x + this.radius, _tmp.y + this.radius);
    return out;
  }

  /**
   * `m = ρ·π·r²`, `I = m·(r²/2 + |c|²)` — the parallel-axis theorem moves the
   * inertia from the centroid to the body origin.
   */
  computeMass(out: MassData, density: Scalar): MassData {
    const r2 = S.mul(this.radius, this.radius);
    out.mass = S.mul(density, S.mul(S.PI, r2));
    out.center.copyFrom(this.center);
    const cSq = S.mulAdd(this.center.x, this.center.x, S.mul(this.center.y, this.center.y));
    out.inertia = S.mul(out.mass, S.half(r2) + cSq);
    return out;
  }

  testPoint(xf: Transform, p: Vec2): boolean {
    Transform.apply(_tmp, xf, this.center);
    _tmp2.set(p.x - _tmp.x, p.y - _tmp.y);
    return _tmp2.lengthSq() <= S.mul(this.radius, this.radius);
  }

  /**
   * Analytic ray/circle intersection.
   *
   * Solves `|p1 + t·d - c|² = r²` and keeps the smaller root, which is the
   * entry point. A ray starting inside the circle reports a miss (consistent
   * with Box2D, and what game code almost always wants).
   */
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean {
    out.hit = false;
    Transform.apply(_tmp, xf, this.center); // world centre

    const sx = input.p1.x - _tmp.x;
    const sy = input.p1.y - _tmp.y;
    const b = S.mulAdd(sx, sx, S.mul(sy, sy)) - S.mul(this.radius, this.radius);

    const dx = input.p2.x - input.p1.x;
    const dy = input.p2.y - input.p1.y;

    const c = S.mulAdd(sx, dx, S.mul(sy, dy));
    const dd = S.mulAdd(dx, dx, S.mul(dy, dy));
    const sigma = S.mulAdd(c, c, -S.mul(dd, b));

    if (sigma < S.ZERO || dd < S.EPSILON_SQ) return false;

    let t = -(c + S.sqrt(sigma));
    if (t < S.ZERO || S.mul(input.maxFraction, dd) < t) return false;
    t = S.div(t, dd);

    out.fraction = t;
    out.point.set(S.mulAdd(dx, t, input.p1.x), S.mulAdd(dy, t, input.p1.y));
    out.normal.set(out.point.x - _tmp.x, out.point.y - _tmp.y);
    out.normal.normalize();
    out.hit = true;
    return true;
  }

  supportIndex(_d: Vec2): number {
    return 0;
  }

  getVertex(_i: number): Vec2 {
    return this.center;
  }

  clone(): Circle {
    return new Circle(this.radius, this.center);
  }
}
