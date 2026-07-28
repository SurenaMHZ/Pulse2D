/**
 * @module collision/AABB
 *
 * Axis-aligned bounding box — the currency of the broad phase.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';

export class AABB {
  /** Lower bound (min x, min y). */
  readonly lower: Vec2;
  /** Upper bound (max x, max y). */
  readonly upper: Vec2;

  constructor() {
    this.lower = Vec2.zero();
    this.upper = Vec2.zero();
  }

  set(lx: Scalar, ly: Scalar, ux: Scalar, uy: Scalar): this {
    this.lower.x = lx;
    this.lower.y = ly;
    this.upper.x = ux;
    this.upper.y = uy;
    return this;
  }

  copyFrom(a: AABB): this {
    this.lower.copyFrom(a.lower);
    this.upper.copyFrom(a.upper);
    return this;
  }

  clone(): AABB {
    return new AABB().copyFrom(this);
  }

  /** Collapse to a single point (the identity for `combine`). */
  setEmpty(): this {
    this.lower.set(S.MAX_VALUE, S.MAX_VALUE);
    this.upper.set(S.MIN_VALUE, S.MIN_VALUE);
    return this;
  }

  /** `out = center` */
  getCenter(out: Vec2): Vec2 {
    out.x = S.half(this.lower.x + this.upper.x);
    out.y = S.half(this.lower.y + this.upper.y);
    return out;
  }

  /** `out = half extents` */
  getExtents(out: Vec2): Vec2 {
    out.x = S.half(this.upper.x - this.lower.x);
    out.y = S.half(this.upper.y - this.lower.y);
    return out;
  }

  /** Perimeter — the cost metric used by the dynamic tree (SAH). */
  perimeter(): Scalar {
    return S.mulInt((this.upper.x - this.lower.x) + (this.upper.y - this.lower.y), 2);
  }

  /** Area. */
  area(): Scalar {
    return S.mul(this.upper.x - this.lower.x, this.upper.y - this.lower.y);
  }

  /** Grow by `r` on every side. */
  expand(r: Scalar): this {
    this.lower.x -= r;
    this.lower.y -= r;
    this.upper.x += r;
    this.upper.y += r;
    return this;
  }

  /** Extend to also contain `p`. */
  addPoint(p: Vec2): this {
    this.lower.x = S.min(this.lower.x, p.x);
    this.lower.y = S.min(this.lower.y, p.y);
    this.upper.x = S.max(this.upper.x, p.x);
    this.upper.y = S.max(this.upper.y, p.y);
    return this;
  }

  /** `out = smallest box containing both a and b`. */
  static combineTo(out: AABB, a: AABB, b: AABB): AABB {
    out.lower.x = S.min(a.lower.x, b.lower.x);
    out.lower.y = S.min(a.lower.y, b.lower.y);
    out.upper.x = S.max(a.upper.x, b.upper.x);
    out.upper.y = S.max(a.upper.y, b.upper.y);
    return out;
  }

  /** Perimeter of `combine(a, b)` without building the box. */
  static combinedPerimeter(a: AABB, b: AABB): Scalar {
    const lx = S.min(a.lower.x, b.lower.x);
    const ly = S.min(a.lower.y, b.lower.y);
    const ux = S.max(a.upper.x, b.upper.x);
    const uy = S.max(a.upper.y, b.upper.y);
    return S.mulInt((ux - lx) + (uy - ly), 2);
  }

  /** `true` when `a` and `b` overlap (touching counts as overlapping). */
  static overlaps(a: AABB, b: AABB): boolean {
    if (b.lower.x > a.upper.x || b.lower.y > a.upper.y) return false;
    if (a.lower.x > b.upper.x || a.lower.y > b.upper.y) return false;
    return true;
  }

  /** `true` when `this` fully contains `b`. */
  contains(b: AABB): boolean {
    return (
      this.lower.x <= b.lower.x &&
      this.lower.y <= b.lower.y &&
      b.upper.x <= this.upper.x &&
      b.upper.y <= this.upper.y
    );
  }

  /** `true` when the point lies inside. */
  containsPoint(p: Vec2): boolean {
    return p.x >= this.lower.x && p.x <= this.upper.x && p.y >= this.lower.y && p.y <= this.upper.y;
  }

  /** Debug guard — non-inverted and finite. */
  isValid(): boolean {
    return (
      this.upper.x >= this.lower.x &&
      this.upper.y >= this.lower.y &&
      this.lower.isValid() &&
      this.upper.isValid()
    );
  }

  /**
   * Slab-method ray cast against this box.
   *
   * @returns the entry parameter `t ∈ [0, maxFraction]`, or `-1` on a miss.
   */
  rayCast(p1: Vec2, d: Vec2, maxFraction: Scalar): Scalar {
    let tmin = S.ZERO;
    let tmax = maxFraction;

    // x slab
    if (S.abs(d.x) < S.EPSILON) {
      if (p1.x < this.lower.x || p1.x > this.upper.x) return S.NEG_ONE;
    } else {
      const invD = S.inv(d.x);
      let t1 = S.mul(this.lower.x - p1.x, invD);
      let t2 = S.mul(this.upper.x - p1.x, invD);
      if (t1 > t2) {
        const t = t1;
        t1 = t2;
        t2 = t;
      }
      tmin = S.max(tmin, t1);
      tmax = S.min(tmax, t2);
      if (tmin > tmax) return S.NEG_ONE;
    }

    // y slab
    if (S.abs(d.y) < S.EPSILON) {
      if (p1.y < this.lower.y || p1.y > this.upper.y) return S.NEG_ONE;
    } else {
      const invD = S.inv(d.y);
      let t1 = S.mul(this.lower.y - p1.y, invD);
      let t2 = S.mul(this.upper.y - p1.y, invD);
      if (t1 > t2) {
        const t = t1;
        t1 = t2;
        t2 = t;
      }
      tmin = S.max(tmin, t1);
      tmax = S.min(tmax, t2);
      if (tmin > tmax) return S.NEG_ONE;
    }

    return tmin;
  }
}
