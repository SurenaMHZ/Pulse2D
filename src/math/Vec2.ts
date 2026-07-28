/**
 * @module math/Vec2
 *
 * 2-component vector.
 *
 * The class is a plain mutable `{x, y}` pair — monomorphic, hidden-class
 * stable and inlineable by every JIT. All operators come in two flavours:
 *
 * * **allocating** — `Vec2.add(a, b)` returns a fresh vector, convenient for
 *   setup code and tests;
 * * **destination** — `Vec2.addTo(out, a, b)` writes into `out` and allocates
 *   nothing. The solver uses exclusively this form, which is why Pulse2D
 *   produces **zero garbage per step**.
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';

export class Vec2 {
  x: Scalar;
  y: Scalar;

  constructor(x: Scalar = S.ZERO, y: Scalar = S.ZERO) {
    this.x = x;
    this.y = y;
  }

  /** Build from plain JS floats (converts through the active backend). */
  static of(x: number, y: number): Vec2 {
    return new Vec2(S.fromFloat(x), S.fromFloat(y));
  }

  /** A new zero vector. */
  static zero(): Vec2 {
    return new Vec2(S.ZERO, S.ZERO);
  }

  set(x: Scalar, y: Scalar): this {
    this.x = x;
    this.y = y;
    return this;
  }

  setZero(): this {
    this.x = S.ZERO;
    this.y = S.ZERO;
    return this;
  }

  copyFrom(v: Vec2): this {
    this.x = v.x;
    this.y = v.y;
    return this;
  }

  clone(): Vec2 {
    return new Vec2(this.x, this.y);
  }

  /** `true` when both components are exactly zero. */
  isZero(): boolean {
    return this.x === S.ZERO && this.y === S.ZERO;
  }

  /** Plain-float view, for logging and rendering. */
  toFloats(): { x: number; y: number } {
    return { x: S.toFloat(this.x), y: S.toFloat(this.y) };
  }

  /* ---------------- allocating helpers ---------------- */

  static add(a: Vec2, b: Vec2): Vec2 {
    return new Vec2(a.x + b.x, a.y + b.y);
  }

  static sub(a: Vec2, b: Vec2): Vec2 {
    return new Vec2(a.x - b.x, a.y - b.y);
  }

  static scale(a: Vec2, s: Scalar): Vec2 {
    return new Vec2(S.mul(a.x, s), S.mul(a.y, s));
  }

  static neg(a: Vec2): Vec2 {
    return new Vec2(-a.x, -a.y);
  }

  /* ---------------- destination helpers --------------- */

  /** `out = a + b` */
  static addTo(out: Vec2, a: Vec2, b: Vec2): Vec2 {
    out.x = a.x + b.x;
    out.y = a.y + b.y;
    return out;
  }

  /** `out = a - b` */
  static subTo(out: Vec2, a: Vec2, b: Vec2): Vec2 {
    out.x = a.x - b.x;
    out.y = a.y - b.y;
    return out;
  }

  /** `out = a * s` */
  static scaleTo(out: Vec2, a: Vec2, s: Scalar): Vec2 {
    out.x = S.mul(a.x, s);
    out.y = S.mul(a.y, s);
    return out;
  }

  /** `out = a + b * s` — the workhorse of the impulse solver. */
  static addScaledTo(out: Vec2, a: Vec2, b: Vec2, s: Scalar): Vec2 {
    out.x = S.mulAdd(b.x, s, a.x);
    out.y = S.mulAdd(b.y, s, a.y);
    return out;
  }

  /** `out = a * sa + b * sb` */
  static combineTo(out: Vec2, a: Vec2, sa: Scalar, b: Vec2, sb: Scalar): Vec2 {
    out.x = S.mulAdd(a.x, sa, S.mul(b.x, sb));
    out.y = S.mulAdd(a.y, sa, S.mul(b.y, sb));
    return out;
  }

  /** `out = -a` */
  static negTo(out: Vec2, a: Vec2): Vec2 {
    out.x = -a.x;
    out.y = -a.y;
    return out;
  }

  /** `out = lerp(a, b, t)` */
  static lerpTo(out: Vec2, a: Vec2, b: Vec2, t: Scalar): Vec2 {
    out.x = S.lerp(a.x, b.x, t);
    out.y = S.lerp(a.y, b.y, t);
    return out;
  }

  /** `out = componentwise min(a, b)` */
  static minTo(out: Vec2, a: Vec2, b: Vec2): Vec2 {
    out.x = S.min(a.x, b.x);
    out.y = S.min(a.y, b.y);
    return out;
  }

  /** `out = componentwise max(a, b)` */
  static maxTo(out: Vec2, a: Vec2, b: Vec2): Vec2 {
    out.x = S.max(a.x, b.x);
    out.y = S.max(a.y, b.y);
    return out;
  }

  /* ---------------- in-place mutators ----------------- */

  add(v: Vec2): this {
    this.x += v.x;
    this.y += v.y;
    return this;
  }

  sub(v: Vec2): this {
    this.x -= v.x;
    this.y -= v.y;
    return this;
  }

  /** `this += v * s` */
  addScaled(v: Vec2, s: Scalar): this {
    this.x = S.mulAdd(v.x, s, this.x);
    this.y = S.mulAdd(v.y, s, this.y);
    return this;
  }

  scale(s: Scalar): this {
    this.x = S.mul(this.x, s);
    this.y = S.mul(this.y, s);
    return this;
  }

  neg(): this {
    this.x = -this.x;
    this.y = -this.y;
    return this;
  }

  /* ---------------- products & metrics ---------------- */

  /** Dot product `a·b`. */
  static dot(a: Vec2, b: Vec2): Scalar {
    return S.mulAdd(a.x, b.x, S.mul(a.y, b.y));
  }

  /** Scalar cross product `a×b` (the z component of the 3D cross). */
  static cross(a: Vec2, b: Vec2): Scalar {
    return S.mulAdd(a.x, b.y, -S.mul(a.y, b.x));
  }

  /** `out = v × s` — rotate `v` by -90° and scale. */
  static crossVS(out: Vec2, v: Vec2, s: Scalar): Vec2 {
    const x = S.mul(s, v.y);
    out.y = -S.mul(s, v.x);
    out.x = x;
    return out;
  }

  /** `out = s × v` — rotate `v` by +90° and scale. */
  static crossSV(out: Vec2, s: Scalar, v: Vec2): Vec2 {
    const x = -S.mul(s, v.y);
    out.y = S.mul(s, v.x);
    out.x = x;
    return out;
  }

  /** `out = perpendicular(v)` = `(-v.y, v.x)`, i.e. a +90° rotation. */
  static perpTo(out: Vec2, v: Vec2): Vec2 {
    const x = -v.y;
    out.y = v.x;
    out.x = x;
    return out;
  }

  /** `out = (v.y, -v.x)`, i.e. a -90° rotation. */
  static rperpTo(out: Vec2, v: Vec2): Vec2 {
    const x = v.y;
    out.y = -v.x;
    out.x = x;
    return out;
  }

  /** Squared magnitude — prefer this whenever you can avoid the sqrt. */
  lengthSq(): Scalar {
    return S.mulAdd(this.x, this.x, S.mul(this.y, this.y));
  }

  /** Magnitude. */
  length(): Scalar {
    return S.sqrt(this.lengthSq());
  }

  /** Squared distance between two points. */
  static distanceSq(a: Vec2, b: Vec2): Scalar {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return S.mulAdd(dx, dx, S.mul(dy, dy));
  }

  /** Distance between two points. */
  static distance(a: Vec2, b: Vec2): Scalar {
    return S.sqrt(Vec2.distanceSq(a, b));
  }

  /**
   * Scale to unit length in place and return the **previous** length.
   * A zero-length vector is left untouched and `0` is returned, so callers
   * never have to guard against NaN.
   */
  normalize(): Scalar {
    const lsq = this.lengthSq();
    if (lsq < S.EPSILON_SQ) return S.ZERO;
    const len = S.sqrt(lsq);
    const inv = S.inv(len);
    this.x = S.mul(this.x, inv);
    this.y = S.mul(this.y, inv);
    return len;
  }

  /** `out = normalize(v)`; returns the original length. */
  static normalizeTo(out: Vec2, v: Vec2): Scalar {
    const lsq = S.mulAdd(v.x, v.x, S.mul(v.y, v.y));
    if (lsq < S.EPSILON_SQ) {
      out.x = S.ZERO;
      out.y = S.ZERO;
      return S.ZERO;
    }
    const len = S.sqrt(lsq);
    const inv = S.inv(len);
    out.x = S.mul(v.x, inv);
    out.y = S.mul(v.y, inv);
    return len;
  }

  /** Clamp the magnitude to `maxLen`, in place. */
  truncate(maxLen: Scalar): this {
    const lsq = this.lengthSq();
    const m2 = S.mul(maxLen, maxLen);
    if (lsq > m2 && lsq > S.EPSILON_SQ) {
      const k = S.div(maxLen, S.sqrt(lsq));
      this.x = S.mul(this.x, k);
      this.y = S.mul(this.y, k);
    }
    return this;
  }

  /** Exact component-wise equality. */
  static equals(a: Vec2, b: Vec2): boolean {
    return a.x === b.x && a.y === b.y;
  }

  /** `true` when both components are finite (debug guard). */
  isValid(): boolean {
    return Number.isFinite(this.x) && Number.isFinite(this.y);
  }

  toString(): string {
    return `Vec2(${S.toFloat(this.x)}, ${S.toFloat(this.y)})`;
  }
}
