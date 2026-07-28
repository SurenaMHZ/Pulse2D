/**
 * @module math/Rot
 *
 * A 2D rotation stored as `(sin θ, cos θ)`.
 *
 * Caching the sine/cosine pair means the solver never calls a trig function in
 * its inner loops — it only needs two multiplies and an add to rotate a
 * vector. The angle itself is recovered with {@link Rot#getAngle} when needed.
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';
import { Vec2 } from './Vec2.js';
import { sinCos, atan2 } from './trig.js';

const _sc: Scalar[] = [S.ZERO, S.ZERO];

export class Rot {
  /** `sin θ` */
  s: Scalar;
  /** `cos θ` */
  c: Scalar;

  constructor(angle: Scalar = S.ZERO) {
    this.s = S.ZERO;
    this.c = S.ONE;
    if (angle !== S.ZERO) this.setAngle(angle);
  }

  /** Build from a plain-float angle in radians. */
  static of(angle: number): Rot {
    return new Rot(S.fromFloat(angle));
  }

  /** Reset to the identity rotation. */
  setIdentity(): this {
    this.s = S.ZERO;
    this.c = S.ONE;
    return this;
  }

  /**
   * Set from an angle in radians.
   *
   * The polynomial kernels are accurate to ~1e-11 but not *exactly* on the
   * unit circle, and the solver relies on `s² + c² = 1` when it inverts
   * rotations. One normalisation here costs a sqrt and makes
   * `rotateT(rotate(v)) === v` hold to full double precision.
   */
  setAngle(angle: Scalar): this {
    sinCos(angle, _sc);
    this.s = _sc[0]!;
    this.c = _sc[1]!;
    return this.normalize();
  }

  /** Set the raw sin/cos pair (caller guarantees `s² + c² = 1`). */
  setSinCos(s: Scalar, c: Scalar): this {
    this.s = s;
    this.c = c;
    return this;
  }

  copyFrom(r: Rot): this {
    this.s = r.s;
    this.c = r.c;
    return this;
  }

  clone(): Rot {
    const r = new Rot();
    r.s = this.s;
    r.c = this.c;
    return r;
  }

  /** Recover the angle in `(-π, π]`. */
  getAngle(): Scalar {
    return atan2(this.s, this.c);
  }

  /** `out = local x-axis of this rotation` = first column. */
  getXAxis(out: Vec2): Vec2 {
    out.x = this.c;
    out.y = this.s;
    return out;
  }

  /** `out = local y-axis of this rotation` = second column. */
  getYAxis(out: Vec2): Vec2 {
    out.x = -this.s;
    out.y = this.c;
    return out;
  }

  /**
   * Re-normalise after incremental updates so `s² + c² = 1` again.
   *
   * Called once per body per step by the integrator. It costs one sqrt and
   * keeps long-running simulations from slowly shearing.
   */
  normalize(): this {
    const mag = S.sqrt(S.mulAdd(this.s, this.s, S.mul(this.c, this.c)));
    if (mag < S.EPSILON) {
      this.s = S.ZERO;
      this.c = S.ONE;
      return this;
    }
    const inv = S.inv(mag);
    this.s = S.mul(this.s, inv);
    this.c = S.mul(this.c, inv);
    return this;
  }

  /**
   * Advance by `w · dt` using the small-angle exponential map
   * `(s, c) += (c, -s)·δ`, then re-normalise.
   *
   * Cheaper than recomputing `sinCos` and just as stable for the angular
   * velocities a game produces.
   */
  integrate(deltaAngle: Scalar): this {
    const s = S.mulAdd(this.c, deltaAngle, this.s);
    const c = S.mulAdd(-this.s, deltaAngle, this.c);
    this.s = s;
    this.c = c;
    return this.normalize();
  }

  /* ---------------- static operators ---------------- */

  /** `out = q · r` (apply `r` first, then `q`). */
  static mulTo(out: Rot, q: Rot, r: Rot): Rot {
    const s = S.mulAdd(q.s, r.c, S.mul(q.c, r.s));
    const c = S.mulAdd(q.c, r.c, -S.mul(q.s, r.s));
    out.s = s;
    out.c = c;
    return out;
  }

  /** `out = qᵀ · r` (relative rotation from `q` to `r`). */
  static mulTTo(out: Rot, q: Rot, r: Rot): Rot {
    const s = S.mulAdd(q.c, r.s, -S.mul(q.s, r.c));
    const c = S.mulAdd(q.c, r.c, S.mul(q.s, r.s));
    out.s = s;
    out.c = c;
    return out;
  }

  /** `out = q · v` — rotate a vector into world space. */
  static rotate(out: Vec2, q: Rot, v: Vec2): Vec2 {
    const x = S.mulAdd(q.c, v.x, -S.mul(q.s, v.y));
    const y = S.mulAdd(q.s, v.x, S.mul(q.c, v.y));
    out.x = x;
    out.y = y;
    return out;
  }

  /** `out = qᵀ · v` — rotate a vector into local space. */
  static rotateT(out: Vec2, q: Rot, v: Vec2): Vec2 {
    const x = S.mulAdd(q.c, v.x, S.mul(q.s, v.y));
    const y = S.mulAdd(-q.s, v.x, S.mul(q.c, v.y));
    out.x = x;
    out.y = y;
    return out;
  }

  /** Shortest signed angle from `a` to `b`, in `(-π, π]`. */
  static relativeAngle(a: Rot, b: Rot): Scalar {
    const s = S.mulAdd(a.c, b.s, -S.mul(a.s, b.c));
    const c = S.mulAdd(a.c, b.c, S.mul(a.s, b.s));
    return atan2(s, c);
  }

  /** Normalised interpolation between two rotations. */
  static nlerpTo(out: Rot, a: Rot, b: Rot, t: Scalar): Rot {
    out.s = S.lerp(a.s, b.s, t);
    out.c = S.lerp(a.c, b.c, t);
    return out.normalize();
  }

  toString(): string {
    return `Rot(${S.toFloat(this.getAngle())} rad)`;
  }
}
