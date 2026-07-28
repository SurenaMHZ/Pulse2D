/**
 * @module math/Mat22
 *
 * A 2×2 matrix stored as two column vectors, used to solve the coupled
 * 2-DOF constraint blocks (revolute point constraint, prismatic, weld…).
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';
import { Vec2 } from './Vec2.js';

export class Mat22 {
  /** First column. */
  readonly ex: Vec2;
  /** Second column. */
  readonly ey: Vec2;

  constructor() {
    this.ex = Vec2.zero();
    this.ey = Vec2.zero();
  }

  set(a11: Scalar, a12: Scalar, a21: Scalar, a22: Scalar): this {
    this.ex.x = a11;
    this.ex.y = a21;
    this.ey.x = a12;
    this.ey.y = a22;
    return this;
  }

  setZero(): this {
    this.ex.setZero();
    this.ey.setZero();
    return this;
  }

  /** Determinant. */
  det(): Scalar {
    return S.mulAdd(this.ex.x, this.ey.y, -S.mul(this.ey.x, this.ex.y));
  }

  /**
   * `out = M⁻¹ · b`, solved by Cramer's rule.
   * A singular matrix yields the zero vector instead of NaN.
   */
  solve(out: Vec2, b: Vec2): Vec2 {
    const a = this.ex.x;
    const bb = this.ey.x;
    const c = this.ex.y;
    const d = this.ey.y;
    let det = S.mulAdd(a, d, -S.mul(bb, c));
    if (S.abs(det) < S.EPSILON) {
      out.setZero();
      return out;
    }
    det = S.inv(det);
    const x = S.mul(det, S.mulAdd(d, b.x, -S.mul(bb, b.y)));
    const y = S.mul(det, S.mulAdd(a, b.y, -S.mul(c, b.x)));
    out.x = x;
    out.y = y;
    return out;
  }

  /** `out = M⁻¹` (zeroed when singular). */
  invertTo(out: Mat22): Mat22 {
    const a = this.ex.x;
    const b = this.ey.x;
    const c = this.ex.y;
    const d = this.ey.y;
    let det = S.mulAdd(a, d, -S.mul(b, c));
    if (S.abs(det) < S.EPSILON) return out.setZero();
    det = S.inv(det);
    out.ex.x = S.mul(det, d);
    out.ey.x = -S.mul(det, b);
    out.ex.y = -S.mul(det, c);
    out.ey.y = S.mul(det, a);
    return out;
  }

  /** `out = M · v` */
  static apply(out: Vec2, m: Mat22, v: Vec2): Vec2 {
    const x = S.mulAdd(m.ex.x, v.x, S.mul(m.ey.x, v.y));
    const y = S.mulAdd(m.ex.y, v.x, S.mul(m.ey.y, v.y));
    out.x = x;
    out.y = y;
    return out;
  }
}

/**
 * A symmetric 3×3 matrix, needed by the weld joint and by the revolute joint
 * when its motor and limit are coupled with the point constraint.
 */
export class Mat33 {
  /** Column 1, xy part. */
  readonly ex: Vec2;
  /** Column 2, xy part. */
  readonly ey: Vec2;

  /** Column 1 z-component. */
  exz: Scalar = S.ZERO;
  /** Column 2 z-component. */
  eyz: Scalar = S.ZERO;
  /** Column 3. */
  ezx: Scalar = S.ZERO;
  ezy: Scalar = S.ZERO;
  ezz: Scalar = S.ZERO;

  constructor() {
    this.ex = Vec2.zero();
    this.ey = Vec2.zero();
  }

  setZero(): this {
    this.ex.setZero();
    this.ey.setZero();
    this.exz = S.ZERO;
    this.eyz = S.ZERO;
    this.ezx = S.ZERO;
    this.ezy = S.ZERO;
    this.ezz = S.ZERO;
    return this;
  }

  /**
   * Solve the 2×2 upper-left block only, keeping the third row/column out of
   * the system. Used when a joint's angular DOF is handled separately.
   */
  solve22(out: Vec2, bx: Scalar, by: Scalar): Vec2 {
    const a11 = this.ex.x;
    const a12 = this.ey.x;
    const a21 = this.ex.y;
    const a22 = this.ey.y;
    let det = S.mulAdd(a11, a22, -S.mul(a12, a21));
    if (S.abs(det) < S.EPSILON) {
      out.setZero();
      return out;
    }
    det = S.inv(det);
    out.x = S.mul(det, S.mulAdd(a22, bx, -S.mul(a12, by)));
    out.y = S.mul(det, S.mulAdd(a11, by, -S.mul(a21, bx)));
    return out;
  }

  /** Solve the full 3×3 system; writes `[x, y, z]` into `out3`. */
  solve33(out3: Scalar[], bx: Scalar, by: Scalar, bz: Scalar): Scalar[] {
    // cross products of the columns
    const c1x = S.mulAdd(this.ey.y, this.ezz, -S.mul(this.eyz, this.ezy));
    const c1y = S.mulAdd(this.eyz, this.ezx, -S.mul(this.ey.x, this.ezz));
    const c1z = S.mulAdd(this.ey.x, this.ezy, -S.mul(this.ey.y, this.ezx));

    let det = S.mulAdd(this.ex.x, c1x, S.mulAdd(this.ex.y, c1y, S.mul(this.exz, c1z)));
    if (S.abs(det) < S.EPSILON) {
      out3[0] = S.ZERO;
      out3[1] = S.ZERO;
      out3[2] = S.ZERO;
      return out3;
    }
    det = S.inv(det);

    // Cramer's rule on each column
    const d1 = S.mulAdd(bx, c1x, S.mulAdd(by, c1y, S.mul(bz, c1z)));

    const c2x = S.mulAdd(by, this.ezz, -S.mul(bz, this.ezy));
    const c2y = S.mulAdd(bz, this.ezx, -S.mul(bx, this.ezz));
    const c2z = S.mulAdd(bx, this.ezy, -S.mul(by, this.ezx));
    const d2 = S.mulAdd(this.ex.x, c2x, S.mulAdd(this.ex.y, c2y, S.mul(this.exz, c2z)));

    const c3x = S.mulAdd(this.ey.y, bz, -S.mul(this.eyz, by));
    const c3y = S.mulAdd(this.eyz, bx, -S.mul(this.ey.x, bz));
    const c3z = S.mulAdd(this.ey.x, by, -S.mul(this.ey.y, bx));
    const d3 = S.mulAdd(this.ex.x, c3x, S.mulAdd(this.ex.y, c3y, S.mul(this.exz, c3z)));

    out3[0] = S.mul(det, d1);
    out3[1] = S.mul(det, d2);
    out3[2] = S.mul(det, d3);
    return out3;
  }
}
