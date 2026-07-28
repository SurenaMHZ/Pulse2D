/**
 * @module math/Transform
 *
 * A rigid transform: a translation `p` plus a rotation `q`.
 * Maps local (shape) space to world space.
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';
import { Vec2 } from './Vec2.js';
import { Rot } from './Rot.js';

export class Transform {
  /** Translation. */
  readonly p: Vec2;
  /** Rotation. */
  readonly q: Rot;

  constructor(p?: Vec2, q?: Rot) {
    this.p = p ? p.clone() : Vec2.zero();
    this.q = q ? q.clone() : new Rot();
  }

  /** Reset to identity. */
  setIdentity(): this {
    this.p.setZero();
    this.q.setIdentity();
    return this;
  }

  set(p: Vec2, angle: Scalar): this {
    this.p.copyFrom(p);
    this.q.setAngle(angle);
    return this;
  }

  copyFrom(t: Transform): this {
    this.p.copyFrom(t.p);
    this.q.copyFrom(t.q);
    return this;
  }

  clone(): Transform {
    return new Transform(this.p, this.q);
  }

  /** `out = xf · v` — local point to world point. */
  static apply(out: Vec2, xf: Transform, v: Vec2): Vec2 {
    const q = xf.q;
    const x = S.mulAdd(q.c, v.x, -S.mul(q.s, v.y)) + xf.p.x;
    const y = S.mulAdd(q.s, v.x, S.mul(q.c, v.y)) + xf.p.y;
    out.x = x;
    out.y = y;
    return out;
  }

  /** `out = xf⁻¹ · v` — world point to local point. */
  static applyT(out: Vec2, xf: Transform, v: Vec2): Vec2 {
    const q = xf.q;
    const px = v.x - xf.p.x;
    const py = v.y - xf.p.y;
    const x = S.mulAdd(q.c, px, S.mul(q.s, py));
    const y = S.mulAdd(-q.s, px, S.mul(q.c, py));
    out.x = x;
    out.y = y;
    return out;
  }

  /** `out = a · b` — compose two transforms. */
  static mulTo(out: Transform, a: Transform, b: Transform): Transform {
    Rot.mulTo(out.q, a.q, b.q);
    Rot.rotate(out.p, a.q, b.p);
    out.p.add(a.p);
    return out;
  }

  /** `out = aᵀ · b` — the transform of `b` expressed in `a`'s frame. */
  static mulTTo(out: Transform, a: Transform, b: Transform): Transform {
    Rot.mulTTo(out.q, a.q, b.q);
    const dx = b.p.x - a.p.x;
    const dy = b.p.y - a.p.y;
    out.p.x = S.mulAdd(a.q.c, dx, S.mul(a.q.s, dy));
    out.p.y = S.mulAdd(-a.q.s, dx, S.mul(a.q.c, dy));
    return out;
  }

  toString(): string {
    return `Transform(p=${this.p}, q=${this.q})`;
  }
}
