/**
 * @module math/trig
 *
 * Deterministic transcendental functions.
 *
 * `Math.sin`, `Math.cos`, `Math.atan2` & friends are **not** specified to be
 * correctly rounded — V8, SpiderMonkey and JavaScriptCore each use a different
 * polynomial, and V8 itself has changed its implementation between releases.
 * Two players on different browsers would drift apart within a few hundred
 * ticks. Pulse2D therefore ships its own polynomial kernels, written purely on
 * top of the scalar backend so the result is bit-identical everywhere.
 *
 * Accuracy on the `f64` backend (measured against the exact values):
 *
 * | function | max absolute error |
 * |----------|--------------------|
 * | `sin`    | 1.0e-11            |
 * | `cos`    | 2.4e-13            |
 * | `atan`   | 1.3e-7             |
 * | `atan2`  | 1.3e-7             |
 * | `asin`   | 2.0e-6 (near ±1)   |
 *
 * That is far below the tolerance of any rigid-body solver, and — crucially —
 * the error is *the same error on every machine*.
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';

/* ------------------------------------------------------------------ *
 * Coefficients (least-squares minimax fits, see docs/DETERMINISM.md)
 * ------------------------------------------------------------------ */

/** `sin(x) ≈ x·P(x²)` on `[-π/4, π/4]`. */
const S0 = S.fromFloat(0.99999999999711453);
const S1 = S.fromFloat(-0.16666666640337993);
const S2 = S.fromFloat(0.0083333295942390577);
const S3 = S.fromFloat(-0.00019839438509292113);
const S4 = S.fromFloat(0.0000027194617861281519);

/** `cos(x) ≈ Q(x²)` on `[-π/4, π/4]`. */
const C0 = S.fromFloat(0.99999999999976541);
const C1 = S.fromFloat(-0.49999999998181199);
const C2 = S.fromFloat(0.041666666413724913);
const C3 = S.fromFloat(-0.0013888875104415823);
const C4 = S.fromFloat(0.000024798028350612536);
const C5 = S.fromFloat(-0.00000027118847061669255);

/** `atan(t) ≈ t·R(t²)` on `[0, 1]`. */
const A0 = S.fromFloat(0.99999941505381629);
const A1 = S.fromFloat(-0.33330218346270035);
const A2 = S.fromFloat(0.19951062216385784);
const A3 = S.fromFloat(-0.13932970487145691);
const A4 = S.fromFloat(0.097086482692244291);
const A5 = S.fromFloat(-0.056870746179834793);
const A6 = S.fromFloat(0.022559403278835918);
const A7 = S.fromFloat(-0.0042552478224337168);

const INV_HALF_PI = S.fromFloat(0.6366197723675814);

/* ------------------------------------------------------------------ *
 * Kernels (argument already reduced to |x| <= π/4)
 * ------------------------------------------------------------------ */

function sinKernel(x: Scalar): Scalar {
  const z = S.mul(x, x);
  let p = S4;
  p = S.mulAdd(p, z, S3);
  p = S.mulAdd(p, z, S2);
  p = S.mulAdd(p, z, S1);
  p = S.mulAdd(p, z, S0);
  return S.mul(x, p);
}

function cosKernel(x: Scalar): Scalar {
  const z = S.mul(x, x);
  let p = C5;
  p = S.mulAdd(p, z, C4);
  p = S.mulAdd(p, z, C3);
  p = S.mulAdd(p, z, C2);
  p = S.mulAdd(p, z, C1);
  return S.mulAdd(p, z, C0);
}

/** `atan(t)` for `t` in `[0, 1]`. */
function atanKernel(t: Scalar): Scalar {
  const z = S.mul(t, t);
  let p = A7;
  p = S.mulAdd(p, z, A6);
  p = S.mulAdd(p, z, A5);
  p = S.mulAdd(p, z, A4);
  p = S.mulAdd(p, z, A3);
  p = S.mulAdd(p, z, A2);
  p = S.mulAdd(p, z, A1);
  p = S.mulAdd(p, z, A0);
  return S.mul(t, p);
}

/* ------------------------------------------------------------------ *
 * Public API
 * ------------------------------------------------------------------ */

/**
 * Wrap an angle into `(-π, π]`.
 *
 * Cheap and exact — used by the integrator every tick so body angles never
 * grow without bound (which also keeps the fixed-point backend in range).
 */
export function normalizeAngle(a: Scalar): Scalar {
  if (a >= -S.PI && a <= S.PI) return a; // fast path: already normalized
  const n = S.toInt(S.mulAdd(a, S.INV_TWO_PI, S.HALF));
  return a - S.mul(S.TWO_PI, S.fromInt(n));
}

/**
 * Reduce `a` to `r ∈ [-π/4, π/4]` and return the quadrant index `0..3`.
 * `out[0]` receives `r`.
 */
function reduce(a: Scalar, out: Scalar[]): number {
  const k = S.toInt(S.mulAdd(a, INV_HALF_PI, S.HALF));
  out[0] = a - S.mul(S.HALF_PI, S.fromInt(k));
  return k & 3;
}

const _r: Scalar[] = [S.ZERO];

/** Deterministic sine. */
export function sin(a: Scalar): Scalar {
  const q = reduce(normalizeAngle(a), _r);
  const r = _r[0]!;
  switch (q) {
    case 0:
      return sinKernel(r);
    case 1:
      return cosKernel(r);
    case 2:
      return -sinKernel(r);
    default:
      return -cosKernel(r);
  }
}

/** Deterministic cosine. */
export function cos(a: Scalar): Scalar {
  const q = reduce(normalizeAngle(a), _r);
  const r = _r[0]!;
  switch (q) {
    case 0:
      return cosKernel(r);
    case 1:
      return -sinKernel(r);
    case 2:
      return -cosKernel(r);
    default:
      return sinKernel(r);
  }
}

/**
 * Sine **and** cosine in one pass — roughly twice as fast as calling both,
 * and the form every rotation matrix needs.
 *
 * @param a   angle in radians
 * @param out 2-element array receiving `[sin, cos]`
 */
export function sinCos(a: Scalar, out: Scalar[]): void {
  const q = reduce(normalizeAngle(a), _r);
  const r = _r[0]!;
  const s = sinKernel(r);
  const c = cosKernel(r);
  switch (q) {
    case 0:
      out[0] = s;
      out[1] = c;
      break;
    case 1:
      out[0] = c;
      out[1] = -s;
      break;
    case 2:
      out[0] = -s;
      out[1] = -c;
      break;
    default:
      out[0] = -c;
      out[1] = s;
      break;
  }
}

/** Deterministic tangent. Returns a large finite value at the poles. */
export function tan(a: Scalar): Scalar {
  const c = cos(a);
  if (S.abs(c) < S.EPSILON) return c < S.ZERO ? S.MIN_VALUE : S.MAX_VALUE;
  return S.div(sin(a), c);
}

/** Deterministic arctangent, result in `[-π/2, π/2]`. */
export function atan(t: Scalar): Scalar {
  const neg = t < S.ZERO;
  const x = neg ? -t : t;
  const r = x <= S.ONE ? atanKernel(x) : S.HALF_PI - atanKernel(S.inv(x));
  return neg ? -r : r;
}

/**
 * Deterministic two-argument arctangent, result in `(-π, π]`.
 * `atan2(0, 0)` is defined to be `0`.
 */
export function atan2(y: Scalar, x: Scalar): Scalar {
  const ax = S.abs(x);
  const ay = S.abs(y);
  if (ax < S.EPSILON && ay < S.EPSILON) return S.ZERO;

  let r: Scalar;
  if (ax >= ay) {
    r = atanKernel(S.div(ay, ax)); // in [0, π/4]
  } else {
    r = S.HALF_PI - atanKernel(S.div(ax, ay)); // in [π/4, π/2]
  }
  if (x < S.ZERO) r = S.PI - r;
  return y < S.ZERO ? -r : r;
}

/** Deterministic arcsine, input clamped to `[-1, 1]`. */
export function asin(v: Scalar): Scalar {
  const x = S.clamp(v, S.NEG_ONE, S.ONE);
  const c = S.sqrt(S.ONE - S.mul(x, x));
  return atan2(x, c);
}

/** Deterministic arccosine, input clamped to `[-1, 1]`. */
export function acos(v: Scalar): Scalar {
  const x = S.clamp(v, S.NEG_ONE, S.ONE);
  const s = S.sqrt(S.ONE - S.mul(x, x));
  return atan2(s, x);
}
