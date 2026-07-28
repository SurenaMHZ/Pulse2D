/**
 * @module math/scalar.f64
 *
 * Deterministic IEEE-754 double-precision scalar backend.
 *
 * ## Why this is deterministic
 *
 * IEEE-754 mandates *correctly rounded* results for `+`, `-`, `*`, `/` and
 * `sqrt`. Every conforming CPU / JS engine therefore produces the exact same
 * bit pattern for those five operations. This backend is built **only** out of
 * them, so a simulation replays bit-identically on x86, ARM, WASM and RISC-V.
 *
 * What is *not* deterministic and is consequently banned engine-wide:
 * `Math.sin`, `Math.cos`, `Math.tan`, `Math.atan2`, `Math.exp`, `Math.pow`,
 * `Math.log`, `Math.hypot`, `Math.cbrt`, `**` with a fractional exponent and
 * `Math.random`. Transcendental replacements live in {@link module:math/trig},
 * randomness in {@link module:math/rng}.
 *
 * @see docs/DETERMINISM.md
 */

/** A scalar value. In this backend the encoding is the identity. */
export type Scalar = number;

/** Human readable backend id, embedded in snapshot headers. */
export const BACKEND = 'f64' as const;

/** `true` when the backend stores fixed-point integers. */
export const IS_FIXED = false;

/** Number of fractional bits (0 => continuous). */
export const FRACTION_BITS = 0;

/* ------------------------------------------------------------------ *
 * Constants
 * ------------------------------------------------------------------ */

export const ZERO: Scalar = 0;
export const ONE: Scalar = 1;
export const TWO: Scalar = 2;
export const HALF: Scalar = 0.5;
export const NEG_ONE: Scalar = -1;

export const PI: Scalar = 3.141592653589793;
export const TWO_PI: Scalar = 6.283185307179586;
export const HALF_PI: Scalar = 1.5707963267948966;
export const INV_TWO_PI: Scalar = 0.15915494309189535;

/** Smallest meaningful magnitude; values below are treated as zero. */
export const EPSILON: Scalar = 1e-9;
/** `EPSILON²`, used to compare squared lengths without a sqrt. */
export const EPSILON_SQ: Scalar = 1e-18;
/** Practical upper bound used for "infinite" separations. */
export const MAX_VALUE: Scalar = 1e18;
export const MIN_VALUE: Scalar = -1e18;
/** Resolution of the representation (1 ulp at 1.0 for f64). */
export const RESOLUTION: Scalar = 2.220446049250313e-16;

/* ------------------------------------------------------------------ *
 * Conversions
 * ------------------------------------------------------------------ */

/** Convert a plain JS float into the backend encoding. */
export function fromFloat(x: number): Scalar {
  return x;
}

/** Convert a backend value back into a plain JS float. */
export function toFloat(x: Scalar): number {
  return x;
}

/** Convert a (safe) integer into the backend encoding. */
export function fromInt(i: number): Scalar {
  return i;
}

/** Truncate towards -Infinity and return a plain integer. */
export function toInt(x: Scalar): number {
  return Math.floor(x);
}

/* ------------------------------------------------------------------ *
 * Arithmetic
 *
 * `+`, `-`, unary `-`, `<`, `<=`, `>`, `>=`, `===` work natively on both
 * backends and are used directly throughout the engine for speed.
 * ------------------------------------------------------------------ */

/** `a * b` */
export function mul(a: Scalar, b: Scalar): Scalar {
  return a * b;
}

/** `a / b` */
export function div(a: Scalar, b: Scalar): Scalar {
  return a / b;
}

/** `1 / a` */
export function inv(a: Scalar): Scalar {
  return 1 / a;
}

/** `sqrt(a)`, correctly rounded. */
export function sqrt(a: Scalar): Scalar {
  return Math.sqrt(a);
}

/** `a * b + c` computed with two rounding steps (never uses FMA). */
export function mulAdd(a: Scalar, b: Scalar, c: Scalar): Scalar {
  return a * b + c;
}

/** `a / 2` */
export function half(a: Scalar): Scalar {
  return a * 0.5;
}

/** `a * i` where `i` is a small plain integer. */
export function mulInt(a: Scalar, i: number): Scalar {
  return a * i;
}

/** `a / i` where `i` is a small positive plain integer. */
export function divInt(a: Scalar, i: number): Scalar {
  return a / i;
}

/** Square. */
export function sq(a: Scalar): Scalar {
  return a * a;
}

/** Absolute value. */
export function abs(a: Scalar): Scalar {
  return a < 0 ? -a : a;
}

/** Branch-deterministic minimum (NaN never occurs in engine state). */
export function min(a: Scalar, b: Scalar): Scalar {
  return a < b ? a : b;
}

/** Branch-deterministic maximum. */
export function max(a: Scalar, b: Scalar): Scalar {
  return a > b ? a : b;
}

/** Clamp `x` into `[lo, hi]`. */
export function clamp(x: Scalar, lo: Scalar, hi: Scalar): Scalar {
  return x < lo ? lo : x > hi ? hi : x;
}

/** `-1`, `0` or `1`. */
export function sign(a: Scalar): Scalar {
  return a > 0 ? ONE : a < 0 ? NEG_ONE : ZERO;
}

/** Linear interpolation, `t` in `[0, 1]`. */
export function lerp(a: Scalar, b: Scalar, t: Scalar): Scalar {
  return a + (b - a) * t;
}

/** Round to the nearest representable value (identity here). */
export function quantize(a: Scalar): Scalar {
  return a;
}

/**
 * Raw bits of the value, as two 32-bit integers `[hi, lo]`.
 * Used by the checksum layer so desyncs are detected on the exact bits.
 */
const _buf = new ArrayBuffer(8);
const _f64 = new Float64Array(_buf);
const _u32 = new Uint32Array(_buf);

export function bitsOf(a: Scalar, out: [number, number]): [number, number] {
  _f64[0] = a;
  out[0] = _u32[0] as number;
  out[1] = _u32[1] as number;
  return out;
}
