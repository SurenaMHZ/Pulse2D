/**
 * @module math/scalar.fixed
 *
 * Deterministic **Q16.16 fixed-point** scalar backend.
 *
 * A value is stored as a 32-bit signed integer equal to `round(x * 65536)`.
 * Every operation ends with `| 0`, so overflow wraps in the single way the
 * ECMAScript spec defines — identical on every CPU, browser and runtime.
 *
 * | property   | value                            |
 * |------------|----------------------------------|
 * | range      | `[-32768, +32767.99998]`         |
 * | resolution | `1 / 65536 ≈ 1.526e-5`           |
 * | exact mul  | operands with `|x| < 1024`       |
 *
 * Use this backend when you must be bit-identical across *architectures with
 * different floating point flush-to-zero behaviour* (some mobile GPUs / older
 * JIT tiers) or when you want to be certain no `x87` 80-bit intermediate can
 * ever leak in. Otherwise prefer `scalar.f64`, which is faster and far more
 * precise. The API is intentionally identical, so the engine compiles against
 * either one unchanged.
 *
 * @see docs/DETERMINISM.md
 */

/** A scalar value, encoded as a Q16.16 int32. */
export type Scalar = number;

export const BACKEND = 'q16.16' as const;
export const IS_FIXED = true;
export const FRACTION_BITS = 16;

/** `1.0` in raw units. */
const SHIFT = 16;
const ONE_RAW = 1 << SHIFT; // 65536
const MASK = ONE_RAW - 1;

/* ------------------------------------------------------------------ *
 * Constants
 * ------------------------------------------------------------------ */

export const ZERO: Scalar = 0;
export const ONE: Scalar = ONE_RAW;
export const TWO: Scalar = ONE_RAW * 2;
export const HALF: Scalar = ONE_RAW >> 1;
export const NEG_ONE: Scalar = -ONE_RAW;

export const PI: Scalar = 205887; // 3.14159265 * 65536
export const TWO_PI: Scalar = 411775;
export const HALF_PI: Scalar = 102944;
export const INV_TWO_PI: Scalar = 10430;

/** One raw unit — the smallest representable positive value. */
export const EPSILON: Scalar = 1;
export const EPSILON_SQ: Scalar = 1;
export const MAX_VALUE: Scalar = 0x7fffffff;
export const MIN_VALUE: Scalar = -0x80000000;
export const RESOLUTION: Scalar = 1;

/* ------------------------------------------------------------------ *
 * Conversions
 * ------------------------------------------------------------------ */

/** Convert a plain JS float into Q16.16 (round-half-away-from-zero). */
export function fromFloat(x: number): Scalar {
  return (x >= 0 ? Math.floor(x * ONE_RAW + 0.5) : Math.ceil(x * ONE_RAW - 0.5)) | 0;
}

/** Convert Q16.16 back into a plain JS float. */
export function toFloat(x: Scalar): number {
  return x / ONE_RAW;
}

/** Convert an integer into Q16.16. */
export function fromInt(i: number): Scalar {
  return (i << SHIFT) | 0;
}

/** Truncate towards -Infinity and return a plain integer. */
export function toInt(x: Scalar): number {
  return x >> SHIFT;
}

/* ------------------------------------------------------------------ *
 * Arithmetic
 * ------------------------------------------------------------------ */

/**
 * `a * b`.
 *
 * The 64-bit intermediate product is emulated by splitting both operands into
 * 16-bit halves, so the result is *exact* (truncated towards -Infinity) and
 * never relies on a double being able to hold 2^62.
 */
export function mul(a: Scalar, b: Scalar): Scalar {
  const ah = a >> SHIFT;
  const al = a & MASK;
  const bh = b >> SHIFT;
  const bl = b & MASK;
  // (ah·2^16 + al)(bh·2^16 + bl) / 2^16
  return (ah * b + al * bh + ((al * bl) >>> SHIFT)) | 0;
}

/** `a / b`. */
export function div(a: Scalar, b: Scalar): Scalar {
  return Math.floor((a * ONE_RAW) / b) | 0;
}

/** `1 / a`. */
export function inv(a: Scalar): Scalar {
  return Math.floor(4294967296 / a) | 0; // 2^32 / a  ==  (1<<32)/a
}

/**
 * `sqrt(a)`.
 *
 * `Math.sqrt` is correctly rounded by IEEE-754, and `a * 65536 < 2^47` is
 * exactly representable, so flooring the result is deterministic everywhere.
 */
export function sqrt(a: Scalar): Scalar {
  if (a <= 0) return 0;
  return Math.floor(Math.sqrt(a * ONE_RAW)) | 0;
}

/** `a * b + c`. */
export function mulAdd(a: Scalar, b: Scalar, c: Scalar): Scalar {
  return (mul(a, b) + c) | 0;
}

/** `a / 2` (arithmetic shift, rounds towards -Infinity). */
export function half(a: Scalar): Scalar {
  return a >> 1;
}

/** `a * i` where `i` is a small plain integer. */
export function mulInt(a: Scalar, i: number): Scalar {
  return (a * i) | 0;
}

/** `a / i` where `i` is a small positive plain integer. */
export function divInt(a: Scalar, i: number): Scalar {
  return Math.floor(a / i) | 0;
}

/** Square. */
export function sq(a: Scalar): Scalar {
  return mul(a, a);
}

export function abs(a: Scalar): Scalar {
  return a < 0 ? -a | 0 : a;
}

export function min(a: Scalar, b: Scalar): Scalar {
  return a < b ? a : b;
}

export function max(a: Scalar, b: Scalar): Scalar {
  return a > b ? a : b;
}

export function clamp(x: Scalar, lo: Scalar, hi: Scalar): Scalar {
  return x < lo ? lo : x > hi ? hi : x;
}

export function sign(a: Scalar): Scalar {
  return a > 0 ? ONE : a < 0 ? NEG_ONE : ZERO;
}

export function lerp(a: Scalar, b: Scalar, t: Scalar): Scalar {
  return (a + mul(b - a, t)) | 0;
}

/** Already quantised — returned unchanged. */
export function quantize(a: Scalar): Scalar {
  return a | 0;
}

/** Raw bits (the value itself, plus a zero high word). */
export function bitsOf(a: Scalar, out: [number, number]): [number, number] {
  out[0] = a | 0;
  out[1] = 0;
  return out;
}
