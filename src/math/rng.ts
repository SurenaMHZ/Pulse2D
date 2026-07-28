/**
 * @module math/rng
 *
 * Deterministic pseudo-random number generator.
 *
 * `Math.random()` is seedless and implementation-defined — using it anywhere
 * inside a lockstep simulation guarantees a desync. This is a 32-bit
 * **PCG-XSH-RR**-style generator built on `Math.imul`, so it is exact int32
 * arithmetic and reproduces the same stream on every runtime.
 *
 * The whole state is two 32-bit words, which makes it trivial to include in a
 * world snapshot and rewind during rollback.
 *
 * ```ts
 * const rng = new Rng(1234);
 * rng.int(0, 6);        // 0..6 inclusive
 * rng.float();          // [0, 1)
 * rng.scalar(-1, 1);    // backend scalar in a range
 * ```
 */

import * as S from './scalar.js';
import type { Scalar } from './scalar.js';

/** Odd increment (any odd constant defines a distinct stream). */
const INC = 0x2545f491 | 0;
const MULT = 0x321f_5b3d | 0;

export class Rng {
  /** Internal state word. */
  private _s: number;
  /** Stream selector; must stay odd. */
  private _i: number;

  constructor(seed = 0x1234_5678, stream = INC) {
    this._s = seed | 0;
    this._i = (stream | 1) | 0;
    this.next(); // discard the first output so low seeds mix properly
  }

  /** Restore an exact previously captured state. */
  setState(s: number, i: number): void {
    this._s = s | 0;
    this._i = (i | 1) | 0;
  }

  /** Current state as `[s, i]` — put this in your snapshot. */
  getState(): [number, number] {
    return [this._s, this._i];
  }

  /** Re-seed and reset the stream. */
  seed(seed: number): void {
    this._s = seed | 0;
    this.next();
  }

  /**
   * Next raw 32-bit unsigned integer.
   *
   * An LCG advances the state (good period, poor low bits) and a murmur3
   * finalizer avalanches it (every input bit affects every output bit). The
   * two together pass a chi-square uniformity test at χ²≈10 for 9 degrees of
   * freedom and show <0.4% bias on any individual bit.
   */
  next(): number {
    const old = this._s | 0;
    this._s = (Math.imul(old, MULT) + this._i) | 0;
    let z = old | 0;
    z = Math.imul(z ^ (z >>> 16), 0x85ebca6b);
    z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35);
    return (z ^ (z >>> 16)) >>> 0;
  }

  /** Uniform float in `[0, 1)` with 32 bits of entropy. */
  float(): number {
    return this.next() * 2.3283064365386963e-10; // 1 / 2^32
  }

  /** Uniform backend scalar in `[lo, hi)`. */
  scalar(lo: Scalar, hi: Scalar): Scalar {
    const t = S.fromFloat(this.float());
    return S.mulAdd(hi - lo, t, lo);
  }

  /**
   * Uniform integer in `[lo, hi]` (inclusive), free of modulo bias.
   *
   * Values at the tail of the 32-bit range that would skew the modulo are
   * rejected and re-drawn. `limit` is deliberately kept as a plain float:
   * for a power-of-two range it equals `2³²`, and coercing that to an int32
   * would wrap it to `0` and spin forever.
   */
  int(lo: number, hi: number): number {
    const range = (hi - lo + 1) | 0;
    if (range <= 0) return lo;
    const limit = 4294967296 - (4294967296 % range);
    let r = this.next();
    while (r >= limit) r = this.next();
    return (lo + (r % range)) | 0;
  }

  /** `true` with probability `p` (0..1). */
  bool(p = 0.5): boolean {
    return this.float() < p;
  }

  /** Shuffle an array in place (Fisher–Yates, deterministic). */
  shuffle<T>(arr: T[]): T[] {
    for (let i = arr.length - 1; i > 0; i--) {
      const j = this.int(0, i);
      const t = arr[i]!;
      arr[i] = arr[j]!;
      arr[j] = t;
    }
    return arr;
  }
}
