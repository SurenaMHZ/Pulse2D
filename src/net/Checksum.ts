/**
 * @module net/Checksum
 *
 * Desync detection.
 *
 * Every peer hashes its world state each tick and exchanges the (tiny) digest.
 * The moment two digests differ you know the exact tick where the simulations
 * diverged, which turns "the game desyncs sometimes" into a reproducible bug.
 *
 * The hash is **FNV-1a over the raw IEEE-754 bits**, not over rounded decimal
 * strings: a difference in the last mantissa bit — exactly the kind of drift
 * you are hunting — changes the digest.
 */

import * as S from './../math/scalar.js';
import type { World } from './../dynamics/World.js';
import type { Snapshot } from './Snapshot.js';

const FNV_PRIME = 0x01000193;
const FNV_OFFSET = 0x811c9dc5;

/** Incremental FNV-1a hasher over 32-bit words. */
export class Hasher {
  private h = FNV_OFFSET;

  /** Mix in a 32-bit integer. */
  int(v: number): this {
    let x = v | 0;
    for (let i = 0; i < 4; i++) {
      this.h = Math.imul(this.h ^ (x & 0xff), FNV_PRIME);
      x >>>= 8;
    }
    return this;
  }

  private static _buf = new ArrayBuffer(8);
  private static _f64 = new Float64Array(Hasher._buf);
  private static _u32 = new Uint32Array(Hasher._buf);

  /**
   * Mix in a float by its raw bits.
   *
   * `-0` is normalised to `+0` first: the two are `===` in JS but have
   * different bit patterns, and a solver can legitimately produce either, so
   * hashing them differently would cause false desync reports.
   */
  float(v: number): this {
    Hasher._f64[0] = v === 0 ? 0 : v;
    return this.int(Hasher._u32[0]!).int(Hasher._u32[1]!);
  }

  /** Mix in a backend scalar. */
  scalar(v: number): this {
    return S.IS_FIXED ? this.int(v) : this.float(v);
  }

  /** Current digest as an unsigned 32-bit integer. */
  digest(): number {
    return this.h >>> 0;
  }

  /** Current digest as an 8-character hex string. */
  hex(): string {
    return (this.h >>> 0).toString(16).padStart(8, '0');
  }

  /** Restart. */
  reset(): this {
    this.h = FNV_OFFSET;
    return this;
  }
}

/**
 * Hash a world's simulation-relevant state.
 *
 * Only what affects the future is included — no ids of destroyed objects, no
 * profiling counters, no user data.
 *
 * @param world     the world to hash
 * @param positionsOnly cheaper variant that skips impulses; still catches
 *                      almost every desync but a tick or two later
 */
export function checksumWorld(world: World, positionsOnly = false): number {
  const h = new Hasher();
  h.int(world.tick);
  const rng = world.rng.getState();
  h.int(rng[0]).int(rng[1]);

  for (const b of world.eachBody()) {
    h.int(b.id);
    h.scalar(b.transform.p.x as number);
    h.scalar(b.transform.p.y as number);
    h.scalar(b.transform.q.s as number);
    h.scalar(b.transform.q.c as number);
    h.scalar(b.linearVelocity.x as number);
    h.scalar(b.linearVelocity.y as number);
    h.scalar(b.angularVelocity as number);
    h.int(b.awake ? 1 : 0);
  }

  if (!positionsOnly) {
    for (const c of world.contacts) {
      if (!c.isTouching || c.isSensor) continue;
      h.int(c.fixtureA.id).int(c.fixtureB.id);
      for (let i = 0; i < c.manifold.pointCount; i++) {
        const p = c.manifold.points[i]!;
        h.int(p.id);
        h.scalar(p.normalImpulse as number);
        h.scalar(p.tangentImpulse as number);
      }
    }
  }
  return h.digest();
}

/** Hash a snapshot buffer directly — cheaper than walking the object graph. */
export function checksumSnapshot(snap: Snapshot): number {
  const h = new Hasher();
  for (let i = 0; i < snap.meta.length; i++) h.int(snap.meta[i]!);
  for (let i = 0; i < snap.data.length; i++) h.float(snap.data[i]!);
  return h.digest();
}

/**
 * A rolling log of per-tick checksums.
 *
 * Keep one on every peer, exchange the digests periodically, and
 * {@link ChecksumLog#findDivergence} tells you the first tick that differs.
 */
export class ChecksumLog {
  private readonly ticks: Int32Array;
  private readonly sums: Uint32Array;
  private head = 0;
  private count = 0;

  constructor(readonly capacity = 256) {
    this.ticks = new Int32Array(capacity);
    this.sums = new Uint32Array(capacity);
  }

  /** Record one tick. */
  record(tick: number, checksum: number): void {
    this.ticks[this.head] = tick;
    this.sums[this.head] = checksum >>> 0;
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }

  /** Convenience: hash the world and record it. */
  recordWorld(world: World): number {
    const sum = checksumWorld(world);
    this.record(world.tick, sum);
    return sum;
  }

  /** Look up a recorded checksum, or `undefined`. */
  get(tick: number): number | undefined {
    for (let i = 0; i < this.count; i++) {
      const idx = (this.head - 1 - i + this.capacity * 2) % this.capacity;
      if (this.ticks[idx] === tick) return this.sums[idx];
    }
    return undefined;
  }

  /**
   * Compare against a remote peer's log.
   * @returns the earliest tick where the two disagree, or `-1` when they match
   */
  findDivergence(remote: Map<number, number>): number {
    let earliest = -1;
    for (let i = 0; i < this.count; i++) {
      const idx = (this.head - 1 - i + this.capacity * 2) % this.capacity;
      const tick = this.ticks[idx]!;
      const mine = this.sums[idx]!;
      const theirs = remote.get(tick);
      if (theirs === undefined) continue;
      if ((theirs >>> 0) !== mine && (earliest === -1 || tick < earliest)) earliest = tick;
    }
    return earliest;
  }

  /** Export as a plain map, ready to be serialised and sent. */
  toMap(): Map<number, number> {
    const out = new Map<number, number>();
    for (let i = 0; i < this.count; i++) {
      const idx = (this.head - 1 - i + this.capacity * 2) % this.capacity;
      out.set(this.ticks[idx]!, this.sums[idx]!);
    }
    return out;
  }

  clear(): void {
    this.head = 0;
    this.count = 0;
  }
}
