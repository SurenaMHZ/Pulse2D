/**
 * @module collision/BroadPhase
 *
 * Wraps the {@link DynamicTree} and produces the set of **new overlapping
 * pairs** each step.
 *
 * Only proxies that actually moved are re-queried — a resting stack costs
 * nothing. Pairs are collected, sorted and de-duplicated so the contact list
 * is built in a canonical order regardless of how the tree happens to be
 * shaped, which is a hard requirement for lockstep determinism.
 */

import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { AABB } from './AABB.js';
import { DynamicTree } from './DynamicTree.js';
import { AABB_MARGIN } from './../util/settings.js';

/** Callback invoked once per newly overlapping pair. */
export type PairCallback = (userDataA: number, userDataB: number) => void;

export class BroadPhase {
  readonly tree: DynamicTree;

  /** Proxy ids whose AABB changed since the last update. */
  private moved: number[] = [];
  private movedSet: Set<number> = new Set();

  /** Pair buffer, stored as interleaved `[a, b, a, b, …]` fixture ids. */
  private pairs: number[] = [];

  /** Reused sort buffer for the pair keys. */
  private _keys = new Float64Array(0);
  private _queryAABB = new AABB();
  private _fatAABB = new AABB();
  private _zero = Vec2.zero();

  constructor(capacity = 64) {
    this.tree = new DynamicTree(capacity);
  }

  /** Number of proxies currently registered. */
  get proxyCount(): number {
    return this.tree.proxyCount;
  }

  /** Insert a proxy and mark it for the next pair pass. */
  createProxy(aabb: AABB, userData: number): number {
    this._fatAABB.copyFrom(aabb).expand(AABB_MARGIN);
    const id = this.tree.createProxy(this._fatAABB, userData);
    this.markMoved(id);
    return id;
  }

  /** Remove a proxy. */
  destroyProxy(id: number): void {
    this.unmarkMoved(id);
    this.tree.destroyProxy(id);
  }

  /**
   * Update a proxy's AABB.
   * @param displacement this step's motion, used to extend the fat AABB
   */
  moveProxy(id: number, aabb: AABB, displacement: Vec2 = this._zero): void {
    if (this.tree.moveProxy(id, aabb, AABB_MARGIN, displacement)) this.markMoved(id);
  }

  /** Force a proxy to be re-tested next update (e.g. after a filter change). */
  touchProxy(id: number): void {
    this.markMoved(id);
  }

  private markMoved(id: number): void {
    if (!this.movedSet.has(id)) {
      this.movedSet.add(id);
      this.moved.push(id);
    }
  }

  private unmarkMoved(id: number): void {
    if (this.movedSet.delete(id)) {
      const i = this.moved.indexOf(id);
      if (i >= 0) this.moved.splice(i, 1);
    }
  }

  /**
   * Find every new overlapping pair and hand it to `cb`.
   *
   * Each unordered pair is reported at most once per update: a pair found from
   * both sides is filtered by the `a < b` rule, and the sorted buffer removes
   * any remaining duplicates.
   */
  updatePairs(cb: PairCallback): void {
    if (this.moved.length === 0) return;
    this.pairs.length = 0;

    for (let i = 0; i < this.moved.length; i++) {
      const queryId = this.moved[i]!;
      // The proxy may have been destroyed since it was marked.
      if (this.tree.getUserData(queryId) < 0) continue;
      const queryData = this.tree.getUserData(queryId);
      this.tree.getAABB(this._queryAABB, queryId);

      this.tree.query(this._queryAABB, (proxyId, userData) => {
        if (proxyId === queryId) return true;
        // Both moved: keep only one ordering so the pair is not duplicated.
        if (this.movedSet.has(proxyId) && proxyId > queryId) return true;
        const a = queryData < userData ? queryData : userData;
        const b = queryData < userData ? userData : queryData;
        this.pairs.push(a, b);
        return true;
      });
    }

    this.moved.length = 0;
    this.movedSet.clear();
    if (this.pairs.length === 0) return;

    /*
     * Canonical order + de-duplication.
     *
     * Pairs are packed as a single key per entry — `a * 2²⁰ + b`, with both
     * ids below 2²⁰ — so ordering by that one number is exactly ordering by
     * `(a, b)`. That lets a plain numeric sort over a reused Float64Array
     * replace an index array plus a comparator closure, both of which were
     * being allocated every step.
     */
    const n = this.pairs.length / 2;
    if (this._keys.length < n) {
      this._keys = new Float64Array(1 << (32 - Math.clz32(Math.max(n, 64) - 1)));
    }
    const keys = this._keys;
    const p = this.pairs;
    for (let i = 0; i < n; i++) keys[i] = p[i * 2]! * 0x100000 + p[i * 2 + 1]!;

    // subarray().sort() is a numeric sort with no comparator allocation.
    const view = keys.subarray(0, n);
    view.sort();

    let last = -1;
    for (let i = 0; i < n; i++) {
      const key = view[i]!;
      if (key === last) continue;
      last = key;
      cb(Math.floor(key / 0x100000), key % 0x100000);
    }
    this.pairs.length = 0;
  }

  /** Direct AABB query against the tree. */
  query(aabb: AABB, cb: (userData: number) => boolean): void {
    this.tree.query(aabb, (_id, userData) => cb(userData));
  }

  /** Direct point query. */
  queryPoint(p: Vec2, cb: (userData: number) => boolean): void {
    this.tree.queryPoint(p, (_id, userData) => cb(userData));
  }

  /** Direct ray cast; see {@link DynamicTree#rayCast}. */
  rayCast(
    p1: Vec2,
    p2: Vec2,
    maxFraction: Scalar,
    cb: (userData: number, p1: Vec2, p2: Vec2, maxFraction: Scalar) => Scalar,
  ): void {
    this.tree.rayCast(p1, p2, maxFraction, (_id, userData, a, b, f) => cb(userData, a, b, f));
  }

  /**
   * Force every proxy to be re-tested on the next {@link updatePairs}.
   *
   * Needed after a snapshot restore: the tree still holds the *fat* AABBs
   * computed along the timeline that was just discarded, and `moveProxy`
   * skips re-insertion whenever the new tight AABB happens to fit inside the
   * stale fat one. Two peers that reached the same state by different routes
   * would then own differently-shaped trees and discover pairs on different
   * ticks — a desync that only shows up seconds later.
   *
   * Rebuilding re-seeds every fat AABB from the current transform, so the
   * broad phase becomes a pure function of the world state again.
   */
  rebuild(fixtures: Iterable<{ proxyId: number; aabb: AABB; id: number }>): void {
    this.tree.clear();
    this.moved.length = 0;
    this.movedSet.clear();
    this.pairs.length = 0;
    for (const f of fixtures) {
      if (f.proxyId < 0) continue;
      this._fatAABB.copyFrom(f.aabb).expand(AABB_MARGIN);
      f.proxyId = this.tree.createProxy(this._fatAABB, f.id);
      this.markMoved(f.proxyId);
    }
  }

  /** Drop everything. */
  clear(): void {
    this.tree.clear();
    this.moved.length = 0;
    this.movedSet.clear();
    this.pairs.length = 0;
  }

  /** Tree quality metric — see {@link DynamicTree#getQuality}. */
  get quality(): number {
    return this.tree.getQuality();
  }
}

