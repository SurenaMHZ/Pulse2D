/**
 * @module collision/DynamicTree
 *
 * **Broad phase** — a dynamic AABB tree (bounding volume hierarchy).
 *
 * Queries and ray casts run in `O(log n)`; insertion picks the branch that
 * grows the tree's total perimeter least (the surface-area heuristic) and
 * rebalances with AVL-style rotations, so quality stays high without ever
 * rebuilding from scratch.
 *
 * ### Layout
 *
 * Nodes live in **flat parallel arrays**, not objects:
 *
 * ```
 * bounds  : Float64Array(4n)   lower.x lower.y upper.x upper.y
 * meta    : Int32Array(4n)     parent | child1 | child2 | height
 * userData: Int32Array(n)      proxy payload (a fixture id)
 * ```
 *
 * That is one contiguous allocation per field, no pointer chasing, and a
 * traversal touches ~4× fewer cache lines than an object graph would. Growing
 * the tree copies the arrays — amortised `O(1)`, and it never happens in the
 * steady state because capacity doubles.
 *
 * ### Determinism
 *
 * Node ids are handed out from a free list in a fixed order, and every
 * comparison is on backend scalars. Two peers that insert the same proxies in
 * the same order build a bit-identical tree, so queries return results in the
 * same order too — which matters, because contact ordering feeds straight into
 * the solver.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { AABB } from './AABB.js';

/** Sentinel for "no node". */
export const NULL_NODE = -1;

/** Callback for {@link DynamicTree#query}; return `false` to stop early. */
export type QueryCallback = (proxyId: number, userData: number) => boolean;

/**
 * Callback for {@link DynamicTree#rayCast}.
 *
 * Return the new maximum fraction to continue (return `0` to stop, or the
 * incoming `maxFraction` to keep searching unchanged).
 */
export type RayCastCallback = (
  proxyId: number,
  userData: number,
  p1: Vec2,
  p2: Vec2,
  maxFraction: Scalar,
) => Scalar;

export class DynamicTree {
  /** `[lx, ly, ux, uy]` per node. */
  private bounds: Float64Array;
  /** `[parent, child1, child2, height]` per node. */
  private meta: Int32Array;
  /** Caller payload per node (a fixture id). */
  private data: Int32Array;

  private root = NULL_NODE;
  private capacity: number;
  /** Total allocated nodes (leaves *and* internal parents). */
  private count = 0;
  /** Live leaves, i.e. what the caller thinks of as proxies. */
  private leaves = 0;
  private freeList = 0;

  /**
   * Number of live proxies.
   *
   * A BVH over `n` leaves also holds `n - 1` internal nodes; only the leaves
   * are proxies, so this counts those.
   */
  get proxyCount(): number {
    return this.leaves;
  }

  /** Total nodes in the tree, including internal ones. Diagnostic. */
  get nodeCount(): number {
    return this.count;
  }

  constructor(capacity = 64) {
    this.capacity = Math.max(16, capacity);
    this.bounds = new Float64Array(this.capacity * 4);
    this.meta = new Int32Array(this.capacity * 4);
    this.data = new Int32Array(this.capacity);
    this.initFreeList(0);
  }

  /** Chain the unused slots `[from, capacity)` into the free list. */
  private initFreeList(from: number): void {
    for (let i = from; i < this.capacity; i++) {
      this.meta[i * 4] = i + 1 < this.capacity ? i + 1 : NULL_NODE; // parent = next free
      this.meta[i * 4 + 3] = -1; // height = -1 marks a free node
    }
    this.freeList = from;
  }

  private grow(): void {
    const old = this.capacity;
    this.capacity *= 2;
    const b = new Float64Array(this.capacity * 4);
    b.set(this.bounds);
    this.bounds = b;
    const m = new Int32Array(this.capacity * 4);
    m.set(this.meta);
    this.meta = m;
    const d = new Int32Array(this.capacity);
    d.set(this.data);
    this.data = d;
    this.initFreeList(old);
  }

  private allocNode(): number {
    if (this.freeList === NULL_NODE) this.grow();
    const id = this.freeList;
    this.freeList = this.meta[id * 4]!;
    const m = id * 4;
    this.meta[m] = NULL_NODE; // parent
    this.meta[m + 1] = NULL_NODE; // child1
    this.meta[m + 2] = NULL_NODE; // child2
    this.meta[m + 3] = 0; // height
    this.data[id] = -1;
    this.count++;
    return id;
  }

  private freeNode(id: number): void {
    this.meta[id * 4] = this.freeList;
    this.meta[id * 4 + 3] = -1;
    this.freeList = id;
    this.count--;
  }

  /* ------------------------- accessors ------------------------- */

  /** `true` when the node is a leaf (holds a proxy rather than children). */
  isLeaf(id: number): boolean {
    return this.meta[id * 4 + 1] === NULL_NODE;
  }

  /** Payload stored with a proxy. */
  getUserData(id: number): number {
    return this.data[id]!;
  }

  /** Copy a proxy's fat AABB into `out`. */
  getAABB(out: AABB, id: number): AABB {
    const b = id * 4;
    out.lower.set(this.bounds[b]!, this.bounds[b + 1]!);
    out.upper.set(this.bounds[b + 2]!, this.bounds[b + 3]!);
    return out;
  }

  /** Tree height; `0` for an empty or single-node tree. */
  getHeight(): number {
    return this.root === NULL_NODE ? 0 : this.meta[this.root * 4 + 3]!;
  }

  /**
   * Sum of internal-node perimeters divided by the root perimeter — the
   * standard quality metric. Values near 1 are ideal; above ~4 means the tree
   * has degenerated and queries are getting slow.
   */
  getQuality(): number {
    if (this.root === NULL_NODE) return 0;
    const rootPerim = this.perimeterOf(this.root);
    if (rootPerim === 0) return 0;
    let total = 0;
    for (let i = 0; i < this.capacity; i++) {
      if (this.meta[i * 4 + 3]! < 1) continue; // free or leaf
      total += this.perimeterOf(i);
    }
    return total / rootPerim;
  }

  private perimeterOf(id: number): number {
    const b = id * 4;
    return 2 * (this.bounds[b + 2]! - this.bounds[b]! + (this.bounds[b + 3]! - this.bounds[b + 1]!));
  }

  /* --------------------------- mutation ------------------------- */

  /**
   * Insert a proxy.
   * @param aabb     the tight AABB; it is stored as given (pad it yourself)
   * @param userData payload returned by queries
   * @returns the proxy id
   */
  createProxy(aabb: AABB, userData: number): number {
    const id = this.allocNode();
    const b = id * 4;
    this.bounds[b] = aabb.lower.x as number;
    this.bounds[b + 1] = aabb.lower.y as number;
    this.bounds[b + 2] = aabb.upper.x as number;
    this.bounds[b + 3] = aabb.upper.y as number;
    this.meta[b + 3] = 0;
    this.data[id] = userData;
    this.leaves++;
    this.insertLeaf(id);
    return id;
  }

  /** Remove a proxy. */
  destroyProxy(id: number): void {
    this.removeLeaf(id);
    this.freeNode(id);
    this.leaves--;
  }

  /**
   * Move a proxy.
   *
   * If the new (tight) AABB is still inside the stored fat AABB nothing
   * happens and `false` is returned — the common case for a slowly moving
   * body, and the reason the broad phase costs almost nothing at rest.
   *
   * @returns `true` when the proxy was actually re-inserted
   */
  moveProxy(id: number, aabb: AABB, margin: Scalar, displacement: Vec2): boolean {
    const b = id * 4;
    if (
      (aabb.lower.x as number) >= this.bounds[b]! &&
      (aabb.lower.y as number) >= this.bounds[b + 1]! &&
      (aabb.upper.x as number) <= this.bounds[b + 2]! &&
      (aabb.upper.y as number) <= this.bounds[b + 3]!
    ) {
      return false;
    }

    this.removeLeaf(id);

    // Fatten, then extend along the direction of travel so a fast body's
    // proxy already covers where it is going.
    let lx = (aabb.lower.x - margin) as number;
    let ly = (aabb.lower.y - margin) as number;
    let ux = (aabb.upper.x + margin) as number;
    let uy = (aabb.upper.y + margin) as number;

    const dx = S.mulInt(displacement.x, 2) as number;
    const dy = S.mulInt(displacement.y, 2) as number;
    if (dx < 0) lx += dx;
    else ux += dx;
    if (dy < 0) ly += dy;
    else uy += dy;

    this.bounds[b] = lx;
    this.bounds[b + 1] = ly;
    this.bounds[b + 2] = ux;
    this.bounds[b + 3] = uy;
    this.meta[b + 3] = 0;

    this.insertLeaf(id);
    return true;
  }

  /** Remove every proxy. */
  clear(): void {
    this.root = NULL_NODE;
    this.count = 0;
    this.leaves = 0;
    this.initFreeList(0);
  }

  /* ------------------------ tree surgery ------------------------ */

  private insertLeaf(leaf: number): void {
    if (this.root === NULL_NODE) {
      this.root = leaf;
      this.meta[leaf * 4] = NULL_NODE;
      return;
    }

    const lb = leaf * 4;
    const llx = this.bounds[lb]!;
    const lly = this.bounds[lb + 1]!;
    const lux = this.bounds[lb + 2]!;
    const luy = this.bounds[lb + 3]!;

    // 1. Descend to the best sibling using the surface-area heuristic.
    let index = this.root;
    while (!this.isLeaf(index)) {
      const ib = index * 4;
      const child1 = this.meta[ib + 1]!;
      const child2 = this.meta[ib + 2]!;

      const area = this.perimeterOf(index);
      const combined = this.combinedPerimeter(index, llx, lly, lux, luy);

      // Cost of making a new parent here.
      const cost = 2 * combined;
      // Minimum cost of pushing the leaf further down.
      const inheritance = 2 * (combined - area);

      const cost1 = this.descendCost(child1, llx, lly, lux, luy) + inheritance;
      const cost2 = this.descendCost(child2, llx, lly, lux, luy) + inheritance;

      if (cost < cost1 && cost < cost2) break;
      index = cost1 < cost2 ? child1 : child2;
    }

    // 2. Splice a new parent above the chosen sibling.
    const sibling = index;
    const oldParent = this.meta[sibling * 4]!;
    const newParent = this.allocNode();
    const nb = newParent * 4;
    this.meta[nb] = oldParent;
    this.data[newParent] = -1;
    this.combineInto(newParent, sibling, leaf);
    this.meta[nb + 3] = this.meta[sibling * 4 + 3]! + 1;

    if (oldParent !== NULL_NODE) {
      if (this.meta[oldParent * 4 + 1] === sibling) this.meta[oldParent * 4 + 1] = newParent;
      else this.meta[oldParent * 4 + 2] = newParent;
    } else {
      this.root = newParent;
    }
    this.meta[nb + 1] = sibling;
    this.meta[nb + 2] = leaf;
    this.meta[sibling * 4] = newParent;
    this.meta[leaf * 4] = newParent;

    // 3. Walk back up fixing bounds and heights.
    this.refitAncestors(this.meta[leaf * 4]!);
  }

  /** Lower bound on the cost of inserting under `child`. */
  private descendCost(child: number, lx: number, ly: number, ux: number, uy: number): number {
    const combined = this.combinedPerimeter(child, lx, ly, ux, uy);
    if (this.isLeaf(child)) return combined;
    return combined - this.perimeterOf(child);
  }

  private combinedPerimeter(id: number, lx: number, ly: number, ux: number, uy: number): number {
    const b = id * 4;
    const nlx = Math.min(this.bounds[b]!, lx);
    const nly = Math.min(this.bounds[b + 1]!, ly);
    const nux = Math.max(this.bounds[b + 2]!, ux);
    const nuy = Math.max(this.bounds[b + 3]!, uy);
    return 2 * (nux - nlx + (nuy - nly));
  }

  private combineInto(dest: number, a: number, b: number): void {
    const d = dest * 4;
    const ab = a * 4;
    const bb = b * 4;
    this.bounds[d] = Math.min(this.bounds[ab]!, this.bounds[bb]!);
    this.bounds[d + 1] = Math.min(this.bounds[ab + 1]!, this.bounds[bb + 1]!);
    this.bounds[d + 2] = Math.max(this.bounds[ab + 2]!, this.bounds[bb + 2]!);
    this.bounds[d + 3] = Math.max(this.bounds[ab + 3]!, this.bounds[bb + 3]!);
  }

  private removeLeaf(leaf: number): void {
    if (leaf === this.root) {
      this.root = NULL_NODE;
      return;
    }
    const parent = this.meta[leaf * 4]!;
    const grandParent = this.meta[parent * 4]!;
    const sibling =
      this.meta[parent * 4 + 1] === leaf ? this.meta[parent * 4 + 2]! : this.meta[parent * 4 + 1]!;

    if (grandParent !== NULL_NODE) {
      if (this.meta[grandParent * 4 + 1] === parent) this.meta[grandParent * 4 + 1] = sibling;
      else this.meta[grandParent * 4 + 2] = sibling;
      this.meta[sibling * 4] = grandParent;
      this.freeNode(parent);
      this.refitAncestors(grandParent);
    } else {
      this.root = sibling;
      this.meta[sibling * 4] = NULL_NODE;
      this.freeNode(parent);
    }
  }

  /** Recompute bounds/heights from `start` to the root, balancing on the way. */
  private refitAncestors(start: number): void {
    let index = start;
    while (index !== NULL_NODE) {
      index = this.balance(index);
      const ib = index * 4;
      const c1 = this.meta[ib + 1]!;
      const c2 = this.meta[ib + 2]!;
      this.combineInto(index, c1, c2);
      this.meta[ib + 3] = 1 + Math.max(this.meta[c1 * 4 + 3]!, this.meta[c2 * 4 + 3]!);
      index = this.meta[ib]!;
    }
  }

  /**
   * AVL rotation: if one subtree is more than one level deeper than its
   * sibling, promote its taller grandchild. Keeps the tree height `O(log n)`
   * with a constant amount of work per insert.
   *
   * @returns the node now occupying `iA`'s position
   */
  private balance(iA: number): number {
    const a = iA * 4;
    if (this.meta[a + 1] === NULL_NODE || this.meta[a + 3]! < 2) return iA;

    const iB = this.meta[a + 1]!;
    const iC = this.meta[a + 2]!;
    const b = iB * 4;
    const c = iC * 4;
    const balance = this.meta[c + 3]! - this.meta[b + 3]!;

    if (balance > 1) return this.rotate(iA, iC, iB); // C is taller: promote it
    if (balance < -1) return this.rotate(iA, iB, iC); // B is taller: promote it
    return iA;
  }

  /** Promote `iUp` above `iA`, keeping `iOther` as `iA`'s remaining child. */
  private rotate(iA: number, iUp: number, iOther: number): number {
    const a = iA * 4;
    const u = iUp * 4;
    const iF = this.meta[u + 1]!;
    const iG = this.meta[u + 2]!;

    // Swap A and Up.
    this.meta[u + 1] = iA;
    this.meta[u] = this.meta[a]!;
    this.meta[a] = iUp;

    // Re-attach Up's old parent.
    const up = this.meta[u]!;
    if (up !== NULL_NODE) {
      if (this.meta[up * 4 + 1] === iA) this.meta[up * 4 + 1] = iUp;
      else this.meta[up * 4 + 2] = iUp;
    } else {
      this.root = iUp;
    }

    // Keep the taller grandchild high.
    const hF = this.meta[iF * 4 + 3]!;
    const hG = this.meta[iG * 4 + 3]!;
    const keep = hF > hG ? iF : iG;
    const drop = hF > hG ? iG : iF;

    this.meta[u + 2] = keep;
    this.meta[keep * 4] = iUp;

    // Attach the shorter one under A, in the slot Up used to occupy.
    if (this.meta[a + 1] === iUp) this.meta[a + 1] = drop;
    else this.meta[a + 2] = drop;
    this.meta[drop * 4] = iA;

    this.combineInto(iA, iOther, drop);
    this.combineInto(iUp, iA, keep);
    this.meta[a + 3] = 1 + Math.max(this.meta[iOther * 4 + 3]!, this.meta[drop * 4 + 3]!);
    this.meta[u + 3] = 1 + Math.max(this.meta[a + 3]!, this.meta[keep * 4 + 3]!);

    return iUp;
  }

  /* ---------------------------- queries -------------------------- */

  /** Reusable traversal stack — avoids allocating on every query. */
  private stack = new Int32Array(256);

  private pushStack(sp: number, value: number): number {
    if (sp === this.stack.length) {
      const s = new Int32Array(this.stack.length * 2);
      s.set(this.stack);
      this.stack = s;
    }
    this.stack[sp] = value;
    return sp + 1;
  }

  /**
   * Report every proxy whose fat AABB overlaps `aabb`.
   * The callback may return `false` to abort the traversal.
   */
  query(aabb: AABB, cb: QueryCallback): void {
    if (this.root === NULL_NODE) return;
    const lx = aabb.lower.x as number;
    const ly = aabb.lower.y as number;
    const ux = aabb.upper.x as number;
    const uy = aabb.upper.y as number;

    let sp = this.pushStack(0, this.root);
    while (sp > 0) {
      const id = this.stack[--sp]!;
      if (id === NULL_NODE) continue;
      const b = id * 4;
      // AABB overlap test, inlined.
      if (this.bounds[b]! > ux || this.bounds[b + 2]! < lx) continue;
      if (this.bounds[b + 1]! > uy || this.bounds[b + 3]! < ly) continue;

      if (this.isLeaf(id)) {
        if (!cb(id, this.data[id]!)) return;
      } else {
        sp = this.pushStack(sp, this.meta[b + 1]!);
        sp = this.pushStack(sp, this.meta[b + 2]!);
      }
    }
  }

  /** Report every proxy whose fat AABB contains `p`. */
  queryPoint(p: Vec2, cb: QueryCallback): void {
    if (this.root === NULL_NODE) return;
    const px = p.x as number;
    const py = p.y as number;
    let sp = this.pushStack(0, this.root);
    while (sp > 0) {
      const id = this.stack[--sp]!;
      if (id === NULL_NODE) continue;
      const b = id * 4;
      if (px < this.bounds[b]! || px > this.bounds[b + 2]!) continue;
      if (py < this.bounds[b + 1]! || py > this.bounds[b + 3]!) continue;
      if (this.isLeaf(id)) {
        if (!cb(id, this.data[id]!)) return;
      } else {
        sp = this.pushStack(sp, this.meta[b + 1]!);
        sp = this.pushStack(sp, this.meta[b + 2]!);
      }
    }
  }

  private _segLower = Vec2.zero();
  private _segUpper = Vec2.zero();
  private _nodeAABB = new AABB();
  private _dir = Vec2.zero();

  /**
   * Ray cast against the tree.
   *
   * The callback shrinks `maxFraction` as closer hits are found, so the
   * traversal prunes aggressively and typically visits only a handful of
   * nodes even in a large world.
   */
  rayCast(p1: Vec2, p2: Vec2, maxFraction: Scalar, cb: RayCastCallback): void {
    if (this.root === NULL_NODE) return;
    let maxF = maxFraction;
    Vec2.subTo(this._dir, p2, p1);

    let sp = this.pushStack(0, this.root);
    while (sp > 0) {
      const id = this.stack[--sp]!;
      if (id === NULL_NODE) continue;

      // Prune with the current segment's AABB before the exact slab test.
      const ex = S.mulAdd(this._dir.x, maxF, p1.x);
      const ey = S.mulAdd(this._dir.y, maxF, p1.y);
      this._segLower.set(S.min(p1.x, ex), S.min(p1.y, ey));
      this._segUpper.set(S.max(p1.x, ex), S.max(p1.y, ey));

      const b = id * 4;
      if (this.bounds[b]! > (this._segUpper.x as number)) continue;
      if (this.bounds[b + 2]! < (this._segLower.x as number)) continue;
      if (this.bounds[b + 1]! > (this._segUpper.y as number)) continue;
      if (this.bounds[b + 3]! < (this._segLower.y as number)) continue;

      this.getAABB(this._nodeAABB, id);
      if (this._nodeAABB.rayCast(p1, this._dir, maxF) < S.ZERO) continue;

      if (this.isLeaf(id)) {
        const value = cb(id, this.data[id]!, p1, p2, maxF);
        if (value === S.ZERO) return; // caller is done
        if (value > S.ZERO) maxF = value;
      } else {
        sp = this.pushStack(sp, this.meta[b + 1]!);
        sp = this.pushStack(sp, this.meta[b + 2]!);
      }
    }
  }

  /**
   * Structural self-check, for tests and debugging.
   * @returns `null` when the tree is sound, otherwise a description
   */
  validate(): string | null {
    if (this.root === NULL_NODE) return this.leaves === 0 ? null : 'root is null but leaves > 0';
    if (this.meta[this.root * 4] !== NULL_NODE) return 'root has a parent';

    const visit = (id: number): string | null => {
      if (id === NULL_NODE) return null;
      const b = id * 4;
      const c1 = this.meta[b + 1]!;
      const c2 = this.meta[b + 2]!;
      if (this.isLeaf(id)) {
        if (this.meta[b + 3] !== 0) return `leaf ${id} has height ${this.meta[b + 3]}`;
        return null;
      }
      if (this.meta[c1 * 4] !== id) return `child1 of ${id} has the wrong parent`;
      if (this.meta[c2 * 4] !== id) return `child2 of ${id} has the wrong parent`;
      const h = 1 + Math.max(this.meta[c1 * 4 + 3]!, this.meta[c2 * 4 + 3]!);
      if (this.meta[b + 3] !== h) return `node ${id} height ${this.meta[b + 3]} != ${h}`;
      // parent must contain both children
      for (const c of [c1, c2]) {
        const cb2 = c * 4;
        if (
          this.bounds[b]! > this.bounds[cb2]! + 1e-9 ||
          this.bounds[b + 1]! > this.bounds[cb2 + 1]! + 1e-9 ||
          this.bounds[b + 2]! < this.bounds[cb2 + 2]! - 1e-9 ||
          this.bounds[b + 3]! < this.bounds[cb2 + 3]! - 1e-9
        ) {
          return `node ${id} does not contain child ${c}`;
        }
      }
      return visit(c1) ?? visit(c2);
    };
    return visit(this.root);
  }
}
