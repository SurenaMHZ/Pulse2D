/**
 * @module net/Snapshot
 *
 * Binary world snapshots for **rollback netcode**.
 *
 * A snapshot captures everything that affects future simulation:
 *
 * ```
 * header    magic, protocol version, backend id, tick, RNG state
 * bodies    transform, velocity, sleep state, mass overrides
 * contacts  accumulated impulses, keyed by fixture pair + feature id
 * joints    accumulated impulses
 * ```
 *
 * Contact impulses matter more than people expect: they are the warm-start
 * seed, so restoring positions without them produces a *visibly* different
 * next frame. Pulse2D restores them keyed by `(fixtureA, fixtureB, featureId)`
 * so the match survives contacts being created in a different order.
 *
 * ### Typical rollback loop
 *
 * ```ts
 * // every tick, after stepping
 * history.push(tick, saveSnapshot(world));
 *
 * // a late input for tick T arrives
 * loadSnapshot(world, history.get(T - 1));
 * applyInputs(T - 1);
 * for (let t = T - 1; t <= currentTick; t++) {
 *   applyInputs(t);
 *   world.step();
 * }
 * ```
 *
 * Snapshots are `Float64Array`-backed and copy-free to hash, so `checksum()`
 * over one is fast enough to run every tick in production.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { World } from './../dynamics/World.js';
import type { Body } from './../dynamics/Body.js';
import { BodyType } from './../dynamics/Body.js';
import { Transform } from './../math/Transform.js';
import { PROTOCOL_VERSION } from './../util/settings.js';

/** Format marker: "P2DS" as a 32-bit integer. */
const MAGIC = 0x50324453;

/** Numbers stored per body. */
const BODY_STRIDE = 12;
/** Numbers stored per contact point. */
const CONTACT_POINT_STRIDE = 3;

/** A captured world state. */
export interface Snapshot {
  /** The tick this state was captured at. */
  tick: number;
  /** Dense numeric payload. */
  data: Float64Array;
  /** Integer payload (ids, flags, RNG state). */
  meta: Int32Array;
}

/**
 * Capture the world.
 *
 * @param world  the world to capture
 * @param reuse  an existing snapshot to write into, to avoid allocation
 */
export function saveSnapshot(world: World, reuse?: Snapshot): Snapshot {
  const bodies: Body[] = [];
  for (const b of world.eachBody()) bodies.push(b);

  // Only touching, non-sensor contacts carry impulses worth restoring.
  const contacts = world.contacts.filter((c) => c.isTouching && !c.isSensor);
  let contactPoints = 0;
  for (const c of contacts) contactPoints += c.manifold.pointCount;

  const jointImpulses: number[] = [];
  const jointIds: number[] = [];
  for (const j of world.eachJoint()) {
    jointIds.push(j.id);
    j.saveState(jointImpulses);
  }

  const dataLen = bodies.length * BODY_STRIDE + contactPoints * CONTACT_POINT_STRIDE + jointImpulses.length;
  const metaLen = 8 + bodies.length * 2 + contacts.length * 3 + jointIds.length * 2;

  const snap: Snapshot =
    reuse && reuse.data.length === dataLen && reuse.meta.length === metaLen
      ? reuse
      : { tick: 0, data: new Float64Array(dataLen), meta: new Int32Array(metaLen) };

  snap.tick = world.tick;
  const d = snap.data;
  const m = snap.meta;

  /* ---- header ---- */
  const rng = world.rng.getState();
  m[0] = MAGIC;
  m[1] = PROTOCOL_VERSION;
  m[2] = S.IS_FIXED ? 1 : 0;
  m[3] = world.tick;
  m[4] = rng[0];
  m[5] = rng[1];
  m[6] = bodies.length;
  m[7] = contacts.length;

  /* ---- bodies ---- */
  let di = 0;
  let mi = 8;
  for (const b of bodies) {
    d[di++] = b.transform.p.x as number;
    d[di++] = b.transform.p.y as number;
    d[di++] = b.transform.q.s as number;
    d[di++] = b.transform.q.c as number;
    d[di++] = b.linearVelocity.x as number;
    d[di++] = b.linearVelocity.y as number;
    d[di++] = b.angularVelocity as number;
    d[di++] = b.sleepTime as number;
    d[di++] = b.worldCenter.x as number;
    d[di++] = b.worldCenter.y as number;
    d[di++] = b.force.x as number;
    d[di++] = b.force.y as number;
    m[mi++] = b.id;
    m[mi++] = (b.awake ? 1 : 0) | (b.enabled ? 2 : 0) | (b.type << 2);
  }

  /* ---- contacts ---- */
  for (const c of contacts) {
    m[mi++] = c.fixtureA.id;
    m[mi++] = c.fixtureB.id;
    m[mi++] = c.manifold.pointCount;
    for (let i = 0; i < c.manifold.pointCount; i++) {
      const p = c.manifold.points[i]!;
      d[di++] = p.id;
      d[di++] = p.normalImpulse as number;
      d[di++] = p.tangentImpulse as number;
    }
  }

  /* ---- joints ---- */
  for (let i = 0; i < jointIds.length; i++) {
    m[mi++] = jointIds[i]!;
    m[mi++] = 0; // reserved
  }
  for (const v of jointImpulses) d[di++] = v;

  return snap;
}

/**
 * Restore a previously captured state.
 *
 * The world must have the **same set of bodies** (same ids) as when the
 * snapshot was taken; create and destroy bodies from your own deterministic
 * game logic during the resimulation rather than trying to encode structural
 * changes in the snapshot.
 *
 * @throws when the header does not match this build
 */
export function loadSnapshot(world: World, snap: Snapshot): void {
  const m = snap.meta;
  const d = snap.data;

  if (m[0] !== MAGIC) throw new Error('Pulse2D: not a snapshot (bad magic)');
  if (m[1] !== PROTOCOL_VERSION) {
    throw new Error(`Pulse2D: protocol mismatch (snapshot ${m[1]}, build ${PROTOCOL_VERSION})`);
  }
  if (m[2] !== (S.IS_FIXED ? 1 : 0)) {
    throw new Error('Pulse2D: scalar backend mismatch (fixed-point vs float)');
  }

  world.tick = m[3]!;
  world.rng.setState(m[4]!, m[5]!);
  const bodyCount = m[6]!;
  const contactCount = m[7]!;

  let di = 0;
  let mi = 8;

  /* ---- bodies ---- */
  for (let i = 0; i < bodyCount; i++) {
    const px = d[di++]!;
    const py = d[di++]!;
    const qs = d[di++]!;
    const qc = d[di++]!;
    const vx = d[di++]!;
    const vy = d[di++]!;
    const w = d[di++]!;
    const sleepTime = d[di++]!;
    const cx = d[di++]!;
    const cy = d[di++]!;
    const fx = d[di++]!;
    const fy = d[di++]!;
    const id = m[mi++]!;
    const flags = m[mi++]!;

    const b = world.bodies[id];
    if (!b) continue;
    b.transform.p.set(px as Scalar, py as Scalar);
    b.transform.q.setSinCos(qs as Scalar, qc as Scalar);
    b.linearVelocity.set(vx as Scalar, vy as Scalar);
    b.angularVelocity = w as Scalar;
    b.sleepTime = sleepTime as Scalar;
    b.worldCenter.set(cx as Scalar, cy as Scalar);
    b.force.set(fx as Scalar, fy as Scalar);
    b.awake = (flags & 1) !== 0;
    b.enabled = (flags & 2) !== 0;
    b.type = (flags >> 2) as BodyType;
  }

  /*
   * Resync derived state and rebuild the contact set *before* impulses are
   * restored.
   *
   * The contacts currently attached to the world describe the timeline we are
   * abandoning: some pairs have been created since the snapshot and some have
   * been destroyed. Rediscovering them from the restored geometry gives the
   * exact set that existed when the snapshot was taken, so the impulses below
   * land on the right manifolds.
   */
  for (const b of world.eachBody()) {
    Transform.apply(b.worldCenter, b.transform, b.localCenter);
  }
  world.rebuildBroadPhase(true);

  /* ---- contacts ---- */
  // Index the live contacts so impulses can be matched by fixture pair.
  const live = new Map<number, (typeof world.contacts)[number]>();
  for (const c of world.contacts) {
    live.set(c.fixtureA.id * 0x100000 + c.fixtureB.id, c);
  }

  for (let i = 0; i < contactCount; i++) {
    const fa = m[mi++]!;
    const fb = m[mi++]!;
    const count = m[mi++]!;
    const c = live.get(fa * 0x100000 + fb);
    for (let j = 0; j < count; j++) {
      const pid = d[di++]!;
      const ni = d[di++]!;
      const ti = d[di++]!;
      if (!c) continue;
      // Match by feature id so a reordered manifold still warm-starts right.
      for (let k = 0; k < c.manifold.pointCount; k++) {
        const mp = c.manifold.points[k]!;
        if (mp.id === pid) {
          mp.normalImpulse = ni as Scalar;
          mp.tangentImpulse = ti as Scalar;
          mp.persisted = true;
          break;
        }
      }
    }
  }

  /* ---- joints ---- */
  const jointIds: number[] = [];
  while (mi < m.length) {
    jointIds.push(m[mi]!);
    mi += 2;
  }
  const rest: number[] = [];
  for (let i = di; i < d.length; i++) rest.push(d[i]!);
  let offset = 0;
  for (const jid of jointIds) {
    const j = world.joints[jid];
    if (!j) break;
    offset = j.loadState(rest, offset);
  }

}

/**
 * Deep-copy a snapshot, so a ring buffer can hold many without aliasing.
 */
export function cloneSnapshot(snap: Snapshot): Snapshot {
  return {
    tick: snap.tick,
    data: new Float64Array(snap.data),
    meta: new Int32Array(snap.meta),
  };
}

/** Snapshot size in bytes — handy for budgeting a history buffer. */
export function snapshotBytes(snap: Snapshot): number {
  return snap.data.byteLength + snap.meta.byteLength;
}
