/**
 * @module collision/Collide
 *
 * **Narrow phase** — turns a pair of overlapping shapes into a
 * {@link Manifold}.
 *
 * Every primitive is viewed as a *rounded convex polygon* (see
 * {@link module:collision/ConvexProxy}), so one algorithm covers all ten shape
 * combinations:
 *
 * 1. **circle vs circle** and **anything vs circle** take a dedicated
 *    single-point path — a circle has no face to clip against;
 * 2. everything else runs **SAT** over the edge normals of both proxies,
 *    picks the axis of least penetration, and **clips the incident edge
 *    against the reference face** to yield one or two contact points.
 *
 * ### Speculative contacts
 *
 * Contacts are generated slightly *before* the surfaces touch. The solver then
 * applies exactly the impulse needed to land on the surface instead of letting
 * the shapes sink in and pushing them apart afterwards. This removes the
 * classic penetration jitter from stacks and stops moderately fast bodies from
 * tunnelling, with no continuous-collision machinery in the common path.
 *
 * Every routine is allocation-free: scratch state lives in module singletons
 * and the manifold is written in place.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Rot } from './../math/Rot.js';
import { Transform } from './../math/Transform.js';
import { Manifold, makeID } from './Manifold.js';
import { ShapeType } from './Shape.js';
import type { Shape } from './Shape.js';
import { getProxy } from './ConvexProxy.js';
import type { ConvexProxy } from './ConvexProxy.js';
import { SPECULATIVE_DISTANCE, LINEAR_SLOP } from './../util/settings.js';
import type { Segment } from './shapes/Segment.js';

/* --------------------------- scratch --------------------------- */

const _xfBA = new Transform();
const _xfAB = new Transform();
const _xfRel = new Transform();
const _tmp = Vec2.zero();
const _tmp2 = Vec2.zero();
const _localNormal = Vec2.zero();
const _v11 = Vec2.zero();
const _v12 = Vec2.zero();
const _i1 = Vec2.zero();
const _i2 = Vec2.zero();
const _c1 = Vec2.zero();
const _c2 = Vec2.zero();
const _tangent = Vec2.zero();
const _idxA = [0];
const _idxB = [0];

/** Distance at which a speculative contact is still created. */
const SPECULATIVE = SPECULATIVE_DISTANCE;

/* ------------------------------------------------------------------ *
 * Circle paths
 * ------------------------------------------------------------------ */

/** Circle vs circle: one point, normal along the line of centres. */
export function collideCircles(
  manifold: Manifold,
  shapeA: Shape,
  xfA: Transform,
  shapeB: Shape,
  xfB: Transform,
): void {
  manifold.pointCount = 0;
  const pA = getProxy(shapeA);
  const pB = getProxy(shapeB);

  Transform.mulTTo(_xfBA, xfA, xfB);
  Transform.apply(_tmp, _xfBA, pB.vertices[0]!); // B's centre in A's frame
  const cA = pA.vertices[0]!;

  const dx = _tmp.x - cA.x;
  const dy = _tmp.y - cA.y;
  const distSq = S.mulAdd(dx, dx, S.mul(dy, dy));
  const rA = pA.radius;
  const rB = pB.radius;
  const radius = rA + rB;
  const limit = radius + SPECULATIVE;
  if (distSq > S.mul(limit, limit)) return;

  const dist = S.sqrt(distSq);
  let nx: Scalar;
  let ny: Scalar;
  if (dist < S.EPSILON) {
    // Perfectly coincident centres — pick a fixed axis so the result stays
    // reproducible instead of depending on rounding noise.
    nx = S.ZERO;
    ny = S.ONE;
  } else {
    const inv = S.inv(dist);
    nx = S.mul(dx, inv);
    ny = S.mul(dy, inv);
  }

  _localNormal.set(nx, ny);
  // Midpoint between the two surfaces.
  const mx = S.half(S.mulAdd(nx, rA, cA.x) + S.mulAdd(nx, -rB, _tmp.x));
  const my = S.half(S.mulAdd(ny, rA, cA.y) + S.mulAdd(ny, -rB, _tmp.y));

  Rot.rotate(manifold.normal, xfA.q, _localNormal);
  const mp = manifold.points[0];
  Transform.apply(mp.point, xfA, _tmp2.set(mx, my));
  mp.separation = dist - radius;
  mp.id = 0;
  manifold.pointCount = 1;
}

/**
 * Polygon / capsule / segment vs circle.
 *
 * Finds the closest feature of the hull to the circle's centre — a face or a
 * vertex — and emits a single point.
 */
export function collidePolygonCircle(
  manifold: Manifold,
  shapeA: Shape,
  xfA: Transform,
  shapeB: Shape,
  xfB: Transform,
): void {
  manifold.pointCount = 0;
  const pA = getProxy(shapeA);
  const pB = getProxy(shapeB);

  Transform.mulTTo(_xfBA, xfA, xfB);
  Transform.apply(_tmp, _xfBA, pB.vertices[0]!); // circle centre, A's frame

  const radius = pA.radius + pB.radius;
  const limit = radius + SPECULATIVE;

  // Face the centre is furthest outside of.
  let faceIndex = 0;
  let separation = S.MIN_VALUE;
  for (let i = 0; i < pA.normals.length; i++) {
    const n = pA.normals[i]!;
    const v = pA.vertices[i]!;
    const s = S.mulAdd(n.x, _tmp.x - v.x, S.mul(n.y, _tmp.y - v.y));
    if (s > separation) {
      separation = s;
      faceIndex = i;
    }
  }
  if (separation > limit) return;

  const i1 = faceIndex;
  const i2 = i1 + 1 < pA.count ? i1 + 1 : 0;
  const v1 = pA.vertices[i1]!;
  const v2 = pA.vertices[i2]!;

  let px: Scalar;
  let py: Scalar;

  if (separation < S.ZERO && pA.count > 2) {
    // Centre is strictly inside a solid polygon: use the face normal.
    const n = pA.normals[faceIndex]!;
    _localNormal.copyFrom(n);
    px = S.mulAdd(n.x, -separation, _tmp.x);
    py = S.mulAdd(n.y, -separation, _tmp.y);
  } else {
    // Voronoi region test along the face.
    const u1 = S.mulAdd(_tmp.x - v1.x, v2.x - v1.x, S.mul(_tmp.y - v1.y, v2.y - v1.y));
    const u2 = S.mulAdd(_tmp.x - v2.x, v1.x - v2.x, S.mul(_tmp.y - v2.y, v1.y - v2.y));
    if (u1 <= S.ZERO) {
      px = v1.x;
      py = v1.y;
    } else if (u2 <= S.ZERO) {
      px = v2.x;
      py = v2.y;
    } else {
      const n = pA.normals[i1]!;
      const d = S.mulAdd(_tmp.x - v1.x, n.x, S.mul(_tmp.y - v1.y, n.y));
      px = S.mulAdd(n.x, -d, _tmp.x);
      py = S.mulAdd(n.y, -d, _tmp.y);
    }
    _localNormal.set(_tmp.x - px, _tmp.y - py);
    if (_localNormal.normalize() === S.ZERO) _localNormal.copyFrom(pA.normals[i1]!);
    separation = S.mulAdd(_tmp.x - px, _localNormal.x, S.mul(_tmp.y - py, _localNormal.y));
  }

  if (separation > limit) return;

  const mx = S.half(
    S.mulAdd(_localNormal.x, pA.radius, px) + S.mulAdd(_localNormal.x, -pB.radius, _tmp.x),
  );
  const my = S.half(
    S.mulAdd(_localNormal.y, pA.radius, py) + S.mulAdd(_localNormal.y, -pB.radius, _tmp.y),
  );

  Rot.rotate(manifold.normal, xfA.q, _localNormal);
  const mp = manifold.points[0];
  Transform.apply(mp.point, xfA, _tmp2.set(mx, my));
  mp.separation = separation - radius;
  mp.id = makeID(faceIndex, 0, 0, 0);
  manifold.pointCount = 1;
}

/* ------------------------------------------------------------------ *
 * SAT + clipping
 * ------------------------------------------------------------------ */

/**
 * Largest separation of `pB` from any face of `pA`.
 *
 * @param xf       `pB` expressed in `pA`'s frame
 * @param outIndex receives the winning face index
 */
function maxSeparation(pA: ConvexProxy, pB: ConvexProxy, xf: Transform, outIndex: number[]): Scalar {
  let best = S.MIN_VALUE;
  let bestIndex = 0;
  for (let i = 0; i < pA.normals.length; i++) {
    const n = pA.normals[i]!;
    const v = pA.vertices[i]!;
    // Support point of B along -n, i.e. the vertex deepest into this face.
    Rot.rotateT(_tmp, xf.q, _tmp2.set(-n.x, -n.y));
    let si = 0;
    let sv = S.mulAdd(pB.vertices[0]!.x, _tmp.x, S.mul(pB.vertices[0]!.y, _tmp.y));
    for (let j = 1; j < pB.count; j++) {
      const w = pB.vertices[j]!;
      const d = S.mulAdd(w.x, _tmp.x, S.mul(w.y, _tmp.y));
      if (d > sv) {
        sv = d;
        si = j;
      }
    }
    Transform.apply(_tmp2, xf, pB.vertices[si]!);
    const s = S.mulAdd(n.x, _tmp2.x - v.x, S.mul(n.y, _tmp2.y - v.y));
    if (s > best) {
      best = s;
      bestIndex = i;
    }
  }
  outIndex[0] = bestIndex;
  return best;
}

/**
 * General convex-vs-convex collision (polygon, capsule and segment in any
 * combination).
 */
export function collidePolygons(
  manifold: Manifold,
  shapeA: Shape,
  xfA: Transform,
  shapeB: Shape,
  xfB: Transform,
): void {
  manifold.pointCount = 0;
  const pA = getProxy(shapeA);
  const pB = getProxy(shapeB);
  const radius = pA.radius + pB.radius;
  const limit = radius + SPECULATIVE;

  Transform.mulTTo(_xfBA, xfA, xfB); // B in A's frame
  const sepA = maxSeparation(pA, pB, _xfBA, _idxA);
  if (sepA > limit) return;

  Transform.mulTTo(_xfAB, xfB, xfA); // A in B's frame
  const sepB = maxSeparation(pB, pA, _xfAB, _idxB);
  if (sepB > limit) return;

  /*
   * Choose the reference face. The small bias makes the choice sticky when
   * the two separations are nearly equal, so the manifold (and therefore the
   * warm-start ids) do not flip-flop between frames.
   */
  const bias = S.fromFloat(0.1 * 0.005);
  const flip = sepB > sepA + bias;

  const refProxy = flip ? pB : pA;
  const incProxy = flip ? pA : pB;
  const refIndex = flip ? _idxB[0]! : _idxA[0]!;
  const refXf = flip ? xfB : xfA;
  const incXf = flip ? xfA : xfB;

  const refN = refProxy.normals[refIndex]!;
  const r1 = refIndex;
  const r2 = r1 + 1 < refProxy.count ? r1 + 1 : 0;
  _v11.copyFrom(refProxy.vertices[r1]!);
  _v12.copyFrom(refProxy.vertices[r2]!);

  // Incident edge = the edge of `inc` most anti-parallel to the reference
  // normal, brought into the reference frame.
  Transform.mulTTo(_xfRel, refXf, incXf);
  Rot.rotateT(_tmp, _xfRel.q, refN);

  let incIndex = 0;
  let minDot = S.MAX_VALUE;
  for (let i = 0; i < incProxy.normals.length; i++) {
    const n = incProxy.normals[i]!;
    const d = S.mulAdd(n.x, _tmp.x, S.mul(n.y, _tmp.y));
    if (d < minDot) {
      minDot = d;
      incIndex = i;
    }
  }
  const j1 = incIndex;
  const j2 = j1 + 1 < incProxy.count ? j1 + 1 : 0;
  Transform.apply(_i1, _xfRel, incProxy.vertices[j1]!);
  Transform.apply(_i2, _xfRel, incProxy.vertices[j2]!);

  // Tangent along the reference face.
  _tangent.set(_v12.x - _v11.x, _v12.y - _v11.y);
  const faceLen = _tangent.normalize();
  if (faceLen === S.ZERO) return;

  // Clip the incident segment to the reference face's side planes, working in
  // the 1-D tangent coordinate measured from _v11.
  let t1 = S.mulAdd(_i1.x - _v11.x, _tangent.x, S.mul(_i1.y - _v11.y, _tangent.y));
  let t2 = S.mulAdd(_i2.x - _v11.x, _tangent.x, S.mul(_i2.y - _v11.y, _tangent.y));

  let idA = j1;
  let idB = j2;
  _c1.copyFrom(_i1);
  _c2.copyFrom(_i2);
  if (t1 > t2) {
    const t = t1;
    t1 = t2;
    t2 = t;
    _c1.copyFrom(_i2);
    _c2.copyFrom(_i1);
    idA = j2;
    idB = j1;
  }

  const lo = S.ZERO;
  const hi = faceLen;
  if (t2 < lo || t1 > hi) return; // no tangential overlap

  const span = t2 - t1;
  const p1x = _c1.x;
  const p1y = _c1.y;
  const p2x = _c2.x;
  const p2y = _c2.y;

  if (t1 < lo && span > S.EPSILON) {
    const u = S.div(lo - t1, span);
    _c1.set(S.lerp(p1x, p2x, u), S.lerp(p1y, p2y, u));
  }
  if (t2 > hi && span > S.EPSILON) {
    const u = S.div(hi - t1, span);
    _c2.set(S.lerp(p1x, p2x, u), S.lerp(p1y, p2y, u));
  }

  // Separation of each clipped point, measured along the reference normal and
  // reduced by both skin radii.
  const s1 = S.mulAdd(_c1.x - _v11.x, refN.x, S.mul(_c1.y - _v11.y, refN.y)) - radius;
  const s2 = S.mulAdd(_c2.x - _v11.x, refN.x, S.mul(_c2.y - _v11.y, refN.y)) - radius;

  /*
   * The clipped points lie on the incident *hull*; the true contact point is
   * halfway between the two surfaces:
   *
   *     point = clip - n · (rInc + s/2)
   */
  const rInc = incProxy.radius;
  let count = 0;

  if (s1 <= SPECULATIVE) {
    const mp = manifold.points[count]!;
    const k = rInc + S.half(s1);
    Transform.apply(
      mp.point,
      refXf,
      _tmp2.set(S.mulAdd(refN.x, -k, _c1.x), S.mulAdd(refN.y, -k, _c1.y)),
    );
    mp.separation = s1;
    mp.id = flip ? makeID(idA, refIndex, 1, 0) : makeID(refIndex, idA, 0, 1);
    count++;
  }
  if (s2 <= SPECULATIVE) {
    const mp = manifold.points[count]!;
    const k = rInc + S.half(s2);
    Transform.apply(
      mp.point,
      refXf,
      _tmp2.set(S.mulAdd(refN.x, -k, _c2.x), S.mulAdd(refN.y, -k, _c2.y)),
    );
    mp.separation = s2;
    mp.id = flip ? makeID(idB, refIndex, 1, 0) : makeID(refIndex, idB, 0, 1);
    count++;
  }
  if (count === 0) return;

  Rot.rotate(manifold.normal, refXf.q, refN);
  if (flip) manifold.normal.neg(); // the normal must always point A -> B
  manifold.pointCount = count;
}

/* ------------------------------------------------------------------ *
 * One-sided chain filtering
 * ------------------------------------------------------------------ */

const _segN = Vec2.zero();

/**
 * Reject contacts that would push a body out through the *back* of a chain
 * segment.
 *
 * A {@link Segment} carrying ghost vertices belongs to a chain and is treated
 * as **one-sided**. This is what kills the classic "ghost collision" jolt: an
 * interior edge shared by two tiles can no longer fire a sideways impulse at a
 * box sliding across the seam, because only its solid face collides.
 *
 * ### Winding convention
 *
 * The solid side is the one **to the left of the direction of travel**, i.e.
 * the normal is the segment direction rotated +90°:
 *
 * ```
 * solidNormal = perp(p2 - p1) = (-(p2.y - p1.y), p2.x - p1.x)
 * ```
 *
 * So a ground contour written left→right (`x` increasing) is solid from above,
 * and a counter-clockwise loop is solid on the inside — a room you cannot
 * escape. Reverse the point order to flip which side is solid.
 *
 * @param manifold manifold to test; its normal points A → B
 * @param shape    the segment
 * @param xf       the segment's transform
 * @param isA      `true` when the segment is shape A of the pair
 */
function filterOneSided(manifold: Manifold, shape: Shape, xf: Transform, isA: boolean): void {
  if (manifold.pointCount === 0) return;
  const seg = shape as Segment;
  if (seg.ghost0 === null && seg.ghost1 === null) return; // standalone: two-sided

  // Solid normal = +90° from the direction of travel, in local space.
  _segN.set(-(seg.p2.y - seg.p1.y), seg.p2.x - seg.p1.x);
  if (_segN.normalize() === S.ZERO) return;
  Rot.rotate(_segN, xf.q, _segN);

  // The manifold normal runs A → B, so when the segment is A the push applied
  // to B must agree with the solid face; when it is B the sense is reversed.
  const d = isA ? Vec2.dot(manifold.normal, _segN) : -Vec2.dot(manifold.normal, _segN);
  if (d < S.ZERO) manifold.pointCount = 0;
}

/* ------------------------------------------------------------------ *
 * Dispatch
 * ------------------------------------------------------------------ */

type Fn = (m: Manifold, a: Shape, xa: Transform, b: Shape, xb: Transform) => void;

/**
 * `table[typeA][typeB]` — a flat lookup instead of a chain of `instanceof`
 * tests. `flipped` marks the entries whose arguments must be swapped, so only
 * the lower triangle needs an implementation.
 */
const table: (Fn | null)[][] = [];
const flippedTable: boolean[][] = [];
for (let i = 0; i < ShapeType.Count; i++) {
  table.push(new Array<Fn | null>(ShapeType.Count).fill(null));
  flippedTable.push(new Array<boolean>(ShapeType.Count).fill(false));
}

function reg(a: ShapeType, b: ShapeType, fn: Fn): void {
  table[a]![b] = fn;
  if (a !== b) {
    table[b]![a] = fn;
    flippedTable[b]![a] = true;
  }
}

reg(ShapeType.Circle, ShapeType.Circle, collideCircles);
reg(ShapeType.Polygon, ShapeType.Circle, collidePolygonCircle);
reg(ShapeType.Capsule, ShapeType.Circle, collidePolygonCircle);
reg(ShapeType.Segment, ShapeType.Circle, collidePolygonCircle);
reg(ShapeType.Polygon, ShapeType.Polygon, collidePolygons);
reg(ShapeType.Capsule, ShapeType.Polygon, collidePolygons);
reg(ShapeType.Capsule, ShapeType.Capsule, collidePolygons);
reg(ShapeType.Segment, ShapeType.Polygon, collidePolygons);
reg(ShapeType.Segment, ShapeType.Capsule, collidePolygons);

/**
 * Compute the contact manifold for any pair of shapes.
 *
 * The resulting normal always points **from A to B**, no matter which internal
 * routine ran. Segment-vs-segment produces no contacts (two massless shapes
 * can never resolve against each other).
 */
export function collide(
  manifold: Manifold,
  shapeA: Shape,
  xfA: Transform,
  shapeB: Shape,
  xfB: Transform,
): void {
  const fn = table[shapeA.type]![shapeB.type];
  if (fn === null || fn === undefined) {
    manifold.pointCount = 0;
    return;
  }
  const call = fn;
  if (flippedTable[shapeA.type]![shapeB.type] === true) {
    call(manifold, shapeB, xfB, shapeA, xfA);
    if (manifold.pointCount > 0) manifold.normal.neg();
  } else {
    call(manifold, shapeA, xfA, shapeB, xfB);
  }
  if (shapeA.type === ShapeType.Segment) filterOneSided(manifold, shapeA, xfA, true);
  else if (shapeB.type === ShapeType.Segment) filterOneSided(manifold, shapeB, xfB, false);
}

export { LINEAR_SLOP, SPECULATIVE_DISTANCE };
