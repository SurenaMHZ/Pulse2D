/**
 * @module collision/Distance
 *
 * **GJK** distance between two convex shapes, plus a linear shape cast built
 * on top of it.
 *
 * The Gilbert–Johnson–Keerthi algorithm walks the Minkowski difference of the
 * two point sets and converges on the closest simplex, usually in 2–4
 * iterations for game-sized shapes. Everything here is written on the scalar
 * backend and uses a fixed iteration cap, so the number of operations — and
 * therefore the result — is identical on every machine.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Transform } from './../math/Transform.js';
import { Rot } from './../math/Rot.js';
import type { Shape } from './Shape.js';

/** Maximum GJK iterations. Fixed so the cost is bounded and reproducible. */
const MAX_ITERS = 20;

/** A convex point cloud + radius, the only thing GJK needs to know. */
export interface DistanceProxy {
  /** Local-space vertices. */
  vertices: Vec2[];
  /** Number of valid vertices. */
  count: number;
  /** Skin radius. */
  radius: Scalar;
}

/** Build a proxy view of a shape (no allocation beyond the array). */
export function makeProxy(shape: Shape): DistanceProxy {
  const n = shape.vertexCount;
  const vs: Vec2[] = new Array(n);
  for (let i = 0; i < n; i++) vs[i] = shape.getVertex(i);
  return { vertices: vs, count: n, radius: shape.radius };
}

export interface DistanceInput {
  proxyA: DistanceProxy;
  proxyB: DistanceProxy;
  xfA: Transform;
  xfB: Transform;
  /** Subtract the skin radii from the result. */
  useRadii: boolean;
}

export interface DistanceOutput {
  /** Closest point on A, world space. */
  readonly pointA: Vec2;
  /** Closest point on B, world space. */
  readonly pointB: Vec2;
  /** Separation; `0` when the shapes overlap. */
  distance: Scalar;
  /** Iterations actually performed (diagnostic only). */
  iterations: number;
  /** Number of simplex vertices at termination: 1 = vertex, 2 = edge. */
  simplexCount: number;
}

/** Allocate a reusable output record. */
export function makeDistanceOutput(): DistanceOutput {
  return { pointA: Vec2.zero(), pointB: Vec2.zero(), distance: S.ZERO, iterations: 0, simplexCount: 0 };
}

/* ------------------------------------------------------------------ *
 * Simplex
 * ------------------------------------------------------------------ */

class SimplexVertex {
  /** Support point on A, world space. */
  readonly wA = Vec2.zero();
  /** Support point on B, world space. */
  readonly wB = Vec2.zero();
  /** `wB - wA`. */
  readonly w = Vec2.zero();
  /** Barycentric coordinate. */
  a: Scalar = S.ZERO;
  indexA = 0;
  indexB = 0;

  copyFrom(o: SimplexVertex): void {
    this.wA.copyFrom(o.wA);
    this.wB.copyFrom(o.wB);
    this.w.copyFrom(o.w);
    this.a = o.a;
    this.indexA = o.indexA;
    this.indexB = o.indexB;
  }
}

/** Scratch simplex — the module is single-threaded, so one instance is safe. */
const _v: [SimplexVertex, SimplexVertex, SimplexVertex] = [
  new SimplexVertex(),
  new SimplexVertex(),
  new SimplexVertex(),
];
let _count = 0;

const _tmp = Vec2.zero();
const _dir = Vec2.zero();
const _localDir = Vec2.zero();
const _saveA = [0, 0, 0];
const _saveB = [0, 0, 0];

/** Search direction = -(closest point). */
function simplexSearchDir(out: Vec2): Vec2 {
  switch (_count) {
    case 1:
      return Vec2.negTo(out, _v[0].w);
    case 2: {
      const e12x = _v[1].w.x - _v[0].w.x;
      const e12y = _v[1].w.y - _v[0].w.y;
      // sign tells us which side of the edge the origin lies on
      const sgn = S.mulAdd(e12x, -_v[0].w.y, -S.mul(e12y, -_v[0].w.x));
      if (sgn > S.ZERO) {
        out.x = -e12y;
        out.y = e12x;
      } else {
        out.x = e12y;
        out.y = -e12x;
      }
      return out;
    }
    default:
      return out.setZero();
  }
}

function simplexWitness(pA: Vec2, pB: Vec2): void {
  switch (_count) {
    case 1:
      pA.copyFrom(_v[0].wA);
      pB.copyFrom(_v[0].wB);
      break;
    case 2:
      Vec2.combineTo(pA, _v[0].wA, _v[0].a, _v[1].wA, _v[1].a);
      Vec2.combineTo(pB, _v[0].wB, _v[0].a, _v[1].wB, _v[1].a);
      break;
    default:
      Vec2.combineTo(pA, _v[0].wA, _v[0].a, _v[1].wA, _v[1].a);
      pA.addScaled(_v[2].wA, _v[2].a);
      pB.copyFrom(pA);
      break;
  }
}

/** Reduce a 2-vertex simplex (line segment) to its closest feature. */
function solve2(): void {
  const w1 = _v[0].w;
  const w2 = _v[1].w;
  const ex = w2.x - w1.x;
  const ey = w2.y - w1.y;

  // origin is outside w1
  const d12_2 = -S.mulAdd(w1.x, ex, S.mul(w1.y, ey));
  if (d12_2 <= S.ZERO) {
    _v[0].a = S.ONE;
    _count = 1;
    return;
  }
  // origin is outside w2
  const d12_1 = S.mulAdd(w2.x, ex, S.mul(w2.y, ey));
  if (d12_1 <= S.ZERO) {
    _v[1].a = S.ONE;
    _count = 1;
    _v[0].copyFrom(_v[1]);
    return;
  }
  const inv = S.inv(d12_1 + d12_2);
  _v[0].a = S.mul(d12_1, inv);
  _v[1].a = S.mul(d12_2, inv);
  _count = 2;
}

/** Reduce a 3-vertex simplex (triangle) to its closest feature. */
function solve3(): void {
  const w1 = _v[0].w;
  const w2 = _v[1].w;
  const w3 = _v[2].w;

  const e12x = w2.x - w1.x;
  const e12y = w2.y - w1.y;
  const w1e12 = S.mulAdd(w1.x, e12x, S.mul(w1.y, e12y));
  const w2e12 = S.mulAdd(w2.x, e12x, S.mul(w2.y, e12y));
  const d12_1 = w2e12;
  const d12_2 = -w1e12;

  const e13x = w3.x - w1.x;
  const e13y = w3.y - w1.y;
  const w1e13 = S.mulAdd(w1.x, e13x, S.mul(w1.y, e13y));
  const w3e13 = S.mulAdd(w3.x, e13x, S.mul(w3.y, e13y));
  const d13_1 = w3e13;
  const d13_2 = -w1e13;

  const e23x = w3.x - w2.x;
  const e23y = w3.y - w2.y;
  const w2e23 = S.mulAdd(w2.x, e23x, S.mul(w2.y, e23y));
  const w3e23 = S.mulAdd(w3.x, e23x, S.mul(w3.y, e23y));
  const d23_1 = w3e23;
  const d23_2 = -w2e23;

  const n123 = S.mulAdd(e12x, e13y, -S.mul(e12y, e13x));
  const d123_1 = S.mul(n123, S.mulAdd(w2.x, w3.y, -S.mul(w2.y, w3.x)));
  const d123_2 = S.mul(n123, S.mulAdd(w3.x, w1.y, -S.mul(w3.y, w1.x)));
  const d123_3 = S.mul(n123, S.mulAdd(w1.x, w2.y, -S.mul(w1.y, w2.x)));

  // w1 region
  if (d12_2 <= S.ZERO && d13_2 <= S.ZERO) {
    _v[0].a = S.ONE;
    _count = 1;
    return;
  }
  // e12 region
  if (d12_1 > S.ZERO && d12_2 > S.ZERO && d123_3 <= S.ZERO) {
    const inv = S.inv(d12_1 + d12_2);
    _v[0].a = S.mul(d12_1, inv);
    _v[1].a = S.mul(d12_2, inv);
    _count = 2;
    return;
  }
  // e13 region
  if (d13_1 > S.ZERO && d13_2 > S.ZERO && d123_2 <= S.ZERO) {
    const inv = S.inv(d13_1 + d13_2);
    _v[0].a = S.mul(d13_1, inv);
    _v[2].a = S.mul(d13_2, inv);
    _count = 2;
    _v[1].copyFrom(_v[2]);
    return;
  }
  // w2 region
  if (d12_1 <= S.ZERO && d23_2 <= S.ZERO) {
    _v[1].a = S.ONE;
    _count = 1;
    _v[0].copyFrom(_v[1]);
    return;
  }
  // w3 region
  if (d13_1 <= S.ZERO && d23_1 <= S.ZERO) {
    _v[2].a = S.ONE;
    _count = 1;
    _v[0].copyFrom(_v[2]);
    return;
  }
  // e23 region
  if (d23_1 > S.ZERO && d23_2 > S.ZERO && d123_1 <= S.ZERO) {
    const inv = S.inv(d23_1 + d23_2);
    _v[1].a = S.mul(d23_1, inv);
    _v[2].a = S.mul(d23_2, inv);
    _count = 2;
    _v[0].copyFrom(_v[2]);
    return;
  }
  // interior — the origin is inside the triangle
  const inv = S.inv(d123_1 + d123_2 + d123_3);
  _v[0].a = S.mul(d123_1, inv);
  _v[1].a = S.mul(d123_2, inv);
  _v[2].a = S.mul(d123_3, inv);
  _count = 3;
}

/** Support index of `proxy` along the local direction `d`. */
function support(proxy: DistanceProxy, d: Vec2): number {
  let best = 0;
  let bestVal = S.mulAdd(proxy.vertices[0]!.x, d.x, S.mul(proxy.vertices[0]!.y, d.y));
  for (let i = 1; i < proxy.count; i++) {
    const v = proxy.vertices[i]!;
    const val = S.mulAdd(v.x, d.x, S.mul(v.y, d.y));
    if (val > bestVal) {
      bestVal = val;
      best = i;
    }
  }
  return best;
}

/**
 * Closest distance between two convex shapes.
 *
 * @returns `out`, filled with the witness points and the separation.
 */
export function shapeDistance(out: DistanceOutput, input: DistanceInput): DistanceOutput {
  const { proxyA, proxyB, xfA, xfB } = input;

  // Initial simplex: one arbitrary vertex pair.
  _count = 1;
  _v[0].indexA = 0;
  _v[0].indexB = 0;
  Transform.apply(_v[0].wA, xfA, proxyA.vertices[0]!);
  Transform.apply(_v[0].wB, xfB, proxyB.vertices[0]!);
  Vec2.subTo(_v[0].w, _v[0].wB, _v[0].wA);
  _v[0].a = S.ONE;

  let iter = 0;
  let saveCount = 0;

  while (iter < MAX_ITERS) {
    saveCount = _count;
    for (let i = 0; i < saveCount; i++) {
      _saveA[i] = _v[i]!.indexA;
      _saveB[i] = _v[i]!.indexB;
    }

    if (_count === 2) solve2();
    else if (_count === 3) solve3();

    if (_count === 3) break; // origin enclosed => overlapping

    simplexSearchDir(_dir);
    if (_dir.lengthSq() < S.EPSILON_SQ) break;

    // New support point on the Minkowski difference along `_dir`.
    const vert = _v[_count]!;
    Rot.rotateT(_localDir, xfA.q, Vec2.negTo(_tmp, _dir));
    vert.indexA = support(proxyA, _localDir);
    Transform.apply(vert.wA, xfA, proxyA.vertices[vert.indexA]!);

    Rot.rotateT(_localDir, xfB.q, _dir);
    vert.indexB = support(proxyB, _localDir);
    Transform.apply(vert.wB, xfB, proxyB.vertices[vert.indexB]!);

    Vec2.subTo(vert.w, vert.wB, vert.wA);
    iter++;

    // Termination: the new vertex repeats one already in the simplex.
    let duplicate = false;
    for (let i = 0; i < saveCount; i++) {
      if (vert.indexA === (_saveA[i] as number) && vert.indexB === (_saveB[i] as number)) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) break;

    _count++;
  }

  out.iterations = iter;
  out.simplexCount = _count;
  simplexWitness(out.pointA, out.pointB);
  out.distance = Vec2.distance(out.pointA, out.pointB);

  if (input.useRadii) {
    const rA = proxyA.radius;
    const rB = proxyB.radius;
    if (out.distance > rA + rB && out.distance > S.EPSILON) {
      out.distance -= rA + rB;
      Vec2.subTo(_tmp, out.pointB, out.pointA);
      _tmp.normalize();
      out.pointA.addScaled(_tmp, rA);
      out.pointB.addScaled(_tmp, -rB);
    } else {
      // Overlapping once the skins are taken into account.
      const mx = S.half(out.pointA.x + out.pointB.x);
      const my = S.half(out.pointA.y + out.pointB.y);
      out.pointA.set(mx, my);
      out.pointB.set(mx, my);
      out.distance = S.ZERO;
    }
  }
  return out;
}

/* ------------------------------------------------------------------ *
 * Linear shape cast (conservative advancement)
 * ------------------------------------------------------------------ */

export interface ShapeCastInput {
  proxyA: DistanceProxy;
  proxyB: DistanceProxy;
  xfA: Transform;
  xfB: Transform;
  /** Translation applied to B over the cast. */
  translationB: Vec2;
  /** Stop early once this fraction is exceeded. */
  maxFraction: Scalar;
}

export interface ShapeCastOutput {
  readonly point: Vec2;
  readonly normal: Vec2;
  fraction: Scalar;
  hit: boolean;
}

export function makeShapeCastOutput(): ShapeCastOutput {
  return { point: Vec2.zero(), normal: Vec2.zero(), fraction: S.ZERO, hit: false };
}

const _castOut = makeDistanceOutput();
const _castXfB = new Transform();
const _castInput: DistanceInput = {
  proxyA: { vertices: [], count: 0, radius: S.ZERO },
  proxyB: { vertices: [], count: 0, radius: S.ZERO },
  xfA: new Transform(),
  xfB: new Transform(),
  useRadii: true,
};

/**
 * Sweep proxy B along `translationB` and find the first touch with A.
 *
 * Implemented as **conservative advancement**: repeatedly measure the gap with
 * GJK and advance by the largest step that provably cannot cause a crossing.
 * The iteration count is capped, so the cost is bounded.
 *
 * This is what bullets and character sweeps use; it never tunnels regardless
 * of speed.
 */
export function shapeCast(out: ShapeCastOutput, input: ShapeCastInput): boolean {
  out.hit = false;
  out.fraction = S.ZERO;

  const target = S.fromFloat(0.005); // linear slop: stop this far apart
  const tolerance = S.fromFloat(0.00025);

  const dist = Vec2.of(0, 0).copyFrom(input.translationB);
  const totalLen = dist.length();
  if (totalLen < S.EPSILON) return false;

  _castInput.proxyA = input.proxyA;
  _castInput.proxyB = input.proxyB;
  _castInput.xfA = input.xfA;
  _castInput.xfB = _castXfB;
  _castInput.useRadii = true;

  let t = S.ZERO;
  for (let iter = 0; iter < 24; iter++) {
    _castXfB.q.copyFrom(input.xfB.q);
    _castXfB.p.x = S.mulAdd(input.translationB.x, t, input.xfB.p.x);
    _castXfB.p.y = S.mulAdd(input.translationB.y, t, input.xfB.p.y);

    shapeDistance(_castOut, _castInput);

    if (_castOut.distance < target + tolerance) {
      if (iter === 0) {
        // Already touching at t = 0.
        out.fraction = S.ZERO;
        out.point.copyFrom(_castOut.pointA);
        Vec2.subTo(out.normal, _castOut.pointB, _castOut.pointA);
        if (out.normal.normalize() === S.ZERO) out.normal.set(S.ZERO, S.ONE);
        out.hit = true;
        return true;
      }
      out.fraction = t;
      out.point.copyFrom(_castOut.pointA);
      Vec2.subTo(out.normal, _castOut.pointA, _castOut.pointB);
      if (out.normal.normalize() === S.ZERO) out.normal.set(S.ZERO, S.ONE);
      out.hit = true;
      return true;
    }

    // Project the motion onto the separation direction to bound the step.
    Vec2.subTo(_tmp, _castOut.pointB, _castOut.pointA);
    _tmp.normalize();
    const approach = -Vec2.dot(_tmp, input.translationB);
    if (approach <= S.EPSILON) return false; // moving away or tangentially

    t += S.div(_castOut.distance - target, approach);
    if (t >= input.maxFraction) return false;
  }
  return false;
}
