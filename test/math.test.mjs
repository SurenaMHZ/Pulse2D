/**
 * Math layer tests: deterministic trig, vectors, rotations, RNG.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  Vec2,
  Rot,
  Transform,
  Mat22,
  Rng,
  sin,
  cos,
  tan,
  sinCos,
  atan,
  atan2,
  asin,
  acos,
  normalizeAngle,
  Scalar as S,
} from '../dist/pulse2d.mjs';

test('trig: sin/cos match the reference within 1e-10', () => {
  let maxSin = 0;
  let maxCos = 0;
  for (let i = -2000; i <= 2000; i++) {
    const a = (i / 2000) * 4 * Math.PI;
    maxSin = Math.max(maxSin, Math.abs(S.toFloat(sin(S.fromFloat(a))) - Math.sin(a)));
    maxCos = Math.max(maxCos, Math.abs(S.toFloat(cos(S.fromFloat(a))) - Math.cos(a)));
  }
  assert.ok(maxSin < 1e-10, `sin error ${maxSin}`);
  assert.ok(maxCos < 1e-10, `cos error ${maxCos}`);
});

test('trig: sinCos agrees with sin and cos separately', () => {
  const out = [0, 0];
  for (let i = -100; i <= 100; i++) {
    const a = S.fromFloat((i / 100) * Math.PI * 3);
    sinCos(a, out);
    assert.equal(out[0], sin(a));
    assert.equal(out[1], cos(a));
  }
});

test('trig: pythagorean identity holds to 1e-10', () => {
  for (let i = 0; i < 500; i++) {
    const a = S.fromFloat((i / 500) * 20 - 10);
    const s = S.toFloat(sin(a));
    const c = S.toFloat(cos(a));
    assert.ok(Math.abs(s * s + c * c - 1) < 1e-10);
  }
});

test('trig: atan2 covers all four quadrants', () => {
  let maxErr = 0;
  for (let i = 0; i < 360; i++) {
    const a = (i / 180) * Math.PI - Math.PI;
    const y = Math.sin(a);
    const x = Math.cos(a);
    const got = S.toFloat(atan2(S.fromFloat(y), S.fromFloat(x)));
    let diff = got - Math.atan2(y, x);
    while (diff > Math.PI) diff -= 2 * Math.PI;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    maxErr = Math.max(maxErr, Math.abs(diff));
  }
  assert.ok(maxErr < 1e-6, `atan2 error ${maxErr}`);
});

test('trig: atan2(0,0) is exactly 0', () => {
  assert.equal(atan2(S.ZERO, S.ZERO), S.ZERO);
});

test('trig: atan / asin / acos are accurate', () => {
  for (let i = -50; i <= 50; i++) {
    const t = i / 10;
    assert.ok(Math.abs(S.toFloat(atan(S.fromFloat(t))) - Math.atan(t)) < 1e-6);
  }
  for (let i = -100; i <= 100; i++) {
    const v = i / 100;
    assert.ok(Math.abs(S.toFloat(asin(S.fromFloat(v))) - Math.asin(v)) < 1e-5, `asin(${v})`);
    assert.ok(Math.abs(S.toFloat(acos(S.fromFloat(v))) - Math.acos(v)) < 1e-5, `acos(${v})`);
  }
});

test('trig: asin/acos clamp out-of-range inputs instead of returning NaN', () => {
  assert.ok(Number.isFinite(S.toFloat(asin(S.fromFloat(5)))));
  assert.ok(Number.isFinite(S.toFloat(acos(S.fromFloat(-5)))));
});

test('trig: tan is finite even at the poles', () => {
  assert.ok(Number.isFinite(S.toFloat(tan(S.HALF_PI))));
});

test('trig: normalizeAngle wraps into (-pi, pi]', () => {
  for (let i = -50; i <= 50; i++) {
    const a = i * 1.7;
    const n = S.toFloat(normalizeAngle(S.fromFloat(a)));
    assert.ok(n > -Math.PI - 1e-9 && n <= Math.PI + 1e-9, `${a} -> ${n}`);
    // must be congruent mod 2pi
    const k = Math.round((a - n) / (2 * Math.PI));
    assert.ok(Math.abs(a - n - k * 2 * Math.PI) < 1e-9);
  }
});

test('trig: repeated evaluation is bit-identical', () => {
  const a = S.fromFloat(1.2345678);
  const first = sin(a);
  for (let i = 0; i < 1000; i++) assert.equal(sin(a), first);
});

test('Vec2: basic algebra', () => {
  const a = Vec2.of(3, 4);
  assert.equal(S.toFloat(a.length()), 5);
  assert.equal(S.toFloat(a.lengthSq()), 25);

  const b = Vec2.of(1, 2);
  assert.equal(S.toFloat(Vec2.dot(a, b)), 11);
  assert.equal(S.toFloat(Vec2.cross(a, b)), 3 * 2 - 4 * 1);

  const out = Vec2.zero();
  Vec2.addTo(out, a, b);
  assert.deepEqual(out.toFloats(), { x: 4, y: 6 });
  Vec2.subTo(out, a, b);
  assert.deepEqual(out.toFloats(), { x: 2, y: 2 });
  Vec2.addScaledTo(out, a, b, S.fromFloat(2));
  assert.deepEqual(out.toFloats(), { x: 5, y: 8 });
});

test('Vec2: normalize returns the old length and yields a unit vector', () => {
  const v = Vec2.of(3, 4);
  const len = v.normalize();
  assert.equal(S.toFloat(len), 5);
  assert.ok(Math.abs(S.toFloat(v.length()) - 1) < 1e-12);
});

test('Vec2: normalizing a zero vector is safe', () => {
  const v = Vec2.zero();
  assert.equal(v.normalize(), S.ZERO);
  assert.ok(v.isZero());
  assert.ok(v.isValid());
});

test('Vec2: perp is a +90 degree rotation', () => {
  // `-0 === 0` is true, but `assert.strictEqual` uses Object.is and would
  // reject it. Negating a zero component is legitimate here, so compare with
  // `===` rather than pinning the sign of zero.
  const out = Vec2.zero();
  Vec2.perpTo(out, Vec2.of(1, 0));
  assert.ok(S.toFloat(out.x) === 0);
  assert.ok(S.toFloat(out.y) === 1);
  Vec2.rperpTo(out, Vec2.of(1, 0));
  assert.ok(S.toFloat(out.x) === 0);
  assert.ok(S.toFloat(out.y) === -1);
});

test('Vec2: truncate caps the magnitude', () => {
  const v = Vec2.of(10, 0);
  v.truncate(S.fromFloat(3));
  assert.ok(Math.abs(S.toFloat(v.length()) - 3) < 1e-12);
  const w = Vec2.of(1, 0);
  w.truncate(S.fromFloat(3));
  assert.equal(S.toFloat(w.length()), 1);
});

test('Rot: round-trips through an angle', () => {
  for (let i = -30; i <= 30; i++) {
    const a = (i / 30) * Math.PI * 0.99;
    const r = Rot.of(a);
    assert.ok(Math.abs(S.toFloat(r.getAngle()) - a) < 1e-6, `angle ${a}`);
  }
});

test('Rot: rotate then rotateT is the identity', () => {
  const r = Rot.of(0.7);
  const v = Vec2.of(2, -3);
  const w = Vec2.zero();
  Rot.rotate(w, r, v);
  Rot.rotateT(w, r, w);
  assert.ok(Math.abs(S.toFloat(w.x) - 2) < 1e-12);
  assert.ok(Math.abs(S.toFloat(w.y) + 3) < 1e-12);
});

test('Rot: integrate stays normalized over many steps', () => {
  const r = new Rot();
  for (let i = 0; i < 100000; i++) r.integrate(S.fromFloat(0.01));
  const mag = Math.hypot(S.toFloat(r.s), S.toFloat(r.c));
  assert.ok(Math.abs(mag - 1) < 1e-12, `magnitude drifted to ${mag}`);
});

test('Rot: relativeAngle measures the shortest signed difference', () => {
  const a = Rot.of(0.2);
  const b = Rot.of(1.1);
  assert.ok(Math.abs(S.toFloat(Rot.relativeAngle(a, b)) - 0.9) < 1e-6);
  assert.ok(Math.abs(S.toFloat(Rot.relativeAngle(b, a)) + 0.9) < 1e-6);
});

test('Transform: apply and applyT are inverses', () => {
  const xf = new Transform(Vec2.of(3, -2), Rot.of(0.9));
  const p = Vec2.of(1.5, 2.5);
  const w = Vec2.zero();
  Transform.apply(w, xf, p);
  Transform.applyT(w, xf, w);
  assert.ok(Math.abs(S.toFloat(w.x) - 1.5) < 1e-12);
  assert.ok(Math.abs(S.toFloat(w.y) - 2.5) < 1e-12);
});

test('Transform: composition matches sequential application', () => {
  const a = new Transform(Vec2.of(1, 2), Rot.of(0.3));
  const b = new Transform(Vec2.of(-2, 1), Rot.of(-0.8));
  const ab = new Transform();
  Transform.mulTo(ab, a, b);

  const p = Vec2.of(0.5, -1.5);
  const viaComposite = Vec2.zero();
  Transform.apply(viaComposite, ab, p);

  const sequential = Vec2.zero();
  Transform.apply(sequential, b, p);
  Transform.apply(sequential, a, sequential);

  assert.ok(Math.abs(S.toFloat(viaComposite.x) - S.toFloat(sequential.x)) < 1e-12);
  assert.ok(Math.abs(S.toFloat(viaComposite.y) - S.toFloat(sequential.y)) < 1e-12);
});

test('Mat22: solve inverts the system', () => {
  const m = new Mat22();
  m.set(S.fromFloat(4), S.fromFloat(1), S.fromFloat(2), S.fromFloat(3));
  const b = Vec2.of(9, 8);
  const x = Vec2.zero();
  m.solve(x, b);
  // verify M·x == b
  const checkX = 4 * S.toFloat(x.x) + 1 * S.toFloat(x.y);
  const checkY = 2 * S.toFloat(x.x) + 3 * S.toFloat(x.y);
  assert.ok(Math.abs(checkX - 9) < 1e-10);
  assert.ok(Math.abs(checkY - 8) < 1e-10);
});

test('Mat22: a singular system yields zero rather than NaN', () => {
  const m = new Mat22();
  m.set(S.ONE, S.TWO, S.TWO, S.fromFloat(4));
  const x = Vec2.zero();
  m.solve(x, Vec2.of(1, 1));
  assert.ok(x.isZero());
});

test('Rng: the same seed produces the same stream', () => {
  const a = new Rng(12345);
  const b = new Rng(12345);
  for (let i = 0; i < 1000; i++) assert.equal(a.next(), b.next());
});

test('Rng: different seeds diverge', () => {
  const a = new Rng(1);
  const b = new Rng(2);
  let same = 0;
  for (let i = 0; i < 100; i++) if (a.next() === b.next()) same++;
  assert.ok(same < 5);
});

test('Rng: state save/restore reproduces the stream', () => {
  const r = new Rng(999);
  for (let i = 0; i < 50; i++) r.next();
  const [s, i0] = r.getState();
  const expected = [];
  for (let i = 0; i < 20; i++) expected.push(r.next());
  r.setState(s, i0);
  for (let i = 0; i < 20; i++) assert.equal(r.next(), expected[i]);
});

test('Rng: float stays inside [0,1) and int inside its range', () => {
  const r = new Rng(7);
  for (let i = 0; i < 10000; i++) {
    const f = r.float();
    assert.ok(f >= 0 && f < 1);
    const n = r.int(3, 9);
    assert.ok(n >= 3 && n <= 9 && Number.isInteger(n));
  }
});

test('Rng: distribution is roughly uniform', () => {
  const r = new Rng(4242);
  const buckets = new Array(10).fill(0);
  const n = 100000;
  for (let i = 0; i < n; i++) buckets[Math.floor(r.float() * 10)]++;
  for (const b of buckets) {
    assert.ok(Math.abs(b - n / 10) < n / 10 * 0.1, `bucket skew ${b}`);
  }
});

test('Rng: shuffle is deterministic and preserves the multiset', () => {
  const make = () => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
  const a = new Rng(5).shuffle(make());
  const b = new Rng(5).shuffle(make());
  assert.deepEqual(a, b);
  assert.deepEqual([...a].sort((x, y) => x - y), make());
});
