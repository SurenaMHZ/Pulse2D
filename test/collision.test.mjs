/**
 * Collision layer: shapes, mass properties, ray casts, manifolds, broad phase.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  Vec2,
  Rot,
  Transform,
  AABB,
  Circle,
  Capsule,
  Polygon,
  Segment,
  ChainShape,
  Manifold,
  collide,
  DynamicTree,
  BroadPhase,
  shapeDistance,
  shapeCast,
  makeProxy,
  makeDistanceOutput,
  makeShapeCastOutput,
  Scalar as S,
} from '../dist/pulse2d.mjs';

const f = S.toFloat;
const identity = new Transform();
const at = (x, y, angle = 0) => new Transform(Vec2.of(x, y), Rot.of(angle));
const massOut = () => ({ mass: S.ZERO, center: Vec2.zero(), inertia: S.ZERO });
const rayOut = () => ({ normal: Vec2.zero(), point: Vec2.zero(), fraction: S.ZERO, hit: false });

/* ----------------------------- AABB ----------------------------- */

test('AABB: overlap, containment and perimeter', () => {
  const a = new AABB().set(S.fromFloat(0), S.fromFloat(0), S.fromFloat(2), S.fromFloat(2));
  const b = new AABB().set(S.fromFloat(1), S.fromFloat(1), S.fromFloat(3), S.fromFloat(3));
  const c = new AABB().set(S.fromFloat(5), S.fromFloat(5), S.fromFloat(6), S.fromFloat(6));
  assert.ok(AABB.overlaps(a, b));
  assert.ok(!AABB.overlaps(a, c));
  assert.equal(f(a.perimeter()), 8);
  assert.equal(f(a.area()), 4);

  const inner = new AABB().set(S.fromFloat(0.5), S.fromFloat(0.5), S.fromFloat(1), S.fromFloat(1));
  assert.ok(a.contains(inner));
  assert.ok(!inner.contains(a));
  assert.ok(a.containsPoint(Vec2.of(1, 1)));
  assert.ok(!a.containsPoint(Vec2.of(3, 1)));
});

test('AABB: touching boxes count as overlapping', () => {
  const a = new AABB().set(S.ZERO, S.ZERO, S.fromFloat(1), S.fromFloat(1));
  const b = new AABB().set(S.fromFloat(1), S.ZERO, S.fromFloat(2), S.fromFloat(1));
  assert.ok(AABB.overlaps(a, b));
});

test('AABB: ray cast hits and misses correctly', () => {
  const box = new AABB().set(S.fromFloat(1), S.fromFloat(-1), S.fromFloat(2), S.fromFloat(1));
  const hit = box.rayCast(Vec2.of(0, 0), Vec2.of(4, 0), S.ONE);
  assert.ok(f(hit) > 0, 'should hit');
  assert.ok(Math.abs(f(hit) - 0.25) < 1e-9, `fraction ${f(hit)}`);
  const miss = box.rayCast(Vec2.of(0, 5), Vec2.of(4, 0), S.ONE);
  assert.ok(f(miss) < 0, 'should miss');
});

/* ---------------------------- shapes ---------------------------- */

test('Circle: mass properties match the closed form', () => {
  const c = Circle.of(2);
  const md = massOut();
  c.computeMass(md, S.fromFloat(3));
  const expectedMass = 3 * Math.PI * 4;
  assert.ok(Math.abs(f(md.mass) - expectedMass) < 1e-9);
  // I about the centre = m·r²/2, and the centre is the origin here
  assert.ok(Math.abs(f(md.inertia) - expectedMass * 2) < 1e-9);
});

test('Circle: offset centre shifts inertia by the parallel-axis term', () => {
  const c = Circle.of(1, 3, 4); // centre 5 units from the origin
  const md = massOut();
  c.computeMass(md, S.ONE);
  const m = Math.PI;
  assert.ok(Math.abs(f(md.inertia) - m * (0.5 + 25)) < 1e-9);
});

test('Circle: AABB follows the transform', () => {
  const c = Circle.of(1);
  const box = new AABB();
  c.computeAABB(box, at(5, -3));
  assert.equal(f(box.lower.x), 4);
  assert.equal(f(box.upper.y), -2);
});

test('Circle: testPoint respects the boundary', () => {
  const c = Circle.of(1);
  assert.ok(c.testPoint(identity, Vec2.of(0.9, 0)));
  assert.ok(!c.testPoint(identity, Vec2.of(1.1, 0)));
});

test('Circle: ray cast reports the near intersection and an outward normal', () => {
  const c = Circle.of(1);
  const out = rayOut();
  const ok = c.rayCast(out, { p1: Vec2.of(-5, 0), p2: Vec2.of(5, 0), maxFraction: S.ONE }, identity);
  assert.ok(ok);
  assert.ok(Math.abs(f(out.point.x) + 1) < 1e-9, `x ${f(out.point.x)}`);
  assert.ok(Math.abs(f(out.normal.x) + 1) < 1e-9, 'normal points back along the ray');
});

test('Polygon: box mass matches the analytic rectangle', () => {
  const p = Polygon.box(1, 2); // 2 x 4
  const md = massOut();
  p.computeMass(md, S.fromFloat(2));
  assert.ok(Math.abs(f(md.mass) - 16) < 1e-9, `mass ${f(md.mass)}`);
  // I = m(w² + h²)/12 = 16·(4+16)/12
  assert.ok(Math.abs(f(md.inertia) - (16 * 20) / 12) < 1e-9, `inertia ${f(md.inertia)}`);
  assert.ok(Math.abs(f(md.center.x)) < 1e-12 && Math.abs(f(md.center.y)) < 1e-12);
});

test('Polygon: hull discards interior and duplicate points', () => {
  const p = new Polygon([
    Vec2.of(-1, -1),
    Vec2.of(1, -1),
    Vec2.of(1, 1),
    Vec2.of(-1, 1),
    Vec2.of(0, 0), // interior
    Vec2.of(1, -1), // duplicate
  ]);
  assert.equal(p.vertexCount, 4);
});

test('Polygon: winding is counter-clockwise with outward normals', () => {
  const p = Polygon.box(1, 1);
  // signed area must be positive for CCW
  let area = 0;
  for (let i = 0; i < p.vertexCount; i++) {
    const a = p.vertices[i];
    const b = p.vertices[(i + 1) % p.vertexCount];
    area += f(a.x) * f(b.y) - f(b.x) * f(a.y);
  }
  assert.ok(area > 0, 'CCW winding');
  // every normal must point away from the centroid
  for (let i = 0; i < p.vertexCount; i++) {
    const n = p.normals[i];
    const v = p.vertices[i];
    assert.ok(f(n.x) * f(v.x) + f(n.y) * f(v.y) > 0, `normal ${i} points outward`);
  }
});

test('Polygon: accepts input in clockwise order', () => {
  const cw = new Polygon([Vec2.of(-1, 1), Vec2.of(1, 1), Vec2.of(1, -1), Vec2.of(-1, -1)]);
  assert.equal(cw.vertexCount, 4);
  const md = massOut();
  cw.computeMass(md, S.ONE);
  assert.ok(f(md.mass) > 0, `mass must be positive, got ${f(md.mass)}`);
  assert.ok(Math.abs(f(md.mass) - 4) < 1e-9);
});

test('Polygon: regular n-gon area approaches the circle', () => {
  const p = Polygon.regular(8, 1);
  const md = massOut();
  p.computeMass(md, S.ONE);
  const exact = 8 * 0.5 * Math.sin((2 * Math.PI) / 8); // area of a regular octagon, r=1
  assert.ok(Math.abs(f(md.mass) - exact) < 1e-6, `${f(md.mass)} vs ${exact}`);
});

test('Polygon: rejects degenerate input', () => {
  assert.throws(() => new Polygon([Vec2.of(0, 0), Vec2.of(1, 1)]));
});

test('Polygon: testPoint and ray cast', () => {
  const p = Polygon.box(1, 1);
  assert.ok(p.testPoint(identity, Vec2.of(0.5, 0.5)));
  assert.ok(!p.testPoint(identity, Vec2.of(1.5, 0)));

  const out = rayOut();
  const ok = p.rayCast(out, { p1: Vec2.of(-5, 0), p2: Vec2.of(5, 0), maxFraction: S.ONE }, identity);
  assert.ok(ok);
  assert.ok(Math.abs(f(out.point.x) + 1) < 1e-9);
  assert.ok(Math.abs(f(out.normal.x) + 1) < 1e-9);
});

test('Capsule: mass is between the inscribed box and bounding box', () => {
  const c = Capsule.vertical(4, 0.5); // total height 4, radius 0.5
  const md = massOut();
  c.computeMass(md, S.ONE);
  const rect = 2 * 0.5 * 3; // 2r x segment length
  const caps = Math.PI * 0.25;
  assert.ok(Math.abs(f(md.mass) - (rect + caps)) < 1e-9, `mass ${f(md.mass)}`);
  assert.ok(f(md.inertia) > 0);
});

test('Capsule: testPoint follows the swept-segment shape', () => {
  const c = Capsule.horizontal(4, 0.5); // endpoints at x = ±1.5
  assert.ok(c.testPoint(identity, Vec2.of(0, 0.4)));
  assert.ok(!c.testPoint(identity, Vec2.of(0, 0.6)));
  assert.ok(c.testPoint(identity, Vec2.of(1.9, 0)), 'inside the end cap');
  assert.ok(!c.testPoint(identity, Vec2.of(2.1, 0)));
});

test('Capsule: ray cast hits the barrel and the caps', () => {
  const c = Capsule.horizontal(4, 0.5);
  const out = rayOut();
  // vertical ray into the barrel
  assert.ok(c.rayCast(out, { p1: Vec2.of(0, 5), p2: Vec2.of(0, -5), maxFraction: S.ONE }, identity));
  assert.ok(Math.abs(f(out.point.y) - 0.5) < 1e-6, `barrel hit y=${f(out.point.y)}`);
  // horizontal ray into the end cap
  assert.ok(c.rayCast(out, { p1: Vec2.of(-9, 0), p2: Vec2.of(9, 0), maxFraction: S.ONE }, identity));
  assert.ok(Math.abs(f(out.point.x) + 2) < 1e-6, `cap hit x=${f(out.point.x)}`);
});

test('Segment: is massless and never contains a point', () => {
  const s = Segment.of(-1, 0, 1, 0);
  const md = massOut();
  s.computeMass(md, S.ONE);
  assert.equal(f(md.mass), 0);
  assert.ok(!s.testPoint(identity, Vec2.of(0, 0)));
});

test('Segment: ray cast normal always faces the incoming ray', () => {
  const s = Segment.of(-1, 0, 1, 0);
  const out = rayOut();
  assert.ok(s.rayCast(out, { p1: Vec2.of(0, 5), p2: Vec2.of(0, -5), maxFraction: S.ONE }, identity));
  assert.ok(f(out.normal.y) > 0, 'normal points up towards the ray from above');
  assert.ok(s.rayCast(out, { p1: Vec2.of(0, -5), p2: Vec2.of(0, 5), maxFraction: S.ONE }, identity));
  assert.ok(f(out.normal.y) < 0, 'normal flips for a ray from below');
});

test('ChainShape: builds segments with ghost vertices wired up', () => {
  const pts = [Vec2.of(-2, 0), Vec2.of(0, 0), Vec2.of(2, 0), Vec2.of(4, 1)];
  const open = ChainShape.fromPoints(pts, false);
  assert.equal(open.length, 3);
  assert.equal(open[0].ghost0, null, 'first segment has no previous neighbour');
  assert.ok(open[1].ghost0 !== null && open[1].ghost1 !== null);

  const loop = ChainShape.fromPoints(pts, true);
  assert.equal(loop.length, 4);
  assert.ok(loop[0].ghost0 !== null, 'a loop wraps around');
});

/* --------------------------- manifolds -------------------------- */

test('collide: overlapping circles produce one point and a correct normal', () => {
  const m = new Manifold();
  const a = Circle.of(1);
  const b = Circle.of(1);
  collide(m, a, at(0, 0), b, at(1.5, 0));
  assert.equal(m.pointCount, 1);
  assert.ok(Math.abs(f(m.normal.x) - 1) < 1e-9, 'normal points A -> B');
  assert.ok(Math.abs(f(m.points[0].separation) + 0.5) < 1e-9, 'penetration of 0.5');
});

test('collide: distant circles produce no contact', () => {
  const m = new Manifold();
  collide(m, Circle.of(1), at(0, 0), Circle.of(1), at(10, 0));
  assert.equal(m.pointCount, 0);
});

test('collide: nearly touching circles make a speculative contact', () => {
  const m = new Manifold();
  collide(m, Circle.of(1), at(0, 0), Circle.of(1), at(2.01, 0));
  assert.equal(m.pointCount, 1, 'speculative contact expected');
  assert.ok(f(m.points[0].separation) > 0, 'positive separation means not yet touching');
});

test('collide: coincident circles still yield a usable normal', () => {
  const m = new Manifold();
  collide(m, Circle.of(1), at(0, 0), Circle.of(1), at(0, 0));
  assert.equal(m.pointCount, 1);
  const len = Math.hypot(f(m.normal.x), f(m.normal.y));
  assert.ok(Math.abs(len - 1) < 1e-9, 'normal must be unit length, not NaN');
});

test('collide: box on box face contact gives two points', () => {
  const m = new Manifold();
  const a = Polygon.box(1, 1);
  const b = Polygon.box(1, 1);
  collide(m, a, at(0, 0), b, at(0, 1.9));
  assert.equal(m.pointCount, 2, 'a flat face pair needs two points');
  assert.ok(Math.abs(f(m.normal.y) - 1) < 1e-9, 'normal points A -> B (upwards)');
  for (let i = 0; i < m.pointCount; i++) {
    assert.ok(Math.abs(f(m.points[i].separation) + 0.1) < 1e-6);
  }
});

test('collide: normal always points from A to B when the order is swapped', () => {
  const m1 = new Manifold();
  const m2 = new Manifold();
  const box = Polygon.box(1, 1);
  const circle = Circle.of(0.5);
  collide(m1, box, at(0, 0), circle, at(0, 1.2));
  collide(m2, circle, at(0, 1.2), box, at(0, 0));
  assert.equal(m1.pointCount, 1);
  assert.equal(m2.pointCount, 1);
  assert.ok(f(m1.normal.y) > 0, 'box -> circle points up');
  assert.ok(f(m2.normal.y) < 0, 'circle -> box points down');
});

test('collide: box resting on a segment produces two points', () => {
  const m = new Manifold();
  const ground = Segment.of(-10, 0, 10, 0);
  const box = Polygon.box(1, 1);
  collide(m, ground, at(0, 0), box, at(0, 0.95));
  assert.equal(m.pointCount, 2);
  assert.ok(Math.abs(f(m.normal.y) - 1) < 1e-9);
});

test('collide: capsule against a box', () => {
  const m = new Manifold();
  const box = Polygon.box(5, 0.5);
  const cap = Capsule.vertical(2, 0.4);
  collide(m, box, at(0, 0), cap, at(0, 1.35));
  assert.ok(m.pointCount >= 1, 'capsule should rest on the box');
  assert.ok(f(m.normal.y) > 0.99, `normal ${f(m.normal.y)}`);
});

test('collide: rotated boxes produce a sane vertex-face manifold', () => {
  const m = new Manifold();
  const ground = Polygon.box(5, 0.5);
  const diamond = Polygon.box(0.7, 0.7);
  collide(m, ground, at(0, 0), diamond, at(0, 1.4, Math.PI / 4));
  assert.ok(m.pointCount >= 1);
  assert.ok(f(m.normal.y) > 0.99, 'normal should be vertical');
  // the deepest point should sit near the bottom vertex of the diamond
  assert.ok(Math.abs(f(m.points[0].point.x)) < 0.2, 'contact near the corner');
});

test('collide: manifold ids are stable across small motions (warm starting)', () => {
  const m1 = new Manifold();
  const m2 = new Manifold();
  const ground = Polygon.box(5, 0.5);
  const box = Polygon.box(1, 1);
  collide(m1, ground, at(0, 0), box, at(0, 1.45));
  collide(m2, ground, at(0, 0), box, at(0.001, 1.451));
  assert.equal(m1.pointCount, m2.pointCount);
  for (let i = 0; i < m1.pointCount; i++) {
    assert.equal(m1.points[i].id, m2.points[i].id, 'feature ids must persist');
  }
});

/* ------------------------ distance / cast ----------------------- */

test('shapeDistance: separated boxes report the true gap', () => {
  const out = makeDistanceOutput();
  shapeDistance(out, {
    proxyA: makeProxy(Polygon.box(1, 1)),
    proxyB: makeProxy(Polygon.box(1, 1)),
    xfA: at(0, 0),
    xfB: at(5, 0),
    useRadii: false,
  });
  assert.ok(Math.abs(f(out.distance) - 3) < 1e-9, `distance ${f(out.distance)}`);
});

test('shapeDistance: overlapping shapes report zero', () => {
  const out = makeDistanceOutput();
  shapeDistance(out, {
    proxyA: makeProxy(Polygon.box(1, 1)),
    proxyB: makeProxy(Polygon.box(1, 1)),
    xfA: at(0, 0),
    xfB: at(0.5, 0),
    useRadii: false,
  });
  assert.ok(f(out.distance) < 1e-9);
});

test('shapeDistance: radii are subtracted when requested', () => {
  const out = makeDistanceOutput();
  shapeDistance(out, {
    proxyA: makeProxy(Circle.of(1)),
    proxyB: makeProxy(Circle.of(1)),
    xfA: at(0, 0),
    xfB: at(5, 0),
    useRadii: true,
  });
  assert.ok(Math.abs(f(out.distance) - 3) < 1e-9, `distance ${f(out.distance)}`);
});

test('shapeCast: finds the time of impact for a moving box', () => {
  const out = makeShapeCastOutput();
  const hit = shapeCast(out, {
    proxyA: makeProxy(Polygon.box(1, 1)),
    proxyB: makeProxy(Polygon.box(1, 1)),
    xfA: at(0, 0),
    xfB: at(10, 0),
    translationB: Vec2.of(-20, 0),
    maxFraction: S.ONE,
  });
  assert.ok(hit, 'the sweep should register a hit');
  // B must travel 8 of its 20 units before the faces touch
  assert.ok(Math.abs(f(out.fraction) - 0.4) < 0.01, `fraction ${f(out.fraction)}`);
});

test('shapeCast: a miss returns false', () => {
  const out = makeShapeCastOutput();
  const hit = shapeCast(out, {
    proxyA: makeProxy(Polygon.box(1, 1)),
    proxyB: makeProxy(Polygon.box(1, 1)),
    xfA: at(0, 0),
    xfB: at(10, 50),
    translationB: Vec2.of(-20, 0),
    maxFraction: S.ONE,
  });
  assert.ok(!hit);
});

/* --------------------------- broad phase ------------------------ */

test('DynamicTree: insert, query and validate', () => {
  const tree = new DynamicTree(16);
  const boxes = [];
  for (let i = 0; i < 50; i++) {
    const b = new AABB().set(
      S.fromFloat(i * 2),
      S.fromFloat(0),
      S.fromFloat(i * 2 + 1),
      S.fromFloat(1),
    );
    boxes.push(tree.createProxy(b, i));
  }
  assert.equal(tree.proxyCount, 50);
  assert.equal(tree.validate(), null, 'tree must be structurally sound');

  const found = [];
  const q = new AABB().set(S.fromFloat(-1), S.fromFloat(-1), S.fromFloat(7), S.fromFloat(2));
  tree.query(q, (_id, data) => {
    found.push(data);
    return true;
  });
  found.sort((a, b) => a - b);
  assert.deepEqual(found, [0, 1, 2, 3]);

  tree.destroyProxy(boxes[0]);
  assert.equal(tree.proxyCount, 49);
  assert.equal(tree.validate(), null);
});

test('DynamicTree: stays balanced under many insertions', () => {
  const tree = new DynamicTree(16);
  for (let i = 0; i < 1000; i++) {
    const x = (i * 37) % 100;
    const y = (i * 61) % 100;
    const b = new AABB().set(
      S.fromFloat(x),
      S.fromFloat(y),
      S.fromFloat(x + 1),
      S.fromFloat(y + 1),
    );
    tree.createProxy(b, i);
  }
  assert.equal(tree.validate(), null);
  // a perfectly balanced tree over 1000 leaves is 10 deep; allow generous slack
  assert.ok(tree.getHeight() < 30, `height ${tree.getHeight()}`);
});

test('DynamicTree: ray cast reports only the crossed proxies', () => {
  const tree = new DynamicTree(16);
  for (let i = 0; i < 10; i++) {
    const b = new AABB().set(
      S.fromFloat(i * 2),
      S.fromFloat(-0.5),
      S.fromFloat(i * 2 + 1),
      S.fromFloat(0.5),
    );
    tree.createProxy(b, i);
  }
  const hits = [];
  tree.rayCast(Vec2.of(-1, 0), Vec2.of(21, 0), S.ONE, (_id, data, _p1, _p2, maxF) => {
    hits.push(data);
    return maxF;
  });
  assert.equal(hits.length, 10, 'a ray along the row crosses every box');
});

test('DynamicTree: moveProxy is a no-op while inside the fat AABB', () => {
  const tree = new DynamicTree(16);
  // createProxy stores the box verbatim, so pad it the way BroadPhase does.
  const box = new AABB().set(S.ZERO, S.ZERO, S.ONE, S.ONE).expand(S.fromFloat(0.1));
  const id = tree.createProxy(box, 0);
  const tiny = new AABB().set(
    S.fromFloat(0.001),
    S.fromFloat(0.001),
    S.fromFloat(1.001),
    S.fromFloat(1.001),
  );
  assert.equal(tree.moveProxy(id, tiny, S.fromFloat(0.1), Vec2.zero()), false);
  const far = new AABB().set(S.fromFloat(50), S.fromFloat(50), S.fromFloat(51), S.fromFloat(51));
  assert.equal(tree.moveProxy(id, far, S.fromFloat(0.1), Vec2.zero()), true);
});

test('BroadPhase: reports each overlapping pair exactly once', () => {
  const bp = new BroadPhase(16);
  const a = new AABB().set(S.ZERO, S.ZERO, S.fromFloat(2), S.fromFloat(2));
  const b = new AABB().set(S.ONE, S.ONE, S.fromFloat(3), S.fromFloat(3));
  const c = new AABB().set(S.fromFloat(50), S.fromFloat(50), S.fromFloat(51), S.fromFloat(51));
  bp.createProxy(a, 0);
  bp.createProxy(b, 1);
  bp.createProxy(c, 2);

  const pairs = [];
  bp.updatePairs((x, y) => pairs.push(`${x}-${y}`));
  assert.deepEqual(pairs, ['0-1']);

  // nothing moved, so there is nothing new to report
  const again = [];
  bp.updatePairs((x, y) => again.push(`${x}-${y}`));
  assert.equal(again.length, 0);
});

test('BroadPhase: pair order is canonical (low id first)', () => {
  const bp = new BroadPhase(16);
  const overlap = () => new AABB().set(S.ZERO, S.ZERO, S.fromFloat(2), S.fromFloat(2));
  bp.createProxy(overlap(), 7);
  bp.createProxy(overlap(), 3);
  const pairs = [];
  bp.updatePairs((x, y) => pairs.push([x, y]));
  assert.equal(pairs.length, 1);
  assert.deepEqual(pairs[0], [3, 7]);
});
