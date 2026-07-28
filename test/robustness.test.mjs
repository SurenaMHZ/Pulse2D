/**
 * Robustness — hostile and degenerate input.
 *
 * The engine is only as good as its behaviour when a game feeds it something
 * unexpected: a `NaN` from a bad normalise, a zero-density fixture, a dropped
 * frame that produces a non-finite delta. Each test here pins a bug that was
 * found by fuzzing the public API and is now fixed.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  World,
  BodyType,
  Circle,
  Capsule,
  Polygon,
  Vec2,
  Rng,
  ChainShape,
  DynamicTree,
  AABB,
  Transform,
  Rot,
  saveSnapshot,
  loadSnapshot,
  cloneSnapshot,
  checksumWorld,
  RollbackManager,
  Scalar as S,
} from '../dist/pulse2d.mjs';

const f = S.toFloat;

/** A world with a wide floor whose top surface sits at y = 0. */
function grounded(friction = 0.7) {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(30, 1), friction });
  return world;
}
const steps = (w, n) => { for (let i = 0; i < n; i++) w.step(); };
const allFinite = (w) => {
  for (const b of w.eachBody()) {
    if (!Number.isFinite(f(b.getPosition().x)) || !Number.isFinite(f(b.getPosition().y))) return false;
  }
  return true;
};

/* ------------------------- zero-density mass ------------------------ */

test('a zero-density dynamic body still gets unit mass', () => {
  // Regression: addFixture only called resetMassData() when density > 0, so a
  // zero-density body kept mass = invMass = 0 — infinitely heavy. It fell
  // under gravity but no contact impulse could stop it, and it sank straight
  // through the floor.
  const world = grounded();
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  body.addFixture({ shape: Circle.of(0.3), density: 0 });

  assert.ok(f(body.mass) > 0, `mass should fall back to 1, got ${f(body.mass)}`);
  assert.ok(f(body.invMass) > 0, 'invMass must be non-zero or nothing can push the body');

  steps(world, 250);
  assert.ok(
    Math.abs(f(body.getPosition().y) - 0.3) < 0.05,
    `zero-density body should rest on the floor, got y=${f(body.getPosition().y)}`,
  );
});

test('a body with no fixtures at all still responds to forces', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  steps(world, 60);
  assert.ok(f(body.getPosition().y) < 5, 'shapeless dynamic body should fall');
});

/* ----------------------------- accumulate --------------------------- */

test('accumulate ignores non-finite and non-positive deltas', () => {
  // Regression: `Math.max(0, NaN)` is NaN, so one bad frame time poisoned the
  // accumulator permanently and the world silently never stepped again.
  const world = new World({ timeStep: 1 / 60, gravity: { x: 0, y: -10 } });
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  body.addFixture({ shape: Circle.of(0.3), density: 1 });

  for (const bad of [NaN, Infinity, -Infinity, -5, 0]) {
    const alpha = world.accumulate(bad);
    assert.equal(world.tick, 0, `accumulate(${bad}) must not step`);
    assert.ok(Number.isFinite(alpha), `accumulate(${bad}) returned ${alpha}`);
  }

  // …and the world must still work afterwards.
  for (let i = 0; i < 60; i++) world.accumulate(1 / 60);
  assert.equal(world.tick, 60, 'world must recover from bad frame times');
  assert.ok(f(body.getPosition().y) < 5, 'body should have fallen');
});

/* ------------------------ NaN through the API ----------------------- */

for (const [label, bad] of [['NaN', NaN], ['Infinity', Infinity], ['-Infinity', -Infinity]]) {
  test(`${label} injected through the Body API is rejected`, () => {
    // A single NaN velocity is catastrophic *and silent*: it spreads into the
    // position, then into every contact, and the visible symptom is a body
    // that stops responding rather than an error anyone can trace.
    const world = grounded();
    const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
    body.addFixture({ shape: Circle.of(0.3), density: 1 });
    steps(world, 30);

    body.setLinearVelocity(bad, 0);
    body.setAngularVelocity(bad);
    body.applyLinearImpulse(bad, bad);
    body.applyAngularImpulse(bad);
    body.applyForce(bad, bad);
    body.applyTorque(bad);
    body.setTransform(bad, 2, 0);
    body.setMassData(bad, bad);

    steps(world, 60);
    assert.ok(allFinite(world), `${label} leaked into the simulation`);
    assert.ok(Number.isFinite(f(body.linearVelocity.x)), `${label} corrupted the velocity`);
    assert.ok(Number.isFinite(f(body.angularVelocity)), `${label} corrupted the spin`);
  });
}

test('a NaN gravity vector is caught at the start of the step', () => {
  // `gravity` is a public mutable Vec2 with no setter to validate, so it is
  // checked once per step instead.
  const world = grounded();
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
  body.addFixture({ shape: Circle.of(0.3), density: 1 });

  world.gravity.set(S.ZERO, S.fromFloat(NaN));
  steps(world, 40);
  assert.ok(allFinite(world), 'NaN gravity poisoned the world');

  // Recovering by setting a real value must work.
  world.gravity.set(S.ZERO, S.fromFloat(-10));
  world.wakeAll();
  steps(world, 250);
  assert.ok(Math.abs(f(body.getPosition().y) - 0.3) < 0.06, 'world should recover and settle');
});

test('one corrupted body cannot infect its neighbour', () => {
  const world = grounded();
  const victim = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 } });
  victim.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  const neighbour = world.createBody({ type: BodyType.Dynamic, position: { x: 1.2, y: 0.5 } });
  neighbour.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  steps(world, 60);

  victim.setLinearVelocity(NaN, NaN);
  steps(world, 60);
  assert.ok(Number.isFinite(f(neighbour.getPosition().y)), 'corruption spread to the neighbour');
});

/* ---------------------------- lifecycle ----------------------------- */

test('destroying every dynamic body mid-simulation is clean', () => {
  const world = grounded();
  const doomed = [];
  for (let i = 0; i < 10; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: i * 0.5 - 2, y: 2 + i } });
    b.addFixture({ shape: Circle.of(0.25), density: 1 });
    doomed.push(b);
  }
  steps(world, 100);
  for (const b of doomed) world.destroyBody(b);
  steps(world, 50);

  assert.equal(world.bodyCount, 1, 'only the ground should remain');
  assert.equal(world.contactCount, 0, 'contacts must go with their bodies');
});

test('body and fixture ids are reused safely', () => {
  const world = grounded();
  const a = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
  a.addFixture({ shape: Circle.of(0.3), density: 1 });
  const oldBodyId = a.id;
  const oldFixtureId = a.fixtures[0].id;
  steps(world, 30);
  world.destroyBody(a);

  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1 });
  assert.equal(b.id, oldBodyId, 'ids should be recycled');
  assert.equal(b.fixtures[0].id, oldFixtureId);

  steps(world, 250);
  assert.ok(Math.abs(f(b.getPosition().y) - 0.3) < 0.05, 'recycled body must simulate normally');
});

test('removing a fixture at runtime keeps the body sane', () => {
  const world = grounded();
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
  body.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  const second = body.addFixture({ shape: Polygon.offsetBox(0.3, 0.3, 1, 0), density: 1 });
  steps(world, 60);

  body.removeFixture(second);
  steps(world, 60);
  assert.equal(body.fixtures.length, 1);
  assert.ok(Number.isFinite(f(body.getPosition().y)));
});

test('clear() leaves the world reusable', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  for (let round = 0; round < 3; round++) {
    const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
    g.addFixture({ shape: Polygon.box(20, 1) });
    for (let i = 0; i < 10; i++) {
      const b = world.createBody({ type: BodyType.Dynamic, position: { x: i - 5, y: 2 + i } });
      b.addFixture({ shape: Circle.of(0.25), density: 1 });
    }
    steps(world, 100);
    assert.ok(allFinite(world), `round ${round} destabilised`);
    world.clear();
    assert.equal(world.bodyCount, 0);
  }
});

/* -------------------------- degenerate input ------------------------ */

test('degenerate polygons throw rather than produce garbage', () => {
  assert.throws(() => new Polygon([Vec2.of(0, 0), Vec2.of(1, 0), Vec2.of(2, 0)]), /3/);
  assert.throws(() => new Polygon([Vec2.of(1, 1), Vec2.of(1, 1), Vec2.of(1, 1)]));
  assert.throws(() => new Polygon([Vec2.of(0, 0), Vec2.of(1, 1)]));
});

test('extreme mass ratios stay stable', () => {
  const world = grounded(0.6);
  const big = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 6 } });
  big.addFixture({ shape: Polygon.box(4, 4), density: 5 });
  const tiny = world.createBody({ type: BodyType.Dynamic, position: { x: 3, y: 1 } });
  tiny.addFixture({ shape: Circle.of(0.03), density: 1 });

  steps(world, 400);
  assert.ok(allFinite(world), 'a ~1e6 mass ratio blew up');
  assert.ok(Math.abs(f(tiny.getPosition().y)) < 50);
});

test('bodies spawned exactly on top of each other separate calmly', () => {
  const world = grounded();
  const bodies = [];
  for (let i = 0; i < 12; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1 } });
    b.addFixture({ shape: Circle.of(0.4), density: 1 });
    bodies.push(b);
  }
  steps(world, 600);
  assert.ok(allFinite(world), 'coincident spawn exploded');
  for (const b of bodies) {
    assert.ok(Math.abs(f(b.getPosition().x)) < 40, 'body was ejected across the level');
  }
});

test('a huge impulse does not produce a non-finite state', () => {
  const world = grounded();
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
  body.addFixture({ shape: Circle.of(0.3), density: 1 });
  body.applyLinearImpulse(1e9, 1e9);
  steps(world, 120);
  assert.ok(allFinite(world));
});

test('a scene far from the origin still rests correctly', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 5000, y: -1 } });
  g.addFixture({ shape: Polygon.box(30, 1), friction: 0.7 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 5000, y: 3 } });
  b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  steps(world, 400);
  assert.ok(Math.abs(f(b.getPosition().y) - 0.5) < 0.05, `y=${f(b.getPosition().y)}`);
});

/* --------------------------- body type churn ------------------------ */

test('cycling through every body type is safe', () => {
  const world = grounded();
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  body.addFixture({ shape: Polygon.box(0.4, 0.4), density: 1 });

  for (const type of [BodyType.Static, BodyType.Kinematic, BodyType.Dynamic,
                      BodyType.Kinematic, BodyType.Static, BodyType.Dynamic]) {
    body.setType(type);
    steps(world, 40);
  }
  assert.ok(f(body.mass) > 0, 'mass lost during type churn');
  assert.ok(Number.isFinite(f(body.getPosition().y)));
});

/* ----------------------- physical correctness ----------------------- */

test('momentum is conserved in an elastic collision', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const a = world.createBody({ type: BodyType.Dynamic, position: { x: -3, y: 0 }, linearVelocity: { x: 5, y: 0 } });
  a.addFixture({ shape: Circle.of(0.4), density: 1, restitution: 1, friction: 0 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 3, y: 0 } });
  b.addFixture({ shape: Circle.of(0.4), density: 1, restitution: 1, friction: 0 });

  const before = f(a.mass) * 5;
  steps(world, 400);
  const after = f(a.mass) * f(a.linearVelocity.x) + f(b.mass) * f(b.linearVelocity.x);
  assert.ok(Math.abs(after - before) / before < 0.05, `momentum ${before} -> ${after}`);
});

test('a pendulum conserves energy over 2000 steps', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, subSteps: 8 });
  const L = 4;
  const angle = -1.0;
  const pivot = world.createBody({ type: BodyType.Static, position: { x: 0, y: 10 } });
  const bob = world.createBody({
    type: BodyType.Dynamic,
    position: { x: Math.sin(angle) * L, y: 10 - Math.cos(angle) * L },
  });
  bob.addFixture({ shape: Circle.of(0.2), density: 5 });
  world.createDistanceJoint({ bodyA: pivot, bodyB: bob, length: L });

  const mass = f(bob.mass);
  const energy = () => 10 * mass * f(bob.getPosition().y) + 0.5 * mass * f(bob.linearVelocity.lengthSq());
  const e0 = energy();
  let lo = e0, hi = e0;
  for (let i = 0; i < 2000; i++) {
    world.step();
    const e = energy();
    lo = Math.min(lo, e);
    hi = Math.max(hi, e);
  }
  assert.ok((hi - lo) / Math.abs(e0) < 0.03, `energy drifted ${(((hi - lo) / e0) * 100).toFixed(1)}%`);
});

test('the friction cone matches the analytic slope condition', () => {
  const slide = (angleDeg, mu) => {
    const world = new World({ gravity: { x: 0, y: -10 } });
    const a = (angleDeg * Math.PI) / 180;
    const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 }, angle: a });
    g.addFixture({ shape: Polygon.box(20, 0.5), friction: mu });
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: -Math.sin(a) * 0.9, y: Math.cos(a) * 0.9 },
      angle: a,
    });
    b.addFixture({ shape: Polygon.box(0.4, 0.4), density: 1, friction: mu });
    const x0 = f(b.getPosition().x);
    steps(world, 600);
    return Math.abs(f(b.getPosition().x) - x0);
  };
  assert.ok(slide(15, 0.8) < 0.2, 'tan(15°)=0.27 < µ=0.8, the box must hold');
  assert.ok(slide(45, 0.2) > 1.5, 'tan(45°)=1.0 > µ=0.2, the box must slide');
});

test('bounce height follows e² over a range of speeds', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, subSteps: 8 });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(30, 1) });
  const h0 = 5;
  const e = 0.8;
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: h0 } });
  ball.addFixture({ shape: Circle.of(0.25), density: 1, restitution: e, friction: 0 });

  let landed = false;
  let peak = -Infinity;
  let prev = h0;
  for (let i = 0; i < 600; i++) {
    world.step();
    const y = f(ball.getPosition().y);
    if (!landed && y < 0.35) landed = true;
    if (landed && y > prev) peak = Math.max(peak, y);
    if (landed && y < prev && peak > 0) break;
    prev = y;
  }
  const expected = (h0 - 0.25) * e * e;
  assert.ok(Math.abs(peak - 0.25 - expected) / expected < 0.25,
    `bounced to ${peak.toFixed(2)}, expected ~${(expected + 0.25).toFixed(2)}`);
});

test('bullets are stopped at 100, 500 and 2000 m/s', () => {
  for (const speed of [100, 500, 2000]) {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const wall = world.createBody({ type: BodyType.Static, position: { x: 10, y: 0 } });
    wall.addFixture({ shape: Polygon.box(0.05, 5) });
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: -10, y: 0 },
      linearVelocity: { x: speed, y: 0 },
      bullet: true,
    });
    b.addFixture({ shape: Circle.of(0.15), density: 1 });
    steps(world, 120);
    assert.ok(f(b.getPosition().x) < 11, `bullet at ${speed} m/s tunnelled to ${f(b.getPosition().x)}`);
  }
});

/* ------------------------- determinism churn ------------------------ */

test('determinism survives a stream of spawns and destroys', () => {
  const run = (seed) => {
    const world = new World({ gravity: { x: 0, y: -10 }, seed });
    const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
    g.addFixture({ shape: Polygon.box(30, 1), friction: 0.6 });
    const rng = new Rng(seed);
    const live = [];
    for (let t = 0; t < 500; t++) {
      if (t % 7 === 0 && live.length < 40) {
        const b = world.createBody({ type: BodyType.Dynamic, position: { x: rng.float() * 10 - 5, y: 8 } });
        b.addFixture({
          shape: rng.int(0, 1) ? Circle.of(0.25) : Polygon.box(0.25, 0.25),
          density: 1, friction: 0.5,
        });
        live.push(b);
      }
      if (t % 23 === 0 && live.length > 5) world.destroyBody(live.shift());
      world.step();
    }
    return checksumWorld(world);
  };
  assert.equal(run(42), run(42));
});

test('determinism survives runtime setting changes', () => {
  const run = () => {
    const world = new World({ gravity: { x: 0, y: -10 }, seed: 3 });
    const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
    g.addFixture({ shape: Polygon.box(30, 1), friction: 0.6 });
    const rng = new Rng(3);
    for (let i = 0; i < 25; i++) {
      const b = world.createBody({ type: BodyType.Dynamic, position: { x: rng.float() * 8 - 4, y: 1 + i * 0.6 } });
      b.addFixture({ shape: Polygon.box(0.25, 0.25), density: 1, friction: 0.5 });
    }
    for (let t = 0; t < 400; t++) {
      if (t === 100) world.subSteps = 8;
      if (t === 200) world.gravity.set(S.ZERO, S.fromFloat(-4));
      if (t === 300) world.enableSleep = false;
      world.step();
    }
    return checksumWorld(world);
  };
  assert.equal(run(), run());
});

test('a busy scene stays bounded over 10 000 steps', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, seed: 77 });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(30, 1), friction: 0.6 });
  for (const x of [-12, 12]) {
    const wall = world.createBody({ type: BodyType.Static, position: { x, y: 6 } });
    wall.addFixture({ shape: Polygon.box(0.5, 7) });
  }
  const rng = new Rng(77);
  for (let i = 0; i < 60; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 20 - 10, y: 2 + rng.float() * 12 },
      angle: rng.float() * 6,
      angularVelocity: rng.float() * 4 - 2,
    });
    const k = rng.int(0, 2);
    if (k === 0) b.addFixture({ shape: Circle.of(0.28), density: 1, restitution: 0.5, friction: 0.4 });
    else if (k === 1) b.addFixture({ shape: Polygon.box(0.28, 0.28), density: 1, friction: 0.5 });
    else b.addFixture({ shape: Capsule.vertical(0.7, 0.18), density: 1, friction: 0.4 });
  }

  steps(world, 10000);
  assert.ok(allFinite(world), 'long run produced a non-finite state');
  for (const b of world.eachBody()) {
    assert.ok(Math.abs(f(b.getPosition().y)) < 60, 'a body escaped the arena');
  }
});

/* --------------------------- snapshot depth ------------------------- */

test('snapshots replay exactly with all five joint types', () => {
  const build = () => {
    const world = new World({ gravity: { x: 0, y: -10 }, seed: 8 });
    const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
    g.addFixture({ shape: Polygon.box(30, 1), friction: 0.7 });
    const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 10 } });
    const mk = (x) => {
      const b = world.createBody({ type: BodyType.Dynamic, position: { x, y: 7 } });
      b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1 });
      return b;
    };
    world.createRevoluteJointAt(anchor, mk(-6), -6, 10);
    world.createDistanceJoint({ bodyA: anchor, bodyB: mk(-3), length: 3 });
    world.createPrismaticJoint({
      bodyA: anchor, bodyB: mk(0),
      localAxisA: { x: 1, y: 0 }, enableLimit: true, lowerLimit: -2, upperLimit: 2,
    });
    world.createWeldJoint({
      bodyA: anchor, bodyB: mk(3),
      localAnchorA: { x: 3, y: -3 }, linearHertz: 5, angularHertz: 5,
    });
    world.createMotorJoint({ bodyA: anchor, bodyB: mk(6), maxForce: 300, maxTorque: 300 });
    return world;
  };

  const world = build();
  steps(world, 200);
  const snap = cloneSnapshot(saveSnapshot(world));
  const expected = [];
  for (let i = 0; i < 120; i++) { world.step(); expected.push(checksumWorld(world)); }

  loadSnapshot(world, snap);
  for (let i = 0; i < 120; i++) {
    world.step();
    assert.equal(checksumWorld(world), expected[i], `replay diverged at tick ${i}`);
  }
});

test('repeated save/load cycles are idempotent', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, seed: 33 });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(20, 1), friction: 0.7 });
  for (let i = 0; i < 10; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: i - 5, y: 1 + i * 0.8 } });
    b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.6 });
  }
  steps(world, 150);

  const reference = checksumWorld(world);
  let snap = cloneSnapshot(saveSnapshot(world));
  for (let i = 0; i < 100; i++) {
    loadSnapshot(world, snap);
    snap = cloneSnapshot(saveSnapshot(world));
  }
  assert.equal(checksumWorld(world), reference, '100 save/load cycles drifted');
});

test('loading a snapshot after a structural change does not crash', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, seed: 5 });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(20, 1) });
  const bodies = [];
  for (let i = 0; i < 6; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: i - 3, y: 1 + i } });
    b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1 });
    bodies.push(b);
  }
  steps(world, 100);

  const snap = cloneSnapshot(saveSnapshot(world));
  world.destroyBody(bodies[2]);
  const extra = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 8 } });
  extra.addFixture({ shape: Circle.of(0.25), density: 1 });
  steps(world, 50);

  // The structure no longer matches the snapshot; this must degrade
  // gracefully rather than throw or produce NaN.
  loadSnapshot(world, snap);
  steps(world, 50);
  assert.ok(allFinite(world));
});

/* ---------------------------- query contract ------------------------ */

test('ray cast callback return codes behave as documented', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  for (const x of [2, 4, 6, 8]) {
    const b = world.createBody({ type: BodyType.Static, position: { x, y: 0 } });
    b.addFixture({ shape: Polygon.box(0.4, 0.4), userData: `b${x}` });
  }

  const all = [];
  world.rayCast(-5, 0, 20, 0, (fx) => { all.push(fx.userData); return S.ONE; });
  assert.equal(all.length, 4, 'returning 1 should report every fixture');

  const stopped = [];
  world.rayCast(-5, 0, 20, 0, (fx) => { stopped.push(fx.userData); return S.ZERO; });
  assert.equal(stopped.length, 1, 'returning 0 should stop immediately');

  const ignored = [];
  world.rayCast(-5, 0, 20, 0, (fx) => { ignored.push(fx.userData); return S.NEG_ONE; });
  assert.equal(ignored.length, 4, 'returning -1 should keep searching');
});

test('queries honour an early exit', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  for (let i = 0; i < 8; i++) {
    const b = world.createBody({ type: BodyType.Static, position: { x: i * 2, y: 0 } });
    b.addFixture({ shape: Polygon.box(0.4, 0.4), userData: i });
  }
  let visited = 0;
  world.queryAABB(-1, -1, 20, 1, () => { visited++; return false; });
  assert.equal(visited, 1);
});

/* ------------------------------ math API ---------------------------- */

test('Vec2 destination helpers tolerate aliasing', () => {
  const a = Vec2.of(3, 4);
  Vec2.addTo(a, a, a);
  assert.equal(f(a.x), 6);
  assert.equal(f(a.y), 8);

  const b = Vec2.of(1, 2);
  Vec2.perpTo(b, b);
  assert.equal(f(b.x), -2);
  assert.equal(f(b.y), 1);
});

test('Rot stays normalised at extreme angles', () => {
  for (const angle of [1e6, -1e6, 1e9, -1e9, 12345.6789]) {
    const r = Rot.of(angle);
    assert.ok(Math.abs(Math.hypot(f(r.s), f(r.c)) - 1) < 1e-9, `angle ${angle} denormalised`);
  }
});

test('Transform apply/applyT round-trips to full precision', () => {
  const rng = new Rng(1);
  const out = Vec2.zero();
  for (let i = 0; i < 500; i++) {
    const xf = new Transform(
      Vec2.of(rng.float() * 100 - 50, rng.float() * 100 - 50),
      Rot.of(rng.float() * 20 - 10),
    );
    const p = Vec2.of(rng.float() * 20 - 10, rng.float() * 20 - 10);
    Transform.apply(out, xf, p);
    Transform.applyT(out, xf, out);
    assert.ok(Math.abs(f(out.x) - f(p.x)) < 1e-10);
    assert.ok(Math.abs(f(out.y) - f(p.y)) < 1e-10);
  }
});

test('the dynamic tree stays valid through heavy churn', () => {
  const tree = new DynamicTree(8);
  const rng = new Rng(4);
  const ids = [];
  for (let i = 0; i < 300; i++) {
    const x = rng.float() * 100;
    const y = rng.float() * 100;
    ids.push(tree.createProxy(
      new AABB().set(S.fromFloat(x), S.fromFloat(y), S.fromFloat(x + 1), S.fromFloat(y + 1)), i,
    ));
  }
  for (let i = 0; i < 150; i++) tree.destroyProxy(ids[i]);
  for (let i = 0; i < 100; i++) {
    const x = rng.float() * 100;
    const y = rng.float() * 100;
    tree.createProxy(
      new AABB().set(S.fromFloat(x), S.fromFloat(y), S.fromFloat(x + 1), S.fromFloat(y + 1)), 1000 + i,
    );
  }
  assert.equal(tree.validate(), null);
  assert.equal(tree.proxyCount, 250);
});

/* ---------------------- joint chain fidelity ------------------------ */

/** Largest gap between any joint's two anchors — its visible "stretch". */
function worstJointStretch(world) {
  const a = Vec2.zero();
  const b = Vec2.zero();
  let worst = 0;
  for (const j of world.eachJoint()) {
    j.getAnchorA(a);
    j.getAnchorB(b);
    worst = Math.max(worst, Math.hypot(f(a.x) - f(b.x), f(a.y) - f(b.y)));
  }
  return worst;
}

test('a swinging chain of revolute joints does not stretch', () => {
  // Regression: anchor arms were computed once in prepare() and never
  // refreshed. A chain link swings tens of degrees within a step, so the stale
  // arm aimed the correction the wrong way — links visibly pulled apart and
  // jittered, and raising subSteps did not help because the *direction* was
  // wrong, not the magnitude.
  const world = new World({ gravity: { x: 0, y: -10 } });
  let prev = world.createBody({ type: BodyType.Static, position: { x: 0, y: 17 } });
  for (let i = 0; i < 11; i++) {
    const link = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 17 - (i + 1) * 0.62 } });
    link.addFixture({ shape: Polygon.box(0.09, 0.3), density: 1, friction: 0.4, filter: { group: -1 } });
    world.createRevoluteJointAt(prev, link, 0, 17 - i * 0.62 - 0.31);
    prev = link;
  }
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 17 - 12 * 0.62 } });
  ball.addFixture({ shape: Circle.of(0.4), density: 4, filter: { group: -1 } });
  world.createRevoluteJointAt(prev, ball, 0, 17 - 11 * 0.62 - 0.31);

  ball.applyLinearImpulse(f(ball.mass) * 6, 0);   // set it swinging

  let worst = 0;
  for (let i = 0; i < 600; i++) {
    world.step();
    worst = Math.max(worst, worstJointStretch(world));
  }
  assert.ok(worst < 0.05, `chain joints stretched to ${worst.toFixed(4)} m`);
});

test('a horizontal chain under full tension holds together', () => {
  // The worst case for a chain: straight out sideways, so every joint carries
  // the full weight of everything beyond it.
  const world = new World({ gravity: { x: 0, y: -10 } });
  let prev = world.createBody({ type: BodyType.Static, position: { x: 0, y: 17 } });
  for (let i = 0; i < 11; i++) {
    const link = world.createBody({ type: BodyType.Dynamic, position: { x: (i + 1) * 0.62, y: 17 } });
    link.addFixture({ shape: Polygon.box(0.09, 0.3), density: 1, filter: { group: -1 } });
    world.createRevoluteJointAt(prev, link, i * 0.62, 17);
    prev = link;
  }
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 12 * 0.62, y: 17 } });
  ball.addFixture({ shape: Circle.of(0.4), density: 4, filter: { group: -1 } });
  world.createRevoluteJointAt(prev, ball, 11 * 0.62, 17);

  let worst = 0;
  for (let i = 0; i < 400; i++) {
    world.step();
    worst = Math.max(worst, worstJointStretch(world));
  }
  assert.ok(worst < 0.06, `tensioned chain stretched to ${worst.toFixed(4)} m`);
});

test('more sub-steps make a chain tighter, not looser', () => {
  // The diagnostic that exposed the bug: with stale anchors the error was
  // insensitive to subSteps, proving it was structural rather than unconverged.
  const stretchFor = (subSteps) => {
    const world = new World({ gravity: { x: 0, y: -10 }, subSteps });
    let prev = world.createBody({ type: BodyType.Static, position: { x: 0, y: 17 } });
    for (let i = 0; i < 11; i++) {
      const link = world.createBody({ type: BodyType.Dynamic, position: { x: (i + 1) * 0.62, y: 17 } });
      link.addFixture({ shape: Polygon.box(0.09, 0.3), density: 1, filter: { group: -1 } });
      world.createRevoluteJointAt(prev, link, i * 0.62, 17);
      prev = link;
    }
    const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 12 * 0.62, y: 17 } });
    ball.addFixture({ shape: Circle.of(0.4), density: 4, filter: { group: -1 } });
    world.createRevoluteJointAt(prev, ball, 11 * 0.62, 17);
    let worst = 0;
    for (let i = 0; i < 400; i++) { world.step(); worst = Math.max(worst, worstJointStretch(world)); }
    return worst;
  };
  const coarse = stretchFor(2);
  const fine = stretchFor(16);
  assert.ok(fine <= coarse, `subSteps 16 (${fine.toFixed(4)}) should beat 2 (${coarse.toFixed(4)})`);
});

test('a loaded rope bridge sags but does not come apart', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(60, 1) });

  const N = 18;
  const span = 13;
  const y = 7;
  let prev = world.createBody({ type: BodyType.Static, position: { x: -span / 2, y } });
  const planks = [];
  for (let i = 0; i < N; i++) {
    const x = -span / 2 + (i + 0.5) * (span / N);
    const p = world.createBody({ type: BodyType.Dynamic, position: { x, y } });
    p.addFixture({ shape: Polygon.box((span / N / 2) * 0.92, 0.09), density: 2, friction: 0.8 });
    world.createRevoluteJointAt(prev, p, x - span / N / 2, y);
    prev = p;
    planks.push(p);
  }
  const right = world.createBody({ type: BodyType.Static, position: { x: span / 2, y } });
  world.createRevoluteJointAt(prev, right, span / 2, y);
  for (let i = 0; i < 4; i++) {
    const crate = world.createBody({ type: BodyType.Dynamic, position: { x: -3 + i * 2, y: 9.5 } });
    crate.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1.2, friction: 0.8 });
  }

  let worst = 0;
  for (let i = 0; i < 900; i++) {
    world.step();
    worst = Math.max(worst, worstJointStretch(world));
  }
  assert.ok(worst < 0.05, `bridge joints stretched to ${worst.toFixed(4)} m`);
  const sag = y - Math.min(...planks.map((p) => f(p.getPosition().y)));
  assert.ok(sag > 0.1 && sag < 5, `bridge sag ${sag.toFixed(2)} m is implausible`);
});

/* --------------------- Newton's cradle physics ---------------------- */

test('a spaced row of balls transmits a shock end to end', () => {
  // Momentum transfer through a chain of balls: strike one end and only the
  // far ball should leave, at nearly the incoming speed.
  const world = new World({ gravity: { x: 0, y: 0 }, subSteps: 8 });
  const R = 0.5;
  const gap = 0.04;
  const balls = [];
  for (let i = 0; i < 5; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: i === 0 ? -3 : (i - 1) * (2 * R + gap), y: 0 },
    });
    b.addFixture({ shape: Circle.of(R), density: 12, restitution: 1, friction: 0 });
    balls.push(b);
  }
  balls[0].setLinearVelocity(5, 0);
  steps(world, 400);

  const v = balls.map((b) => f(b.linearVelocity.x));
  const mass = f(balls[0].mass);
  const energy = v.reduce((acc, x) => acc + 0.5 * mass * x * x, 0) / (0.5 * mass * 25);

  assert.ok(v[4] > 4, `far ball should carry the shock, got ${v[4].toFixed(2)}`);
  for (let i = 0; i < 4; i++) {
    assert.ok(Math.abs(v[i]) < 0.5, `ball ${i} should be near rest, got ${v[i].toFixed(2)}`);
  }
  assert.ok(energy < 1.05, `restitution added energy: E/E0 = ${energy.toFixed(3)}`);
});

test('a two-ball elastic collision transfers velocity exactly', () => {
  const world = new World({ gravity: { x: 0, y: 0 }, subSteps: 4 });
  const a = world.createBody({ type: BodyType.Dynamic, position: { x: -2, y: 0 }, linearVelocity: { x: 5, y: 0 } });
  a.addFixture({ shape: Circle.of(0.5), density: 12, restitution: 1, friction: 0 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 } });
  b.addFixture({ shape: Circle.of(0.5), density: 12, restitution: 1, friction: 0 });

  steps(world, 200);
  assert.ok(Math.abs(f(a.linearVelocity.x)) < 0.1, `striker should stop, got ${f(a.linearVelocity.x)}`);
  assert.ok(Math.abs(f(b.linearVelocity.x) - 5) < 0.2, `target should take the speed, got ${f(b.linearVelocity.x)}`);
});
