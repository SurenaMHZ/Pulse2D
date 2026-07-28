/**
 * Conservation laws — the engine must not invent energy or momentum.
 *
 * A physics engine that quietly adds energy looks fine in a screenshot and
 * falls apart after a minute of play: stacks creep, ragdolls twitch, piles
 * slowly launch themselves apart. These tests pin the invariants that stop
 * that happening, and they are written as *bounds* rather than exact equalities
 * because a discrete solver is allowed to lose a little energy — it is only
 * ever forbidden from gaining it.
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
  Scalar as S,
} from '../dist/pulse2d.mjs';

const f = S.toFloat;
const steps = (w, n) => { for (let i = 0; i < n; i++) w.step(); };

/** Total mechanical energy: kinetic + rotational + gravitational potential. */
function energy(world, g = 10) {
  let total = 0;
  for (const b of world.eachBody()) {
    if (b.type !== BodyType.Dynamic) continue;
    const mass = f(b.mass);
    const inertia = f(b.inertia);
    const omega = f(b.angularVelocity);
    total += 0.5 * mass * f(b.linearVelocity.lengthSq());
    total += 0.5 * inertia * omega * omega;
    total += mass * g * f(b.worldCenter.y);
  }
  return total;
}

/** Total linear momentum of the dynamic bodies. */
function momentum(world) {
  let px = 0;
  let py = 0;
  for (const b of world.eachBody()) {
    if (b.type !== BodyType.Dynamic) continue;
    const mass = f(b.mass);
    px += mass * f(b.linearVelocity.x);
    py += mass * f(b.linearVelocity.y);
  }
  return { px, py };
}

/* --------------------------- momentum ------------------------------- */

test('momentum is conserved exactly through 500 steps of collisions', () => {
  // No gravity, no friction, no walls: nothing may change the total momentum,
  // and "nothing" here should mean to the last bit, not approximately.
  const world = new World({ gravity: { x: 0, y: 0 } });
  const rng = new Rng(1);
  for (let i = 0; i < 30; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 16 - 8, y: rng.float() * 16 - 8 },
      linearVelocity: { x: rng.float() * 8 - 4, y: rng.float() * 8 - 4 },
    });
    b.addFixture({ shape: Circle.of(0.3), density: 1, restitution: 1, friction: 0 });
  }

  const before = momentum(world);
  steps(world, 500);
  const after = momentum(world);

  const magnitude = Math.hypot(before.px, before.py);
  const drift = Math.hypot(after.px - before.px, after.py - before.py) / magnitude;
  assert.ok(drift < 1e-9, `momentum drifted by ${drift.toExponential(2)} (relative)`);
});

test('angular momentum is conserved for a free spinning body', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const b = world.createBody({ type: BodyType.Dynamic, angularVelocity: 7 });
  b.addFixture({ shape: Polygon.box(0.5, 0.2), density: 1 });
  const before = f(b.inertia) * f(b.angularVelocity);
  steps(world, 1000);
  const after = f(b.inertia) * f(b.angularVelocity);
  assert.ok(Math.abs(after - before) < 1e-12, `spin changed: ${before} -> ${after}`);
});

/* ---------------------------- energy -------------------------------- */

test('a closed elastic box never gains energy', () => {
  // 25 perfectly elastic balls in a sealed box for 3000 steps. Energy may be
  // lost to discretisation but must never be created.
  const world = new World({ gravity: { x: 0, y: 0 } });
  for (const [x, y, hw, hh] of [[0, 9, 10, 0.5], [0, -9, 10, 0.5], [-9.5, 0, 0.5, 9], [9.5, 0, 0.5, 9]]) {
    const wall = world.createBody({ type: BodyType.Static, position: { x, y } });
    wall.addFixture({ shape: Polygon.box(hw, hh), restitution: 1, friction: 0 });
  }
  const rng = new Rng(2);
  for (let i = 0; i < 25; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 14 - 7, y: rng.float() * 14 - 7 },
      linearVelocity: { x: rng.float() * 10 - 5, y: rng.float() * 10 - 5 },
    });
    b.addFixture({ shape: Circle.of(0.35), density: 1, restitution: 1, friction: 0 });
  }

  const e0 = energy(world, 0);
  let peak = e0;
  for (let i = 0; i < 3000; i++) {
    world.step();
    peak = Math.max(peak, energy(world, 0));
  }
  assert.ok(peak <= e0 * 1.02, `energy grew to ${(100 * peak / e0).toFixed(1)}% of the start`);
});

test('free fall conserves mechanical energy', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 100 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1 });

  const e0 = energy(world);
  let worst = 0;
  for (let i = 0; i < 200; i++) {
    world.step();
    worst = Math.max(worst, Math.abs(energy(world) - e0) / Math.abs(e0));
  }
  assert.ok(worst < 0.02, `energy varied by ${(worst * 100).toFixed(3)}%`);
});

test('a pendulum holds its energy over 5000 steps', () => {
  const world = new World({ gravity: { x: 0, y: -10 }, subSteps: 8 });
  const L = 4;
  const angle = -1.2;
  const pivot = world.createBody({ type: BodyType.Static, position: { x: 0, y: 20 } });
  const bob = world.createBody({
    type: BodyType.Dynamic,
    position: { x: Math.sin(angle) * L, y: 20 - Math.cos(angle) * L },
  });
  bob.addFixture({ shape: Circle.of(0.2), density: 5 });
  world.createDistanceJoint({ bodyA: pivot, bodyB: bob, length: L });

  const e0 = energy(world);
  let lo = e0;
  let hi = e0;
  for (let i = 0; i < 5000; i++) {
    world.step();
    const e = energy(world);
    lo = Math.min(lo, e);
    hi = Math.max(hi, e);
  }
  assert.ok((hi - lo) / Math.abs(e0) < 0.03, `energy swung ${(100 * (hi - lo) / Math.abs(e0)).toFixed(2)}%`);
  assert.ok(hi <= Math.abs(e0) * 1.02, 'pendulum gained energy');
});

test('a joint chain never gains energy', () => {
  // The case that exposed the stale-anchor bug: a horizontal chain is the
  // worst case for joint error, and a joint that fights itself pumps energy.
  const world = new World({ gravity: { x: 0, y: -10 } });
  let prev = world.createBody({ type: BodyType.Static, position: { x: 0, y: 20 } });
  const links = [];
  for (let i = 0; i < 10; i++) {
    const link = world.createBody({ type: BodyType.Dynamic, position: { x: (i + 1) * 0.6, y: 20 } });
    link.addFixture({ shape: Polygon.box(0.09, 0.28), density: 1, filter: { group: -1 } });
    world.createRevoluteJointAt(prev, link, i * 0.6, 20);
    prev = link;
    links.push(link);
  }

  const e0 = energy(world);
  let peak = e0;
  for (let i = 0; i < 3000; i++) {
    world.step();
    peak = Math.max(peak, energy(world));
  }
  assert.ok(peak <= e0 + Math.abs(e0) * 0.02,
    `chain gained ${(100 * (peak - e0) / Math.abs(e0)).toFixed(3)}% energy`);

  // And it must be winding down, not winding up.
  const late = Math.max(...links.map((l) => f(l.linearVelocity.length())));
  assert.ok(late < 5, `chain still moving at ${late.toFixed(2)} m/s after 3000 steps`);
});

/* --------------------------- dissipation ---------------------------- */

test('friction removes energy and brings a slide to rest', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(60, 1), friction: 0.5 });
  const b = world.createBody({
    type: BodyType.Dynamic,
    position: { x: -20, y: 0.5 },
    linearVelocity: { x: 15, y: 0 },
  });
  b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0.5 });

  const e0 = energy(world);
  steps(world, 600);
  assert.ok(energy(world) < e0 * 0.999, 'friction did not dissipate energy');
  assert.ok(Math.abs(f(b.linearVelocity.x)) < 0.5, `still sliding at ${f(b.linearVelocity.x)}`);
});

test('a zero-restitution impact does not bounce', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(60, 1), friction: 0.5 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 20 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1, restitution: 0 });

  let landed = false;
  let highest = -Infinity;
  for (let i = 0; i < 600; i++) {
    world.step();
    const y = f(b.getPosition().y);
    if (y < 0.4) landed = true;
    if (landed) highest = Math.max(highest, y);
  }
  assert.ok(highest < 0.6, `inelastic ball rebounded to ${highest.toFixed(3)}`);
});

/* ---------------------------- settling ------------------------------ */

test('everything that should come to rest does', () => {
  const cases = {
    'single box': () => {
      const w = new World({ gravity: { x: 0, y: -10 } });
      const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      g.addFixture({ shape: Polygon.box(60, 1), friction: 0.6 });
      const b = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
      b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0.6 });
      return w;
    },
    'bouncing ball': () => {
      const w = new World({ gravity: { x: 0, y: -10 } });
      const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      g.addFixture({ shape: Polygon.box(60, 1), friction: 0.5 });
      const b = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 10 } });
      b.addFixture({ shape: Circle.of(0.4), density: 1, restitution: 0.7, friction: 0.5 });
      return w;
    },
    'pile of 40': () => {
      const w = new World({ gravity: { x: 0, y: -10 } });
      const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      g.addFixture({ shape: Polygon.box(60, 1), friction: 0.7 });
      const rng = new Rng(9);
      for (let i = 0; i < 40; i++) {
        const b = w.createBody({
          type: BodyType.Dynamic,
          position: { x: rng.float() * 8 - 4, y: 1 + i * 0.7 },
          angle: rng.float() * 6,
        });
        b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.7 });
      }
      return w;
    },
    'damped chain': () => {
      const w = new World({ gravity: { x: 0, y: -10 } });
      let prev = w.createBody({ type: BodyType.Static, position: { x: 0, y: 20 } });
      for (let i = 0; i < 10; i++) {
        const l = w.createBody({
          type: BodyType.Dynamic,
          position: { x: (i + 1) * 0.6, y: 20 },
          linearDamping: 0.4,
          angularDamping: 0.4,
        });
        l.addFixture({ shape: Polygon.box(0.09, 0.28), density: 1, filter: { group: -1 } });
        w.createRevoluteJointAt(prev, l, i * 0.6, 20);
        prev = l;
      }
      return w;
    },
  };

  for (const [name, build] of Object.entries(cases)) {
    const world = build();
    let sleptAt = -1;
    for (let i = 0; i < 4000; i++) {
      world.step();
      if (world.awakeBodyCount === 0) { sleptAt = i; break; }
    }
    assert.ok(sleptAt > 0, `"${name}" never came to rest (${world.awakeBodyCount} still awake)`);
  }
});

/* ------------------------ overlap recovery -------------------------- */

test('overlapping bodies separate without being launched', () => {
  // Push-out injects real velocity that relaxation only partly removes, so
  // MAX_BIAS_VELOCITY bounds how violently a bad spawn can erupt. Eight
  // circles dropped on the same spot make 28 simultaneous contacts, and at the
  // old 4 m/s cap the pile flung itself apart at ~10 m/s.
  const world = new World({ gravity: { x: 0, y: 0 } });
  const bodies = [];
  for (let i = 0; i < 8; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: i * 0.5, y: 0 } });
    b.addFixture({ shape: Circle.of(0.5), density: 1 });
    bodies.push(b);
  }

  let peak = 0;
  for (let i = 0; i < 400; i++) {
    world.step();
    peak = Math.max(peak, ...bodies.map((b) => f(b.linearVelocity.length())));
  }
  assert.ok(peak < 6, `pile-up ejected at ${peak.toFixed(2)} m/s`);
  for (const b of bodies) {
    assert.ok(Number.isFinite(f(b.getPosition().x)), 'overlap recovery produced a non-finite state');
  }
});

test('a body embedded in a wall works its way out promptly', () => {
  // The other side of the same trade-off: the cap must still be high enough to
  // clear any penetration a game realistically produces.
  for (const depth of [0.1, 0.3, 0.6]) {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const wall = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
    wall.addFixture({ shape: Polygon.box(1, 5) });
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 1 + 0.3 - depth, y: 0 } });
    b.addFixture({ shape: Circle.of(0.3), density: 1 });

    let freedAt = -1;
    for (let i = 0; i < 300; i++) {
      world.step();
      if (f(b.getPosition().x) > 1 + 0.29) { freedAt = i; break; }
    }
    assert.ok(freedAt >= 0 && freedAt < 60,
      `${depth} m penetration took ${freedAt < 0 ? '>300' : freedAt} steps to clear`);
  }
});

test('a normal stack is unaffected by the push-out cap', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(30, 1), friction: 0.7 });
  const boxes = [];
  for (let i = 0; i < 12; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 + i * 1.002 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0.7 });
    boxes.push(b);
  }
  steps(world, 900);

  const drift = Math.max(...boxes.map((b, i) => Math.abs(f(b.getPosition().y) - (0.5 + i))));
  assert.ok(drift < 0.15, `stack drifted ${drift.toFixed(4)} m`);
  assert.equal(world.awakeBodyCount, 0, 'stack should be fully asleep');
});
