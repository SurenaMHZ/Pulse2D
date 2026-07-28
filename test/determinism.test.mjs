/**
 * Determinism, snapshots, checksums and rollback.
 *
 * These are the tests that matter most for lockstep netcode: the same inputs
 * must produce bit-identical output, every time, on every machine.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  World,
  BodyType,
  Circle,
  Capsule,
  Polygon,
  Rng,
  saveSnapshot,
  loadSnapshot,
  cloneSnapshot,
  checksumWorld,
  checksumSnapshot,
  ChecksumLog,
  RollbackManager,
  Hasher,
  Scalar as S,
} from '../dist/pulse2d.mjs';

const f = S.toFloat;

/**
 * Build a deliberately chaotic scene: mixed shapes, stacking, bouncing and
 * spinning, so any non-determinism has plenty of chances to show up.
 */
function buildScene(seed = 1234) {
  const world = new World({ gravity: { x: 0, y: -10 }, seed });

  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(40, 1), friction: 0.5 });

  const leftWall = world.createBody({ type: BodyType.Static, position: { x: -12, y: 5 } });
  leftWall.addFixture({ shape: Polygon.box(1, 6) });
  const rightWall = world.createBody({ type: BodyType.Static, position: { x: 12, y: 5 } });
  rightWall.addFixture({ shape: Polygon.box(1, 6) });

  const rng = new Rng(seed);
  const bodies = [];
  for (let i = 0; i < 30; i++) {
    const x = rng.float() * 16 - 8;
    const y = 1 + rng.float() * 12;
    const body = world.createBody({
      type: BodyType.Dynamic,
      position: { x, y },
      angle: rng.float() * 6 - 3,
      angularVelocity: rng.float() * 4 - 2,
      linearVelocity: { x: rng.float() * 6 - 3, y: rng.float() * 2 },
    });
    const kind = rng.int(0, 2);
    if (kind === 0) {
      body.addFixture({ shape: Circle.of(0.25 + rng.float() * 0.3), density: 1, restitution: 0.4 });
    } else if (kind === 1) {
      body.addFixture({
        shape: Polygon.box(0.25 + rng.float() * 0.3, 0.25 + rng.float() * 0.3),
        density: 1,
        friction: 0.4,
      });
    } else {
      body.addFixture({ shape: Capsule.vertical(1, 0.22), density: 1, friction: 0.3 });
    }
    bodies.push(body);
  }

  // A couple of joints, so the joint solver is exercised too.
  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 14 } });
  world.createDistanceJoint({ bodyA: anchor, bodyB: bodies[0], length: 3 });
  world.createRevoluteJointAt(anchor, bodies[1], 0, 14);

  return { world, bodies };
}

/** Full-precision state dump, for exact comparison. */
function stateOf(world) {
  const out = [];
  for (const b of world.eachBody()) {
    out.push(
      b.transform.p.x,
      b.transform.p.y,
      b.transform.q.s,
      b.transform.q.c,
      b.linearVelocity.x,
      b.linearVelocity.y,
      b.angularVelocity,
    );
  }
  return out;
}

/* ---------------------- core determinism ------------------------ */

test('two identical worlds stay bit-identical for 1000 steps', () => {
  const a = buildScene(777);
  const b = buildScene(777);
  for (let i = 0; i < 1000; i++) {
    a.world.step();
    b.world.step();
    if (i % 100 === 0 || i === 999) {
      assert.deepEqual(stateOf(a.world), stateOf(b.world), `diverged at step ${i}`);
    }
  }
  assert.equal(checksumWorld(a.world), checksumWorld(b.world));
});

test('checksums match every single tick', () => {
  const a = buildScene(31337);
  const b = buildScene(31337);
  for (let i = 0; i < 300; i++) {
    a.world.step();
    b.world.step();
    assert.equal(checksumWorld(a.world), checksumWorld(b.world), `checksum split at tick ${i}`);
  }
});

test('interleaved stepping does not change the outcome', () => {
  // Run one world to completion, then the other; order of execution between
  // independent worlds must not matter.
  const a = buildScene(555);
  for (let i = 0; i < 400; i++) a.world.step();
  const b = buildScene(555);
  for (let i = 0; i < 400; i++) b.world.step();
  assert.deepEqual(stateOf(a.world), stateOf(b.world));
});

test('a different seed gives a different result', () => {
  const a = buildScene(1);
  const b = buildScene(2);
  for (let i = 0; i < 200; i++) {
    a.world.step();
    b.world.step();
  }
  assert.notEqual(checksumWorld(a.world), checksumWorld(b.world));
});

test('no NaN or Infinity appears in a long chaotic run', () => {
  const { world } = buildScene(8888);
  for (let i = 0; i < 2000; i++) world.step();
  for (const b of world.eachBody()) {
    assert.ok(Number.isFinite(f(b.transform.p.x)), 'position x is finite');
    assert.ok(Number.isFinite(f(b.transform.p.y)), 'position y is finite');
    assert.ok(Number.isFinite(f(b.linearVelocity.x)), 'velocity x is finite');
    assert.ok(Number.isFinite(f(b.angularVelocity)), 'angular velocity is finite');
    assert.ok(Math.abs(f(b.transform.p.y)) < 1e4, 'nothing was launched into orbit');
  }
});

test('a pile of loose bodies settles and goes to sleep', () => {
  // No joints here: a body hanging on an ideal frictionless joint is a
  // pendulum and never comes to rest, which is physically correct.
  const world = new World({ gravity: { x: 0, y: -10 }, seed: 4242 });
  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(40, 1), friction: 0.5 });
  const rng = new Rng(4242);
  for (let i = 0; i < 20; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 10 - 5, y: 1 + rng.float() * 8 },
      angularVelocity: rng.float() * 4 - 2,
    });
    b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.5 });
  }

  for (let i = 0; i < 1500; i++) world.step();
  assert.equal(world.awakeBodyCount, 0, 'the pile should be fully asleep');

  // Once asleep the state must be a true fixed point. (checksumWorld hashes
  // the tick counter, which advances by design, so compare the state itself.)
  const before = stateOf(world);
  for (let i = 0; i < 100; i++) world.step();
  assert.deepEqual(stateOf(world), before, 'a sleeping world must not drift');
});

/* --------------------------- hashing ---------------------------- */

test('Hasher distinguishes values that differ in the last bit', () => {
  const a = new Hasher().float(1.0).digest();
  const b = new Hasher().float(1.0000000000000002).digest(); // 1 ulp apart
  assert.notEqual(a, b, 'a one-ulp difference must change the digest');
});

test('Hasher treats -0 and +0 as equal', () => {
  assert.equal(new Hasher().float(0).digest(), new Hasher().float(-0).digest());
});

test('Hasher is order sensitive', () => {
  assert.notEqual(
    new Hasher().int(1).int(2).digest(),
    new Hasher().int(2).int(1).digest(),
  );
});

test('checksumWorld reacts to a tiny nudge', () => {
  const { world, bodies } = buildScene(11);
  for (let i = 0; i < 50; i++) world.step();
  const before = checksumWorld(world);
  bodies[0].linearVelocity.x += S.fromFloat(1e-9);
  assert.notEqual(checksumWorld(world), before);
});

/* -------------------------- snapshots --------------------------- */

test('snapshot then immediate restore is a no-op', () => {
  const { world } = buildScene(99);
  for (let i = 0; i < 100; i++) world.step();
  const before = stateOf(world);
  const snap = saveSnapshot(world);
  loadSnapshot(world, snap);
  assert.deepEqual(stateOf(world), before);
});

test('restoring an old snapshot reproduces the future exactly', () => {
  const { world } = buildScene(2024);
  for (let i = 0; i < 200; i++) world.step();

  const snap = cloneSnapshot(saveSnapshot(world));
  const tickAtSave = world.tick;

  // Run forward and record where we end up.
  for (let i = 0; i < 150; i++) world.step();
  const expected = stateOf(world);
  const expectedSum = checksumWorld(world);

  // Rewind and replay.
  loadSnapshot(world, snap);
  assert.equal(world.tick, tickAtSave);
  for (let i = 0; i < 150; i++) world.step();

  assert.deepEqual(stateOf(world), expected, 'replay must land on the same state');
  assert.equal(checksumWorld(world), expectedSum, 'and on the same checksum');
});

test('repeated rewind and replay is stable', () => {
  const { world } = buildScene(31);
  for (let i = 0; i < 100; i++) world.step();
  const snap = cloneSnapshot(saveSnapshot(world));

  let reference = null;
  for (let attempt = 0; attempt < 5; attempt++) {
    loadSnapshot(world, snap);
    for (let i = 0; i < 80; i++) world.step();
    const sum = checksumWorld(world);
    if (reference === null) reference = sum;
    else assert.equal(sum, reference, `replay ${attempt} diverged`);
  }
});

test('snapshots preserve contact impulses (warm-start continuity)', () => {
  // A tall stack is very sensitive to losing its accumulated impulses.
  const world = new World({ gravity: { x: 0, y: -10 } });
  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(20, 1) });
  for (let i = 0; i < 6; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 + i * 1.001 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  }
  for (let i = 0; i < 120; i++) world.step();

  const snap = cloneSnapshot(saveSnapshot(world));
  for (let i = 0; i < 30; i++) world.step();
  const expected = checksumWorld(world);

  loadSnapshot(world, snap);
  for (let i = 0; i < 30; i++) world.step();
  assert.equal(checksumWorld(world), expected, 'impulses must survive the round trip');
});

test('snapshot restores the RNG stream', () => {
  const world = new World({ seed: 4321 });
  world.rng.next();
  world.rng.next();
  const snap = saveSnapshot(world);
  const expected = [world.rng.next(), world.rng.next(), world.rng.next()];
  loadSnapshot(world, snap);
  assert.deepEqual([world.rng.next(), world.rng.next(), world.rng.next()], expected);
});

test('snapshot preserves sleep state', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(20, 1) });
  const box = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 } });
  box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  for (let i = 0; i < 300; i++) world.step();
  assert.ok(!box.awake, 'precondition: the box should be asleep');

  const snap = cloneSnapshot(saveSnapshot(world));
  box.setAwake(true);
  loadSnapshot(world, snap);
  assert.ok(!box.awake, 'sleep state must be restored');
});

test('checksumSnapshot agrees for equal states and differs otherwise', () => {
  const a = buildScene(64);
  const b = buildScene(64);
  for (let i = 0; i < 60; i++) {
    a.world.step();
    b.world.step();
  }
  assert.equal(checksumSnapshot(saveSnapshot(a.world)), checksumSnapshot(saveSnapshot(b.world)));
  b.world.step();
  assert.notEqual(checksumSnapshot(saveSnapshot(a.world)), checksumSnapshot(saveSnapshot(b.world)));
});

test('loadSnapshot rejects a corrupt header', () => {
  const { world } = buildScene(5);
  const snap = saveSnapshot(world);
  const bad = cloneSnapshot(snap);
  bad.meta[0] = 0xdeadbeef;
  assert.throws(() => loadSnapshot(world, bad), /magic/);

  const wrongVersion = cloneSnapshot(snap);
  wrongVersion.meta[1] = 999;
  assert.throws(() => loadSnapshot(world, wrongVersion), /protocol/);
});

/* ------------------------- checksum log ------------------------- */

test('ChecksumLog records and finds divergence', () => {
  const log = new ChecksumLog(64);
  const remote = new Map();
  for (let tick = 0; tick < 20; tick++) {
    const sum = 1000 + tick;
    log.record(tick, sum);
    remote.set(tick, tick === 13 ? 999999 : sum);
  }
  assert.equal(log.get(5), 1005);
  assert.equal(log.findDivergence(remote), 13);
});

test('ChecksumLog reports no divergence when logs agree', () => {
  const log = new ChecksumLog(32);
  const remote = new Map();
  for (let t = 0; t < 10; t++) {
    log.record(t, t * 7);
    remote.set(t, t * 7);
  }
  assert.equal(log.findDivergence(remote), -1);
});

test('ChecksumLog overwrites the oldest entries when full', () => {
  const log = new ChecksumLog(8);
  for (let t = 0; t < 20; t++) log.record(t, t);
  assert.equal(log.get(0), undefined, 'evicted');
  assert.equal(log.get(19), 19, 'newest retained');
});

/* --------------------------- rollback --------------------------- */

test('rollback reproduces the state a correct prediction would have given', () => {
  // Two managers driven with the same inputs. One gets the remote input late
  // and has to roll back; the other has it on time. They must agree.
  const makeSim = (seed) => {
    const world = new World({ gravity: { x: 0, y: -10 }, seed });
    const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
    ground.addFixture({ shape: Polygon.box(30, 1), friction: 0.5 });
    const player = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1 } });
    player.addFixture({ shape: Polygon.box(0.4, 0.4), density: 1, friction: 0.5 });
    return { world, player };
  };

  const applyFor = (player) => (tick, inputs) => {
    const move = inputs.get(1);
    if (move) player.applyLinearImpulse(move.x, move.y);
  };

  const late = makeSim(1);
  const onTime = makeSim(1);

  const rbLate = new RollbackManager(late.world, {
    maxRollbackFrames: 20,
    applyInputs: applyFor(late.player),
    predictInput: () => ({ x: 0, y: 0 }), // always predicts "no input"
  });
  const rbOnTime = new RollbackManager(onTime.world, {
    maxRollbackFrames: 20,
    applyInputs: applyFor(onTime.player),
  });

  const realInputs = new Map();
  for (let tick = 0; tick < 40; tick++) {
    const input = { x: tick % 5 === 0 ? 2 : 0, y: 0 };
    realInputs.set(tick, input);

    // The on-time sim receives the input before stepping.
    rbOnTime.addRemoteInput(tick, 1, input);
    rbOnTime.advance();

    // The late sim predicts, and gets the truth 5 ticks later.
    rbLate.advance();
    if (tick >= 5) rbLate.addRemoteInput(tick - 5, 1, realInputs.get(tick - 5));
  }
  // Deliver the remaining late inputs.
  for (let tick = 35; tick < 40; tick++) rbLate.addRemoteInput(tick, 1, realInputs.get(tick));

  assert.ok(rbLate.rollbackCount > 0, 'the late peer should have rolled back');
  assert.equal(
    checksumWorld(late.world),
    checksumWorld(onTime.world),
    'both peers must converge on the same state',
  );
});

test('rollback to an evicted tick fails gracefully', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1 });
  const rb = new RollbackManager(world, { maxRollbackFrames: 4, applyInputs: () => {} });
  for (let i = 0; i < 20; i++) rb.advance();
  assert.equal(rb.rollbackTo(0), false, 'too old to recover');
  assert.equal(rb.rollbackTo(world.tick - 1), true, 'recent enough');
});

test('rollback keeps history bounded', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1 });
  const rb = new RollbackManager(world, { maxRollbackFrames: 10, applyInputs: () => {} });
  for (let i = 0; i < 200; i++) rb.advance();
  assert.equal(rb.historyLength, 10);
  assert.ok(rb.historyBytes > 0);
});

test('rollback with checksums enabled records every tick', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  b.addFixture({ shape: Circle.of(0.3), density: 1 });
  const rb = new RollbackManager(world, {
    maxRollbackFrames: 8,
    applyInputs: () => {},
    enableChecksums: true,
  });
  for (let i = 0; i < 20; i++) rb.advance();
  assert.ok(rb.checksums.get(19) !== undefined, 'the latest tick must be logged');
});

/* ------------------- full-engine determinism audit ------------------ */

/**
 * A scene that exercises every subsystem at once: mixed shapes, walls,
 * stacking, bouncing, and all the joint types together. If any part of the
 * engine develops a history dependence, this is what catches it.
 */
function auditScene(seed) {
  const world = new World({ gravity: { x: 0, y: -10 }, seed });
  const g = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(40, 1), friction: 0.5 });
  for (const x of [-12, 12]) {
    const wall = world.createBody({ type: BodyType.Static, position: { x, y: 6 } });
    wall.addFixture({ shape: Polygon.box(0.5, 7) });
  }

  const rng = new Rng(seed);
  const bodies = [];
  for (let i = 0; i < 40; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 20 - 10, y: 1 + rng.float() * 14 },
      angle: rng.float() * 6 - 3,
      angularVelocity: rng.float() * 4 - 2,
      linearVelocity: { x: rng.float() * 6 - 3, y: rng.float() * 2 },
    });
    const kind = rng.int(0, 2);
    if (kind === 0) b.addFixture({ shape: Circle.of(0.25 + rng.float() * 0.25), density: 1, restitution: 0.4, friction: 0.4 });
    else if (kind === 1) b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.5 });
    else b.addFixture({ shape: Capsule.vertical(0.8, 0.2), density: 1, friction: 0.4 });
    bodies.push(b);
  }

  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 16 } });
  world.createDistanceJoint({ bodyA: anchor, bodyB: bodies[0], length: 3 });
  world.createRevoluteJointAt(anchor, bodies[1], 0, 16);
  world.createWeldJoint({
    bodyA: anchor, bodyB: bodies[2],
    localAnchorA: { x: 2, y: -2 }, linearHertz: 5, angularHertz: 5,
  });
  world.createPrismaticJoint({
    bodyA: anchor, bodyB: bodies[3],
    localAxisA: { x: 1, y: 0 }, enableLimit: true, lowerLimit: -3, upperLimit: 3,
  });
  return world;
}

test('audit: the whole engine stays bit-identical for 1500 steps', () => {
  const a = auditScene(4242);
  const b = auditScene(4242);
  for (let i = 0; i < 1500; i++) {
    a.step();
    b.step();
    if (i % 100 === 0 || i === 1499) {
      assert.equal(checksumWorld(a), checksumWorld(b), `diverged at tick ${i}`);
    }
  }
});

test('audit: five independent runs agree exactly', () => {
  const sums = [];
  for (let run = 0; run < 5; run++) {
    const world = auditScene(777);
    for (let i = 0; i < 800; i++) world.step();
    sums.push(checksumWorld(world));
  }
  assert.ok(sums.every((s) => s === sums[0]), `runs disagreed: ${sums.map((s) => s.toString(16)).join(' ')}`);
});

test('audit: snapshot replay is exact across every subsystem', () => {
  const world = auditScene(31337);
  for (let i = 0; i < 300; i++) world.step();
  const snap = cloneSnapshot(saveSnapshot(world));

  const expected = [];
  for (let i = 0; i < 200; i++) { world.step(); expected.push(checksumWorld(world)); }

  loadSnapshot(world, snap);
  for (let i = 0; i < 200; i++) {
    world.step();
    assert.equal(checksumWorld(world), expected[i], `replay diverged at tick ${i}`);
  }
});
