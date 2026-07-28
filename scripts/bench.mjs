/**
 * Benchmark suite.
 *
 * Reports median step time over a warmed-up run, so the numbers reflect
 * steady-state JIT performance rather than start-up cost.
 *
 *   node scripts/bench.mjs
 */

import {
  World,
  BodyType,
  Circle,
  Capsule,
  Polygon,
  Rng,
  saveSnapshot,
  cloneSnapshot,
  loadSnapshot,
  checksumWorld,
  Scalar as S,
} from '../dist/pulse2d.mjs';

const WARMUP = 60;
const SAMPLES = 300;

/** Median is far more stable than the mean when a GC pause lands mid-run. */
function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const mid = sorted.length >> 1;
  return sorted.length % 2 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
}

function percentile(values, p) {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * p))];
}

function bench(name, setup, { warmup = WARMUP, samples = SAMPLES } = {}) {
  const world = setup();
  for (let i = 0; i < warmup; i++) world.step();

  const times = [];
  for (let i = 0; i < samples; i++) {
    const t0 = performance.now();
    world.step();
    times.push(performance.now() - t0);
  }

  const med = median(times);
  const p99 = percentile(times, 0.99);
  const budget = (med / (1000 / 60)) * 100; // % of a 60 Hz frame
  console.log(
    `  ${name.padEnd(34)} ${med.toFixed(3).padStart(7)} ms   ` +
      `p99 ${p99.toFixed(3).padStart(7)} ms   ` +
      `${budget.toFixed(1).padStart(5)}% of 60Hz   ` +
      `${world.bodyCount} bodies, ${world.contactCount} contacts`,
  );
  return med;
}

const ground = (world, halfWidth = 100) => {
  const b = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  b.addFixture({ shape: Polygon.box(halfWidth, 1), friction: 0.6 });
  return b;
};

/* ------------------------------------------------------------------ */

console.log('\nPulse2D benchmarks');
console.log(`node ${process.version}  ${process.arch}\n`);
console.log('scenario                             median        p99      frame budget');
console.log('-'.repeat(100));

/** A pyramid is the classic stress test: deep stacks, many resting contacts. */
function pyramid(rows) {
  return () => {
    const world = new World({ gravity: { x: 0, y: -10 } });
    ground(world);
    const size = 0.5;
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col <= row; col++) {
        const b = world.createBody({
          type: BodyType.Dynamic,
          position: {
            x: (col - row * 0.5) * size * 2.05,
            y: (rows - row) * size * 2.05 + size,
          },
        });
        b.addFixture({ shape: Polygon.box(size, size), density: 1, friction: 0.6 });
      }
    }
    return world;
  };
}

bench('pyramid, 10 rows (55 boxes)', pyramid(10));
bench('pyramid, 20 rows (210 boxes)', pyramid(20));
bench('pyramid, 30 rows (465 boxes)', pyramid(30));

/** Loose debris: lots of bodies, shallow contact graph, broad-phase heavy. */
function debris(count, mixed) {
  return () => {
    const world = new World({ gravity: { x: 0, y: -10 } });
    ground(world);
    const rng = new Rng(1234);
    for (let i = 0; i < count; i++) {
      const b = world.createBody({
        type: BodyType.Dynamic,
        position: { x: rng.float() * 80 - 40, y: 1 + rng.float() * 30 },
        angle: rng.float() * 6,
      });
      const kind = mixed ? rng.int(0, 2) : 0;
      if (kind === 0) b.addFixture({ shape: Circle.of(0.3), density: 1 });
      else if (kind === 1) b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1 });
      else b.addFixture({ shape: Capsule.vertical(0.8, 0.2), density: 1 });
    }
    return world;
  };
}

bench('500 circles, falling', debris(500, false));
bench('1000 circles, falling', debris(1000, false));
bench('1000 mixed shapes, falling', debris(1000, true));
bench('2000 mixed shapes, falling', debris(2000, true));

/** A settled world should cost almost nothing thanks to sleeping. */
bench('1000 bodies, all asleep', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  ground(world);
  const rng = new Rng(99);
  for (let i = 0; i < 1000; i++) {
    const b = world.createBody({
      type: BodyType.Dynamic,
      position: { x: rng.float() * 90 - 45, y: 0.3 },
    });
    b.addFixture({ shape: Circle.of(0.25), density: 1 });
  }
  for (let i = 0; i < 900; i++) world.step(); // let everything settle
  return world;
});

/** Joint-heavy: a chain of linked bodies. */
bench('30 ragdoll chains (300 joints)', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  ground(world);
  for (let c = 0; c < 30; c++) {
    let prev = world.createBody({
      type: BodyType.Static,
      position: { x: c * 3 - 45, y: 25 },
    });
    for (let i = 0; i < 10; i++) {
      const link = world.createBody({
        type: BodyType.Dynamic,
        position: { x: c * 3 - 45, y: 25 - (i + 1) * 0.6 },
      });
      link.addFixture({ shape: Polygon.box(0.1, 0.3), density: 1 });
      world.createRevoluteJointAt(prev, link, c * 3 - 45, 25 - i * 0.6 - 0.3);
      prev = link;
    }
  }
  return world;
});

/* ---------------------------- netcode ----------------------------- */

console.log('\nnetcode');
console.log('-'.repeat(100));

{
  const world = debris(500, true)();
  for (let i = 0; i < 120; i++) world.step();

  let t0 = performance.now();
  let snap;
  for (let i = 0; i < 200; i++) snap = saveSnapshot(world);
  const saveMs = (performance.now() - t0) / 200;

  const copy = cloneSnapshot(snap);
  t0 = performance.now();
  for (let i = 0; i < 200; i++) loadSnapshot(world, copy);
  const loadMs = (performance.now() - t0) / 200;

  t0 = performance.now();
  for (let i = 0; i < 500; i++) checksumWorld(world);
  const sumMs = (performance.now() - t0) / 500;

  const bytes = snap.data.byteLength + snap.meta.byteLength;
  console.log(`  saveSnapshot (500 bodies)          ${saveMs.toFixed(3).padStart(7)} ms   ${(bytes / 1024).toFixed(1)} KB per snapshot`);
  console.log(`  loadSnapshot (500 bodies)          ${loadMs.toFixed(3).padStart(7)} ms`);
  console.log(`  checksumWorld (500 bodies)         ${sumMs.toFixed(3).padStart(7)} ms`);
  console.log(`  1 s of rollback history @60Hz      ${((bytes * 60) / 1024 / 1024).toFixed(2)} MB`);
}

/* ---------------------------- geometry ---------------------------- */

console.log('\nraw operations (nanoseconds each)');
console.log('-'.repeat(100));

{
  const world = debris(1000, true)();
  for (let i = 0; i < 60; i++) world.step();

  const N = 20000;
  let t0 = performance.now();
  let hits = 0;
  for (let i = 0; i < N; i++) {
    const a = (i / N) * Math.PI * 2;
    if (world.rayCastClosest(0, 15, Math.cos(a) * 60, 15 + Math.sin(a) * 60)) hits++;
  }
  const rayNs = ((performance.now() - t0) / N) * 1e6;
  console.log(`  rayCastClosest                     ${rayNs.toFixed(0).padStart(7)} ns   (${hits}/${N} hit)`);

  t0 = performance.now();
  let found = 0;
  for (let i = 0; i < N; i++) {
    const x = (i % 100) - 50;
    world.queryAABB(x, 0, x + 2, 4, () => {
      found++;
      return true;
    });
  }
  const queryNs = ((performance.now() - t0) / N) * 1e6;
  console.log(`  queryAABB                          ${queryNs.toFixed(0).padStart(7)} ns   (${found} results)`);
}

{
  const N = 2000000;
  const a = S.fromFloat(0.7);
  let acc = 0;
  const t0 = performance.now();
  for (let i = 0; i < N; i++) acc += S.toFloat(S.mul(a, a));
  const ns = ((performance.now() - t0) / N) * 1e6;
  console.log(`  scalar multiply                    ${ns.toFixed(1).padStart(7)} ns   (checksum ${acc.toFixed(0)})`);
}

console.log();
