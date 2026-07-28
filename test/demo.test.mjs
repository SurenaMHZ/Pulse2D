/**
 * Demo scenes — extracted from `demo/index.html` and executed headless.
 *
 * The demo is the first thing anyone sees, so a scene that explodes, leaks
 * bodies to infinity or breaks determinism is a real bug. This test parses the
 * scene definitions straight out of the HTML, so the file under test is the
 * one that actually ships — it cannot drift out of sync with a copy.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, writeFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '..');

/**
 * Pull the module script out of the demo, strip everything that needs a DOM,
 * and rewrite the import so it resolves from a temp directory.
 */
function extractScenes() {
  const html = readFileSync(join(root, 'demo/index.html'), 'utf8');
  const open = html.indexOf('<script type="module">');
  const body = html.slice(open + '<script type="module">'.length, html.lastIndexOf('</script>'));

  // Everything above the UI section is pure simulation code.
  const cut = body.indexOf('/* ------------------------------- UI ---');
  assert.ok(cut > 0, 'demo layout changed: UI marker not found');

  const distUrl = pathToFileURL(join(root, 'dist/pulse2d.mjs')).href;

  const source = body
    .slice(0, cut)
    .replace("from '../dist/pulse2d.mjs';", `from ${JSON.stringify(distUrl)};`)
    .replace("const canvas = document.getElementById('c');", 'const canvas = { width: 1200, height: 800 };')
    .replace("const ctx = canvas.getContext('2d');", 'const ctx = null;')
    .replace("document.getElementById('ver').textContent = 'v' + VERSION;", '')
    .replace(
      'const draw = new DebugDraw(ctx, { pixelsPerMeter: 26, cameraY: 7 });',
      'const draw = { pixelsPerMeter: 26, cameraX: 0, cameraY: 7, flags: {}, ctx: null };',
    )
    .replace('function toast(msg) {', 'function toast(msg) { return;')
    .replace('_setGravity0()', '');

  const dir = mkdtempSync(join(tmpdir(), 'pulse2d-demo-'));
  const file = join(dir, 'scenes.mjs');
  writeFileSync(file, `${source}\nexport { scenes, spawn, cannon, explode, blasts, world, draw };\n`);
  return pathToFileURL(file).href;
}

const mod = await import(extractScenes());
const { scenes } = mod;
const { Scalar: S, checksumWorld, BodyType, Polygon, Circle } = await import('../dist/pulse2d.mjs');
const f = S.toFloat;

const names = Object.keys(scenes);

test('demo exposes a healthy number of scenes', () => {
  assert.ok(names.length >= 16, `expected 16+ scenes, found ${names.length}`);
});

for (const name of names) {
  test(`demo scene "${name}" is stable over 700 steps`, () => {
    const world = scenes[name]();
    for (let i = 0; i < 700; i++) world.step();

    let checked = 0;
    for (const body of world.eachBody()) {
      const x = f(body.getPosition().x);
      const y = f(body.getPosition().y);
      assert.ok(Number.isFinite(x) && Number.isFinite(y), `${name}: non-finite position`);
      // Nothing should be launched out of the visible world.
      assert.ok(Math.abs(x) < 400 && Math.abs(y) < 400, `${name}: body escaped to ${x}, ${y}`);
      checked++;
    }
    assert.ok(checked > 0, `${name}: scene is empty`);
  });

  test(`demo scene "${name}" is deterministic`, () => {
    const a = scenes[name]();
    const b = scenes[name]();
    for (let i = 0; i < 300; i++) {
      a.step();
      b.step();
      if (i % 25 === 0 || i === 299) {
        assert.equal(checksumWorld(a), checksumWorld(b), `${name}: diverged at tick ${i}`);
      }
    }
  });
}

test('demo tools: spawning into a live scene is safe', () => {
  const world = scenes['Castle siege']();
  for (let i = 0; i < 200; i++) world.step();

  const before = world.bodyCount;
  const box = world.createBody({ type: BodyType.Dynamic, position: { x: 6, y: 9 } });
  box.addFixture({ shape: Polygon.box(0.32, 0.32), density: 1, friction: 0.5 });
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 8, y: 11 } });
  ball.addFixture({ shape: Circle.of(0.3), density: 1, restitution: 0.45 });
  assert.equal(world.bodyCount, before + 2);

  for (let i = 0; i < 300; i++) world.step();
  for (const body of world.eachBody()) {
    assert.ok(Number.isFinite(f(body.getPosition().y)), 'spawned body destabilised the scene');
  }
});

test('demo tools: a radial impulse scatters bodies without blowing up', () => {
  const world = scenes.Pyramid();
  for (let i = 0; i < 300; i++) world.step();

  const before = [...world.eachBody()].map((b) => f(b.getPosition().y));
  const cx = 0, cy = 2, radius = 6, power = 120;
  world.queryAABB(cx - radius, cy - radius, cx + radius, cy + radius, (fixture) => {
    const body = fixture.body;
    if (body.type !== BodyType.Dynamic) return true;
    const p = body.worldCenter;
    const dx = f(p.x) - cx;
    const dy = f(p.y) - cy;
    const d = Math.hypot(dx, dy);
    if (d > radius || d < 1e-6) return true;
    const falloff = 1 - d / radius;
    body.applyLinearImpulse((dx / d) * power * falloff, (dy / d) * power * falloff, f(p.x), f(p.y));
    return true;
  });

  for (let i = 0; i < 150; i++) world.step();
  const after = [...world.eachBody()].map((b) => f(b.getPosition().y));
  const moved = before.filter((y, i) => Math.abs(y - after[i]) > 0.25).length;
  assert.ok(moved > 8, `explosion only moved ${moved} bodies`);
  for (const y of after) assert.ok(Number.isFinite(y));
});

test('demo tools: ray cast reports hits and misses', () => {
  const world = scenes.Pyramid();
  for (let i = 0; i < 200; i++) world.step();
  assert.ok(world.rayCastClosest(-20, 2, 20, 2) !== null, 'ray through the pyramid should hit');
  assert.equal(world.rayCastClosest(-20, 60, 20, 60), null, 'ray above the scene should miss');
});

test('demo panel: runtime setting changes are safe', () => {
  const world = scenes['Mixed shapes']();
  for (const subSteps of [1, 4, 12]) {
    world.subSteps = subSteps;
    for (let i = 0; i < 80; i++) world.step();
  }
  world.gravity.set(S.ZERO, S.fromFloat(6));       // gravity can even invert
  world.wakeAll();
  for (let i = 0; i < 150; i++) world.step();
  world.enableSleep = false;
  world.enableWarmStarting = false;
  for (let i = 0; i < 150; i++) world.step();

  for (const body of world.eachBody()) {
    assert.ok(Number.isFinite(f(body.getPosition().y)), 'setting change destabilised the world');
  }
});

test('demo scene "Stress: 800 bodies" stays within a sane step budget', () => {
  const world = scenes['Stress: 800 bodies']();
  for (let i = 0; i < 60; i++) world.step();   // warm up the JIT

  const times = [];
  for (let i = 0; i < 60; i++) {
    const t0 = process.hrtime.bigint();
    world.step();
    times.push(Number(process.hrtime.bigint() - t0) / 1e6);
  }
  times.sort((x, y) => x - y);
  const median = times[times.length >> 1];

  // Generous: this only needs to catch an order-of-magnitude regression,
  // since CI machines vary wildly.
  assert.ok(median < 200, `800-body step took ${median.toFixed(1)} ms`);
});
