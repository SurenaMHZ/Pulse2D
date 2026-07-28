/**
 * Golden checksums — the cross-machine determinism contract.
 *
 * Every other determinism test compares two runs *on the same machine*. That
 * catches history dependence, but it cannot catch the thing Pulse2D actually
 * promises: that a run on a Windows laptop and a run on a Linux server produce
 * the same bits.
 *
 * This file pins the answer. The constants below were recorded once; CI then
 * replays them on Linux, macOS and Windows across Node 18 → 24. If any of
 * those platforms disagrees by a single bit, the build goes red.
 *
 * **If a test here fails you have two possibilities, and only two:**
 *
 *  1. You changed engine arithmetic. That is a protocol break — bump
 *     `PROTOCOL_VERSION` in `src/util/settings.ts`, re-record the constants
 *     with `node scripts/golden.mjs --write`, and say so in the changelog.
 *  2. You did not change engine arithmetic, in which case determinism is
 *     broken on this platform and *must* be fixed, not re-recorded.
 *
 * Never re-record to make a red build green without knowing which case you
 * are in.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import * as f64 from '../dist/pulse2d.mjs';
import * as fixed from '../dist/pulse2d.fixed.mjs';
import { scenes, digestScene } from '../scripts/golden-scenes.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const golden = JSON.parse(readFileSync(resolve(here, 'golden.json'), 'utf8'));

for (const backend of [
  { name: 'float64', api: f64 },
  { name: 'fixed', api: fixed },
]) {
  for (const scene of scenes) {
    test(`golden [${backend.name}] ${scene.name}`, () => {
      const expected = golden[backend.name]?.[scene.name];
      assert.ok(expected !== undefined, `no golden value recorded for ${backend.name}/${scene.name}`);

      const actual = digestScene(backend.api, scene);

      assert.equal(
        actual,
        expected,
        `${backend.name}/${scene.name} diverged from the recorded result — ` +
          'either engine arithmetic changed (bump PROTOCOL_VERSION and re-record) ' +
          'or this platform is not deterministic (fix it, do not re-record)',
      );
    });
  }
}

test('golden: the recorded protocol version matches the build', () => {
  assert.equal(
    golden.protocolVersion,
    f64.PROTOCOL_VERSION,
    'golden.json was recorded against a different protocol version — re-record it',
  );
});

test('golden: both backends agree on the recorded protocol version', () => {
  assert.equal(f64.PROTOCOL_VERSION, fixed.PROTOCOL_VERSION);
});

test('golden: the two backends are independent (they must NOT match)', () => {
  // Q16.16 cannot reproduce float64 bit for bit; if these ever matched it
  // would mean the fixed-point build silently fell back to doubles.
  const a = golden.float64[scenes[0].name];
  const b = golden.fixed[scenes[0].name];
  assert.notEqual(a, b, 'the fixed-point build appears to be using the float64 backend');
});
