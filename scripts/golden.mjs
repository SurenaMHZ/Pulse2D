/**
 * Golden-checksum recorder.
 *
 *   node scripts/golden.mjs           print what this machine computes
 *   node scripts/golden.mjs --write   overwrite test/golden.json
 *   node scripts/golden.mjs --check   exit non-zero on any mismatch (CI)
 *
 * `--write` is deliberately a manual step. Re-recording is how you *lose*
 * determinism coverage, so it should only ever happen when you meant to change
 * engine arithmetic — and in that case you must also bump PROTOCOL_VERSION.
 */
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import * as f64 from '../dist/pulse2d.mjs';
import * as fixed from '../dist/pulse2d.fixed.mjs';
import { scenes, digestScene } from './golden-scenes.mjs';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const out = join(root, 'test/golden.json');

const write = process.argv.includes('--write');
const check = process.argv.includes('--check');

const backends = [
  ['float64', f64],
  ['fixed', fixed],
];

const computed = { protocolVersion: f64.PROTOCOL_VERSION, float64: {}, fixed: {} };

for (const [name, api] of backends) {
  for (const scene of scenes) {
    computed[name][scene.name] = digestScene(api, scene);
  }
}

const pad = (s, n) => String(s).padEnd(n);
console.log(`\nPulse2D golden checksums — node ${process.version} ${process.platform}/${process.arch}`);
console.log(`protocol version ${computed.protocolVersion}\n`);
console.log(`  ${pad('scene', 22)}${pad('float64', 12)}fixed`);
console.log(`  ${'-'.repeat(46)}`);
for (const scene of scenes) {
  console.log(`  ${pad(scene.name, 22)}${pad(computed.float64[scene.name], 12)}${computed.fixed[scene.name]}`);
}
console.log();

if (write) {
  writeFileSync(out, JSON.stringify(computed, null, 2) + '\n');
  console.log(`written → ${out}`);
  console.log('remember: if the values changed, bump PROTOCOL_VERSION and note it in CHANGELOG.md\n');
  process.exit(0);
}

if (!existsSync(out)) {
  console.error(`no ${out} — run with --write to record it`);
  process.exit(1);
}

const golden = JSON.parse(readFileSync(out, 'utf8'));
let bad = 0;
for (const [name] of backends) {
  for (const scene of scenes) {
    const want = golden[name]?.[scene.name];
    const got = computed[name][scene.name];
    if (want !== got) {
      console.error(`MISMATCH ${name}/${scene.name}: recorded ${want}, this machine ${got}`);
      bad++;
    }
  }
}
if (golden.protocolVersion !== computed.protocolVersion) {
  console.error(`MISMATCH protocolVersion: recorded ${golden.protocolVersion}, build ${computed.protocolVersion}`);
  bad++;
}

if (bad) {
  console.error(`\n${bad} mismatch(es). Either engine arithmetic changed (re-record + bump PROTOCOL_VERSION)`);
  console.error('or this platform is not bit-identical with the reference (a real bug — fix it).\n');
  process.exit(1);
}
console.log(`all ${scenes.length * backends.length} golden checksums match\n`);
if (check) process.exit(0);
