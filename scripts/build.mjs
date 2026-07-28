/**
 * Build script.
 *
 * Produces six bundles plus type declarations:
 *
 *   dist/pulse2d.mjs         ESM,  float64 backend
 *   dist/pulse2d.cjs         CJS,  float64 backend
 *   dist/pulse2d.umd.js      UMD,  float64 backend (minified, for <script>)
 *   dist/pulse2d.fixed.mjs   ESM,  Q16.16 fixed-point backend
 *   dist/pulse2d.fixed.cjs   CJS,  Q16.16 fixed-point backend
 *   dist/types/**.d.ts       declarations
 *
 * The fixed-point variants are produced by aliasing `math/scalar.js` to
 * `math/scalar.fixed.ts` — no source duplication, no runtime branch.
 */

import { build } from 'esbuild';
import { execSync } from 'node:child_process';
import { rmSync, mkdirSync, existsSync, statSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, join } from 'node:path';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const entry = join(root, 'src/index.ts');
const outdir = join(root, 'dist');

rmSync(outdir, { recursive: true, force: true });
mkdirSync(outdir, { recursive: true });

/** Alias plugin that swaps the scalar backend. */
const fixedBackend = {
  name: 'pulse2d-fixed-backend',
  setup(b) {
    b.onResolve({ filter: /scalar\.js$/ }, (args) => {
      if (args.importer.includes('scalar.f64') || args.importer.includes('scalar.fixed')) return null;
      return { path: join(root, 'src/math/scalar.fixed.ts') };
    });
    // The selector module itself must also point at the fixed backend.
    b.onResolve({ filter: /scalar\.f64\.js$/ }, () => ({
      path: join(root, 'src/math/scalar.fixed.ts'),
    }));
  },
};

const common = {
  entryPoints: [entry],
  bundle: true,
  target: ['es2020', 'chrome90', 'firefox90', 'safari15', 'node18'],
  sourcemap: true,
  // The published package ships `src/` alongside `dist/`, so the maps' relative
  // `../src/...` paths resolve for real. Embedding a second copy of every
  // source inside each of the four maps would triple the tarball for nothing.
  sourcesContent: false,
  legalComments: 'none',
  logLevel: 'info',
};

const targets = [
  { outfile: 'pulse2d.mjs', format: 'esm', plugins: [] },
  { outfile: 'pulse2d.cjs', format: 'cjs', plugins: [] },
  {
    outfile: 'pulse2d.umd.js',
    format: 'iife',
    globalName: 'Pulse2D',
    minify: true,
    sourcemap: false,
    plugins: [],
  },
  { outfile: 'pulse2d.fixed.mjs', format: 'esm', plugins: [fixedBackend] },
  { outfile: 'pulse2d.fixed.cjs', format: 'cjs', plugins: [fixedBackend] },
];

for (const t of targets) {
  const { outfile, plugins, ...rest } = t;
  await build({ ...common, ...rest, plugins, outfile: join(outdir, outfile) });
}

console.log('\ngenerating type declarations…');
execSync('npx tsc -p tsconfig.build.json', { cwd: root, stdio: 'inherit' });

/*
 * A CommonJS view of the declarations.
 *
 * The package is `"type": "module"`, so every `.d.ts` in it is an *ESM*
 * declaration file. Under TypeScript's `node16`/`nodenext` resolution a
 * `require('pulse2d')` consumer therefore gets TS1479 ("cannot be imported
 * with require") even though `dist/pulse2d.cjs` is perfectly valid CommonJS —
 * the runtime works and only the types are wrong, which is the worst kind of
 * broken because it hits at compile time in the user's editor.
 *
 * The fix is the standard dual-package one: ship a second, byte-identical set
 * of declarations under the `.d.cts` extension, which is unconditionally
 * CommonJS regardless of the `type` field, and point the `require` condition
 * of the exports map at it. Relative specifiers have to be rewritten from
 * `./x.js` to `./x.cjs` so they land on the `.d.cts` siblings rather than back
 * on the ESM originals.
 *
 * Declaration maps are not copied: they would point at the same `src/*.ts`
 * files the ESM maps already cover, and "go to definition" resolves through
 * those.
 */
function emitCjsTypes(from, to) {
  rmSync(to, { recursive: true, force: true });
  mkdirSync(to, { recursive: true });
  let count = 0;

  const walk = (srcDir, outDir) => {
    for (const entry of readdirSync(srcDir, { withFileTypes: true })) {
      const srcPath = join(srcDir, entry.name);
      if (entry.isDirectory()) {
        const nested = join(outDir, entry.name);
        mkdirSync(nested, { recursive: true });
        walk(srcPath, nested);
        continue;
      }
      if (!entry.name.endsWith('.d.ts')) continue;

      const text = readFileSync(srcPath, 'utf8')
        // Only rewrite *relative* specifiers; the engine has no bare imports,
        // but being explicit keeps this correct if one is ever added.
        .replace(/(from\s*['"]\.[^'"]*?)\.js(['"])/g, '$1.cjs$2')
        .replace(/(import\s*\(\s*['"]\.[^'"]*?)\.js(['"]\s*\))/g, '$1.cjs$2')
        // The .d.ts.map beside the ESM copy does not describe this file.
        .replace(/^\/\/#\s*sourceMappingURL=.*$/gm, '');

      writeFileSync(join(outDir, entry.name.replace(/\.d\.ts$/, '.d.cts')), text);
      count++;
    }
  };

  walk(from, to);
  return count;
}

const cjsTypeCount = emitCjsTypes(join(outdir, 'types'), join(outdir, 'types-cjs'));
console.log(`  ${cjsTypeCount} CommonJS declaration files → dist/types-cjs/`);

// Report bundle sizes.
console.log('\nbundle sizes:');
const files = readdirSync(outdir).filter((f) => f.endsWith('.js') || f.endsWith('.mjs') || f.endsWith('.cjs'));
for (const f of files.sort()) {
  const size = statSync(join(outdir, f)).size;
  console.log(`  ${f.padEnd(24)} ${(size / 1024).toFixed(1)} KB`);
}
if (existsSync(join(outdir, 'types/index.d.ts'))) console.log('  types/                   ok');
