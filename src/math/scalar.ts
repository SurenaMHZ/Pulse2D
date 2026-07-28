/**
 * @module math/scalar
 *
 * **Backend selector.** Every other module in the engine imports its scalar
 * arithmetic from here and from nowhere else:
 *
 * ```ts
 * import * as S from '../math/scalar.js';
 * const area = S.mul(w, h);
 * ```
 *
 * The default export chain points at the {@link module:math/scalar.f64}
 * backend. To build the fixed-point flavour, alias this module to
 * `scalar.fixed.ts` at bundle time — the public API of both files is
 * byte-for-byte compatible, so nothing else has to change:
 *
 * ```js
 * // esbuild
 * alias: { './math/scalar.js': './src/math/scalar.fixed.ts' }
 *
 * // vite / rollup
 * resolve.alias = { '/src/math/scalar.js': '/src/math/scalar.fixed.ts' }
 * ```
 *
 * `npm run build` emits both variants (`pulse2d.mjs` and
 * `pulse2d.fixed.mjs`) out of the box.
 *
 * Operations that are *not* re-exported (`+`, `-`, unary `-`, `<`, `<=`, `>`,
 * `>=`, `===`) behave identically in both encodings and are written inline for
 * speed.
 */

export * from './scalar.f64.js';
