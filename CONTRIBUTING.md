# Contributing to Pulse2D

Thanks for taking the time. Pulse2D has one rule that outranks every other
consideration, so it comes first.

---

## The one rule: determinism is not negotiable

Pulse2D exists because lockstep and rollback netcode need a physics engine that
gives **bit-identical** results on every machine. A change that is a 1e-16
improvement in accuracy but breaks that guarantee is not an improvement — it is
a regression that will show up as a desync in someone's shipped game.

Concretely, in `src/`:

| Banned | Use instead |
|---|---|
| `Math.sin`, `Math.cos`, `Math.tan`, `Math.atan2`, `Math.asin`, `Math.acos` | `sin`/`cos`/`sinCos`/`atan2`/… from `math/trig.js` |
| `Math.pow`, `Math.exp`, `Math.log`, `Math.hypot`, `Math.cbrt`, `Math.fround` | explicit `* / + -` and `S.sqrt` |
| `Math.random` | `Rng` (seeded PCG-style stream, part of world state) |
| `Date.now`, `performance.now` inside the step | nothing — the step must not observe the clock |
| iterating a `Map`/`Set` whose insertion order depends on discovery history | sort by a stable id first |
| `Array.prototype.sort` with a comparator that can return 0 for distinct items | make the comparator a total order (fall back to id) |

Only `+ - * /` and `sqrt` are guaranteed by IEEE-754 to be correctly rounded,
so they are the only float operations the engine is allowed to depend on.
`Math.sin` genuinely differs between V8, JavaScriptCore and SpiderMonkey.

Two further invariants that are just as easy to break by accident:

- **Solve order must be a function of world *state*, never of history.** A
  client that reaches a state by rewinding and replaying must compute exactly
  what a client that walked there directly computes. This is why contacts are
  sorted by `(fixtureA.id, fixtureB.id)` every step rather than kept in
  discovery order.
- **Floating-point addition is not associative.** Re-ordering `a + b + c` into
  `a + (b + c)` changes results. It is allowed — it is just a protocol break,
  see below.

### If you deliberately change simulation results

That is fine and sometimes necessary. It is a **protocol break**, and it needs
all four of these in the same commit:

1. Bump `PROTOCOL_VERSION` in `src/util/settings.ts`. It is written into every
   snapshot header, so mismatched peers are rejected at connect time instead of
   desyncing ten minutes in.
2. Re-record the golden checksums: `node scripts/golden.mjs --write`.
3. Explain the behavioural change in `CHANGELOG.md`.
4. Treat it as at least a **minor** version bump, even if the API is untouched.

Never re-record golden checksums to turn a red CI green unless you know you
changed the arithmetic. If you did not, a red golden test means *this platform
is not deterministic* — which is a genuine bug and the whole point of the test.

---

## Getting set up

```bash
git clone https://github.com/SurenaMHZ/pulse2d.git
cd pulse2d
npm install
npm test
```

Node 18+ is required. There are exactly two dev dependencies (esbuild and
TypeScript) and zero runtime dependencies; please keep it that way.

### The commands

```bash
npm test          # build, then run all test files
npm run test:only # run the tests without rebuilding
npm run check     # type-check, no emit
npm run build     # ESM + CJS + UMD for both backends, plus .d.ts
npm run bench     # performance suite
npm run demo      # interactive demo on http://localhost:8080
node scripts/golden.mjs          # print this machine's determinism digests
node scripts/golden.mjs --check  # compare them against the recorded contract
```

> **Tests run against `dist/`, not `src/`.** They exercise the same bundle a
> user imports, which is why `npm test` builds first. If you are iterating
> quickly, run `npm run build && npm run test:only`.

The test-file list in `package.json` is spelled out explicitly on purpose:
`node --test test/` was removed in Node 22, the `test/**/*.test.mjs` glob needs
Node 21+, and PowerShell does not expand globs at all. An explicit list is the
only form that works everywhere. **If you add a test file, add it to both the
`test` and `test:only` scripts.**

---

## Making a change

### Reproduce before you fix

Every bug fixed in this engine so far was first reproduced headlessly, in the
smallest scene that shows it. Do that before touching the solver. It is very
easy to "fix" jitter by raising `subSteps` and conclude you solved something,
when in fact you only hid a structural error behind extra convergence — that is
precisely how the stale-joint-anchor bug survived as long as it did. A useful
diagnostic: **if raising `subSteps` does not help, the error is structural, not
a convergence problem.**

### Prove the fix by reverting it

Add the regression test first, or at least confirm it fails when you undo the
fix. A test that passes both with and without your change is testing nothing.

### Measure, do not guess, on performance

Wall-clock timing has roughly 30% run-to-run spread on a typical laptop. Use
CPU-profile *ratios* between functions, or sustained throughput over a fixed
budget, best of several trials. `npm run bench` is the shared yardstick. If a
change does not move it, say so honestly rather than claiming an improvement.

### Both backends must pass

`src/math/scalar.ts` is a one-line re-export that selects the scalar backend;
the build aliases it to `scalar.fixed.ts` for the Q16.16 variant. Anything you
write must work in both. In practice that means:

- Do not assume a scalar is a JS `number` — go through `S.add`, `S.mul`, … .
- Do not assume unlimited range. Q16.16 saturates around ±32768 with a
  resolution of 1/65536.

---

## Checklist before opening a pull request

- [ ] `npm run check` is clean
- [ ] `npm test` passes (all test files, both backends exercised)
- [ ] `node scripts/golden.mjs --check` passes, **or** the change is an
      intentional protocol break with `PROTOCOL_VERSION` bumped and the
      checksums re-recorded
- [ ] A new test fails without your fix and passes with it
- [ ] `npm run bench` shows no unexplained regression
- [ ] No banned `Math.*` call entered `src/`
- [ ] `CHANGELOG.md` has an entry under *Unreleased*
- [ ] Public API changes are documented in `docs/API.md` **and** `docs/fa/API.md`

## Style

- TypeScript, strict mode, ESM, `.js` extensions in relative imports (required
  for real ESM resolution even though the sources are `.ts`).
- Comments explain **why**, not what. Every module has a header comment stating
  the invariant it maintains; keep that up to date when you change behaviour.
- English in code, comments and English docs. Persian documentation lives in
  `docs/fa/` and mirrors the English structure.
- No new dependencies without a strong argument.

## Reporting bugs

Please include the scene that reproduces it — ideally as a small script using
the public API, since that is what maintainers can run directly. For suspected
desyncs, `checksumWorld()` per tick on both peers and the first differing tick
number is worth more than any description.

Note that three behaviours look like bugs and are not:

1. An undamped hanging chain swings for a very long time. Energy *does* decay;
   add `linearDamping` if you want it to settle.
2. Bodies spawned overlapping push apart on the first step. Spawn them apart.
3. Separated bodies keep their velocity forever in a frictionless void. That is
   Newton's first law.

One known limitation is documented rather than fixed: in a Newton's cradle
built from *exactly touching* balls, the impulse is shared between them instead
of passing cleanly to the last one. This is inherent to sequential-impulse
solvers (Box2D behaves identically); see `docs/ARCHITECTURE.md` §3.8 for the
four approaches that were tried and measured. Leave a small gap between the
balls and it behaves correctly.

## Licence

By contributing you agree that your contributions are licensed under the MIT
licence, the same as the rest of the project.
