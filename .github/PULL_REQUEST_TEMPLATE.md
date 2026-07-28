<!--
Thanks for the pull request. The checklist is short but the determinism items
are load-bearing — please do not tick them without running the commands.
-->

## What this changes

<!-- One or two sentences. Link the issue if there is one. -->

## Why

<!-- What was wrong, or what became possible. -->

---

## Does this change simulation results?

- [ ] **No** — `node scripts/golden.mjs --check` passes unchanged.
- [ ] **Yes** — this is a protocol break, and therefore:
  - [ ] `PROTOCOL_VERSION` bumped in `src/util/settings.ts`
  - [ ] golden checksums re-recorded with `node scripts/golden.mjs --write`
  - [ ] the behavioural change is described in `CHANGELOG.md`
  - [ ] version bumped at least a minor, even if the API is unchanged

## Checklist

- [ ] `npm run check` is clean
- [ ] `npm test` passes
- [ ] A new test fails without this change and passes with it
      <!-- If you cannot say this, please explain why below. -->
- [ ] `npm run bench` shows no unexplained regression
- [ ] No `Math.sin`/`cos`/`pow`/`random`/`Date.now` entered `src/`
- [ ] Works on both scalar backends
- [ ] `CHANGELOG.md` updated
- [ ] Public API changes documented in **both** `docs/API.md` and `docs/fa/API.md`

## Evidence

<!--
For a bug fix: what the failing test printed before, and after.
For a performance change: the relevant `npm run bench` lines, before and after.
Please paste numbers rather than describing them — measured beats claimed.
-->
