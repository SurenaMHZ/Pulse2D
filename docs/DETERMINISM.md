# Determinism

[نسخهٔ فارسی](fa/DETERMINISM.md)

> **The guarantee.** Given the same starting state and the same sequence of
> inputs, Pulse2D produces **bit-identical** results on every CPU
> architecture, operating system, browser and JavaScript engine — forever,
> across process restarts, and regardless of whether the state was reached by
> stepping forward or by rewinding and replaying.

This document explains how that is achieved, what would break it, and the rules
your game code must follow to keep it.

---

## 1. Why physics engines normally drift

A lockstep or rollback game sends **inputs**, not positions. Every peer runs
the same simulation and must arrive at the same answer. Floating point makes
that harder than it sounds.

Consider two clients where a ball's velocity differs by one unit in the last
place — `1e-16` relative error. Contacts are decided by *comparisons*: is this
separation negative? Does this point fall inside that face? Sooner or later a
comparison lands on the other side of a boundary on one machine and not the
other. One client generates a contact, the other does not, and from that frame
on the two worlds are unrelated. Typical time to visible divergence: **two to
ten seconds**.

So "close enough" is not a weaker form of determinism. It is the absence of it.

---

## 2. The four sources of non-determinism

### 2.1 Transcendental functions

The ECMAScript specification says `Math.sin` must be "an implementation-
approximated value". It does **not** require correct rounding. V8, SpiderMonkey
and JavaScriptCore each use different polynomial approximations, and V8 has
changed its own implementation between releases.

```js
// This can legitimately differ between two browsers:
Math.sin(1.0)   // 0.8414709848078965  … or the neighbouring double
```

**Banned engine-wide:** `Math.sin`, `Math.cos`, `Math.tan`, `Math.asin`,
`Math.acos`, `Math.atan`, `Math.atan2`, `Math.exp`, `Math.log`, `Math.pow`,
`Math.hypot`, `Math.cbrt`, `**` with a fractional exponent, and `Math.random`.

**Replaced by** [`src/math/trig.ts`](../src/math/trig.ts), which implements
each function as a polynomial built only from `+ - * /`. Same coefficients,
same operation order, same result everywhere.

| Function | Max absolute error vs. exact | Method |
|---|---|---|
| `sin`, `cos` | 1.0e-11 / 2.4e-13 | argument reduction to ±π/4 + degree-9/11 polynomial |
| `atan`, `atan2` | 1.3e-7 | range reduction to [0,1] + degree-15 odd polynomial |
| `asin`, `acos` | 2.0e-6 | via `atan2` and one `sqrt` |

The error is larger than the platform `Math` — and that is fine. What matters
is that **it is the same error on every machine**. A solver tolerance of `5e-3`
does not care about `1e-11`.

### 2.2 What *is* safe

IEEE-754 mandates correct rounding for exactly five operations:

```
+   -   *   /   sqrt
```

Every conforming CPU and JS engine produces the identical bit pattern for
these. The entire engine is built from them and nothing else.

Two related hazards are avoided by construction:

- **FMA (fused multiply-add)** computes `a*b + c` with a single rounding
  instead of two, giving a different result. JavaScript has no FMA operator and
  engines are not permitted to introduce one, so `S.mulAdd(a, b, c)` is written
  as plain `a * b + c` and is safe.
- **x87 80-bit intermediates** plagued 32-bit C++ builds. JavaScript numbers
  are specified as exactly `binary64`, so this cannot occur.

### 2.3 Iteration and container order

A sequential-impulse solver processes constraints one at a time, each seeing
the velocities left by the previous one. **Changing the order changes the
result** in the low bits.

Three places where order could have leaked in, and how each is pinned:

| Risk | Fix |
|---|---|
| Broad-phase pair discovery depends on BVH shape | Pairs are collected, then **sorted by fixture id** and de-duplicated before contacts are created |
| Contact solve order depends on creation history | The contact list is **re-sorted by `(fixtureA.id, fixtureB.id)`** whenever a contact is added |
| `Set`/`Map` iteration order depends on insertion | Never iterated where it affects simulation; all hot paths use dense arrays indexed by id |

The middle one is subtle and worth dwelling on, because it is the bug that
breaks most home-grown rollback implementations:

> A peer that reaches state *S* by stepping forward has discovered its contacts
> in one order. A peer that reaches the *same* state *S* by loading a snapshot
> and replaying discovers them in a different order. Both have identical
> positions and velocities — and then produce different results on the next
> step.

Sorting makes solve order a pure function of state, not of history.

### 2.4 Stale derived state

Snapshots restore bodies. But the broad-phase tree, the contact list, and
accumulated impulses are *derived* state that also affects the future. If
restoring leaves any of it belonging to the abandoned timeline, replay diverges
— usually a second or two later, which makes it maddening to debug.

`loadSnapshot` therefore:

1. restores every body's transform, velocity, sleep state and the RNG;
2. **rebuilds the broad phase from scratch**, because the tree still holds fat
   AABBs computed along the discarded timeline, and those feed the "did this
   proxy move far enough to re-insert?" test;
3. **discards and rediscovers the contact set**, so pairs created or destroyed
   since the snapshot are correctly present or absent;
4. restores accumulated impulses onto the rediscovered manifolds, matched by
   `(fixtureA, fixtureB, featureId)`.

Step 4 matters more than it looks: those impulses are the warm-start seed.
Restoring positions without them produces a visibly different next frame.

---

## 3. The two scalar backends

All arithmetic goes through one module, `src/math/scalar.ts`, which is a
one-line re-export of the active backend.

### Float64 (default)

`Scalar = number`, operations map to native IEEE-754 doubles. ~15 significant
digits, full dynamic range, fastest option. Deterministic because it uses only
the five correctly-rounded operations.

**Use this** unless you have a specific reason not to.

### Q16.16 fixed-point

`Scalar = int32`, interpreted as `value × 65536`. Every operation ends in `| 0`,
so overflow wraps in the single way the ECMAScript spec defines.

| Property | Value |
|---|---|
| Range | ±32768 |
| Resolution | 1/65536 ≈ 1.5e-5 |
| Exact multiply | operands below ~1024 |

Multiplication splits both operands into 16-bit halves so the 64-bit
intermediate is exact without relying on a double holding 2⁶².

**Trade-offs:** slower, far less precise, and a hard range limit — a body that
flies past ±32768 wraps around instead of sailing off. In exchange you get
integer arithmetic that cannot vary even in principle.

**Use this** if you must be bit-identical across platforms where you do not
trust the floating point environment (some older mobile JIT tiers, exotic
embedded runtimes), or if a regulator/anti-cheat requirement demands integers.

```ts
import { World } from 'pulse2d/fixed';    // that is the whole switch
```

Both backends expose an identical API, so the engine compiles against either
unchanged. Snapshots record which backend produced them and `loadSnapshot`
throws on a mismatch rather than silently misreading the data.

---

## 4. Rules for your game code

The engine's guarantee covers the engine. These are the ways *calling code*
usually breaks it.

### 4.1 Never use `Math.random()`

Use the world's seeded generator, which is snapshotted and rewound with
everything else:

```ts
const world = new World({ seed: 12345 });

const angle  = world.rng.scalar(0, Scalar.TWO_PI);
const damage = world.rng.int(5, 10);
const crit   = world.rng.bool(0.15);
```

### 4.2 Never use wall-clock time

`Date.now()` and `performance.now()` differ on every machine. Derive
everything from the tick counter:

```ts
// ✗ if (Date.now() - spawnTime > 3000) despawn();
// ✓
if (world.tick - spawnTick > 180) despawn();   // 3 s at 60 Hz
```

### 4.3 Never branch on floating point that came from outside

Screen coordinates, pointer positions and analogue stick values arrive with
device-dependent precision. **Quantise inputs before they reach the
simulation** — and quantise the value you *transmit*, so every peer applies
bit-identical numbers:

```ts
// Quantise to 1/1024 m, then send that integer over the network.
const quantise = (v) => Math.round(v * 1024);

const input = { aimX: quantise(rawMouseX), aimY: quantise(rawMouseY) };
socket.send(input);
applyLocally(input);          // apply the same quantised value locally
```

### 4.4 Keep object creation deterministic

Body ids are assigned in creation order and drive solve order. Every peer must
create and destroy the same objects in the same order on the same tick. Spawn
from simulation logic, never from a network callback that fires whenever a
packet happens to arrive.

### 4.5 Keep world settings identical

Gravity, `timeStep`, `subSteps`, `velocityIterations`, `relaxIterations` and
everything in `util/settings.ts` must match across peers. Constants in
`settings.ts` are compile-time on purpose — a value that can change at runtime
is a value that will eventually differ between two clients.

`PROTOCOL_VERSION` is embedded in every snapshot header. Bump it whenever you
change a tuning constant, so a mismatched build is rejected at connect time
instead of desyncing ten minutes in.

### 4.6 Mind the `MouseJoint`

`MouseJoint` is normally driven by pointer input. The target position is part
of your input stream and must be transmitted and quantised like any other
input, or peers will diverge.

---

## 5. Verifying determinism

### Checksums

```ts
import { checksumWorld, ChecksumLog } from 'pulse2d';

const log = new ChecksumLog(512);

// Each tick, on every peer:
world.step();
log.recordWorld(world);

// Periodically exchange digests and compare:
const divergedAt = log.findDivergence(remoteDigests);
if (divergedAt >= 0) console.error(`desync first visible at tick ${divergedAt}`);
```

The hash is FNV-1a over the **raw IEEE-754 bits**, not over rounded decimal
strings, so a one-ulp difference — exactly what you are hunting — changes the
digest. `-0` is normalised to `+0` first, since the solver can legitimately
produce either and they are numerically equal.

Cost is about 0.4 ms for 500 bodies, cheap enough to run every tick in
development and every few ticks in production.

### The test suite

`test/determinism.test.mjs` is the executable version of this document:

- two identical worlds stay bit-identical over 1000 steps of a chaotic scene
  (30 mixed bodies, walls, joints, stacking, bouncing);
- checksums match on **every single tick**, not just at the end;
- a snapshot restored after 200 steps reproduces the next 150 steps exactly;
- repeated rewind-and-replay from the same snapshot is stable across five
  attempts;
- contact impulses survive the snapshot round trip (verified via a tall stack,
  which is very sensitive to losing warm-start data);
- a late-input peer that must roll back converges on the same state as a peer
  that received everything on time;
- no NaN or Infinity appears in a 2000-step chaotic run;
- a settled scene reaches an exact fixed point.

```bash
npm test
```

---

## 6. Debugging a desync

1. **Enable checksums on all peers** and record per-tick digests.
2. **Find the first differing tick** with `ChecksumLog.findDivergence`. The
   first one is the only one that matters; everything after is noise.
3. **Snapshot both peers at `tick - 1`** and diff the raw arrays. Identical
   snapshots mean the divergence is in *inputs*; different snapshots mean it is
   in simulation.
4. **If inputs differ**, you have an input-delivery or quantisation bug — check
   rules 4.1–4.3 above.
5. **If state differs**, dump per-body values at the divergence tick and find
   the first body that differs. Then work back: is it touching something whose
   contact appeared on one peer only? That points at a comparison landing on
   different sides of a boundary, which in turn points at accumulated error
   from an earlier tick.
6. **Bisect with `positionsOnly`.** `checksumWorld(world, true)` skips impulses.
   If the position-only digest matches but the full one does not, the drift is
   in the solver's accumulated impulses rather than in the visible state.

A useful sanity check: run the same scenario twice **in one process** and
compare. If that already diverges, the bug is in your game code (rules 4.1–4.4),
not in cross-platform floating point.

---

## 7. What is *not* guaranteed

- **Across Pulse2D versions.** Any change to the solver, a tuning constant or a
  polynomial coefficient changes results. Peers must run the same version;
  `PROTOCOL_VERSION` enforces it.
- **Across scalar backends.** A float64 client and a fixed-point client will
  not agree. Pick one for the whole session; snapshots carry the backend id and
  refuse to cross-load.
- **Under structural divergence.** Snapshots capture state, not structure. If
  one peer created a body the other did not, restoring will not fix it — create
  and destroy objects from deterministic simulation logic.
- **With user data.** Anything in `userData` is yours; the engine never reads
  it and it is not snapshotted.

---

## 8. Summary

| Requirement | Mechanism |
|---|---|
| Same arithmetic everywhere | Only `+ - * / sqrt`; own polynomial trig |
| Same randomness | Seeded PCG-style RNG, in snapshots |
| Same solve order | Pairs and contacts sorted by fixture id |
| Same state after rewind | Broad phase rebuilt, contacts rediscovered, impulses re-matched by feature id |
| Detect any drift | FNV-1a over raw bits, per tick |
| Reject mismatched builds | `PROTOCOL_VERSION` + backend id in snapshot headers |
