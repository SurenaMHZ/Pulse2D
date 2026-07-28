# Architecture

[نسخهٔ فارسی](fa/ARCHITECTURE.md)

Internals of Pulse2D: the step pipeline, the solver mathematics, spatial
structures, and why each design choice was made.

You do not need any of this to use the engine — see the
[README](../README.md) and [API.md](API.md) for that. This is for people
extending the engine, debugging odd behaviour, or evaluating the design.

---

## 1. Module layout

```
src/
  math/          scalar backends · Vec2 · Rot · Transform · Mat22 · trig · rng
  collision/     shapes · AABB · GJK distance · SAT narrow phase · BVH broad phase
  dynamics/      Body · Fixture · Contact · Solver · World · continuous collision
    joints/      six joint types on a shared base
  net/           snapshots · checksums · rollback driver
  render/        Canvas debug renderer (optional)
  util/          tuning constants
```

Dependencies point strictly downward: `math` knows nothing about collision,
`collision` knows nothing about dynamics, and `net`/`render` sit on top. Every
module is side-effect free, so a bundler drops whatever you do not import.

**The scalar indirection.** Every module gets its arithmetic from
`math/scalar.ts`, which is a one-line re-export of the active backend. Swapping
that line (the build does it with an alias) switches the entire engine between
float64 and fixed-point without touching another file.

Only `mul`, `div`, `sqrt`, `inv` and `mulAdd` go through the module; `+`, `-`,
unary minus and comparisons behave identically in both encodings and are
written inline for speed.

---

## 2. The step pipeline

One `world.step()`:

```
1. broad phase      moved proxies re-queried → new pairs → sorted → contacts created
2. narrow phase     manifolds recomputed; begin/end events; preSolve
3. solve            prepare → warm start → sub-steps → restitution → store
3b. continuous      swept tests for bullet bodies
4. sleep            per-island settling
5. sync             fixture AABBs pushed back into the broad phase
6. postSolve        impact events with the impulses that were applied
```

### 2.1 Broad phase

A dynamic AABB tree (BVH) with surface-area-heuristic insertion and AVL
rotations. Only proxies that **moved** are re-queried, so a resting stack costs
essentially nothing.

Each proxy stores a *fat* AABB — the tight box padded by `AABB_MARGIN` and
extended along the direction of travel. A body can move within that padding
without any tree surgery, which is the common case.

Discovered pairs are collected, **sorted by fixture id**, and de-duplicated
before contacts are created. Without the sort, pair order would depend on the
tree's shape, and therefore on simulation history — fatal for rollback.

### 2.2 Narrow phase

Every primitive is viewed as a *rounded convex polygon* (`ConvexProxy`), so one
algorithm covers all shape combinations:

| Case | Method |
|---|---|
| circle vs circle | distance between centres; one point |
| anything vs circle | closest feature (face or vertex); one point |
| everything else | SAT over both proxies' edge normals, then clip the incident edge against the reference face |

Proxies are cached on the shape instance the first time they are needed, so the
steady-state step allocates nothing.

**Speculative contacts.** A contact is created *before* the shapes touch, up to
`SPECULATIVE_DISTANCE` (4 × linear slop). The solver then applies exactly enough
impulse to land on the surface rather than letting the shapes interpenetrate and
pushing them apart afterwards. This removes penetration jitter from stacks and
stops moderately fast bodies from tunnelling without any CCD machinery.

**One-sided chains.** A `Segment` carrying ghost vertices is part of a chain and
collides only on its solid face — the side to the left of the direction of
travel. This is what kills the classic ghost-collision jolt where a box sliding
across tiled ground catches on an interior edge.

### 2.3 Solve

Detailed in [§3](#3-the-solver).

### 2.4 Continuous collision

Only bodies flagged `bullet` are swept, and only when they moved further than
their own smallest half-extent. For each, a conservative-advancement shape cast
against nearby proxies finds the earliest impact; the body is placed just short
of it, leaving a sliver of gap so the next step's narrow phase produces a normal
speculative contact.

Candidates are sorted by fixture id before evaluation, so ties resolve
identically everywhere.

### 2.5 Sleeping

Sleeping is decided per **island**, not per body: a box resting on a moving
platform must stay awake even though it is not moving itself. Islands are found
with a union-find over contacts and joints — near-linear, and index-ordered so
the result is deterministic.

A body accumulates `sleepTime` while below both the linear and angular
tolerances. When the *minimum* over an island exceeds `TIME_TO_SLEEP`, the whole
island sleeps.

One subtlety: waking is propagated only to bodies that are *already asleep*.
Calling `setAwake(true)` on an awake body resets its timer, and doing that every
step for every touching pair would keep a perfectly settled stack awake forever.

---

## 3. The solver

A **soft-step sequential impulse** solver with relaxation, in the spirit of
Box2D v3's TGS-Soft.

### 3.1 Structure

```
prepareContacts / prepareJoints        once per step: anchors, effective masses
for each sub-step:
    integrateVelocities                gravity, forces, damping
    warmStart                          re-apply accumulated impulses
    solve × velocityIterations         with position bias      (useBias = true)
    integratePositions                 move the bodies
    solve × relaxIterations            no bias                 (useBias = false)
applyRestitution                       one bounce pass for the whole step
storeImpulses                          seed next step's warm start
```

Constraints are prepared **once**, using the pose at the start of the step; each
sub-step then tracks how far bodies have moved via the solver's `dp` deltas.
Rebuilding the mass matrices every sub-step would recompute identical values
`subSteps` times — that mistake cost 2.9× performance during development.

Warm starting, by contrast, *is* re-applied each sub-step; it is what keeps deep
stacks converging.

### 3.2 Effective mass

The resistance of a body pair to an impulse along direction `n` at anchors
`rA`, `rB`:

```
k = mA⁻¹ + mB⁻¹ + IA⁻¹·(rA × n)² + IB⁻¹·(rB × n)²
effectiveMass = 1 / k
```

Computed once per contact point per step, turning each solver iteration into a
handful of multiplies.

### 3.3 Soft constraints

A hard constraint tries to remove all penetration in one step, which injects
energy and makes stacks jitter. Instead each contact is a stiff spring-damper
described by a frequency and damping ratio:

```
ω            = 2π · hertz
a1           = 2ζ + h·ω
a2           = h·ω·a1
biasRate     = ω / a1
massScale    = a2 / (1 + a2)
impulseScale = 1 / (1 + a2)
```

The impulse for one point becomes:

```
λ = −(effectiveMass · massScale) · (vn + bias)  −  impulseScale · λ_accumulated
```

Push-out is spread over several steps, so the simulation stays calm — and since
the coefficients depend only on `h` and compile-time constants, they are
reproducible.

> **The feedback term must not be scaled by the effective mass.** Writing this
> as `−m·(massScale·(vn + bias) + impulseScale·λ)` — which reads naturally and
> is an easy mistake — makes the relaxation gain proportional to mass. Light
> bodies then behave correctly while heavy ones diverge: during development a
> pendulum was stable at density 1 and exploded at density 12, with the error
> growing geometrically at ~2.2× per step. The regression test
> `joint accuracy is independent of mass` pins this down.

Contacts against static bodies use double stiffness: ground contacts feel crisp
while body-on-body stacks stay soft.

### 3.4 Three separation regimes

```ts
if (separation > 0) {
  bias = separation * invH;        // speculative: stop exactly at the surface
} else if (useBias) {
  bias = max(biasRate * separation, -maxPushOut);   // soft push-out
  massScale = soft.massScale;
  impulseScale = soft.impulseScale;
}                                   // relax pass: no bias at all
```

### 3.5 Why relaxation

The biased pass deliberately overshoots to close the gap, adding energy. The
relax pass re-solves the *same* constraints with the bias off, removing exactly
the extra velocity the bias introduced. Without it a deep stack visibly pops
apart.

### 3.6 Accumulated impulse clamping

Contacts push but never pull, and friction obeys Coulomb's law. Both clamp the
**accumulated** impulse, not the increment — clamping increments would let a
constraint drift arbitrarily far over many iterations:

```ts
const newImpulse = max(cp.normalImpulse + impulse, 0);
impulse = newImpulse - cp.normalImpulse;    // apply only the delta
cp.normalImpulse = newImpulse;
```

```ts
const maxFriction = friction * cp.normalImpulse;
const newImpulse = clamp(cp.tangentImpulse + impulse, -maxFriction, maxFriction);
```

### 3.7 Restitution as a separate pass

Bounce runs *after* the main solve, using the approach speed captured before any
sub-step ran. Folding it into the bias would compute the bounce from an
already-modified velocity, so a ball dropped from a fixed height would slowly
gain or lose energy. Impacts slower than `RESTITUTION_THRESHOLD` get no
restitution at all, so resting objects do not buzz on numerical noise.

### 3.8 Chained collisions and Newton's cradle

A sequential-impulse solver resolves every contact **within one step**, which
sets a hard limit on how a shock travels through a row of touching bodies.

Strike one end of five balls that are exactly touching. Physically the impulse
passes ball to ball and only the far one leaves. In the solver all four
contacts are prepared together, so the inner ones record an approach speed of
*zero* — they are at rest when the step begins. Restitution bounces each
contact off its recorded speed, so those inner contacts behave perfectly
inelastically: the shock is shared out and all five balls drift off together.

Several plausible fixes were tried during development and each failed
measurably:

| Attempt | Result |
|---|---|
| Iterate the restitution pass | No change — the recorded speeds are still zero |
| Re-measure approach speed between sweeps | Energy grew 1.5× (three balls) to 3.3× (five) |
| Derive approach from the delivered impulse | Diverged; the estimate includes restitution's own impulse |
| Cap the bounce by the pre-restitution impulse | Over-tight: everything became inelastic |

The honest conclusion is that this is a property of the solver family, not a
tuning value — Box2D exhibits it too. **Leave a small gap (~3 cm) between
bodies that should transmit a shock**, so each impact lands on its own step.
With a 4 cm gap the cradle in the demo returns `[0 0 0 0 4.84]` from a 5 m/s
strike, with no energy gained.

### 3.9 Warm starting

Manifold points carry a **feature id** encoding which vertex/face pair produced
them. Each step, impulses are transferred from last step's points to this step's
by matching ids. In a settled stack the correct impulses barely change, so the
first iteration already lands near the answer — the single biggest reason
stacks converge in a couple of iterations instead of hundreds.

The SAT reference-face choice includes a small bias so it stays sticky when two
separations are nearly equal; otherwise ids would flip-flop and warm starting
would be lost.

---

## 4. Solver body layout

The solver copies the hot fields of each awake body into a dense
`SolverBody[]`:

```ts
class SolverBody {
  v: Vec2; w: Scalar;          // velocity
  dp: Vec2; dq: Rot;           // deltas accumulated this step
  q0, q, c0, c;                // rotation/centre at step start and now
  invMass, invInertia, linearDamping, angularDamping, gravityScale;
  bodyIndex; isKinematic; enableSleep;
}
```

Inner loops walk contiguous memory instead of chasing pointers through user
objects, and `Body` stays free to grow without slowing the solver down.

The solver integrates the **centre of mass** — the point about which rotation is
inertia-free. The body origin is recovered at the end as `p = c − R·localCenter`.

---

## 5. Damping

Implicit rather than explicit:

```ts
v *= 1 / (1 + h·d);        // implicit  — unconditionally stable
// v *= (1 - h·d);         // explicit  — flips sign when h·d > 1
```

No combination of step size and damping coefficient can make the velocity
reverse or blow up, which matters when a game drops frames. A damping of 10000
is harmless.

---

## 6. Spatial structures

### Dynamic tree

Nodes live in **flat parallel typed arrays**, not objects:

```
bounds   Float64Array(4n)   lower.x lower.y upper.x upper.y
meta     Int32Array(4n)     parent | child1 | child2 | height
data     Int32Array(n)      payload (fixture id)
```

One contiguous allocation per field; a traversal touches far fewer cache lines
than an object graph. Growth doubles capacity, so it is amortised O(1) and never
happens in the steady state.

- **Insertion** descends to the sibling that grows total perimeter least (SAH).
- **Balancing** uses AVL rotations, keeping height O(log n) with constant work
  per insert.
- **Queries** use a reusable traversal stack, so they allocate nothing.
- **`validate()`** checks parent/child links, heights and bound containment.

`getQuality()` returns internal-node perimeter over root perimeter; near 1 is
ideal, above ~4 means the tree has degenerated.

### GJK distance

Gilbert–Johnson–Keerthi walks the Minkowski difference and converges on the
closest simplex, usually in 2–4 iterations. The iteration cap is fixed, so cost
and results are bounded.

`shapeCast` builds conservative advancement on top: measure the gap, advance by
the largest provably safe step, repeat.

---

## 7. Performance engineering

The hot path was tuned against CPU profiles rather than intuition. What
actually mattered, in order of measured impact:

| Change | Why it mattered |
|---|---|
| Prepare constraints once per step, not per sub-step | Removed `subSteps`× redundant mass-matrix builds — **2.9×** |
| Hoist per-constraint values out of the point loop in `solveContacts` | The loop runs `subSteps × (velIters + relaxIters)` times per step; a property load left inside is paid thousands of times per frame |
| Keep body velocity in locals across both manifold points | Turns repeated `a.v.x` loads/stores into register traffic |
| Pool the contact/impact event records | `preSolve` fires once per touching contact per step; ~2000 short-lived objects per frame became zero |
| Reuse the union-find buffers in `updateSleep` | Two typed arrays and two closures were allocated **every step** |
| Tight-AABB early-out before the narrow phase | The broad phase pads proxies by `AABB_MARGIN`, which otherwise forces full SAT on every near-miss pair |
| Pack broad-phase pairs into one sortable key | Replaced an index array plus comparator closure with a reused `Float64Array` |
| Hoist `makeSoft` and scalar literals out of loops | Identical inputs for every contact; `S.fromFloat(4)` was running in the innermost loop |

Net effect on a 1000-body mixed scene, measured as sustained throughput on a
fixed time budget: **113 → 127 steps/s (+12%)**. A settled 1000-body world
improved far more — 5.0 ms → 1.4 ms per step — because the early-out skips work
that sleeping alone did not.

> **Measure, do not guess.** Wall-clock timing on this machine has ~30%
> run-to-run spread, which is larger than most individual optimizations. The
> reliable signals were (a) CPU-profile *ratios* between functions and
> (b) sustained throughput over a fixed budget, taking the best of several
> trials. A single `median step time` reading is noise.

## 8. Memory

The steady-state step performs **zero allocations**:

- module-level scratch vectors in every hot file;
- destination-form vector math (`Vec2.addTo(out, a, b)`) throughout the solver;
- pooled solver bodies and contact constraints that grow but never shrink;
- typed arrays in the broad phase;
- convex proxies cached on shape instances.

No garbage means no GC pauses mid-frame, which matters more for frame-time
consistency than raw throughput.

---

## 9. Conservation laws

The engine is audited against the invariants a discrete solver must respect.
The asymmetry matters: **losing** a little energy to discretisation is
acceptable and unavoidable, **gaining** any is a bug — it compounds into
creeping stacks, twitching ragdolls and piles that slowly launch themselves
apart.

| Invariant | Measured |
|---|---|
| Linear momentum (no gravity/friction) | conserved to 4.9e-16 over 500 steps |
| Angular momentum of a free body | conserved to 1e-12 over 1000 steps |
| Energy in a closed elastic box | never exceeds the starting value |
| Free-fall mechanical energy | ±0.07% |
| Pendulum over 5000 steps | ±0.05%, no net gain |
| Joint chain over 3000 steps | 0.000% growth |

`test/conservation.test.mjs` enforces all of these.

### The one place energy does enter

Positional push-out is not energy-conserving by construction: it injects
velocity to resolve overlap, and the relax pass can only withdraw part of it.
That is a deliberate trade — the alternative is letting bodies stay embedded —
but it must be **bounded**, which is what `MAX_BIAS_VELOCITY` does.

The cap is a genuine trade-off between eruption and recovery:

| cap | 0.3 m penetration | 0.6 m penetration | 8-body pile-up |
|---|---:|---:|---:|
| 4 m/s | 0.23 s | 0.38 s | ejected at 9.8 m/s |
| **2 m/s** | **0.23 s** | **0.38 s** | **4.9 m/s** |
| 1 m/s | 0.33 s | never clears | 2.4 m/s |

2 m/s is the setting: it halves the worst-case eruption without slowing
recovery from any penetration a game realistically produces.

### What is *not* a bug

Three behaviours look like leaks and are not:

* **A chain with no damping swings forever.** An ideal pendulum does. Check
  that its energy is flat or falling, not that it stops. Set `linearDamping`
  if the game wants it to settle.
* **Bodies drift apart after spawning overlapped.** Separating is the correct
  response to interpenetration.
* **A body keeps its speed in a frictionless void.** Newton's first law.

## 10. Determinism-critical invariants

If you modify the engine, these are the properties that must not break:

1. **Only `+ - * / sqrt`.** No `Math.sin`/`cos`/`atan2`/`pow`/`random`; use
   `math/trig.ts` and `world.rng`.
2. **Fixed iteration counts.** Never `while (error > tolerance)` — always a hard
   cap, so every machine performs the same number of operations.
3. **Sorted ordering.** Broad-phase pairs, the contact list and CCD candidates
   are sorted by id. Solve order must be a function of state, never of history.
4. **No `Set`/`Map` iteration** anywhere that affects simulation.
5. **Snapshot completeness.** Any new solver state that survives a step must be
   captured, or restored worlds will diverge later.
6. **Bump `PROTOCOL_VERSION`** when results change.

The test suite enforces most of this: `test/determinism.test.mjs` checks
bit-identical replay, snapshot fidelity and rollback convergence.

### The cross-platform contract

`test/determinism.test.mjs` compares two runs *on the same machine*. That is
enough to catch history dependence, and not enough to catch the claim the
library actually makes — that a Windows client and a Linux server agree.

`test/golden.test.mjs` closes that gap. It replays four scenes defined in
`scripts/golden-scenes.mjs` (a 60-body tumble, every joint type at once,
bullet CCD, and a pyramid that sleeps and is then woken) and folds
`checksumWorld` from **every tick** into a single 32-bit digest. The digests
live in `test/golden.json`, and CI replays them on Linux, Windows and macOS,
on x64 and arm64, across Node 18 → 24, for both scalar backends.

Two details matter:

- **The whole trajectory is digested, not the final state.** Most scenes settle
  and fall asleep. A divergence at tick 200 can be damped out by friction long
  before the last tick, so a final-state comparison would miss it.
- **The digest is itself portable.** It uses only `Math.imul`, `^` and `>>>`,
  all of which are exactly specified on 32-bit integers — a digest that relied
  on float arithmetic would be checking the checker.

The recorder and the verifier import the same scene module, so they cannot
drift apart. Re-recording is a manual step (`node scripts/golden.mjs --write`)
precisely because it is how you *lose* this coverage: a red golden test means
either you changed engine arithmetic — in which case bump `PROTOCOL_VERSION` —
or that platform is not deterministic, which is the bug this whole design
exists to catch.

---

## 11. Design trade-offs

| Choice | Rationale | Cost |
|---|---|---|
| Own polynomial trig | Platform `Math` is not reproducible | ~1e-11 error instead of ~1e-16 |
| Single-threaded | Parallel reductions are not bit-reproducible | No multi-core scaling |
| Sorted contacts | Solve order must depend on state only | An O(n log n) sort per step |
| Speculative contacts | Removes jitter and most tunnelling | Contacts exist slightly before touching |
| Sub-stepping over iterations | Better convergence per unit work | More position integrations |
| Flat typed arrays in the BVH | Cache locality | More verbose code |
| Prepare-once, warm-start-per-sub-step | 2.9× faster than re-preparing | Anchors are slightly stale by the last sub-step |
| Joint impulses accumulate per sub-step | Matches the solve structure | Reaction forces need `world.invSubStep`, not `1/dt` |
| Max 8 polygon vertices | Short, cache-friendly SAT loops | Complex shapes need compounds |

---

## 12. Extending the engine

### A custom shape

Implement `Shape` (see `collision/Shape.ts`), then register collision routines
in the `Collide.ts` dispatch table. If your shape is convex, the simplest path
is to expose it as a `ConvexProxy` and reuse `collidePolygons`.

### A custom joint

Extend `Joint` and implement `prepare`, `warmStart`, `solve`, the reaction
accessors and `saveState`/`loadState`. Follow the existing joints' structure:
cache anchors and effective mass in `prepare`, clamp accumulated impulses in
`solve`, and honour the `useBias` flag so relaxation works.

Two traps worth naming, both of which produced real bugs here:

* **Do not multiply the `impulseScale` feedback term by the effective mass**
  (see §3.3). Test with a heavy body — a light one will not reveal it.
* **Refresh the anchor arms every iteration** with `Joint#refreshAnchors`. A
  link that swings tens of degrees within one step invalidates the arms cached
  at prepare time, and the correction then points the wrong way — the joint
  visibly stretches and no amount of sub-stepping helps, because the error is a
  wrong direction rather than an unconverged magnitude.
* **Do not project a sub-step delta onto a cached axis** when that axis rotates
  appreciably within a step. A pendulum's constraint axis sweeps tens of
  degrees per step; projecting onto the stale axis made the joint read itself
  as slack and haul the bob into its anchor. Recompute the separation vector
  and take its true magnitude.

Implementing `saveState`/`loadState` is not optional — without them, rollback
will diverge.

### A custom force

Apply forces before `world.step()`. For deterministic behaviour, derive them
from world state and `world.tick`, never from wall-clock time.

---

## 13. Build and packaging

`scripts/build.mjs` produces five bundles and two sets of declarations from the
single entry point `src/index.ts`:

```
dist/pulse2d.mjs         ESM, float64          dist/pulse2d.fixed.mjs   ESM, Q16.16
dist/pulse2d.cjs         CJS, float64          dist/pulse2d.fixed.cjs   CJS, Q16.16
dist/pulse2d.umd.js      UMD, float64, minified — for a plain <script> tag
dist/types/**.d.ts       ESM declarations
dist/types-cjs/**.d.cts  CommonJS declarations
```

### Swapping the scalar backend

Every module imports arithmetic from `math/scalar.ts`, which is a one-line
re-export. The fixed-point bundles are built by an esbuild alias plugin that
resolves that module to `scalar.fixed.ts` instead. There is no runtime branch
and no duplicated source: the same engine code is compiled twice against two
different number representations, which is also why a backend cannot silently
diverge from the other in behaviour.

### Why two sets of type declarations

The package is `"type": "module"`, which makes every emitted `.d.ts` an *ESM*
declaration file. Under TypeScript's `node16`/`nodenext` resolution a consumer
writing `require('pulse2d')` would then get `TS1479` — "cannot be imported with
require" — despite `dist/pulse2d.cjs` being valid CommonJS that runs fine. The
runtime is correct and only the types are wrong, so the failure appears in the
user's editor rather than in their tests.

The fix is the conventional dual-package one: the build copies the declarations
to `dist/types-cjs/` with the `.d.cts` extension, which is unconditionally
CommonJS regardless of the `type` field, rewriting relative specifiers from
`./x.js` to `./x.cjs` so they resolve to their `.d.cts` siblings. The `require`
condition of the `exports` map points there. CI compiles a probe file against
the *packed tarball* under `node16`, `nodenext` and `bundler`, for both the
default and the `/fixed` entry point, so a broken `exports` map cannot reach
npm.

### Source maps

`sourcesContent` is disabled. The published package ships `src/` alongside
`dist/`, so the maps' relative `../src/…` paths resolve to real files and
stepping into engine source in a debugger still works — embedding a second copy
of every source inside each of four map files would roughly triple the tarball
for no benefit.
