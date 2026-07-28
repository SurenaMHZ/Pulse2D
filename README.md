<div align="center">

# Pulse2D

**A fast, modular, fully deterministic 2D physics engine for real-time games.**

TypeScript · zero dependencies · 33 KB gzipped · bit-identical on every device

[![CI](https://github.com/SurenaMHZ/pulse2d/actions/workflows/ci.yml/badge.svg)](https://github.com/SurenaMHZ/pulse2d/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/pulse2d.svg)](https://www.npmjs.com/package/pulse2d)
[![bundle size](https://img.shields.io/badge/gzipped-33%20KB-blue)](#performance)
[![dependencies](https://img.shields.io/badge/dependencies-0-brightgreen)](package.json)
[![licence](https://img.shields.io/badge/licence-MIT-blue)](LICENSE)

 [Quick start](#quick-start) · [Documentation](#documentation) · [Determinism](docs/DETERMINISM.md) · [API reference](docs/API.md) · [Benchmarks](#performance)

**[مستندات فارسی](docs/fa/README.md)**

</div>

---

## Why Pulse2D

Most JavaScript physics engines give *approximately* the same answer on every
machine. That is fine for single-player, and fatal for lockstep multiplayer: a
one-bit difference in the last mantissa place compounds into two players seeing
completely different worlds within a few seconds.

Pulse2D is built the other way round — **determinism first**:

- **Bit-identical everywhere.** Only the five IEEE-754 operations that are
  *required* to be correctly rounded (`+ - * / sqrt`) are used. `Math.sin`,
  `Math.atan2` and friends are banned engine-wide and replaced with our own
  polynomial kernels, because their results differ between browsers.
- **No hidden state.** Solve order is a function of world *state*, never of
  discovery history — so a client that arrives at a state by rewinding and
  replaying computes exactly what a client that arrived directly computes.
- **Rollback built in.** Snapshots, checksums and a GGPO-style rollback driver
  ship in the box, not as an afterthought.
- **Two scalar backends.** Float64 by default; a Q16.16 fixed-point build for
  when you need to rule out floating point entirely.

Everything else you would expect is here too: circles, capsules, convex
polygons, chains, six joint types, sensors, ray casts, sleeping, continuous
collision, and a Canvas debug renderer.

---

## Install

```bash
npm install pulse2d
```

```ts
import { World, BodyType, Polygon, Circle } from 'pulse2d';
```

The fixed-point build is a separate entry point:

```ts
import { World } from 'pulse2d/fixed';
```

No build step? Drop in the UMD bundle:

```html
<script src="https://unpkg.com/pulse2d/dist/pulse2d.umd.js"></script>
<script>const world = new Pulse2D.World({ gravity: { x: 0, y: -10 } });</script>
```

---

## Quick start

```ts
import { World, BodyType, Polygon, Circle } from 'pulse2d';

// 1. A world. The time step is fixed at construction — see "Fixed time step".
const world = new World({ gravity: { x: 0, y: -10 } });

// 2. Static ground whose top surface sits at y = 0.
const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
ground.addFixture({ shape: Polygon.box(50, 1), friction: 0.6 });

// 3. A bouncing ball.
const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 8 } });
ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0.6 });

// 4. Step.
for (let i = 0; i < 120; i++) world.step();

console.log(ball.getPosition().toFloats()); // { x: 0, y: ~1.4 }
```

### Units

Pulse2D works in **metres, kilograms and seconds**, and is tuned for objects
roughly `0.1 m`–`10 m` across. If your game thinks in pixels, divide by a
constant (30–100 is typical) on the way in and multiply on the way out.
Simulating a 1000-pixel-wide crate directly will feel like watching a building
fall over, because that is what you asked for.

---

## Core concepts

### Bodies, fixtures and shapes

Three layers, each with one job:

| Layer     | Owns                                    | Shared? |
|-----------|-----------------------------------------|---------|
| **Shape** | geometry in local space                 | yes — reuse one `Polygon` across a thousand crates |
| **Fixture** | density, friction, restitution, filtering, sensor flag | no — one per body attachment |
| **Body**  | position, velocity, mass                | no |

```ts
const crateShape = Polygon.box(0.5, 0.5);      // shared geometry

for (let i = 0; i < 1000; i++) {
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: i, y: 5 } });
  body.addFixture({ shape: crateShape, density: 1, friction: 0.5 });
}
```

Attach several fixtures to one body to build **compound** (concave) objects:

```ts
const table = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
table.addFixture({ shape: Polygon.offsetBox(1.0, 0.1,  0.0, 0.5) }); // top
table.addFixture({ shape: Polygon.offsetBox(0.1, 0.5, -0.8, 0.0) }); // left leg
table.addFixture({ shape: Polygon.offsetBox(0.1, 0.5,  0.8, 0.0) }); // right leg
```

### Body types

| Type        | Moved by      | Mass     | Use for |
|-------------|---------------|----------|---------|
| `Static`    | you, directly | infinite | ground, walls — free, never simulated |
| `Kinematic` | its velocity  | infinite | moving platforms, elevators — pushes, is never pushed |
| `Dynamic`   | forces        | finite   | everything else |

### Shapes

```ts
Circle.of(radius, cx?, cy?)                    // cheapest primitive
Capsule.vertical(height, radius)               // best for characters
Capsule.horizontal(width, radius)
Polygon.box(halfWidth, halfHeight)             // convex, up to 8 vertices
Polygon.offsetBox(hw, hh, cx, cy, angle?)
Polygon.regular(sides, radius)
new Polygon(points)                            // convex hull is taken for you
Segment.of(x1, y1, x2, y2)                     // massless, static geometry
ChainShape.fromPoints(points, loop?)           // terrain, with ghost vertices
```

`new Polygon(points)` runs a deterministic gift-wrap hull, so an unordered or
slightly concave point cloud still produces a valid shape.

For terrain, use `ChainShape` rather than a row of loose segments — it wires up
ghost vertices so a box sliding across a seam does not catch on the joins.

```ts
const ground = world.createBody({ type: BodyType.Static });
const contour = [Vec2.of(-50, 0), Vec2.of(0, -2), Vec2.of(50, 0)];
for (const seg of ChainShape.fromPoints(contour)) {
  ground.addFixture({ shape: seg, friction: 0.8 });
}
```

> **Winding matters.** A chain is one-sided: the solid face is to the **left of
> the direction of travel**. Written left-to-right, a contour is solid from
> above. Reverse the points to flip it.

### Materials

```ts
body.addFixture({
  shape: Circle.of(0.5),
  density: 1,        // kg/m² — drives mass and inertia
  friction: 0.6,     // 0 = ice, 1+ = rubber; pair value = sqrt(a·b)
  restitution: 0.4,  // 0 = clay, 1 = perfectly elastic; pair value = max(a, b)
  isSensor: false,   // true = detect overlap, apply no force
  tangentSpeed: 0,   // non-zero turns the surface into a conveyor belt
});
```

Friction uses the **geometric mean** so one very slippery surface dominates the
pair, which matches intuition. Restitution uses the **maximum** so a bouncy
ball stays bouncy against a dead floor.

---

## The fixed time step

`world.step()` takes no delta time. This is deliberate: a variable `dt` makes
results depend on frame rate, which destroys both determinism and stability.

Drive it from a variable render loop with `accumulate`:

```ts
let last = performance.now();

function frame(now) {
  const dt = (now - last) / 1000;
  last = now;

  // Runs 0..maxSteps whole steps; returns the leftover fraction.
  const alpha = world.accumulate(dt, 5);

  render(alpha); // interpolate between the last two states for smooth visuals
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
```

The `maxSteps` cap prevents the death spiral where a slow frame causes extra
steps, which make the next frame slower still.

---

## Events

```ts
world.setListener({
  beginContact({ fixtureA, fixtureB }) { /* started touching */ },
  endContact({ fixtureA, fixtureB })   { /* stopped touching */ },
  beginSensor({ fixtureA, fixtureB })  { /* entered a trigger volume */ },
  endSensor({ fixtureA, fixtureB })    { /* left it */ },

  // Called after the manifold is built, before the solve.
  preSolve({ contact }) {
    // One-way platform: ignore this contact for one step.
    if (playerIsJumpingUp) contact.setEnabled(false);
  },

  // Called after the solve, with the impulses that were actually applied.
  postSolve({ maxNormalImpulse, approachSpeed, fixtureA, fixtureB }) {
    if (maxNormalImpulse > 5) playCrashSound(maxNormalImpulse);
  },
});
```

`postSolve` is how you detect impact *strength* — a light brush and a
head-on collision both fire `beginContact`, but only one has a large impulse.

---

## Collision filtering

Two independent mechanisms, evaluated in order:

```ts
// 1. Groups: same non-zero group always (positive) or never (negative) collide.
//    Perfect for "all parts of this ragdoll ignore each other".
body.addFixture({ shape, filter: { group: -1 } });

// 2. Category / mask bitfields: both directions must agree.
const PLAYER = 0x0001, ENEMY = 0x0002, PICKUP = 0x0004;

body.addFixture({
  shape,
  filter: { category: PLAYER, mask: ENEMY | PICKUP }, // collides with enemies and pickups
});
```

---

## Joints

| Joint | Constrains | Typical use |
|-------|-----------|-------------|
| `RevoluteJoint`  | shared point, free rotation | hinges, wheels, ragdoll elbows |
| `PrismaticJoint` | motion along one axis       | elevators, pistons, sliding doors |
| `DistanceJoint`  | distance between anchors    | ropes, springs, suspension |
| `WeldJoint`      | all three DOF               | breakable structures (use soft mode) |
| `MouseJoint`     | pulls towards a target      | dragging, tractor beams, magnets |
| `MotorJoint`     | drives to a target offset   | collision-aware moving platforms |

```ts
// A hinge at a world point — local anchors are derived for you.
const hinge = world.createRevoluteJointAt(chassis, wheel, wheelX, wheelY, {
  enableMotor: true,
  motorSpeed: -20,        // rad/s
  maxMotorForce: 500,     // N·m the motor may apply
});

// A rope that hangs slack but never stretches past 4 m.
world.createDistanceJoint({
  bodyA: anchor, bodyB: load,
  enableRigid: false, enableLimit: true,
  minLength: 0, maxLength: 4,
});

// A suspension spring.
world.createDistanceJoint({
  bodyA: chassis, bodyB: wheel,
  enableSpring: true, hertz: 4, dampingRatio: 0.7,
});
```

Connected bodies do not collide with each other by default; pass
`collideConnected: true` if you want them to.

---

## Queries

```ts
// Closest hit along a ray.
const hit = world.rayCastClosest(0, 0, 10, 0);
if (hit) {
  console.log(hit.fixture.body.userData, hit.point.toFloats(), hit.normal.toFloats());
}

// Every hit, with full control over the traversal.
world.rayCast(0, 0, 10, 0, (fixture, point, normal, fraction) => {
  if (fixture.isSensor) return -1;  // ignore, keep the current range
  hits.push(fixture);
  return fraction;                  // shrink the search to closer hits only
});

// Everything overlapping a box, or containing a point.
world.queryAABB(-5, 0, 5, 10, (fixture) => { found.push(fixture); return true; });
world.queryPoint(mouseX, mouseY, (fixture) => { picked = fixture; return false; });
```

---

## Fast-moving objects

Bodies that could cross a wall in a single step should set `bullet: true`,
which enables a swept test (conservative advancement) against nearby geometry:

```ts
const projectile = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 1 },
  linearVelocity: { x: 150, y: 0 },  // 2.5 m per step at 60 Hz
  bullet: true,
});
projectile.addFixture({ shape: Circle.of(0.05), density: 5 });
```

Only bullets are swept, so leaving the flag off costs nothing. Moderately fast
objects are already handled by speculative contacts without it.

---

## Networking

Pulse2D ships the three pieces lockstep and rollback netcode need. See
**[docs/NETWORKING.md](docs/NETWORKING.md)** for the full guide.

```ts
import { saveSnapshot, loadSnapshot, checksumWorld, RollbackManager } from 'pulse2d';

// Desync detection: exchange this number and compare.
const digest = checksumWorld(world);

// Manual save / restore.
const snap = saveSnapshot(world);
loadSnapshot(world, snap);

// Or let the rollback driver handle prediction and re-simulation.
const rb = new RollbackManager(world, {
  maxRollbackFrames: 12,
  applyInputs: (tick, inputs) => applyToGame(tick, inputs),
});

rb.addPlayer(localId);
rb.addPlayer(remoteId);

rb.addLocalInput(localId, readInput());
rb.advance();                                  // steps the world once

socket.on('input', ({ tick, playerId, input }) => {
  rb.addRemoteInput(tick, playerId, input);    // rolls back automatically if mispredicted
});
```

> Use `world.rng` instead of `Math.random()` for anything that affects the
> simulation. It is seeded, captured in snapshots and rewound correctly.

### How the determinism claim is enforced

"Bit-identical on every device" is easy to write in a README and hard to keep
true, so it is a test rather than a promise. `test/golden.test.mjs` replays four
scenes — a 60-body tumble, every joint type at once, bullet CCD, and a pyramid
that sleeps then gets woken — and folds a checksum of **every tick** into one
digest. Those digests are recorded in `test/golden.json`, and CI replays them on
Linux, Windows and macOS, on x64 and arm64, across Node 18 → 24, for both scalar
backends.

Digesting the whole trajectory rather than the final state matters: most scenes
settle and fall asleep, so a divergence at tick 200 could otherwise be damped
away before the last tick and go unnoticed.

You can run the same check against your own hardware:

```bash
node scripts/golden.mjs          # print this machine's digests
node scripts/golden.mjs --check  # compare them to the recorded contract
```

If that command passes on your target platform, Pulse2D will agree with every
other platform that passes it.

---

## Debug rendering

```ts
import { DebugDraw } from 'pulse2d';

const draw = new DebugDraw(canvas.getContext('2d'), { pixelsPerMeter: 32 });
draw.flags.contacts = true;
draw.flags.stats = true;

function frame() {
  world.step();
  draw.begin();       // installs the world→screen transform (+y is up)
  draw.drawWorld(world);
  draw.end();
  requestAnimationFrame(frame);
}
```

Layers: `shapes`, `fill`, `joints`, `contacts`, `contactNormals`,
`contactImpulses`, `aabbs`, `centerOfMass`, `sleepState`, `velocities`,
`stats`. Nothing in the simulation depends on the renderer, so it is
tree-shaken out of a production bundle if you never import it.

---

## Performance

Measured on Node 20, x64, single core. Median of 300 steps after warm-up.

| Scenario | Median step | % of a 60 Hz frame |
|---|---:|---:|
| Pyramid, 210 boxes (588 contacts) | 1.46 ms | 9% |
| Pyramid, 465 boxes (1324 contacts) | 3.45 ms | 21% |
| 500 circles falling | 3.30 ms | 20% |
| 1000 circles falling | 7.38 ms | 44% |
| 1000 mixed shapes falling | 8.03 ms | 48% |
| 1000 bodies, all asleep | 1.36 ms | 8% |
| 300 joints (30 ragdoll chains) | <0.01 ms | 0% |

| Netcode operation | Cost |
|---|---:|
| `saveSnapshot` (500 bodies) | 0.22 ms, 71 KB |
| `loadSnapshot` (500 bodies) | 3.84 ms |
| `checksumWorld` (500 bodies) | 0.39 ms |
| 1 s of rollback history @ 60 Hz | 4.2 MB |

Reproduce with `npm run bench`.

**Tuning.** The main knob is `subSteps` (default `4`). More sub-steps means
stiffer stacks and better fast-motion handling, at linear cost. Prefer raising
it over `velocityIterations`.

```ts
const world = new World({
  subSteps: 8,             // stiffer, ~2x the solve cost
  velocityIterations: 2,   // biased iterations per sub-step
  relaxIterations: 1,      // removes the bias overshoot
});
```

---

## Documentation

| Document | Contents |
|---|---|
| **[docs/API.md](docs/API.md)** | Complete API reference for every public class and function |
| **[docs/DETERMINISM.md](docs/DETERMINISM.md)** | How determinism is achieved, what breaks it, and the rules you must follow |
| **[docs/NETWORKING.md](docs/NETWORKING.md)** | Lockstep and rollback integration, desync debugging |
| **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** | Internals: the step pipeline, solver math, broad phase, module layout |
| **[docs/RECIPES.md](docs/RECIPES.md)** | Ready-made solutions: character controllers, one-way platforms, vehicles, explosions |
| **[docs/fa/](docs/fa/README.md)** | مستندات کامل فارسی — Persian translation of all five documents |
| **[CONTRIBUTING.md](CONTRIBUTING.md)** | Development setup, the determinism rules contributors must follow, PR checklist |
| **[CHANGELOG.md](CHANGELOG.md)** | Every release, and which ones changed simulation results |
| **[SECURITY.md](SECURITY.md)** | Threat model — in particular, why snapshots are not a trust boundary |
| **[docs/RELEASING.md](docs/RELEASING.md)** | Release process: one-time GitHub setup, versioning rules, tagging |

Every source file also carries a module-level doc comment explaining *why* it
works the way it does, not just what it does.

---

## Project layout

```
src/
  math/          Vec2, Rot, Transform, Mat22 · deterministic trig & RNG
    scalar.ts        backend selector  ← swap this to change number type
    scalar.f64.ts    IEEE-754 double backend (default)
    scalar.fixed.ts  Q16.16 fixed-point backend
  collision/     shapes, AABB, GJK distance, SAT narrow phase, BVH broad phase
  dynamics/      Body, Fixture, Contact, Solver, World, continuous collision
    joints/          six joint types on a shared base class
  net/           snapshots, checksums, rollback driver
  render/        Canvas debug renderer (optional)
  util/          tuning constants
```

Every module is side-effect free and independently importable.

---

## Building from source

```bash
npm install
npm test          # 260 tests (builds first — see note)
npm run build     # bundles + type declarations into dist/
npm run bench     # performance suite
npm run check     # type-check without emitting
npm run demo      # interactive demo on http://localhost:8080

node scripts/golden.mjs --check   # verify the determinism contract on this machine
```

### The demo

`npm run demo` serves an interactive playground with **16 scenes** (pyramids,
dominoes, a rope bridge, plinko, a tumbler, billiards, a destructible castle,
a soft body, conveyors, a car with spring suspension, an 800-body stress
test…) and **6 tools** — drag, spawn, cannon, explosion and a live ray cast.

Sub-steps, gravity, sleeping and warm starting are all adjustable while it
runs, and a one-click button re-runs the current scene twice from the same seed
to prove the checksums match tick for tick.

The build emits ESM, CJS and UMD for both scalar backends, plus `.d.ts` files.

> **The tests run against `dist/`, not `src/`**, so they exercise the same
> bundle your game will import. `npm test` therefore builds first via a
> `pretest` hook — no separate build step needed.

---

## Requirements

Any ES2020 runtime: Chrome 90+, Firefox 90+, Safari 15+, Node 18+.
No dependencies, no WebAssembly, no build-time code generation.

Verified on Node 20 and Node 24, on Linux and Windows (PowerShell).

---

## Licence

MIT
