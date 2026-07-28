# API Reference

[نسخهٔ فارسی](fa/API.md)

Complete reference for every public export of Pulse2D.

Types are shown in TypeScript. `Scalar` is the active backend's number type —
`number` in both builds, but treat it as opaque and convert explicitly with
`Scalar.fromFloat` / `Scalar.toFloat` rather than doing raw arithmetic on it.

**Contents**

- [World](#world) · [Body](#body) · [Fixture](#fixture) · [Shapes](#shapes)
- [Joints](#joints) · [Contacts & events](#contacts--events) · [Filtering](#filtering)
- [Math](#math) · [Networking](#networking) · [Debug rendering](#debug-rendering)
- [Collision internals](#collision-internals) · [Settings](#settings)

---

## World

The simulation container. Owns bodies, fixtures, contacts, joints, the broad
phase and the solver.

### `new World(def?: WorldDef)`

```ts
interface WorldDef {
  gravity?: { x: number; y: number };  // default (0, -10) m/s²
  timeStep?: number;                   // default 1/60 s — fixed for the world's life
  subSteps?: number;                   // default 4
  velocityIterations?: number;         // default 2, per sub-step
  relaxIterations?: number;            // default 1, per sub-step
  enableSleep?: boolean;               // default true
  enableWarmStarting?: boolean;        // default true (disable only to debug)
  enableRestitution?: boolean;         // default true
  seed?: number;                       // seed for world.rng
}
```

`subSteps` is the main quality/cost knob: more sub-steps means stiffer stacks
and better fast-motion handling, at linear cost. Prefer raising it over
`velocityIterations`.

### Stepping

| Member | Description |
|---|---|
| `step(): void` | Advance exactly one fixed step. Deterministic. |
| `accumulate(dt: number, maxSteps?: number): number` | Run whole steps from a variable frame time. Returns the leftover fraction in `[0,1)` for render interpolation. `maxSteps` (default `5`) caps catch-up. |
| `tick: number` | Steps elapsed. The lockstep tick counter. |
| `time: Scalar` | Total simulated time. |
| `timeStep: Scalar` | Read-only step duration. |
| `invSubStep: Scalar` | `1 / (timeStep / subSteps)`. Converts joint impulses to forces. |

### Bodies and joints

| Member | Description |
|---|---|
| `createBody(def?: BodyDef): Body` | Create a body. |
| `destroyBody(body: Body): void` | Destroy it with its fixtures, contacts and joints. |
| `createRevoluteJoint(def)` / `createRevoluteJointAt(bodyA, bodyB, x, y, extra?)` | Hinge. The `At` form derives local anchors from a world point. |
| `createPrismaticJoint(def)` | Slider. |
| `createDistanceJoint(def)` | Distance / spring / rope. |
| `createWeldJoint(def)` | Rigid or soft weld. |
| `createMouseJoint(def)` | Pull towards a target. |
| `createMotorJoint(def)` | Drive to a target offset. |
| `destroyJoint(joint: Joint): void` | Remove a joint. |

### Queries

```ts
rayCastClosest(x1, y1, x2, y2, filter?): { fixture, point, normal, fraction } | null
```
Closest hit, or `null`. `filter` may reject fixtures before they are considered.

```ts
rayCast(x1, y1, x2, y2, cb: (fixture, point, normal, fraction) => Scalar): void
```
Every hit. The callback's return value steers the traversal:

| Return | Meaning |
|---|---|
| `-1` | Ignore this fixture, continue with the current range |
| `0` | Stop immediately |
| `fraction` | Continue, but only report closer hits |
| `1` | Continue with the full range |

```ts
queryAABB(lowerX, lowerY, upperX, upperY, cb: (fixture) => boolean): void
queryPoint(x, y, cb: (fixture) => boolean): void
```
Return `false` from the callback to stop early. `queryPoint` tests actual shape
containment, not just the AABB.

### Inspection

| Member | Description |
|---|---|
| `bodyCount` / `awakeBodyCount` / `contactCount` / `jointCount` | Counts. |
| `eachBody(): IterableIterator<Body>` | Live bodies in id order. |
| `eachJoint(): IterableIterator<Joint>` | Live joints in id order. |
| `contacts: Contact[]` | Live contacts, in canonical solve order. |
| `rng: Rng` | The world's seeded RNG — use it instead of `Math.random`. |
| `profile` | Per-step timings in ms: `total`, `broadPhase`, `narrowPhase`, `solve`, `continuous`, plus counts. |
| `gravity: Vec2` | Mutable, but must match across peers. |

### Other

| Member | Description |
|---|---|
| `setListener(l: WorldListener \| null)` | Install event callbacks. |
| `wakeAll()` | Wake every body. |
| `clear()` | Remove everything; the world stays reusable. |
| `rebuildBroadPhase(discardContacts?)` | Rebuild from current transforms. Called by `loadSnapshot`; also useful after teleporting many bodies at once. |

---

## Body

### `world.createBody(def)`

```ts
interface BodyDef {
  type?: BodyType;                        // default Dynamic
  position?: { x: number; y: number };    // world position of the body origin
  angle?: number;                         // radians
  linearVelocity?: { x: number; y: number };
  angularVelocity?: number;               // rad/s
  linearDamping?: number;                 // 1/s
  angularDamping?: number;                // 1/s
  gravityScale?: number;                  // default 1; 0 makes the body float
  fixedRotation?: boolean;                // lock rotation (platformer characters)
  allowSleep?: boolean;                   // default true
  awake?: boolean;                        // default true
  enabled?: boolean;                      // default true
  bullet?: boolean;                       // enable continuous collision
  userData?: unknown;                     // never touched by the engine
}
```

### `BodyType`

| Value | Moved by | Mass | Collides with |
|---|---|---|---|
| `BodyType.Static` | you, directly | infinite | dynamic, kinematic |
| `BodyType.Kinematic` | its velocity | infinite | dynamic |
| `BodyType.Dynamic` | forces | finite | everything |

### Transform

> The public API works with the body **origin**; the solver internally
> integrates the **centre of mass**. `localCenter` / `worldCenter` bridge them.

| Member | Description |
|---|---|
| `getPosition(): Vec2` | World position of the origin. |
| `getAngle(): Scalar` | Rotation in `(-π, π]`. |
| `setTransform(x, y, angle)` | Teleport. Preserves velocity, wakes the body. |
| `transform: Transform` | Read-only view of position + rotation. |
| `worldCenter` / `localCenter: Vec2` | Centre of mass. |
| `getWorldPoint(out, local)` / `getLocalPoint(out, world)` | Point conversion. |
| `getWorldVector(out, local)` / `getLocalVector(out, world)` | Direction conversion (ignores translation). |
| `getVelocityAtPoint(out, p): Vec2` | Velocity of a world point on this body. |

### Velocity and forces

| Member | Description |
|---|---|
| `linearVelocity: Vec2`, `angularVelocity: Scalar` | Of the centre of mass. |
| `setLinearVelocity(vx, vy)` / `setAngularVelocity(w)` | Setters that also wake the body. |

> **Non-finite input is ignored.** Every setter and force method on `Body`
> silently drops `NaN` and `Infinity` rather than letting it reach the solver,
> where it would spread through contacts and leave a body mysteriously
> unresponsive. `world.gravity` is validated once per step for the same reason.
| `applyForce(fx, fy, px?, py?, wake?)` | Force at a world point; accumulates until the next step. |
| `applyForceToCenter(fx, fy, wake?)` | Force with no torque. |
| `applyTorque(t, wake?)` | Pure torque, N·m. |
| `applyLinearImpulse(ix, iy, px?, py?, wake?)` | Instant velocity change, N·s. For hits, jumps, explosions. |
| `applyAngularImpulse(i, wake?)` | Angular impulse, kg·m²/s. |

**Force vs. impulse:** a force applied every frame produces smooth
acceleration; an impulse changes velocity *now*.

### Mass

| Member | Description |
|---|---|
| `mass`, `invMass`, `inertia`, `invInertia: Scalar` | Read-only. Inertia is about the centre of mass. |
| `resetMassData()` | Recompute from fixtures. Automatic when fixtures change. |
| `setMassData(mass, inertia, cx?, cy?)` | Override manually. |
| `clearMassOverride()` | Return to computed values. |
| `getKineticEnergy(): Scalar` | Total kinetic energy, J. |

A dynamic body with no fixtures (or zero density) is given `mass = 1` so it
still responds to forces instead of silently becoming immovable.

### State

| Member | Description |
|---|---|
| `awake: boolean`, `setAwake(b)` | Sleeping bodies are skipped entirely. Waking resets the sleep timer. |
| `enabled: boolean`, `setEnabled(b)` | Disabled bodies are **frozen**: skipped by the solver and removed from the broad phase. Set the position *before* re-enabling. |
| `setType(t)` | Change body type; mass and contacts are rebuilt. |
| `setFixedRotation(b)` | Lock/unlock rotation. |
| `bullet: boolean` | Continuous collision for this body. |
| `fixtures: Fixture[]` | Attached fixtures, in creation order. |
| `id: number` | Stable dense index. |
| `userData: unknown` | Yours. |

---

## Fixture

Binds a shape to a body with material properties.

### `body.addFixture(def)`

```ts
interface FixtureDef {
  shape: Shape;             // required; may be shared between fixtures
  density?: number;         // kg/m², default 1
  friction?: number;        // default 0.6; pair value = sqrt(a·b)
  restitution?: number;     // default 0;   pair value = max(a, b)
  isSensor?: boolean;       // default false — detects overlap, applies no force
  filter?: Partial<Filter>;
  tangentSpeed?: number;    // conveyor-belt surface speed, m/s
  userData?: unknown;
}
```

| Member | Description |
|---|---|
| `setFilter(f)` | Change filtering; re-evaluates pairs next step. |
| `setDensity(d)` | Then call `body.resetMassData()`. |
| `testPoint(x, y): boolean` | Exact containment test. |
| `aabb: AABB` | Cached world AABB. |
| `body`, `shape`, `id` | Read-only. |

`body.removeFixture(fixture)` detaches and destroys it.

---

## Shapes

Shapes are immutable geometry in local space and can be shared freely.

### Circle

```ts
Circle.of(radius: number, cx?: number, cy?: number): Circle
new Circle(radius: Scalar, center?: Vec2)
```

### Capsule

A segment with a radius — the best character-controller primitive, since it
climbs steps and slides along walls without catching on corners.

```ts
Capsule.vertical(height: number, radius: number): Capsule    // total height
Capsule.horizontal(width: number, radius: number): Capsule
Capsule.of(x1, y1, x2, y2, radius): Capsule
```

### Polygon

Convex, counter-clockwise, up to `MAX_POLYGON_VERTICES` (8).

```ts
Polygon.box(halfWidth, halfHeight, radius?): Polygon
Polygon.offsetBox(hw, hh, cx, cy, angle?): Polygon
Polygon.regular(sides, radius, angleOffset?): Polygon
new Polygon(points: Vec2[], radius?: Scalar)
```

The constructor runs a deterministic gift-wrap convex hull, so unordered,
clockwise, or slightly concave input all produce a valid shape. Duplicate and
collinear points are dropped. The optional `radius` rounds the corners.

Throws if fewer than 3 non-collinear points remain.

### Segment

Zero-thickness edge with **no mass** — static geometry only.

```ts
Segment.of(x1, y1, x2, y2): Segment
segment.setGhosts(prev: Vec2 | null, next: Vec2 | null)
```

### ChainShape

Turns a polyline into segments with ghost vertices wired up, so a body sliding
across a seam does not catch on the join.

```ts
ChainShape.fromPoints(points: Vec2[], loop?: boolean): Segment[]
```

> **Winding:** a chain is one-sided. The solid face is to the **left of the
> direction of travel**, so a contour written left-to-right is solid from
> above, and a counter-clockwise loop is solid on the inside. Reverse the
> points to flip it.

### Common interface

```ts
interface Shape {
  readonly type: ShapeType;        // Circle | Capsule | Polygon | Segment
  readonly radius: Scalar;
  readonly vertexCount: number;
  computeAABB(out: AABB, xf: Transform): AABB;
  computeMass(out: MassData, density: Scalar): MassData;
  testPoint(xf: Transform, p: Vec2): boolean;
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean;
  supportIndex(d: Vec2): number;
  getVertex(i: number): Vec2;
  clone(): Shape;
}
```

`MassData` is `{ mass, center, inertia }`, with inertia about the **local
origin** (not the centroid).

---

## Joints

All joints share a base class:

```ts
interface JointDefBase {
  bodyA: Body;
  bodyB: Body;
  collideConnected?: boolean;   // default false
  userData?: unknown;
}
```

| Member | Description |
|---|---|
| `getAnchorA(out)` / `getAnchorB(out): Vec2` | World anchors. |
| `getReactionForce(out, invDt): Vec2` | Force on body B, N. Pass `world.invSubStep`. |
| `getReactionTorque(invDt): Scalar` | Torque on body B, N·m. Pass `world.invSubStep`. |
| `wake()` | Wake both bodies. |
| `isActive(): boolean` | At least one body awake and simulated. |

Reaction force is how you build breakable joints: destroy the joint when it
exceeds a threshold.

> Joint impulses accumulate **per sub-step**, so the correct conversion factor
> is `world.invSubStep`, not `1 / timeStep`. With it, a hanging load reports
> exactly its weight regardless of the `subSteps` setting.

### RevoluteJoint

Hinge — the two bodies share a point and rotate freely about it.

```ts
interface RevoluteJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  localAnchorA?: { x: number; y: number };
  localAnchorB?: { x: number; y: number };
  referenceAngle?: number;      // defaults to the angle at creation
}
```

`getJointAngle()`, `getJointSpeed()`, `getMotorTorque(invDt)`,
`setLimits(lower, upper)`, `setMotorSpeed(w)`, `setMaxMotorTorque(t)`.

### PrismaticJoint

Slider — B translates along one axis of A; perpendicular motion and relative
rotation are locked.

```ts
interface PrismaticJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  localAnchorA?, localAnchorB?: { x, y };
  localAxisA?: { x: number; y: number };   // normalised for you; default (1,0)
  referenceAngle?: number;
}
```

`getJointTranslation()`, `getJointSpeed()`, `getMotorForce(invDt)`,
`setLimits(lo, hi)`, `setMotorSpeed(v)`.

### DistanceJoint

```ts
interface DistanceJointDef extends JointDefBase, SpringDef, LimitDef, MotorDef {
  localAnchorA?, localAnchorB?: { x, y };
  length?: number;        // defaults to the current anchor distance
  minLength?: number;
  maxLength?: number;
  enableRigid?: boolean;  // default true unless enableSpring is set
}
```

Three modes: **rigid** (fixed length), **spring** (`enableSpring` + `hertz` +
`dampingRatio`), **rope** (`enableLimit` with `minLength`/`maxLength`).

`getCurrentLength()`, `setLength(l)`, `setLengthRange(min, max)`.

### WeldJoint

Fuses all three DOF. A perfectly rigid weld is usually better expressed as one
body with two fixtures — the reason to use this joint is **soft** mode.

```ts
interface WeldJointDef extends JointDefBase {
  localAnchorA?, localAnchorB?: { x, y };
  referenceAngle?: number;
  linearHertz?: number;          // 0 = rigid
  linearDampingRatio?: number;
  angularHertz?: number;         // 0 = rigid
  angularDampingRatio?: number;
}
```

### MouseJoint

Drags body B towards a world target with a soft spring. `bodyA` is ignored
(conventionally a static ground body).

```ts
interface MouseJointDef extends JointDefBase {
  target?: { x: number; y: number };
  hertz?: number;          // default 5
  dampingRatio?: number;   // default 0.7
  maxForce?: number;       // default 1000 N
}
```

`setTarget(x, y)`, `setTargetScalar(x, y)`.

> **Networking:** the target is part of your input stream and must be
> transmitted and quantised like any other input.

### MotorJoint

Drives B to a target offset and angle relative to A, with capped force and
torque. Use it for kinematic characters and moving platforms that must still
respect collisions — unlike teleporting, the body stops naturally against
obstacles.

```ts
interface MotorJointDef extends JointDefBase {
  linearOffset?: { x: number; y: number };
  angularOffset?: number;
  maxForce?: number;           // default 1000
  maxTorque?: number;          // default 1000
  correctionFactor?: number;   // 0..1, default 0.3
}
```

`setLinearOffset(x, y)`, `setAngularOffset(a)`.

### Shared option groups

```ts
interface SpringDef { enableSpring?: boolean; hertz?: number; dampingRatio?: number }
interface MotorDef  { enableMotor?: boolean; motorSpeed?: number; maxMotorForce?: number }
interface LimitDef  { enableLimit?: boolean; lowerLimit?: number; upperLimit?: number }
```

`dampingRatio`: `<1` underdamped (bouncy), `1` critical, `>1` overdamped.

---

## Contacts & events

```ts
world.setListener({
  beginContact(e: ContactEvent) {},
  endContact(e: ContactEvent) {},
  beginSensor(e: ContactEvent) {},
  endSensor(e: ContactEvent) {},
  preSolve(e: ContactEvent) {},   // before the solve — may disable the contact
  postSolve(e: ImpactEvent) {},   // after the solve — carries impulses
});
```

```ts
interface ContactEvent { fixtureA: Fixture; fixtureB: Fixture; contact: Contact }

interface ImpactEvent extends ContactEvent {
  maxNormalImpulse: Scalar;   // largest impulse over the manifold, N·s
  approachSpeed: Scalar;      // pre-impact closing speed, m/s (negative)
}
```

> ⚠️ **Event objects are pooled.** `preSolve` fires once per touching contact
> per step, so allocating a fresh record each time cost measurable GC churn.
> The object passed to a callback is reused on the next call — copy the fields
> you need rather than storing the event.
>
> ```ts
> postSolve(e) {
>   if (Scalar.toFloat(e.maxNormalImpulse) > 5) {
>     hits.push({ id: e.fixtureA.body.id });   // ✓ copy
>   }
> }
> ```

### Contact

| Member | Description |
|---|---|
| `isTouching: boolean` | Shapes actually overlap (not just AABBs). |
| `isSensor: boolean` | Either fixture is a sensor. |
| `setEnabled(b)` / `isEnabled` | Disable for the current step only — the one-way-platform trick. |
| `manifold: Manifold` | Contact points and normal (normal points **A → B**). |
| `friction`, `restitution`, `tangentSpeed: Scalar` | Combined pair values. |
| `getTotalNormalImpulse(): Scalar` | Sum over manifold points. |

### Manifold

```ts
class Manifold {
  points: [ManifoldPoint, ManifoldPoint];
  normal: Vec2;        // from A to B
  pointCount: number;  // 0, 1 or 2
}

class ManifoldPoint {
  point: Vec2;               // world space
  separation: Scalar;        // negative = overlapping
  normalImpulse: Scalar;     // accumulated, persists for warm starting
  tangentImpulse: Scalar;
  maxNormalImpulse: Scalar;
  relativeVelocity: Scalar;  // captured before the solve
  id: ContactID;             // feature id — matches points across steps
  persisted: boolean;
}
```

Two points are enough in 2D to represent both a vertex-on-face and two flat
faces in contact.

---

## Filtering

```ts
interface Filter {
  category: number;   // bitfield: which categories this fixture is
  mask: number;       // bitfield: which categories it collides with
  group: number;      // override; see below
}
```

Evaluated in order:

1. **Group** — two fixtures with the same non-zero group always collide
   (positive) or never collide (negative), overriding masks.
2. **Category/mask** — otherwise both directions must agree:
   `A.mask & B.category` and `B.mask & A.category` must both be non-zero.

Helpers: `makeFilter(partial?)`, `shouldCollide(a, b)`, `DEFAULT_FILTER`.

---

## Math

### Vec2

Mutable `{x, y}` pair. Operators come in **allocating** (`Vec2.add`) and
**destination** (`Vec2.addTo(out, …)`) forms — the solver uses only the latter,
which is why a steady-state step allocates nothing.

```ts
Vec2.of(x: number, y: number): Vec2      // from plain floats
Vec2.zero(): Vec2
v.toFloats(): { x: number; y: number }   // back to plain floats
```

| Static | Meaning |
|---|---|
| `add`, `sub`, `scale`, `neg` | Allocating. |
| `addTo`, `subTo`, `scaleTo`, `negTo`, `lerpTo`, `minTo`, `maxTo` | Into `out`. |
| `addScaledTo(out, a, b, s)` | `out = a + b·s` — the solver workhorse. |
| `combineTo(out, a, sa, b, sb)` | `out = a·sa + b·sb`. |
| `dot(a, b)`, `cross(a, b)` | Products (`cross` is the scalar z component). |
| `crossVS`, `crossSV`, `perpTo`, `rperpTo` | Rotations by ±90°. |
| `distance(a, b)`, `distanceSq(a, b)` | Metrics. |
| `normalizeTo(out, v): Scalar` | Normalise; returns the original length. |
| `equals(a, b)` | Exact component equality. |

Instance: `set`, `setZero`, `copyFrom`, `clone`, `add`, `sub`, `addScaled`,
`scale`, `neg`, `length`, `lengthSq`, `normalize`, `truncate`, `isZero`,
`isValid`.

`normalize()` returns the **previous** length and leaves a zero vector
untouched, so callers never have to guard against NaN.

### Rot

A rotation stored as `(sin θ, cos θ)`, so the solver never calls trig in its
inner loops.

```ts
new Rot(angle?: Scalar)
Rot.of(angle: number): Rot
```

`setAngle`, `setSinCos`, `setIdentity`, `getAngle`, `getXAxis`, `getYAxis`,
`normalize`, `integrate(dAngle)`, `copyFrom`, `clone`.
Statics: `Rot.mulTo`, `mulTTo`, `rotate`, `rotateT`, `relativeAngle`, `nlerpTo`.

`integrate` advances by the small-angle exponential map and re-normalises —
cheaper than recomputing `sinCos`, and stable over long runs.

### Transform

Translation `p` + rotation `q`, mapping local space to world space.

```ts
new Transform(p?: Vec2, q?: Rot)
Transform.apply(out, xf, v)     // local point  → world point
Transform.applyT(out, xf, v)    // world point  → local point
Transform.mulTo(out, a, b)      // compose
Transform.mulTTo(out, a, b)     // b expressed in a's frame
```

### Mat22 / Mat33

2×2 and symmetric 3×3 solvers for coupled constraint blocks.
`set`, `det`, `solve(out, b)`, `invertTo(out)`, `Mat22.apply(out, m, v)`;
`Mat33.solve22`, `solve33`. Singular systems yield zero rather than NaN.

### Deterministic trig

```ts
sin(a), cos(a), tan(a)
sinCos(a, out)        // both at once — roughly 2× faster than calling both
atan(t), atan2(y, x), asin(v), acos(v)
normalizeAngle(a)     // wrap into (-π, π]
```

Polynomial implementations that produce identical results on every platform.
See [DETERMINISM.md](DETERMINISM.md#21-transcendental-functions) for accuracy.

`atan2(0, 0)` is defined as `0`; `asin`/`acos` clamp out-of-range inputs.

### Rng

Seeded PCG-style generator: an LCG state advance plus a murmur3 finalizer.
Chi-square uniform, <0.4% bias on any bit.

```ts
const rng = new Rng(seed?, stream?);
rng.next(): number             // raw uint32
rng.float(): number            // [0, 1)
rng.scalar(lo, hi): Scalar     // backend scalar in a range
rng.int(lo, hi): number        // inclusive, no modulo bias
rng.bool(p?): boolean
rng.shuffle(array): T[]        // Fisher–Yates, deterministic
rng.getState(): [number, number]
rng.setState(s, i): void
rng.seed(n): void
```

### Scalar

```ts
import { Scalar as S } from 'pulse2d';

S.fromFloat(x) / S.toFloat(x)        // conversion — always use these
S.fromInt(i) / S.toInt(x)
S.mul, S.div, S.inv, S.sqrt, S.mulAdd(a, b, c)   // a*b + c
S.half, S.sq, S.abs, S.min, S.max, S.clamp, S.sign, S.lerp
S.ZERO, S.ONE, S.TWO, S.HALF, S.PI, S.TWO_PI, S.HALF_PI, S.EPSILON
S.BACKEND       // 'f64' | 'q16.16'
S.IS_FIXED      // boolean
```

`+`, `-`, unary `-` and comparisons work natively in both backends and are used
directly for speed; only `mul`/`div`/`sqrt` need the module.

---

## Networking

See [NETWORKING.md](NETWORKING.md) for the integration guide.

### Snapshots

```ts
saveSnapshot(world, reuse?): Snapshot
loadSnapshot(world, snap): void          // throws on protocol/backend mismatch
cloneSnapshot(snap): Snapshot            // deep copy, for a history buffer
snapshotBytes(snap): number

interface Snapshot { tick: number; data: Float64Array; meta: Int32Array }
```

Captures body transforms and velocities, sleep state, contact impulses, joint
impulses and the RNG state. `loadSnapshot` rebuilds the broad phase and
rediscovers contacts, so the restored world is a pure function of the snapshot.

The world must have the **same set of bodies** (same ids) as when the snapshot
was taken.

### Checksums

```ts
checksumWorld(world, positionsOnly?): number   // FNV-1a over raw bits
checksumSnapshot(snap): number
new Hasher().int(n).float(x).scalar(s).digest() / .hex()

const log = new ChecksumLog(capacity?);
log.record(tick, sum) / log.recordWorld(world) / log.get(tick)
log.findDivergence(remote: Map<number, number>): number   // earliest bad tick, or -1
log.toMap(): Map<number, number>
```

### RollbackManager

```ts
const rb = new RollbackManager(world, {
  maxRollbackFrames: 16,
  applyInputs: (tick, inputs: Map<number, I>) => void,   // must be pure
  predictInput?: (tick, playerId, last) => I | undefined, // default: repeat last
  inputsEqual?: (a, b) => boolean,                        // default: Object.is
  enableChecksums?: boolean,
  onRollback?: (fromTick, toTick, frames) => void,
});
```

| Member | Description |
|---|---|
| `addPlayer(id)` | Register a player so their first input can be predicted and corrected. |
| `addLocalInput(id, input)` | Queue for the next `advance()`. |
| `addRemoteInput(tick, id, input)` | Deliver authoritative input; rolls back automatically on a misprediction. |
| `advance(): number` | Snapshot, apply inputs, step once. |
| `rollbackTo(tick): boolean` | Manual rewind + replay. `false` if too old. |
| `tick`, `oldestTick`, `historyLength`, `historyBytes` | Inspection. |
| `rollbackCount`, `resimulatedTicks` | Diagnostics. |
| `checksums: ChecksumLog` | Populated when `enableChecksums` is set. |
| `reset()` | Drop all history (after a hard resync). |

---

## Debug rendering

```ts
const draw = new DebugDraw(ctx: CanvasRenderingContext2D, {
  pixelsPerMeter?: number;   // default 32
  cameraX?, cameraY?: number;
  lineWidth?: number;        // in pixels, constant regardless of zoom
  flags?: Partial<DebugDrawFlags>;
  colors?: Partial<DebugDrawColors>;
});
```

| Member | Description |
|---|---|
| `begin(clear?)` / `end()` | Install / restore the world→screen transform (**+y is up**). |
| `drawWorld(world)` | Draw all enabled layers. |
| `drawBody(body)`, `drawShape(shape, xf, color, dashed?)` | Individual pieces. |
| `drawContacts(world)`, `drawJoint(joint)`, `drawTree(world)` | Overlays. |
| `strokeAABB`, `fillDot`, `drawCross`, `drawArrow`, `drawStats` | Primitives. |
| `screenToWorld(px, py)` / `worldToScreen(x, y)` | Coordinate conversion — use for mouse picking. |

Flags: `shapes`, `fill`, `joints`, `contacts`, `contactNormals`,
`contactImpulses`, `aabbs`, `centerOfMass`, `sleepState`, `velocities`,
`stats`.

---

## Collision internals

Useful for custom tools; not needed for normal gameplay code.

### AABB

`set`, `copyFrom`, `clone`, `setEmpty`, `getCenter`, `getExtents`, `perimeter`,
`area`, `expand`, `addPoint`, `contains`, `containsPoint`, `isValid`,
`rayCast(p1, d, maxFraction)`; statics `AABB.combineTo`, `combinedPerimeter`,
`overlaps`.

### Narrow phase

```ts
collide(manifold, shapeA, xfA, shapeB, xfB): void
```
Dispatches on shape type and always produces a normal pointing **A → B**.
Also exported individually: `collideCircles`, `collidePolygonCircle`,
`collidePolygons`.

### Distance and shape casting

```ts
shapeDistance(out, { proxyA, proxyB, xfA, xfB, useRadii }): DistanceOutput
shapeCast(out, { proxyA, proxyB, xfA, xfB, translationB, maxFraction }): boolean
makeProxy(shape), makeDistanceOutput(), makeShapeCastOutput()
```

GJK distance between convex shapes, and a conservative-advancement sweep built
on it. Both use a fixed iteration cap, so cost and results are bounded and
reproducible.

### Broad phase

```ts
new DynamicTree(capacity?)
  createProxy(aabb, userData) / destroyProxy(id) / moveProxy(id, aabb, margin, displacement)
  query(aabb, cb) / queryPoint(p, cb) / rayCast(p1, p2, maxFraction, cb)
  getAABB(out, id) / getUserData(id) / getHeight() / getQuality() / validate()
  proxyCount / nodeCount

new BroadPhase(capacity?)
  createProxy / destroyProxy / moveProxy / touchProxy
  updatePairs(cb) / query / queryPoint / rayCast / rebuild / clear
```

A BVH with surface-area-heuristic insertion and AVL rotations, stored in flat
typed arrays. `validate()` returns `null` when the tree is structurally sound,
otherwise a description — handy in tests.

---

## Settings

```ts
import { Settings } from 'pulse2d';
```

Compile-time constants, in metres/kilograms/seconds. All peers must agree on
them; bump `PROTOCOL_VERSION` if you change any.

| Constant | Default | Meaning |
|---|---|---|
| `LINEAR_SLOP` | 0.005 | Tolerated overlap — prevents contact flicker. |
| `ANGULAR_SLOP` | 2° | Angular equivalent. |
| `SPECULATIVE_DISTANCE` | 4 × slop | Range at which speculative contacts form. |
| `AABB_MARGIN` | 0.1 | Broad-phase padding. |
| `MAX_TRANSLATION` | 4 | Per-step movement cap. |
| `MAX_ROTATION` | 0.5π | Per-step rotation cap. |
| `RESTITUTION_THRESHOLD` | 1 | Below this approach speed, no bounce. |
| `SLEEP_LINEAR_TOLERANCE` | 0.01 | Sleep velocity threshold. |
| `SLEEP_ANGULAR_TOLERANCE` | 2°/s | Sleep rotation threshold. |
| `TIME_TO_SLEEP` | 0.5 | Seconds below tolerance before sleeping. |
| `CONTACT_HERTZ` | 30 | Contact stiffness. |
| `CONTACT_DAMPING_RATIO` | 10 | Heavily overdamped, suppresses bounce. |
| `JOINT_HERTZ` | 60 | Default joint stiffness. |
| `JOINT_DAMPING_RATIO` | 2 | Default joint damping. |
| `PROTOCOL_VERSION` | 1 | Embedded in snapshot headers. |

Also exported: `VERSION`, the library version string.
