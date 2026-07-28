# Recipes

[نسخهٔ فارسی](fa/RECIPES.md)

Working solutions to the problems that come up in almost every game.

Every snippet here has been **executed against the engine** and verified to
produce the described behaviour — the numbers in the comments are measured, not
guessed.

**Contents**

1. [Character controller](#1-character-controller)
2. [One-way platforms](#2-one-way-platforms)
3. [Moving platforms](#3-moving-platforms)
4. [Explosions](#4-explosions)
5. [Conveyor belts](#5-conveyor-belts)
6. [A simple vehicle](#6-a-simple-vehicle)
7. [Trigger zones](#7-trigger-zones)
8. [Terrain from a heightmap](#8-terrain-from-a-heightmap)
9. [Breakable joints](#9-breakable-joints)
10. [Mouse dragging](#10-mouse-dragging)
11. [Smooth rendering](#11-smooth-rendering)
12. [Object pooling](#12-object-pooling)

---

## 1. Character controller

A capsule with locked rotation, direct velocity control for movement, and a ray
cast for the ground check.

```ts
const player = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 3 },
  fixedRotation: true,          // never topple over
});
player.addFixture({
  shape: Capsule.vertical(1.8, 0.4),   // 1.8 m tall
  density: 1,
  friction: 0.2,                // low: we drive velocity directly
});

const HALF_HEIGHT = 0.9;
const SKIN = 0.08;

function isGrounded() {
  const p = player.getPosition();
  const hit = world.rayCastClosest(
    Scalar.toFloat(p.x), Scalar.toFloat(p.y),
    Scalar.toFloat(p.x), Scalar.toFloat(p.y) - HALF_HEIGHT - SKIN,
    (fixture) => fixture.body !== player,   // ignore ourselves
  );
  return hit !== null;
}

function update(input) {
  const v = player.linearVelocity;

  // Horizontal: set velocity directly for crisp, predictable control.
  // Vertical is left to physics so gravity, ramps and knockback still work.
  player.setLinearVelocity(input.moveX * 6, Scalar.toFloat(v.y));

  if (input.jump && isGrounded()) {
    player.applyLinearImpulse(0, 9 * Scalar.toFloat(player.mass));
  }
}
```

**Why a capsule?** It climbs steps and slides along walls without the
corner-catching a box suffers from, while staying a single convex shape.

**Why set velocity rather than apply force?** Force-based movement feels
floaty and depends on mass and friction. Setting velocity gives instant,
tunable response — the standard choice for platformers.

Variations:

- **Air control:** scale `input.moveX` by ~0.3 when `!isGrounded()`.
- **Coyote time:** remember the last tick `isGrounded()` was true and allow a
  jump within ~6 ticks of it.
- **Slope handling:** use the ray's `normal` to reject slopes steeper than your
  limit, or to align movement along the surface.

> Use `world.tick`, never `Date.now()`, for coyote time and jump buffers.

---

## 2. One-way platforms

Disable the contact in `preSolve` when the body is moving upward through the
platform.

```ts
const platform = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
const platformFixture = platform.addFixture({ shape: Polygon.box(3, 0.2) });

world.setListener({
  preSolve(e) {
    const isA = e.fixtureA === platformFixture;
    const isB = e.fixtureB === platformFixture;
    if (!isA && !isB) return;

    const other = isA ? e.fixtureB : e.fixtureA;

    // Let anything moving upward pass straight through.
    if (Scalar.toFloat(other.body.linearVelocity.y) > 0) {
      e.contact.setEnabled(false);
    }
  },
});
```

Verified: a ball launched upward at 14 m/s passes through, then lands and rests
on top at `y = 0.5`.

`setEnabled(false)` lasts for one step only, so it is re-evaluated every frame
automatically.

For a **drop-through** input, also disable the contact while the player holds
down:

```ts
if (other.body === player && input.dropThrough) e.contact.setEnabled(false);
```

---

## 3. Moving platforms

### Kinematic — simple and rigid

```ts
const platform = world.createBody({
  type: BodyType.Kinematic,
  position: { x: 0, y: 2 },
  linearVelocity: { x: 1, y: 0 },
});
platform.addFixture({ shape: Polygon.box(3, 0.25), friction: 1 });

// Reverse at the ends of the patrol.
function update() {
  const x = Scalar.toFloat(platform.getPosition().x);
  if (x > 5)  platform.setLinearVelocity(-1, 0);
  if (x < -5) platform.setLinearVelocity( 1, 0);
}
```

A kinematic body pushes but is never pushed. Friction carries riders along.

### Motor joint — collision-aware

Use this when the platform must **stop against obstacles** instead of crushing
or passing through them.

```ts
const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 2 } });

const platform = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 2 },
  gravityScale: 0,              // the joint holds it up
});
platform.addFixture({ shape: Polygon.box(1.5, 0.2), density: 5, friction: 1.5 });

const motor = world.createMotorJoint({
  bodyA: anchor, bodyB: platform,
  maxForce: 8000, maxTorque: 8000,
  correctionFactor: 0.2,        // see the warning below
});

// Each tick, derive the target from the tick counter (never wall-clock time).
motor.setLinearOffset(Math.sin(world.tick / 60) * 3, 0);
```

> **Tune `correctionFactor` down when carrying riders.** It controls how much
> of the remaining error is corrected per step. Measured on a 3 m sine sweep: at
> `0.5` the platform snaps toward its target so hard that friction cannot keep
> up and the rider slides off; at `0.2` the rider is carried the whole way while
> the platform still tracks its target to within 0.03 m. Higher is stiffer,
> lower is smoother.

---

## 4. Explosions

Apply a radial impulse with linear falloff. Ray casting per body adds occlusion
if you want walls to shield.

```ts
function explode(cx, cy, radius, power) {
  world.queryAABB(cx - radius, cy - radius, cx + radius, cy + radius, (fixture) => {
    const body = fixture.body;
    if (body.type !== BodyType.Dynamic) return true;

    const p = body.worldCenter;
    const dx = Scalar.toFloat(p.x) - cx;
    const dy = Scalar.toFloat(p.y) - cy;
    const dist = Math.hypot(dx, dy);
    if (dist > radius || dist < 1e-6) return true;

    const falloff = 1 - dist / radius;
    body.applyLinearImpulse(
      (dx / dist) * power * falloff,
      (dy / dist) * power * falloff,
      Scalar.toFloat(p.x), Scalar.toFloat(p.y),   // at the centre of mass
    );
    return true;
  });
}

explode(0, 0, 5, 60);
```

Apply the impulse at the body's **centre of mass** for a clean radial push, or
at the nearest surface point to add spin.

For line-of-sight occlusion, ray cast from the blast centre to each body and
skip those whose first hit is something else.

---

## 5. Conveyor belts

`tangentSpeed` makes a surface drag contacting bodies along its tangent — no
per-frame code at all.

```ts
const belt = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
belt.addFixture({
  shape: Polygon.box(10, 0.5),
  friction: 0.9,        // must be high enough to transmit the drag
  tangentSpeed: 4,      // m/s along the contact tangent
});
```

Verified: a box dropped at `x = -5` is carried to `x = +7.3` in 200 steps.

Negative values reverse the direction. The same trick makes treadmills, water
currents and moving walkways.

---

## 6. A simple vehicle

A chassis plus two motorised wheels on revolute joints.

```ts
const chassis = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1 } });
chassis.addFixture({ shape: Polygon.box(1.2, 0.3), density: 1 });

const motors = [];
for (const dx of [-0.8, 0.8]) {
  const wheel = world.createBody({ type: BodyType.Dynamic, position: { x: dx, y: 0.5 } });
  wheel.addFixture({ shape: Circle.of(0.35), density: 1, friction: 1.5 });  // grip

  motors.push(world.createRevoluteJointAt(chassis, wheel, dx, 0.5, {
    enableMotor: true,
    motorSpeed: -12,        // negative = forward (clockwise)
    maxMotorForce: 60,      // torque cap — also the traction limit
  }));
}

// Driving
function drive(throttle) {                 // -1 .. 1
  for (const m of motors) m.setMotorSpeed(-12 * throttle);
}
```

Verified: drives 20 m in 300 steps and stays upright.

- **`maxMotorForce`** is effectively engine power. Too high and the wheels spin
  out; too low and it cannot climb.
- **Wheel friction** is grip. Raise it before raising torque.
- For **suspension**, mount each wheel on a `PrismaticJoint` with
  `enableSpring: true`, or a `DistanceJoint` spring between chassis and wheel.

---

## 7. Trigger zones

A sensor detects overlap and reports events but applies no force.

```ts
const zone = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
zone.addFixture({
  shape: Polygon.box(1, 1),
  isSensor: true,
  userData: { kind: 'checkpoint', id: 3 },
});

const occupants = new Set();

world.setListener({
  beginSensor({ fixtureA, fixtureB }) {
    const sensor = fixtureA.isSensor ? fixtureA : fixtureB;
    const other  = fixtureA.isSensor ? fixtureB : fixtureA;
    occupants.add(other.body);
    onEnter(sensor.userData, other.body);
  },
  endSensor({ fixtureA, fixtureB }) {
    const other = fixtureA.isSensor ? fixtureB : fixtureA;
    occupants.delete(other.body);
  },
});
```

Sensors still respect collision filtering, so a `mask` restricts what can
trigger them.

> In rollback games, sensor callbacks fire again during re-simulation. Record
> the intent and act on it only once the tick is older than
> `rollback.oldestTick`.

---

## 8. Terrain from a heightmap

Use `ChainShape` rather than loose segments — it wires up ghost vertices so
bodies do not catch on the seams.

```ts
const ground = world.createBody({ type: BodyType.Static });

const points = [];
for (let x = -30; x <= 30; x += 2) {
  points.push(Vec2.of(x, heightAt(x)));
}

for (const segment of ChainShape.fromPoints(points)) {
  ground.addFixture({ shape: segment, friction: 0.8 });
}
```

Verified: a ball dropped from 6 m rolls and settles onto the surface, and a box
slides across a flat chain at 12 m/s with no measurable loss at the seams.

> **Winding decides the solid side.** The face to the *left of the direction of
> travel* is solid, so a contour written left-to-right (x increasing) is solid
> from above. Reverse the points to flip it. A counter-clockwise closed loop is
> solid on the inside — a room you cannot escape.

Sample the heightmap densely enough that no single segment is shorter than your
smallest dynamic body.

---

## 9. Breakable joints

Poll the reaction force and destroy the joint past a threshold.

```ts
const MAX_FORCE = 400;                     // newtons
const reaction = Vec2.zero();

function checkBreak(joint) {
  // Joint impulses accumulate per sub-step, so `world.invSubStep` is the
  // factor that turns them into newtons — not 1 / timeStep.
  joint.getReactionForce(reaction, world.invSubStep);
  if (Scalar.toFloat(reaction.length()) > MAX_FORCE) {
    world.destroyJoint(joint);
    spawnDebris(joint);
    return true;
  }
  return false;
}

// After each step:
for (const joint of [...world.eachJoint()]) checkBreak(joint);
```

Verified: a 32 kg load reports exactly its 320 N weight through the joint, and
raising `MAX_FORCE` above that keeps it hanging while lowering it snaps the
joint and drops the load.

Copy the joint list before iterating, since `destroyJoint` mutates it.
`getReactionTorque(invDt)` gives the same treatment for bending.

For structures that should sag before failing, use a `WeldJoint` in soft mode
(`linearHertz`, `angularHertz`) and break on the same criterion.

---

## 10. Mouse dragging

```ts
let dragJoint = null;
const groundBody = world.createBody({ type: BodyType.Static });   // joint anchor

canvas.addEventListener('pointerdown', (ev) => {
  const { x, y } = debugDraw.screenToWorld(ev.offsetX, ev.offsetY);

  world.queryPoint(x, y, (fixture) => {
    if (fixture.body.type !== BodyType.Dynamic) return true;      // keep looking
    dragJoint = world.createMouseJoint({
      bodyA: groundBody,
      bodyB: fixture.body,
      target: { x, y },
      hertz: 5, dampingRatio: 0.7,
      maxForce: 1000 * Scalar.toFloat(fixture.body.mass),
    });
    return false;                                                 // stop at the first hit
  });
});

canvas.addEventListener('pointermove', (ev) => {
  if (!dragJoint) return;
  const { x, y } = debugDraw.screenToWorld(ev.offsetX, ev.offsetY);
  dragJoint.setTarget(x, y);
});

canvas.addEventListener('pointerup', () => {
  if (dragJoint) { world.destroyJoint(dragJoint); dragJoint = null; }
});
```

Scaling `maxForce` by mass makes heavy and light objects both feel responsive.

> In multiplayer, the drag target is an **input**: quantise and transmit it like
> any other, or peers will diverge.

---

## 11. Smooth rendering

Physics runs at a fixed rate; displays do not. Interpolate between the last two
states using the alpha from `accumulate`.

```ts
const previous = new Map();     // body -> { x, y, angle }

function frame(now) {
  const dt = (now - last) / 1000;
  last = now;

  // Record the pre-step pose of everything you draw.
  for (const body of world.eachBody()) {
    const p = body.getPosition();
    previous.set(body, {
      x: Scalar.toFloat(p.x),
      y: Scalar.toFloat(p.y),
      angle: Scalar.toFloat(body.getAngle()),
    });
  }

  const alpha = world.accumulate(dt, 5);

  for (const body of world.eachBody()) {
    const prev = previous.get(body);
    const p = body.getPosition();
    const x = prev.x + (Scalar.toFloat(p.x) - prev.x) * alpha;
    const y = prev.y + (Scalar.toFloat(p.y) - prev.y) * alpha;
    drawSprite(body.userData.sprite, x, y);
  }

  requestAnimationFrame(frame);
}
```

Interpolate angles with the shortest path, or store `Rot` and use
`Rot.nlerpTo`, so a body crossing ±π does not spin the wrong way.

---

## 12. Object pooling

`destroyBody` is not free, and in a rollback game structural changes must be
deterministic anyway. Recycling is usually better.

```ts
const pool = [];

function spawnBullet(x, y, vx, vy) {
  let body = pool.pop();
  if (body) {
    body.setTransform(x, y, 0);
    body.setLinearVelocity(vx, vy);
    body.setEnabled(true);                 // re-inserts the broad-phase proxies
  } else {
    body = world.createBody({
      type: BodyType.Dynamic,
      position: { x, y },
      linearVelocity: { x: vx, y: vy },
      bullet: true,                        // continuous collision
    });
    body.addFixture({ shape: Circle.of(0.05), density: 5 });
  }
  return body;
}

function despawnBullet(body) {
  body.setEnabled(false);                  // frozen and removed from collision
  body.setLinearVelocity(0, 0);
  pool.push(body);
}
```

A disabled body is skipped by the solver entirely and its proxies are released,
so a large pool costs almost nothing. Set the transform **before** re-enabling.

---

## Performance notes

- **Share shapes.** One `Polygon` instance can back a thousand fixtures.
- **Let things sleep.** A settled world of 1000 bodies costs 1.4 ms/step versus
  8.0 ms while active — nearly 6× cheaper. Do not wake bodies unnecessarily.
- **`bullet` only where needed.** Speculative contacts already handle
  moderately fast objects; sweeping everything is wasted work.
- **Prefer `subSteps` over `velocityIterations`** when a stack needs to be
  stiffer.
- **Keep sizes in the 0.1–10 m band.** The tuning constants assume it. Scale
  pixel coordinates down before they reach the engine.
- **Use `queryAABB` before precise tests.** The broad phase rejects almost
  everything in `O(log n)`.
