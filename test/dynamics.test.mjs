/**
 * Dynamics: integration, contacts, stacking, friction, restitution, joints,
 * sleeping, events and queries.
 */
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  World,
  BodyType,
  Circle,
  Capsule,
  Polygon,
  ChainShape,
  Vec2,
  Scalar as S,
} from '../dist/pulse2d.mjs';

const f = S.toFloat;

/** A world with a wide static floor whose top surface sits at y = 0. */
function worldWithGround(def = {}) {
  const world = new World({ gravity: { x: 0, y: -10 }, ...def });
  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(50, 1), friction: 0.6 });
  return { world, ground };
}

const steps = (world, n) => {
  for (let i = 0; i < n; i++) world.step();
};

/* ------------------------- free fall ---------------------------- */

test('a free body accelerates under gravity', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 } });
  body.addFixture({ shape: Circle.of(0.5), density: 1 });

  steps(world, 60); // one second
  const vy = f(body.linearVelocity.y);
  assert.ok(Math.abs(vy + 10) < 0.2, `after 1s velocity should be ~-10, got ${vy}`);
  const y = f(body.getPosition().y);
  // semi-implicit Euler overshoots the analytic -5 slightly; allow 5%
  assert.ok(y < -4.5 && y > -5.5, `after 1s y should be ~-5, got ${y}`);
});

test('gravityScale zero makes a body float', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const body = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 5 },
    gravityScale: 0,
  });
  body.addFixture({ shape: Circle.of(0.5), density: 1 });
  steps(world, 120);
  assert.ok(Math.abs(f(body.getPosition().y) - 5) < 1e-6);
});

test('linear damping slows a body without reversing it', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0 },
    linearVelocity: { x: 10, y: 0 },
    linearDamping: 2,
  });
  body.addFixture({ shape: Circle.of(0.5), density: 1 });
  steps(world, 120);
  const vx = f(body.linearVelocity.x);
  assert.ok(vx > 0 && vx < 1, `damped velocity should decay towards 0, got ${vx}`);
});

test('damping is stable even with an absurd coefficient', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({
    type: BodyType.Dynamic,
    linearVelocity: { x: 10, y: 0 },
    linearDamping: 10000,
  });
  body.addFixture({ shape: Circle.of(0.5), density: 1 });
  steps(world, 10);
  const vx = f(body.linearVelocity.x);
  assert.ok(Number.isFinite(vx) && vx >= 0 && vx < 10, `implicit damping must not blow up: ${vx}`);
});

/* --------------------------- resting ---------------------------- */

test('a box comes to rest on the ground at the right height', () => {
  const { world } = worldWithGround();
  const box = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });

  steps(world, 180);
  const y = f(box.getPosition().y);
  // half-height 0.5, resting on y=0, tolerating the linear slop
  assert.ok(Math.abs(y - 0.5) < 0.02, `box should rest at y≈0.5, got ${y}`);
  assert.ok(Math.abs(f(box.linearVelocity.y)) < 0.05, 'should be at rest');
});

test('a sphere rests on the ground without sinking', () => {
  const { world } = worldWithGround();
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  ball.addFixture({ shape: Circle.of(0.5), density: 1 });
  steps(world, 180);
  assert.ok(Math.abs(f(ball.getPosition().y) - 0.5) < 0.02);
});

test('a capsule character rests upright', () => {
  const { world } = worldWithGround();
  const body = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 3 },
    fixedRotation: true,
  });
  body.addFixture({ shape: Capsule.vertical(2, 0.4), density: 1 });
  steps(world, 180);
  assert.ok(Math.abs(f(body.getPosition().y) - 1) < 0.03, `y=${f(body.getPosition().y)}`);
  assert.ok(Math.abs(f(body.getAngle())) < 1e-9, 'fixedRotation must hold the angle at 0');
});

test('penetration stays within the slop under load', () => {
  const { world } = worldWithGround();
  for (let i = 0; i < 6; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 + i * 1.02 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  }
  steps(world, 400);
  // the bottom box carries five others and must not be crushed into the floor
  const bottom = [...world.eachBody()].find((b) => b.type === BodyType.Dynamic);
  assert.ok(f(bottom.getPosition().y) > 0.47, `bottom box sank to ${f(bottom.getPosition().y)}`);
});

/* -------------------------- stacking ---------------------------- */

test('a stack of boxes stays standing and settles', () => {
  const { world } = worldWithGround();
  const n = 8;
  const boxes = [];
  for (let i = 0; i < n; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 + i * 1.001 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0.6 });
    boxes.push(b);
  }

  steps(world, 600);
  for (let i = 0; i < n; i++) {
    const p = boxes[i].getPosition();
    assert.ok(Math.abs(f(p.x)) < 0.2, `box ${i} drifted to x=${f(p.x)}`);
    const expected = 0.5 + i * 1.0;
    assert.ok(Math.abs(f(p.y) - expected) < 0.15, `box ${i} at y=${f(p.y)}, expected ~${expected}`);
  }
});

test('a settled stack goes to sleep', () => {
  const { world } = worldWithGround();
  for (let i = 0; i < 5; i++) {
    const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 + i * 1.001 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  }
  steps(world, 600);
  const dynamic = [...world.eachBody()].filter((b) => b.type === BodyType.Dynamic);
  assert.ok(dynamic.every((b) => !b.awake), 'the whole island should sleep');
  assert.equal(world.awakeBodyCount, 0);
});

test('a sleeping body wakes when hit', () => {
  const { world } = worldWithGround();
  const sleeper = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 } });
  sleeper.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  steps(world, 300);
  assert.ok(!sleeper.awake, 'should have fallen asleep');

  const projectile = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 5, y: 0.5 },
    linearVelocity: { x: -20, y: 0 },
    bullet: true,
  });
  projectile.addFixture({ shape: Circle.of(0.3), density: 5 });
  steps(world, 30);
  assert.ok(sleeper.awake, 'impact must wake the sleeping body');
});

/* -------------------------- materials --------------------------- */

test('restitution makes a ball bounce to a sensible height', () => {
  const { world } = worldWithGround();
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0.8 });

  let peakAfterBounce = -Infinity;
  let hasBounced = false;
  let previousY = 5;
  for (let i = 0; i < 400; i++) {
    world.step();
    const y = f(ball.getPosition().y);
    if (!hasBounced && y < 0.6) hasBounced = true;
    if (hasBounced && y > previousY) peakAfterBounce = Math.max(peakAfterBounce, y);
    previousY = y;
  }
  // e=0.8 returns ~64% of the drop height (4.5 m) => ~2.9 m
  assert.ok(peakAfterBounce > 1.5, `bounce too weak: ${peakAfterBounce}`);
  assert.ok(peakAfterBounce < 4.5, `bounce gained energy: ${peakAfterBounce}`);
});

test('zero restitution does not bounce', () => {
  const { world } = worldWithGround();
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0 });
  let maxAfterLanding = -Infinity;
  let landed = false;
  for (let i = 0; i < 300; i++) {
    world.step();
    const y = f(ball.getPosition().y);
    if (y < 0.55) landed = true;
    if (landed) maxAfterLanding = Math.max(maxAfterLanding, y);
  }
  assert.ok(maxAfterLanding < 0.75, `inelastic ball bounced to ${maxAfterLanding}`);
});

test('a bouncing ball loses energy rather than gaining it', () => {
  const { world } = worldWithGround();
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0.9 });
  const startEnergy = 10 * 5; // m·g·h with m=1 normalised
  for (let i = 0; i < 1200; i++) world.step();
  const y = f(ball.getPosition().y);
  const v = f(ball.linearVelocity.length());
  const energy = 10 * y + 0.5 * v * v;
  assert.ok(energy < startEnergy, `energy grew from ${startEnergy} to ${energy}`);
  assert.ok(Number.isFinite(energy));
});

test('high friction stops a sliding box; zero friction lets it glide', () => {
  const rough = worldWithGround();
  const slick = new World({ gravity: { x: 0, y: -10 } });
  const slickGround = slick.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  slickGround.addFixture({ shape: Polygon.box(50, 1), friction: 0 });

  const a = rough.world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0.5 },
    linearVelocity: { x: 10, y: 0 },
  });
  a.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0.9 });

  const b = slick.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0.5 },
    linearVelocity: { x: 10, y: 0 },
  });
  b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0 });

  steps(rough.world, 120);
  steps(slick, 120);

  assert.ok(Math.abs(f(a.linearVelocity.x)) < 1, `friction should stop it: ${f(a.linearVelocity.x)}`);
  assert.ok(f(b.linearVelocity.x) > 8, `frictionless should keep sliding: ${f(b.linearVelocity.x)}`);
});

/* ------------------------- fast motion -------------------------- */

test('a fast body does not tunnel through a thin wall', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const wall = world.createBody({ type: BodyType.Static, position: { x: 5, y: 0 } });
  wall.addFixture({ shape: Polygon.box(0.05, 5) });

  const bullet = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0 },
    linearVelocity: { x: 150, y: 0 },
    bullet: true,
  });
  bullet.addFixture({ shape: Circle.of(0.2), density: 1 });

  steps(world, 60);
  assert.ok(f(bullet.getPosition().x) < 5.5, `bullet tunnelled to x=${f(bullet.getPosition().x)}`);
});

/* --------------------------- kinematic -------------------------- */

test('a kinematic platform moves and carries a box', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const platform = world.createBody({
    type: BodyType.Kinematic,
    position: { x: 0, y: 0 },
    linearVelocity: { x: 1, y: 0 },
  });
  platform.addFixture({ shape: Polygon.box(3, 0.5), friction: 1 });

  const box = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1.2 } });
  box.addFixture({ shape: Polygon.box(0.4, 0.4), density: 1, friction: 1 });

  steps(world, 120);
  assert.ok(Math.abs(f(platform.getPosition().x) - 2) < 0.05, 'platform advances at 1 m/s');
  assert.ok(f(box.getPosition().x) > 1.0, `box should be carried along, at x=${f(box.getPosition().x)}`);
  assert.ok(f(box.getPosition().y) > 0.85, 'box should stay on top');
});

test('a dynamic body cannot push a kinematic body', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const kin = world.createBody({ type: BodyType.Kinematic, position: { x: 5, y: 0 } });
  kin.addFixture({ shape: Polygon.box(1, 1) });
  const dyn = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0 },
    linearVelocity: { x: 20, y: 0 },
  });
  dyn.addFixture({ shape: Polygon.box(0.5, 0.5), density: 10 });
  steps(world, 60);
  assert.equal(f(kin.linearVelocity.x), 0, 'kinematic bodies ignore impulses');
  assert.ok(f(dyn.linearVelocity.x) < 0.1, 'the dynamic body should have been stopped');
});

/* ----------------------------- forces --------------------------- */

test('an impulse changes velocity by impulse / mass', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic });
  body.addFixture({ shape: Polygon.box(0.5, 0.5), density: 4 }); // mass = 1·4 = 4
  const mass = f(body.mass);
  body.applyLinearImpulse(mass * 3, 0);
  assert.ok(Math.abs(f(body.linearVelocity.x) - 3) < 1e-9);
});

test('a force applied off-centre induces spin', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic });
  body.addFixture({ shape: Polygon.box(1, 1), density: 1 });
  body.applyLinearImpulse(0, 5, 1, 0); // upward impulse at the right edge
  assert.ok(f(body.angularVelocity) > 0, 'should rotate counter-clockwise');
});

test('applyForce accumulates over a step and then clears', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic });
  body.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  body.applyForce(10, 0);
  world.step();
  assert.ok(f(body.linearVelocity.x) > 0);
  assert.equal(f(body.force.x), 0, 'forces reset each step');
  const v = f(body.linearVelocity.x);
  world.step();
  assert.ok(Math.abs(f(body.linearVelocity.x) - v) < 1e-9, 'no force means no further acceleration');
});

/* ----------------------------- joints --------------------------- */

test('a distance joint holds two bodies apart', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const bob = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
  bob.addFixture({ shape: Circle.of(0.3), density: 1 });
  world.createDistanceJoint({ bodyA: anchor, bodyB: bob, length: 3 });

  steps(world, 300);
  const d = Math.hypot(f(bob.getPosition().x) - 0, f(bob.getPosition().y) - 5);
  assert.ok(Math.abs(d - 3) < 0.05, `distance drifted to ${d}`);
});

test('a revolute joint keeps its anchors coincident', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const arm = world.createBody({ type: BodyType.Dynamic, position: { x: 2, y: 5 } });
  arm.addFixture({ shape: Polygon.box(2, 0.2), density: 1 });
  const joint = world.createRevoluteJointAt(anchor, arm, 0, 5);

  steps(world, 300);
  const a = Vec2.zero();
  const b = Vec2.zero();
  joint.getAnchorA(a);
  joint.getAnchorB(b);
  const gap = Math.hypot(f(a.x) - f(b.x), f(a.y) - f(b.y));
  assert.ok(gap < 0.05, `revolute anchors separated by ${gap}`);
});

test('a revolute motor drives rotation', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const base = world.createBody({ type: BodyType.Static });
  const wheel = world.createBody({ type: BodyType.Dynamic });
  wheel.addFixture({ shape: Circle.of(1), density: 1 });
  world.createRevoluteJointAt(base, wheel, 0, 0, {
    enableMotor: true,
    motorSpeed: 5,
    maxMotorForce: 1000,
  });
  steps(world, 60);
  assert.ok(Math.abs(f(wheel.angularVelocity) - 5) < 0.5, `ω=${f(wheel.angularVelocity)}`);
});

test('revolute limits clamp the swing', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const arm = world.createBody({ type: BodyType.Dynamic, position: { x: 1, y: 5 } });
  arm.addFixture({ shape: Polygon.box(1, 0.1), density: 1 });
  const joint = world.createRevoluteJointAt(anchor, arm, 0, 5, {
    enableLimit: true,
    lowerLimit: -0.5,
    upperLimit: 0.5,
  });
  steps(world, 300);
  const angle = f(joint.getJointAngle());
  assert.ok(angle > -0.6 && angle < 0.6, `limit violated: ${angle}`);
});

test('a prismatic joint constrains motion to one axis', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const base = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const slider = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  slider.addFixture({ shape: Polygon.box(0.4, 0.4), density: 1 });
  world.createPrismaticJoint({
    bodyA: base,
    bodyB: slider,
    localAxisA: { x: 1, y: 0 },
    enableLimit: true,
    lowerLimit: -2,
    upperLimit: 2,
  });
  steps(world, 300);
  // gravity pulls down but the joint locks the perpendicular axis
  assert.ok(Math.abs(f(slider.getPosition().y) - 5) < 0.05, `y drifted to ${f(slider.getPosition().y)}`);
  assert.ok(Math.abs(f(slider.getAngle())) < 0.05, 'rotation is locked');
});

test('a weld joint keeps two bodies rigidly together', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const arm = world.createBody({ type: BodyType.Dynamic, position: { x: 1, y: 5 } });
  arm.addFixture({ shape: Polygon.box(1, 0.2), density: 1 });
  world.createWeldJoint({
    bodyA: anchor,
    bodyB: arm,
    localAnchorA: { x: 0, y: 0 },
    localAnchorB: { x: -1, y: 0 },
  });
  steps(world, 300);
  assert.ok(Math.abs(f(arm.getPosition().y) - 5) < 0.1, `weld sagged to ${f(arm.getPosition().y)}`);
  assert.ok(Math.abs(f(arm.getAngle())) < 0.1, 'weld should hold the angle');
});

test('a mouse joint pulls a body to its target', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const ground = world.createBody({ type: BodyType.Static });
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 } });
  body.addFixture({ shape: Circle.of(0.5), density: 1 });
  const joint = world.createMouseJoint({ bodyA: ground, bodyB: body, hertz: 5, maxForce: 500 });
  joint.setTarget(3, 2);
  steps(world, 240);
  const p = body.getPosition();
  assert.ok(Math.hypot(f(p.x) - 3, f(p.y) - 2) < 0.2, `body at ${f(p.x)},${f(p.y)}`);
});

test('a motor joint drives a body to its target offset', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const base = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 } });
  body.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  const joint = world.createMotorJoint({ bodyA: base, bodyB: body, maxForce: 500, maxTorque: 500 });
  joint.setLinearOffset(4, 0);
  steps(world, 240);
  assert.ok(Math.abs(f(body.getPosition().x) - 4) < 0.2, `x=${f(body.getPosition().x)}`);
});

test('connected bodies do not collide unless asked', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const a = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 } });
  a.addFixture({ shape: Polygon.box(1, 1), density: 1 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0.5, y: 0 } });
  b.addFixture({ shape: Polygon.box(1, 1), density: 1 });
  world.createRevoluteJointAt(a, b, 0, 0);
  steps(world, 10);
  const touching = world.contacts.filter((c) => c.isTouching).length;
  assert.equal(touching, 0, 'overlapping jointed bodies must not generate contacts');
});

/* ----------------------------- events --------------------------- */

test('begin and end contact events fire once each', () => {
  const { world } = worldWithGround();
  let begins = 0;
  let ends = 0;
  world.setListener({
    beginContact: () => begins++,
    endContact: () => ends++,
  });
  const ball = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 3 },
    linearVelocity: { x: 0, y: 5 },
  });
  ball.addFixture({ shape: Circle.of(0.3), density: 1, restitution: 0.5 });
  steps(world, 300);
  assert.ok(begins >= 1, 'the ball should touch the ground at least once');
  assert.ok(begins - ends <= 1, `unbalanced begin/end: ${begins}/${ends}`);
});

test('a sensor reports overlap but applies no force', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const trigger = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  trigger.addFixture({ shape: Polygon.box(2, 0.5), isSensor: true });

  let entered = 0;
  let exited = 0;
  world.setListener({
    beginSensor: () => entered++,
    endSensor: () => exited++,
  });

  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  ball.addFixture({ shape: Circle.of(0.3), density: 1 });
  steps(world, 180);

  assert.equal(entered, 1, 'sensor should trigger once');
  assert.equal(exited, 1, 'and release once');
  assert.ok(f(ball.getPosition().y) < -5, 'the ball must fall straight through');
});

test('preSolve can disable a contact (one-way platform)', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const platform = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  platform.addFixture({ shape: Polygon.box(3, 0.2) });
  world.setListener({
    preSolve: (e) => e.contact.setEnabled(false),
  });
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  ball.addFixture({ shape: Circle.of(0.3), density: 1 });
  steps(world, 180);
  assert.ok(f(ball.getPosition().y) < -3, 'a disabled contact must not block the fall');
});

test('postSolve reports the impulse of an impact', () => {
  const { world } = worldWithGround();
  let maxImpulse = 0;
  world.setListener({
    postSolve: (e) => {
      maxImpulse = Math.max(maxImpulse, f(e.maxNormalImpulse));
    },
  });
  const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 8 } });
  ball.addFixture({ shape: Circle.of(0.5), density: 5 });
  steps(world, 200);
  assert.ok(maxImpulse > 0, 'a landing must report a non-zero impulse');
});

/* ---------------------------- filtering ------------------------- */

test('category and mask prevent collisions', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  ground.addFixture({ shape: Polygon.box(50, 1), filter: { category: 0x0002, mask: 0x0002 } });

  const ghost = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  ghost.addFixture({
    shape: Circle.of(0.5),
    density: 1,
    filter: { category: 0x0004, mask: 0x0004 },
  });
  steps(world, 120);
  assert.ok(f(ghost.getPosition().y) < -2, 'mismatched masks must not collide');
});

test('a negative group makes fixtures ignore each other', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const a = world.createBody({
    type: BodyType.Dynamic,
    position: { x: 0, y: 0 },
    linearVelocity: { x: 5, y: 0 },
  });
  a.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, filter: { group: -1 } });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 3, y: 0 } });
  b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, filter: { group: -1 } });
  steps(world, 120);
  assert.ok(Math.abs(f(a.linearVelocity.x) - 5) < 1e-6, 'should pass straight through');
});

/* ----------------------------- queries -------------------------- */

test('rayCastClosest finds the nearest fixture', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  for (const x of [2, 5, 8]) {
    const b = world.createBody({ type: BodyType.Static, position: { x, y: 0 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), userData: `box${x}` });
  }
  const hit = world.rayCastClosest(-5, 0, 20, 0);
  assert.ok(hit, 'the ray should hit something');
  assert.equal(hit.fixture.userData, 'box2');
  assert.ok(Math.abs(f(hit.point.x) - 1.5) < 1e-6, `hit at x=${f(hit.point.x)}`);
});

test('rayCastClosest returns null when nothing is hit', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const b = world.createBody({ type: BodyType.Static, position: { x: 0, y: 50 } });
  b.addFixture({ shape: Polygon.box(0.5, 0.5) });
  assert.equal(world.rayCastClosest(-5, 0, 5, 0), null);
});

test('queryAABB and queryPoint find the right fixtures', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  for (let i = 0; i < 5; i++) {
    const b = world.createBody({ type: BodyType.Static, position: { x: i * 3, y: 0 } });
    b.addFixture({ shape: Polygon.box(0.5, 0.5), userData: i });
  }
  const found = [];
  world.queryAABB(-1, -1, 4, 1, (fx) => {
    found.push(fx.userData);
    return true;
  });
  found.sort();
  assert.deepEqual(found, [0, 1]);

  const atPoint = [];
  world.queryPoint(3, 0, (fx) => {
    atPoint.push(fx.userData);
    return true;
  });
  assert.deepEqual(atPoint, [1]);

  const empty = [];
  world.queryPoint(1.5, 0, (fx) => {
    empty.push(fx.userData);
    return true;
  });
  assert.deepEqual(empty, [], 'the gap between boxes contains nothing');
});

/* --------------------------- lifecycle -------------------------- */

test('destroying a body removes its contacts and joints', () => {
  const { world } = worldWithGround();
  const a = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.5 } });
  a.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  const b = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1.6 } });
  b.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  world.createDistanceJoint({ bodyA: a, bodyB: b, length: 1.1 });
  steps(world, 30);
  assert.ok(world.contactCount > 0);
  assert.equal(world.jointCount, 1);

  world.destroyBody(b);
  steps(world, 5);
  assert.equal(world.jointCount, 0, 'joints referencing the body must go');
  for (const c of world.contacts) {
    assert.ok(c.fixtureA.body !== b && c.fixtureB.body !== b);
  }
  assert.doesNotThrow(() => steps(world, 30));
});

test('setEnabled removes a body from collision and restores it', () => {
  const { world } = worldWithGround();
  const box = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  steps(world, 5);

  // Disabling freezes the body and releases its broad-phase proxy.
  box.setEnabled(false);
  assert.equal(box.fixtures[0].proxyId, -1, 'the proxy should be released');
  const frozenY = f(box.getPosition().y);
  steps(world, 300);
  assert.equal(f(box.getPosition().y), frozenY, 'a disabled body is not simulated at all');

  // Re-enabling re-inserts it and simulation resumes from the current pose.
  box.setTransform(0, 5, 0);
  box.setEnabled(true);
  assert.ok(box.fixtures[0].proxyId >= 0, 'the proxy should be restored');
  steps(world, 300);
  assert.ok(Math.abs(f(box.getPosition().y) - 0.5) < 0.05, 're-enabled body should land');
});

test('changing body type at runtime works', () => {
  const { world } = worldWithGround();
  const box = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1 });
  steps(world, 60);
  assert.equal(f(box.getPosition().y), 5, 'static bodies do not move');

  box.setType(BodyType.Dynamic);
  steps(world, 240);
  assert.ok(Math.abs(f(box.getPosition().y) - 0.5) < 0.05, 'now it should fall and land');
});

test('a compound body combines its fixtures mass', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic });
  body.addFixture({ shape: Polygon.offsetBox(0.5, 0.5, -1, 0), density: 1 });
  body.addFixture({ shape: Polygon.offsetBox(0.5, 0.5, 1, 0), density: 1 });
  assert.ok(Math.abs(f(body.mass) - 2) < 1e-9, `mass ${f(body.mass)}`);
  assert.ok(Math.abs(f(body.localCenter.x)) < 1e-9, 'centre of mass sits between the two');
});

test('setMassData overrides the computed mass', () => {
  const world = new World({ gravity: { x: 0, y: 0 } });
  const body = world.createBody({ type: BodyType.Dynamic });
  body.addFixture({ shape: Polygon.box(1, 1), density: 1 });
  body.setMassData(10, 5);
  assert.equal(f(body.mass), 10);
  assert.equal(f(body.inertia), 5);
  body.clearMassOverride();
  assert.ok(Math.abs(f(body.mass) - 4) < 1e-9, 'clearing restores the computed mass');
});

/* -------------------------- accumulate -------------------------- */

test('accumulate runs whole steps and returns an interpolation alpha', () => {
  const world = new World({ timeStep: 1 / 60, gravity: { x: 0, y: 0 } });
  const alpha = world.accumulate(1 / 60);
  assert.equal(world.tick, 1);
  assert.ok(alpha >= 0 && alpha < 1);

  world.accumulate(1 / 120); // half a step: no tick
  assert.equal(world.tick, 1);
  world.accumulate(1 / 120); // completes the step
  assert.equal(world.tick, 2);
});

test('accumulate clamps the catch-up burst', () => {
  const world = new World({ timeStep: 1 / 60, gravity: { x: 0, y: 0 } });
  world.accumulate(10, 5); // 600 steps worth, capped at 5
  assert.equal(world.tick, 5);
});

/* ------------------------- chain ground ------------------------- */

test('a box slides along a chain without catching on the seams', () => {
  const world = new World({ gravity: { x: 0, y: -10 } });
  const ground = world.createBody({ type: BodyType.Static });
  const pts = [];
  for (let x = -20; x <= 20; x += 2) pts.push(Vec2.of(x, 0));
  for (const seg of ChainShape.fromPoints(pts, false)) {
    ground.addFixture({ shape: seg, friction: 0 });
  }

  const box = world.createBody({
    type: BodyType.Dynamic,
    position: { x: -15, y: 0.5 },
    linearVelocity: { x: 12, y: 0 },
  });
  box.addFixture({ shape: Polygon.box(0.5, 0.5), density: 1, friction: 0 });

  let minVx = Infinity;
  for (let i = 0; i < 150; i++) {
    world.step();
    if (f(box.getPosition().x) > -14 && f(box.getPosition().x) < 14) {
      minVx = Math.min(minVx, f(box.linearVelocity.x));
    }
  }
  assert.ok(minVx > 11, `ghost collision slowed the box to ${minVx}`);
  assert.ok(Math.abs(f(box.getPosition().y) - 0.5) < 0.1, 'box should stay on the surface');
});

/* --------------------- solver correctness ----------------------- */

test('joint reaction force equals the supported weight', () => {
  // A hanging load: statics says the joint must carry exactly m·g, and that
  // must not depend on how many sub-steps the solver happens to use.
  for (const subSteps of [1, 4, 8]) {
    const world = new World({ gravity: { x: 0, y: -10 }, subSteps });
    const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
    const load = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
    load.addFixture({ shape: Polygon.box(0.4, 0.4), density: 50 });
    const joint = world.createDistanceJoint({ bodyA: anchor, bodyB: load, length: 2 });

    steps(world, 200);

    const out = Vec2.zero();
    joint.getReactionForce(out, world.invSubStep);
    const weight = f(load.mass) * 10;
    assert.ok(
      Math.abs(f(out.length()) - weight) < weight * 0.02,
      `subSteps=${subSteps}: reaction ${f(out.length()).toFixed(1)} N, expected ${weight.toFixed(1)} N`,
    );
  }
});

test('joint accuracy is independent of mass', () => {
  // The soft-constraint feedback term must not be scaled by the effective
  // mass, or heavy bodies diverge. A pendulum is the sharpest test.
  const errors = [];
  for (const density of [1, 12, 50]) {
    const world = new World({ gravity: { x: 0, y: -10 }, subSteps: 8 });
    const L = 5;
    const angle = -1.05;
    const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 14 } });
    const bob = world.createBody({
      type: BodyType.Dynamic,
      position: { x: Math.sin(angle) * L, y: 14 - Math.cos(angle) * L },
    });
    bob.addFixture({ shape: Circle.of(0.5), density });
    world.createDistanceJoint({ bodyA: anchor, bodyB: bob, length: L });

    let maxError = 0;
    for (let i = 0; i < 600; i++) {
      world.step();
      const p = bob.getPosition();
      const len = Math.hypot(f(p.x), f(p.y) - 14);
      maxError = Math.max(maxError, Math.abs(len - L));
    }
    assert.ok(maxError < 0.01, `density ${density}: length error ${maxError}`);
    errors.push(maxError);
  }
  // All three densities should give essentially the same error.
  assert.ok(
    Math.abs(errors[0] - errors[2]) < 1e-6,
    `error varies with mass: ${errors.join(', ')}`,
  );
});

test('a heavy pendulum stays stable at high sub-step counts', () => {
  for (const subSteps of [1, 2, 4, 8, 16]) {
    const world = new World({ gravity: { x: 0, y: -10 }, subSteps });
    const L = 5;
    const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 14 } });
    const bob = world.createBody({ type: BodyType.Dynamic, position: { x: -4.34, y: 11.53 } });
    bob.addFixture({ shape: Circle.of(0.5), density: 50 });
    world.createDistanceJoint({ bodyA: anchor, bodyB: bob, length: L });

    steps(world, 600);
    const p = bob.getPosition();
    assert.ok(Number.isFinite(f(p.x)) && Number.isFinite(f(p.y)), `subSteps=${subSteps}: NaN`);
    const len = Math.hypot(f(p.x), f(p.y) - 14);
    assert.ok(Math.abs(len - L) < 0.05, `subSteps=${subSteps}: length drifted to ${len}`);
  }
});
