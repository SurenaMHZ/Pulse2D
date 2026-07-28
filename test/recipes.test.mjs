/**
 * Recipes — the patterns documented in docs/RECIPES.md, executed.
 *
 * These guard the documentation: if a recipe stops working, this test fails.
 */
import test from 'node:test';
import assert from 'node:assert/strict';

const m = await import('../dist/pulse2d.mjs');
const { World, BodyType, Polygon, Circle, Capsule, Vec2, ChainShape, Scalar: S } = m;
const f = S.toFloat;
const check = (name, cond, info = '') => assert.ok(cond, `${name} ${info}`);

test('character controller: ground check via ray + capsule', () => {
  const w = new World({ gravity: { x: 0, y: -20 } });
  const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(50, 1), friction: 0.6 });
  const p = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 }, fixedRotation: true });
  p.addFixture({ shape: Capsule.vertical(1.8, 0.4), density: 1, friction: 0.2 });
  const HALF = 0.9;
  const grounded = () => {
    const o = p.getPosition();
    const hit = w.rayCastClosest(f(o.x), f(o.y), f(o.x), f(o.y) - HALF - 0.08, fx => fx.body !== p);
    return !!hit;
  };
  for (let i = 0; i < 120; i++) w.step();
  check('character lands', Math.abs(f(p.getPosition().y) - 0.9) < 0.05, 'y=' + f(p.getPosition().y).toFixed(3));
  check('ground check true', grounded());
  // jump
  p.applyLinearImpulse(0, 9 * f(p.mass));
  w.step();
  check('airborne after jump', !grounded() || f(p.linearVelocity.y) > 1);
  // horizontal move with velocity control
  for (let i = 0; i < 200; i++) {
    const v = p.linearVelocity;
    p.setLinearVelocity(6, f(v.y));
    w.step();
  }
  check('moves right', f(p.getPosition().x) > 5, 'x=' + f(p.getPosition().x).toFixed(2));
});

test('one-way platform', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const plat = w.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  const pf = plat.addFixture({ shape: Polygon.box(3, 0.2) });
  const ball = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: -4 }, linearVelocity: { x: 0, y: 14 } });
  ball.addFixture({ shape: Circle.of(0.3), density: 1 });
  w.setListener({
    preSolve(e) {
      const other = e.fixtureA === pf ? e.fixtureB : (e.fixtureB === pf ? e.fixtureA : null);
      if (!other) return;
      const sign = e.fixtureA === pf ? 1 : -1;
      // block only when the body is moving downward relative to the platform
      if (f(other.body.linearVelocity.y) > 0) e.contact.setEnabled(false);
    }
  });
  let passedThrough = false, landedOnTop = false;
  for (let i = 0; i < 400; i++) { w.step(); const y = f(ball.getPosition().y);
    if (y > 0.6) passedThrough = true;
    if (passedThrough && Math.abs(y - 0.5) < 0.06 && Math.abs(f(ball.linearVelocity.y)) < 0.3) landedOnTop = true; }
  check('one-way: passes up', passedThrough);
  check('one-way: lands on top', landedOnTop, 'y=' + f(ball.getPosition().y).toFixed(3));
});

test('explosion', () => {
  const w = new World({ gravity: { x: 0, y: 0 } });
  const bodies = [];
  for (let i = 0; i < 12; i++) {
    const a = (i / 12) * Math.PI * 2;
    const b = w.createBody({ type: BodyType.Dynamic, position: { x: Math.cos(a) * 2, y: Math.sin(a) * 2 } });
    b.addFixture({ shape: Circle.of(0.2), density: 1 });
    bodies.push(b);
  }
  const cx = 0, cy = 0, R = 5, power = 60;
  for (const b of bodies) {
    const p = b.getPosition();
    const dx = f(p.x) - cx, dy = f(p.y) - cy;
    const d = Math.hypot(dx, dy);
    if (d > R || d < 1e-6) continue;
    const falloff = 1 - d / R;
    b.applyLinearImpulse((dx / d) * power * falloff, (dy / d) * power * falloff, f(p.x), f(p.y));
  }
  for (let i = 0; i < 30; i++) w.step();
  const outward = bodies.every(b => {
    const p = b.getPosition();
    return Math.hypot(f(p.x), f(p.y)) > 2.2;
  });
  check('explosion pushes all outward', outward);
});

test('conveyor belt', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const belt = w.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  belt.addFixture({ shape: Polygon.box(10, 0.5), friction: 0.9, tangentSpeed: 4 });
  const box = w.createBody({ type: BodyType.Dynamic, position: { x: -5, y: 1 } });
  box.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.9 });
  for (let i = 0; i < 200; i++) w.step();
  check('conveyor moves box', Math.abs(f(box.getPosition().x) + 5) > 1.0, 'x=' + f(box.getPosition().x).toFixed(2));
});

test('vehicle with motorised wheels', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
  g.addFixture({ shape: Polygon.box(100, 1), friction: 0.9 });
  const chassis = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 1 } });
  chassis.addFixture({ shape: Polygon.box(1.2, 0.3), density: 1 });
  const wheels = [];
  for (const dx of [-0.8, 0.8]) {
    const wh = w.createBody({ type: BodyType.Dynamic, position: { x: dx, y: 0.5 } });
    wh.addFixture({ shape: Circle.of(0.35), density: 1, friction: 1.5 });
    const j = w.createRevoluteJointAt(chassis, wh, dx, 0.5, { enableMotor: true, motorSpeed: -12, maxMotorForce: 60 });
    wheels.push(j);
  }
  for (let i = 0; i < 300; i++) w.step();
  check('vehicle drives forward', f(chassis.getPosition().x) > 3, 'x=' + f(chassis.getPosition().x).toFixed(2));
  check('vehicle stays upright', Math.abs(f(chassis.getAngle())) < 0.6);
});

test('trigger zone with sensor', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const z = w.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  z.addFixture({ shape: Polygon.box(1, 1), isSensor: true });
  let inside = 0;
  w.setListener({ beginSensor: () => inside++, endSensor: () => inside-- });
  const b = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 5 } });
  b.addFixture({ shape: Circle.of(0.2), density: 1 });
  let sawInside = false;
  for (let i = 0; i < 200; i++) { w.step(); if (inside > 0) sawInside = true; }
  check('sensor fires', sawInside);
  check('sensor releases', inside === 0);
});

test('platform via MotorJoint (collision-aware)', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const g = w.createBody({ type: BodyType.Static, position: { x: 0, y: -5 } });
  g.addFixture({ shape: Polygon.box(50, 1) });
  const anchor = w.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
  const plat = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0 }, gravityScale: 0 });
  plat.addFixture({ shape: Polygon.box(1.5, 0.2), density: 5, friction: 1.5 });
  const mj = w.createMotorJoint({ bodyA: anchor, bodyB: plat, maxForce: 8000, maxTorque: 8000, correctionFactor: 0.2 });
  const rider = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 0.46 } });
  rider.addFixture({ shape: Polygon.box(0.25, 0.25), density: 1, friction: 1.5 });
  for (let i = 0; i < 240; i++) {
    const t = i / 60;
    mj.setLinearOffset(Math.sin(t) * 3, 0);
    w.step();
  }
  check('motor platform tracks target', Math.abs(f(plat.getPosition().x) - Math.sin(4) * 3) < 0.6,
    'x=' + f(plat.getPosition().x).toFixed(2) + ' target=' + (Math.sin(4)*3).toFixed(2));
  check('rider carried', f(rider.getPosition().y) > 0.35, 'y=' + f(rider.getPosition().y).toFixed(2));
});

test('terrain from a heightmap', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const g = w.createBody({ type: BodyType.Static });
  const pts = [];
  for (let x = -30; x <= 30; x += 2) pts.push(Vec2.of(x, Math.sin(x * 0.2) * 1.5));
  for (const s of ChainShape.fromPoints(pts)) g.addFixture({ shape: s, friction: 0.8 });
  const ball = w.createBody({ type: BodyType.Dynamic, position: { x: -20, y: 6 } });
  ball.addFixture({ shape: Circle.of(0.4), density: 1, friction: 0.5 });
  for (let i = 0; i < 400; i++) w.step();
  const y = f(ball.getPosition().y), x = f(ball.getPosition().x);
  check('ball rests on terrain', y > Math.sin(x * 0.2) * 1.5 - 0.2 && y < 6, 'x=' + x.toFixed(2) + ' y=' + y.toFixed(2));
});

test('breakable joint', () => {
  const w = new World({ gravity: { x: 0, y: -10 } });
  const anchor = w.createBody({ type: BodyType.Static, position: { x: 0, y: 5 } });
  const load = w.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 3 } });
  load.addFixture({ shape: Polygon.box(0.4, 0.4), density: 50 });
  let j = w.createDistanceJoint({ bodyA: anchor, bodyB: load, length: 2 });
  const MAXF = 200;                       // below the 320 N weight -> must snap
  const out = Vec2.zero();
  let broke = false;
  for (let i = 0; i < 200; i++) {
    w.step();
    if (j) {
      j.getReactionForce(out, w.invSubStep);
      if (f(out.length()) > MAXF) { w.destroyJoint(j); j = null; broke = true; }
    }
  }
  check('joint breaks under load', broke);
  check('load falls after break', f(load.getPosition().y) < 2);
});
