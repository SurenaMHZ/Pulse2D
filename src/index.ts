/**
 * Pulse2D — a fast, modular, fully deterministic 2D physics engine.
 *
 * ```ts
 * import { World, BodyType, Polygon, Circle, Vec2 } from 'pulse2d';
 *
 * const world = new World({ gravity: { x: 0, y: -10 } });
 *
 * const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
 * ground.addFixture({ shape: Polygon.box(50, 1) });
 *
 * const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 8 } });
 * ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0.6 });
 *
 * for (let i = 0; i < 120; i++) world.step();
 * console.log(ball.getPosition().toFloats());
 * ```
 *
 * Everything is exported from this single entry point, and every module is
 * side-effect free, so a bundler drops whatever you do not use.
 *
 * @packageDocumentation
 */

/* ----------------------------- math ------------------------------ */

export * as Scalar from './math/scalar.js';
export type { Scalar as ScalarValue } from './math/scalar.js';
export { Vec2 } from './math/Vec2.js';
export { Rot } from './math/Rot.js';
export { Transform } from './math/Transform.js';
export { Mat22, Mat33 } from './math/Mat22.js';
export { Rng } from './math/rng.js';
export {
  sin,
  cos,
  tan,
  sinCos,
  atan,
  atan2,
  asin,
  acos,
  normalizeAngle,
} from './math/trig.js';

/* --------------------------- collision --------------------------- */

export { AABB } from './collision/AABB.js';
export { ShapeType } from './collision/Shape.js';
export type { Shape, MassData, RayCastInput, RayCastOutput } from './collision/Shape.js';
export { Circle } from './collision/shapes/Circle.js';
export { Capsule } from './collision/shapes/Capsule.js';
export { Polygon, MAX_POLYGON_VERTICES } from './collision/shapes/Polygon.js';
export { Segment, ChainShape } from './collision/shapes/Segment.js';
export { Manifold, ManifoldPoint, makeID } from './collision/Manifold.js';
export type { ContactID } from './collision/Manifold.js';
export { collide, collideCircles, collidePolygonCircle, collidePolygons } from './collision/Collide.js';
export { getProxy } from './collision/ConvexProxy.js';
export type { ConvexProxy } from './collision/ConvexProxy.js';
export {
  shapeDistance,
  shapeCast,
  makeProxy,
  makeDistanceOutput,
  makeShapeCastOutput,
} from './collision/Distance.js';
export type {
  DistanceInput,
  DistanceOutput,
  DistanceProxy,
  ShapeCastInput,
  ShapeCastOutput,
} from './collision/Distance.js';
export { DynamicTree, NULL_NODE } from './collision/DynamicTree.js';
export type { QueryCallback, RayCastCallback } from './collision/DynamicTree.js';
export { BroadPhase } from './collision/BroadPhase.js';
export type { PairCallback } from './collision/BroadPhase.js';

/* --------------------------- dynamics ---------------------------- */

export { World } from './dynamics/World.js';
export type {
  WorldDef,
  WorldListener,
  ContactEvent,
  ImpactEvent,
} from './dynamics/World.js';
export { Body, BodyType } from './dynamics/Body.js';
export type { BodyDef } from './dynamics/Body.js';
export { Fixture } from './dynamics/Fixture.js';
export type { FixtureDef } from './dynamics/Fixture.js';
export { Contact, ContactFlags } from './dynamics/Contact.js';
export { DEFAULT_FILTER, makeFilter, shouldCollide } from './dynamics/Filter.js';
export type { Filter } from './dynamics/Filter.js';
export { Solver, SolverBody, makeSoft } from './dynamics/Solver.js';
export type { SoftConstraint, StepContext } from './dynamics/Solver.js';

/* ---------------------------- joints ----------------------------- */

export { Joint, JointType } from './dynamics/joints/Joint.js';
export type { JointDefBase, SpringDef, MotorDef, LimitDef } from './dynamics/joints/Joint.js';
export { RevoluteJoint } from './dynamics/joints/RevoluteJoint.js';
export type { RevoluteJointDef } from './dynamics/joints/RevoluteJoint.js';
export { DistanceJoint } from './dynamics/joints/DistanceJoint.js';
export type { DistanceJointDef } from './dynamics/joints/DistanceJoint.js';
export { PrismaticJoint } from './dynamics/joints/PrismaticJoint.js';
export type { PrismaticJointDef } from './dynamics/joints/PrismaticJoint.js';
export { WeldJoint } from './dynamics/joints/WeldJoint.js';
export type { WeldJointDef } from './dynamics/joints/WeldJoint.js';
export { MouseJoint } from './dynamics/joints/MouseJoint.js';
export type { MouseJointDef } from './dynamics/joints/MouseJoint.js';
export { MotorJoint } from './dynamics/joints/MotorJoint.js';
export type { MotorJointDef } from './dynamics/joints/MotorJoint.js';

/* ------------------------------ net ------------------------------ */

export { saveSnapshot, loadSnapshot, cloneSnapshot, snapshotBytes } from './net/Snapshot.js';
export type { Snapshot } from './net/Snapshot.js';
export { Hasher, checksumWorld, checksumSnapshot, ChecksumLog } from './net/Checksum.js';
export { RollbackManager } from './net/Rollback.js';
export type { RollbackConfig, InputFrame } from './net/Rollback.js';

/* ---------------------------- render ----------------------------- */

export { DebugDraw } from './render/DebugDraw.js';
export type { DebugDrawFlags, DebugDrawColors, DebugDrawOptions } from './render/DebugDraw.js';

/* ---------------------------- settings --------------------------- */

export * as Settings from './util/settings.js';
export { PROTOCOL_VERSION } from './util/settings.js';

/** Library version, kept in sync with `package.json`. */
export const VERSION = '1.4.0';
