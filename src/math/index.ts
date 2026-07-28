/**
 * @module math
 *
 * Barrel for the deterministic math layer.
 *
 * ```ts
 * import { Vec2, Rot, Transform, Rng, sinCos } from 'pulse2d';
 * ```
 */

export * as Scalar from './scalar.js';
export type { Scalar as ScalarValue } from './scalar.js';
export { Vec2 } from './Vec2.js';
export { Rot } from './Rot.js';
export { Transform } from './Transform.js';
export { Mat22, Mat33 } from './Mat22.js';
export { Rng } from './rng.js';
export { sin, cos, tan, sinCos, atan, atan2, asin, acos, normalizeAngle } from './trig.js';
