/**
 * @module dynamics/Fixture
 *
 * A {@link Shape} bound to a {@link Body}, with material properties.
 *
 * Shapes are pure geometry and may be shared; a fixture is the per-instance
 * binding that carries density, friction, restitution and collision filtering,
 * and owns the broad-phase proxy.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { AABB } from './../collision/AABB.js';
import type { Shape } from './../collision/Shape.js';
import type { Body } from './Body.js';
import { makeFilter, DEFAULT_FILTER } from './Filter.js';
import type { Filter } from './Filter.js';

/** Options accepted by {@link Body#addFixture}. */
export interface FixtureDef {
  /** The geometry. Required. */
  shape: Shape;
  /** Mass per unit area, kg/m². Default `1`. */
  density?: number;
  /**
   * Coulomb friction coefficient, `0` (ice) to `1`+ (rubber). Default `0.6`.
   * The pair value is the geometric mean of the two fixtures'.
   */
  friction?: number;
  /**
   * Bounciness, `0` (clay) to `1` (perfectly elastic). Default `0`.
   * The pair value is the maximum of the two fixtures'.
   */
  restitution?: number;
  /**
   * A sensor detects overlap and reports events but generates no impulses —
   * trigger volumes, pickup zones, water. Default `false`.
   */
  isSensor?: boolean;
  /** Collision filtering. */
  filter?: Partial<Filter>;
  /** Arbitrary payload; never touched by the engine. */
  userData?: unknown;
  /**
   * Surface conveyor speed along the contact tangent, m/s. Non-zero turns the
   * fixture into a moving walkway or a treadmill. Default `0`.
   */
  tangentSpeed?: number;
}

export class Fixture {
  /** Dense index into {@link World}'s fixture table; also the proxy payload. */
  readonly id: number;
  /** Owning body. */
  readonly body: Body;
  /** Geometry (may be shared with other fixtures). */
  readonly shape: Shape;

  density: Scalar;
  friction: Scalar;
  restitution: Scalar;
  tangentSpeed: Scalar;
  isSensor: boolean;
  filter: Filter;
  userData: unknown;

  /** Broad-phase proxy id, or `-1` when not in the tree. */
  proxyId = -1;
  /** Cached world AABB, refreshed whenever the body moves. */
  readonly aabb = new AABB();

  /** @internal — constructed by {@link Body#addFixture}. */
  constructor(id: number, body: Body, def: FixtureDef) {
    this.id = id;
    this.body = body;
    this.shape = def.shape;
    this.density = S.fromFloat(def.density ?? 1);
    this.friction = S.fromFloat(def.friction ?? 0.6);
    this.restitution = S.fromFloat(def.restitution ?? 0);
    this.tangentSpeed = S.fromFloat(def.tangentSpeed ?? 0);
    this.isSensor = def.isSensor ?? false;
    this.filter = def.filter ? makeFilter(def.filter) : makeFilter(DEFAULT_FILTER);
    this.userData = def.userData;
  }

  /** Recompute {@link aabb} from the body's current transform. */
  synchronize(): AABB {
    return this.shape.computeAABB(this.aabb, this.body.transform);
  }

  /**
   * Change the filter and force the broad phase to re-evaluate the fixture's
   * pairs, so contacts that are no longer allowed disappear next step.
   */
  setFilter(filter: Partial<Filter>): void {
    this.filter = makeFilter({ ...this.filter, ...filter });
    this.body.world.refilterFixture(this);
  }

  /** Change the density; call {@link Body#resetMassData} afterwards. */
  setDensity(density: number): void {
    this.density = S.fromFloat(density);
  }

  /** `true` when the world point lies inside this fixture. */
  testPoint(px: number, py: number): boolean {
    const p = this.body.world.scratchPoint.set(S.fromFloat(px), S.fromFloat(py));
    return this.shape.testPoint(this.body.transform, p);
  }
}
