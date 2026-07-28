/**
 * @module dynamics/Contact
 *
 * A persistent pair of potentially-touching fixtures.
 *
 * The broad phase creates a contact as soon as two fat AABBs overlap; the
 * narrow phase then runs every step and fills in the {@link Manifold}. Keeping
 * the object alive across steps is what makes **warm starting** possible: the
 * accumulated impulses from the previous step are matched by contact id and
 * reused as the initial guess, which is the single biggest reason a stack of
 * boxes settles in ~8 iterations instead of hundreds.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Manifold } from './../collision/Manifold.js';
import { collide } from './../collision/Collide.js';
import type { Fixture } from './Fixture.js';
import { shouldCollide } from './Filter.js';

/** Bit flags describing a contact's current state. */
export const enum ContactFlags {
  None = 0,
  /** The two fat AABBs overlap. */
  Touching = 1 << 0,
  /** Was touching at the end of the previous step. */
  WasTouching = 1 << 1,
  /** At least one fixture is a sensor: report overlap, apply no impulse. */
  Sensor = 1 << 2,
  /** Marked for removal at the end of the step. */
  Destroy = 1 << 3,
  /** Both bodies are non-static; the contact participates in islands. */
  Simulated = 1 << 4,
  /** Explicitly disabled from a `preSolve` callback for this step only. */
  Disabled = 1 << 5,
}

const _prevImpulses: Scalar[] = [S.ZERO, S.ZERO, S.ZERO, S.ZERO];
const _prevIds: number[] = [0, 0];

export class Contact {
  /** Dense index in the world's contact table. */
  readonly id: number;
  readonly fixtureA: Fixture;
  readonly fixtureB: Fixture;

  /** Current manifold; `pointCount === 0` when not touching. */
  readonly manifold = new Manifold();

  flags: ContactFlags = ContactFlags.None;

  /** Combined friction, cached at creation. */
  friction: Scalar;
  /** Combined restitution, cached at creation. */
  restitution: Scalar;
  /** Combined surface (conveyor) speed. */
  tangentSpeed: Scalar;

  /** @internal Index into the solver's constraint array for this step. */
  constraintIndex = -1;

  /** @internal Set while the contact is queued in the island builder. */
  islandVisited = false;

  constructor(id: number, fixtureA: Fixture, fixtureB: Fixture) {
    this.id = id;
    this.fixtureA = fixtureA;
    this.fixtureB = fixtureB;
    this.friction = Contact.mixFriction(fixtureA.friction, fixtureB.friction);
    this.restitution = Contact.mixRestitution(fixtureA.restitution, fixtureB.restitution);
    this.tangentSpeed = fixtureA.tangentSpeed - fixtureB.tangentSpeed;
    if (fixtureA.isSensor || fixtureB.isSensor) this.flags |= ContactFlags.Sensor;
  }

  /**
   * Geometric mean — `sqrt(fA · fB)`.
   *
   * Chosen over the arithmetic mean because it lets a single very slippery
   * surface (ice, `f = 0`) dominate the pair, which matches intuition.
   */
  static mixFriction(a: Scalar, b: Scalar): Scalar {
    return S.sqrt(S.mul(a, b));
  }

  /** Maximum — a bouncy ball stays bouncy against a dead floor. */
  static mixRestitution(a: Scalar, b: Scalar): Scalar {
    return a > b ? a : b;
  }

  /** `true` when the shapes are actually overlapping (not just their AABBs). */
  get isTouching(): boolean {
    return (this.flags & ContactFlags.Touching) !== 0;
  }

  /** `true` when either fixture is a sensor. */
  get isSensor(): boolean {
    return (this.flags & ContactFlags.Sensor) !== 0;
  }

  /** Turn this contact off for the remainder of the current step. */
  setEnabled(enabled: boolean): void {
    if (enabled) this.flags &= ~ContactFlags.Disabled;
    else this.flags |= ContactFlags.Disabled;
  }

  /** `true` when the contact will be solved this step. */
  get isEnabled(): boolean {
    return (this.flags & ContactFlags.Disabled) === 0;
  }

  /**
   * Re-run the narrow phase and transfer accumulated impulses onto the new
   * manifold points.
   *
   * @returns `true` when the touching state changed (so the world can fire
   *          `begin`/`end` events)
   */
  update(): boolean {
    const wasTouching = (this.flags & ContactFlags.Touching) !== 0;
    if (wasTouching) this.flags |= ContactFlags.WasTouching;
    else this.flags &= ~ContactFlags.WasTouching;
    this.flags &= ~ContactFlags.Disabled;

    const fA = this.fixtureA;
    const fB = this.fixtureB;

    // Filtering can change at runtime.
    if (!shouldCollide(fA.filter, fB.filter)) {
      this.manifold.pointCount = 0;
      this.flags &= ~ContactFlags.Touching;
      return wasTouching;
    }

    if (this.isSensor) {
      // Sensors only need a yes/no answer, so reuse the manifold test but
      // never keep the points.
      collide(this.manifold, fA.shape, fA.body.transform, fB.shape, fB.body.transform);
      const touching = this.manifold.pointCount > 0;
      this.manifold.pointCount = 0;
      if (touching) this.flags |= ContactFlags.Touching;
      else this.flags &= ~ContactFlags.Touching;
      return touching !== wasTouching;
    }

    // Save the old impulses so they can be matched onto the new points.
    const old = this.manifold;
    const oldCount = old.pointCount;
    for (let i = 0; i < oldCount; i++) {
      _prevImpulses[i * 2] = old.points[i]!.normalImpulse;
      _prevImpulses[i * 2 + 1] = old.points[i]!.tangentImpulse;
      _prevIds[i] = old.points[i]!.id;
    }

    collide(this.manifold, fA.shape, fA.body.transform, fB.shape, fB.body.transform);

    // Warm start: carry impulses across on matching feature ids.
    const count = this.manifold.pointCount;
    for (let i = 0; i < count; i++) {
      const mp = this.manifold.points[i]!;
      mp.normalImpulse = S.ZERO;
      mp.tangentImpulse = S.ZERO;
      mp.maxNormalImpulse = S.ZERO;
      mp.persisted = false;
      for (let j = 0; j < oldCount; j++) {
        if (_prevIds[j] === mp.id) {
          mp.normalImpulse = _prevImpulses[j * 2]!;
          mp.tangentImpulse = _prevImpulses[j * 2 + 1]!;
          mp.persisted = true;
          break;
        }
      }
    }

    const touching = count > 0;
    if (touching) this.flags |= ContactFlags.Touching;
    else this.flags &= ~ContactFlags.Touching;
    return touching !== wasTouching;
  }

  /** Total normal impulse applied last step, N·s. Great for hit detection. */
  getTotalNormalImpulse(): Scalar {
    let sum = S.ZERO;
    for (let i = 0; i < this.manifold.pointCount; i++) {
      sum += this.manifold.points[i]!.normalImpulse;
    }
    return sum;
  }

  /** Reset for reuse from the pool. */
  reset(): void {
    this.manifold.clear();
    this.flags = ContactFlags.None;
    this.constraintIndex = -1;
    this.islandVisited = false;
  }
}
