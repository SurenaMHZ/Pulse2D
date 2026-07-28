/**
 * @module dynamics/Continuous
 *
 * **Continuous collision detection** (CCD) for fast-moving bodies.
 *
 * The ordinary pipeline is discrete: a body teleports from its old position to
 * its new one each step, and speculative contacts catch anything that moved
 * less than a few slops. A bullet travelling 150 m/s at 60 Hz covers 2.5 m per
 * step, so it can jump clean over a 10 cm wall — no contact is ever generated.
 *
 * CCD closes that hole. For each body flagged {@link Body#bullet}, the swept
 * motion is tested against nearby geometry with a conservative-advancement
 * shape cast; if an impact is found at fraction `t`, the body is placed at the
 * point of impact and the normal solver handles the response on the next step.
 *
 * ### Cost
 *
 * Only bullets are swept, and only against proxies their swept AABB actually
 * touches, so leaving the flag off (the default) costs nothing at all. Turn it
 * on for projectiles and small fast debris — not for everything.
 *
 * ### Determinism
 *
 * The candidate list is gathered from the broad phase and then **sorted by
 * fixture id**, so the earliest impact is chosen in a fixed order even when
 * two hits share the same fraction.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';
import { Vec2 } from './../math/Vec2.js';
import { Transform } from './../math/Transform.js';
import { AABB } from './../collision/AABB.js';
import { makeProxy, shapeCast, makeShapeCastOutput } from './../collision/Distance.js';
import type { DistanceProxy } from './../collision/Distance.js';
import { BodyType } from './Body.js';
import type { Body } from './Body.js';
import type { Fixture } from './Fixture.js';
import type { World } from './World.js';
import { shouldCollide } from './Filter.js';
import { LINEAR_SLOP } from './../util/settings.js';

const _sweptAABB = new AABB();
const _fixtureAABB = new AABB();
const _translation = Vec2.zero();
const _startXf = new Transform();
const _otherXf = new Transform();
const _castOut = makeShapeCastOutput();
const _candidates: Fixture[] = [];

/** Proxy cache so a shape's vertex list is built once, not once per step. */
const _proxyCache = new WeakMap<object, DistanceProxy>();

function proxyFor(fixture: Fixture): DistanceProxy {
  let p = _proxyCache.get(fixture.shape as object);
  if (p === undefined) {
    p = makeProxy(fixture.shape);
    _proxyCache.set(fixture.shape as object, p);
  }
  return p;
}

/**
 * Sweep every bullet body and stop it at its first impact.
 *
 * Called by {@link World#step} after positions have been integrated but before
 * the broad phase is re-synchronised, so a body moved back to its impact point
 * gets the correct AABB.
 *
 * @param world     the world being stepped
 * @param moved     the bodies the solver just moved
 * @returns the number of bodies that were pulled back
 */
export function solveContinuous(world: World, moved: Body[]): number {
  let adjusted = 0;

  for (const body of moved) {
    if (!body.bullet || body.type !== BodyType.Dynamic) continue;

    // How far did this body travel this step?
    Vec2.subTo(_translation, body.worldCenter, body.sweepCenter0);
    const distanceSq = _translation.lengthSq();
    if (distanceSq < S.EPSILON_SQ) continue;

    // Skip the sweep unless the motion is long enough to risk a miss. A body
    // that moved less than its own smallest extent cannot tunnel.
    const extent = smallestExtent(body);
    if (distanceSq < S.mul(extent, extent)) continue;

    // Rebuild the transform the body had at the start of the step.
    _startXf.q.copyFrom(body.sweepRot0);
    Vec2.subTo(_startXf.p, body.sweepCenter0, rotateLocalCenter(body));

    let earliest = S.ONE;
    let hit = false;

    for (const fixture of body.fixtures) {
      if (fixture.isSensor) continue;
      const proxyB = proxyFor(fixture);

      // Swept AABB covering the whole motion.
      fixture.shape.computeAABB(_sweptAABB, _startXf);
      fixture.shape.computeAABB(_fixtureAABB, body.transform);
      AABB.combineTo(_sweptAABB, _sweptAABB, _fixtureAABB);

      // Gather candidates, then sort for a deterministic evaluation order.
      _candidates.length = 0;
      world.broadPhase.query(_sweptAABB, (otherId) => {
        const other = world.fixtures[otherId];
        if (other && other.body !== body) _candidates.push(other);
        return true;
      });
      _candidates.sort((a, b) => a.id - b.id);

      for (const other of _candidates) {
        if (other.isSensor) continue;
        const otherBody = other.body;
        // Only sweep against things that cannot get out of the way.
        if (otherBody.type === BodyType.Dynamic && !otherBody.bullet) {
          if (otherBody.awake) continue;
        }
        if (!shouldCollide(fixture.filter, other.filter)) continue;

        _otherXf.copyFrom(otherBody.transform);
        const result = shapeCast(_castOut, {
          proxyA: proxyFor(other),
          proxyB,
          xfA: _otherXf,
          xfB: _startXf,
          translationB: _translation,
          maxFraction: earliest,
        });
        if (result && _castOut.fraction < earliest) {
          earliest = _castOut.fraction;
          hit = true;
        }
      }
    }

    if (!hit || earliest >= S.ONE) continue;

    /*
     * Back the body up to just before the impact. Leaving a sliver of gap
     * means the next step's narrow phase sees a normal (speculative) contact
     * and resolves it with the usual solver, instead of starting deeply
     * overlapped.
     */
    const backoff = S.max(S.ZERO, earliest - S.fromFloat(0.02));
    Vec2.addScaledTo(body.worldCenter, body.sweepCenter0, _translation, backoff);
    Vec2.subTo(body.transform.p, body.worldCenter, rotateLocalCenter(body));
    world.synchronizeFixtures(body);
    adjusted++;
  }

  return adjusted;
}

const _rotated = Vec2.zero();

/** `R · localCenter` for the body's current rotation. */
function rotateLocalCenter(body: Body): Vec2 {
  const q = body.transform.q;
  _rotated.set(
    S.mulAdd(q.c, body.localCenter.x, -S.mul(q.s, body.localCenter.y)),
    S.mulAdd(q.s, body.localCenter.x, S.mul(q.c, body.localCenter.y)),
  );
  return _rotated;
}

/**
 * The smallest half-extent across all of a body's fixtures — the distance it
 * can safely move in one step without any chance of tunnelling.
 */
function smallestExtent(body: Body): Scalar {
  let min = S.MAX_VALUE;
  for (const fixture of body.fixtures) {
    fixture.shape.computeAABB(_fixtureAABB, body.transform);
    const hx = S.half(_fixtureAABB.upper.x - _fixtureAABB.lower.x);
    const hy = S.half(_fixtureAABB.upper.y - _fixtureAABB.lower.y);
    min = S.min(min, S.min(hx, hy));
  }
  return min === S.MAX_VALUE ? LINEAR_SLOP : S.max(min, LINEAR_SLOP);
}
