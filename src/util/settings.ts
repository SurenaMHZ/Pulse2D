/**
 * @module util/settings
 *
 * Global tuning constants.
 *
 * These are **compile-time** values on purpose: every peer in a lockstep
 * session must agree on them, and a value that can be changed at runtime is a
 * value that will eventually differ between two clients. If you need to tune
 * them, change them here, rebuild, and bump {@link PROTOCOL_VERSION} — the
 * snapshot header carries it so a mismatched build is rejected at connect
 * time instead of desyncing ten minutes in.
 *
 * All lengths are in **metres**. Pulse2D is tuned for objects roughly
 * `0.1 m – 10 m` across; if your game works in pixels, divide by ~30–100
 * before handing coordinates to the engine.
 */

import * as S from './../math/scalar.js';
import type { Scalar } from './../math/scalar.js';

/**
 * Bumped whenever a change alters simulation results. Peers with different
 * protocol versions cannot interoperate.
 */
export const PROTOCOL_VERSION = 2;

/* ----------------------------- lengths ----------------------------- */

/**
 * Allowed overlap between two shapes. Solving to *exactly* zero would make
 * contacts flicker on and off; a small tolerated overlap keeps them stable.
 */
export const LINEAR_SLOP: Scalar = S.fromFloat(0.005);

/** Angular equivalent of {@link LINEAR_SLOP} (≈2°). */
export const ANGULAR_SLOP: Scalar = S.fromFloat((2 / 180) * Math.PI);

/**
 * How far apart two shapes may be and still get a (speculative) contact.
 * Four slops gives the solver a frame of warning before an impact.
 */
export const SPECULATIVE_DISTANCE: Scalar = S.mulInt(LINEAR_SLOP, 4);

/**
 * Extra padding around each broad-phase AABB. A body may move this far before
 * its proxy has to be re-inserted into the tree, which is what keeps the
 * broad phase cheap for slowly moving objects.
 */
export const AABB_MARGIN: Scalar = S.fromFloat(0.1);

/**
 * Predictive AABB extension, as a multiple of the per-step displacement.
 * Catches fast movers before they cross a thin wall.
 */
export const AABB_VELOCITY_SCALE: Scalar = S.fromFloat(4);

/** Maximum positional correction applied in a single step. */
export const MAX_LINEAR_CORRECTION: Scalar = S.fromFloat(0.2);

/**
 * Cap on the speed at which overlap is pushed out, m/s.
 *
 * Push-out injects *real* velocity that the relax pass can only partly remove,
 * so this cap is what bounds the energy a bad initial overlap can add. It
 * matters most when many contacts stack on one body: eight circles spawned
 * on top of each other produce 28 simultaneous contacts, and at the old cap of
 * 4 m/s their combined push flung the pile apart at ~10 m/s, never to settle.
 *
 * The trade-off is recovery speed, measured against a body embedded in a wall:
 *
 * | cap    | 0.3 m deep | 0.6 m deep | 8-body pile-up |
 * |--------|-----------:|-----------:|---------------:|
 * | 4 m/s  |    0.23 s  |    0.38 s  |     9.8 m/s    |
 * | 2 m/s  |    0.23 s  |    0.38 s  |     4.9 m/s    |
 * | 1 m/s  |    0.33 s  |   never    |     2.4 m/s    |
 *
 * 2 m/s halves the ejection energy while still clearing any penetration a game
 * realistically produces. Normal stacks are unaffected — their bias never
 * approaches the cap.
 *
 * Precomputed here because it is read in the solver's innermost loop.
 */
export const MAX_BIAS_VELOCITY: Scalar = S.fromFloat(-2);

/** Maximum angular correction applied in a single step (≈8°). */
export const MAX_ANGULAR_CORRECTION: Scalar = S.fromFloat((8 / 180) * Math.PI);

/* --------------------------- velocities ---------------------------- */

/**
 * Speed cap per step, expressed in *body lengths*: a body may never travel
 * more than 4 m in one step. Prevents one bad impulse from launching an object
 * into the next county and blowing up the tree.
 */
export const MAX_TRANSLATION: Scalar = S.fromFloat(4);

/** Rotation cap per step (0.5 rev). */
export const MAX_ROTATION: Scalar = S.fromFloat(0.5 * Math.PI);

/**
 * Impacts slower than this get no restitution. Without the threshold a
 * resting ball would bounce forever on numerical noise.
 */
export const RESTITUTION_THRESHOLD: Scalar = S.fromFloat(1);

/**
 * Maximum restitution sweeps per step.
 *
 * A shock travelling through a row of touching bodies — a Newton's cradle, a
 * line of pool balls — moves one contact per sweep, so a single pass leaves
 * the wave half-propagated and every body drifts off together. The loop stops
 * early once a sweep changes nothing, so an isolated bounce still costs one
 * pass; the cap only bounds the worst case.
 */
export const RESTITUTION_ITERATIONS = 8;

/** Below this, a restitution correction counts as converged. */
export const RESTITUTION_TOLERANCE: Scalar = S.fromFloat(1e-5);

/* ----------------------------- sleeping ---------------------------- */

/** A body may sleep once it moves slower than this. */
export const SLEEP_LINEAR_TOLERANCE: Scalar = S.fromFloat(0.01);

/** …and rotates slower than this (≈2°/s). */
export const SLEEP_ANGULAR_TOLERANCE: Scalar = S.fromFloat((2 / 180) * Math.PI);

/** Seconds a body must stay below the tolerances before it sleeps. */
export const TIME_TO_SLEEP: Scalar = S.fromFloat(0.5);

/* ---------------------------- iteration ---------------------------- */

/**
 * Default biased velocity iterations **per sub-step**.
 *
 * With soft-step TGS the sub-step count is the main convergence knob, so one
 * iteration per sub-step is the right default. Raise it only for scenes that
 * need unusual rigidity; raising `subSteps` is normally the better trade.
 */
export const DEFAULT_VELOCITY_ITERATIONS = 2;

/** Default relax iterations per sub-step (removes the bias overshoot). */
export const DEFAULT_RELAX_ITERATIONS = 1;

/** Contact push-out stiffness, in Hz. Higher = firmer, less squishy stacks. */
export const CONTACT_HERTZ: Scalar = S.fromFloat(30);

/** Contact damping ratio. 10 is heavily overdamped, which suppresses bounce. */
export const CONTACT_DAMPING_RATIO: Scalar = S.fromFloat(10);

/** Joint stiffness default, in Hz. */
export const JOINT_HERTZ: Scalar = S.fromFloat(60);

/** Joint damping ratio default. */
export const JOINT_DAMPING_RATIO: Scalar = S.fromFloat(2);

/* ---------------------------- capacities --------------------------- */

/** Initial capacity of the body pool; it grows geometrically. */
export const INITIAL_BODY_CAPACITY = 256;

/** Initial capacity of the contact pool. */
export const INITIAL_CONTACT_CAPACITY = 512;

/** Highest number of shapes the broad phase will report for one query. */
export const MAX_QUERY_RESULTS = 4096;
