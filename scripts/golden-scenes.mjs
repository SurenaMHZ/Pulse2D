/**
 * Scenes used by the cross-platform golden-checksum contract.
 *
 * They live in their own module, backend-agnostic (every scene receives the
 * API object to build against), so that `scripts/golden.mjs` and
 * `test/golden.test.mjs` can never drift apart: the recorder and the verifier
 * run literally the same code.
 *
 * **Do not edit an existing scene.** Its recorded checksum is a contract with
 * every peer that ever shipped against it. If you need different coverage,
 * append a new scene and record it; leave the old ones alone.
 */

/** @typedef {{ name: string, steps: number, build: (api: any) => any }} GoldenScene */

/**
 * Run a scene and digest its whole **trajectory**, not just its final state.
 *
 * Digesting only the end state would be weak: most scenes settle and fall
 * asleep, and a transient divergence early on can be damped away by friction
 * before the last tick. Folding the per-tick checksum into a rolling hash
 * means a single wrong bit at any tick changes the recorded value.
 *
 * The mixing constants are the standard 32-bit FNV-1a prime and a xorshift
 * finalizer; all arithmetic goes through `Math.imul` and `>>>` so the digest
 * itself is exactly as portable as the engine it is checking.
 *
 * @param {any} api      the backend module (`pulse2d` or `pulse2d/fixed`)
 * @param {GoldenScene} scene
 * @returns {string} 8 lowercase hex digits
 */
export function digestScene(api, scene) {
  const world = scene.build(api);
  let h = 0x811c9dc5;
  for (let i = 0; i < scene.steps; i++) {
    world.step();
    h = Math.imul(h ^ (api.checksumWorld(world) >>> 0), 0x01000193) >>> 0;
  }
  h ^= h >>> 16;
  h = Math.imul(h, 0x7feb352d) >>> 0;
  h ^= h >>> 15;
  return (h >>> 0).toString(16).padStart(8, '0');
}

/** @type {GoldenScene[]} */
export const scenes = [
  {
    name: 'stack-and-tumble',
    steps: 900,
    build(api) {
      const { World, BodyType, Circle, Capsule, Polygon, Rng } = api;
      const world = new World({ gravity: { x: 0, y: -10 }, seed: 20260728 });

      const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      ground.addFixture({ shape: Polygon.box(30, 1), friction: 0.6 });
      for (const x of [-10, 10]) {
        const wall = world.createBody({ type: BodyType.Static, position: { x, y: 6 } });
        wall.addFixture({ shape: Polygon.box(0.5, 7), friction: 0.4 });
      }

      const rng = new Rng(20260728);
      for (let i = 0; i < 60; i++) {
        const b = world.createBody({
          type: BodyType.Dynamic,
          position: { x: rng.float() * 16 - 8, y: 1 + rng.float() * 12 },
          angle: rng.float() * 6 - 3,
          angularVelocity: rng.float() * 4 - 2,
          linearVelocity: { x: rng.float() * 5 - 2.5, y: rng.float() * 2 },
        });
        const kind = rng.int(0, 2);
        if (kind === 0) b.addFixture({ shape: Circle.of(0.22 + rng.float() * 0.22), density: 1, restitution: 0.35, friction: 0.4 });
        else if (kind === 1) b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.5 });
        else b.addFixture({ shape: Capsule.vertical(0.7, 0.18), density: 1, friction: 0.4 });
      }
      return world;
    },
  },

  {
    name: 'every-joint',
    steps: 900,
    build(api) {
      const { World, BodyType, Circle, Polygon } = api;
      const world = new World({ gravity: { x: 0, y: -10 }, seed: 7 });

      const anchor = world.createBody({ type: BodyType.Static, position: { x: 0, y: 12 } });

      const mk = (x, y) => {
        const b = world.createBody({ type: BodyType.Dynamic, position: { x, y } });
        b.addFixture({ shape: Polygon.box(0.4, 0.2), density: 1, friction: 0.3 });
        return b;
      };

      const d = mk(-6, 9);
      world.createDistanceJoint({ bodyA: anchor, bodyB: d, length: 3 });

      const r = mk(-3, 9);
      world.createRevoluteJointAt(anchor, r, -3, 12);

      const w = mk(0, 9);
      world.createWeldJoint({
        bodyA: anchor, bodyB: w,
        localAnchorA: { x: 0, y: -3 }, linearHertz: 6, angularHertz: 6,
      });

      const p = mk(3, 9);
      world.createPrismaticJoint({
        bodyA: anchor, bodyB: p,
        localAxisA: { x: 1, y: 0 }, enableLimit: true, lowerLimit: -2, upperLimit: 2,
      });

      const m = mk(6, 9);
      world.createMotorJoint({ bodyA: anchor, bodyB: m, maxForce: 40, maxTorque: 12 });

      // A hanging chain so the joint solver has a genuine multi-body island.
      let prev = anchor;
      for (let i = 0; i < 10; i++) {
        const link = world.createBody({ type: BodyType.Dynamic, position: { x: -1 + i * 0.5, y: 11.5 } });
        link.addFixture({ shape: Circle.of(0.16), density: 1, friction: 0.2 });
        world.createRevoluteJointAt(prev, link, -1.25 + i * 0.5, 11.5);
        prev = link;
      }

      const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: 0 } });
      ground.addFixture({ shape: Polygon.box(30, 1), friction: 0.6 });
      return world;
    },
  },

  {
    name: 'fast-bodies-ccd',
    steps: 600,
    build(api) {
      const { World, BodyType, Circle, Polygon, Rng } = api;
      const world = new World({ gravity: { x: 0, y: -10 }, seed: 99 });

      // A thin box the bullets would tunnel through without continuous collision.
      const wall = world.createBody({ type: BodyType.Static, position: { x: 8, y: 5 } });
      wall.addFixture({ shape: Polygon.box(0.05, 5), friction: 0.2, restitution: 0.4 });
      const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      ground.addFixture({ shape: Polygon.box(40, 1), friction: 0.6 });

      const rng = new Rng(99);
      for (let i = 0; i < 24; i++) {
        const b = world.createBody({
          type: BodyType.Dynamic,
          position: { x: -8, y: 1 + i * 0.4 },
          linearVelocity: { x: 120 + rng.float() * 60, y: rng.float() * 4 - 2 },
          bullet: true,
        });
        b.addFixture({ shape: Circle.of(0.08), density: 4, restitution: 0.5, friction: 0.1 });
      }
      return world;
    },
  },

  {
    name: 'sleep-and-wake',
    steps: 1500,
    build(api) {
      const { World, BodyType, Circle, Polygon } = api;
      const world = new World({ gravity: { x: 0, y: -10 }, seed: 5 });
      const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
      ground.addFixture({ shape: Polygon.box(30, 1), friction: 0.7 });

      // A pyramid that settles and sleeps…
      let n = 8;
      for (let row = 0; row < n; row++) {
        for (let col = 0; col <= row; col++) {
          const b = world.createBody({
            type: BodyType.Dynamic,
            position: { x: col * 0.62 - row * 0.31, y: 0.3 + (n - row) * 0.62 },
          });
          b.addFixture({ shape: Polygon.box(0.3, 0.3), density: 1, friction: 0.6 });
        }
      }
      // …then a heavy ball that arrives late and wakes it up again.
      const ball = world.createBody({
        type: BodyType.Dynamic,
        position: { x: -14, y: 7 },
        linearVelocity: { x: 14, y: 0 },
      });
      ball.addFixture({ shape: Circle.of(0.7), density: 6, restitution: 0.2, friction: 0.4 });
      return world;
    },
  },
];
