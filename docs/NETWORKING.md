# Networking

[نسخهٔ فارسی](fa/NETWORKING.md)

How to build deterministic multiplayer on Pulse2D: lockstep, rollback, desync
detection and debugging.

Read [DETERMINISM.md](DETERMINISM.md) first — the rules there are what make any
of this work.

---

## 1. The model

Deterministic netcode sends **inputs**, not state. Every peer runs the same
simulation and independently arrives at the same result.

|  | Sends | Bandwidth | Latency feel | Cheating |
|---|---|---|---|---|
| **State sync** | positions of every object | scales with object count | server-authoritative | hard |
| **Lockstep** | inputs only | tiny, constant | waits for the slowest peer | needs care |
| **Rollback** | inputs only | tiny, constant | instant local response | needs care |

Inputs for a 2-player game are a handful of bytes per tick, regardless of
whether the world holds ten objects or ten thousand. That is the whole appeal —
and it only works if the simulation is bit-identical, which is what Pulse2D
guarantees.

---

## 2. Lockstep

The simplest correct model: every peer waits until it has *all* inputs for tick
*N*, then steps.

```ts
import { World, checksumWorld } from 'pulse2d';

const world = new World({ gravity: { x: 0, y: -10 }, seed: MATCH_SEED });
buildLevel(world);                       // identical on every peer

const inputBuffer = new Map();           // tick -> Map<playerId, Input>
const INPUT_DELAY = 3;                   // ticks of scheduling lead

function tryStep() {
  const tick = world.tick;
  const frame = inputBuffer.get(tick);
  if (!frame || frame.size < playerCount) return false;   // still waiting

  for (const [playerId, input] of frame) applyInput(playerId, input);
  world.step();
  inputBuffer.delete(tick);
  return true;
}

// Send local input for a tick a few frames in the future, so it arrives in time.
function sendLocalInput() {
  const targetTick = world.tick + INPUT_DELAY;
  const input = readLocalInput();
  socket.send({ tick: targetTick, playerId: myId, input });
  record(targetTick, myId, input);       // apply to ourselves too
}
```

**Pros:** trivial to reason about, no rollback machinery.
**Cons:** every player feels the worst player's latency. Fine for RTS, turn-
based and co-op; too sluggish for fighting or action games.

The `INPUT_DELAY` buys time for packets to arrive. Three ticks at 60 Hz is
50 ms of scheduling headroom, at the cost of 50 ms of input lag.

---

## 3. Rollback

The world always runs at the local tick, **predicting** remote inputs. When a
real input arrives and contradicts the prediction, the world rewinds and
re-simulates. Local input is applied instantly, so the game feels responsive
regardless of ping.

`RollbackManager` implements the whole loop:

```ts
import { World, RollbackManager } from 'pulse2d';

const world = new World({ gravity: { x: 0, y: -10 }, seed: MATCH_SEED });
const players = buildLevel(world);

const rb = new RollbackManager(world, {
  maxRollbackFrames: 12,                 // ~200 ms of history at 60 Hz

  // Called once per tick, during both normal advance and re-simulation.
  // MUST be pure with respect to anything outside the world.
  applyInputs(tick, inputs) {
    for (const [playerId, input] of inputs) {
      const body = players[playerId];
      if (input.left)  body.applyLinearImpulse(-8, 0);
      if (input.right) body.applyLinearImpulse( 8, 0);
      if (input.jump && onGround(body)) body.applyLinearImpulse(0, 40);
    }
  },

  // Repeat-last-input is the right default for most games.
  predictInput: (tick, playerId, last) => last ?? { left: false, right: false, jump: false },

  onRollback: (from, to, frames) => stats.record(frames),
});

rb.addPlayer(localId);
rb.addPlayer(remoteId);

// --- every frame ---
const input = readLocalInput();
rb.addLocalInput(localId, input);
socket.send({ tick: rb.tick, playerId: localId, input });
rb.advance();                            // snapshots, applies inputs, steps once
render();

// --- whenever the network delivers something ---
socket.on('input', ({ tick, playerId, input }) => {
  rb.addRemoteInput(tick, playerId, input);   // rolls back automatically if mispredicted
});
```

### What `advance()` does

1. Save a snapshot of the pre-step state.
2. Collect inputs: local ones are authoritative; missing remote ones are
   predicted and flagged.
3. Push `{ tick, snapshot, inputs, predicted }` onto the history ring.
4. Call `applyInputs`, then `world.step()`.

### What `addRemoteInput()` does

If the tick was already simulated and the input was predicted (or missing), the
prediction is compared with the truth. On a mismatch it restores the snapshot
from *before* that tick and replays every tick since, using corrected inputs
where available.

> **Register your players.** Call `addPlayer(id)` at match start. Without it, a
> player's very first input cannot be predicted — the tick is simulated with no
> input at all, and since nothing was marked as predicted, the mistake can
> never be rolled back.

### Purity of `applyInputs`

`applyInputs` is called again for every replayed tick, so it must not touch
anything outside the world:

```ts
// ✗ these run again on every rollback
applyInputs(tick, inputs) {
  playSound('jump');            // stuttering audio
  score += 10;                  // score inflates
  spawnParticles();             // visual garbage
}

// ✓ record intent; act on it after the tick is confirmed
applyInputs(tick, inputs) {
  if (input.jump) { body.applyLinearImpulse(0, 40); pendingEvents.push({ tick, kind: 'jump' }); }
}
```

Flush `pendingEvents` only for ticks older than `rb.oldestTick`, which can no
longer be rolled back.

---

## 4. Cost and budget

Measured on Node 20, x64, 500 bodies:

| Operation | Cost |
|---|---|
| `saveSnapshot` | 0.26 ms, 71 KB |
| `loadSnapshot` | 4.33 ms |
| `checksumWorld` | 0.39 ms |
| `world.step()` (500 circles) | 3.25 ms |

A rollback of *n* frames costs roughly `loadSnapshot + n × (step + saveSnapshot)`.
For a 500-body world rolling back 6 frames: `4.3 + 6 × 3.5 ≈ 25 ms` — too much
for one frame at 60 Hz.

Practical guidance:

- **Keep the simulated world small.** Rollback games typically simulate only
  what affects gameplay (2–4 characters, a few projectiles) and leave scenery
  as non-simulated decoration.
- **Budget history.** 71 KB × 60 = 4.2 MB per second of history at 500 bodies.
  `maxRollbackFrames: 12` is ~850 KB. Check with `rb.historyBytes`.
- **Cap rollback depth.** Beyond ~8 frames, prefer an input delay of 1–2 ticks
  to cut misprediction frequency.
- **Snapshot less often.** For large worlds, snapshot every *k* ticks and
  replay up to *k* extra ticks on a rollback — cheaper in memory, more expensive
  in CPU.

---

## 5. Desync detection

Ship checksums from day one. A desync found in development is an afternoon; one
found in production is a week.

```ts
import { ChecksumLog } from 'pulse2d';

const log = new ChecksumLog(512);

// every tick
world.step();
const digest = log.recordWorld(world);

// every ~30 ticks, exchange digests
if (world.tick % 30 === 0) socket.send({ type: 'sum', tick: world.tick, digest });

socket.on('sum', ({ tick, digest }) => {
  const mine = log.get(tick);
  if (mine !== undefined && mine !== digest) {
    console.error(`DESYNC at tick ${tick}: local ${mine.toString(16)} remote ${digest.toString(16)}`);
    captureDebugState(tick);
  }
});
```

`ChecksumLog.findDivergence(remoteMap)` returns the **earliest** differing tick
from a batch. The first one is the only one that matters; everything after it is
downstream noise.

---

## 6. Debugging a desync

1. **Find the first bad tick** — `findDivergence`, not the tick you noticed the
   problem on.
2. **Compare snapshots at `tick - 1`** on both peers.
   - *Snapshots identical* → the divergence came from **inputs**. Check input
     delivery, ordering and quantisation.
   - *Snapshots differ* → the divergence is in **simulation**, and started
     earlier than you think. Walk back further.
3. **Diff per-body.** Dump `id, position, rotation, velocity` for every body and
   find the first that differs. Then ask what touched it.
4. **Split the hash.** `checksumWorld(world, true)` covers positions only. If
   position-only matches but the full hash does not, the drift is in accumulated
   contact impulses rather than in visible state.
5. **Reproduce locally.** Record the full input stream, then replay it twice in
   one process. If that already diverges, the bug is in your game code, not in
   cross-platform floating point — check the rules in
   [DETERMINISM.md §4](DETERMINISM.md#4-rules-for-your-game-code).

### The usual culprits

| Symptom | Cause |
|---|---|
| Diverges immediately on tick 0 | Different world config, different level build order, or an unregistered player |
| Diverges only with 3+ players | Object creation order depends on packet arrival |
| Diverges only after a rollback | `applyInputs` is not pure, or game state lives outside the world |
| Diverges only across browsers | A `Math.sin`/`Math.random`/`Date.now` call in game code |
| Diverges after several minutes | Accumulated error crossing a comparison boundary — usually an un-quantised input |

---

## 7. Input quantisation

Analogue inputs must be quantised **before** they are transmitted, and the same
quantised value applied locally. Otherwise the sender simulates with full
precision and the receiver with whatever survived serialisation.

```ts
const QUANT = 1024;                                // 1/1024 m resolution
const q = (v) => Math.round(v * QUANT);            // for the wire
const dq = (v) => v / QUANT;                       // for the sim

const input = { aimX: q(mouseWorldX), aimY: q(mouseWorldY), buttons: bitmask };

socket.send(input);
applyLocally(input);                               // same integers as the remote peer

function applyLocally({ aimX, aimY }) {
  turret.setTarget(dq(aimX), dq(aimY));
}
```

Bitmask your buttons rather than sending booleans — smaller, and immune to
JSON key-order differences.

---

## 8. Joining mid-match

Snapshots capture *state*, not *structure*, so a late joiner needs both:

```ts
// Host
socket.send({
  type: 'sync',
  tick: world.tick,
  level: serialiseLevel(),               // your own body/fixture description
  snapshot: {
    tick: snap.tick,
    data: Array.from(snap.data),
    meta: Array.from(snap.meta),
  },
  rngState: world.rng.getState(),
});

// Joiner
const world = new World({ gravity: HOST_GRAVITY, seed: MATCH_SEED });
rebuildLevel(world, msg.level);          // must produce identical body ids
loadSnapshot(world, {
  tick: msg.snapshot.tick,
  data: new Float64Array(msg.snapshot.data),
  meta: new Int32Array(msg.snapshot.meta),
});
world.rng.setState(...msg.rngState);
```

The joiner must recreate bodies in the **same order** as the host so ids line
up. Verify immediately with a checksum exchange before allowing input.

For binary transport, send `snap.data.buffer` and `snap.meta.buffer` directly
rather than converting through arrays.

---

## 9. Version safety

`PROTOCOL_VERSION` and the scalar backend id are embedded in every snapshot
header. `loadSnapshot` throws on a mismatch instead of silently misreading:

```
Pulse2D: protocol mismatch (snapshot 1, build 2)
Pulse2D: scalar backend mismatch (fixed-point vs float)
```

Check at connect time too:

```ts
import { PROTOCOL_VERSION, VERSION, Scalar } from 'pulse2d';

socket.send({ type: 'hello', protocol: PROTOCOL_VERSION, version: VERSION, backend: Scalar.BACKEND });
```

Bump `PROTOCOL_VERSION` whenever you change a tuning constant in
`util/settings.ts`, a solver detail, or the Pulse2D version — anything that
alters simulation results.

---

## 10. Checklist

Before shipping:

- [ ] No `Math.random` anywhere in simulation code — use `world.rng`
- [ ] No `Date.now` / `performance.now` driving simulation — use `world.tick`
- [ ] All analogue inputs quantised before transmit, same values applied locally
- [ ] Every player registered with `addPlayer` before the first tick
- [ ] `applyInputs` is pure; audio, particles and scoring are deferred
- [ ] Bodies created and destroyed in deterministic order on all peers
- [ ] World config (gravity, `timeStep`, `subSteps`, iterations) identical
- [ ] `PROTOCOL_VERSION` checked at connect time
- [ ] Checksums exchanged and logged
- [ ] Replaying a recorded input stream twice gives identical checksums
