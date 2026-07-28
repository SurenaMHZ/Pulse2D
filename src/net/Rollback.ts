/**
 * @module net/Rollback
 *
 * A ready-made **rollback / GGPO-style** driver on top of the snapshot layer.
 *
 * The model is the standard one:
 *
 * * the simulation always runs at the local tick, predicting remote inputs;
 * * when a real remote input arrives for a past tick and it differs from what
 *   was predicted, the world is restored to the snapshot **before** that tick
 *   and re-simulated forward with the corrected inputs;
 * * because Pulse2D is deterministic, the re-simulation lands on exactly the
 *   state every other peer computed.
 *
 * ```ts
 * const rb = new RollbackManager(world, {
 *   maxRollbackFrames: 12,
 *   applyInputs: (tick, inputs) => applyToGame(tick, inputs),
 *   predictInput: (tick, playerId, last) => last, // repeat-last-input
 * });
 *
 * // every frame
 * rb.addLocalInput(myPlayerId, myInput);
 * rb.advance();                       // steps the world once
 *
 * // whenever the network delivers something
 * rb.addRemoteInput(tick, playerId, input);
 * ```
 */

import type { World } from './../dynamics/World.js';
import { saveSnapshot, loadSnapshot, cloneSnapshot } from './Snapshot.js';
import type { Snapshot } from './Snapshot.js';
import { checksumWorld, ChecksumLog } from './Checksum.js';

/** One player's input for one tick. Opaque to the engine. */
export type InputFrame = unknown;

/** Configuration for {@link RollbackManager}. */
export interface RollbackConfig<I = InputFrame> {
  /** How many ticks of history to keep. Default `16`. */
  maxRollbackFrames?: number;
  /**
   * Apply the inputs for a tick to the world. Called once per tick during
   * both normal advancement and re-simulation, so it **must be pure** with
   * respect to anything outside the world.
   */
  applyInputs: (tick: number, inputs: Map<number, I>) => void;
  /**
   * Predict a missing remote input.
   * The default (repeat the last known input) is right for most games.
   */
  predictInput?: (tick: number, playerId: number, last: I | undefined) => I | undefined;
  /** Compare two inputs; defaults to `Object.is`. */
  inputsEqual?: (a: I | undefined, b: I | undefined) => boolean;
  /** Record a checksum every tick. Costs a little; invaluable while debugging. */
  enableChecksums?: boolean;
  /** Called when a rollback happens — useful for telemetry. */
  onRollback?: (fromTick: number, toTick: number, frames: number) => void;
}

interface Frame<I> {
  tick: number;
  snapshot: Snapshot;
  /** Inputs used when this tick was simulated. */
  inputs: Map<number, I>;
  /** Which player inputs were guesses. */
  predicted: Set<number>;
}

export class RollbackManager<I = InputFrame> {
  private readonly world: World;
  private readonly cfg: Required<
    Pick<RollbackConfig<I>, 'maxRollbackFrames' | 'applyInputs' | 'enableChecksums'>
  > &
    RollbackConfig<I>;

  /** Ring buffer of confirmed frames, oldest first. */
  private history: Frame<I>[] = [];
  /** Authoritative inputs received from the network, by tick. */
  private confirmed = new Map<number, Map<number, I>>();
  /** Last known real input per player, used for prediction. */
  private lastKnown = new Map<number, I>();
  /**
   * Every player the manager has ever seen.
   *
   * Tracked separately from {@link lastKnown} so a player who is *known to
   * exist* but has not sent anything yet still gets a predicted (and therefore
   * correctable) input slot. Without this, tick 0 would silently apply no
   * input at all and the mistake could never be rolled back, because nothing
   * was ever marked as predicted.
   */
  private players = new Set<number>();
  /** Local inputs queued for the next tick. */
  private pendingLocal = new Map<number, I>();

  /** Per-tick checksums, when enabled. */
  readonly checksums = new ChecksumLog(512);

  /** Number of rollbacks performed, for diagnostics. */
  rollbackCount = 0;
  /** Total ticks re-simulated, for diagnostics. */
  resimulatedTicks = 0;

  constructor(world: World, config: RollbackConfig<I>) {
    this.world = world;
    this.cfg = {
      ...config,
      maxRollbackFrames: config.maxRollbackFrames ?? 16,
      applyInputs: config.applyInputs,
      enableChecksums: config.enableChecksums ?? false,
    };
  }

  /** The tick the world is currently at. */
  get tick(): number {
    return this.world.tick;
  }

  /**
   * Register a player before the session starts.
   *
   * Optional but recommended: it lets the manager predict (and later correct)
   * a player's very first input instead of treating "no input yet" as fact.
   */
  addPlayer(playerId: number): void {
    this.players.add(playerId);
  }

  /** Queue a local input for the next {@link advance}. */
  addLocalInput(playerId: number, input: I): void {
    this.players.add(playerId);
    this.pendingLocal.set(playerId, input);
  }

  /**
   * Deliver an authoritative input.
   *
   * If it contradicts what was predicted for an already-simulated tick, a
   * rollback is triggered immediately.
   */
  addRemoteInput(tick: number, playerId: number, input: I): void {
    this.players.add(playerId);
    let frame = this.confirmed.get(tick);
    if (!frame) {
      frame = new Map();
      this.confirmed.set(tick, frame);
    }
    frame.set(playerId, input);
    this.lastKnown.set(playerId, input);

    // Did we already simulate this tick with a different guess?
    const simulated = this.history.find((f) => f.tick === tick);
    if (!simulated) return;

    const wasPredicted = simulated.predicted.has(playerId);
    const hadNoInput = !simulated.inputs.has(playerId);
    // Neither predicted nor missing => it was already authoritative.
    if (!wasPredicted && !hadNoInput) return;

    const equal = this.cfg.inputsEqual ?? Object.is;
    if (!hadNoInput && equal(simulated.inputs.get(playerId), input)) {
      simulated.predicted.delete(playerId); // the guess was right
      return;
    }
    simulated.inputs.set(playerId, input);
    simulated.predicted.delete(playerId);
    this.rollbackTo(tick);
  }

  /**
   * Advance the simulation by one tick, predicting any missing inputs.
   * @returns the tick that was just simulated
   */
  advance(): number {
    const tick = this.world.tick;

    // Snapshot *before* stepping, so we can return to this exact state.
    const snapshot = saveSnapshot(this.world);
    const inputs = new Map<number, I>();
    const predicted = new Set<number>();

    // Local inputs are always authoritative.
    for (const [pid, input] of this.pendingLocal) {
      inputs.set(pid, input);
      this.lastKnown.set(pid, input);
      let frame = this.confirmed.get(tick);
      if (!frame) {
        frame = new Map();
        this.confirmed.set(tick, frame);
      }
      frame.set(pid, input);
    }
    this.pendingLocal.clear();

    // Fill in the rest from the network, or predict.
    const confirmedFrame = this.confirmed.get(tick);
    for (const pid of this.players) {
      if (inputs.has(pid)) continue;
      const real = confirmedFrame?.get(pid);
      if (real !== undefined) {
        inputs.set(pid, real);
      } else {
        const last = this.lastKnown.get(pid);
        const guess = this.cfg.predictInput ? this.cfg.predictInput(tick, pid, last) : last;
        if (guess !== undefined) {
          inputs.set(pid, guess);
        }
        // Mark it predicted even when the guess is `undefined`: the tick was
        // simulated without authoritative input and may need correcting.
        predicted.add(pid);
      }
    }

    this.history.push({ tick, snapshot: cloneSnapshot(snapshot), inputs, predicted });
    while (this.history.length > this.cfg.maxRollbackFrames) this.history.shift();

    this.cfg.applyInputs(tick, inputs);
    this.world.step();

    if (this.cfg.enableChecksums) this.checksums.record(tick, checksumWorld(this.world));

    // Forget inputs that can no longer be rolled back to.
    const oldest = this.history[0]?.tick ?? tick;
    for (const t of this.confirmed.keys()) if (t < oldest - 1) this.confirmed.delete(t);

    return tick;
  }

  /**
   * Rewind to just before `tick` and re-simulate up to the present.
   *
   * @returns `false` when the tick is older than the retained history — at
   *          that point the peer is unrecoverably behind and needs a full
   *          state sync from the host.
   */
  rollbackTo(tick: number): boolean {
    const index = this.history.findIndex((f) => f.tick === tick);
    if (index < 0) return false;

    const target = this.history[index]!;
    const currentTick = this.world.tick;
    const frames = currentTick - tick;

    loadSnapshot(this.world, target.snapshot);
    this.world.tick = tick;

    // Drop the invalidated tail; it is about to be recomputed.
    const replay = this.history.splice(index);

    for (const frame of replay) {
      const t = frame.tick;
      const snapshot = saveSnapshot(this.world);
      const inputs = new Map<number, I>();
      const predicted = new Set<number>();

      const confirmedFrame = this.confirmed.get(t);
      for (const [pid, previous] of frame.inputs) {
        const real = confirmedFrame?.get(pid);
        if (real !== undefined) {
          inputs.set(pid, real);
        } else {
          inputs.set(pid, previous);
          if (frame.predicted.has(pid)) predicted.add(pid);
        }
      }

      this.history.push({ tick: t, snapshot: cloneSnapshot(snapshot), inputs, predicted });
      this.cfg.applyInputs(t, inputs);
      this.world.step();
      if (this.cfg.enableChecksums) this.checksums.record(t, checksumWorld(this.world));
      this.resimulatedTicks++;
    }

    this.rollbackCount++;
    this.cfg.onRollback?.(currentTick, tick, frames);
    return true;
  }

  /** The oldest tick still recoverable. */
  get oldestTick(): number {
    return this.history[0]?.tick ?? this.world.tick;
  }

  /** Number of retained frames. */
  get historyLength(): number {
    return this.history.length;
  }

  /** Approximate memory held by the history, in bytes. */
  get historyBytes(): number {
    let total = 0;
    for (const f of this.history) total += f.snapshot.data.byteLength + f.snapshot.meta.byteLength;
    return total;
  }

  /** Drop all history and inputs (after a hard resync). */
  reset(): void {
    this.history.length = 0;
    this.confirmed.clear();
    this.lastKnown.clear();
    this.players.clear();
    this.pendingLocal.clear();
    this.checksums.clear();
    this.rollbackCount = 0;
    this.resimulatedTicks = 0;
  }
}
