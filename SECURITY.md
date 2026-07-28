# Security policy

## Supported versions

| Version | Supported |
|---|---|
| 1.4.x | ✅ |
| < 1.4 | ❌ |

## Reporting a vulnerability

Please report security issues privately through GitHub's
[private vulnerability reporting](https://github.com/SurenaMHZ/pulse2d/security/advisories/new)
rather than opening a public issue. You should get an initial response within a
few days.

## Threat model

Pulse2D is a pure-computation library: it has no runtime dependencies, performs
no I/O, opens no network connections, reads no files, and does not use `eval`
or the `Function` constructor. That rules out most of the usual categories.
What remains is worth being explicit about, because the library is designed to
be used in networked games where **snapshot data arrives from another machine**.

### Snapshots are not a trust boundary

`loadSnapshot()` validates its header (magic bytes, protocol version, length)
and rejects malformed input, but it is **not hardened against a hostile peer**.
A crafted snapshot can put the world into a nonsensical but internally
consistent state. It cannot escape the sandbox of the JS engine, execute code,
or read memory it should not — the worst case is a corrupted simulation or
wasted CPU.

If you accept snapshots from untrusted clients, apply the standard multiplayer
rule: **the server simulates, clients send inputs, not state.** Pulse2D's
rollback design already assumes this.

### Denial of service

Simulation cost scales with body and contact count. A peer that is allowed to
spawn unbounded bodies can exhaust CPU. Cap the number of bodies your game
lets any player create; the engine deliberately does not impose a policy.

### What counts as a vulnerability here

- Memory-unsafe behaviour, infinite loops or unbounded allocation reachable
  from valid public-API input
- `loadSnapshot()` crashing the process (rather than throwing) on malformed
  bytes
- Any path that executes attacker-supplied strings as code

### What does not

- A physically implausible result from physically implausible input
- Performance degradation from a scene you deliberately made huge
- Determinism divergence — that is a correctness bug; please file it as a
  normal issue, and it will be treated as high priority.
