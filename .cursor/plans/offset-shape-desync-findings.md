# Off-center shape desync — findings and next steps

Status as of 2026-08-11 (second native session). Context for continuing the investigation.

## TL;DR
Two determinism bugs have been found and fixed. The first (Layer 1) was an ill-conditioned
mass frame plus a last-bit centre of mass on near-spherical compounds; that fix ships.

**Layer 3's conclusion — "PhysX's un-snapshottable solver warm-start residue" — is WRONG
and has been disproved.** Layer 4 measured it directly: under the framework's cold-step
discipline, `restore + step` is a **pure function of the restored state**, even after the
scene has been driven through deliberately hostile histories. No difference in how two
peers reached a confirmed tick can move that tick. The whole residue family of
explanations is dead, and with it the plan to snapshot/restore persistent manifolds.

What is left, and what Layer 4 reproduces exactly, is **body CONSTRUCTION**: the shapes,
their local poses and offsets, the materials, the mass, the depenetration clamp, the
solver iteration counts. Every solve reads them, no snapshot carries them, and no hash the
session exchanged covered them. A **one-ULP difference in one spike's local pose leaves
the mass hash and the state hash bit-identical and desyncs the ball 102 steps after two
pushers start squeezing it** — while leaving it in perfect agreement for as long as it is
only rolling on the floor. That is the demo's reported signature precisely.

**The fix, now shipping: `PxwWorldHashConstruction` / `PxwWorldHashConstructionPerEntry`**,
exposed as `DeterministicWorld.HashConstruction` / `ReadConstructionHashes` /
`CompareConstruction` and `SimConstructionCheck`. Next step for the demo is to exchange it
between the two peers and read off which body was built differently.

---

## Layer 1 — the mass-frame bug (understood and fixed)

### Mechanism
A "spiked ball" is a core sphere plus ~24 offset sphere shapes (Fibonacci layout).
Letting PhysX compute the compound mass (`updateMassAndInertia`) produces:
- A **near-isotropic** inertia tensor — principal moments differ by only ~**1.30%**.
  PhysX stores the diagonalizing eigenvectors as the mass-frame orientation, and for a
  near-sphere those eigenvectors are numerically meaningless (noise). A last-bit change
  in the inputs swings the frame arbitrarily.
- A **centre of mass** that is a sum of per-shape contributions, landing a last bit off
  origin.

Both feed straight into the solver. The instant the ball shares a solver island
(touching the floor, or squeezed between two players), peers that computed even slightly
different frames/COMs diverge.

### Evidence (from `tests/PxwOffsetShapeRepro.cpp`)
Two-peer model = same body built two ways, differing only in the floating-point
association of the mass sum (a proxy for a heterogeneous peer):

| Config | peers agree on mass frame? | cross-peer sim |
|---|---|---|
| plain sphere | AGREE (all 0) | exact |
| computed, translate offset | DIFFER: dCOM 4.4e-9, dQuat 1.3e-5 | desync at step 0 |
| computed, rotation-only offset | DIFFER: dCOM 0, dQuat 0.603 | desync at step 0 |
| collapsed frame only (no COM snap) | DIFFER: dCOM 4.4e-9, dQuat 0 | desync at step 36 |
| collapsed frame + COM snap | AGREE (all 0) | exact |
| authored isotropic | AGREE (all 0) | exact |

Key secondary findings:
- **`paired scenes agree` PASSED for every config** — within a single build/process PhysX
  is fully deterministic here. Divergence appears only across the FP-perturbed "peer."
  This is decisive for the demo (see Layer 2).
- **Pose round trip through `setGlobalPose` is EXACT** in all cases, so the earlier
  "lossy pose round trip" hypothesis was ruled out. Collapsing the frame to identity keeps
  it exact regardless.
- Spike-count bisection: 1 spike is well-conditioned (agrees); 6 desync at step 38; 24
  desync at step 0. Fragility rises with how spherical the inertia becomes.
- Collapsing the frame **alone was not enough** — the residual last-bit COM still desynced
  (step 36). That is why the COM snap was added.

### The fix (implemented, built, deployed)
Extended the existing isotropy-collapse path in `PxwComputeMassProperties`:
1. Raised default `PXW_DEFAULT_ISOTROPY_TOLERANCE` from `0.01f` to `0.05f` (the 1.3% ball
   never entered the collapse path at 1%).
2. When collapsing: identity frame + mean moments (as before) **plus snap a near-origin
   COM to the actor origin** when `|COM| <= 0.001 * sqrt(meanI / mass)` (0.1% of the radius
   of gyration).

Files changed:
- Native: `src/PxwUndpwr.cpp` (collapse branch ~1597), `include/PxwUndpwr.h`
  (tolerance + docs, `massFrameCollapsed` doc).
- Managed mirror: `physx5-for-unity/Runtime/UNDPWR/Core/SimConfig.cs`
  (`MassIsotropyTolerance = 0.05f`), `.../Core/SimMass.cs`, `.../Interop/NativeTypes.cs`
  (docs only).
- Tests: `tests/PxwUndpwrTests.cpp` — `TestIsotropyCollapseCanonicalisesCentreOfMass`
  (COM snapped to exactly origin; hash identical across sum order) and
  `TestOffCentreMassIsPreserved` (a genuine 0.31 m COM is preserved, not snapped).
  149/149 pass.
- Repro: `tests/PxwOffsetShapeRepro.cpp` — `eCollapsedFrame` now models the full framework
  path (canonical sort + collapse + COM snap) and is a must-agree invariant. 16/16 pass.
- Demo: `UNDPWR2_sample/.../SampleActors.cs` `CreateSpikedSphere` now just calls
  `SimMass.Setup(actor, density)` (removed the hand-authored closed-form mass).
- DLL: `PhysXUnity` rebuilt (Release) and deployed with the 5 PhysX runtime DLLs into
  `physx5-for-unity/Plugins/Windows/x86_64` (they had been missing — only `.meta` present).

---

## Layer 2 — why the demo likely STILL desyncs

Decisive reason the mass fix may not be the demo's actual failure:

> In-process/same-build determinism already held for the computed ball (`paired scenes
> agree` passed for every config). If the demo runs **identical binaries** — Unity
> Multiplayer Play Mode (MPPM) virtual players, or same-arch clients — there is **no
> floating-point difference between peers**, so the mass frame is bit-identical on every
> peer even without the fix. In that setup the mass frame could not have been the cause.

So either the demo is genuinely heterogeneous (different CPU arch/build — then Layer 1 was
real and necessary), or the "both players touch the ball" desync is a separate,
order/replay-dependent mechanism. Ranked hypotheses:

1. **Rollback replay transparency in a shared solver island (leading hypothesis).**
   - `SimConfig.Solver` defaults to **TemporalGaussSeidel**; the framework docs say TGS
     "carries per-substep state that a restore does not reach, so replay is never
     transparent." Only **PGS** was measured replay-transparent under the cold-step
     discipline.
   - When both players touch the ball, ball + 2 capsules form **one island** with
     warm-started contacts/manifolds. A late input triggers a rollback; the replay steps
     "cold" while the confirmed timeline stepped "warm," and they diverge.
     `tests/PxwRollbackRepro.cpp` already established this warm/cold asymmetry (rollback was
     non-transparent for essentially everything with contacts).
   - This would desync even a **plain sphere** once it is contested during a rollback,
     which fits "only when both players touch it."

2. **Contact / island composition ordering.** A 25-shape compound plus two dynamic capsules
   is a large, dense island. If actor creation/registration order or pooled-slot assignment
   for players differs across peers, PhysX's internal island/constraint ordering can differ.
   Check whether stable-ID ordering fully covers pooled player entities.

3. **Input / force application asymmetry.** `SoccerPlayerEntity.OnSimUpdate` applies
   `AddForce` + `SetAngularVelocity` every tick. If any runs outside the step handler, or
   input for two simultaneous players is applied in a peer-dependent order, that desyncs.
   The plugin header explicitly warns forces must be applied inside the step handler for
   replay safety.

4. **Capsule mass (low probability).** Capsules are elongated so their axes are
   well-conditioned (not collapsed), but confirm their anisotropy and that the upright
   quarter-turn local pose does not create a near-degenerate case.

Note the fix did help one rollback aspect: the ball's collapsed identity frame + origin COM
makes its `setGlobalPose` restore bitwise lossless, so pose restoration of the ball is no
longer a divergence source.

---

## Layer 3 — the live-session investigation (2026-08-11)

Ran the demo as two MPPM peers (identical binaries) and instrumented it end-to-end. The
result overturns several Layer-2 guesses and pins the mechanism precisely.

### What was ruled OUT (all with matching cross-peer hashes)
- **Mass — exonerated by direct comparison.** Added a per-client mass-hash log to
  `CreateSpikedSphere`. Both peers print byte-identical mass:
  `mass hash 0x5A65E290… | mass 47.9673538 | inertia (7.04755831 ×3) | com (0,0,0) |
  frame (0,0,0,1) | anisotropy 1.2992% | collapsed True | shapes 25`. Same binaries ⇒ no
  FP delta ⇒ Layer 1 was necessary for cross-arch but is NOT the demo's cause.
- **FP control word — pinned and confirmed identical.** Added MXCSR read/pin/restore
  around `PxwWorldSimulate`/`PxwWorldFetchResults` (canonicalise on entry, restore on
  fetch). Both peers log `MXCSR = 0x1FA3 (FTZ=0 DAZ=0 RC=0)`. Not the cause. (Pin left in;
  cheap insurance for cross-arch.)
- **Registration / actor-index order — matches.** `SimRegistrationCheck` reports
  `Registration order matches peer 2 across 26 bodies`. Stable-ID ordered insertion holds
  for pooled players and the ball. Layer-2 hypothesis 2 rejected for the demo.
  - Note: a native Stage D in `PxwOffsetShapeRepro.cpp` DID reproduce a desync by forcing
    *differing* registration order across peers — so order matters in principle, it just
    isn't what the demo does.
- **Rebuild / join state — identical.** Both peers log
  `Rebuild complete; state hash is 0x8ED2E366DECCBAE1` after a mid-match join. The join is
  deterministic.
- **Rollback / prediction — not required to trigger it.** Repro'd with NO new input at the
  moment of contact, i.e. no rollback occurred. It is a pure forward-stepping divergence,
  not a warm/cold replay asymmetry. This weakens the original Layer-2 hypothesis 1 (TGS
  replay transparency) as the *primary* driver — though TGS vs PGS is still worth a
  confirmation pass.
- **Threads — single-threaded.** `CpuWorkerThreads = 0` on both, so no task-scheduling
  nondeterminism.
- **Managed game logic — clean.** Per-entity hashes show only the two *active contested*
  bodies fork (the joining player's capsule id 268435457 and the ball id 268435472);
  player 1's capsule and all 14 dormant pool capsules stay bit-identical across peers.

### The smoking gun — contact digest
Added a confirmed-tick contact fingerprint in `SoccerGameMode` (FNV-1a over sorted
contact id-pair / point / normal / impulse) surfaced via `SoccerMatch`:

```
HOST   tick 866  scene 7 0x1EB39BF3…  ball 5 0x59C6F064…
CLIENT tick 866  scene 7 0xF6EEC5E0…  ball 5 0x8A0EA193…   <-- fork here
HOST   tick 867  Diverged bodies …
CLIENT tick 867  Desync … Diverged: Physics.
```

The **contact hash forks at tick 866, one tick BEFORE the body-pose desync at 867**, and
crucially the **contact COUNT and ORDER are identical** (scene 7 / ball 5 on both). Same
contacts, same order, different solved values. That is the signature of solver warm-start
residue: PhysX carries per-manifold accumulated impulses / friction anchors that a
snapshot cannot read or write, so two peers that reached tick 866 along different
prediction+rollback histories carry different warm-start and the solve on the stiff island
produces slightly different impulses → full desync one tick later.

### Reset-on-restore was tried in the live session and made it WORSE
Hypothesis: wipe PhysX's carried contact state on every cold restore so all peers rebuild
the manifold identically (the "I used to implicitly reset it every step" memory). Added
`SimConfig.ContactResetOnRestore` (`None` / `ResetFiltering` / `Reinsert`), applied in
`RollbackEngine.RestoreTo`, mirroring the native suite's `RestoreAndReset` ordering.

- `ResetFiltering` → **immediate** divergence when player 2 connects.
- `Reinsert` (remove + re-add every actor in stable-ID order) → **broke the mid-match
  join**: bit-identical before, forks within a few ticks of the rebuild. Host then re-flags
  the mismatch every tick (88, 90, 91, 92, 93…) while the client forks once at 89 and falls
  behind.

**Why it fails live but passes the native suite:** the native suite drives both peers at
the *same* rollback depth, so they reset the same number of times. A live session does
not — two independently timed processes rewind by different depths every frame and call
the reset a *different number of times*, and the reset's own side effects on
scene/broadphase/island bookkeeping are **not invariant** across that. So per-restore reset
is fundamentally incompatible with variable-depth rollback and is **rejected**. Reverted to
`None`; added a variable-depth warning to the `ContactResetOnRestore` XML doc.

### Conclusion
The ball works fine touched by one player; it desyncs only when **squeezed between two**,
because that is the only time its 25 manifolds form a stiff, over-constrained island whose
solve is sensitive to the warm-start residue. The residue is real, un-snapshottable PhysX
state. There are exactly two honest ways out:
1. **Remove the amplifier** (fewer/one collision sphere) — rejected by product: the spiky
   ball must work as-is.
2. **Make the warm-start deterministic across peers** — either snapshot+restore it, or
   force a canonical rebuild that IS invariant to rollback depth. **This is the new
   session's job.**

---

## Layer 4 — the native session that overturned Layer 3 (2026-08-11)

Layer 3's conclusion was reached from demo logs. Layer 4 tested it in isolation in
`tests/PxwOffsetShapeRepro.cpp`, which links PhysX only, and it does not survive contact
with a measurement.

### First: the scenario is real (contact census)
Every negative result below is only worth the contact load behind it, so that is now
measured rather than assumed:

| scenario | pairs/step | ball pairs/step | peak | ball touched |
|---|---|---|---|---|
| plain sphere, drop only | 0.91 | 0.91 | 1 | 90% of steps |
| plain sphere, squeezed | 3.93 | 1.24 | 3 | 100% of steps |
| spiked x24, squeezed | 6.56 | 3.73 | 8 | 100% of steps |

The spiked ball carries 3.7 contact pairs per step, peaking at 8 — the same order as the
demo's "ball 5". The stages below were genuinely squeezing a 25-shape compound.

### What was disproved
- **Stage E — asymmetric prediction lead. AGREES.** Models the real `RollbackEngine` loop:
  one cold confirmed restore-and-step per frame, then a prediction window of `lead` further
  cold steps that is thrown away, with the host at lead 0 and the client at lead 6, plus a
  jittered variant. Only confirmed states are compared. No divergence in 200 frames.
- **Stage F — a pooled player joins the contested ball. AGREES.** 14 parked slots, one
  unparked into the squeeze part way through, with the peers on different leads. Internal
  actor and island-node ids agree throughout; no divergence. Unparking is deterministic.
- **Stage G — the general question: is `restore + step` pure? YES, in every case.**
  One reference result is computed from a clean world, then the same world is put through
  a hostile history and asked to restore the identical snapshot and step:

  | history before the restore | result |
  |---|---|
  | nothing (control) | pure |
  | 5 free steps | pure |
  | teleported 1 km away, 4 steps (every broadphase pair dropped) | pure |
  | teleport, step, restore, step, teleport again | pure |
  | slept and woken | pure |
  | filtering reset | pure |
  | deep free run (40 steps) | pure |

  Pure for the plain sphere, the spiked collapsed-mass ball, and the pooled world. The
  cold-step discipline's `setGlobalPose` on every body every step already invalidates the
  contact cache, so there is no residue left to carry. **This is the result that kills
  Layer 3.**
- Stage C (variable rollback depth) also agrees, as it did before — consistent with the above.

### Stage H — what actually reproduces it
Two peers built from identical source apart from ONE construction property, stepped
forward in lockstep. No rollback, no prediction — the plainest possible setting:

| perturbation | drop only | squeezed |
|---|---|---|
| none (control) | agrees | agrees |
| **one spike local pose, 1 ULP** | **agrees** | **desyncs at step 102** |
| max depenetration velocity unclamped | agrees | desyncs at step 0 |
| material friction, 1 ULP | agrees | agrees |

The middle two rows have the demo's exact shape: invisible while the ball is lightly
loaded, fatal once it is squeezed. The depenetration clamp is the sharpest case because it
does literally nothing until bodies are deeply overlapped.

Note the trap this creates: the Layer 1 mass canonicalisation (identity frame, mean
moments, origin COM) is *designed* to discard small shape differences, so **"both peers
print the same mass hash" no longer proves their shapes match.** Layer 3 leaned on exactly
that evidence.

### Fidelity gaps in the repro, now closed
The repro was not applying what `DeterministicWorld.Register` applies to every dynamic
body (`PxwApplyDeterministicRigidDefaults`): 8/2 solver iterations, speculative CCD off,
max depenetration velocity 3.0. It now does, so it drives PhysX the way the demo does.

### The fix
`PxwWorldHashConstruction(world)` and `PxwWorldHashConstructionPerEntry(world, dst, cap)`
in `src/PxwUndpwr.cpp` / `include/PxwUndpwr.h`. They hash, per registered entry:

- shape count and **attachment order** (PhysX generates contacts in that order)
- per shape: geometry type and dimensions, local pose, contact and rest offsets, shape
  flags, simulation and query filter data, and every material's static/dynamic friction,
  restitution, combine modes and flags
- per body: mass, inertia, centre-of-mass pose, rigid-body and actor flags, linear and
  angular damping, max linear/angular velocity, max depenetration velocity, max contact
  impulse, solver iteration counts, sleep/stabilization/contact-report thresholds
- articulations: every link, plus the articulation's own iteration counts

No addresses are hashed — meshes are identified by vertex and element counts — so the
value is comparable across processes and machines. It does not change as the simulation
runs, so peers on different ticks can compare it directly.

Managed surface: `DeterministicWorld.HashConstruction()`,
`DeterministicWorld.ReadConstructionHashes(out count)`,
`DeterministicWorld.CompareConstruction(peer, peerCount, out problem)`, and
`UNDPWR.Net.SimConstructionCheck.Compare` / `.Describe` for the wire path, mirroring
`SimRegistrationCheck`.

Native tests in `tests/PxwUndpwrTests.cpp` (160 checks, 0 failures):
- `TestConstructionHashAgreesForIdenticalBuilds` — identical builds hash equal, and the
  hash is unchanged by stepping.
- `TestConstructionHashCatchesWhatMassAndStateHashesMiss` — the headline: one spike moved
  by 1 ULP prints `mass hash AGREES, state hash AGREES, construction hash differs`.
- `TestConstructionHashCatchesSolverProperties` — depenetration clamp, material friction,
  iteration counts, and shape attachment order are all caught.
- `TestConstructionHashPerEntryNamesTheBody` — exactly one entry differs, and it is the
  right stable id.

All four native suites pass via `ctest -C Release`; the DLL is rebuilt and deployed.

---

## Layer 5 - managed session repro and root cause (2026-08-11)

The Unity-side batch harness in
`UNDPWR2_sample/Assets/UNDPWR2Sample/basic_soccer/Editor/SoccerDeterminismHarness.cs`
finally reproduced the live failure without MPPM or manual input. It runs the actual
`SoccerMatch`, join rebuild, `SimSession`, delayed/reordered loopback transport, rollback
engine, player logic, and 25-shape ball.

### What it ruled out

Two independent Unity batch processes exported the complete 26-entry construction table.
The files were byte-identical (SHA-256
`8651AA1F49CEBA481A5112CADD01183E86C8914BD0EC8CBC526321462DE4A7C6`) and both reported
construction hash `0xE8A98D3A69DC4636`. The ball, spike poses, mass, materials, solver
properties, and every pooled capsule were built identically. Stage H described a valid
failure mode, but it was not the demo's failure.

### Automated failure

With 5 steps of latency and packet reordering, the unfixed harness failed at confirmed
tick 268:

- physics: `FF3E5A851757925A != 525C1CBE62568921`
- entity: `83EABCDF5BF5D1B5 == 83EABCDF5BF5D1B5`
- game: `88201FB960FF6465 == 88201FB960FF6465`

This is the exact live signature: only native physics forks, after the two players reach
the ball.

### Root cause

`SoccerMatch.ProduceRebuild` did not leave the host on the same rebuild path as receivers:

1. The host captured a one-player snapshot, changed its roster, applied that old snapshot,
   and ran `ReconcileEntities`, which enabled the joining capsule.
2. It captured the resulting final two-player payload and sent it.
3. A receiver recreated its world and restored that already-final payload, so reconcile was
   idempotent and it did not perform the host's disabled-to-enabled transition.

The snapshot bytes and all public internal-id diagnostics agreed, but PhysX scene/island
bookkeeping created by that transition is not snapshot state. It remained harmless until
the newly enabled capsule joined the ball's heavily constrained contact island. Stage F
missed this because it unparked the pooled actor on both peers; the real join path unparked
it only on the producer.

This also explains why `RecreateNativeWorld` did not already solve the join: it canonicalized
the scene before the host produced the final state, but the host then mutated that fresh
scene to create the payload. Receivers began from the final payload instead.

### Fix and validation

The invariant now lives in the framework. `RollbackEngine.TryProduceRebuildState` owns the
whole producer protocol: capture the old confirmed state, apply the target roster and
reconcile it, capture the finalized broadcast state, then consume that finalized payload
on the producer before returning it. Every peer therefore recreates its native world and
restores the exact same final bytes through the exact same lifecycle.

`SoccerMatch.ProduceRebuild` only sets its gameplay roster, calls the framework method,
then performs ordinary sample bookkeeping (player rebinding, session notification and
presentation refresh). It no longer knows about or implements the native-history rule.

Results:

- exact 600-frame repro: PASS, common confirmed tick 748, peak ball contacts 7 on both
  peers, 635 replayed ticks on both, final hash `0xE0BDC5C24045D45D`
- the same 600-frame repro after moving the fix into `RollbackEngine`: PASS with the same
  tick, contact counts, replay counts, and final hash
- 2,400-frame stress pass: PASS, common confirmed tick 2548, peak ball contacts 7 on both
  peers, 2,435 replayed ticks on both, final hash `0x34BFC62497BB9E0C`
- native `PxwOffsetShapeRepro`: 21 checks, 0 failures

## Superseded concrete next steps (demo session)

The native layer is now exonerated for residue and instrumented for construction. The
remaining work is to find WHICH construction differs in the soccer demo.

1. **Exchange the construction hash between the two peers.** Call
   `DeterministicWorld.HashConstruction()` on each peer after the world is built and again
   after the mid-match rebuild, log both, and compare. If they differ, this is the bug.
2. **Name the body.** Send `ReadConstructionHashes` over the wire and run
   `SimConstructionCheck.Compare`; it reports the stable id that was built differently.
   Expect it to be the ball (id 268435472) or the joining player's capsule (268435457) —
   the two bodies Layer 3 saw fork.
3. **Then find the field.** Once the body is named, diff its construction directly: shape
   count, each spike's local pose, the material, the mass, the depenetration clamp. The
   most likely candidates, given the demo builds spikes from `Mathf.Cos/Sin` in
   `SampleActors.SpikeDirections`, are the per-spike local poses.
4. **If the construction hashes AGREE**, then the physics inputs match and the divergence
   has to be in what gameplay applies during the step — the joining player's *input* for
   one tick. Hash the per-tick input frame alongside the state hash and compare. Layer 3's
   "only the ball and the joining player's capsule fork" fits an input mismatch just as
   well as a construction one, and nothing has yet ruled it out.
5. **Discount the Layer 3 contact digest.** PhysX contact reports reflect the most recent
   `simulate()` call, and each frame ends with the prediction window, not the confirmed
   step — so the demo's "confirmed-tick contact digest" was very likely reporting
   *prediction* contacts, which legitimately differ between peers. That was probably a red
   herring, and it is the evidence Layer 3's conclusion rested on.

## Superseded next steps (do NOT pursue)

Superseded by Layer 4: the premise below — that PhysX carries un-snapshottable warm-start
residue across a restore — was measured and is false (Stage G). Snapshotting persistent
manifolds would be a large amount of native surface area for a problem that does not
exist. Kept only so the reasoning is not repeated.

Goal: keep the 25-shape compound ball exactly as authored and make the two-capsule squeeze
deterministic across peers. The lever is PhysX's persistent-manifold warm-start.

1. **Build the isolating repro FIRST.** Extend `PxwOffsetShapeRepro.cpp` with a
   "two capsules squeeze the spiked ball" scene stepped as **variable-depth rollback**
   across two identical-binary peers (peer A rewinds N, peer B rewinds M, N≠M, both land on
   the same snapshot+inputs). This must reproduce the tick-866-style contact-hash fork in
   isolation. Everything below is validated against this, not the demo.
   - Confirm the fork survives with mass authored isotropic, FP pinned, PGS solver,
     single-threaded, identical registration order — i.e. reproduce it with ALL of Layer 3's
     exonerated factors held constant, proving warm-start is the sole remaining variable.
2. **Confirm warm-start is the variable.** After a restore, before stepping, log/compare the
   per-manifold cached impulses (see `PxsContactManagerOutput` / `PxContactPatch` /
   friction anchors) between the two peers. Expect them to differ while pose/velocity match.
3. **Then pick a fix and prove it on the repro:**
   - **(a) Snapshot + restore the warm-start.** Expose native calls to read every persistent
     manifold's cached normal/friction impulses and anchor points into the snapshot blob,
     and write them back on restore, keyed by a stable contact-pair id (stable-ID pair, not
     PhysX pointer). Highest fidelity; most native surface area. Watch for: pairs that exist
     on one side of a rollback but not the other, and PhysX regenerating manifolds between
     save and restore.
   - **(b) Canonical rollback-depth-invariant rebuild.** Find a manifold rebuild that is a
     pure function of (snapshot, inputs) regardless of how many times it ran — unlike
     `Reinsert`/`ResetFiltering`, whose side effects accumulate with call count (that is
     exactly why they broke live play, Layer 3). If one exists, it is far cheaper than (a).
   - **(c) Zero the warm-start every step symmetrically** (cold contacts always). Only viable
     if it is genuinely call-count invariant; verify against the variable-depth repro, since
     the demo already disproved the naive per-restore reset.
4. **TGS vs PGS confirmation pass.** Demo ran default TGS. Even though no rollback was needed
   to trigger the fork, re-run the repro under **PGS** — if PGS makes the squeeze transparent
   under variable-depth rollback, that is the cheapest possible fix and may moot (a).
5. **Only after the repro is green**, wire the chosen mechanism into the framework
   snapshot/restore path and re-test the demo squeeze across two MPPM peers.

### Demo cleanup after resolution
The sample-only investigation scaffolding was removed once the managed regression passed:
the confirmed-tick contact digest, verbose/native logging toggle, per-entity hash capture,
and spiked-ball mass dump. The editor harness remains as the automated regression, and now
detects the contested-ball path from body positions rather than runtime tracing.

Removing the toggle exposed a callback-lifetime bug in `SimLog`: Unity can reload the managed
domain while the native DLL remains loaded, leaving `PxwSetLogCallback` pointing at the old
managed delegate. The next native info message (material creation in this sample) then failed
at `SampleActors.EnsureInitialised`. `SimLog` now detaches before editor assembly reloads,
unconditionally clears the native slot even when its managed field is null, and clears it again
at subsystem registration. `SimNetTests.DetachNativeSinkClearsPointerAfterManagedStateWasReset`
locks down that stale-domain case.

---

## Handy references
- Repro harness: `tests/PxwOffsetShapeRepro.cpp` — run `build\Release\PxwOffsetShapeRepro.exe`.
  Stages: A mass frame, B forward squeeze, C variable rollback depth, D registration order,
  E asymmetric prediction lead, F pooled spawn, G restore purity, H construction
  differences, plus a contact census that proves the squeeze is real.
- Construction hash: `PxwWorldHashConstruction` in `src/PxwUndpwr.cpp`; managed wrappers in
  `Runtime/UNDPWR/Core/DeterministicWorld.cs` and `Runtime/UNDPWR/Net/SimConstructionCheck.cs`.
- Prior rollback investigation: `tests/PxwRollbackRepro.cpp` — documents warm/cold and TGS
  non-transparency; the natural place to mine for Layer 2.
- Mass path: `PxwComputeMassProperties` / `PxwApplyMassProperties` /
  `PxwSetupDeterministicMass` in `src/PxwUndpwr.cpp`; managed wrapper
  `physx5-for-unity/Runtime/UNDPWR/Core/SimMass.cs`.
- Demo entities: `UNDPWR2_sample/Assets/UNDPWR2Sample/basic_soccer/Scripts/SoccerBallEntity.cs`,
  `.../SoccerPlayerEntity.cs`; actor factory `.../basic_player_sample/Scripts/SampleActors.cs`.
- Build/deploy: configure `physx5-native-plugin` with
  `-DBUILD_TESTS=ON -DBUILD_PHYSX_FIRST=OFF -DDEPLOY_TO_UNITY=ON`, build target `PhysXUnity`
  (auto-copies DLLs into the package). Rebuild after any native change so Unity picks it up.
  `MassIsotropyTolerance` is hashed into `SimConfig`, so all peers must run the updated
  package.

## Key numbers to remember
- 24-spike ball anisotropy: ~1.2993% (below the new 5% collapse tolerance, above the old 1%).
- computed translate peer delta: dCOM 4.36e-9, dQuat 1.27e-5, dInertia 1.65e-6.
- computed rotation-only peer delta: dQuat 0.603 (frame is entirely undetermined).
- COM snap threshold: 0.001 * radius of gyration; off-centre test COM 0.31 m preserved.
- Squeezed spiked ball carries 3.73 contact pairs/step, peak 8, on 100% of steps.
- One spike local pose moved 1 ULP: agrees on drop, desyncs at step 102 when squeezed.
- Unclamped max depenetration velocity: agrees on drop, desyncs at step 0 when squeezed.
- `restore + step` is pure under all 7 hostile histories, for all 3 ball configurations.
