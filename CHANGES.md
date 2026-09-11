# Native Plugin - Changes

This document describes changes made to the native plugin: first the PhysX 5.6.1 upgrade, then the robot-removal / vehicle-support refactor.

## Arcade tuner support (2026-09-10)

`ResetVehicleState` clears wheel, suspension, sticky-tire and drivetrain integrators plus commands
between simulation steps. It retains the native chassis, its pose/velocities and all parameters.
Call it alongside a teleport for an in-place gameplay reset; network code must execute it in the
replayable before-step handler. Direct and engine drive tests verify reset, actor/construction
preservation and reset after snapshot restoration under both solvers.

`SetVehicleTireFrictionTable` now refreshes every finalized wheel's bound material table pointer,
count and default friction. Previously it changed only the wrapper's storage, leaving the solver's
bound default stale and potentially retaining a pointer invalidated by vector reallocation.
Native telemetry tests verify that live default-friction changes reach tires.

The updated native suite passes 284 checks. These tests do not establish Unity kart handling parity;
the PACK project's `Assets/PACK/KartTest/PORT_VALIDATION.md` tracks those measurements separately.

## Convex core geometry, collision filtering and configurable vehicle wheel shapes

### Convex core geometry

`CreateConvexCoreGeometry` exposes PhysX 5.6's `PxConvexCoreGeometry`: a pre-authored GJK support
core swept by a margin, giving true cylinders, cones and ellipsoids without cooking a hull. It is
a separate export rather than another case in `CreatePxGeometry` because it is parameterised by a
core type and a margin, neither of which fits that function's `(type, params, ref)` shape.
`eCONVEXCORE` in `CreatePxGeometry` now warns and points at the new export instead of falling
through to the unsupported-type default. Cylinder, cone and segment cores run along local +X,
matching the capsule convention, so orientation comes from the shape's local pose.

`HashGeometryConstruction` handles `eCONVEXCORE` carefully. `PxConvexCoreGeometry` holds a fixed
`PxU8[24]` core buffer but memcpys only `sizeof(Core)` bytes into it — eight, for a cylinder —
leaving the rest at whatever the stack or allocator contained. Hashing the buffer wholesale would
fold uninitialised memory into the construction hash and make two peers holding identical
cylinders report a mismatch, so only the active core's bytes participate, alongside the core type
and the margin. `PxwConvexCoreParamCount` in `DataInterop.h` is the single definition of how many
that is, shared by the factory and the hash.

### Collision filtering

Per-shape filter data and PhysX's group collision table were previously unreachable from managed
code, which made the group table both unusable and a silent determinism hole. Added
`SetShapeSimulationFilterData`, `SetShapeQueryFilterData`, `GetShapeSimulationFilterData`,
`SetGroupCollisionFlag`, `GetGroupCollisionFlag` and `ResetGroupCollisionFlags`.

`PxwWorldHashConstruction` now folds the group table in. The table decides whether two shapes
collide at all, so peers that disagree about it simulate differently while every actor, shape and
geometry hashes identically; and because PhysXExtensions keeps it as process-global state, nothing
else in the construction hash can stand in for it. Only the *disabled* pairs contribute, in a
fixed order. That means a world which never touches groups hashes exactly as it did before this
existed, and the contribution describes the table's meaning rather than its storage.
`ResetGroupCollisionFlags` exists because the table outlives any one scene, so a session that
changed it would otherwise leak that state into whatever runs next in the process.

### Configurable vehicle wheel shapes

Wheel shapes were cooked 16-sided convex prisms created with `PxShapeFlags(0)`, so they collided
with nothing and were invisible to scene queries. `PxwVehicleWheelShapeDesc` and
`SetVehicleWheelShapeParams` make the geometry, the shape flags and the filter data per-wheel
choices, with `PxwVehicleWheelGeometryMode` selecting the cooked prism, a true cylinder, or a
caller-supplied geometry.

`PhysXIntegrationState::create` no longer calls `PxVehiclePhysXActorCreate`. It calls
`PxVehiclePhysXActorConfigure` — the same call that helper makes — and then a plugin-owned
`createShapes`, because `PxShape::setGeometry` cannot change a shape's geometry type, so per-wheel
geometry has to be decided when the shape is created. `CookWheelPrism` reproduces the SDK's hull
bit for bit in the default mode, down to the segment count, vertex ordering and cooking flags,
which is what keeps every existing vehicle's construction hash unchanged.

The cylinder mode needs its local +X turned onto the wheel's rotation axis. That rotation is
written into `PhysXIntegrationParams::physxWheelShapeLocalPoses` rather than the shape's own local
pose, because the vehicle overwrites the latter every step from the suspension and spin state and
composes it with the former. `create` therefore takes its params by non-const reference.

Because that rotation is construction rather than runtime output, the construction hash has to see
it: a convex-core cylinder is frame-independent geometry, so two peers whose vehicle frames disagree
build the same wheel geometry, would hash equal on it alone, and still point their wheels different
ways. `PxwWorldHashConstruction` now folds in each vehicle's `physxWheelShapeLocalPoses` in axle
order, but only for wheels whose pose is not identity, keyed by wheel id. This is deliberately
sparse: on the shipped Unity frame the wheel axis already is the cylinder's local +X, so the
axis-alignment rotation is identity and contributes nothing — a default cooked-prism vehicle and a
default-frame cylinder vehicle both leave every pose at identity, so their construction hashes are
bit-identical to before this existed. Only a frame that turns the cylinder axis off the wheel axis
bakes a non-identity pose, and that is exactly the divergence the fold now catches. The runtime
PxShape wheel poses stay excluded, exactly as before. The diagnostic
`PxwWorldHashConstructionPartPerEntry` is now vehicle-aware to match — parts 0/5/7 skip the runtime
wheel poses and a new part 12 isolates the construction poses.

Enabling `simulationShape` is not safe on its own, and the descriptor says so at length: the
suspension raycast and tire model already resolve the wheel against the road, so a simulating
wheel needs a collision group that excludes the drivable surface or the road is resolved twice and
the vehicle rides high while contact friction fights the tire model.

`PxwVehicleChassisDesc::boxLocalPose` is renamed `shapeLocalPose`. It has applied to a
caller-supplied chassis geometry as much as to the fallback box ever since custom chassis geometry
was added; only the name still said box.

### Tests

`TestConstructionHashDescribesConvexCores` covers the core type, each dimension, the margin, and —
the point of the exercise — that garbage in the core bytes a cylinder does not use is ignored.
`TestConstructionHashIncludesCollisionGroupTable` covers a disabled pair, the pair's symmetry, and
that resetting the table restores the hash a world with no filtering had.
`TestVehicleDefaultWheelShapesAreUnchanged` is the regression guard for the createShapes rewrite:
a vehicle left alone hashes identically to one that explicitly asks for the defaults, its wheels
are still non-colliding cooked hulls, and each override does reach the shapes.
`TestVehicleCylinderWheelsAreDeterministic` drives two vehicles on simulating, filtered cylinder
wheels for 200 rollback ticks and requires them to stay bit-identical, since convex core
narrowphase is a different code path from the cooked hull it replaces.
`TestVehicleConstructionHashIncludesWheelShapeLocalPoses` pins the new hashing contract: on the
shipped frame a cylinder vehicle's wheel poses are identity, so part 12 and the aggregate match the
cooked-prism default (the backward-compatibility guarantee); a cylinder vehicle built on a frame
that rotates the axis off the wheel axis carries a non-identity pose, so part 12 and the aggregate
move to separate it; two identical cylinder builds agree; and the runtime wheel poses that move
every step change neither the aggregate nor part 12 as the vehicle drives.

`TestConvexCoreOnMeshIsReproducibleUnderRollback` is the soak that had to pass before wheels were
allowed to depend on convex core geometry: cylinders resting and rolling on an uneven triangle
mesh, since mesh contact is resolved per triangle and a rolling cylinder re-derives its contact
set constantly, and a cylinder's contact patch is a line rather than a point or a facet.

It checks the two things that matter. Two peers running the same rollback pattern agree for the
full 600 ticks. And a run that rewinds every eight ticks and resimulates the window lands on the
same state as a run that never rewound, bit for bit, for cylinders and for boxes alike — asserted
under PGS and characterised under TGS, as elsewhere in the suite. Both hold as measured: TGS
reports bit-exact replay for the cylinder and the box as well, it is simply not asserted there.

Both sides of that second comparison restore before every step, which is not a detail of the test
but the reason the property holds: a step following a restore narrowphases cold, a step following
another step warm-starts from persistent contact manifolds the snapshot cannot carry, and the two
do not agree. Restoring unconditionally makes every step cold and removes the asymmetry. This is
the discipline `RollbackEngine` already follows, so an uninterrupted warm run is not a
configuration that occurs at runtime, and comparing against one measures the contact cache rather
than the geometry. An earlier draft of this test did exactly that and reported cylinders parting
company with a warm run after ~20 ticks; that number described PhysX's warm start, not convex
core geometry, and the comparison has been replaced with the cold-versus-cold one above.

## Restoring a world that holds a parked pool slot no longer crashes

Any session that used an entity pool crashed the process on its first rollback. An entity pool
registers every slot up front and parks the ones nobody has spawned, which is what keeps the
snapshot layout constant while players come and go. Parking goes through `ApplyEnabled`, which
raises `PxActorFlag::eDISABLE_SIMULATION`: that tears down the body's simulation object but leaves
the actor in the scene, so a parked slot still answers `getScene()` with the scene it is in while
having nothing behind it to drive.

`RestoreRigid` did not distinguish the two. It set velocities and cleared the force and torque
accumulators of every dynamic it walked, and its only guard was a `getScene()` check placed *after*
those calls, which a parked slot passes anyway. On a parked slot `clearForce` reached
`Sc::BodySim::raiseVelocityModFlagAndNotify` through the torn-down sim and wrote through an invalid
node index. Nothing reported it first: PhysX states the precondition as
`PX_CHECK_AND_RETURN(!eDISABLE_SIMULATION)` on every velocity, force and sleep setter, and that is
compiled out of a release PhysX build. `setGlobalPose` is the one call that stays legal while parked.

- Added `IsSimulationDisabled` / `IsParked` helpers and guarded on them rather than on scene
  membership. `RestoreRigid` now restores a parked slot's pose and stops there.
- `CaptureRigid` zeroes the fields restore can no longer put back for a parked slot, so the two stay
  symmetric — otherwise a peer that rebuilt a slot from the snapshot would report a different
  confirmed hash than the peer that parked it.
- `PxwBodyTeleport` had the same fault and is fixed the same way: it still places a parked body,
  which is what a pool does on the tick it spawns one, and skips the rest until the slot is enabled.
- `PxwBodyAddForce`, `PxwBodyAddTorque`, `PxwBodySetLinearVelocity` and `PxwBodySetAngularVelocity`
  guarded only on scene membership and shared the same latent crash; they now no-op on a parked body.
- `PxwWorldRestoreState` now applies an entry's parked state *before* restoring the rest of it rather
  than after. How much of an entry can be restored depends on whether it is parked, so the two have
  to be in the snapshot's order. With the enable applied afterwards, rewinding past a despawn left the
  respawned body carrying whatever velocity it happened to hold: the restore had skipped the velocity
  as belonging to a parked slot, and the enable that followed brought the body back anyway.
- `tests/PxwUndpwrTests.cpp` gains `TestRestoreWithParkedPoolSlot`, covering capture/restore of a
  parked slot and a rewind past a despawn. No existing test parked an entry, which is why this
  survived: the suite covered pooling's snapshot-layout contract but never restored a world with a
  parked slot in it. 144 checks, 0 failures.
- No public API or snapshot-layout change.

## Articulation restore no longer drains joint velocity

A driven articulation spun up under rollback — the `basic_articulation` sample's pendulum reached
many times its warm-run speed even with a near-zero drive target. The cause was operation order in
`RestoreArticulation` (`src/PxwUndpwr.cpp`): it applied the joint cache (positions and velocities)
and only *then* called `setRootGlobalPose`. `setRootGlobalPose` recomputes every descendant link's
world pose from the joint positions via `teleportRootLink`, and that recompute rebuilds the link
spatial velocities as though the joints were momentarily at rest — it discards the joint velocities
the cache had just restored. A passive chain therefore froze on every cold restore; a stiffly driven
joint was worse, because the position servo pumped the mismatch between the drained velocity and its
target back in as energy on every cold step.

- `RestoreArticulation` now restores the root pose and root velocities *first* and applies the joint
  cache *last*, so joint-velocity propagation is the final operation and is not clobbered.
- `tests/PxwUndpwrTests.cpp` gains `TestDrivenArticulationColdStepTransparency`: a fixed-base driven
  pendulum (matching the sample's stiffness/damping) run warm and cold-stepped in lockstep, asserting
  the cold timeline tracks the warm one. Before the fix the cold peak joint speed was ~8x the warm
  peak; after, they agree to floating-point noise under both PGS and TGS. A companion diagnostic
  confirms a capture/restore round trip with no step preserves joint velocity, isolating the fault to
  the step that followed a restore rather than to the cache itself.
- No public API or snapshot-layout change; existing articulation determinism and rollback tests are
  unaffected (134 checks, 0 failures).

## Per-link articulation contact reporting

Motion policies trained with a per-body contact observation need to know which links touched
something during a control step. PhysX only exposes this as pair events during `fetchResults`,
and a control loop that decimates physics spans several of those per decision, so the events have
to be accumulated rather than sampled.

- Added `src/ArticulationContacts.{h,cpp}`: `ArticulationContactTracker`, a
  `PxSimulationEventCallback` that ORs per-link touch flags into a buffer keyed by articulation
  and indexed by `PxArticulationLink::getLinkIndex()` (the same low-level index the articulation
  cache uses). `Clear()` opens a new accumulation window; the caller decides how wide it is.
  Guarded by a mutex because `onContact` may run on several PhysX worker threads.
- `PhysXWrapper::CreateSceneEx` now installs the tracker as the simulation event callback for any
  scene created with `PxwSceneFlag::eENABLE_CONTACT_EVENTS`. The flag already existed and already
  selected the notification-adding filter shader; previously the events had no consumer unless the
  caller supplied one. The UNDPWR world layer still overrides the callback with its own
  immediately afterwards, so its behaviour is unchanged.
- New exports `ClearArticulationContactFlags()` and
  `GetArticulationContactFlags(articulation, destFlags, capacity)`, plus a `Forget` call in
  `ReleaseArticulation` so a reused allocation cannot inherit stale flags.
- No change to any collision or solve decision, so a scene simulates identically with contact
  events on. `CMakeLists.txt` gained the new source file.
- `tests/PxwArticulationContactTests.cpp` pins the accumulation semantics: flags OR across a
  window, `Clear` opens a new one, reporting is per link and not per articulation, a scene without
  the flag reports nothing, and a released articulation is forgotten. It also pins the one
  surprise — PhysX runs no narrowphase for a sleeping island, so a resting articulation that falls
  asleep reports no contact until something wakes it. A policy-driven articulation never sleeps,
  since its joint targets are rewritten every step.
- The two `BUILD_TESTS` targets that compile the plugin sources in now share a
  `pxw_add_plugin_test` function rather than repeating the include/define/link block.

---

## Robot Removal + Vehicle Support Refactor

### Legacy robot layer removed

- Deleted `src/Robotics.{h,cpp}` and `src/PhysXWrapper_Robotics.cpp`, and the entire vendored `src/Eigen/` tree (Eigen was only used by the robot FK/IK code).
- Removed the `Pxw*Robot` / `PxwArticulationKinematicTree` exports from `include/PxwAPIs.h` / `src/PxwAPIs.cpp`, the `#include "Robotics.h"` and factory declarations from `PhysXWrapper.h`, and the `ToEigenMatrix4()` / `PxwSpatialForceData` / `PxwRobotJointType` definitions from `DataInterop.h`.
- `RemoveArticulationFromScene` now takes a raw `PxArticulationReducedCoordinate*` (matching what the retained `PhysxArticulationBody` uses); the unused `AddArticulationToScene` was dropped.
- The low-level `PxArticulationReducedCoordinate` API (~60 exports) is untouched.

### PhysX Vehicle2 support added

- Vendored and adapted the NVIDIA vehicle common snippets into `src/vehicle/` under namespace `pxw`: `VehicleBase`, `VehiclePhysXIntegration`, `VehicleDirectDrive`, `VehicleEngineDrive`. `PhysXActorVehicle::initialize` was extended to accept an optional chassis `PxGeometry*`, and `DirectDriveVehicle` gained a `mUseDirectWheelControl` flag that omits the command-response component for raw per-wheel control.
- Added blittable descriptors in `src/VehicleInterop.h`, the `PxwVehicle` opaque handle in `src/VehicleHelper.{h,cpp}`, and per-scene management in `src/VehicleModule.h` / `src/PhysXWrapper_Vehicle.cpp`.
- Wired `PxInitVehicleExtension` / `PxCloseVehicleExtension` and per-scene register/unregister into `PhysXWrapper_Basics.cpp`, stepping registered vehicles before each `PxScene::simulate()`.
- Added the vehicle C exports (lifecycle, per-part setup, commands, raw per-wheel control including `SetVehicleUseDirectWheelControl` / `SetVehicleDirectDriveThrottleParams`, and state readback) to `include/PxwAPIs.h` / `src/PxwAPIs.cpp`.
- `CMakeLists.txt`: dropped the removed robot sources, added the new vehicle sources, and linked `PhysXVehicle2_static_64`.
- `src/PhysXWrapper_Utils.cpp`: added an explicit `#include <unordered_map>` (previously pulled in transitively via Eigen).

---

## PhysX 5.6.1 Upgrade

This section describes changes made to the native plugin when upgrading from PhysX 5.4.2 to 5.6.1.

## Unity-Side Impact

### No breaking changes for Unity C# code

All extern "C" function signatures remain **binary-compatible**. The PhysX type renames
(e.g. `PxFEMSoftBodyMaterial*` to `PxDeformableVolumeMaterial*`) are typedefs to the same
underlying type, so `IntPtr` handles marshalled from Unity continue to work without changes.

### PxGeometryType enum shift (requires C# update)

PhysX 5.6.1 inserted `eCONVEXCORE` at index 4 and removed `eHAIRSYSTEM`, shifting values:

| Type | 5.4.x | 5.6.1 |
|------|-------|-------|
| eSPHERE | 0 | 0 |
| ePLANE | 1 | 1 |
| eCAPSULE | 2 | 2 |
| eBOX | 3 | 3 |
| eCONVEXCORE | — | 4 (new) |
| eCONVEXMESH | 4 | 5 |
| ePARTICLESYSTEM | 5 | 6 |
| eTETRAHEDRONMESH | 6 | 7 |
| eTRIANGLEMESH | 7 | 8 |
| eHEIGHTFIELD | 8 | 9 |
| eCUSTOM | 9 | 10 |

The Unity C# `PxGeometryType` enum must be updated to match: insert `ConvexCore` after `Box`
and remove `HairSystem`.

Other enum parameters (solver types, articulation flags, etc.) appear unchanged but should
be verified if unexpected behavior occurs.

### Function names preserved

All exported function names (`CreateFEMSoftBody`, `CreatePxFEMSoftBodyMaterial`, etc.) are
unchanged. No Unity DllImport attributes need updating.

## Build System Changes

### CMakeLists.txt

- **C++ standard bumped from 14 to 17** (recommended for PhysX 5.6.x).
- **New `USE_GPU` option** (`-DUSE_GPU=ON`):
  - Default: `OFF` (CPU-only build). Links only CPU PhysX libraries.
  - When `ON`: Links `PhysXGpu_64` and CUDA libraries. Required for PBD particle systems
    and FEM deformable volumes.
- Windows GPU builds use `find_package(CUDAToolkit)` to locate CUDA.
- Linux GPU builds expect `CUDA_TOOLKIT_ROOT_DIR` environment variable or `/usr/local/cuda`.
- CPU-only builds: CUDA initialization in `InitPhysX()` is compiled out via `#ifdef USE_GPU`.
  PBD particle system functions are also compiled out in CPU-only builds (the DLL will not
  export those symbols). FEM deformable volumes still compile but require GPU at runtime.
- The `USE_GPU` define is automatically set as a preprocessor definition when the CMake
  option is enabled, allowing source code to use `#ifdef USE_GPU` guards.

## Source Code Changes

### Deformable Volume API Migration (PxSoftBody -> PxDeformableVolume)

PhysX 5.5 renamed "Soft Body" to "Deformable Volume". All internal code has been migrated
to use the new API names. The old names still exist as deprecated typedefs in PhysX 5.6.1
headers but will eventually be removed.

**Type renames applied:**

| Old (5.4.2) | New (5.6.1) |
|---|---|
| `PxSoftBody` | `PxDeformableVolume` |
| `PxSoftBodyMesh` | `PxDeformableVolumeMesh` |
| `PxFEMSoftBodyMaterial` | `PxDeformableVolumeMaterial` |
| `PxFEMSoftBodyMaterialModel` | `PxDeformableVolumeMaterialModel` |
| `PxSoftBodyExt` | `PxDeformableVolumeExt` |
| `PxSoftBodyDataFlag` | `PxDeformableVolumeDataFlag` |
| `PxSoftBodyFlag::eDISABLE_SELF_COLLISION` | `PxDeformableBodyFlag::eDISABLE_SELF_COLLISION` |

**Method renames applied:**

| Old (5.4.2) | New (5.6.1) |
|---|---|
| `PxSoftBodyExt::createSoftBodyMesh()` | `PxDeformableVolumeExt::createDeformableVolumeMesh()` |
| `PxSoftBodyExt::createSoftBodyMeshNoVoxels()` | `PxDeformableVolumeExt::createDeformableVolumeMeshNoVoxels()` |
| `PxSoftBodyExt::allocateAndInitializeHostMirror()` | `PxDeformableVolumeExt::allocateAndInitializeHostMirror()` |
| `PxSoftBodyExt::transform()` | `PxDeformableVolumeExt::transform()` |
| `PxSoftBodyExt::updateMass()` | `PxDeformableVolumeExt::updateMass()` |
| `PxSoftBodyExt::copyToDevice()` | `PxDeformableVolumeExt::copyToDevice()` |
| `PxPhysics::createSoftBody()` | `PxPhysics::createDeformableVolume()` |
| `PxPhysics::createFEMSoftBodyMaterial()` | `PxPhysics::createDeformableVolumeMaterial()` |
| `softBodyMesh->getSoftBodyAuxData()` | `softBodyMesh->getDeformableVolumeAuxData()` |
| `softBody->setSoftBodyFlag()` | `softBody->setDeformableBodyFlag()` |

**Material method change:**

- `PxDeformableVolumeMaterial::setDamping()` has been replaced by `setElasticityDamping()`.

### Include Changes

- `extensions/PxSoftBodyExt.h` replaced with `extensions/PxDeformableVolumeExt.h`
- Added `extensions/PxDeformableVolumeExt.h` to `PhysXWrapper.h`

### Bug Fix

- **`CreateArticulationInternalStateCache`** in `PxwAPIs.cpp`: Fixed missing `return`
  statement. The function was calling `articulation->createCache()` but not returning the
  result. Now correctly returns the created cache pointer.

### Removed / Stubbed APIs

- **`PxMaterialFlag::eIMPROVED_PATCH_FRICTION`**: This flag was removed entirely in PhysX
  5.6.1. The HACK line that set it to `false` has been replaced with a comment noting the
  removal. No Unity-side action needed (flag had no binary representation).
- **`PxArticulationReducedCoordinate::setMaxCOMLinearVelocity()` /
  `setMaxCOMAngularVelocity()`**: These methods were removed in PhysX 5.6.1 with no direct
  replacement. The wrapper functions `SetArticulationMaxCOMLinearVelocity` and
  `SetArticulationMaxCOMAngularVelocity` are now no-ops. Unity callers won't crash but the
  velocity clamping will have no effect.
- **`PX_MAX_TETID`**: Renamed to `PX_MAX_NB_DEFORMABLE_VOLUME_TET` in PhysX 5.6.1.
  Updated in `SoftBodyHelper.cpp`.

### GPU / CPU Conditional Compilation

PBD particle system code (particle fluids, cloth, attachments) requires GPU support and is
now conditionally compiled:

- **CPU-only builds** (`USE_GPU=OFF`): The PBD/particle source files
  (`PhysXWrapper_PBD.cpp`, `ParticleSystemHelper.cpp`) are excluded from compilation. The
  corresponding extern "C" functions are not exported from the DLL. CUDA initialization in
  `InitPhysX()` is also compiled out.
- **GPU builds** (`USE_GPU=ON`): All PBD/particle functionality is compiled and linked as
  before.

Unity C# code that uses PBD features will need to handle the missing DLL exports gracefully
when using the CPU-only variant (e.g., check for `EntryPointNotFoundException`).

### HACK Values Preserved

All HACK-marked values have been preserved unchanged:

- `bounceThresholdVelocity = 0.2f` (scene creation)
- `PxMaterialFlag::eIMPROVED_PATCH_FRICTION` flag removed in 5.6.1 (was HACK set to false)
- `joint->setFrictionCoefficient(0.0f)` (articulation link creation)
- `joint->setMaxJointVelocity(100.0f)` (articulation link creation)
- `link->setMaxDepenetrationVelocity(1.0f)` (articulation link creation)
- `link->setMaxLinearVelocity(1000.0f)` (articulation link creation)
- `link->setMaxAngularVelocity(17.453f)` (articulation link creation)
- `link->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true)` (articulation link creation)
- `PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES` set to `true` (articulation root creation)
- Solver iteration counts set to same value for position and velocity (articulation root)

### Deprecated APIs Still in Use

The following deprecated PhysX APIs are still used and functional in 5.6.1. They should be
migrated in a future pass (see SUGGESTIONS.md):

- `PxFEMParameters` struct and `setParameter()` method
- `PxCudaContextManager::allocPinnedHostBuffer` / `freePinnedHostBuffer`
- `PX_PINNED_HOST_ALLOC_T` / `PX_PINNED_HOST_FREE` macros
- `PxBVH33MidphaseDesc` (BVH33 midphase in triangle mesh cooking)
- `PxParticleClothCooker` (particle cloth cooking)
- `PxAnisotropyCallback` / `PxAnisotropyGenerator` (anisotropy for PBD)

## Files Modified

| File | Changes |
|---|---|
| `CMakeLists.txt` | C++17, `USE_GPU` option, conditional GPU linking, conditional PBD source files |
| `src/PhysXWrapper.h` | Include + type migrations, `#ifdef USE_GPU` guards for PBD includes/methods |
| `src/PhysXWrapper_Basics.cpp` | Version comment, `#ifdef USE_GPU` around CUDA init |
| `src/PhysXWrapper_Rigid_Soft_Bodies.cpp` | Full deformable volume migration |
| `src/PhysXWrapper_Utils.cpp` | Material type migration |
| `src/SoftBodyHelper.h` | Include + type migrations |
| `src/SoftBodyHelper.cpp` | Full deformable volume ext migration, `PX_MAX_TETID` rename |
| `include/PxwAPIs.h` | Material type in function signatures, `#ifdef USE_GPU` for PBD declarations |
| `src/PxwAPIs.cpp` | Material type, bug fix, `eIMPROVED_PATCH_FRICTION` removal, articulation velocity stub, `#ifdef USE_GPU` for PBD functions |
| `README.md` | Version, build instructions |
