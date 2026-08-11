# Native Plugin - Changes

This document describes changes made to the native plugin: first the PhysX 5.6.1 upgrade, then the robot-removal / vehicle-support refactor.

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
