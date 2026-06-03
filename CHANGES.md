# PhysX 5.6.1 Upgrade - Changes

This document describes changes made to the native plugin when upgrading from PhysX 5.4.2 to 5.6.1.

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
