# Interface Improvement Suggestions

Recommendations for future refactoring of the PhysX Unity wrapper. These are not required
for the 5.6.1 upgrade but will improve maintainability, performance, and forward-compatibility.

---

## 1. Migrate Remaining Deprecated APIs

### PxFEMParameters -> Individual Setters

`PxFEMParameters` struct and `PxDeformableBody::setParameter()` are deprecated. The
individual setter methods should be used instead:

| Old (via PxFEMParameters) | New Setter |
|---|---|
| `velocityDamping` | `PxDeformableBody::setLinearDamping()` |
| `settlingThreshold` | `PxDeformableBody::setSettlingThreshold()` |
| `sleepThreshold` | `PxDeformableBody::setSleepThreshold()` |
| `sleepDamping` | `PxDeformableBody::setSettlingDamping()` |
| `selfCollisionFilterDistance` | `PxDeformableBody::setSelfCollisionFilterDistance()` |
| `selfCollisionStressTolerance` | `PxDeformableVolume::setSelfCollisionStressTolerance()` |

This would also allow exposing these parameters individually through the C API, giving
Unity more fine-grained control.

### Pinned Host Buffer Allocation

`PxCudaContextManager::allocPinnedHostBuffer()` and `freePinnedHostBuffer()` are deprecated.
Migrate to `PxCudaHelpersExt::allocPinnedHostBuffer()` and
`PxCudaHelpersExt::freePinnedHostBuffer()` from `extensions/PxCudaHelpersExt.h`.

Similarly, `PX_PINNED_HOST_ALLOC_T` and `PX_PINNED_HOST_FREE` macros should be replaced
with the extension equivalents.

### BVH33 -> BVH34 Midphase

`PxMeshMidPhase::eBVH33` and `PxBVH33MidphaseDesc` are deprecated. The default midphase
is now `eBVH34` which offers better performance. The `CreateBV33TriangleMesh` function
should be updated to use BVH34, and ideally renamed (or the midphase type made a parameter).

### Particle Cloth Cooker

`PxParticleClothCooker` is deprecated. Evaluate whether the new `PxDeformableSurface` API
(added in PhysX 5.5) is a better fit for cloth simulation use cases.

---

## 2. Interface Design Improvements

### Separate Scene-Level and Global Operations

Currently `StepPhysics()` iterates all scenes. Consider exposing per-scene stepping:

```cpp
// Current: steps ALL scenes
PHYSX_WRAPPER_API void StepPhysics(PxReal dt);

// Suggested: explicit per-scene control
PHYSX_WRAPPER_API void StepScene(PxScene* scene, PxReal dt);
```

The `StepScene` method already exists internally but is not consistently used.

### Error Handling Pattern

The current `GetPhysxErrors()` approach returns all accumulated errors as a single string.
Consider:

- Adding error severity levels to the C API
- Providing a callback-based error notification mechanism
- Adding per-call error codes as return values for critical functions

### Material Creation Naming

Function names like `CreatePxFEMSoftBodyMaterial` reference the old PhysX type name. For
clarity in the Unity API, consider renaming to `CreateDeformableVolumeMaterial` in a future
version. Since these are DllImport names, this would require corresponding Unity C# changes.

### Articulation Cache API

The articulation cache functions have some inconsistencies:

- `CreateArticulationCache` and `CreateArticulationInternalStateCache` appear to do the
  same thing. Consider consolidating.
- `ApplyArticulationCache` takes a reference `PxArticulationCache&` while the direct cache
  access functions take pointers. Standardize on pointers for C interop consistency.
- The `GetArticulationJointPositions` / `SetArticulationJointPositions` functions
  both create/apply a cache internally. Provide an option to batch multiple cache
  operations for better performance.

### Soft Body Helper Cleanup

`PxwSoftBodyHelper` has a potential double-free bug in `Release()`:

```cpp
if (mCollisionMeshData->velocity)
    PX_PINNED_HOST_FREE(mCudaContextManager, mCollisionMeshData->positionInvMass);
```

This frees `positionInvMass` when checking `velocity`. Should likely free `velocity` instead,
or remove the block if `velocity` is never allocated.

### Expose More Deformable Volume Parameters

The C API exposes `CreateFEMSoftBody` with limited parameters. Consider exposing:

- Self-collision filter distance
- Linear damping / settling threshold / sleep parameters
- Kinematic target buffers for partially kinematic deformable volumes
- Speculative CCD toggle (`PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD`)

### Particle System Configuration

The `CreatePBDParticleSystem` function hardcodes several values (e.g. `maxParticlesPerVolume = 96`).
Consider exposing these as parameters.

---

## 3. GPU Variant Considerations

### Runtime GPU Detection

Instead of compile-time `USE_GPU`, consider runtime detection:

- Always link against GPU libraries if available
- Check CUDA device at runtime and fall back to CPU-only gracefully
- This is partially implemented already (the `PxGetSuggestedCudaDeviceOrdinal` check) but
  link-time dependencies prevent CPU-only execution when built with GPU libs

### PhysXGpu DLL Side-Loading

On Windows, `PhysXGpu_64.dll` can be loaded at runtime via `LoadLibrary`. This would allow
a single build that supports both CPU and GPU modes without recompilation. PhysX supports
this pattern through `PxSetPhysXGpuLoadHook`.

### CUDA Version Compatibility

PhysX 5.6.1 GPU features require CUDA compute capability. If building with newer CUDA
(e.g. 13.0), you may need to pass `-DCUDA_ARCH=compute_75` or similar to avoid targeting
removed architectures like `compute_70`. Check the PhysX build system for the supported
minimum compute capability.

---

## 4. General Code Quality

### Const Correctness

Many function parameters could be `const` (e.g. `PxScene*` in read-only operations).

### Memory Management

- Use RAII patterns or smart pointers for `PxwSoftBodyHelper`, `PxwPBDParticleSystemHelper`,
  etc. instead of raw `new`/`delete`.
- The `PxwParticleSystemObject` destructor is virtual but base `Release()` is not called in
  all derived class destructors consistently.

### Thread Safety

The `gPhysXWrapper` global singleton and `gMaterial` global variable are not thread-safe.
If multiple Unity threads call into the plugin, consider adding synchronization or making
the wrapper instance-based rather than global.

### StepPhysics Locking

The `mStep` flag used to prevent double-stepping is fragile. Consider using a proper state
machine or mutex-based approach for `StepPhysicsStart` / `StepPhysicsFetchResults` pairs.
