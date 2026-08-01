# PhysX 5 Native Plugin for Unity

# Built for PhysX 5.6.1

A simple wrapper for using PhysX 5 in Unity. Supports rigid bodies, articulations, FEM soft bodies, PBD cloth/fluid and PhysX Vehicle2 vehicles.

## Vehicles (PhysX Vehicle2)

The wrapper exposes PhysX Vehicle2 through an opaque `PxwVehicle` handle. The NVIDIA vehicle snippet classes are vendored and adapted under `src/vehicle/` (`PxwBaseVehicle`, `PxwPhysXActorVehicle`, `PxwDirectDriveVehicle`, `PxwEngineDriveVehicle`). Vehicles are authored from plain, blittable descriptors defined in `src/VehicleInterop.h` (one per part: wheel, tire, suspension, engine, gearbox, autobox, clutch, differential, etc.), so the interop ABI is independent of PhysX's internal structs. Per-scene setup (`PxVehiclePhysXSimulationContext`, unit-cylinder sweep mesh and the vehicle registry) lives in `src/PhysXWrapper_Vehicle.cpp`; registered vehicles are stepped before `PxScene::simulate()`. All three drive models are supported (direct drive, engine drive, and raw per-wheel control). The C exports live in `include/PxwAPIs.h` / `src/PxwAPIs.cpp`.

> The legacy robot layer (`Robotics.*`, `PhysXWrapper_Robotics.cpp`) and the vendored Eigen library have been removed. The low-level `PxArticulationReducedCoordinate` API remains.

## Supported Platforms

Supported platforms: Windows and Linux, x86_64 (for details, see PhysX 5's platform requirements).

GPU features (PBD particle systems, FEM soft bodies) require a CUDA-compatible GPU (i.e., an Nvidia GPU). CPU-only builds support rigid bodies, articulations, and collision queries without CUDA.

## Prerequisites

Knowledge about [Nvidia PhysX 5 SDK](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/) is needed, which should be downloaded and built.
This repo assumes that it is located in the same directory as the PhysX folder. Otherwise, adjust `CMakeLists.txt` accordingly.

```plaintext
ParentDirectory/
├── physx5-native-plugin/
├── PhysX/
│   ├── physx/
│   ├── ...
```

## Build

Note that `CMAKE_BUILD_TYPE` determines the PhysX libs used for both Release and Debug builds after the project is made.

### CPU-only build (default)

```bash
git clone git@github.com:yafei-ou/physx5-native-plugin.git
cd physx5-native-plugin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

### GPU build (requires CUDA toolkit)

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
cmake --build . --config Release
```