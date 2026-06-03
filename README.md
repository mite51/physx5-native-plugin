# PhysX 5 Native Plugin for Unity

# Built for PhysX 5.6.1

A simple wrapper for using PhysX 5 in Unity.

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