# PhysX 5 Native Plugin for Unity

A simple wrapper for using PhysX 5 in Unity.

## Supported Platforms

As PhysX 5 requires CUDA to run the GPU simulations, including position-based dynamics (PBD) and finite-element method (FEM), this plugin requires a CUDA-compatible GPU (i.e., an Nvidia GPU).

Supported platforms: Windows and Linux, x86_64 (for details, see PhysX 5's platform requirements).

## Prerequisites

Knowledge about [Nvidia PhysX 5 SDK](https://nvidia-omniverse.github.io/PhysX/physx/5.2.1/index.html) (version 5.2.1) is needed, which should be downloaded and built.
This repo assumes that it is located in the same directory as the PhysX folder. Otherwise, adjust `CMakeLists.txt` accordingly.

```plaintext
ParentDirectory/
├── physx5-native-plugin/
├── PhysX/
│   ├── physx/
│   ├── ...
```

## Build

```bash
git clone git@github.com:yafei-ou/physx5-native-plugin.git
cd physx5-native-plugin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```