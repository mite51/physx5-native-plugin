---
name: rebuild-plugin-dlls
description: >-
  Rebuild PhysXUnity and deploy plugin DLLs into the Unity package. Use when
  the user asks to rebuild the plugin, update plugin DLLs, refresh native
  binaries, or copy PhysXUnity into physx5-for-unity.
---

# Rebuild plugin DLLs

Rebuild the Windows Release `PhysXUnity` plugin and deploy it (plus PhysX
runtime DLLs) into `../physx5-for-unity/Plugins/Windows/x86_64`.

## Critical: which cmake to use

On this machine, the first `cmake` on PATH is often
`c:\devkitPro\msys2\usr\bin\cmake.exe`. That build **cannot** drive the
Visual Studio generator and fails with:

```text
Error: could not create CMAKE_GENERATOR "Visual Studio 17 2022"
```

Always invoke the official CMake explicitly:

```powershell
& "C:\Program Files\CMake\bin\cmake.exe" ...
```

Do not rely on bare `cmake` unless you have verified `Get-Command cmake`
points at `C:\Program Files\CMake\bin\cmake.exe`.

## Default workflow

Assume the existing `build/` tree is already configured. Do not reconfigure
unless the cache is missing or the user asks for different options.

1. Confirm cache defaults (expect these unless the user overrides):
   - Generator: Visual Studio 17 2022 / x64
   - Config: Release
   - `USE_GPU=OFF`
   - `DEPLOY_TO_UNITY=ON`
   - `BUILD_PHYSX_FIRST=OFF`
   - Deploy dir: `../physx5-for-unity/Plugins/Windows/x86_64`

2. Clean-rebuild the plugin target so Unity never gets a stale incremental
   output:

```powershell
& "C:\Program Files\CMake\bin\cmake.exe" --build build --config Release --target PhysXUnity --clean-first
```

3. Verify timestamps after the build:

```powershell
Get-ChildItem build/bin/Release/PhysXUnity.dll, ../physx5-for-unity/Plugins/Windows/x86_64/*.dll |
  Format-Table LastWriteTime, Length, FullName -AutoSize
```

`PhysXUnity.dll` must be fresh in **both** locations. PhysX runtime DLLs
(`PhysX_64.dll`, `PhysXCommon_64.dll`, `PhysXCooking_64.dll`,
`PhysXFoundation_64.dll`, `PVDRuntime_64.dll`) are copied via
`copy_if_different`; unchanged timestamps are fine when PhysX itself was
not rebuilt.

## Optional overrides

Only change these when the user explicitly asks:

| Request | Action |
|---------|--------|
| GPU build | Reconfigure with `-DUSE_GPU=ON`, then rebuild |
| Also rebuild PhysX | Reconfigure with `-DBUILD_PHYSX_FIRST=ON`, or build PhysX separately first |
| Incremental only | Drop `--clean-first` (faster; risk of stale output if sources look unchanged) |
| Skip Unity deploy | Reconfigure with `-DDEPLOY_TO_UNITY=OFF` |

## First-time / broken build tree

If `build/CMakeCache.txt` is missing or misconfigured:

```powershell
& "C:\Program Files\CMake\bin\cmake.exe" -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
& "C:\Program Files\CMake\bin\cmake.exe" --build build --config Release --target PhysXUnity --clean-first
```

PhysX release binaries must already exist at
`../PhysX/physx/bin/win.x86_64.vc143.mt/release` (default `PHYSX_CRT=mt`).

## Report back

Keep the reply short:

- Whether the clean rebuild succeeded
- Where `PhysXUnity.dll` landed and its timestamp
- Note if only the wrapper was rebuilt (PhysX runtimes unchanged)
- Mention if a leftover `PhysXGpu_64.dll` is present from an older GPU build
  while the current config is CPU-only
