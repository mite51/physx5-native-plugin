cmake_minimum_required(VERSION 3.15)

project(PhysX5ForUnity)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
OPTION(API_EXPORTS "Whether export or import the APIs." ON)
OPTION(USE_GPU "Enable GPU support (requires CUDA toolkit and PhysXGpu library)." OFF)

# Platform-agnostic hidden visibility settings
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)

# Check for CMAKE_BUILD_TYPE and enforce it
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (Debug, Release, etc.)" FORCE)
    message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to Release.")
endif()

# Check the build type
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PHYSX_BUILD_TYPE "debug")
    message("Using PhysX debug version.")
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(PHYSX_BUILD_TYPE "release")
    message("Using PhysX release version.")
else()
    message(WARNING "Unknown build type: ${CMAKE_BUILD_TYPE}")
endif()

# ---------------------------------------------------------------------------
# Where PhysX comes from
#
# Three separate things have to agree about which PhysX build is in play: the
# import libraries this project links, the static libraries it absorbs, and the
# DLLs it copies next to its own output. They used to be derived independently,
# and when they drifted apart the result did not look like a build problem. It
# looked like physics: a page of confident failures in determinism tests that
# could not depend on the change being tested, and, once a virtual was added to
# an exported class, an access violation. So there is one directory here, and
# everything downstream is derived from it.
#
# PhysX bakes its CRT choice into the output directory name, and a static
# library built against one CRT cannot be linked into a binary using the other.
# The CRT is therefore also chosen once, and this project's own compilation is
# made to match rather than being set independently.
#
#   mt  NV_USE_STATIC_WINCRT=ON,  compiler/vc17win64-cpu-only
#   md  NV_USE_STATIC_WINCRT=OFF, compiler/vc17win64-cpu-only-md
#
# mt is the default because a Unity native plugin cannot assume the machine it
# lands on has the Visual C++ redistributable installed.
# ---------------------------------------------------------------------------
set(PHYSX_ROOT "${PROJECT_SOURCE_DIR}/../PhysX/physx" CACHE PATH "Root of the PhysX SDK checkout.")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PHYSX_PLATFORM_DIR "linux.clang")
else()
    set(PHYSX_CRT "mt" CACHE STRING "Windows CRT the PhysX build was made with: mt (static) or md (dynamic).")
    set_property(CACHE PHYSX_CRT PROPERTY STRINGS mt md)
    if(NOT PHYSX_CRT STREQUAL "mt" AND NOT PHYSX_CRT STREQUAL "md")
        message(FATAL_ERROR "PHYSX_CRT must be mt or md, got '${PHYSX_CRT}'.")
    endif()
    set(PHYSX_PLATFORM_DIR "win.x86_64.vc143.${PHYSX_CRT}")

    if(PHYSX_CRT STREQUAL "mt")
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    endif()
endif()

set(PHYSX_BIN_DIR "${PHYSX_ROOT}/bin/${PHYSX_PLATFORM_DIR}/${PHYSX_BUILD_TYPE}")

if(NOT EXISTS "${PHYSX_BIN_DIR}")
    message(FATAL_ERROR
        "No PhysX ${PHYSX_BUILD_TYPE} build at ${PHYSX_BIN_DIR}.\n"
        "Build PhysX first, or point PHYSX_ROOT / PHYSX_CRT at one that exists.")
endif()

message(STATUS "Using PhysX from ${PHYSX_BIN_DIR}")

# The CMake build directory PhysX itself was configured in, so that the PhysX
# binaries can be brought up to date as part of building this project. Nothing
# here needs it to exist; it is only used when BUILD_PHYSX_FIRST is on.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PHYSX_DEFAULT_COMPILER_DIR "${PHYSX_ROOT}/compiler/linux-${PHYSX_BUILD_TYPE}")
elseif(PHYSX_CRT STREQUAL "md")
    set(PHYSX_DEFAULT_COMPILER_DIR "${PHYSX_ROOT}/compiler/vc17win64-cpu-only-md")
else()
    set(PHYSX_DEFAULT_COMPILER_DIR "${PHYSX_ROOT}/compiler/vc17win64-cpu-only")
endif()
set(PHYSX_COMPILER_DIR "${PHYSX_DEFAULT_COMPILER_DIR}" CACHE PATH "PhysX CMake build directory, used by BUILD_PHYSX_FIRST.")

# Include directories
include_directories(
    ${PHYSX_ROOT}/include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

link_directories(${PHYSX_BIN_DIR})

link_directories(/usr/lib/wsl/lib) # For WSL

# ---------------------------------------------------------------------------
# Keep the PhysX binaries current
#
# This project is developed against a patched PhysX, so its sources change as
# often as this project's own do. Building here without rebuilding there tests
# the previous PhysX and reports the results as if they were this one's, which
# is the failure mode described above. Making every target depend on a PhysX
# build removes the possibility rather than documenting it.
#
# Turn off with -DBUILD_PHYSX_FIRST=OFF when PhysX is a fixed dependency.
# ---------------------------------------------------------------------------
OPTION(BUILD_PHYSX_FIRST "Rebuild PhysX before building this project." ON)

if(BUILD_PHYSX_FIRST AND EXISTS "${PHYSX_COMPILER_DIR}/CMakeCache.txt")
    add_custom_target(PhysXRuntime ALL
        COMMAND ${CMAKE_COMMAND} --build "${PHYSX_COMPILER_DIR}" --config ${PHYSX_BUILD_TYPE}
            --target PhysX PhysXCommon PhysXFoundation PhysXCooking
                     PhysXExtensions PhysXVehicle2 PhysXPvdSDK
        COMMENT "Bringing PhysX in ${PHYSX_COMPILER_DIR} up to date"
        USES_TERMINAL
    )
elseif(BUILD_PHYSX_FIRST)
    message(WARNING
        "BUILD_PHYSX_FIRST is on but ${PHYSX_COMPILER_DIR} is not a configured CMake "
        "build directory; the PhysX binaries in ${PHYSX_BIN_DIR} will be used as they are.")
endif()

# Adds the PhysX rebuild as a prerequisite of a target, where one is available.
function(pxw_depends_on_physx target)
    if(TARGET PhysXRuntime)
        add_dependencies(${target} PhysXRuntime)
    endif()
endfunction()

# Puts the PhysX DLLs beside an executable, from the one build directory chosen
# above. Only Windows needs this; the Linux build links PhysX statically and
# uses an $ORIGIN rpath for what remains.
function(pxw_copy_physx_dlls target)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        return()
    endif()
    set(dlls PhysX_64.dll PhysXFoundation_64.dll PhysXCommon_64.dll PhysXCooking_64.dll PVDRuntime_64.dll)
    if(USE_GPU)
        list(APPEND dlls PhysXGpu_64.dll)
    endif()
    foreach(dll ${dlls})
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${PHYSX_BIN_DIR}/${dll}" $<TARGET_FILE_DIR:${target}>)
    endforeach()
endfunction()

if(USE_GPU)
    message(STATUS "GPU support ENABLED")
    add_compile_definitions(USE_GPU)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        if(DEFINED ENV{CUDA_TOOLKIT_ROOT_DIR})
            set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_TOOLKIT_ROOT_DIR})
        elseif(NOT CUDA_TOOLKIT_ROOT_DIR)
            set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
        endif()
        link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)
    endif()
else()
    message(STATUS "GPU support DISABLED (CPU-only build)")
endif()

# Where the built plugin should be copied so the Unity package picks it up.
# Override with -DUNITY_PLUGIN_DIR=... when the package lives elsewhere.
set(UNITY_PLUGIN_DIR "" CACHE PATH "Unity package Plugins directory to copy build output into.")
if(NOT UNITY_PLUGIN_DIR)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(UNITY_PLUGIN_DIR "${PROJECT_SOURCE_DIR}/../physx5-for-unity/Plugins/Linux/x86_64")
    else()
        set(UNITY_PLUGIN_DIR "${PROJECT_SOURCE_DIR}/../physx5-for-unity/Plugins/Windows/x86_64")
    endif()
endif()

# Source files
set(SOURCES
    src/PxwAPIs.cpp
    src/PxwUndpwr.cpp
    src/ArticulationContacts.cpp
    src/PhysXWrapper_Basics.cpp
    src/PhysXWrapper_Rigid_Soft_Bodies.cpp
    src/PhysXWrapper_Utils.cpp
    src/PhysXWrapper_Vehicle.cpp
    src/SoftBodyHelper.cpp
    src/VehicleHelper.cpp
    src/vehicle/VehicleBase.cpp
    src/vehicle/VehiclePhysXIntegration.cpp
    src/vehicle/VehicleDirectDrive.cpp
    src/vehicle/VehicleEngineDrive.cpp
)

if(USE_GPU)
    list(APPEND SOURCES
        src/PhysXWrapper_PBD.cpp
        src/ParticleSystemHelper.cpp
    )
endif()

add_library(PhysXUnity SHARED ${SOURCES})

# These were previously written as bare set() calls, which created stray variables
# instead of configuring the target, so the output landed wherever MSVC defaulted to.
set_target_properties(PhysXUnity PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_DEBUG   ${PROJECT_SOURCE_DIR}/build/bin/Debug
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/bin/Release
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${PROJECT_SOURCE_DIR}/build/bin/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/bin/Release
)

if(API_EXPORTS)
    target_compile_definitions(PhysXUnity PRIVATE API_EXPORTS)
endif()

# Disable "Treat Warnings as Errors" for all platforms. The CRT is not selected
# here: it is CMAKE_MSVC_RUNTIME_LIBRARY above, derived from the PhysX build
# being linked, because the two have to agree.
if(MSVC)
    target_compile_options(PhysXUnity PRIVATE /W4 /WX-)
else()
    target_compile_options(PhysXUnity PRIVATE -Wno-error)
endif()

pxw_depends_on_physx(PhysXUnity)

# Platform-specific linking
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PHYSX_CORE_LIBS
        PhysX_static_64
        PhysXFoundation_static_64
        PhysXCommon_static_64
        PhysXCooking_static_64
        PhysXExtensions_static_64
        PhysXVehicle2_static_64
        PhysXPvdSDK_static_64
    )

    if(USE_GPU)
        target_link_libraries(PhysXUnity PRIVATE
            -Wl,--start-group
            ${PHYSX_CORE_LIBS}
            PhysXGpu_64
            -Wl,--end-group
            cuda
        )
        set_target_properties(PhysXUnity PROPERTIES
            BUILD_RPATH "$ORIGIN:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
        )
    else()
        target_link_libraries(PhysXUnity PRIVATE
            -Wl,--start-group
            ${PHYSX_CORE_LIBS}
            -Wl,--end-group
        )
        set_target_properties(PhysXUnity PROPERTIES
            BUILD_RPATH "$ORIGIN"
        )
    endif()

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PHYSX_CORE_LIBS
        PhysX_64
        PhysXFoundation_64
        PhysXCommon_64
        PhysXCooking_64
        PhysXExtensions_static_64
        PhysXVehicle2_static_64
        PhysXPvdSDK_static_64
    )

    if(USE_GPU)
        find_package(CUDAToolkit QUIET)
        target_link_libraries(PhysXUnity PRIVATE
            ${PHYSX_CORE_LIBS}
            PhysXGpu_64
        )
        if(CUDAToolkit_FOUND)
            target_link_libraries(PhysXUnity PRIVATE CUDA::cuda_driver)
        endif()
    else()
        target_link_libraries(PhysXUnity PRIVATE
            ${PHYSX_CORE_LIBS}
        )
    endif()
endif()

# Common properties for all platforms
target_include_directories(PhysXUnity PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ---------------------------------------------------------------------------
# Deploy to the Unity package
#
# Copying was previously a manual step, which made it easy to test Unity against
# a stale DLL. Disable with -DDEPLOY_TO_UNITY=OFF.
# ---------------------------------------------------------------------------
OPTION(DEPLOY_TO_UNITY "Copy build output into the Unity package Plugins folder." ON)

if(DEPLOY_TO_UNITY AND EXISTS "${UNITY_PLUGIN_DIR}")
    message(STATUS "Deploying build output to ${UNITY_PLUGIN_DIR}")

    add_custom_command(TARGET PhysXUnity POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:PhysXUnity> "${UNITY_PLUGIN_DIR}"
        COMMENT "Copying PhysXUnity into the Unity package"
    )

    # The PhysX runtime libraries have to sit next to the wrapper or Unity cannot
    # load it. They are only shipped as shared libraries on Windows.
    #
    # The existence test is deliberately at build time rather than configure
    # time: PhysXRuntime may have produced the DLL moments ago, and a configure
    # time test would have decided it was absent and silently left Unity running
    # against whatever was already deployed.
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(PHYSX_RUNTIME_DLLS
            PhysX_64.dll
            PhysXFoundation_64.dll
            PhysXCommon_64.dll
            PhysXCooking_64.dll
            PVDRuntime_64.dll
        )
        if(USE_GPU)
            list(APPEND PHYSX_RUNTIME_DLLS PhysXGpu_64.dll)
        endif()

        foreach(dll ${PHYSX_RUNTIME_DLLS})
            add_custom_command(TARGET PhysXUnity POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${PHYSX_BIN_DIR}/${dll}" "${UNITY_PLUGIN_DIR}"
            )
        endforeach()
    endif()
elseif(DEPLOY_TO_UNITY)
    message(WARNING "DEPLOY_TO_UNITY is on but ${UNITY_PLUGIN_DIR} does not exist; skipping deployment.")
endif()

# ---------------------------------------------------------------------------
# Native determinism tests
#
# Built from the plugin sources rather than linked against the DLL, so the test
# and the code under test share a single PhysX instance and the test can reach
# PhysX objects directly.
# ---------------------------------------------------------------------------
OPTION(BUILD_TESTS "Build the native determinism test executable." OFF)

if(BUILD_TESTS)
    # A test executable that compiles the plugin sources in alongside its own.
    function(pxw_add_plugin_test target source)
        add_executable(${target} ${source} ${SOURCES})

        target_include_directories(${target} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

        # The plugin sources are compiled into the executable, so the API macro must
        # resolve to dllexport rather than dllimport.
        target_compile_definitions(${target} PRIVATE API_EXPORTS)

        if(MSVC)
            target_compile_options(${target} PRIVATE /W4 /WX-)
        else()
            target_compile_options(${target} PRIVATE -Wno-error)
        endif()

        if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
            if(USE_GPU)
                target_link_libraries(${target} PRIVATE
                    -Wl,--start-group ${PHYSX_CORE_LIBS} PhysXGpu_64 -Wl,--end-group cuda)
            else()
                target_link_libraries(${target} PRIVATE
                    -Wl,--start-group ${PHYSX_CORE_LIBS} -Wl,--end-group)
            endif()
            set_target_properties(${target} PROPERTIES BUILD_RPATH "$ORIGIN")
        else()
            target_link_libraries(${target} PRIVATE ${PHYSX_CORE_LIBS})
            if(USE_GPU)
                target_link_libraries(${target} PRIVATE PhysXGpu_64)
                if(CUDAToolkit_FOUND)
                    target_link_libraries(${target} PRIVATE CUDA::cuda_driver)
                endif()
            endif()
        endif()

        pxw_copy_physx_dlls(${target})
        pxw_depends_on_physx(${target})
    endfunction()

    pxw_add_plugin_test(PxwUndpwrTests tests/PxwUndpwrTests.cpp)

    # Per-link articulation contact reporting: accumulation window semantics.
    pxw_add_plugin_test(PxwArticulationContactTests tests/PxwArticulationContactTests.cpp)

    # Investigates the actor-pose round trip in PhysX itself. Links only PhysX, no
    # plugin sources, so nothing in the wrapper can influence the measurement.
    add_executable(PxwPoseRoundTripTests tests/PxwPoseRoundTripTests.cpp)
    target_include_directories(PxwPoseRoundTripTests PRIVATE ${PHYSX_ROOT}/include)
    if(MSVC)
        target_compile_options(PxwPoseRoundTripTests PRIVATE /W4 /WX- /fp:precise)
    endif()
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(PxwPoseRoundTripTests PRIVATE
            -Wl,--start-group ${PHYSX_CORE_LIBS} -Wl,--end-group)
        set_target_properties(PxwPoseRoundTripTests PROPERTIES BUILD_RPATH "$ORIGIN")
    else()
        target_link_libraries(PxwPoseRoundTripTests PRIVATE ${PHYSX_CORE_LIBS})
    endif()

    pxw_copy_physx_dlls(PxwPoseRoundTripTests)
    pxw_depends_on_physx(PxwPoseRoundTripTests)

    # Bisects where bitwise divergence enters a rollback. Links only PhysX, so a
    # failure here cannot be blamed on anything in the wrapper.
    add_executable(PxwRollbackRepro tests/PxwRollbackRepro.cpp)
    target_include_directories(PxwRollbackRepro PRIVATE ${PHYSX_ROOT}/include)
    if(MSVC)
        target_compile_options(PxwRollbackRepro PRIVATE /W4 /WX- /fp:precise)
    endif()
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        target_link_libraries(PxwRollbackRepro PRIVATE
            -Wl,--start-group ${PHYSX_CORE_LIBS} -Wl,--end-group)
        set_target_properties(PxwRollbackRepro PROPERTIES BUILD_RPATH "$ORIGIN")
    else()
        target_link_libraries(PxwRollbackRepro PRIVATE ${PHYSX_CORE_LIBS})
    endif()

    pxw_copy_physx_dlls(PxwRollbackRepro)
    pxw_depends_on_physx(PxwRollbackRepro)

    enable_testing()
    add_test(NAME PxwUndpwrTests COMMAND PxwUndpwrTests)
    add_test(NAME PxwPoseRoundTripTests COMMAND PxwPoseRoundTripTests)
    add_test(NAME PxwRollbackRepro COMMAND PxwRollbackRepro)
endif()
