cmake_minimum_required(VERSION 3.10)

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

# Platform detection
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PHYSX_PLATFORM_DIR "linux.clang")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PHYSX_PLATFORM_DIR "win.x86_64.vc143.mt")
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/../PhysX/physx/include
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)

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

# Link directories
if(PHYSX_BUILD_TYPE AND PHYSX_BUILD_TYPE STREQUAL "debug")
    message("Using PhysX debug version.")
endif()
link_directories(${PROJECT_SOURCE_DIR}/../PhysX/physx/bin/${PHYSX_PLATFORM_DIR}/${PHYSX_BUILD_TYPE})

link_directories(/usr/lib/wsl/lib) # For WSL

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

# Source files
set(SOURCES
    src/PxwAPIs.cpp
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
set(PhysXUnity LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/bin/Debug)
set(PhysXUnity LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/bin/Release)
set(PhysXUnity RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/build/bin/Debug)
set(PhysXUnity RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/build/bin/Release)
add_compile_definitions(PhysXUnity INTERFACE API_EXPORTS)

# Disable "Treat Warnings as Errors" for all platforms
if(MSVC)
    target_compile_options(PhysXUnity PRIVATE /W4 /WX-)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
else()
    target_compile_options(PhysXUnity PRIVATE -Wno-error)
endif()

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
