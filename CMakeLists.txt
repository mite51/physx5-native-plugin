cmake_minimum_required(VERSION 3.10)

project(PhysX5ForUnity)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
OPTION(API_EXPORTS "Whether export or import the APIs." ON)

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
link_directories(${PROJECT_SOURCE_DIR}/../PhysX/physx/bin/${PHYSX_PLATFORM_DIR}/${PHYSX_BUILD_TYPE}) # This is the path where PhysX libraries are installed

link_directories(/usr/lib/wsl/lib) # For WSL

# Source files
set(SOURCES
    src/PxwAPIs.cpp
    src/PhysXWrapper_Basics.cpp
    src/PhysXWrapper_PBD.cpp
    src/PhysXWrapper_Rigid_Soft_Bodies.cpp
    src/PhysXWrapper_Robotics.cpp
    src/PhysXWrapper_Utils.cpp
    src/ParticleSystemHelper.cpp
    src/Robotics.cpp
    src/SoftBodyHelper.cpp
)

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
    target_link_libraries(PhysXUnity PRIVATE
        -Wl,--start-group
        PhysX_static_64
        PhysXFoundation_static_64
        PhysXCommon_static_64
        PhysXCooking_static_64
        PhysXExtensions_static_64
        PhysXGpu_64
        -Wl,--end-group
        cuda
    )
    set_target_properties(PhysXUnity PROPERTIES 
        BUILD_RPATH "$ORIGIN:${CUDA_TOOLKIT_ROOT_DIR}/lib64"
    )

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(PhysXUnity PRIVATE
        PhysX_64
        PhysXFoundation_64
        PhysXCommon_64
        PhysXCooking_64
        PhysXExtensions_static_64
    )
endif()

# Common properties for all platforms
target_include_directories(PhysXUnity PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
