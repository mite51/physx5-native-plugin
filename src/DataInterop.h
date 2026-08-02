#pragma once
#include "PxPhysicsAPI.h"

using namespace physx;

namespace pxw
{
	struct PxwTransformData
	{
		PxVec3 position;
		PxQuat quaternion;
		
		PxwTransformData()
		{
			position = PxVec3();
			quaternion = PxQuat();
		}

		PxwTransformData(PxVec3 p, PxQuat q)
		{
			position = p;
			quaternion = q;
		}

		PxwTransformData(PxTransform transform)
		{
			position = transform.p;
			quaternion = transform.q;
		}

		PxTransform ToPxTransform() const
		{
			return PxTransform(position, quaternion);
		}

	};

	// Bit flags for PxwSceneDesc::flags. Mirrors the subset of PxSceneFlag that the
	// deterministic simulation framework needs to control from managed code, plus a
	// plugin-specific flag to suppress the PVD connection attempt.
	struct PxwSceneFlag
	{
		enum Enum : PxU32
		{
			eENABLE_PCM = 1u << 0,
			eENABLE_CCD = 1u << 1,
			eENABLE_STABILIZATION = 1u << 2,
			eENABLE_ACTIVE_ACTORS = 1u << 3,
			eENABLE_ENHANCED_DETERMINISM = 1u << 4,
			eENABLE_DIRECT_GPU_API = 1u << 5,
			eDISABLE_PVD = 1u << 6
		};
	};

	// Full scene configuration. Everything here used to be hardcoded inside
	// PhysXWrapper::CreateScene; deterministic simulation needs explicit control of
	// all of it, in particular eENABLE_ENHANCED_DETERMINISM and the worker count.
	struct PxwSceneDesc
	{
		PxVec3 gravity;
		PxU32  flags;
		PxI32  pruningStructureType;
		PxI32  solverType;
		PxI32  broadPhaseType;          // -1 selects ABP on CPU and GPU broadphase when useGpu
		PxI32  cpuWorkerThreads;        // 0 runs solver tasks on the calling thread, which is what determinism requires
		PxI32  useGpu;
		PxReal bounceThresholdVelocity;
		PxReal frictionOffsetThreshold;
		PxU32  ccdMaxPasses;
	};

	struct PxwParticleData
	{
		int numParticles;
		PxVec4* positionInvMass;
		PxVec4* velocity;
		PxU32* phase;
	};

	struct PxwAnisotropyBuffer
	{
		PxVec4* anisotropy1;
		PxVec4* anisotropy2;
		PxVec4* anisotropy3;
	};

	struct PxwParticleSpringsData
	{
		int numSprings;
		PxParticleSpring* springs;
	};

	struct PxwFEMSoftBodyMeshData
	{
		int numVertices;
		PxVec4* positionInvMass;
		PxVec3* velocity;
	};
}
