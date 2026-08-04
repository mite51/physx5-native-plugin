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

	// Quaternion-first transform, laid out to match physx::PxTransform (PxQuat q; PxVec3 p;)
	// and the managed UNDPWR SimTransform, which is quaternion-first for exactly that reason.
	//
	// PxwTransformData above is position-first and cannot be reordered without breaking the
	// legacy PxwAPIs surface and its managed PxTransformData counterpart, which agree with
	// each other. The UNDPWR interop structs that a snapshot never touches but that cross to
	// managed code field-by-field (pose readback and the mass frame) use this type instead,
	// so their bytes line up with SimTransform rather than being silently transposed.
	struct PxwPose
	{
		PxQuat quaternion;
		PxVec3 position;

		PxwPose()
		{
			quaternion = PxQuat();
			position = PxVec3();
		}

		PxwPose(PxTransform transform)
		{
			quaternion = transform.q;
			position = transform.p;
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
