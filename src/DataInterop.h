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
