#pragma once
#include "PxPhysicsAPI.h"
#include "Eigen/Dense"

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

		Eigen::Matrix4f ToEigenMatrix4() const
		{
			// Directly compute the rotation matrix elements from the quaternion
			float qx = quaternion.x;
			float qy = quaternion.y;
			float qz = quaternion.z;
			float qw = quaternion.w;

			Eigen::Matrix4f transformationMatrix;
			transformationMatrix << 1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw, position.x,
									2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw, position.y,
									2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy, position.z,
									0, 0, 0, 1;

			return transformationMatrix;
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

	struct PxwSpatialForceData
	{
		PxVec3 force;
		PxVec3 torque;
	};

	struct PxwRobotJointType
	{
		enum Enum
		{
			eFIX = 0,
			ePRISMATIC = 1,
			eREVOLUTE = 2,
		};

		// Convert to PxArticulationJointType::Enum
		static PxArticulationJointType::Enum ToPxArticulationJointType(Enum e)
		{
			switch (e)
			{
			case eFIX: return PxArticulationJointType::eFIX;
			case ePRISMATIC: return PxArticulationJointType::ePRISMATIC;
			case eREVOLUTE: return PxArticulationJointType::eREVOLUTE;
			default: return PxArticulationJointType::eUNDEFINED;
			}
		}
	};
}
