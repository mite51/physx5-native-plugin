#pragma once

#include "PxPhysicsAPI.h"
#include "DataInterop.h"
#include <vector>
#include <type_traits>
#include "Eigen/Dense"

using namespace physx;
using namespace pxw;
using namespace Eigen;

namespace pxw
{
	class PxwArticulationKinematicTree
	{
	protected:
		int mNbDriveJoints;
		PxScene* mScene;
		PxPhysics* mPhysics;
		PxArticulationReducedCoordinate* mArticulation;
		std::vector<PxArticulationLink*> mLinks;
		std::vector <std::pair< PxArticulationJointReducedCoordinate*, PxArticulationAxis::Enum>> mDriveJoints;

		PxwTransformData* mLinkPoses;
		PxArticulationCache* mArticulationCache;
		PxArticulationCache* mInitialArticulationCache;
		bool mInitialized;
	public:
		PxwArticulationKinematicTree(PxScene* scene, PxPhysics* physics, bool fixBase, bool disableSelfCollision)
		{
			mNbDriveJoints = 0;
			mScene = scene;
			mPhysics = physics;
			mArticulation = mPhysics->createArticulationReducedCoordinate();
			mArticulation->setSolverIterationCounts(32);
			mArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, fixBase);
			mArticulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, disableSelfCollision);
			mLinkPoses = NULL;
			mArticulationCache = NULL;
			mInitialArticulationCache = NULL;
			mInitialized = false;
		}

		virtual void Initialize();

		virtual void AddToScene();

		void RemoveFromScene();

		PxArticulationLink* CreateBase(PxwTransformData basePose, PxShape* shape, float density);

		PxArticulationLink* AddLink(
			PxArticulationLink* parentLink,
			PxwTransformData linkPose,
			PxwRobotJointType::Enum type,
			PxwTransformData jointPoseParent,
			PxwTransformData jointPoseChild,
			PxArticulationAxis::Enum dofAxis,
			PxShape* shape,
			float jointLimLower,
			float jointLimUpper,
			bool isDriveJoint,
			float driveGainP,
			float driveGainD,
			float driveMaxForce,
			float density
		);

		// Get the number of drive joints
		int GetNbDriveJoints() { return mNbDriveJoints; }

		float* GetJointPositions();

		PxwTransformData* GetLinkPoses();

		void DriveJoints(float* targetJointPositions);

		void ResetObject();

		virtual void Release();
	};

	// A simple fixed-base serial robot with a fixed base
	class PxwArticulationRobot : public PxwArticulationKinematicTree
	{
	protected:
		std::vector<PxArticulationLink*> mBodyLinks;
		std::vector<PxArticulationLink*> mEELinks;
		std::vector<PxArticulationJointReducedCoordinate*> mBodyLinkJoints;
		std::vector<PxArticulationJointReducedCoordinate*> mEEJoints;
		std::vector<PxArticulationJointReducedCoordinate*> mSerialLinkJoints;

		std::vector<Eigen::Matrix4f> mJointToJointTransforms;
		std::vector<Eigen::Matrix4d> mJointToJointTransformsDouble;
		Eigen::Matrix4f mBasePose;
		Eigen::Matrix4d mBasePoseD;

		Eigen::MatrixXf Jn; // Body Jacobian
		Eigen::MatrixXf Js; // Spatial Jacobian
	public:
		PxwArticulationRobot(PxScene* scene, PxPhysics* physics) : PxwArticulationKinematicTree(scene, physics, true, true) { }

		void Initialize() override;

		void CreateBase(PxwTransformData basePose, float density);

		PxArticulationLink* AddBodyLink(
			PxwTransformData linkPose,
			PxwRobotJointType::Enum type,
			PxwTransformData jointPoseParent,
			PxwTransformData jointPoseChild,
			PxShape* shape,
			float jointLimLower,
			float jointLimUpper,
			float driveGainP,
			float driveGainD,
			float driveMaxForce,
			float density
		);

		// Multiple EEs can be attached to the last link; must be called only after all other links have been added.
		PxArticulationLink* AddEndEffectorLink(
			PxwTransformData linkPose,
			PxwRobotJointType::Enum type,
			PxwTransformData jointPoseParent,
			PxwTransformData jointPoseChild,
			PxShape* shape,
			float jointLimLower,
			float jointLimUpper,
			float driveGainP,
			float driveGainD,
			float driveMaxForce,
			float density
		);

		/*
		* \brief Get the link incoming joint force for the n-th link
		*/
		PxwSpatialForceData GetLinkIncomingJointForce(int n);

		/*
		* \brief Calculate the forward kinematics from world to the joint frame of the last EE
		*/
		Eigen::Matrix4f ForwardKinematics(float* q);

		/*
		* \brief Calculate the forward kinematics of a given joint n
		*/
		Eigen::Matrix4f ForwardKinematicsJoint(float q, int n);

		Eigen::MatrixXf ComputeJacobianBody(float* q);

		Eigen::MatrixXf ComputeJacobianSpatial(float* q);

		bool InverseKinematics(float* qInit, PxwTransformData targetTransformEEJoint, float tolerance, int numIterations, float lambda);
		
		void Release() override;

	private:
		void NormalizeAngles(float* jointPositions);

		float** Allocate2DArray(size_t rows, size_t cols) {
			int i;
			float** array;

			// Allocate memory for row pointers
			array = (float**)malloc(rows * sizeof(float*));
			if (array == NULL) {
				// Handle memory allocation failure
				fprintf(stderr, "Memory allocation failed for row pointers\n");
				return NULL;
			}

			// Allocate memory for each row
			for (i = 0; i < rows; i++) {
				array[i] = (float*)malloc(cols * sizeof(float));
				if (array[i] == NULL) {
					// Handle memory allocation failure
					fprintf(stderr, "Memory allocation failed for row %d\n", i);

					// Free previously allocated memory before returning
					while (--i >= 0) {
						free(array[i]);
					}
					free(array);
					return NULL;
				}
			}

			return array;
		}

		PxTransformd ConvertPxTransformF2D(const physx::PxTransform& sourceTransform)
		{
			return PxTransformd(
				static_cast<double>(sourceTransform.p.x),
				static_cast<double>(sourceTransform.p.y),
				static_cast<double>(sourceTransform.p.z),
				PxQuatd(
					static_cast<double>(sourceTransform.q.x),
					static_cast<double>(sourceTransform.q.y),
					static_cast<double>(sourceTransform.q.z),
					static_cast<double>(sourceTransform.q.w)
				)
			);
		}

		template <typename T>
		Eigen::Matrix<T, 4, 4> PxMat44ToEigenMatrix(const PxMat44T<T>& mat) {
			// Map the PxMat44 data directly to an Eigen Matrix4f
			return Eigen::Map<const Eigen::Matrix<T, 4, 4>>(&mat[0][0]);
		}

		template <typename T, int row, int col>
		Eigen::Matrix<T, row, col> MapToEigenMatrix(const float* mat)
		{
			return Eigen::Map<const Eigen::Matrix<T, row, col>>(mat);
		}

	};
}