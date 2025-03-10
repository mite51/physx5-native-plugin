#include "Robotics.h"

namespace pxw
{
	void PxwArticulationKinematicTree::Initialize()
	{
		mLinkPoses = new PxwTransformData[mLinks.size()];
		// Add to scene then remove, for recording the initial data
		mScene->addArticulation(*mArticulation);
		mArticulationCache = mArticulation->createCache();
		mInitialArticulationCache = mArticulation->createCache();
		mArticulation->copyInternalStateToCache(*mArticulationCache, PxArticulationCacheFlag::eALL);
		mArticulation->copyInternalStateToCache(*mInitialArticulationCache, PxArticulationCacheFlag::eALL | PxArticulationCacheFlag::eFORCE | PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
		mScene->removeArticulation(*mArticulation);
		mInitialized = true;
	}

	void PxwArticulationKinematicTree::AddToScene()
	{
		if (!mInitialized) Initialize();
		mScene->addArticulation(*mArticulation);
	}

	void PxwArticulationKinematicTree::RemoveFromScene()
	{
		mScene->removeArticulation(*mArticulation);
	}

	PxArticulationLink* PxwArticulationKinematicTree::CreateBase(PxwTransformData basePose, PxShape* shape, float density)
	{
		PxArticulationLink* base = mArticulation->createLink(NULL, basePose.ToPxTransform());
		if (shape != NULL)
		{
			base->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*base, density);
		}
		else
		{
			// Assign a default small mass and inertia tensor
			// TODO: customize mass and inertia
			base->setMass(1.f);
			base->setMassSpaceInertiaTensor(PxVec3(1.f));
		}
		mLinks.push_back(base);
		return base;
	}

	PxArticulationLink* PxwArticulationKinematicTree::AddLink(PxArticulationLink* parentLink, PxwTransformData linkPose, PxwRobotJointType::Enum type, PxwTransformData jointPoseParent, PxwTransformData jointPoseChild, PxArticulationAxis::Enum dofAxis, PxShape* shape, float jointLimLower, float jointLimUpper, bool isDriveJoint, float stiffness, float damping, float driveMaxForce, float density)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add link\n");

		PxArticulationLink* link = mArticulation->createLink(parentLink, linkPose.ToPxTransform());
		// Remember that triangle mesh is not allowed for rigid body collision. External check before calling AddLink is required.
		if (shape != NULL)
		{
			// Remember that triangle mesh is not allowed for rigid body collision. External check before calling AddLink is required.
			link->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*link, density);
			float mass = link->getMass();
			std::string str = std::to_string(mass);
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, str.c_str());
		}
		else
		{
			// Assign a default small mass and inertia tensor
			// TODO: customize mass and inertia
			link->setMass(density);
			link->setMassSpaceInertiaTensor(PxVec3(density));
		}
		mLinks.push_back(link);

		PxArticulationJointReducedCoordinate* joint;

		//Set up the drive joint	
		joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
		joint->setJointType(PxwRobotJointType::ToPxArticulationJointType(type));
		if (type == PxwRobotJointType::ePRISMATIC)
		{
			joint->setMotion(dofAxis, PxArticulationMotion::eLIMITED);
			joint->setLimitParams(dofAxis, PxArticulationLimit(jointLimLower, jointLimUpper));
			if (isDriveJoint) joint->setDriveParams(dofAxis, PxArticulationDrive(stiffness, damping, driveMaxForce));
		}
		else if (type == PxwRobotJointType::eREVOLUTE)
		{
			joint->setMotion(dofAxis, PxArticulationMotion::eLIMITED);
			joint->setLimitParams(dofAxis, PxArticulationLimit(jointLimLower, jointLimUpper));
			if (isDriveJoint) joint->setDriveParams(dofAxis, PxArticulationDrive(stiffness, damping, driveMaxForce));
		}
		else if (type == PxwRobotJointType::eSPHERICAL)
		{
			joint->setMotion(dofAxis, PxArticulationMotion::eLIMITED);
			joint->setLimitParams(dofAxis, PxArticulationLimit(jointLimLower, jointLimUpper));
			if (isDriveJoint) joint->setDriveParams(dofAxis, PxArticulationDrive(stiffness, damping, driveMaxForce));
		}

		joint->setParentPose(jointPoseParent.ToPxTransform());
		joint->setChildPose(jointPoseChild.ToPxTransform());

		if (type != PxwRobotJointType::eFIX && isDriveJoint)
		{
			mDriveJoints.push_back(std::make_pair(joint, dofAxis));
			mNbDriveJoints++;
		}

		return link;
	}

	float* PxwArticulationKinematicTree::GetJointPositions()
	{
		mArticulation->copyInternalStateToCache(*mArticulationCache, PxArticulationCacheFlag::ePOSITION);
		return mArticulationCache->jointPosition;
	}

	PxwTransformData* PxwArticulationKinematicTree::GetLinkPoses()
	{
		for (int i = 0; i < mLinks.size(); i++)
		{
			mLinkPoses[i] = PxwTransformData(mLinks[i]->getGlobalPose());
		}
		return mLinkPoses;
	}

	void PxwArticulationKinematicTree::DriveJoints(float* targetJointPositions)
	{
		for (int i = 0; i < mNbDriveJoints; i++)
		{
			mDriveJoints[i].first->setDriveTarget(mDriveJoints[i].second, targetJointPositions[i]);
		}
	}

	void PxwArticulationKinematicTree::ResetObject()
	{
		if (mInitialArticulationCache)
		{
			mArticulation->applyCache(*mInitialArticulationCache, PxArticulationCacheFlag::eALL | PxArticulationCacheFlag::eFORCE | PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
			// Drive targets should be reset
			for (int i = 0; i < mNbDriveJoints; i++)
			{
				mDriveJoints[i].first->setDriveTarget(mDriveJoints[i].second, 0.0f);
			}
		}
	}

	void PxwArticulationKinematicTree::Release()
	{
		//mScene->removeArticulation(*mArticulation);
		mLinks.clear();
		mDriveJoints.clear();
		mArticulation->release();
		mArticulationCache->release();
		mInitialArticulationCache->release();
		delete mLinkPoses;
	}

	void PxwArticulationRobot::Initialize()
	{
		PxwArticulationKinematicTree::Initialize();

		if (mBodyLinkJoints.size() > 0)
		{
			// Base to joint 1
			PxTransform b2j1 = mBodyLinkJoints[0]->getParentPose();
			mJointToJointTransforms.push_back(PxMat44ToEigenMatrix<float>(b2j1));

			// Calculate joint-to-joint transformation
			for (int i = 0; i < mBodyLinkJoints.size() - 1; i++)
			{
				PxTransform poseJointOnLink = mBodyLinkJoints[i]->getChildPose();
				PxTransform poseNextJointOnLink = mBodyLinkJoints[i + 1]->getParentPose();
				PxTransform posej2j = poseJointOnLink.transformInv(poseNextJointOnLink);
				mJointToJointTransforms.push_back(PxMat44ToEigenMatrix<float>(posej2j));
				mSerialLinkJoints.push_back(mBodyLinkJoints[i]);
			}
			mSerialLinkJoints.push_back(mBodyLinkJoints[mBodyLinkJoints.size() - 1]);

			if (mEEJoints.size() > 0)
			{
				// Add only the last EE link's joint
				PxTransform poseJointOnLink = mBodyLinkJoints[mBodyLinkJoints.size() - 1]->getChildPose();
				PxTransform poseNextJointOnLink = mEEJoints[mEEJoints.size() - 1]->getParentPose();
				PxTransform posej2j = poseJointOnLink.transformInv(poseNextJointOnLink);
				mJointToJointTransforms.push_back(PxMat44ToEigenMatrix<float>(posej2j));
				mSerialLinkJoints.push_back(mEEJoints[mEEJoints.size() - 1]);
			}
		}

		Jn.resize(mSerialLinkJoints.size(), 6);
	}

	void PxwArticulationRobot::CreateBase(PxwTransformData basePose, float density)
	{
		// Robot with fixed base does not need a geometry
		PxArticulationLink* base = PxwArticulationKinematicTree::CreateBase(basePose, NULL, density);
		mBodyLinks.push_back(base);
		mBasePose = PxMat44ToEigenMatrix<float>(PxMat44(basePose.ToPxTransform()));
	}

	PxArticulationLink* PxwArticulationRobot::AddBodyLink(
		PxwTransformData linkPose,
		PxwRobotJointType::Enum type,
		PxwTransformData jointPoseParent,
		PxwTransformData jointPoseChild,
		PxShape* shape,
		float jointLimLower,
		float jointLimUpper,
		float stiffness,
		float damping,
		float driveMaxForce,
		float density
	) {
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add robot link\n");
		PxArticulationLink* parentLink = mBodyLinks.back();
		PxArticulationAxis::Enum dofAxis;
		bool isDriveJoint = true;
		if (type == PxwRobotJointType::ePRISMATIC)
		{
			dofAxis = PxArticulationAxis::eY;
		}
		else if (type == PxwRobotJointType::eREVOLUTE)
		{
			dofAxis = PxArticulationAxis::eSWING1;
		}
		else if (type == PxwRobotJointType::eSPHERICAL)
		{
			//needs to do all 3 axis
		}
		else
		{
			isDriveJoint = false;
		}
		PxArticulationLink* link = PxwArticulationKinematicTree::AddLink(parentLink, linkPose, type, jointPoseParent, jointPoseChild, dofAxis,
			shape, jointLimLower, jointLimUpper, isDriveJoint, stiffness, damping, driveMaxForce, density);

		mBodyLinks.push_back(link);

		if (isDriveJoint)
		{
			// add the drive joint to the list
			PxArticulationJointReducedCoordinate* driveJoint;
			driveJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
			mBodyLinkJoints.push_back(driveJoint);
		}

		return link;
	}

	PxArticulationLink* PxwArticulationRobot::AddEndEffectorLink(
		PxwTransformData linkPose,
		PxwRobotJointType::Enum type,
		PxwTransformData jointPoseParent,
		PxwTransformData jointPoseChild,
		PxShape* shape,
		float jointLimLower,
		float jointLimUpper,
		float stiffness,
		float damping,
		float driveMaxForce,
		float density
	) {
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add robot EE link\n");
		PxArticulationLink* parentLink = mBodyLinks.back();

		PxArticulationAxis::Enum dofAxis;
		bool isDriveJoint = true;
		if (type == PxwRobotJointType::ePRISMATIC)
		{
			dofAxis = PxArticulationAxis::eY;
		}
		else if (type == PxwRobotJointType::eREVOLUTE)
		{
			dofAxis = PxArticulationAxis::eSWING1;
		}
		else
		{
			isDriveJoint = false;
		}
		PxArticulationLink* link = PxwArticulationKinematicTree::AddLink(parentLink, linkPose, type, jointPoseParent, jointPoseChild, dofAxis,
			shape, jointLimLower, jointLimUpper, isDriveJoint, stiffness, damping, driveMaxForce, density);

		mEELinks.push_back(link);

		if (isDriveJoint)
		{
			// add the drive joint to the list
			PxArticulationJointReducedCoordinate* driveJoint;
			driveJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
			mEEJoints.push_back(driveJoint);
		}		
		return link;
	}

	PxwSpatialForceData PxwArticulationRobot::GetLinkIncomingJointForce(int n)
	{
		if (n >= mBodyLinks.size()) return PxwSpatialForceData();
		mArticulation->copyInternalStateToCache(*mArticulationCache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
		PxwSpatialForceData spatialForceData;
		
		spatialForceData.force = mArticulationCache->linkIncomingJointForce[n].force;
		spatialForceData.torque = mArticulationCache->linkIncomingJointForce[n].torque;
		return spatialForceData;
	}

	Eigen::Matrix4f PxwArticulationRobot::ForwardKinematics(float* q)
	{
		Eigen::Matrix4f cumulativeTransform = mBasePose; // Assuming mBasePose is already an Eigen::Matrix4f
		for (size_t i = 0; i < mSerialLinkJoints.size(); ++i) {
			Eigen::Matrix4f localTransform = Eigen::Matrix4f::Identity(); // Initialize as identity matrix

			// Calculate local transform
			if (mSerialLinkJoints[i]->getJointType() == PxArticulationJointType::eREVOLUTE)
			{
				float angle = q[i];
				Eigen::Vector3f axis(0, 1, 0); // Y-axis
				Eigen::Matrix3f rotationMatrix = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
				localTransform.block<3, 3>(0, 0) = rotationMatrix; // Set rotation part
			}
			else if (mSerialLinkJoints[i]->getJointType() == PxArticulationJointType::ePRISMATIC)
			{
				// Prismatic joint: translate along joint axis by q[i]
				localTransform.block<3, 1>(0, 3) = Eigen::Vector3f(0, q[i], 0); // Set translation part
			}
			cumulativeTransform *= mJointToJointTransforms[i] * localTransform; // Accumulate transformation
		}

		return cumulativeTransform;
	}

	Eigen::Matrix4f PxwArticulationRobot::ForwardKinematicsJoint(float q, int n)
	{
		Eigen::Matrix4f localTransform = Eigen::Matrix4f::Identity(); // Initialize with identity matrix

		// Calculate local transform
		if (mSerialLinkJoints[n]->getJointType() == PxArticulationJointType::eREVOLUTE)
		{
			float angle = q;
			Eigen::Vector3f axis(0, 1, 0); // Y-axis
			Eigen::Matrix3f rotationMatrix = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
			localTransform.block<3, 3>(0, 0) = rotationMatrix; // Set rotation part
		}
		else if (mSerialLinkJoints[n]->getJointType() == PxArticulationJointType::ePRISMATIC)
		{
			// Prismatic joint: translate along joint axis by q[i]
			localTransform.block<3, 1>(0, 3) = Eigen::Vector3f(0, q, 0); // Set translation part
		}
		localTransform = mJointToJointTransforms[n] * localTransform;

		return localTransform;
	}

	Eigen::MatrixXf PxwArticulationRobot::ComputeJacobianBody(float* q)
	{
		/*
		* Modified from cisstRobot:
		* https://github.com/jhu-cisst/cisst/blob/6b0f23f9f1d2ef70755b52505b578491c6911f9f/cisstRobot/code/robManipulator.cpp#L747
		*/
		Eigen::Matrix4f U  = Eigen::Matrix4f::Identity();  // set identity

		for (int j = mSerialLinkJoints.size() - 1; 0 <= j; j--) {

			if (mSerialLinkJoints[j]->getJointType() == PxArticulationJointType::eREVOLUTE)\
			{
				// Revolute joint

				// Add a minus sign due to left-handed coordinate system
				Jn(0, j) = U(2, 3) * U(0, 0) - U(0, 3) * U(2, 0);
				Jn(1, j) = U(2, 3) * U(0, 1) - U(0, 3) * U(2, 1);
				Jn(2, j) = U(2, 3) * U(0, 2) - U(0, 3) * U(2, 2);

				Jn(3, j) = U(1, 0); // ny
				Jn(4, j) = U(1, 1); // oy
				Jn(5, j) = U(1, 2); // ay

			}

			if (mSerialLinkJoints[j]->getJointType() == PxArticulationJointType::ePRISMATIC) {   // Prismatic joint
				Jn(0, j) = U(1, 0); // ny
				Jn(1, j) = U(1, 1); // oy
				Jn(2, j) = U(1, 2); // ay

				Jn(3, j) = 0.0;
				Jn(4, j) = 0.0;
				Jn(5, j) = 0.0;
			}
			U = ForwardKinematicsJoint(q[j], j) * U;
		}
		return Jn;
	}

	Eigen::MatrixXf PxwArticulationRobot::ComputeJacobianSpatial(float* q)
	{
		ComputeJacobianBody(q);

		// Get the adjoint matrix to flip the body jacobian to spatial jacobian
		Eigen::Matrix4f Rt0n = ForwardKinematics(q);
		Eigen::Matrix<float, 6, 6> Ad;

		// Extract rotation matrix R and translation vector p from Rt0n
		Eigen::Matrix3f R = Rt0n.block<3, 3>(0, 0);
		Eigen::Vector3f p = Rt0n.block<3, 1>(0, 3);

		// Compute the skew-symmetric matrix of p
		Eigen::Matrix3f p_skew;
		p_skew << 0, -p.z(), p.y(),
			p.z(), 0, -p.x(),
			-p.y(), p.x(), 0;

		// Build the adjoint matrix
		// upper left block is the rotation matrix
		Ad.block<3, 3>(0, 0) = R;

		// upper right block is zero
		Ad.block<3, 3>(0, 3).setZero();

		// lower left block is the product of skew-symmetric matrix of p and R
		Ad.block<3, 3>(3, 0) = -p_skew * R;

		// lower right block is the rotation matrix
		Ad.block<3, 3>(3, 3) = R;

		Js = Ad * Jn;

		return Js;
	}

	bool PxwArticulationRobot::InverseKinematics(float* qInit, PxwTransformData targetTransformEEJoint, float tolerance, int numIterations, float lambda)
	{
		// The number of columns
		const int N = mSerialLinkJoints.size();

		Eigen::Matrix4f Rts = targetTransformEEJoint.ToEigenMatrix4();
		// e is the error vector
		Eigen::Vector<float, 6> e;

		// Squared tolerance
		float toleranceSq = tolerance * tolerance;

		// Squred norm of the iteration error
		float ndqSq = 1;

		// Iteration counter
		size_t i;

		// dq at each step
		float* dq = new float[N];

		// loop until Niter are executed or the error is bellow the tolerance
		for (i = 0; i < numIterations && toleranceSq < ndqSq; i++) {

			// Evaluate the forward kinematics
			Eigen::Matrix4f Rt = ForwardKinematics(qInit);
			// Evaluate the spatial Jacobian (also evaluate the forward kin)
			ComputeJacobianSpatial(qInit);

			// compute the translation error
			Eigen::Vector3f dt(
				Rts(0, 3) - Rt(0, 3),
				Rts(1, 3) - Rt(1, 3),
				Rts(2, 3) - Rt(2, 3)
			);

			// compute the orientation error
			// first build the [ n o a ] vectors
			Eigen::Vector3f n1(Rt(0, 0), Rt(1, 0), Rt(2, 0));
			Eigen::Vector3f o1(Rt(0, 1), Rt(1, 1), Rt(2, 1));
			Eigen::Vector3f a1(Rt(0, 2), Rt(1, 2), Rt(2, 2));
			Eigen::Vector3f n2(Rts(0, 0), Rts(1, 0), Rts(2, 0));
			Eigen::Vector3f o2(Rts(0, 1), Rts(1, 1), Rts(2, 1));
			Eigen::Vector3f a2(Rts(0, 2), Rts(1, 2), Rts(2, 2));

			// This is the orientation error
			Eigen::Vector3f dr = 0.5 * ((n1.cross(n2)) + (o1.cross(o2)) + (a1.cross(a2)));

			// combine both errors in one R^6 vector
			e = Eigen::Vector<float, 6>(dt[0], dt[1], dt[2], dr[0], dr[1], dr[2]);

			// We need to solve dq = J' ( JJ' + lambda I )^-1 e

			Matrix<float, 6, 6> JJt = Js * Js.transpose();
			Matrix<float, 6, 6> I = Matrix<float, 6, 6>::Identity(6, 6);
			Matrix<float, 6, 6> JJt_lambdaI = JJt + lambda * I;

			// Solve the linear system (JJ' + lambda * I) x = e
			VectorXf x = JJt_lambdaI.ldlt().solve(e);

			// Compute dq = J' * x
			VectorXf dq = Js.transpose() * x;

			// Update ndq
			ndqSq = dq.squaredNorm();

			// update the solution
			for (size_t j = 0; j < N; j++) qInit[j] += dq(j);
		}

		NormalizeAngles(qInit);

		delete[] dq;

		if (i == numIterations) {
			return false;
		}

		return true;
	}

	void PxwArticulationRobot::NormalizeAngles(float* jointPositions)
	{
		for (int i = 0; i < mSerialLinkJoints.size(); i++)
		{
			if (mSerialLinkJoints[i]->getJointType() == PxArticulationJointType::eREVOLUTE)
			{
				jointPositions[i] = fmod(jointPositions[i], 2.0 * PxPi);
				if (PxPi < jointPositions[i]) {
					jointPositions[i] = jointPositions[i] - 2.0 * PxPi;
				}
				else if (jointPositions[i] < -PxPi) {
					jointPositions[i] = jointPositions[i] + 2.0 * PxPi;
				}
			}
		}
	}

	void PxwArticulationRobot::Release()
	{
		PxwArticulationKinematicTree::Release();
		mBodyLinks.clear();
		mEELinks.clear();
		mBodyLinkJoints.clear();
		mEEJoints.clear();
		mSerialLinkJoints.clear();
	}
}