#include "PhysXWrapper.h"

using namespace pxw;

namespace pxw
{
	PxwArticulationKinematicTree* PhysXWrapper::CreatePxArticulationKinematicTree(PxScene* scene, bool fixBase, bool disableSelfCollision)
	{
		PxwArticulationKinematicTree* kinematicTree = new PxwArticulationKinematicTree(scene, mPhysics, fixBase, disableSelfCollision);
		return kinematicTree;
	}

	PxwArticulationRobot* PhysXWrapper::CreateArticulationRobot(PxScene* scene, PxwTransformData basePose, float density)
	{
		PxwArticulationRobot* robot = new PxwArticulationRobot(scene, mPhysics);
		robot->CreateBase(basePose, density);
		return robot;
	}
}