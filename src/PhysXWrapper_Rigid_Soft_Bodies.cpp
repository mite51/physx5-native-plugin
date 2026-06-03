#include "PhysXWrapper.h"

using namespace pxw;

namespace pxw
{
	PxRigidDynamic*PhysXWrapper::CreateDynamicRigidActor(PxScene* /*scene*/, const PxwTransformData transform, PxShape *shape)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add dynamic rigid body\n");
		PxRigidDynamic* actor = mPhysics->createRigidDynamic(transform.ToPxTransform());
		actor->attachShape(*shape);
		return actor;
	}

	PxActor* PhysXWrapper::CreateKinematicRigidActor(PxScene* /*scene*/, PxwTransformData transform, PxShape* shape)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add kinematic rigid body\n");
		PxRigidDynamic* actor = mPhysics->createRigidDynamic(transform.ToPxTransform());
		actor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		actor->attachShape(*shape);
		return actor;
	}

	PxActor* PhysXWrapper::CreateStaticRigidActor(PxScene* /*scene*/, PxwTransformData transform, PxShape* shape)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add static rigid body\n");
		PxActor* actor = PxCreateStatic(*mPhysics, transform.ToPxTransform(), *shape);
		return actor;
	}

	/*
	* \brief Add an FEM soft body
	*/
	PxwSoftBodyHelper* PhysXWrapper::CreateFEMSoftBody(PxScene* scene, const PxU32 numVertices, const PxVec3* triVerts, const PxU32 numTriangles, const int* triIndices, PxwTransformData pose, PxDeformableVolumeMaterial* material, PxReal density, PxU32 iterationCount, bool useCollisionMeshForSimulation, PxU32 numVoxelsAlongLongestAABBAxis)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add soft body\n");

		PxTolerancesScale scale;
		PxCookingParams params(scale);
		SetupCommonCookingParams(params, false, false);
		params.buildGPUData = true;

		PxDeformableVolumeMesh* softBodyMesh;

		PxSimpleTriangleMesh surfaceMesh;
		surfaceMesh.points.count = numVertices;
		surfaceMesh.points.data = triVerts;
		surfaceMesh.triangles.count = numTriangles;
		surfaceMesh.triangles.data = triIndices;

		if (useCollisionMeshForSimulation)
		{
			softBodyMesh = PxDeformableVolumeExt::createDeformableVolumeMeshNoVoxels(params, surfaceMesh, mPhysics->getPhysicsInsertionCallback());
		}
		else
		{
			softBodyMesh = PxDeformableVolumeExt::createDeformableVolumeMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, mPhysics->getPhysicsInsertionCallback());
		}

		PX_ASSERT(softBodyMesh);

		if (!mCudaContextManager)
			return NULL;
		PxDeformableVolume* softBody = mPhysics->createDeformableVolume(*mCudaContextManager);
		PxwSoftBodyHelper* softBodyHelper = new PxwSoftBodyHelper(mCudaContextManager);

		if (softBody)
		{
			PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
			PxTetrahedronMeshGeometry geometry(softBodyMesh->getCollisionMesh());
			PxShape* shape = mPhysics->createShape(geometry, &material, 1, true, shapeFlags);
			if (shape)
			{
				softBody->attachShape(*shape);
				shape->release();
			}
			softBody->attachSimulationMesh(*softBodyMesh->getSimulationMesh(), *softBodyMesh->getDeformableVolumeAuxData());

			PxFEMParameters femParams;
			softBodyHelper->AttachSoftBodyToDevice(scene, softBody, femParams, pose.ToPxTransform(), density, 1.0f, iterationCount);
			softBody->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, false);
		}
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add soft body done\n");
		return softBodyHelper;
	}
}