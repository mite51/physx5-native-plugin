#include "PhysXWrapper.h"

#define PVD_HOST "127.0.0.1"	//Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.

namespace pxw
{
	PhysXWrapper::PhysXWrapper()
	{
		mFoundation = NULL;
		mPhysics = NULL;
		mDefaultMaterial = NULL;
		mScenes = PxArray<PxScene*>();
		mCudaContextManager = NULL;
		mDispatcher = NULL;
		mIsRunning = false;
		mIsPhysXInitialized = false;
		mStep = false;
		mPvd = NULL;
	}

	void PhysXWrapper::InitPhysX()
	{
		if (mIsPhysXInitialized == false)
		{
			mIsRunning = true;
			mIsPhysXInitialized = true;
			mStep = true;

			mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, mErrorCallback);

			//=====PVD Setup
			mPvd = PxCreatePvd(*mFoundation);
			mTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
			//mTransport = PxDefaultPvdFileTransportCreate("c:/temp/output.pxd2");

			// Create physics after PVD is connected
			mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, mPvd);
			
			// Initialize extensions (important for PVD)
			PxInitExtensions(*mPhysics, mPvd);

			mDispatcher = PxDefaultCpuDispatcherCreate(2);

			// Init CUDA
			if (PxGetSuggestedCudaDeviceOrdinal(mFoundation->getErrorCallback()) >= 0)
			{
				// initialize CUDA
				PxCudaContextManagerDesc cudaContextManagerDesc;
				mCudaContextManager = PxCreateCudaContextManager(*mFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
				if (mCudaContextManager && !mCudaContextManager->contextIsValid())
				{
					mCudaContextManager->release();
					mCudaContextManager = NULL;
				}
			}
			if (mCudaContextManager == NULL)
			{
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Failed to initialize CUDA!\n");
			}


			PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
			sceneDesc.cpuDispatcher = mDispatcher;
			sceneDesc.filterShader = PxDefaultSimulationFilterShader;
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "creating client!\n");
			PxScene* scene = mPhysics->createScene(sceneDesc);
			PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
			if (pvdClient)
			{
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}
			else
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "pvdClient missing!\n");
			}

			//
			mDefaultMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.0f);
		}
	}

	bool PhysXWrapper::GetPhysXInitStatus()
	{
		return mIsPhysXInitialized;
	}

	PxScene* PhysXWrapper::CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu)
	{

		if (mPvd && mTransport && mScenes.size() == 0)
		{
			if (mTransport && !mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eALL))
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "PVD connection failed!\n");
			}

			if (mPvd->isConnected())
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "PVD is connected!\n");
			}
			else
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "PVD is NOT connected!\n");
			}

		}

		PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
		sceneDesc.gravity = *gravity;
		sceneDesc.cpuDispatcher = mDispatcher;
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;

		sceneDesc.cudaContextManager = mCudaContextManager;
		sceneDesc.staticStructure = pruningStructureType;
		sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
		sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
		
		sceneDesc.solverType = solverType;
		if (useGpu)
		{
			// enable GPU dynamics and collision
			sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
			sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
		}
		else
		{
			sceneDesc.broadPhaseType = PxBroadPhaseType::eABP;
		}


		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "creating client!\n");
		PxScene* scene = mPhysics->createScene(sceneDesc);

		// Make sure PVD flags are set immediately after scene creation
		PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
		if(pvdClient)
		{
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "pvdClient missing!\n");
		}		

		mScenes.pushBack(scene);
		return scene; // Return the scene
	}

	void PhysXWrapper::StepPhysics(PxReal dt)
	{
		if (mIsRunning)
		{
			if (mStep)
			{
				for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
					(*it)->simulate(dt);
				}
			}

			for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
				(*it)->fetchResults(true);
				(*it)->fetchResultsParticleSystem();
			}

			// Flush PVD transport
			if (mTransport)
			{
				mTransport->flush();
			}

			mStep = true;
		}
	}

	void PhysXWrapper::StepPhysicsStart(PxReal dt)
	{
		if (mIsRunning && mStep)
		{
			mStep = false;

			for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
				(*it)->simulate(dt);
			}

			// Flush PVD transport
			if (mTransport)
			{
				mTransport->flush();
			}			
		}
	}

	void PhysXWrapper::StepPhysicsFetchResults()
	{
		if (mIsRunning && !mStep)
		{
			for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
				(*it)->fetchResults(true);
				(*it)->fetchResultsParticleSystem();
			}
			mStep = true;
		}
	}

	void PhysXWrapper::StepScene(PxScene* scene, PxReal dt)
	{
		if (mIsRunning && mStep)
		{
			mStep = false;
			scene->simulate(dt);
			scene->fetchResults(true);
			scene->fetchResultsParticleSystem();
			mStep = true;
		}
	}

	void PhysXWrapper::ReleaseScene(PxScene* scene)
	{
		if (mScenes.size() > 1)
		{
			mScenes.findAndReplaceWithLast(scene);
		}
		else
		{
			// TODO: this assumes that scene is valid!
			mScenes.clear();
		}
		scene->release();

		if (mScenes.size() == 0)
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "PVD disconnected!\n");
			mPvd->disconnect();
		}
	}

	void PhysXWrapper::CleanupPhysX()
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "CleanupPhysX!\n");
		// First, disconnect PVD to ensure all data is flushed
		if (mPvd)
		{
			mPvd->disconnect();
			mPvd->release();
			if (mTransport)
			{
				mTransport->flush();
				PX_RELEASE(mTransport);
			}
		}

		// Release all scenes
		for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
			(*it)->release();
		}
		mScenes.reset();

		PX_RELEASE(mDispatcher);
		PxCloseExtensions();
		PX_RELEASE(mPhysics);
		PX_RELEASE(mFoundation);
		mIsPhysXInitialized = false;
	}

	void PhysXWrapper::AddActorToScene(PxScene* scene, PxActor* actor)
	{
		scene->addActor(*actor);
	}

	void PhysXWrapper::RemoveActorFromScene(PxScene* scene, PxActor* actor)
	{
		scene->removeActor(*actor);
	}

	PxShape* PhysXWrapper::CreateShape(PxGeometry* geometry, PxMaterial* material, bool isExclusive)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create shape\n");
		return mPhysics->createShape(*geometry, *material, isExclusive);
	}
/*
	void PhysXWrapper::TestPVD()
	{
		// Create a scene with gravity
		PxVec3 gravity(0.0f, -9.81f, 0.0f);
		PxScene* scene = CreateScene(&gravity, PxPruningStructureType::eNONE, PxSolverType::ePGS, false);

		// Create ground plane
		PxPlane plane(PxVec3(0,1,0), 0);
		PxRigidStatic* groundPlane = PxCreatePlane(*mPhysics, plane, *mDefaultMaterial);
		AddActorToScene(scene, groundPlane);

		// Create a dynamic box
		PxTransform boxPose(PxVec3(0.0f, 10.0f, 0.0f));
		PxRigidDynamic* box = mPhysics->createRigidDynamic(boxPose);
		PxBoxGeometry boxGeom(1.0f, 1.0f, 1.0f);
		PxShape* boxShape = CreateShape(&boxGeom, mDefaultMaterial, true);
		box->attachShape(*boxShape);
		AddActorToScene(scene, box);

		// Simulate for a few steps
		for(int i = 0; i < 100; i++)
		{
			StepPhysics(1.0f/60.0f);
		}

		// Cleanup
		boxShape->release();
		CleanupPhysX();
	}
*/	
}