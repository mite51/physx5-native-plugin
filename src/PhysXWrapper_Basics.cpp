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
	}

	void PhysXWrapper::InitPhysX()
	{
		mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, mErrorCallback);
		mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, PxTolerancesScale(), true, NULL);
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
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Failed to initialize CUDA!\n");
		}
		mIsRunning = true;
		mIsPhysXInitialized = true;
		mStep = true;
		mDefaultMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.6f);
	}

	bool PhysXWrapper::GetPhysXInitStatus()
	{
		return mIsPhysXInitialized;
	}

	PxScene* PhysXWrapper::CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu)
	{
		//=====PVD
		mPvd = PxCreatePvd(*mFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
		mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

		PxInitExtensions(*mPhysics, mPvd);
		//=====PVD

		PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
		sceneDesc.gravity = *gravity;
		sceneDesc.cpuDispatcher = mDispatcher;
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;

		sceneDesc.cudaContextManager = mCudaContextManager;
		sceneDesc.staticStructure = pruningStructureType;
		sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
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

		PxScene* scene = NULL;
		scene = mPhysics->createScene(sceneDesc);

		//=====PVD
		PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
		if(pvdClient)
		{
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
			pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}	
		//=====PVD


		mScenes.pushBack(scene);
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create Scene\n");
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
	}

	void PhysXWrapper::CleanupPhysX()
	{
		// Release all scenes
		for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
			(*it)->release();
		}
		mScenes.reset(); // delete all and frees the memory

		//PX_RELEASE(mCudaContextManager);
		PX_RELEASE(mDispatcher);
		PX_RELEASE(mPhysics);
		PX_RELEASE(mFoundation);
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
}