#include "PhysXWrapper.h"
#include "ArticulationContacts.h"
#include "VehicleModule.h"

#define PVD_HOST "127.0.0.1"	//Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.

// Targets PHYSX 5.4.2.9950ad0d

namespace pxw
{
	// The default filter shader with contact and trigger notifications ORed on. It changes
	// no collision or solve decision: for a solid pair it delegates to the default and only
	// adds report flags, and for a trigger pair it applies the standard trigger behaviour.
	// A scene built with this therefore simulates identically to one built with
	// PxDefaultSimulationFilterShader, which is what lets contact reporting be added to the
	// deterministic runtime without moving the measured numbers. Reporting only happens when
	// a scene also has a simulation event callback set; the flags alone are inert.
	static PxFilterFlags UndpwrEventFilterShader(
		PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			// Triggers have no contact points to solve or report; the default trigger
			// behaviour already reports found and lost.
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlag::eDEFAULT;
		}

		const PxFilterFlags flags = PxDefaultSimulationFilterShader(
			attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND
			| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
			| PxPairFlag::eNOTIFY_CONTACT_POINTS;
		return flags;
	}

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

			// 0 worker threads -> solver/contact tasks run on the calling (Unity main)
			// thread, matching OVProtomotionsCpp (PxDefaultCpuDispatcherCreate(0)).
			// PhysX results are only reproducible for a fixed thread count: with >1
			// worker the constraint partitioning and floating-point reduction order
			// change, which perturbs the contact impulse at stiff footstrike contacts
			// (instantaneous ankle/toe dof.vel kicks) even though positions barely move.
			mDispatcher = (PxDefaultCpuDispatcher*)GetOrCreateDispatcher(0);

#ifdef USE_GPU
			// Init CUDA
			if (PxGetSuggestedCudaDeviceOrdinal(mFoundation->getErrorCallback()) >= 0)
			{
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
#else
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "GPU support disabled (CPU-only build).\n");
#endif

			mDefaultMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.0f);

			// Initialise the PhysX Vehicle2 extension and shared sweep mesh.
			VehicleInit(mFoundation, mPhysics);
		}
	}

	bool PhysXWrapper::GetPhysXInitStatus()
	{
		return mIsPhysXInitialized;
	}

	PxCpuDispatcher* PhysXWrapper::GetOrCreateDispatcher(int workerThreads)
	{
		if (workerThreads < 0)
		{
			workerThreads = 0;
		}

		for (size_t i = 0; i < mDispatchers.size(); ++i)
		{
			if (mDispatchers[i].first == workerThreads)
			{
				return mDispatchers[i].second;
			}
		}

		PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate((PxU32)workerThreads);
		mDispatchers.push_back(std::make_pair(workerThreads, dispatcher));
		return dispatcher;
	}

	void PhysXWrapper::TryConnectPvd()
	{
		if (mPvdConnectAttempted || mPvd == NULL || mTransport == NULL)
		{
			return;
		}
		mPvdConnectAttempted = true;

		if (!mPvd->connect(*mTransport, PxPvdInstrumentationFlag::eALL))
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

	PxScene* PhysXWrapper::CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu, PxU32 extraFlags)
	{
		// Preserve the historical defaults exactly so existing callers are unaffected.
		PxwSceneDesc desc;
		desc.gravity = *gravity;
		desc.flags = PxwSceneFlag::eENABLE_PCM | extraFlags;
		desc.pruningStructureType = (PxI32)pruningStructureType;
		desc.solverType = (PxI32)solverType;
		desc.broadPhaseType = -1;
		desc.cpuWorkerThreads = 0;
		desc.useGpu = useGpu ? 1 : 0;
		desc.bounceThresholdVelocity = 0.2f;
		desc.frictionOffsetThreshold = 0.04f;
		desc.ccdMaxPasses = 1;

		if (useGpu)
		{
			desc.flags |= PxwSceneFlag::eENABLE_DIRECT_GPU_API;
		}

		return CreateSceneEx(desc);
	}

	PxScene* PhysXWrapper::CreateSceneEx(const PxwSceneDesc& desc)
	{
		const bool useGpu = desc.useGpu != 0;
		const bool pvdDisabled = (desc.flags & PxwSceneFlag::eDISABLE_PVD) != 0;

		if (!pvdDisabled)
		{
			TryConnectPvd();
		}

		PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
		sceneDesc.gravity = desc.gravity;
		sceneDesc.cpuDispatcher = GetOrCreateDispatcher(desc.cpuWorkerThreads);
		sceneDesc.filterShader = (desc.flags & PxwSceneFlag::eENABLE_CONTACT_EVENTS)
			? UndpwrEventFilterShader
			: PxDefaultSimulationFilterShader;
		sceneDesc.staticStructure = (PxPruningStructureType::Enum)desc.pruningStructureType;
		sceneDesc.solverType = (PxSolverType::Enum)desc.solverType;
		sceneDesc.bounceThresholdVelocity = desc.bounceThresholdVelocity;
		sceneDesc.frictionOffsetThreshold = desc.frictionOffsetThreshold;
		sceneDesc.ccdMaxPasses = desc.ccdMaxPasses;

		if (desc.flags & PxwSceneFlag::eENABLE_PCM)           sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
		if (desc.flags & PxwSceneFlag::eENABLE_CCD)           sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
		if (desc.flags & PxwSceneFlag::eENABLE_STABILIZATION) sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
		if (desc.flags & PxwSceneFlag::eENABLE_ACTIVE_ACTORS) sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;

		if (desc.flags & PxwSceneFlag::eENABLE_ENHANCED_DETERMINISM)
		{
			// PhysX does not support enhanced determinism together with GPU dynamics.
			// Refuse the combination loudly rather than silently producing a scene that
			// cannot deliver the guarantee the caller asked for.
			if (useGpu)
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
					"Enhanced determinism is not supported with GPU dynamics; the flag was ignored for this scene.\n");
			}
			else
			{
				sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
			}
		}

		if (useGpu)
		{
			if (mCudaContextManager == NULL)
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
					"A GPU scene was requested but no CUDA context is available; falling back to CPU simulation.\n");
			}
			else
			{
				sceneDesc.cudaContextManager = mCudaContextManager;
				sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
				sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;

				// The direct GPU API bypasses the CPU-visible actor state, so
				// getGlobalPose and friends return stale data while it is on. Only
				// enable it when the caller has explicitly asked for it.
				if (desc.flags & PxwSceneFlag::eENABLE_DIRECT_GPU_API)
				{
					sceneDesc.flags |= PxSceneFlag::eENABLE_DIRECT_GPU_API;
				}
			}
		}

		if (desc.broadPhaseType >= 0)
		{
			sceneDesc.broadPhaseType = (PxBroadPhaseType::Enum)desc.broadPhaseType;
		}
		else if (sceneDesc.broadPhaseType != PxBroadPhaseType::eGPU)
		{
			sceneDesc.broadPhaseType = PxBroadPhaseType::eABP;
		}

		PxScene* scene = mPhysics->createScene(sceneDesc);
		if (scene == NULL)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "createScene returned null.\n");
			return NULL;
		}

		if (desc.flags & PxwSceneFlag::eENABLE_CONTACT_EVENTS)
		{
			// The notification flags the filter shader adds are inert without a callback,
			// so install the articulation contact tracker as the default consumer. A
			// caller that wants the events for itself -- the UNDPWR world layer does --
			// simply calls setSimulationEventCallback again after this returns.
			scene->setSimulationEventCallback(&GetArticulationContactTracker());
		}

		if (!pvdDisabled)
		{
			PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
			if (pvdClient)
			{
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
				pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
			}
		}

		mScenes.pushBack(scene);

		// Register a per-scene vehicle simulation context.
		VehicleRegisterScene(scene);

		return scene;
	}

	void PhysXWrapper::StepPhysics(PxReal dt)
	{
		if (mIsRunning)
		{
			if (mStep)
			{
				for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it) {
					VehicleStepScene(*it, dt);
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
				VehicleStepScene(*it, dt);
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

	// Steps a single scene without touching the global mStep gate used by the
	// step-all-scenes entry points. Independent scenes must be able to advance
	// independently, which is what lets several simulation worlds run side by side in
	// one process.
	void PhysXWrapper::StepScene(PxScene* scene, PxReal dt)
	{
		if (!mIsRunning || scene == NULL)
		{
			return;
		}

		VehicleStepScene(scene, dt);
		scene->simulate(dt);
		scene->fetchResults(true);
		scene->fetchResultsParticleSystem();
	}

	void PhysXWrapper::ReleaseScene(PxScene* scene)
	{
		// Drop the per-scene vehicle context before releasing the scene.
		VehicleUnregisterScene(scene);

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

		if (mScenes.size() == 0 && mPvd != NULL && mPvd->isConnected())
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "PVD disconnected!\n");
			mPvd->disconnect();
			mPvdConnectAttempted = false;
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

		// Tear down the vehicle extension before core extensions/physics.
		VehicleCleanup();

		for (size_t i = 0; i < mDispatchers.size(); ++i)
		{
			PX_RELEASE(mDispatchers[i].second);
		}
		mDispatchers.clear();
		mDispatcher = NULL;
		mPvdConnectAttempted = false;

		PxCloseExtensions();
		PX_RELEASE(mPhysics);
		PX_RELEASE(mFoundation);
		mIsPhysXInitialized = false;
	}

	void PhysXWrapper::FlushPVD()
	{
		/*
		for (PxArray<PxScene*>::ConstIterator it = mScenes.begin(); it != mScenes.end(); ++it)
		{
			PxPvdSceneClient* pvdClient = (*it)->getScenePvdClient();
			if (pvdClient)
			{
				pvdClient->updatePvdProperties();
			}
		}
		*/

		if (mTransport)
		{
			mTransport->flush();
		}
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