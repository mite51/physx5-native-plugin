#pragma once

#include <utility>
#include <limits.h>
#include "PxPhysicsAPI.h"
// Not reachable through PxPhysicsAPI.h.
#include "geometry/PxConvexCoreGeometry.h"
#include "extensions/PxParticleExt.h"
#include <list>
#include <sstream>
#include "extensions/PxDeformableVolumeExt.h"

#ifdef USE_GPU
#include "cudamanager/PxCudaContext.h"
#include <gpu/PxPhysicsGpu.h>
#include <PxAnisotropy.h>
#include "ParticleSystemHelper.h"
#endif

#include "SoftBodyHelper.h"

#include <mutex>
#include <sstream>
#include <vector>

using namespace physx;
#ifdef USE_GPU
using namespace ExtGpu;
#endif
using namespace std;

namespace pxw {

	// Optional immediate log sink installed via PxwSetLogCallback. Declared here so
	// BufferedErrorCallback can forward diagnostics the moment they are reported
	// instead of waiting for the managed side to poll GetPhysxErrors.
	typedef void(*PxwLogSink)(int severity, const char* message);
	extern PxwLogSink gPxwLogSink;

	// Add this new class before PhysXWrapper
	class BufferedErrorCallback : public PxDefaultErrorCallback 
	{
	private:
		std::string mErrorBuffer;
		std::mutex mMutex;

	public:
		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override {
			std::string formatted;
			{
				std::lock_guard<std::mutex> lock(mMutex);
				// Format error message
				std::stringstream ss;
				ss << "PhysX Error [" << code << "] in " << file << ":" << line << " - " << message << "\n";
				formatted = ss.str();
				mErrorBuffer += formatted;
			}

			// Forward immediately when a sink is installed. Done outside the lock so a
			// callback that logs back into the plugin cannot deadlock.
			PxwLogSink sink = gPxwLogSink;
			if (sink != NULL)
			{
				int severity;
				switch (code)
				{
				case PxErrorCode::eDEBUG_INFO:     severity = 1; break;
				case PxErrorCode::eDEBUG_WARNING:
				case PxErrorCode::ePERF_WARNING:   severity = 2; break;
				default:                           severity = 3; break;
				}
				sink(severity, formatted.c_str());
			}

			// Also call parent implementation for default console output
			PxDefaultErrorCallback::reportError(code, message, file, line);
		}

		std::string getAndClearErrors() {
			std::lock_guard<std::mutex> lock(mMutex);
			std::string errors = mErrorBuffer;
			mErrorBuffer.clear();
			return errors;
		}
	};

	class PhysXWrapper
	{
	private:
		PxDefaultAllocator mAllocator;
		BufferedErrorCallback mErrorCallback;
		PxFoundation* mFoundation;
		PxPhysics* mPhysics;
		PxDefaultCpuDispatcher* mDispatcher;
		PxMaterial* mDefaultMaterial;
		bool mIsRunning;
		bool mIsPhysXInitialized;
		bool mStep;
		PxCudaContextManager* mCudaContextManager;
		PxArray<PxScene*> mScenes;

		PxPvd* mPvd = nullptr;
		PxPvdTransport* mTransport = nullptr;

		// PhysX only reproduces results for a fixed worker count, so dispatchers are
		// cached per requested thread count rather than shared globally.
		std::vector<std::pair<int, PxDefaultCpuDispatcher*>> mDispatchers;

		// Set once the first scene has asked for a PVD connection. Scenes created with
		// PxwSceneFlag::eDISABLE_PVD never trigger the connection attempt, which keeps
		// headless test runs free of socket timeouts.
		bool mPvdConnectAttempted = false;

		static void SetupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData);

		PxCpuDispatcher* GetOrCreateDispatcher(int workerThreads);

		void TryConnectPvd();

	public:
		PhysXWrapper();
		
		PxPhysics* GetPhysics()
		{
			return mPhysics;
		}

		// PhysX basics
		void InitPhysX();

		bool GetPhysXInitStatus();

		// extraFlags is ORed onto the historical default flags, so a caller can opt into a
		// single capability -- contact reporting, in practice -- without the whole
		// PxwSceneDesc crossing an interop boundary. Zero reproduces the original behaviour.
		PxScene* CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu, PxU32 extraFlags = 0);

		// Fully explicit scene creation. CreateScene above is a thin shim over this so
		// existing callers keep their previous behaviour.
		PxScene* CreateSceneEx(const PxwSceneDesc& desc);

		PxMaterial* GetDefaultMaterial() { return mDefaultMaterial; }

		bool HasCudaContext() const { return mCudaContextManager != NULL; }

		void StepPhysics(PxReal dt);

		void StepPhysicsStart(PxReal dt);

		void StepPhysicsFetchResults();

		void StepScene(PxScene* scene, PxReal dt);

		void ReleaseScene(PxScene* scene);

		void CleanupPhysX();

		void FlushPVD();

		// PxActor basics

		void AddActorToScene(PxScene* scene, PxActor* actor);

		void RemoveActorFromScene(PxScene* scene, PxActor* actor);

		PxShape* CreateShape(PxGeometry* geometry, PxMaterial* material, bool isExclusive);

#ifdef USE_GPU
		// Particle system (GPU only)
		PxwPBDParticleSystemHelper* CreatePBDParticleSystem(PxScene* scene, const PxReal particleSpacing = 0.2f, int maxNumParticlesForAnisotropy = 0);

		void ReleasePBDParticleSystem(PxScene* scene, PxPBDParticleSystem* particleSystem);

		PxwPBDBoxFluid* CreateCubeFluid(
			PxScene* scene,
			PxwPBDParticleSystemHelper* particleSystem,
			PxPBDMaterial* material,
			const PxU32 numX,
			const PxU32 numY,
			const PxU32 numZ,
			const PxVec3& position = PxVec3(0, 0, 0),
			const PxReal particleSpacing = 0.2f,
			const PxReal fluidDensity = 1000.f,
			const PxU32 maxDiffuseParticles = 100000,
			const PxReal buoyancy = 0.9f
		);

		PxwPBDFluid* CreateFluid(
			PxScene* scene,
			PxwPBDParticleSystemHelper* particleSystem,
			PxPBDMaterial* material,
			PxVec4* positions,
			const PxU32 numParticles,
			const PxReal particleSpacing = 0.2f,
			const PxReal fluidDensity = 1000.f,
			const PxU32 maxDiffuseParticles = 100000,
			const PxReal buoyancy = 0.9f
		);

		PxwPBDTriMeshCloth* CreateTriMeshCloth(
			PxScene* scene,
			PxwPBDParticleSystemHelper* particleSystem,
			PxPBDMaterial* material,
			PxVec3* vertices,
			const int numVertices,
			int* indices,
			const int numIndices,
			const PxVec3 position,
			const PxReal totalMass,
			const bool inflatable,
			const PxReal blendScale,
			const PxReal pressure,
			const PxReal particleSpacing = 0.2f
		);
#endif

		// Rigid and Soft Bodies

		PxRigidDynamic* CreateDynamicRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxActor* CreateKinematicRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxActor* CreateStaticRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxwSoftBodyHelper* CreateFEMSoftBody(PxScene* scene, const PxU32 numVertices, const PxVec3* triVerts, const PxU32 numTriangles, const int* triIndices, PxwTransformData pose, PxDeformableVolumeMaterial* material, PxReal density, PxU32 iterationCount, bool useCollisionMeshForSimulation = false, PxU32 numVoxelsAlongLongestAABBAxis = 8);

		// Utility functions

		PxTriangleMesh* CreateBV33TriangleMesh(
			PxU32 numVertices,
			const PxVec3* vertices,
			PxU32 numTriangles,
			const PxU32* indices,
			bool skipMeshCleanup,
			bool skipEdgeData,
			bool inserted,
			bool cookingPerformance,
			bool meshSizePerfTradeoff,
			bool buildGpuData,
			PxReal sdfSpacing = 0.f,
			PxU32 sdfSubgridSize = 6,
			PxSdfBitsPerSubgridPixel::Enum bitsPerSdfSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL
		);

		PxConvexMesh* CreateConvexMesh(PxU32 numVerts, const PxVec3* verts, bool directInsertion, PxU32 gaussMapLimit);

		static int CreateWeldedMeshIndices(const PxVec3* vertices, int numVertices, int* uniqueVerts, int* originalToUniqueMap, float threshold);

		static PxGeometry* CreatePxGeometry(const PxGeometryType::Enum type, const int numShapeParams, const float* shapeParams, void* shapeRef);

		// Convex core geometry is kept off CreatePxGeometry because it is parameterised by a
		// core type and a margin, neither of which fits that function's (type, params, ref)
		// shape. The core is a GJK support function; the collision shape is the core swept by
		// margin, so a cylinder with a non-zero margin has rounded edges.
		static PxGeometry* CreateConvexCoreGeometry(const PxConvexCore::Type coreType, const int numCoreParams, const float* coreParams, const float margin);

		PxMaterial* CreateMaterial(const float staticFriction, const float dynamicFriction, const float restitution);

		PxPBDMaterial* CreatePBDMaterial(const float friction, const float damping, const float adhesion, const float viscosity, const float vorticityConfinement,
			const float surfaceTension, const float cohesion, const float lift, const float drag, const float cflCoefficient, const float gravityScale);

		PxDeformableVolumeMaterial* CreateFEMSoftBodyMaterial(const float youngs, const float poissons, const float dynamicFriction, const float damping, const PxDeformableVolumeMaterialModel::Enum model = PxDeformableVolumeMaterialModel::eCO_ROTATIONAL);

		// Add this new method
		std::string GetAndClearErrors() {
			return mErrorCallback.getAndClearErrors();
		}
	};

	// The process-wide wrapper instance, defined in PxwAPIs.cpp. Exposed so that
	// translation units added after the original design (such as the deterministic
	// simulation layer) can reach PxPhysics without another global.
	PhysXWrapper& GetGlobalPhysXWrapper();
}