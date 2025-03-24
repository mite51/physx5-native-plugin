#pragma once

#include <utility>
#include <limits.h>
#include "PxPhysicsAPI.h"
//#include "pvd/PxPvd.h"
#include "extensions/PxParticleExt.h"
#include <list>
#include <sstream>
#include "cudamanager/PxCudaContext.h"
#include <gpu/PxPhysicsGpu.h>
#include <PxAnisotropy.h>

#include "ParticleSystemHelper.h"
#include "SoftBodyHelper.h"
#include "Robotics.h"

#include <mutex>
#include <sstream>

using namespace physx;
using namespace ExtGpu;
using namespace std;

namespace pxw {

	// Add this new class before PhysXWrapper
	class BufferedErrorCallback : public PxDefaultErrorCallback 
	{
	private:
		std::string mErrorBuffer;
		std::mutex mMutex;

	public:
		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line) override {
			std::lock_guard<std::mutex> lock(mMutex);
			// Format error message
			std::stringstream ss;
			ss << "PhysX Error [" << code << "] in " << file << ":" << line << " - " << message << "\n";
			mErrorBuffer += ss.str();
			
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

		static void SetupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData);

	public:
		PhysXWrapper();
		
		PxPhysics* GetPhysics()
		{
			return mPhysics;
		}

		// PhysX basics
		void InitPhysX();

		bool GetPhysXInitStatus();

		PxScene* CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu);

		void StepPhysics(PxReal dt);

		void StepPhysicsStart(PxReal dt);

		void StepPhysicsFetchResults();

		void StepScene(PxScene* scene, PxReal dt);

		void ReleaseScene(PxScene* scene);

		void CleanupPhysX();

		// PxActor basics

		void AddActorToScene(PxScene* scene, PxActor* actor);

		void RemoveActorFromScene(PxScene* scene, PxActor* actor);

		PxShape* CreateShape(PxGeometry* geometry, PxMaterial* material, bool isExclusive);

		// Particle system
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

		// Rigid and Soft Bodies

		PxActor* CreateDynamicRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxActor* CreateKinematicRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxActor* CreateStaticRigidActor(PxScene* scene, const PxwTransformData transform, PxShape* shape);

		PxwSoftBodyHelper* CreateFEMSoftBody(PxScene* scene, const PxU32 numVertices, const PxVec3* triVerts, const PxU32 numTriangles, const int* triIndices, PxwTransformData pose, PxFEMSoftBodyMaterial* material, PxReal density, PxU32 iterationCount, bool useCollisionMeshForSimulation = false, PxU32 numVoxelsAlongLongestAABBAxis = 8);

		// Robotics
		PxwArticulationKinematicTree* CreatePxArticulationKinematicTree(PxScene* scene, bool fixBase, bool disableSelfCollision);

		PxwArticulationRobot* CreateArticulationRobot(PxScene* scene, PxwTransformData basePose, float density);

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

		PxMaterial* CreateMaterial(const float staticFriction, const float dynamicFriction, const float restitution);

		PxPBDMaterial* CreatePBDMaterial(const float friction, const float damping, const float adhesion, const float viscosity, const float vorticityConfinement,
			const float surfaceTension, const float cohesion, const float lift, const float drag, const float cflCoefficient, const float gravityScale);

		PxFEMSoftBodyMaterial* CreateFEMSoftBodyMaterial(const float youngs, const float poissons, const float dynamicFriction, const float damping, const PxFEMSoftBodyMaterialModel::Enum model = PxFEMSoftBodyMaterialModel::eCO_ROTATIONAL);

		// Add this new method
		std::string GetAndClearErrors() {
			return mErrorCallback.getAndClearErrors();
		}
	};
}