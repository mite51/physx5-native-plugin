#pragma once

#include <unordered_map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "DataInterop.h"
#include <PxAnisotropy.h>
#include <gpu/PxPhysicsGpu.h>
#include "extensions/PxParticleExt.h"
#include "extensions/PxParticleClothCooker.h"

using namespace physx;
using namespace ExtGpu;

namespace pxw
{
	class PxwPBDParticleSystemHelper
	{
	private:
		PxScene* mScene;
		PxPBDParticleSystem* mPBDParticleSystem;
		PxCudaContextManager* mCudaContextManager;
		PxU32 mMaxNumParticlesForAnisotropy;
		PxAnisotropyGenerator* mAnisotropyGenerator;
		PxwAnisotropyBuffer mAnisotropyBuffer;
		PxU32 mObjectParticleIndexOffset;
	public:
		PxwPBDParticleSystemHelper(PxScene* scene, PxPBDParticleSystem* pbdParticleSystem, PxU32 maxNumParticlesForAnisotropy)
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "new pbd helper\n");
			mScene = scene;
			mPBDParticleSystem = pbdParticleSystem;
			mMaxNumParticlesForAnisotropy = maxNumParticlesForAnisotropy;

			mCudaContextManager = scene->getCudaContextManager();
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "new pbd helper 1\n");

			// Anisotropy
			PxPhysicsGpu* physicsGpu = PxGetPhysicsGpu();
			PxAnisotropyGenerator* aniGen = physicsGpu->createAnisotropyGenerator(mCudaContextManager, maxNumParticlesForAnisotropy, 2.0f, 0.2f, 1.0f);
			mAnisotropyBuffer.anisotropy1 = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxNumParticlesForAnisotropy);
			mAnisotropyBuffer.anisotropy2 = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxNumParticlesForAnisotropy);
			mAnisotropyBuffer.anisotropy3 = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxNumParticlesForAnisotropy);
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "new pbd helper 2\n");
			aniGen->setResultBufferHost(
				mAnisotropyBuffer.anisotropy1,
				mAnisotropyBuffer.anisotropy2,
				mAnisotropyBuffer.anisotropy3);
			mAnisotropyGenerator = aniGen;
			PxAnisotropyCallback* aniCallback = new PxAnisotropyCallback();
			aniCallback->initialize(aniGen);
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "new pbd helper 3\n");
			mPBDParticleSystem->setParticleSystemCallback(aniCallback);
			mObjectParticleIndexOffset = 0;
		}

		~PxwPBDParticleSystemHelper() { }

		void AddToScene();

		void RemoveFromScene();

		PxBounds3 GetBounds();

		PxPBDParticleSystem* GetPBDParticleSystem()
		{
			return mPBDParticleSystem;
		}

		PxU32 GetMaxNumParticlesForAnisotropy()
		{
			return mMaxNumParticlesForAnisotropy;
		}

		PxU32 GetObjectParticleIndexOffset()
		{
			return mObjectParticleIndexOffset;
		}

		void IncrementObjectParticleIndexOffset(int numParticles)
		{
			mObjectParticleIndexOffset += numParticles;
		}

		void DecrementObjectParticleIndexOffset(int numParticles)
		{
			mObjectParticleIndexOffset -= numParticles;
		}

		PxwAnisotropyBuffer GetAnistropyData() { return mAnisotropyBuffer; }

		void Release()
		{
			mAnisotropyGenerator->release();
			mCudaContextManager->freePinnedHostBuffer(mAnisotropyBuffer.anisotropy1);
			mCudaContextManager->freePinnedHostBuffer(mAnisotropyBuffer.anisotropy2);
			mCudaContextManager->freePinnedHostBuffer(mAnisotropyBuffer.anisotropy3);
			mPBDParticleSystem->release();
		}
	};

	class PxwParticleSystemObject
	{
	protected:
		PxReal mParticleSpacing;
		PxScene* mScene;
		PxCudaContextManager* mCudaContextManager;
		PxParticleSystem* mParticleSystem;
		PxwPBDParticleSystemHelper* mPBDParticleSystemHelper;
		PxPBDMaterial* mMaterial;
		PxParticleBuffer* mDeviceBuffer;
		PxwParticleData* mHostBuffer;
		PxwParticleData* mHostBufferInitialState;
		PxParticleAttachmentBuffer* mAttachmentBuffer;
	public:
		PxwParticleSystemObject()
		{
			mParticleSpacing = 0.0f;
			mScene = NULL;
			mCudaContextManager = NULL;
			mParticleSystem = NULL;
			mPBDParticleSystemHelper = NULL;
			mMaterial = NULL;
			mDeviceBuffer = NULL;
			mHostBuffer = new PxwParticleData();
			mHostBufferInitialState = new PxwParticleData();
			mAttachmentBuffer = NULL;
		}

		virtual ~PxwParticleSystemObject() {}

		void SetScene(PxScene* scene) { mScene = scene; mCudaContextManager = scene->getCudaContextManager(); }

		void SetParticleSystem(PxwPBDParticleSystemHelper* particleSystemHelper, PxReal particleSpacing)
		{
			mPBDParticleSystemHelper = particleSystemHelper;
			mParticleSystem = particleSystemHelper->GetPBDParticleSystem();
			mParticleSpacing = particleSpacing;
		}

		void AddToParticleSystem();

		void RemoveFromParticleSystem();

		void SetMaterial(PxPBDMaterial* material) { mMaterial = material; }

		// Get the current host buffer
		PxwParticleData GetParticleData() { return *mHostBuffer; }

		// Sync particle data from device buffer to host
		void SyncParticleDataDeviceToHost(bool copyPhase);
		
		// Sync particle data from host buffer to device
		void SyncParticleDataHostToDevice(bool copyPhase);

		void AddRigidFilter(PxRigidActor* rigidActor, int idx);

		void RemoveRigidFilter(PxRigidActor* rigidActor, int idx);

		void AddRigidAttachment(int particleIdx, PxRigidActor* actor, PxVec3* pos);

		void RemoveRigidAttachment(int particleIdx, PxRigidActor* actor);

		// Reset the object
		virtual void ResetObject() = 0;

		// Release the buffers corresponding to the object
		void Release();
	};

	class PxwPBDBoxFluid : public PxwParticleSystemObject
	{
	protected:
		PxU32 mNumX;
		PxU32 mNumY;
		PxU32 mNumZ;
		PxVec3 mPosition;
		PxReal mFluidDensity;
		PxU32 mMaxDiffuseParticles;
		PxReal mBuoyancy;
		//PxAnisotropyGenerator* mAnisotropyGenerator;
		PxwAnisotropyBuffer mAnisotropyBuffer;
	public:
		PxwPBDBoxFluid(
			PxU32 numX,
			PxU32 numY,
			PxU32 numZ,
			PxVec3 position,
			PxReal fluidDensity,
			PxU32 maxDiffuseParticles,
			PxReal buoyancy
		) : PxwParticleSystemObject() {
			mNumX = numX;
			mNumY = numY;
			mNumZ = numZ;
			mPosition = position;
			mFluidDensity = fluidDensity;
			mMaxDiffuseParticles = maxDiffuseParticles;
			mBuoyancy = buoyancy;

			//mAnisotropyGenerator = NULL;
			mAnisotropyBuffer = PxwAnisotropyBuffer();
		}

		PxwAnisotropyBuffer GetAnistropyData() { return mAnisotropyBuffer; }

		void Create();

		void ResetObject();

		void Release();
	};

	class PxwPBDFluid : public PxwParticleSystemObject
	{
	protected:
		PxVec4* mPositions;
		PxU32 mNumParticles;
		PxReal mFluidDensity;
		PxU32 mMaxDiffuseParticles;
		PxReal mBuoyancy;
		PxwAnisotropyBuffer mAnisotropyBuffer;
	public:
		PxwPBDFluid(
			PxVec4* positions,
			PxU32 numParticles,
			PxReal fluidDensity,
			PxU32 maxDiffuseParticles,
			PxReal buoyancy
		) : PxwParticleSystemObject() {
			mPositions = new PxVec4[numParticles];
			mNumParticles = numParticles;

			PxMemCopy(mPositions, positions, numParticles * sizeof(PxVec4));

			mFluidDensity = fluidDensity;
			mMaxDiffuseParticles = maxDiffuseParticles;
			mBuoyancy = buoyancy;

			mAnisotropyBuffer = PxwAnisotropyBuffer();
		}

		PxwAnisotropyBuffer GetAnistropyData() { return mAnisotropyBuffer; }

		void Create();

		void ResetObject();

		void Release();
	};

	class PxwPBDCloth : public PxwParticleSystemObject
	{
	protected:
		PxVec3 mPosition;
		PxReal mTotalMass;
		bool mInflatable;
		PxReal mBlendScale;
		PxReal mRestVolume;
		PxReal mPressure;
		PxArray<PxParticleSpring> mSprings;
		PxArray<PxParticleSpring> mInitialSprings;
		PxArray<PxU32> mTriangles;
	public:
		PxwPBDCloth(
			PxVec3 position,
			PxReal totalMass,
			bool inflatable,
			PxReal blendScale,
			PxReal restVolume,
			PxReal pressure
		) : PxwParticleSystemObject()
		{
			mPosition = position;
			mTotalMass = totalMass;
			mInflatable = inflatable;
			mBlendScale = blendScale;
			mRestVolume = restVolume;
			mPressure = pressure;
		}

		void ResetObject();

		void Release();

		// Get the a struct containing the number of springs and a pointer to springs 
		PxwParticleSpringsData GetSpringData();

		// Update the springs and re-partition the cloth
		void UpdateSprings(PxParticleSpring* springs, int numSprings);
	};

	
	class PxwPBDRectCloth : public PxwPBDCloth
	{
	protected:
		PxU32 mNumX;
		PxU32 mNumZ;
	public:
		// A simple rectangular piece of cloth
		PxwPBDRectCloth(
			PxU32 numX,
			PxU32 numZ,
			PxVec3 position,
			PxReal totalMass
		) : PxwPBDCloth(position, totalMass, false, 0.0f, 0.0f, 0.0f)
		{
			mNumX = numX;
			mNumZ = numZ;
		}

		static inline PxU32 Id(PxU32 x, PxU32 y, PxU32 numY)
		{
			return x * numY + y;
		}

		void Create();
	};

	class PxwPBDTriMeshCloth : public PxwPBDCloth
	{
	protected:
		PxArray<PxVec4> mVertices;
		PxArray<PxU32> mIndices;
	public:
		PxwPBDTriMeshCloth(
			PxVec3* vertices,
			int numVertices,
			int* indices,
			int numIndices,
			PxVec3 position,
			PxReal totalMass,
			bool inflatable,
			PxReal blendScale,
			PxReal pressure
		) : PxwPBDCloth(position, totalMass, inflatable, blendScale, 0.0f, pressure)
		{
			mVertices.resize(numVertices);
			PxReal invMass = 1.0f / (totalMass / numVertices);
			for (int i = 0; i < numVertices; ++i)
			{
				mVertices[i] = PxVec4(vertices[i] + mPosition, invMass);
			}
			mIndices.resize(numIndices);
			for (int i = 0; i < numIndices; ++i)
			{
				mIndices[i] = indices[i];
			}
		}

		void Create();
	};
}
