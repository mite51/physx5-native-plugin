#include "ParticleSystemHelper.h"

using namespace physx::ExtGpu;
using namespace pxw;

namespace pxw
{
	void PxwParticleSystemObject::AddToParticleSystem()
	{
		mParticleSystem->addParticleBuffer(mDeviceBuffer);
	}

	void PxwParticleSystemObject::RemoveFromParticleSystem()
	{
		mParticleSystem->removeParticleBuffer(mDeviceBuffer);
	}

	void PxwParticleSystemObject::SyncParticleDataDeviceToHost(bool copyPhase)
	{
		if (mDeviceBuffer == NULL)
		{
			return;
		}

		// Device pointers
		PxVec4* positionInvMassDevice = mDeviceBuffer->getPositionInvMasses();
		PxVec4* velocityDevice = mDeviceBuffer->getVelocities();

		// Host pointers
		PxVec4* positionInvMassHost = mHostBuffer->positionInvMass;
		PxVec4* velocityHost = mHostBuffer->velocity;

		mCudaContextManager->acquireContext();
		PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

		int numParticles = mHostBuffer->numParticles;
		cudaContext->memcpyDtoH(positionInvMassHost, CUdeviceptr(positionInvMassDevice), sizeof(PxVec4) * numParticles);
		cudaContext->memcpyDtoH(velocityHost, CUdeviceptr(velocityDevice), sizeof(PxVec4) * numParticles);
		if (copyPhase)
		{
			PxU32* phaseDevice = mDeviceBuffer->getPhases();
			PxU32* phaseHost = mHostBuffer->phase;
			cudaContext->memcpyDtoH(phaseHost, CUdeviceptr(phaseDevice), sizeof(PxU32) * numParticles);
		}
		mCudaContextManager->releaseContext();
	}

	void PxwParticleSystemObject::SyncParticleDataHostToDevice(bool copyPhase)
	{
		if (mDeviceBuffer == NULL)
		{
			return;
		}

		// Device pointers
		PxVec4* positionInvMassDevice = mDeviceBuffer->getPositionInvMasses();
		PxVec4* velocityDevice = mDeviceBuffer->getVelocities();

		// Host pointers
		PxVec4* positionInvMassHost = mHostBuffer->positionInvMass;
		PxVec4* velocityHost = mHostBuffer->velocity;

		mCudaContextManager->acquireContext();
		PxCudaContext* cudaContext = mCudaContextManager->getCudaContext();

		int numParticles = mHostBuffer->numParticles;
		cudaContext->memcpyHtoD(CUdeviceptr(positionInvMassDevice), positionInvMassHost, sizeof(PxVec4) * numParticles);
		cudaContext->memcpyHtoD(CUdeviceptr(velocityDevice), velocityHost, sizeof(PxVec4) * numParticles);
		if (copyPhase)
		{
			PxU32* phaseDevice = mDeviceBuffer->getPhases();
			PxU32* phaseHost = mHostBuffer->phase;
			if (copyPhase) cudaContext->memcpyHtoD(CUdeviceptr(phaseDevice), phaseHost, sizeof(PxU32) * numParticles);
			mDeviceBuffer->raiseFlags(static_cast<PxParticleBufferFlag::Enum>(PxParticleBufferFlag::eUPDATE_POSITION | PxParticleBufferFlag::eUPDATE_VELOCITY | PxParticleBufferFlag::eUPDATE_PHASE));
		}
		else
		{
			mDeviceBuffer->raiseFlags(static_cast<PxParticleBufferFlag::Enum>(PxParticleBufferFlag::eUPDATE_POSITION | PxParticleBufferFlag::eUPDATE_VELOCITY));
		}
		mCudaContextManager->releaseContext();
	}

	void PxwParticleSystemObject::AddRigidFilter(PxRigidActor* rigidActor, int idx)
	{
		if (mAttachmentBuffer == NULL)
		{
			mAttachmentBuffer = PxCreateParticleAttachmentBuffer(*mDeviceBuffer, *mParticleSystem);
		}
		mAttachmentBuffer->addRigidFilter(rigidActor, idx);
		mAttachmentBuffer->copyToDevice();
	}

	void PxwParticleSystemObject::RemoveRigidFilter(PxRigidActor* rigidActor, int idx)
	{
		if (mAttachmentBuffer != NULL)
		{
			mAttachmentBuffer->removeRigidFilter(rigidActor, idx);
			mAttachmentBuffer->copyToDevice();
		}
	}

	void PxwParticleSystemObject::AddRigidAttachment(int particleIdx, PxRigidActor* actor, PxVec3* pos)
	{
		if (mAttachmentBuffer == NULL)
		{
			mAttachmentBuffer = PxCreateParticleAttachmentBuffer(*mDeviceBuffer, *mParticleSystem);
		}
		mAttachmentBuffer->addRigidAttachment(actor, particleIdx, *pos);
		mAttachmentBuffer->copyToDevice();
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add rigid attachment\n");
	}

	void PxwParticleSystemObject::RemoveRigidAttachment(int particleIdx, PxRigidActor* actor)
	{
		if (mAttachmentBuffer != NULL)
		{
			mAttachmentBuffer->removeRigidAttachment(actor, particleIdx);
			mAttachmentBuffer->copyToDevice();
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Remove rigid attachment\n");
		}
	}

	void PxwParticleSystemObject::Release()
	{
		if (mAttachmentBuffer != NULL)
		{
			delete mAttachmentBuffer;
		}
	}

	void PxwPBDBoxFluid::Create()
	{
		if (mScene == NULL || mParticleSystem == NULL)
		{
			return;
		}

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add fluid\n");

		const PxU32 maxParticles = mNumX * mNumY * mNumZ;

		// Diffuse particles setting
		PxDiffuseParticleParams dpParams;
		dpParams.threshold = 300.0f;
		dpParams.bubbleDrag = 0.9f;
		dpParams.buoyancy = mBuoyancy;
		dpParams.airDrag = 0.0f;
		dpParams.kineticEnergyWeight = 0.01f;
		dpParams.pressureWeight = 1.0f;
		dpParams.divergenceWeight = 10.f;
		dpParams.lifetime = 1.0f;
		dpParams.useAccurateVelocity = false;

		// Create particles and add them to the particle system
		const PxU32 particlePhase = mParticleSystem->createPhase(mMaterial, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));
		const PxReal particleMass = mFluidDensity * 1.333f * 3.14159f * mParticleSpacing * mParticleSpacing * mParticleSpacing;

		PxU32* phase = mCudaContextManager->allocPinnedHostBuffer<PxU32>(maxParticles);
		PxVec4* positionInvMass = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);
		PxVec4* velocity = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);

		PxReal halfExtentX = mParticleSpacing * (mNumX - 1) / 2.0f;
		PxReal halfExtentY = mParticleSpacing * (mNumY - 1) / 2.0f;
		PxReal halfExtentZ = mParticleSpacing * (mNumZ - 1) / 2.0f;

		PxReal startX = mPosition.x - halfExtentX;
		PxReal startY = mPosition.y - halfExtentY;
		PxReal startZ = mPosition.z - halfExtentZ;

		for (PxU32 i = 0; i < mNumX; ++i)
		{
			for (PxU32 j = 0; j < mNumY; ++j)
			{
				for (PxU32 k = 0; k < mNumZ; ++k)
				{
					const PxU32 index = i * (mNumY * mNumZ) + j * mNumZ + k;

					PxVec4 pos(startX, startY, startZ, 1.0f / particleMass);
					phase[index] = particlePhase;
					positionInvMass[index] = pos;
					velocity[index] = PxVec4(0.0f);

					startZ += mParticleSpacing;
				}
				startZ = mPosition.z - halfExtentZ;
				startY += mParticleSpacing;
			}
			startY = mPosition.y - halfExtentY;
			startX += mParticleSpacing;
		}

		ExtGpu::PxParticleAndDiffuseBufferDesc bufferDesc;
		bufferDesc.maxParticles = maxParticles;
		bufferDesc.numActiveParticles = maxParticles;
		bufferDesc.maxDiffuseParticles = mMaxDiffuseParticles;
		bufferDesc.maxActiveDiffuseParticles = mMaxDiffuseParticles;
		bufferDesc.diffuseParams = dpParams;

		bufferDesc.positions = positionInvMass;
		bufferDesc.velocities = velocity;
		bufferDesc.phases = phase;

		PxParticleAndDiffuseBuffer* ParticleBuffer = physx::ExtGpu::PxCreateAndPopulateParticleAndDiffuseBuffer(bufferDesc, mCudaContextManager);
		//mParticleSystem->addParticleBuffer(ParticleBuffer);

		mDeviceBuffer = ParticleBuffer;
		mHostBuffer->positionInvMass = positionInvMass;
		mHostBuffer->velocity = velocity;
		mHostBuffer->phase = phase;
		mHostBuffer->numParticles = maxParticles;

		// Set the initial buffer states for fast recreating the object; No need for pinning
		mHostBufferInitialState->positionInvMass = new PxVec4[maxParticles];
		mHostBufferInitialState->velocity = new PxVec4[maxParticles];
		mHostBufferInitialState->phase = new PxU32[maxParticles];
		mHostBufferInitialState->numParticles = maxParticles;
		memcpy(mHostBufferInitialState->positionInvMass, mHostBuffer->positionInvMass, sizeof(PxVec4) * maxParticles);
		memcpy(mHostBufferInitialState->velocity, mHostBuffer->velocity, sizeof(PxVec4) * maxParticles);
		memcpy(mHostBufferInitialState->phase, mHostBuffer->phase, sizeof(PxU32) * maxParticles);

		// Anisotropy buffers are offset by the existing PBD objects in the same particle system
		const int aniOffset = mPBDParticleSystemHelper->GetObjectParticleIndexOffset();
		if (aniOffset + maxParticles > mPBDParticleSystemHelper->GetMaxNumParticlesForAnisotropy())
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Reached max num for anisotropy\n");
			mAnisotropyBuffer.anisotropy1 = NULL;
			mAnisotropyBuffer.anisotropy2 = NULL;
			mAnisotropyBuffer.anisotropy3 = NULL;
			return;
		}
		mAnisotropyBuffer.anisotropy1 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy1 + aniOffset;
		mAnisotropyBuffer.anisotropy2 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy2 + aniOffset;
		mAnisotropyBuffer.anisotropy3 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy3 + aniOffset;
		mPBDParticleSystemHelper->IncrementObjectParticleIndexOffset(maxParticles);

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add fluid done\n");
	}

	void PxwPBDBoxFluid::ResetObject()
	{
		int numParticles = mHostBuffer->numParticles;
		memcpy(mHostBuffer->positionInvMass, mHostBufferInitialState->positionInvMass, sizeof(PxVec4) * numParticles);
		memcpy(mHostBuffer->velocity, mHostBufferInitialState->velocity, sizeof(PxVec4) * numParticles);
		memcpy(mHostBuffer->phase, mHostBufferInitialState->phase, sizeof(PxU32) * numParticles);
		SyncParticleDataHostToDevice(true);
	}

	void PxwPBDBoxFluid::Release()
	{
		if (mDeviceBuffer != nullptr)
		{
			mParticleSystem->removeParticleBuffer(mDeviceBuffer);
			
			// Free host buffer
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->positionInvMass);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->velocity);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->phase);
			mHostBuffer->positionInvMass = nullptr;
			mHostBuffer->velocity = nullptr;
			mHostBuffer->phase = nullptr;
			delete mHostBuffer;

			// Free initial state buffer
			delete mHostBufferInitialState->positionInvMass;
			delete mHostBufferInitialState->velocity;
			delete mHostBufferInitialState->phase;
			mHostBufferInitialState->positionInvMass = nullptr;
			mHostBufferInitialState->velocity = nullptr;
			mHostBufferInitialState->phase = nullptr;
			delete mHostBufferInitialState;

			// Release device buffer
			mDeviceBuffer->release();
			mDeviceBuffer = nullptr;
		}
		PxwParticleSystemObject::Release();
	}

	void PxwPBDFluid::Create()
	{
		if (mScene == NULL || mParticleSystem == NULL)
		{
			return;
		}

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add fluid\n");

		const PxU32 maxParticles = mNumParticles;

		// Diffuse particles setting
		PxDiffuseParticleParams dpParams;
		dpParams.threshold = 300.0f;
		dpParams.bubbleDrag = 0.9f;
		dpParams.buoyancy = mBuoyancy;
		dpParams.airDrag = 0.0f;
		dpParams.kineticEnergyWeight = 0.01f;
		dpParams.pressureWeight = 1.0f;
		dpParams.divergenceWeight = 10.f;
		dpParams.lifetime = 1.0f;
		dpParams.useAccurateVelocity = false;

		// Create particles and add them to the particle system
		const PxU32 particlePhase = mParticleSystem->createPhase(mMaterial, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

		PxU32* phase = mCudaContextManager->allocPinnedHostBuffer<PxU32>(maxParticles);
		PxVec4* positionInvMass = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);
		PxVec4* velocity = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);

		for (PxU32 i = 0; i < mNumParticles; ++i)
		{
			phase[i] = particlePhase;
			positionInvMass[i] = mPositions[i];
			velocity[i] = PxVec4(0.0f);
		}

		ExtGpu::PxParticleAndDiffuseBufferDesc bufferDesc;
		bufferDesc.maxParticles = maxParticles;
		bufferDesc.numActiveParticles = maxParticles;
		bufferDesc.maxDiffuseParticles = mMaxDiffuseParticles;
		bufferDesc.maxActiveDiffuseParticles = mMaxDiffuseParticles;
		bufferDesc.diffuseParams = dpParams;

		bufferDesc.positions = positionInvMass;
		bufferDesc.velocities = velocity;
		bufferDesc.phases = phase;

		PxParticleAndDiffuseBuffer* ParticleBuffer = physx::ExtGpu::PxCreateAndPopulateParticleAndDiffuseBuffer(bufferDesc, mCudaContextManager);

		mDeviceBuffer = ParticleBuffer;
		mHostBuffer->positionInvMass = positionInvMass;
		mHostBuffer->velocity = velocity;
		mHostBuffer->phase = phase;
		mHostBuffer->numParticles = maxParticles;

		// Set the initial buffer states for fast recreating the object; No need for pinning
		mHostBufferInitialState->positionInvMass = new PxVec4[maxParticles];
		mHostBufferInitialState->velocity = new PxVec4[maxParticles];
		mHostBufferInitialState->phase = new PxU32[maxParticles];
		mHostBufferInitialState->numParticles = maxParticles;
		memcpy(mHostBufferInitialState->positionInvMass, mHostBuffer->positionInvMass, sizeof(PxVec4) * maxParticles);
		memcpy(mHostBufferInitialState->velocity, mHostBuffer->velocity, sizeof(PxVec4) * maxParticles);
		memcpy(mHostBufferInitialState->phase, mHostBuffer->phase, sizeof(PxU32) * maxParticles);

		// Anisotropy buffers are offset by the existing PBD objects in the same particle system
		const int aniOffset = mPBDParticleSystemHelper->GetObjectParticleIndexOffset();
		if (aniOffset + maxParticles > mPBDParticleSystemHelper->GetMaxNumParticlesForAnisotropy())
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Reached max num for anisotropy\n");
			mAnisotropyBuffer.anisotropy1 = NULL;
			mAnisotropyBuffer.anisotropy2 = NULL;
			mAnisotropyBuffer.anisotropy3 = NULL;
			return;
		}
		mAnisotropyBuffer.anisotropy1 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy1 + aniOffset;
		mAnisotropyBuffer.anisotropy2 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy2 + aniOffset;
		mAnisotropyBuffer.anisotropy3 = mPBDParticleSystemHelper->GetAnistropyData().anisotropy3 + aniOffset;
		mPBDParticleSystemHelper->IncrementObjectParticleIndexOffset(maxParticles);

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add fluid done\n");
	}

	void PxwPBDFluid::ResetObject()
	{
		memcpy(mHostBuffer->positionInvMass, mHostBufferInitialState->positionInvMass, sizeof(PxVec4) * mNumParticles);
		memcpy(mHostBuffer->velocity, mHostBufferInitialState->velocity, sizeof(PxVec4) * mNumParticles);
		memcpy(mHostBuffer->phase, mHostBufferInitialState->phase, sizeof(PxU32) * mNumParticles);
		SyncParticleDataHostToDevice(true);
	}

	void PxwPBDFluid::Release()
	{
		delete[] mPositions;
		mPositions = NULL;
		if (mDeviceBuffer != NULL)
		{
			mParticleSystem->removeParticleBuffer(mDeviceBuffer);

			// Free host buffer
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->positionInvMass);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->velocity);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->phase);
			mHostBuffer->positionInvMass = NULL;
			mHostBuffer->velocity = NULL;
			mHostBuffer->phase = NULL;
			delete mHostBuffer;

			// Free initial state buffer
			delete mHostBufferInitialState->positionInvMass;
			delete mHostBufferInitialState->velocity;
			delete mHostBufferInitialState->phase;
			mHostBufferInitialState->positionInvMass = NULL;
			mHostBufferInitialState->velocity = NULL;
			mHostBufferInitialState->phase = NULL;
			delete mHostBufferInitialState;

			// Release device buffer
			mDeviceBuffer->release();
			mDeviceBuffer = NULL;
		}
		PxwParticleSystemObject::Release();
	}

	void PxwPBDCloth::ResetObject()
	{
		const PxU32 numParticles = mHostBuffer->numParticles;
		memcpy(mHostBuffer->positionInvMass, mHostBufferInitialState->positionInvMass, sizeof(PxVec4) * numParticles);
		memcpy(mHostBuffer->velocity, mHostBufferInitialState->velocity, sizeof(PxVec4) * numParticles);
		memcpy(mHostBuffer->phase, mHostBufferInitialState->phase, sizeof(PxU32) * numParticles);

		// Re-partition the cloth using PxParticleClothBufferHelper
		const PxU32 numTriangles = mTriangles.size() / 3;
		const PxU32 numSprings = mSprings.size();

		PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, mCudaContextManager);
		clothBuffers->addCloth(mBlendScale, mRestVolume, mPressure, mTriangles.begin(), numTriangles, mSprings.begin(), numSprings, mHostBufferInitialState->positionInvMass, numParticles);

		const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
		PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(mCudaContextManager);

		PxPartitionedParticleCloth output;
		clothPreProcessor->partitionSprings(clothDesc, output);
		clothPreProcessor->release();

		clothBuffers->release();

		SyncParticleDataHostToDevice(true);
	}

	void PxwPBDCloth::Release()
	{
		if (mDeviceBuffer != nullptr)
		{
			// Decrement the particle index in the helper
			mPBDParticleSystemHelper->DecrementObjectParticleIndexOffset(mHostBuffer->numParticles);

			mParticleSystem->removeParticleBuffer(mDeviceBuffer);

			// Free host buffer
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->positionInvMass);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->velocity);
			mCudaContextManager->freePinnedHostBuffer(mHostBuffer->phase);
			mHostBuffer->positionInvMass = nullptr;
			mHostBuffer->velocity = nullptr;
			mHostBuffer->phase = nullptr;
			delete mHostBuffer;

			// Free initial state buffer
			delete mHostBufferInitialState->positionInvMass;
			delete mHostBufferInitialState->velocity;
			delete mHostBufferInitialState->phase;
			mHostBufferInitialState->positionInvMass = nullptr;
			mHostBufferInitialState->velocity = nullptr;
			mHostBufferInitialState->phase = nullptr;
			delete mHostBufferInitialState;

			// Release device buffer
			mDeviceBuffer->release();
			mDeviceBuffer = nullptr;
		}
		PxwParticleSystemObject::Release();
	}

	PxwParticleSpringsData PxwPBDCloth::GetSpringData()
	{
		PxwParticleSpringsData springData = PxwParticleSpringsData();
		springData.numSprings = mSprings.size();
		springData.springs = mSprings.begin();
		return springData;
	}

	void PxwPBDCloth::UpdateSprings(PxParticleSpring* springs, int numSprings)
	{
		const int numTriangles = mTriangles.size() / 3;
		const int numParticles = mHostBuffer->numParticles;

		// Re-partition the cloth using PxParticleClothBufferHelper
		PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, mCudaContextManager);
		clothBuffers->addCloth(mBlendScale, mRestVolume, mPressure, mTriangles.begin(), numTriangles, springs, numSprings, mHostBufferInitialState->positionInvMass, numParticles);

		const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
		PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(mCudaContextManager);

		PxPartitionedParticleCloth output;
		clothPreProcessor->partitionSprings(clothDesc, output);
		clothPreProcessor->release();

		clothBuffers->release();

		static_cast<PxParticleClothBuffer*>(mDeviceBuffer)->setCloths(output);
	}

	void PxwPBDRectCloth::Create()
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add cloth\n");

		const PxU32 numParticles = mNumX * mNumZ;
		const PxU32 numSprings = (mNumX - 1) * (mNumZ - 1) * 4 + (mNumX - 1) + (mNumZ - 1);
		const PxU32 numTriangles = (mNumX - 1) * (mNumZ - 1) * 2;

		const PxReal stretchStiffness = 10000.f;
		const PxReal shearStiffness = 100.f;
		const PxReal springDamping = 0.001f;

		const PxReal particleMass = mTotalMass / numParticles;

		// Create particles and add them to the particle system
		const PxU32 particlePhase = mParticleSystem->createPhase(mMaterial, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

		PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, mCudaContextManager);

		PxU32* phase = mCudaContextManager->allocPinnedHostBuffer<PxU32>(numParticles);
		PxVec4* positionInvMass = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(numParticles);
		PxVec4* velocity = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(numParticles);

		PxReal x = mPosition.x;
		PxReal y = mPosition.y;
		PxReal z = mPosition.z;

		// Define springs and triangles
		PxArray<PxParticleSpring> springs;
		springs.reserve(numSprings);
		PxArray<PxU32> triangles;
		triangles.reserve(numTriangles * 3);

		for (PxU32 i = 0; i < mNumX; ++i)
		{
			for (PxU32 j = 0; j < mNumZ; ++j)
			{
				const PxU32 index = i * mNumZ + j;

				PxVec4 pos(x, y, z, 1.0f / particleMass);
				phase[index] = particlePhase;
				positionInvMass[index] = pos;
				velocity[index] = PxVec4(0.0f);

				if (i > 0)
				{
					PxParticleSpring spring = { Id(i - 1, j, mNumZ), Id(i, j, mNumZ), mParticleSpacing, stretchStiffness, springDamping, 0 };
					springs.pushBack(spring);
				}
				if (j > 0)
				{
					PxParticleSpring spring = { Id(i, j - 1, mNumZ), Id(i, j, mNumZ), mParticleSpacing, stretchStiffness, springDamping, 0 };
					springs.pushBack(spring);
				}

				if (i > 0 && j > 0)
				{
					PxParticleSpring spring0 = { Id(i - 1, j - 1, mNumZ), Id(i, j, mNumZ), PxSqrt(2.0f) * mParticleSpacing, shearStiffness, springDamping, 0 };
					springs.pushBack(spring0);
					PxParticleSpring spring1 = { Id(i - 1, j, mNumZ), Id(i, j - 1, mNumZ), PxSqrt(2.0f) * mParticleSpacing, shearStiffness, springDamping, 0 };
					springs.pushBack(spring1);

					//Triangles are used to compute approximated aerodynamic forces for cloth falling down
					triangles.pushBack(Id(i - 1, j - 1, mNumZ));
					triangles.pushBack(Id(i - 1, j, mNumZ));
					triangles.pushBack(Id(i, j - 1, mNumZ));

					triangles.pushBack(Id(i - 1, j, mNumZ));
					triangles.pushBack(Id(i, j - 1, mNumZ));
					triangles.pushBack(Id(i, j, mNumZ));
				}

				z += mParticleSpacing;
			}
			z = mPosition.z;
			x += mParticleSpacing;
		}

		PX_ASSERT(numSprings == springs.size());
		PX_ASSERT(numTriangles == triangles.size() / 3);

		clothBuffers->addCloth(0.0f, 0.0f, 0.0f, triangles.begin(), numTriangles, springs.begin(), numSprings, positionInvMass, numParticles);

		ExtGpu::PxParticleBufferDesc bufferDesc;
		bufferDesc.maxParticles = numParticles;
		bufferDesc.numActiveParticles = numParticles;
		bufferDesc.positions = positionInvMass;
		bufferDesc.velocities = velocity;
		bufferDesc.phases = phase;

		const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
		PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(mCudaContextManager);

		PxPartitionedParticleCloth output;
		clothPreProcessor->partitionSprings(clothDesc, output);
		clothPreProcessor->release();

		mDeviceBuffer = physx::ExtGpu::PxCreateAndPopulateParticleClothBuffer(bufferDesc, clothDesc, output, mCudaContextManager);
		//mParticleSystem->addParticleBuffer(mDeviceBuffer);

		clothBuffers->release();

		mHostBuffer->numParticles = numParticles;
		mHostBuffer->positionInvMass = positionInvMass;
		mHostBuffer->velocity = velocity;
		mHostBuffer->phase = phase;

		// Set the initial buffer states for fast recreating the object; No need for pinning
		mHostBufferInitialState->positionInvMass = new PxVec4[numParticles];
		mHostBufferInitialState->velocity = new PxVec4[numParticles];
		mHostBufferInitialState->phase = new PxU32[numParticles];
		mHostBufferInitialState->numParticles = numParticles;
		memcpy(mHostBufferInitialState->positionInvMass, mHostBuffer->positionInvMass, sizeof(PxVec4) * numParticles);
		memcpy(mHostBufferInitialState->velocity, mHostBuffer->velocity, sizeof(PxVec4) * numParticles);
		memcpy(mHostBufferInitialState->phase, mHostBuffer->phase, sizeof(PxU32) * numParticles);

		mSprings = springs;
		mTriangles = triangles;
		mInitialSprings = springs;

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add cloth done\n");
	}

	void PxwPBDTriMeshCloth::Create()
	{
		const PxU32 numParticles = mVertices.size();
		const PxReal stretchStiffness = 10000.f;
		const PxReal shearStiffness = 100.f;
		const PxReal springDamping = 0.001f;
		const PxReal bendStiffness = 10.f;

		// Cook cloth
		PxParticleClothCooker* cooker = PxCreateParticleClothCooker(mVertices.size(), mVertices.begin(), mIndices.size(), mIndices.begin(),
			PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT | PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT | PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT | PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT);
		cooker->cookConstraints();
		if (mInflatable) cooker->calculateMeshVolume();

		// Apply cooked constraints to particle springs
		PxU32 constraintCount = cooker->getConstraintCount();
		PxParticleClothConstraint* constraintBuffer = cooker->getConstraints();
		PxArray<PxParticleSpring> springs;
		springs.reserve(constraintCount);

		std::vector<PxU32> particlesLeftSide;
		std::vector<PxU32> particlesRightSide;

		for (PxU32 i = 0; i < constraintCount; i++)
		{
			const PxParticleClothConstraint& c = constraintBuffer[i];
			PxReal stiffness = 0.0f;
			switch (c.constraintType)
			{
			case PxParticleClothConstraint::eTYPE_INVALID_CONSTRAINT:
				continue;
			case PxParticleClothConstraint::eTYPE_HORIZONTAL_CONSTRAINT:
			case PxParticleClothConstraint::eTYPE_VERTICAL_CONSTRAINT:
				stiffness = stretchStiffness;
				break;
			case PxParticleClothConstraint::eTYPE_DIAGONAL_CONSTRAINT:
				stiffness = shearStiffness;
				break;
			case PxParticleClothConstraint::eTYPE_BENDING_CONSTRAINT:
				stiffness = bendStiffness;
				break;
			default:
				PX_ASSERT("Invalid cloth constraint generated by PxParticleClothCooker");
			}
			PxParticleSpring spring;
			spring.ind0 = c.particleIndexA;
			spring.ind1 = c.particleIndexB;
			spring.stiffness = stiffness;
			spring.damping = springDamping;
			spring.length = c.length;
			springs.pushBack(spring);
		}

		// Update the rest volume if is an inflatable
		if (mInflatable)
		{
			mRestVolume = cooker->getMeshVolume();
		}

		// Read results
		const PxU32 numSprings = springs.size();
		const PxU32 numTriangles = cooker->getTriangleIndicesCount() / 3;
		const PxU32* triangles = cooker->getTriangleIndices();

		// Create particles and add them to the particle system
		const PxU32 particlePhase = mParticleSystem->createPhase(mMaterial, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide));
		PxU32* phases = mCudaContextManager->allocPinnedHostBuffer<PxU32>(numParticles);
		PxVec4* positionInvMass = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(numParticles);
		PxVec4* velocity = mCudaContextManager->allocPinnedHostBuffer<PxVec4>(numParticles);


		for (PxU32 v = 0; v < numParticles; v++)
		{
			positionInvMass[v] = mVertices[v];
			velocity[v] = PxVec4(0.0f, 0.0f, 0.0f, 0.0f);
			phases[v] = particlePhase;
		}

		PxParticleClothBufferHelper* clothBuffers = PxCreateParticleClothBufferHelper(1, numTriangles, numSprings, numParticles, mCudaContextManager);
		clothBuffers->addCloth(0.0f, mRestVolume, mPressure, triangles, numTriangles, springs.begin(), numSprings, positionInvMass, numParticles);
		cooker->release();

		ExtGpu::PxParticleBufferDesc bufferDesc;
		bufferDesc.maxParticles = numParticles;
		bufferDesc.numActiveParticles = numParticles;
		bufferDesc.positions = positionInvMass;
		bufferDesc.velocities = velocity;
		bufferDesc.phases = phases;

		PxParticleVolumeBufferHelper* volumeBuffers = NULL;
		if (mInflatable)
		{
			volumeBuffers = PxCreateParticleVolumeBufferHelper(1, numTriangles, mCudaContextManager); //Volumes are optional. They are used to accelerate scene queries, e. g. to support picking.
			volumeBuffers->addVolume(0, numParticles, triangles, numTriangles);
			bufferDesc.maxVolumes = volumeBuffers->getMaxVolumes();
			bufferDesc.numVolumes = volumeBuffers->getNumVolumes();
			bufferDesc.volumes = volumeBuffers->getParticleVolumes();
		}

		PxParticleClothPreProcessor* clothPreProcessor = PxCreateParticleClothPreProcessor(mCudaContextManager);

		PxPartitionedParticleCloth output;
		const PxParticleClothDesc& clothDesc = clothBuffers->getParticleClothDesc();
		clothPreProcessor->partitionSprings(clothDesc, output);
		clothPreProcessor->release();

		mDeviceBuffer = physx::ExtGpu::PxCreateAndPopulateParticleClothBuffer(bufferDesc, clothDesc, output, mCudaContextManager);
		//mParticleSystem->addParticleBuffer(mDeviceBuffer);

		mHostBuffer->numParticles = numParticles;
		mHostBuffer->positionInvMass = positionInvMass;
		mHostBuffer->velocity = velocity;
		mHostBuffer->phase = phases;

		// Set the initial buffer states for fast recreating the object; No need for pinning
		mHostBufferInitialState->positionInvMass = new PxVec4[numParticles];
		mHostBufferInitialState->velocity = new PxVec4[numParticles];
		mHostBufferInitialState->phase = new PxU32[numParticles];
		mHostBufferInitialState->numParticles = numParticles;
		memcpy(mHostBufferInitialState->positionInvMass, mHostBuffer->positionInvMass, sizeof(PxVec4) * numParticles);
		memcpy(mHostBufferInitialState->velocity, mHostBuffer->velocity, sizeof(PxVec4) * numParticles);
		memcpy(mHostBufferInitialState->phase, mHostBuffer->phase, sizeof(PxU32) * numParticles);

		mSprings = springs;
		mTriangles = PxArray<PxU32>(numTriangles, *triangles);
		mInitialSprings = springs;

		clothBuffers->release();
		if (volumeBuffers)
		{
			volumeBuffers->release();
		}
	}

	void PxwPBDParticleSystemHelper::AddToScene()
	{
		mScene->addActor(*mPBDParticleSystem);
	}

	void PxwPBDParticleSystemHelper::RemoveFromScene()
	{
		mScene->removeActor(*mPBDParticleSystem);
	}

	PxBounds3 PxwPBDParticleSystemHelper::GetBounds()
	{
		return mPBDParticleSystem->getWorldBounds();
	}
}