#include "PhysXWrapper.h"

using namespace pxw;

namespace pxw
{
	PxwPBDParticleSystemHelper* PhysXWrapper::CreatePBDParticleSystem(
		PxScene* scene,
		const PxReal particleSpacing,
		int maxNumParticlesForAnisotropy
	) {
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add particle system\n");

		if (mCudaContextManager == NULL)
			return NULL;

		const PxReal restOffset = 0.5f * particleSpacing / 0.6f;

		PxPBDParticleSystem* particleSystem = mPhysics->createPBDParticleSystem(*mCudaContextManager, 96);

		// General particle system setting

		const PxReal solidRestOffset = restOffset;
		const PxReal fluidRestOffset = restOffset * 0.6f;
		particleSystem->setRestOffset(restOffset);
		particleSystem->setContactOffset(restOffset + 0.01f);
		particleSystem->setParticleContactOffset(fluidRestOffset / 0.6f);
		particleSystem->setSolidRestOffset(solidRestOffset);
		particleSystem->setFluidRestOffset(fluidRestOffset);
		particleSystem->enableCCD(false);
		particleSystem->setMaxVelocity(solidRestOffset * 100.f);
		//scene->addActor(*particleSystem);

		PxwPBDParticleSystemHelper* particleSystemHelper = new PxwPBDParticleSystemHelper(scene, particleSystem, maxNumParticlesForAnisotropy);

		return particleSystemHelper;
	}

	// Release the PBD particle system and remove from scene. Make sure to release the associated PBD objects first
	void PhysXWrapper::ReleasePBDParticleSystem(PxScene* scene, PxPBDParticleSystem* particleSystem)
	{
		scene->removeActor(*particleSystem);
		particleSystem->release();
	}

	PxwPBDBoxFluid* PhysXWrapper::CreateCubeFluid(
		PxScene* scene,
		PxwPBDParticleSystemHelper* particleSystem,
		PxPBDMaterial* material,
		const PxU32 numX,
		const PxU32 numY,
		const PxU32 numZ,
		const PxVec3& position,
		const PxReal particleSpacing,
		const PxReal fluidDensity,
		const PxU32 maxDiffuseParticles,
		const PxReal buoyancy
	) {
		PxwPBDBoxFluid* boxFluid = new PxwPBDBoxFluid(
			numX,
			numY,
			numZ,
			position,
			fluidDensity,
			maxDiffuseParticles,
			buoyancy
		);
		boxFluid->SetScene(scene);
		boxFluid->SetParticleSystem(particleSystem, particleSpacing);
		boxFluid->SetMaterial(material);
		boxFluid->Create();
		return boxFluid;
	}

	PxwPBDFluid* PhysXWrapper::CreateFluid(PxScene* scene, PxwPBDParticleSystemHelper* particleSystem, PxPBDMaterial* material, PxVec4* positions, const PxU32 numParticles, const PxReal particleSpacing, const PxReal fluidDensity, const PxU32 maxDiffuseParticles, const PxReal buoyancy)
	{
		PxwPBDFluid* fluid = new PxwPBDFluid(
			positions,
			numParticles,
			fluidDensity,
			maxDiffuseParticles,
			buoyancy
		);
		fluid->SetScene(scene);
		fluid->SetParticleSystem(particleSystem, particleSpacing);
		fluid->SetMaterial(material);
		fluid->Create();
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Add fluid done 2\n");
		return fluid;
	}

	PxwPBDTriMeshCloth* PhysXWrapper::CreateTriMeshCloth(PxScene* scene, PxwPBDParticleSystemHelper* particleSystem, PxPBDMaterial* material, PxVec3* vertices, const int numVertices, int* indices, const int numIndices, const PxVec3 position, const PxReal totalMass, const bool inflatable, const PxReal blendScale, const PxReal pressure, const PxReal particleSpacing)
	{
		PxwPBDTriMeshCloth* cloth = new PxwPBDTriMeshCloth(
			vertices,
			numVertices,
			indices,
			numIndices,
			position,
			totalMass,
			inflatable,
			blendScale,
			pressure
		);
		cloth->SetScene(scene);
		cloth->SetParticleSystem(particleSystem, particleSpacing);
		cloth->SetMaterial(material);
		cloth->Create();

		return cloth;
	}
}