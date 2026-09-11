#include "VehicleModule.h"
#include "VehicleHelper.h"

#include <vector>

namespace pxw
{
	namespace
	{
		struct SceneVehicles
		{
			PxScene* scene;
			PxVehiclePhysXSimulationContext context;
			std::vector<PxwVehicle*> vehicles;
			// Substep policy shared by every vehicle in the scene. Defaults match the
			// historical hardcoded substep group count of 3, applied at every speed.
			PxU8 lowSubstepCount = 3;
			PxU8 highSubstepCount = 3;
			PxReal substepThresholdSpeed = 5.0f;
			// Set true once the first vehicle is registered. The context and substep policy
			// are immutable from then on so a finalized vehicle's context cannot change.
			bool contextLocked = false;
		};

		bool gInitialized = false;
		PxPhysics* gPhysics = NULL;
		PxConvexMesh* gUnitCylinderSweepMesh = NULL;
		std::vector<SceneVehicles> gScenes;

		SceneVehicles* FindScene(PxScene* scene)
		{
			for (size_t i = 0; i < gScenes.size(); i++)
			{
				if (gScenes[i].scene == scene)
					return &gScenes[i];
			}
			return NULL;
		}

		void ApplyUnityFrame(PxVehicleFrame& frame)
		{
			frame.lngAxis = PxVehicleAxes::ePosZ;
			frame.latAxis = PxVehicleAxes::ePosX;
			frame.vrtAxis = PxVehicleAxes::ePosY;
		}
	}

	void VehicleInit(PxFoundation* foundation, PxPhysics* physics)
	{
		if (gInitialized || !foundation || !physics)
			return;

		if (!PxInitVehicleExtension(*foundation))
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
				"PxInitVehicleExtension failed\n");
			return;
		}

		gPhysics = physics;

		// The unit cylinder sweep mesh is required when suspension sweeps are used
		// for wheel vs. ground queries. It is shared across all scenes.
		PxTolerancesScale scale = physics->getTolerancesScale();
		PxCookingParams cooking(scale);

		PxVehicleFrame frame;
		frame.setToDefault();
		ApplyUnityFrame(frame);

		gUnitCylinderSweepMesh = PxVehicleUnitCylinderSweepMeshCreate(frame, *physics, cooking);

		gInitialized = true;
	}

	void VehicleCleanup()
	{
		if (!gInitialized)
			return;

		gScenes.clear();

		if (gUnitCylinderSweepMesh)
		{
			PxVehicleUnitCylinderSweepMeshDestroy(gUnitCylinderSweepMesh);
			gUnitCylinderSweepMesh = NULL;
		}

		PxCloseVehicleExtension();

		gPhysics = NULL;
		gInitialized = false;
	}

	void VehicleRegisterScene(PxScene* scene)
	{
		if (!gInitialized || !scene || FindScene(scene))
			return;

		SceneVehicles entry;
		entry.scene = scene;
		entry.context.setToDefault();
		ApplyUnityFrame(entry.context.frame);
		entry.context.scale.scale = 1.0f;
		entry.context.gravity = scene->getGravity();
		entry.context.physxScene = scene;
		entry.context.physxActorUpdateMode = PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION;
		entry.context.physxUnitCylinderSweepMesh = gUnitCylinderSweepMesh;
		// Larger lateral sticky damping avoids drift when nearly at rest.
		entry.context.tireStickyParams.stickyParams[PxVehicleTireDirectionModes::eLATERAL].damping = 1.0f;

		gScenes.push_back(entry);
	}

	void VehicleUnregisterScene(PxScene* scene)
	{
		for (size_t i = 0; i < gScenes.size(); i++)
		{
			if (gScenes[i].scene == scene)
			{
				gScenes.erase(gScenes.begin() + i);
				return;
			}
		}
	}

	void VehicleSetSceneFrame(PxScene* scene, const PxwVehicleFrameDesc& frame)
	{
		SceneVehicles* entry = FindScene(scene);
		if (!entry)
			return;
		// Once a vehicle is registered the scene frame is part of the frozen context; a later
		// vehicle must not redefine the axes/scale the earlier one was finalized against.
		if (entry->contextLocked)
			return;
		entry->context.frame.lngAxis = static_cast<PxVehicleAxes::Enum>(frame.lngAxis);
		entry->context.frame.latAxis = static_cast<PxVehicleAxes::Enum>(frame.latAxis);
		entry->context.frame.vrtAxis = static_cast<PxVehicleAxes::Enum>(frame.vrtAxis);
		entry->context.scale.scale = frame.scale;
		entry->context.gravity = scene->getGravity();
	}

	bool VehicleSetSceneContext(PxScene* scene, const PxwVehicleSceneContextDesc& desc)
	{
		SceneVehicles* entry = FindScene(scene);
		if (!entry)
			return false;

		if (entry->contextLocked)
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
				"VehicleSetSceneContext ignored: a vehicle is already registered against this "
				"scene, so its context is immutable. Configure the scene context before adding "
				"vehicles.\n");
			return false;
		}

		entry->context.physxActorUpdateMode = (desc.physxActorUpdateMode == 1)
			? PxVehiclePhysXActorUpdateMode::eAPPLY_ACCELERATION
			: PxVehiclePhysXActorUpdateMode::eAPPLY_VELOCITY;

		// A non-positive denominator means "keep the PhysX default".
		if (desc.minActiveLongSlipDenominator > 0.0f)
			entry->context.tireSlipParams.minActiveLongSlipDenominator = desc.minActiveLongSlipDenominator;
		if (desc.minPassiveLongSlipDenominator > 0.0f)
			entry->context.tireSlipParams.minPassiveLongSlipDenominator = desc.minPassiveLongSlipDenominator;
		if (desc.minLatSlipDenominator > 0.0f)
			entry->context.tireSlipParams.minLatSlipDenominator = desc.minLatSlipDenominator;

		if (desc.lowSubstepCount > 0)
			entry->lowSubstepCount = static_cast<PxU8>(desc.lowSubstepCount);
		if (desc.highSubstepCount > 0)
			entry->highSubstepCount = static_cast<PxU8>(desc.highSubstepCount);
		if (desc.substepThresholdSpeed > 0.0f)
			entry->substepThresholdSpeed = desc.substepThresholdSpeed;

		return true;
	}

	PxwSceneSubstepPolicy VehicleGetSceneSubstepPolicy(PxScene* scene)
	{
		PxwSceneSubstepPolicy policy = { 3, 3, 5.0f };
		SceneVehicles* entry = FindScene(scene);
		if (entry)
		{
			policy.lowSubstepCount = entry->lowSubstepCount;
			policy.highSubstepCount = entry->highSubstepCount;
			policy.thresholdSpeed = entry->substepThresholdSpeed;
		}
		return policy;
	}

	void VehicleRegister(PxScene* scene, PxwVehicle* vehicle)
	{
		SceneVehicles* entry = FindScene(scene);
		if (!entry || !vehicle)
			return;
		for (size_t i = 0; i < entry->vehicles.size(); i++)
		{
			if (entry->vehicles[i] == vehicle)
				return;
		}
		// From the first vehicle onward the scene context is frozen.
		entry->contextLocked = true;
		entry->vehicles.push_back(vehicle);
	}

	void VehicleUnregister(PxwVehicle* vehicle)
	{
		for (size_t s = 0; s < gScenes.size(); s++)
		{
			std::vector<PxwVehicle*>& v = gScenes[s].vehicles;
			for (size_t i = 0; i < v.size(); i++)
			{
				if (v[i] == vehicle)
				{
					v.erase(v.begin() + i);
					return;
				}
			}
		}
	}

	void VehicleStepScene(PxScene* scene, PxReal dt)
	{
		SceneVehicles* entry = FindScene(scene);
		if (!entry)
			return;
		// Keep gravity in sync in case it was changed on the scene.
		entry->context.gravity = scene->getGravity();
		for (size_t i = 0; i < entry->vehicles.size(); i++)
		{
			// A vehicle parked through the UNDPWR registry has eDISABLE_SIMULATION set on
			// its chassis; stepping its drivetrain while it is out of play would advance
			// state a snapshot no longer tracks, so skip it here as well.
			PxRigidBody* chassis = entry->vehicles[i]->GetActor();
			if (chassis != NULL && (chassis->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			{
				continue;
			}
			entry->vehicles[i]->SetSubstepPolicy(
				entry->lowSubstepCount, entry->highSubstepCount, entry->substepThresholdSpeed);
			entry->vehicles[i]->Step(dt, entry->context);
		}
	}

	const PxVehiclePhysXSimulationContext* VehicleGetSceneContext(PxScene* scene)
	{
		SceneVehicles* entry = FindScene(scene);
		return entry ? &entry->context : NULL;
	}
}
