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
		entry->context.frame.lngAxis = static_cast<PxVehicleAxes::Enum>(frame.lngAxis);
		entry->context.frame.latAxis = static_cast<PxVehicleAxes::Enum>(frame.latAxis);
		entry->context.frame.vrtAxis = static_cast<PxVehicleAxes::Enum>(frame.vrtAxis);
		entry->context.scale.scale = frame.scale;
		entry->context.gravity = scene->getGravity();
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
			entry->vehicles[i]->Step(dt, entry->context);
		}
	}

	const PxVehiclePhysXSimulationContext* VehicleGetSceneContext(PxScene* scene)
	{
		SceneVehicles* entry = FindScene(scene);
		return entry ? &entry->context : NULL;
	}
}
