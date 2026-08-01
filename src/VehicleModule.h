#pragma once

// Global + per-scene management for the PhysX Vehicle2 integration.
//
// This module owns the vehicle SDK extension lifetime, the shared unit-cylinder
// sweep mesh and, for each PhysX scene, a PxVehiclePhysXSimulationContext plus
// the list of vehicles that must be stepped before that scene simulates.

#include "PxPhysicsAPI.h"
#include "vehicle2/PxVehicleAPI.h"
#include "VehicleInterop.h"

using namespace physx;
using namespace physx::vehicle2;

namespace pxw
{
	class PxwVehicle;

	// Global lifecycle. Safe to call multiple times; only the first has effect.
	void VehicleInit(PxFoundation* foundation, PxPhysics* physics);
	void VehicleCleanup();

	// Per-scene simulation context.
	void VehicleRegisterScene(PxScene* scene);
	void VehicleUnregisterScene(PxScene* scene);
	void VehicleSetSceneFrame(PxScene* scene, const PxwVehicleFrameDesc& frame);

	// Vehicle registry used for stepping.
	void VehicleRegister(PxScene* scene, PxwVehicle* vehicle);
	void VehicleUnregister(PxwVehicle* vehicle);

	// Step all vehicles registered against a scene using that scene's context.
	// Must be called before PxScene::simulate for correct ordering.
	void VehicleStepScene(PxScene* scene, PxReal dt);

	// Access the simulation context for a scene (NULL if none). Exposed so the C
	// API can drive a single vehicle step outside the scene loop if needed.
	const PxVehiclePhysXSimulationContext* VehicleGetSceneContext(PxScene* scene);
}
