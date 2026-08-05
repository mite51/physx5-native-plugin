#pragma once

#include "PxPhysicsAPI.h"
#include "VehicleInterop.h"
#include "vehicle/VehicleDirectDrive.h"
#include "vehicle/VehicleEngineDrive.h"

#include <vector>

using namespace physx;

namespace pxw
{
	// PxwVehicle is the single opaque handle the Unity side works with. It owns
	// one direct-drive or engine-drive vehicle (adapted from the NVIDIA snippet
	// classes) and translates the plugin's blittable descriptors into the raw
	// PxVehicle* parameter structs.
	//
	// Lifecycle:
	//   CreateVehicle -> Set* (parameters) -> Finalize -> AddToScene
	//   ... per frame: SetCommands / SetWheelControl, Step, Get* ...
	//   RemoveFromScene -> delete
	class PxwVehicle
	{
	public:
		PxwVehicle(PxScene* scene, PxwVehicleDriveMode::Enum driveMode,
			const PxwVehicleChassisDesc& chassis, const PxGeometry* chassisGeometry, PxMaterial* material);
		~PxwVehicle();

		// --- Setup (call before Finalize) ---
		void SetFrame(const PxwVehicleFrameDesc& frame);
		void SetAxleDescription(int nbAxles, const int* nbWheelsPerAxle, const int* wheelIdsInAxleOrder);
		void SetWheel(int wheelId, const PxwVehicleWheelDesc& d);
		void SetSuspension(int wheelId, const PxwVehicleSuspensionDesc& d);
		void SetSuspensionCompliance(int wheelId, const PxwVehicleSuspensionComplianceDesc& d);
		void SetTire(int wheelId, const PxwVehicleTireDesc& d);
		void SetBrake(int brakeSet, const PxwVehicleBrakeDesc& d);
		void SetSteer(const PxwVehicleSteerDesc& d);
		void SetAckermann(const PxwVehicleAckermannDesc& d);
		void SetDifferential(const PxwVehicleDifferentialDesc& d);
		void SetEngine(const PxwVehicleEngineDesc& d);
		void SetGearbox(const PxwVehicleGearboxDesc& d);
		void SetAutobox(const PxwVehicleAutoboxDesc& d);
		void SetClutch(const PxwVehicleClutchDesc& d);
		void SetTireFriction(PxMaterial** materials, float* frictions, int count, float defaultFriction);
		void SetRoadQueryType(PxwVehicleRoadQueryType::Enum type);
		void SetUseDirectWheelControl(bool use);
		// Direct-drive throttle -> per-wheel drive torque response (direct drive only).
		void SetDirectDriveThrottle(float maxResponse, const float* wheelResponseMultipliers, int nbWheels);

		bool Finalize(PxPhysics* physics, const PxCookingParams& cooking, PxMaterial* defaultMaterial);
		bool IsFinalized() const { return mFinalized; }
		PxScene* Scene() const { return mScene; }

		void AddToScene();
		// wakeOnLostTouch defaults to true to match PxScene::removeActor. A synchronised
		// rebuild passes false so tearing the chassis out does not perturb the sleep state
		// it is about to restore.
		void RemoveFromScene(bool wakeOnLostTouch = true);

		// --- Control (call after Finalize) ---
		void SetCommands(float brake0, float brake1, float throttle, float steer);
		void SetTransmissionCommand(int targetGear, float clutch);
		void SetTankThrusts(float thrust0, float thrust1);
		void SetWheelControl(int wheelId, float driveTorque, float brakeTorque, float steerAngle);

		// --- Step + readback ---
		void Step(float dt, const PxVehiclePhysXSimulationContext& context);
		void GetRigidBodyPose(PxwTransformData* dest);
		void GetWheelStates(PxwVehicleWheelState* dest, int length);
		void GetDriveState(PxwVehicleDriveState* dest);
		PxRigidBody* GetActor();

		// --- Rollback integrator state ---
		// Only the state the vehicle integrates over time is (de)serialised here:
		// per wheel, the 1D wheel rigid-body state, the suspension state and the
		// sticky-tire timers; for engine drive, the engine, gearbox, autobox and
		// clutch states. Everything else -- road geometry, tire slip/force, wheel
		// poses, command responses, differential and constraint state -- is recomputed
		// each step from params, road geometry and commands, so it is deliberately
		// excluded: capturing it is waste, and it cannot desync. Per-tick commands
		// (throttle, brake, steer, gear) are input and flow through the input buffer,
		// not this snapshot. The chassis rigid body is captured separately, through its
		// PxRigidActor, the same as any other dynamic.
		PxwVehicleDriveMode::Enum GetDriveMode() const { return mDriveMode; }
		PxU32 GetWheelCount() const;
		PxU32 SnapshotSize() const;
		bool CaptureSnapshot(void* dst, PxU32 capacity) const;
		bool RestoreSnapshot(const void* src, PxU32 size);

	private:
		PhysXActorVehicle* ActorVehicle();
		const PhysXActorVehicle* ActorVehicleConst() const;
		BaseVehicleParams& Base();
		PhysXIntegrationParams& PhysXParams();
		void SetupCommandResponseDefaults(PxVehicleCommandResponseParams& params);

		PxScene* mScene;
		PxwVehicleDriveMode::Enum mDriveMode;
		PxwVehicleChassisDesc mChassis;
		const PxGeometry* mChassisGeometry;
		PxMaterial* mMaterial;

		PxwDirectDriveVehicle* mDirect;
		PxwEngineDriveVehicle* mEngine;

		std::vector<PxVehiclePhysXMaterialFriction> mMaterialFrictions;
		float mDefaultFriction;

		PxwVehicleRoadQueryType::Enum mRoadQueryType;
		PxwVehicleDifferentialType::Enum mDiffType;
		bool mUseDirectWheelControl;
		bool mFinalized;
		bool mInScene;
	};

	// Bytes a rollback snapshot occupies for a vehicle of this drive mode and wheel
	// count. Exposed as a free function so the UNDPWR registry can size a vehicle's
	// payload from the counts it caches at register time, without having to hold or
	// dereference the vehicle -- the same way an articulation is sized from its
	// cached DOF count. Must stay in step with PxwVehicle::CaptureSnapshot.
	PxU32 PxwVehicleSnapshotSize(PxwVehicleDriveMode::Enum driveMode, PxU32 nbWheels);
}
