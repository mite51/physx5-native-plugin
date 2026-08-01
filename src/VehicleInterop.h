#pragma once

// Blittable POD descriptors for the PhysX Vehicle2 integration.
//
// These structs are owned by this plugin and are intentionally decoupled from
// the raw PxVehicle* parameter structs so that the interop ABI stays stable
// even if the PhysX vehicle SDK layout changes. The Unity C# side mirrors these
// 1:1 with [StructLayout(LayoutKind.Sequential)].

#include "PxPhysicsAPI.h"
#include "vehicle2/PxVehicleLimits.h"
#include "DataInterop.h"

using namespace physx;
using namespace physx::vehicle2;

namespace pxw
{
	// Mirrors PxVehicleAxes::Enum for the vehicle frame.
	struct PxwVehicleAxis
	{
		enum Enum
		{
			ePosX = 0,
			eNegX = 1,
			ePosY = 2,
			eNegY = 3,
			ePosZ = 4,
			eNegZ = 5
		};
	};

	// How the vehicle is driven.
	struct PxwVehicleDriveMode
	{
		enum Enum
		{
			eDIRECT = 0, // Omniverse PhysxVehicleDriveBasicAPI equivalent
			eENGINE = 1  // Omniverse PhysxVehicleDriveStandardAPI equivalent
		};
	};

	// Differential type for engine-drive vehicles.
	struct PxwVehicleDifferentialType
	{
		enum Enum
		{
			eMULTIWHEEL = 0,
			eFOURWHEEL = 1,
			eTANK = 2
		};
	};

	// Road-geometry query strategy.
	struct PxwVehicleRoadQueryType
	{
		enum Enum
		{
			eNONE = 0,
			eRAYCAST = 1,
			eSWEEP = 2
		};
	};

	// Longitudinal/lateral/vertical axes + length scale for the vehicle frame.
	struct PxwVehicleFrameDesc
	{
		int lngAxis;
		int latAxis;
		int vrtAxis;
		float scale;
	};

	// Chassis rigid body + fallback collision box.
	struct PxwVehicleChassisDesc
	{
		float mass;
		PxVec3 moi;
		PxwTransformData cmassLocalPose;
		// Used only when no explicit chassis geometry is supplied to CreateVehicle.
		PxVec3 boxHalfExtents;
		PxwTransformData boxLocalPose;
	};

	struct PxwVehicleWheelDesc
	{
		float radius;
		float halfWidth;
		float mass;
		float moi;
		float dampingRate;
	};

	struct PxwVehicleSuspensionDesc
	{
		PxwTransformData suspensionAttachment;
		PxVec3 travelDir;
		float travelDist;
		PxwTransformData wheelAttachment;
		float stiffness;
		float damping;
		float sprungMass;
	};

	// Constant (single sample) suspension compliance.
	struct PxwVehicleSuspensionComplianceDesc
	{
		float toeAngle;
		float camberAngle;
		PxVec3 suspForceAppPoint;
		PxVec3 tireForceAppPoint;
	};

	struct PxwVehicleTireDesc
	{
		float latStiffX;
		float latStiffY;
		float longStiff;
		float camberStiff;
		float frictionVsSlip[3][2];
		float restLoad;
		float loadFilter[2][2];
	};

	// Per-wheel response multipliers (index by wheel id) for brake/steer commands.
	struct PxwVehicleBrakeDesc
	{
		float maxResponse;
		float wheelResponseMultipliers[PxVehicleLimits::eMAX_NB_WHEELS];
		int nbWheels;
	};

	struct PxwVehicleSteerDesc
	{
		float maxResponse;
		float wheelResponseMultipliers[PxVehicleLimits::eMAX_NB_WHEELS];
		int nbWheels;
	};

	struct PxwVehicleAckermannDesc
	{
		int wheelIds[2];
		float wheelBase;
		float trackWidth;
		float strength;
		int enabled;
	};

	struct PxwVehicleEngineDesc
	{
		float torqueCurveX[8];
		float torqueCurveY[8];
		int nbTorquePoints;
		float moi;
		float peakTorque;
		float idleOmega;
		float maxOmega;
		float dampingRateFullThrottle;
		float dampingRateZeroThrottleClutchEngaged;
		float dampingRateZeroThrottleClutchDisengaged;
	};

	struct PxwVehicleGearboxDesc
	{
		int neutralGear;
		float ratios[32];
		int nbRatios;
		float finalRatio;
		float switchTime;
	};

	struct PxwVehicleAutoboxDesc
	{
		float upRatios[32];
		float downRatios[32];
		float latency;
	};

	struct PxwVehicleClutchDesc
	{
		int accuracyMode; // 0 = estimate, 1 = best possible
		int estimateIterations;
		float strength; // clutch command response max
	};

	struct PxwVehicleDifferentialDesc
	{
		int type; // PxwVehicleDifferentialType::Enum
		float torqueRatios[PxVehicleLimits::eMAX_NB_WHEELS];
		float aveWheelSpeedRatios[PxVehicleLimits::eMAX_NB_WHEELS];

		// Four-wheel drive specific.
		int frontWheelIds[2];
		int rearWheelIds[2];
		float frontBias;
		float frontTarget;
		float rearBias;
		float rearTarget;
		float centerBias;
		float centerTarget;
		float rate;

		// Tank drive specific.
		int nbTracks;
		int thrustIdPerTrack[PxVehicleLimits::eMAX_NB_WHEELS];
		int nbWheelsPerTrack[PxVehicleLimits::eMAX_NB_WHEELS];
		int trackToWheelIds[PxVehicleLimits::eMAX_NB_WHEELS];
		int wheelIdsInTrackOrder[PxVehicleLimits::eMAX_NB_WHEELS];
	};

	// Per-wheel readback state.
	struct PxwVehicleWheelState
	{
		PxwTransformData localPose;
		float rotationSpeed;
		float rotationAngle;
		float jounce;
		float steerAngle;
	};

	// Whole-vehicle drivetrain readback state.
	struct PxwVehicleDriveState
	{
		float engineRotationSpeed;
		int currentGear;
		int targetGear;
		float clutchSlip;
		float longitudinalSpeed;
		float lateralSpeed;
	};
}
