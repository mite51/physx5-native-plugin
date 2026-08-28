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
		// Local pose of the chassis collision shape, applied to a caller-supplied chassis
		// geometry just as much as to the fallback box, hence not named after the box.
		PxwTransformData shapeLocalPose;
	};

	struct PxwVehicleWheelDesc
	{
		float radius;
		float halfWidth;
		float mass;
		float moi;
		float dampingRate;
	};

	// How a wheel's collision shape gets its geometry.
	struct PxwVehicleWheelGeometryMode
	{
		enum Enum
		{
			// The cooked 16-sided convex prism PxVehiclePhysXActorCreate would have built, from
			// the wheel's radius and half width. Reproduced byte for byte by this plugin so the
			// default configuration is unchanged by the geometry override existing.
			eCOOKED_PRISM = 0,

			// A true cylinder (convex core), sized from the wheel's radius and half width. The
			// prism's facets rotate with the wheel because the vehicle writes the spin angle into
			// the shape's local pose every step, so contact points snap from facet to facet as it
			// turns. A cylinder is rotationally invariant and does not do that, which matters as
			// soon as the wheel is a simulation shape.
			eCYLINDER = 1,

			// A caller-supplied PxGeometry, used as-is.
			eGEOMETRY = 2
		};
	};

	// Per-wheel collision shape configuration, applied when the vehicle is finalized.
	//
	// The defaults reproduce the historical behaviour exactly: a cooked prism that is neither a
	// simulation nor a scene query shape, so wheels collide with nothing and are invisible to
	// raycasts. Vehicle2 drives wheels from suspension raycasts and the tire model, so that is
	// a deliberate default rather than an oversight.
	//
	// Enabling sceneQueryShape is harmless: it lets gameplay raycasts and overlaps hit a wheel
	// without changing anything the solver does.
	//
	// Enabling simulationShape is not harmless on its own. The suspension raycast and tire model
	// already resolve the wheel against the ground, and the wheel's local pose puts its contact
	// patch exactly at the ground surface, so a simulation wheel also generates rigid contacts
	// with the road: the road ends up resolved twice, the vehicle rides high on the doubled
	// normal force and contact friction fights the tire model's steering. Give wheels a collision
	// group that does not collide with the drivable surface (see SetGroupCollisionFlag) so they
	// only meet walls, obstacles and other vehicles.
	struct PxwVehicleWheelShapeDesc
	{
		int geometryMode;      // PxwVehicleWheelGeometryMode::Enum
		int simulationShape;   // non-zero adds PxShapeFlag::eSIMULATION_SHAPE
		int sceneQueryShape;   // non-zero adds PxShapeFlag::eSCENE_QUERY_SHAPE
		float margin;          // convex core margin, used by eCYLINDER
		PxU32 simFilterData[4];
		PxU32 queryFilterData[4];
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
