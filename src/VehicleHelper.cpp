#include "VehicleHelper.h"

#include <cstring>

namespace pxw
{
	namespace
	{
		// The per-wheel and per-vehicle blocks of a rollback snapshot. Each holds only
		// integrator state -- values the simulation carries from one step to the next
		// and cannot recompute. The wrappers are zeroed before use so their padding is
		// a constant, which keeps the per-entry hash reproducible across peers.
		struct VehicleWheelSnapshot
		{
			PxVehicleWheelRigidBody1dState wheel;      // rotation angle and speed
			PxVehicleSuspensionState       suspension; // jounce, jounce speed, separation
			PxVehicleTireStickyState       sticky;     // time-below-threshold accumulator
		};

		struct VehicleEngineSnapshot
		{
			PxVehicleEngineState     engine;   // engine rotation speed
			PxVehicleGearboxState    gearbox;  // current/target gear and in-progress shift timer
			PxVehicleAutoboxState    autobox;  // time since last automatic shift
			PxVehicleClutchSlipState clutch;   // clutch slip
		};
	}

	PxU32 PxwVehicleSnapshotSize(PxwVehicleDriveMode::Enum driveMode, PxU32 nbWheels)
	{
		PxU32 size = nbWheels * static_cast<PxU32>(sizeof(VehicleWheelSnapshot));
		if (driveMode == PxwVehicleDriveMode::eENGINE)
			size += static_cast<PxU32>(sizeof(VehicleEngineSnapshot));
		return size;
	}

	static PxVehicleAxes::Enum ToPxAxis(int axis)
	{
		return static_cast<PxVehicleAxes::Enum>(axis);
	}

	PxwVehicle::PxwVehicle(PxScene* scene, PxwVehicleDriveMode::Enum driveMode,
		const PxwVehicleChassisDesc& chassis, const PxGeometry* chassisGeometry, PxMaterial* material)
		: mScene(scene)
		, mDriveMode(driveMode)
		, mChassis(chassis)
		, mChassisGeometry(chassisGeometry)
		, mMaterial(material)
		, mDirect(NULL)
		, mEngine(NULL)
		, mDefaultFriction(1.0f)
		, mRoadQueryType(PxwVehicleRoadQueryType::eRAYCAST)
		, mDiffType(PxwVehicleDifferentialType::eMULTIWHEEL)
		, mUseDirectWheelControl(false)
		, mFinalized(false)
		, mInScene(false)
	{
		if (driveMode == PxwVehicleDriveMode::eENGINE)
			mEngine = new PxwEngineDriveVehicle();
		else
			mDirect = new PxwDirectDriveVehicle();

		BaseVehicleParams& base = Base();

		// Frame (Unity convention by default: lng = +Z, lat = +X, vrt = +Y).
		base.frame.lngAxis = PxVehicleAxes::ePosZ;
		base.frame.latAxis = PxVehicleAxes::ePosX;
		base.frame.vrtAxis = PxVehicleAxes::ePosY;
		base.scale.scale = 1.0f;

		base.suspensionStateCalculationParams.suspensionJounceCalculationType = PxVehicleSuspensionJounceCalculationType::eRAYCAST;
		base.suspensionStateCalculationParams.limitSuspensionExpansionVelocity = false;

		base.rigidBodyParams.mass = chassis.mass;
		base.rigidBodyParams.moi = chassis.moi;

		base.axleDescription.setToDefault();

		SetupCommandResponseDefaults(base.brakeResponseParams[0]);
		SetupCommandResponseDefaults(base.brakeResponseParams[1]);
		SetupCommandResponseDefaults(base.steerResponseParams);

		// A single, disabled Ackermann correction by default.
		base.ackermannParams[0].wheelIds[0] = 0;
		base.ackermannParams[0].wheelIds[1] = 1;
		base.ackermannParams[0].wheelBase = 1.0f;
		base.ackermannParams[0].trackWidth = 1.0f;
		base.ackermannParams[0].strength = 0.0f;

		ActorVehicle()->mCommandState.setToDefault();

		// Fallback PhysX integration params (cmass + box shape) come from the chassis
		// descriptor; these are overwritten in Finalize() before actor creation.
		PhysXIntegrationParams& px = PhysXParams();
		px.physxActorCMassLocalPose = chassis.cmassLocalPose.ToPxTransform();
		px.physxActorBoxShapeHalfExtents = chassis.boxHalfExtents;
		px.physxActorBoxShapeLocalPose = chassis.boxLocalPose.ToPxTransform();

		if (mDirect)
		{
			mDirect->mTransmissionCommandState.setToDefault();
			// Sensible default so a minimally configured direct-drive vehicle finalizes.
			mDirect->mDirectDriveParams.directDriveThrottleResponseParams.nonlinearResponse.clear();
			mDirect->mDirectDriveParams.directDriveThrottleResponseParams.maxResponse = 0.0f;
			for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
				mDirect->mDirectDriveParams.directDriveThrottleResponseParams.wheelResponseMultipliers[i] = 0.0f;
		}

		if (mEngine)
		{
			mEngine->mTransmissionCommandState.setToDefault();
			mEngine->mTankDriveTransmissionCommandState.setToDefault();

			EngineDrivetrainParams& e = mEngine->mEngineDriveParams;

			// Engine.
			e.engineParams.moi = 1.0f;
			e.engineParams.peakTorque = 500.0f;
			e.engineParams.idleOmega = 0.0f;
			e.engineParams.maxOmega = 600.0f;
			e.engineParams.dampingRateFullThrottle = 0.15f;
			e.engineParams.dampingRateZeroThrottleClutchEngaged = 2.0f;
			e.engineParams.dampingRateZeroThrottleClutchDisengaged = 0.35f;
			e.engineParams.torqueCurve.clear();
			e.engineParams.torqueCurve.addPair(0.0f, 0.8f);
			e.engineParams.torqueCurve.addPair(0.33f, 1.0f);
			e.engineParams.torqueCurve.addPair(1.0f, 0.8f);

			// Gearbox: reverse, neutral, first.
			e.gearBoxParams.neutralGear = 1;
			for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
				e.gearBoxParams.ratios[i] = 0.0f;
			e.gearBoxParams.ratios[0] = -4.0f;
			e.gearBoxParams.ratios[1] = 0.0f;
			e.gearBoxParams.ratios[2] = 4.0f;
			e.gearBoxParams.nbRatios = 3;
			e.gearBoxParams.finalRatio = 4.0f;
			e.gearBoxParams.switchTime = 0.5f;

			// Autobox.
			for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
			{
				e.autoboxParams.upRatios[i] = 0.65f;
				e.autoboxParams.downRatios[i] = 0.5f;
			}
			e.autoboxParams.latency = 2.0f;

			// Clutch.
			e.clutchParams.accuracyMode = PxVehicleClutchAccuracyMode::eESTIMATE;
			e.clutchParams.estimateIterations = 5;
			e.clutchCommandResponseParams.maxResponse = 10.0f;
		}
	}

	PxwVehicle::~PxwVehicle()
	{
		if (mInScene)
			RemoveFromScene();

		if (mFinalized)
		{
			ActorVehicle()->destroy();
		}

		delete mDirect;
		delete mEngine;
	}

	PhysXActorVehicle* PxwVehicle::ActorVehicle()
	{
		if (mDirect) return mDirect;
		return mEngine;
	}

	const PhysXActorVehicle* PxwVehicle::ActorVehicleConst() const
	{
		if (mDirect) return mDirect;
		return mEngine;
	}

	BaseVehicleParams& PxwVehicle::Base()
	{
		return ActorVehicle()->mBaseParams;
	}

	PhysXIntegrationParams& PxwVehicle::PhysXParams()
	{
		return ActorVehicle()->mPhysXParams;
	}

	void PxwVehicle::SetupCommandResponseDefaults(PxVehicleCommandResponseParams& params)
	{
		params.nonlinearResponse.clear();
		for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
			params.wheelResponseMultipliers[i] = 0.0f;
		params.maxResponse = 0.0f;
	}

	void PxwVehicle::SetFrame(const PxwVehicleFrameDesc& frame)
	{
		BaseVehicleParams& base = Base();
		base.frame.lngAxis = ToPxAxis(frame.lngAxis);
		base.frame.latAxis = ToPxAxis(frame.latAxis);
		base.frame.vrtAxis = ToPxAxis(frame.vrtAxis);
		base.scale.scale = frame.scale;
	}

	void PxwVehicle::SetAxleDescription(int nbAxles, const int* nbWheelsPerAxle, const int* wheelIdsInAxleOrder)
	{
		PxVehicleAxleDescription& axle = Base().axleDescription;
		axle.setToDefault();
		int offset = 0;
		for (int a = 0; a < nbAxles; a++)
		{
			PxU32 ids[PxVehicleLimits::eMAX_NB_WHEELS];
			const int count = nbWheelsPerAxle[a];
			for (int w = 0; w < count; w++)
				ids[w] = static_cast<PxU32>(wheelIdsInAxleOrder[offset++]);
			axle.addAxle(static_cast<PxU32>(count), ids);
		}
	}

	void PxwVehicle::SetWheel(int wheelId, const PxwVehicleWheelDesc& d)
	{
		PxVehicleWheelParams& w = Base().wheelParams[wheelId];
		w.radius = d.radius;
		w.halfWidth = d.halfWidth;
		w.mass = d.mass;
		w.moi = d.moi;
		w.dampingRate = d.dampingRate;
	}

	void PxwVehicle::SetSuspension(int wheelId, const PxwVehicleSuspensionDesc& d)
	{
		PxVehicleSuspensionParams& s = Base().suspensionParams[wheelId];
		s.suspensionAttachment = d.suspensionAttachment.ToPxTransform();
		s.suspensionTravelDir = d.travelDir;
		s.suspensionTravelDist = d.travelDist;
		s.wheelAttachment = d.wheelAttachment.ToPxTransform();

		PxVehicleSuspensionForceParams& f = Base().suspensionForceParams[wheelId];
		f.stiffness = d.stiffness;
		f.damping = d.damping;
		f.sprungMass = d.sprungMass;
	}

	void PxwVehicle::SetSuspensionCompliance(int wheelId, const PxwVehicleSuspensionComplianceDesc& d)
	{
		PxVehicleSuspensionComplianceParams& c = Base().suspensionComplianceParams[wheelId];
		c.wheelToeAngle.clear();
		c.wheelToeAngle.addPair(0.0f, d.toeAngle);
		c.wheelCamberAngle.clear();
		c.wheelCamberAngle.addPair(0.0f, d.camberAngle);
		c.suspForceAppPoint.clear();
		c.suspForceAppPoint.addPair(0.0f, d.suspForceAppPoint);
		c.tireForceAppPoint.clear();
		c.tireForceAppPoint.addPair(0.0f, d.tireForceAppPoint);
	}

	void PxwVehicle::SetTire(int wheelId, const PxwVehicleTireDesc& d)
	{
		PxVehicleTireForceParams& t = Base().tireForceParams[wheelId];
		t.latStiffX = d.latStiffX;
		t.latStiffY = d.latStiffY;
		t.longStiff = d.longStiff;
		t.camberStiff = d.camberStiff;
		t.restLoad = d.restLoad;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 2; j++)
				t.frictionVsSlip[i][j] = d.frictionVsSlip[i][j];
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 2; j++)
				t.loadFilter[i][j] = d.loadFilter[i][j];
	}

	void PxwVehicle::SetBrake(int brakeSet, const PxwVehicleBrakeDesc& d)
	{
		if (brakeSet < 0 || brakeSet > 1)
			return;
		PxVehicleBrakeCommandResponseParams& p = Base().brakeResponseParams[brakeSet];
		p.nonlinearResponse.clear();
		p.maxResponse = d.maxResponse;
		for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
			p.wheelResponseMultipliers[i] = (static_cast<int>(i) < d.nbWheels) ? d.wheelResponseMultipliers[i] : 0.0f;
	}

	void PxwVehicle::SetSteer(const PxwVehicleSteerDesc& d)
	{
		PxVehicleSteerCommandResponseParams& p = Base().steerResponseParams;
		p.nonlinearResponse.clear();
		p.maxResponse = d.maxResponse;
		for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
			p.wheelResponseMultipliers[i] = (static_cast<int>(i) < d.nbWheels) ? d.wheelResponseMultipliers[i] : 0.0f;
	}

	void PxwVehicle::SetAckermann(const PxwVehicleAckermannDesc& d)
	{
		PxVehicleAckermannParams& a = Base().ackermannParams[0];
		a.wheelIds[0] = static_cast<PxU32>(d.wheelIds[0]);
		a.wheelIds[1] = static_cast<PxU32>(d.wheelIds[1]);
		a.wheelBase = d.wheelBase;
		a.trackWidth = d.trackWidth;
		a.strength = d.enabled ? d.strength : 0.0f;
	}

	void PxwVehicle::SetDifferential(const PxwVehicleDifferentialDesc& d)
	{
		if (!mEngine)
			return;

		mDiffType = static_cast<PxwVehicleDifferentialType::Enum>(d.type);
		EngineDrivetrainParams& e = mEngine->mEngineDriveParams;

		// Copy the wheel torque/speed split into every differential variant so the
		// EngineDrivetrainParams::isValid() check (which validates all three) passes.
		for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
		{
			e.multiWheelDifferentialParams.torqueRatios[i] = d.torqueRatios[i];
			e.multiWheelDifferentialParams.aveWheelSpeedRatios[i] = d.aveWheelSpeedRatios[i];
			e.fourWheelDifferentialParams.torqueRatios[i] = d.torqueRatios[i];
			e.fourWheelDifferentialParams.aveWheelSpeedRatios[i] = d.aveWheelSpeedRatios[i];
			e.tankDifferentialParams.torqueRatios[i] = d.torqueRatios[i];
			e.tankDifferentialParams.aveWheelSpeedRatios[i] = d.aveWheelSpeedRatios[i];
		}

		// Four-wheel specific. Deactivated (bias == 0) unless four-wheel drive is used.
		PxVehicleFourWheelDriveDifferentialParams& f = e.fourWheelDifferentialParams;
		if (mDiffType == PxwVehicleDifferentialType::eFOURWHEEL)
		{
			f.frontWheelIds[0] = static_cast<PxU32>(d.frontWheelIds[0]);
			f.frontWheelIds[1] = static_cast<PxU32>(d.frontWheelIds[1]);
			f.rearWheelIds[0] = static_cast<PxU32>(d.rearWheelIds[0]);
			f.rearWheelIds[1] = static_cast<PxU32>(d.rearWheelIds[1]);
			f.frontBias = d.frontBias;
			f.frontTarget = d.frontTarget;
			f.rearBias = d.rearBias;
			f.rearTarget = d.rearTarget;
			f.centerBias = d.centerBias;
			f.centerTarget = d.centerTarget;
			f.rate = d.rate;
		}
		else
		{
			f.frontBias = 0.0f;
			f.frontTarget = 0.0f;
			f.rearBias = 0.0f;
			f.rearTarget = 0.0f;
			f.centerBias = 0.0f;
			f.centerTarget = 0.0f;
			f.rate = 0.0f;
			f.frontWheelIds[0] = 0;
			f.frontWheelIds[1] = 1;
			f.rearWheelIds[0] = 2;
			f.rearWheelIds[1] = 3;
		}

		// Tank specific.
		PxVehicleTankDriveDifferentialParams& t = e.tankDifferentialParams;
		t.nbTracks = 0;
		if (mDiffType == PxwVehicleDifferentialType::eTANK)
		{
			for (int track = 0; track < d.nbTracks; track++)
			{
				PxU32 wheelIds[PxVehicleLimits::eMAX_NB_WHEELS];
				const int nbInTrack = d.nbWheelsPerTrack[track];
				const int base = d.trackToWheelIds[track];
				for (int w = 0; w < nbInTrack; w++)
					wheelIds[w] = static_cast<PxU32>(d.wheelIdsInTrackOrder[base + w]);
				t.addTankTrack(static_cast<PxU32>(nbInTrack), wheelIds, static_cast<PxU32>(d.thrustIdPerTrack[track]));
			}
		}
	}

	void PxwVehicle::SetEngine(const PxwVehicleEngineDesc& d)
	{
		if (!mEngine)
			return;
		PxVehicleEngineParams& e = mEngine->mEngineDriveParams.engineParams;
		e.moi = d.moi;
		e.peakTorque = d.peakTorque;
		e.idleOmega = d.idleOmega;
		e.maxOmega = d.maxOmega;
		e.dampingRateFullThrottle = d.dampingRateFullThrottle;
		e.dampingRateZeroThrottleClutchEngaged = d.dampingRateZeroThrottleClutchEngaged;
		e.dampingRateZeroThrottleClutchDisengaged = d.dampingRateZeroThrottleClutchDisengaged;
		e.torqueCurve.clear();
		for (int i = 0; i < d.nbTorquePoints && i < 8; i++)
			e.torqueCurve.addPair(d.torqueCurveX[i], d.torqueCurveY[i]);
	}

	void PxwVehicle::SetGearbox(const PxwVehicleGearboxDesc& d)
	{
		if (!mEngine)
			return;
		PxVehicleGearboxParams& g = mEngine->mEngineDriveParams.gearBoxParams;
		g.neutralGear = static_cast<PxU32>(d.neutralGear);
		g.nbRatios = static_cast<PxU32>(d.nbRatios);
		g.finalRatio = d.finalRatio;
		g.switchTime = d.switchTime;
		for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
			g.ratios[i] = (static_cast<int>(i) < d.nbRatios) ? d.ratios[i] : 0.0f;
	}

	void PxwVehicle::SetAutobox(const PxwVehicleAutoboxDesc& d)
	{
		if (!mEngine)
			return;
		PxVehicleAutoboxParams& a = mEngine->mEngineDriveParams.autoboxParams;
		for (PxU32 i = 0; i < PxVehicleGearboxParams::eMAX_NB_GEARS; i++)
		{
			a.upRatios[i] = d.upRatios[i];
			a.downRatios[i] = d.downRatios[i];
		}
		a.latency = d.latency;
	}

	void PxwVehicle::SetClutch(const PxwVehicleClutchDesc& d)
	{
		if (!mEngine)
			return;
		EngineDrivetrainParams& e = mEngine->mEngineDriveParams;
		e.clutchParams.accuracyMode = (d.accuracyMode == 1)
			? PxVehicleClutchAccuracyMode::eBEST_POSSIBLE
			: PxVehicleClutchAccuracyMode::eESTIMATE;
		e.clutchParams.estimateIterations = static_cast<PxU32>(d.estimateIterations > 0 ? d.estimateIterations : 1);
		e.clutchCommandResponseParams.maxResponse = d.strength;
	}

	void PxwVehicle::SetTireFriction(PxMaterial** materials, float* frictions, int count, float defaultFriction)
	{
		mDefaultFriction = defaultFriction;
		mMaterialFrictions.clear();
		for (int i = 0; i < count; i++)
		{
			PxVehiclePhysXMaterialFriction mf;
			mf.material = materials[i];
			mf.friction = frictions[i];
			mMaterialFrictions.push_back(mf);
		}
	}

	void PxwVehicle::SetRoadQueryType(PxwVehicleRoadQueryType::Enum type)
	{
		mRoadQueryType = type;
	}

	void PxwVehicle::SetUseDirectWheelControl(bool use)
	{
		mUseDirectWheelControl = use;
		if (mDirect)
			mDirect->mUseDirectWheelControl = use;
	}

	void PxwVehicle::SetDirectDriveThrottle(float maxResponse, const float* wheelResponseMultipliers, int nbWheels)
	{
		if (!mDirect)
			return;
		PxVehicleDirectDriveThrottleCommandResponseParams& p =
			mDirect->mDirectDriveParams.directDriveThrottleResponseParams;
		p.nonlinearResponse.clear();
		p.maxResponse = maxResponse;
		for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
			p.wheelResponseMultipliers[i] =
				(static_cast<int>(i) < nbWheels && wheelResponseMultipliers) ? wheelResponseMultipliers[i] : 0.0f;
	}

	bool PxwVehicle::Finalize(PxPhysics* physics, const PxCookingParams& cooking, PxMaterial* defaultMaterial)
	{
		if (mFinalized)
			return true;

		BaseVehicleParams& base = Base();

		// Populate the PhysX integration params (road geometry query + per-wheel
		// material friction + suspension limit constraints).
		PhysXIntegrationParams& px = PhysXParams();
		const PxQueryFilterData queryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC);
		PxVehiclePhysXMaterialFriction* frictions = mMaterialFrictions.empty() ? NULL : mMaterialFrictions.data();
		const PxU32 nbFrictions = static_cast<PxU32>(mMaterialFrictions.size());

		px.create(
			base.axleDescription,
			queryFilterData, NULL,
			frictions, nbFrictions, mDefaultFriction,
			px.physxActorCMassLocalPose,
			px.physxActorBoxShapeHalfExtents, px.physxActorBoxShapeLocalPose);

		// Apply the requested road-geometry query type (create() defaults to raycast).
		switch (mRoadQueryType)
		{
		case PxwVehicleRoadQueryType::eNONE:
			px.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eNONE;
			break;
		case PxwVehicleRoadQueryType::eSWEEP:
			px.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eSWEEP;
			break;
		case PxwVehicleRoadQueryType::eRAYCAST:
		default:
			px.physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
			break;
		}

		PxMaterial* mat = mMaterial ? mMaterial : defaultMaterial;

		bool ok = false;
		if (mEngine)
		{
			EngineDriveVehicle::Enum diff = EngineDriveVehicle::eDIFFTYPE_MULTIWHEELDRIVE;
			if (mDiffType == PxwVehicleDifferentialType::eFOURWHEEL)
				diff = EngineDriveVehicle::eDIFFTYPE_FOURWHEELDRIVE;
			else if (mDiffType == PxwVehicleDifferentialType::eTANK)
				diff = EngineDriveVehicle::eDIFFTYPE_TANKDRIVE;

			ok = mEngine->initialize(*physics, cooking, *mat, diff, true, mChassisGeometry);
		}
		else
		{
			// Direct-drive: when the host drives the wheels directly the PhysX
			// begin/end components are still required; only the command-response
			// component is skipped (handled via mUseDirectWheelControl).
			ok = mDirect->initialize(*physics, cooking, *mat, true, mChassisGeometry);
		}

		mFinalized = ok;
		return ok;
	}

	void PxwVehicle::AddToScene()
	{
		if (!mFinalized || mInScene)
			return;
		PxRigidBody* body = GetActor();
		if (body)
		{
			mScene->addActor(*body);
			mInScene = true;
		}
	}

	void PxwVehicle::RemoveFromScene(bool wakeOnLostTouch)
	{
		if (!mInScene)
			return;
		PxRigidBody* body = GetActor();
		if (body && body->getScene())
			body->getScene()->removeActor(*body, wakeOnLostTouch);
		mInScene = false;
	}

	void PxwVehicle::SetCommands(float brake0, float brake1, float throttle, float steer)
	{
		PxVehicleCommandState& cmd = ActorVehicle()->mCommandState;
		cmd.brakes[0] = brake0;
		cmd.brakes[1] = brake1;
		cmd.nbBrakes = 2;
		cmd.throttle = throttle;
		cmd.steer = steer;
	}

	void PxwVehicle::SetTransmissionCommand(int targetGear, float clutch)
	{
		if (mEngine)
		{
			mEngine->mTransmissionCommandState.targetGear = static_cast<PxU32>(targetGear);
			mEngine->mTransmissionCommandState.clutch = clutch;
			mEngine->mTankDriveTransmissionCommandState.targetGear = static_cast<PxU32>(targetGear);
			mEngine->mTankDriveTransmissionCommandState.clutch = clutch;
		}
		else if (mDirect)
		{
			mDirect->mTransmissionCommandState.gear =
				static_cast<PxVehicleDirectDriveTransmissionCommandState::Enum>(targetGear);
		}
	}

	void PxwVehicle::SetTankThrusts(float thrust0, float thrust1)
	{
		if (mEngine)
		{
			mEngine->mTankDriveTransmissionCommandState.thrusts[0] = thrust0;
			mEngine->mTankDriveTransmissionCommandState.thrusts[1] = thrust1;
		}
	}

	void PxwVehicle::SetWheelControl(int wheelId, float driveTorque, float brakeTorque, float steerAngle)
	{
		if (!mDirect)
			return;
		if (wheelId < 0 || wheelId >= PxVehicleLimits::eMAX_NB_WHEELS)
			return;
		mDirect->mDirectDriveState.directDriveThrottleResponseStates[wheelId] = driveTorque;
		mDirect->mBaseState.brakeCommandResponseStates[wheelId] = brakeTorque;
		mDirect->mBaseState.steerCommandResponseStates[wheelId] = steerAngle;
	}

	void PxwVehicle::Step(float dt, const PxVehiclePhysXSimulationContext& context)
	{
		if (!mFinalized)
			return;
		ActorVehicle()->step(dt, context);
	}

	void PxwVehicle::GetRigidBodyPose(PxwTransformData* dest)
	{
		PxRigidBody* body = GetActor();
		if (body && dest)
			*dest = PxwTransformData(body->getGlobalPose());
	}

	void PxwVehicle::GetWheelStates(PxwVehicleWheelState* dest, int length)
	{
		if (!dest)
			return;
		BaseVehicleState& state = ActorVehicle()->mBaseState;
		for (int i = 0; i < length && i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
		{
			dest[i].localPose = PxwTransformData(state.wheelLocalPoses[i].localPose);
			dest[i].rotationSpeed = state.wheelRigidBody1dStates[i].rotationSpeed;
			dest[i].rotationAngle = state.wheelRigidBody1dStates[i].rotationAngle;
			dest[i].jounce = state.suspensionStates[i].jounce;
			dest[i].steerAngle = state.steerCommandResponseStates[i];
		}
	}

	void PxwVehicle::GetDriveState(PxwVehicleDriveState* dest)
	{
		if (!dest)
			return;

		const BaseVehicleState& base = ActorVehicle()->mBaseState;
		const PxVec3 lngAxis = Base().frame.getLngAxis();
		const PxVec3 latAxis = Base().frame.getLatAxis();
		dest->longitudinalSpeed = base.rigidBodyState.linearVelocity.dot(lngAxis);
		dest->lateralSpeed = base.rigidBodyState.linearVelocity.dot(latAxis);

		if (mEngine)
		{
			const EngineDrivetrainState& e = mEngine->mEngineDriveState;
			dest->engineRotationSpeed = e.engineState.rotationSpeed;
			dest->currentGear = static_cast<int>(e.gearboxState.currentGear);
			dest->targetGear = static_cast<int>(e.gearboxState.targetGear);
			dest->clutchSlip = e.clutchState.clutchSlip;
		}
		else
		{
			dest->engineRotationSpeed = 0.0f;
			dest->currentGear = mDirect ? static_cast<int>(mDirect->mTransmissionCommandState.gear) : 0;
			dest->targetGear = dest->currentGear;
			dest->clutchSlip = 0.0f;
		}
	}

	PxRigidBody* PxwVehicle::GetActor()
	{
		if (!ActorVehicle())
			return NULL;
		return ActorVehicle()->mPhysXState.physxActor.rigidBody;
	}

	PxU32 PxwVehicle::GetWheelCount() const
	{
		const PhysXActorVehicle* v = ActorVehicleConst();
		return v ? v->mBaseParams.axleDescription.nbWheels : 0u;
	}

	PxU32 PxwVehicle::SnapshotSize() const
	{
		return PxwVehicleSnapshotSize(mDriveMode, GetWheelCount());
	}

	bool PxwVehicle::CaptureSnapshot(void* dst, PxU32 capacity) const
	{
		const PhysXActorVehicle* v = ActorVehicleConst();
		if (v == NULL || dst == NULL || capacity < SnapshotSize())
			return false;

		// Wheels are written in axle order so two peers with the same axle
		// description lay the snapshot out identically, regardless of how the
		// underlying per-wheel arrays happen to be indexed.
		const PxVehicleAxleDescription& axle = v->mBaseParams.axleDescription;
		PxU8* cursor = static_cast<PxU8*>(dst);
		for (PxU32 i = 0; i < axle.nbWheels; ++i)
		{
			const PxU32 wheelId = axle.wheelIdsInAxleOrder[i];
			VehicleWheelSnapshot ws;
			std::memset(&ws, 0, sizeof(ws));
			ws.wheel = v->mBaseState.wheelRigidBody1dStates[wheelId];
			ws.suspension = v->mBaseState.suspensionStates[wheelId];
			ws.sticky = v->mBaseState.tireStickyStates[wheelId];
			std::memcpy(cursor, &ws, sizeof(ws));
			cursor += sizeof(ws);
		}

		if (mDriveMode == PxwVehicleDriveMode::eENGINE && mEngine != NULL)
		{
			VehicleEngineSnapshot es;
			std::memset(&es, 0, sizeof(es));
			es.engine = mEngine->mEngineDriveState.engineState;
			es.gearbox = mEngine->mEngineDriveState.gearboxState;
			es.autobox = mEngine->mEngineDriveState.autoboxState;
			es.clutch = mEngine->mEngineDriveState.clutchState;
			std::memcpy(cursor, &es, sizeof(es));
			cursor += sizeof(es);
		}

		return true;
	}

	bool PxwVehicle::RestoreSnapshot(const void* src, PxU32 size)
	{
		PhysXActorVehicle* v = ActorVehicle();
		if (v == NULL || src == NULL || size < SnapshotSize())
			return false;

		const PxVehicleAxleDescription& axle = v->mBaseParams.axleDescription;
		const PxU8* cursor = static_cast<const PxU8*>(src);
		for (PxU32 i = 0; i < axle.nbWheels; ++i)
		{
			const PxU32 wheelId = axle.wheelIdsInAxleOrder[i];
			VehicleWheelSnapshot ws;
			std::memcpy(&ws, cursor, sizeof(ws));
			cursor += sizeof(ws);
			v->mBaseState.wheelRigidBody1dStates[wheelId] = ws.wheel;
			v->mBaseState.suspensionStates[wheelId] = ws.suspension;
			v->mBaseState.tireStickyStates[wheelId] = ws.sticky;
		}

		if (mDriveMode == PxwVehicleDriveMode::eENGINE && mEngine != NULL)
		{
			VehicleEngineSnapshot es;
			std::memcpy(&es, cursor, sizeof(es));
			cursor += sizeof(es);
			mEngine->mEngineDriveState.engineState = es.engine;
			mEngine->mEngineDriveState.gearboxState = es.gearbox;
			mEngine->mEngineDriveState.autoboxState = es.autobox;
			mEngine->mEngineDriveState.clutchState = es.clutch;
		}

		return true;
	}
}
