// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "VehiclePhysXIntegration.h"

#include "cooking/PxCooking.h"
#include "extensions/PxDefaultStreams.h"

namespace pxw
{

void PhysXIntegrationParams::create
(const PxVehicleAxleDescription& axleDescription,
 const PxQueryFilterData& queryFilterData, PxQueryFilterCallback* queryFilterCallback,
 PxVehiclePhysXMaterialFriction* materialFrictions, const PxU32 nbMaterialFrictions, const PxReal defaultFriction,
 const PxTransform& actorCMassLocalPose,
 const PxVec3& actorBoxShapeHalfExtents, const PxTransform& actorBoxShapeLocalPose)
{
	physxRoadGeometryQueryParams.roadGeometryQueryType = PxVehiclePhysXRoadGeometryQueryType::eRAYCAST;
	physxRoadGeometryQueryParams.defaultFilterData = queryFilterData;
	physxRoadGeometryQueryParams.filterCallback = queryFilterCallback;
	physxRoadGeometryQueryParams.filterDataEntries = NULL;

	for(PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32  wheelId = axleDescription.wheelIdsInAxleOrder[i];
		physxMaterialFrictionParams[wheelId].defaultFriction = defaultFriction;
		physxMaterialFrictionParams[wheelId].materialFrictions = materialFrictions;
		physxMaterialFrictionParams[wheelId].nbMaterialFrictions = nbMaterialFrictions;

		physxSuspensionLimitConstraintParams[wheelId].restitution = 0.0f;
		physxSuspensionLimitConstraintParams[wheelId].directionForSuspensionLimitConstraint = PxVehiclePhysXSuspensionLimitConstraintParams::eROAD_GEOMETRY_NORMAL;

		physxWheelShapeLocalPoses[wheelId] = PxTransform(PxIdentity);
	}

	physxActorCMassLocalPose = actorCMassLocalPose;
	physxActorBoxShapeHalfExtents = actorBoxShapeHalfExtents;
	physxActorBoxShapeLocalPose = actorBoxShapeLocalPose;
}

PhysXIntegrationParams PhysXIntegrationParams::transformAndScale
(const PxVehicleFrame& srcFrame, const PxVehicleFrame& trgFrame, const PxVehicleScale& srcScale, const PxVehicleScale& trgScale) const
{
	PhysXIntegrationParams r = *this;
	r.physxRoadGeometryQueryParams = physxRoadGeometryQueryParams.transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; i++)
	{
		r.physxSuspensionLimitConstraintParams[i] = physxSuspensionLimitConstraintParams[i].transformAndScale(srcFrame, trgFrame, srcScale, trgScale);
	}
	r.physxActorCMassLocalPose = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorCMassLocalPose);
	r.physxActorBoxShapeHalfExtents = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorBoxShapeHalfExtents);
	r.physxActorBoxShapeLocalPose = PxVehicleTransformFrameToFrame(srcFrame, trgFrame, srcScale, trgScale, physxActorBoxShapeLocalPose);
	return r;
}

namespace
{

// Cooks the wheel collision hull PxVehiclePhysXActorCreate builds: a 16-sided prism around the
// wheel, from its radius and half width, in the vehicle's frame.
//
// This mirrors createShapes() in the SDK (VhPhysXActorHelpers.cpp) deliberately and exactly,
// including the segment count, the vertex ordering and the cooking flags. Wheel shapes are
// built here rather than by the SDK helper so each wheel can be given its own geometry, and
// PxShape::setGeometry cannot change a shape's geometry type after the fact. Reproducing the
// SDK's hull bit for bit is what keeps every existing vehicle - and its construction hash -
// unchanged now that the plugin owns this step.
PxConvexMesh* CookWheelPrism(const PxVehicleFrame& vehicleFrame, const PxVehicleWheelParams& wheelParams,
	PxPhysics& physics, const PxCookingParams& params)
{
	const PxF32 radius = wheelParams.radius;
	const PxF32 halfWidth = wheelParams.halfWidth;

	PxVec3 verts[32];
	for (PxU32 k = 0; k < 16; k++)
	{
		const PxF32 lng = radius * PxCos(k * 2.0f * PxPi / 16.0f);
		const PxF32 lat = halfWidth;
		const PxF32 vrt = radius * PxSin(k * 2.0f * PxPi / 16.0f);

		const PxVec3 pos0 = vehicleFrame.getFrame() * PxVec3(lng, lat, vrt);
		const PxVec3 pos1 = vehicleFrame.getFrame() * PxVec3(lng, -lat, vrt);
		verts[2 * k + 0] = pos0;
		verts[2 * k + 1] = pos1;
	}

	PxConvexMeshDesc convexDesc;
	convexDesc.points.count = 32;
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.points.data = verts;
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxDefaultMemoryOutputStream buf;
	if (!PxCookConvexMesh(params, convexDesc, buf))
		return NULL;

	PxDefaultMemoryInputData id(buf.getData(), buf.getSize());
	return physics.createConvexMesh(id);
}

// The cylinder equivalent of the cooked prism: same radius and width, but exact.
//
// The wheel's axis is the vehicle frame's lateral axis, while a convex core cylinder always
// runs along its own local +X, so the geometry needs a rotation taking +X onto that axis. That
// rotation belongs in the shape's local pose, except the vehicle overwrites wheel shape local
// poses every step from the suspension and spin state. Instead the rotation is folded into the
// wheel shape local pose the vehicle composes with, which is what physxWheelShapeLocalPoses is
// for.
PxQuat CylinderAxisRotation(const PxVehicleFrame& vehicleFrame)
{
	const PxVec3 wheelAxis = vehicleFrame.getLatAxis();
	const PxVec3 cylinderAxis(1.0f, 0.0f, 0.0f);

	const PxReal dot = cylinderAxis.dot(wheelAxis);
	if (dot > 0.99999f)
		return PxQuat(PxIdentity);
	if (dot < -0.99999f)
	{
		// Antiparallel: any axis perpendicular to the cylinder axis will do for a half turn,
		// and the shape is symmetric about its own axis so the choice is not observable.
		return PxQuat(PxPi, PxVec3(0.0f, 1.0f, 0.0f));
	}

	const PxVec3 rotationAxis = cylinderAxis.cross(wheelAxis).getNormalized();
	return PxQuat(PxAcos(dot), rotationAxis);
}

// Builds the chassis shape and one shape per wheel, replacing PxVehiclePhysXActorCreate's
// createShapes so wheel geometry and shape flags can be chosen per wheel.
void createShapes
(const PxVehicleFrame& vehicleFrame,
 const PxVehiclePhysXRigidActorShapeParams& rigidActorShapeParams,
 const PxVehicleAxleDescription& axleDescription, const PxVehicleWheelParams* wheelParams,
 const WheelShapeConfig* wheelShapeConfigs, PxMaterial& wheelMaterial,
 PxTransform* wheelShapeLocalPoses,
 PxRigidBody* rd, PxPhysics& physics, const PxCookingParams& params,
 PxVehiclePhysXActor& vehiclePhysXActor)
{
	//Create a shape for the vehicle body.
	{
		PxShape* shape = physics.createShape(rigidActorShapeParams.geometry, rigidActorShapeParams.material, true);
		shape->setLocalPose(rigidActorShapeParams.localPose);
		shape->setFlags(rigidActorShapeParams.flags);
		shape->setSimulationFilterData(rigidActorShapeParams.simulationFilterData);
		shape->setQueryFilterData(rigidActorShapeParams.queryFilterData);
		rd->attachShape(*shape);
		shape->release();
	}

	//Create shapes for wheels.
	for (PxU32 i = 0; i < axleDescription.nbWheels; i++)
	{
		const PxU32 wheelId = axleDescription.wheelIdsInAxleOrder[i];
		const WheelShapeConfig& config = wheelShapeConfigs[wheelId];

		PxShapeFlags flags = PxShapeFlags(0);
		if (config.desc.simulationShape)
			flags |= PxShapeFlag::eSIMULATION_SHAPE;
		if (config.desc.sceneQueryShape)
			flags |= PxShapeFlag::eSCENE_QUERY_SHAPE;

		const PxFilterData simFilterData(
			config.desc.simFilterData[0], config.desc.simFilterData[1],
			config.desc.simFilterData[2], config.desc.simFilterData[3]);
		const PxFilterData queryFilterData(
			config.desc.queryFilterData[0], config.desc.queryFilterData[1],
			config.desc.queryFilterData[2], config.desc.queryFilterData[3]);

		PxShape* wheelShape = NULL;
		PxConvexMesh* convexMesh = NULL;

		switch (config.desc.geometryMode)
		{
		case PxwVehicleWheelGeometryMode::eCYLINDER:
		{
			// PxConvexCore::Cylinder takes a full height, and the geometry is the core swept
			// by the margin, so the core is shrunk by the margin to keep the wheel the size
			// the wheel params asked for.
			const PxReal margin = PxMax(0.0f, config.desc.margin);
			const PxReal height = PxMax(0.001f, 2.0f * wheelParams[wheelId].halfWidth - 2.0f * margin);
			const PxReal radius = PxMax(0.001f, wheelParams[wheelId].radius - margin);
			const PxConvexCoreGeometry cylinder(PxConvexCore::Cylinder(height, radius), margin);
			wheelShape = physics.createShape(cylinder, wheelMaterial, true);

			// Turn the cylinder's local +X onto the wheel's rotation axis. Composed into the
			// pose the vehicle multiplies its per-step wheel pose by, because the shape's own
			// local pose is overwritten every step.
			wheelShapeLocalPoses[wheelId] = PxTransform(PxVec3(0.0f), CylinderAxisRotation(vehicleFrame));
			break;
		}
		case PxwVehicleWheelGeometryMode::eGEOMETRY:
		{
			if (config.geometry == NULL)
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
					"Wheel shape geometry mode is eGEOMETRY but no geometry was supplied; cooking the default prism\n");
			}
			else
			{
				wheelShape = physics.createShape(*config.geometry, wheelMaterial, true);
			}
			break;
		}
		default:
			break;
		}

		if (wheelShape == NULL)
		{
			convexMesh = CookWheelPrism(vehicleFrame, wheelParams[wheelId], physics, params);
			PxConvexMeshGeometry convexMeshGeom(convexMesh);
			wheelShape = physics.createShape(convexMeshGeom, wheelMaterial, true);
		}

		wheelShape->setFlags(flags);
		wheelShape->setSimulationFilterData(simFilterData);
		wheelShape->setQueryFilterData(queryFilterData);

		rd->attachShape(*wheelShape);
		wheelShape->release();
		if (convexMesh != NULL)
			convexMesh->release();

		vehiclePhysXActor.wheelShapes[wheelId] = wheelShape;
	}
}

} // anonymous namespace

void PhysXIntegrationState::create
(const BaseVehicleParams& baseParams, PhysXIntegrationParams& physxParams,
 PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial,
 const PxGeometry* chassisGeometry, const WheelShapeConfig* wheelShapeConfigs)
{
	setToDefault();

	//physxActor needs to be populated with an actor and its shapes.
	{
		const PxVehiclePhysXRigidActorParams physxActorParams(baseParams.rigidBodyParams, NULL);
		const PxBoxGeometry boxGeom(physxParams.physxActorBoxShapeHalfExtents);
		// Use the caller-supplied chassis geometry when available, otherwise fall
		// back to a box built from the descriptor half extents.
		const PxGeometry& chassisGeom = chassisGeometry ? *chassisGeometry : static_cast<const PxGeometry&>(boxGeom);
		// The chassis is a simulation shape so vehicles collide with each other and with
		// other dynamic/static rigid bodies. Zero filter data collides with everything under
		// the scene's default filter shader. Suspension queries use PxQueryFlag::eSTATIC only
		// (see setPhysXIntegrationParams), so a dynamic chassis is never hit by any wheel raycast.
		const PxVehiclePhysXRigidActorShapeParams physxActorShapeParams(chassisGeom, physxParams.physxActorBoxShapeLocalPose, defaultMaterial, PxShapeFlags(PxShapeFlag::eSIMULATION_SHAPE), PxFilterData(), PxFilterData());

		// Wheels default to the raycast-driven, non-colliding shape Vehicle2 expects.
		WheelShapeConfig defaultConfigs[PxVehicleLimits::eMAX_NB_WHEELS];
		if (wheelShapeConfigs == NULL)
		{
			for (PxU32 i = 0; i < PxVehicleLimits::eMAX_NB_WHEELS; ++i)
				defaultConfigs[i].setToDefault();
			wheelShapeConfigs = defaultConfigs;
		}

		// The actor is built here rather than by PxVehiclePhysXActorCreate so createShapes can
		// give each wheel its own geometry; PxVehiclePhysXActorConfigure is the same call that
		// helper makes, so the actor itself is configured identically.
		PxRigidDynamic* rd = physics.createRigidDynamic(PxTransform(PxIdentity));
		physxActor.rigidBody = rd;
		PxVehiclePhysXActorConfigure(physxActorParams, physxParams.physxActorCMassLocalPose, *rd);

		createShapes(
			baseParams.frame, physxActorShapeParams,
			baseParams.axleDescription, baseParams.wheelParams,
			wheelShapeConfigs, defaultMaterial,
			physxParams.physxWheelShapeLocalPoses,
			rd, physics, params,
			physxActor);
	}

	//physxConstraints needs to be populated with constraints.
	PxVehicleConstraintsCreate(baseParams.axleDescription, physics, *physxActor.rigidBody, physxConstraints);
}

void PhysXIntegrationState::destroy()
{
	PxVehicleConstraintsDestroy(physxConstraints);
	PxVehiclePhysXActorDestroy(physxActor);
}


void setPhysXIntegrationParams(const PxVehicleAxleDescription& axleDescription,
	PxVehiclePhysXMaterialFriction* physXMaterialFrictions, PxU32 nbPhysXMaterialFrictions,
	PxReal physXDefaultMaterialFriction, PhysXIntegrationParams& physXParams)
{
	//The physx integration params are hardcoded rather than loaded from file.
	const PxQueryFilterData queryFilterData(PxFilterData(0, 0, 0, 0), PxQueryFlag::eSTATIC);
	PxQueryFilterCallback* queryFilterCallback = NULL;
	const PxTransform physxActorCMassLocalPose(PxVec3(0.0f, 0.55f, 1.594f), PxQuat(PxIdentity));
	const PxVec3 physxActorBoxShapeHalfExtents(0.84097f, 0.65458f, 2.46971f);
	const PxTransform physxActorBoxShapeLocalPose(PxVec3(0.0f, 0.830066f, 1.37003f), PxQuat(PxIdentity));

	physXParams.create(
		axleDescription,
		queryFilterData, queryFilterCallback,
		physXMaterialFrictions, nbPhysXMaterialFrictions, physXDefaultMaterialFriction,
		physxActorCMassLocalPose,
		physxActorBoxShapeHalfExtents, physxActorBoxShapeLocalPose);
}


bool PhysXActorVehicle::initialize(PxPhysics& physics, const PxCookingParams& params, PxMaterial& defaultMaterial,
	const PxGeometry* chassisGeometry, const WheelShapeConfig* wheelShapeConfigs)
{
	mCommandState.setToDefault();

	if (!BaseVehicle::initialize())
		return false;
	
	if (!mPhysXParams.isValid(mBaseParams.axleDescription))
		return false;

	mPhysXState.create(mBaseParams, mPhysXParams, physics, params, defaultMaterial, chassisGeometry, wheelShapeConfigs);

	return true;
}

void PhysXActorVehicle::destroy()
{
	mPhysXState.destroy();

	BaseVehicle::destroy();
}

void PhysXActorVehicle::setUpActor(PxScene& scene, const PxTransform& pose, const char* vehicleName)
{
	//Give the vehicle a start pose by appylying a pose to the PxRigidDynamic associated with the vehicle. 
	//This vehicle has components that are configured to read the pose from the PxRigidDynamic 
	//at the start of the vehicle simulation update and to write back an updated pose at the end of the 
	//vehicle simulation update. This allows PhysX to manage any collisions that might happen in-between 
	//each vehicle update. This is not essential but it is anticipated that this will be a typical component 
	//configuration. 
	mPhysXState.physxActor.rigidBody->setGlobalPose(pose);

	//Add the physx actor to the physx scene.
	//As described above, a vehicle may be coupled to a physx scene or it can be simulated without any reference to 
	//to a PxRigidDynamic or PxScene. This snippet vehicle employs a configuration that includes coupling to a PxScene and a 
	//PxRigidDynamic. This being the case, the PxRigidDynamic associated with the vehicle needs to be added to the 
	//PxScene instance.
	scene.addActor(*mPhysXState.physxActor.rigidBody);

	//Give the physx actor a name to help identification in PVD
	mPhysXState.physxActor.rigidBody->setName(vehicleName);
}

}//namespace pxw
