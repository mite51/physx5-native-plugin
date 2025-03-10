#include "PxwAPIs.h"


static PhysXWrapper gPhysXWrapper;
static PxMaterial* gMaterial = NULL;
// Basics

bool InitializePhysX() {
	gPhysXWrapper.InitPhysX();
	gMaterial = gPhysXWrapper.GetPhysics()->createMaterial(0.5f, 0.5f, 0.f);
	return true;
}

bool GetPhysXInitStatus() {
	return gPhysXWrapper.GetPhysXInitStatus();
}

PxScene* CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu) {
	return gPhysXWrapper.CreateScene(gravity, pruningStructureType, solverType, useGpu);
}

void StepPhysics(PxReal dt) {
	gPhysXWrapper.StepPhysics(dt);
}

void StepPhysicsStart(PxReal dt)
{
	gPhysXWrapper.StepPhysicsStart(dt);
}

void StepPhysicsFetchResults()
{
	gPhysXWrapper.StepPhysicsFetchResults();
}

void ReleaseScene(PxScene* scene)
{
	gPhysXWrapper.ReleaseScene(scene);
}

void ReleasePhysX() {
	gPhysXWrapper.CleanupPhysX();
}

void ReleaseActor(PxActor* actor)
{
	actor->release();
}

void AddActorToScene(PxScene* scene, PxActor* actor)
{
	return gPhysXWrapper.AddActorToScene(scene, actor);
}

void RemoveActorFromScene(PxScene* scene, PxActor* actor)
{
	return gPhysXWrapper.RemoveActorFromScene(scene, actor);
}

PxBoxGeometry* CreateBoxGeometry(PxReal halfWidth, PxReal halfHeight, PxReal halfDepth)
{
	return new PxBoxGeometry(halfWidth, halfHeight, halfDepth);
}

PxCapsuleGeometry* CreateCapsuleGeometry(PxReal radius, PxReal halfHeight)
{
	return new PxCapsuleGeometry(radius, halfHeight);
}

PxShape* CreateShape(PxGeometry* geometry, PxMaterial* material, bool isExclusive)
{
	return gPhysXWrapper.CreateShape(geometry, material, isExclusive);
}

void ReleaseShape(PxShape* shape)
{
	shape->release();
}

void AddSoftActorToScene(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->AddToScene();
}

void RemoveSoftActorFromScene(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->RemoveFromScene();
}

void AddPBDParticleSystemToScene(PxwPBDParticleSystemHelper* particleSystemHelper)
{
	particleSystemHelper->AddToScene();
}

void RemovePBDParticleSystemFromScene(PxwPBDParticleSystemHelper* particleSystemHelper)
{
	particleSystemHelper->RemoveFromScene();
}

void AddPBDObjectToParticleSystem(PxwParticleSystemObject* particleSystemObject)
{
	particleSystemObject->AddToParticleSystem();
}

void RemovePBDObjectFromParticleSystem(PxwParticleSystemObject* particleSystemObject)
{
	particleSystemObject->RemoveFromParticleSystem();
}

void AddArticulationToScene(PxwArticulationKinematicTree* articulation)
{
	articulation->AddToScene();
}

void RemoveArticulationFromScene(PxwArticulationKinematicTree* articulation)
{
	articulation->RemoveFromScene();
}

// Particle system

PxwPBDParticleSystemHelper* CreatePBDParticleSystem(PxScene* scene, PxReal particleSpacing, int maxNumParticlesForAnisotropy)
{
	return gPhysXWrapper.CreatePBDParticleSystem(scene, particleSpacing, maxNumParticlesForAnisotropy);
}

void ReleasePBDParticleSystem(PxwPBDParticleSystemHelper* particleSystemHelper)
{
	particleSystemHelper->Release();
	delete particleSystemHelper;
}

void PBDParticleSystemGetBounds(PxwPBDParticleSystemHelper* particleSystemHelper, PxBounds3& bounds)
{
	bounds = particleSystemHelper->GetBounds();
}

PxwPBDBoxFluid* CreateCubeFluid(
	PxScene* scene,
	PxwPBDParticleSystemHelper* particleSystem,
	PxPBDMaterial* material,
	const PxU32 numX,
	const PxU32 numY,
	const PxU32 numZ,
	const PxVec3* position,
	const PxReal particleSpacing,
	const PxReal fluidDensity,
	const PxU32 maxDiffuseParticles,
	const PxReal buoyancy)
{
	return gPhysXWrapper.CreateCubeFluid(
		scene,
		particleSystem,
		material,
		numX,
		numY,
		numZ,
		*position,
		particleSpacing,
		fluidDensity,
		maxDiffuseParticles,
		buoyancy);
}

PxwPBDFluid* CreateFluid(PxScene* scene, PxwPBDParticleSystemHelper* particleSystem, PxPBDMaterial* material, PxVec4* positions, PxU32 numParticles, PxReal particleSpacing, PxReal fluidDensity, PxU32 maxDiffuseParticles, PxReal buoyancy)
{
	return gPhysXWrapper.CreateFluid(scene, particleSystem, material, positions, numParticles, particleSpacing, fluidDensity, maxDiffuseParticles, buoyancy);
}

PxwPBDTriMeshCloth* CreateTriMeshCloth(PxScene* scene, PxwPBDParticleSystemHelper* particleSystem, PxPBDMaterial* material, PxVec3* vertices,
	const int numVertices, int* indices, const int numIndices, const PxVec3* position, const PxReal totalMass, const bool inflatable, 
	const PxReal blendScale, const PxReal pressure, const PxReal particleSpacing)
{
	return gPhysXWrapper.CreateTriMeshCloth(
		scene,
		particleSystem,
		material,
		vertices,
		numVertices,
		indices,
		numIndices,
		*position,
		totalMass,
		inflatable,
		blendScale,
		pressure,
		particleSpacing);
}

void ResetParticleSystemObject(PxwParticleSystemObject* object)
{
	object->ResetObject();
}

PxwParticleData GetParticleData(PxwParticleSystemObject* object) {
	object->SyncParticleDataDeviceToHost(false);
	return object->GetParticleData();
}

PxwAnisotropyBuffer GetAnisotropy(PxwPBDBoxFluid* object)
{
	return object->GetAnistropyData();
}

PxwAnisotropyBuffer GetAnisotropyAll(PxwPBDParticleSystemHelper* object)
{
	return object->GetAnistropyData();
}

PxwParticleSpringsData GetParticleSpringData(PxwPBDCloth* object)
{
	return object->GetSpringData();
}

void UpdateParticleSprings(PxwPBDCloth* object, PxParticleSpring* springs, int numSprings)
{
	object->UpdateSprings(springs, numSprings);
}

void AttachParticleToRigidBody(PxwPBDCloth* pbdObject, int particleIdx, PxRigidActor* rigidActor, PxVec3* pos)
{
	pbdObject->AddRigidAttachment(particleIdx, rigidActor, pos);
}

void DetachParticleFromRigidBody(PxwPBDCloth* pbdObject, int particleIdx, PxRigidActor* rigidActor, PxVec3* /*pos*/)
{
	pbdObject->RemoveRigidAttachment(particleIdx, rigidActor);
}

void SyncParticleDataHostToDevice(PxwParticleSystemObject* object, bool copyPhase)
{
	object->SyncParticleDataHostToDevice(copyPhase);
}

void AddParticleRigidFilter(PxwParticleSystemObject* particleObject, PxRigidActor* rigidActor, int particleIndex)
{
	particleObject->AddRigidFilter(rigidActor, particleIndex);
}

void RemoveParticleRigidFilter(PxwParticleSystemObject* particleObject, PxRigidActor* rigidActor, int particleIndex)
{
	particleObject->RemoveRigidFilter(rigidActor, particleIndex);
}

void ReleaseParticleSystemObject(PxwParticleSystemObject* object)
{
	object->Release();
	delete object;
}

// Rigid and soft bodies

PxActor* CreateDynamicRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape)
{
	return gPhysXWrapper.CreateDynamicRigidActor(scene, *pose, shape);
}

PxActor* CreateKinematicRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape)
{
	return gPhysXWrapper.CreateKinematicRigidActor(scene, *pose, shape);
}

PxActor* CreateStaticRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape)
{
	return gPhysXWrapper.CreateStaticRigidActor(scene, *pose, shape);
}


PxwSoftBodyHelper* CreateFEMSoftBody(PxScene* scene, const PxU32 numVertices, const PxVec3* triVerts, const PxU32 numTriangles, const int* triIndices, PxwTransformData* pose, PxFEMSoftBodyMaterial* material, PxReal density, PxU32 iterationCount, bool useCollisionMeshForSimulation, PxU32 numVoxelsAlongLongestAABBAxis)
{
	return gPhysXWrapper.CreateFEMSoftBody(scene, numVertices, triVerts, numTriangles, triIndices, *pose, material, density, iterationCount, useCollisionMeshForSimulation, numVoxelsAlongLongestAABBAxis);
}

void ResetFEMSoftBody(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->ResetObject();
}

void SyncFEMSoftBodyCollisionMeshDtoH(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->SyncCollisionVerticesDtoH();
}

PxwFEMSoftBodyMeshData GetFEMSoftBodyCollisionMeshData(PxwSoftBodyHelper* softBodyHelper)
{
	return softBodyHelper->GetCollisionMesh();
}

int AttachFEMSoftBodyVertexToRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* actor, PxU32 vertId, const PxVec3* actorSpacePose)
{
	return softBodyHelper->AttachVertexToRigidBody(actor, vertId, actorSpacePose);
}

void DetachFEMSoftBodyVertexFromRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* actor, PxU32 attachmentHandle)
{
	softBodyHelper->DetachVertexFromRigidBody(actor, attachmentHandle);
}

int AttachFEMSoftBodyOverlappingAreaToRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* rigidActor, PxGeometry* rigidGeometry)
{
	return softBodyHelper->AttachOverlappingAreaToRigidBody(rigidActor, rigidGeometry);
}

void DetachFEMSoftBodyRigidBodyAttachments(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->RemoveRigidAttachments();
}

int AttachFEMSoftBodyOverlappingAreaToSoftBody(PxwSoftBodyHelper* softBodyHelper0, PxwSoftBodyHelper* softBodyHelper1)
{
	return softBodyHelper0->AttachOverlappingAreaToSoftBody(softBodyHelper1);
}

void ReleaseFEMSoftBody(PxwSoftBodyHelper* softBodyHelper)
{
	softBodyHelper->Release();
	delete softBodyHelper;
}

void SetKinematicTarget(PxActor* actor, PxwTransformData* pose)
{
	PxRigidDynamic* a = static_cast<PxRigidDynamic*>(actor);
	a->setKinematicTarget(PxTransform(pose->ToPxTransform()));
}

void ReleaseMesh(PxTriangleMesh* mesh)
{
	mesh->release();
}

// Robotics

PxwArticulationKinematicTree* CreateArticulationKinematicTree(PxScene* scene, bool fixBase, bool disableSelfCollision)
{
	return gPhysXWrapper.CreatePxArticulationKinematicTree(scene, fixBase, disableSelfCollision);
}

PxArticulationLink* CreateArticulationKinematicTreeBase(PxwArticulationKinematicTree* kinematicTree, PxwTransformData* basePose, PxShape* shape, float density)
{
	return kinematicTree->CreateBase(*basePose, shape, density);
}

PxArticulationLink* AddLinkToArticulationKinematicTree(PxwArticulationKinematicTree* kinematicTree, PxArticulationLink* parentLink, PxwTransformData* linkPose, PxwRobotJointType::Enum type, PxwTransformData* jointPoseParent, PxwTransformData* jointPoseChild, PxArticulationAxis::Enum dofAxis, PxShape* shape, float jointLimLower, float jointLimUpper, bool isDriveJoint, float stiffness, float damping, float driveMaxForce, float density)
{
	return kinematicTree->AddLink(parentLink, *linkPose, type, *jointPoseParent, *jointPoseChild, dofAxis, shape, jointLimLower, jointLimUpper, isDriveJoint, stiffness, damping, driveMaxForce, density);
}

void ResetArticulationKinematicTree(PxwArticulationKinematicTree* kinematicTree)
{
	kinematicTree->ResetObject();
}

void DriveArticulationKinematicTreeJoints(PxwArticulationKinematicTree* kinematicTree, float* targetJointPositions)
{
	kinematicTree->DriveJoints(targetJointPositions);
}

void ReleaseArticulationKinematicTree(PxwArticulationKinematicTree* kinematicTree)
{
	kinematicTree->Release();
	delete kinematicTree;
}

void GetArticulationKinematicTreeJointPositions(PxwArticulationKinematicTree* kinematicTree, void* destArray, int length)
{
	memcpy(destArray, kinematicTree->GetJointPositions(), length * sizeof(float));;
}

void GetArticulationKinematicTreeLinkPoses(PxwArticulationKinematicTree* kinematicTree, void* destArray, int length)
{
	PxwTransformData* poseData = kinematicTree->GetLinkPoses();
	memcpy(destArray, poseData, length * sizeof(PxwTransformData));
}

PxwArticulationRobot* CreateArticulationRobot(PxScene* scene, PxwTransformData* pose, float density)
{
	return gPhysXWrapper.CreateArticulationRobot(scene, *pose, density);
}

PxArticulationLink* AddLinkToRobot(
	PxwArticulationRobot* robot,
	PxwTransformData* linkPose,
	PxwRobotJointType::Enum type,
	PxwTransformData* jointPoseParent,
	PxwTransformData* jointPoseChild,
	PxShape* shape,
	float jointLimLower,
	float jointLimUpper,
	float stiffness,
	float damping,
	float driveMaxForce,
	float density
)
{
	return robot->AddBodyLink(*linkPose, type, *jointPoseParent, *jointPoseChild, shape, jointLimLower, jointLimUpper, stiffness, damping, driveMaxForce, density);
}

PxArticulationLink* AddEndEffectorLinkToRobot(
	PxwArticulationRobot* robot,
	PxwTransformData* linkPose,
	PxwRobotJointType::Enum type,
	PxwTransformData* jointPoseParent,
	PxwTransformData* jointPoseChild,
	PxShape* shape,
	float jointLimLower,
	float jointLimUpper,
	float stiffness,
	float damping,
	float driveMaxForce,
	float density
) {
	return robot->AddEndEffectorLink(*linkPose, type, *jointPoseParent, *jointPoseChild, shape, jointLimLower, jointLimUpper, stiffness, damping, driveMaxForce, density);
}

void ResetArticulationRobot(PxwArticulationRobot* robot)
{
	robot->ResetObject();
}

void DriveJoints(PxwArticulationRobot* robot, float* targetJointPositions)
{
	robot->DriveJoints(targetJointPositions);
}

void ReleaseArticulationRobot(PxwArticulationRobot* robot)
{
	robot->Release();
	delete robot;
}

void GetRobotJointPositions(PxwArticulationRobot* robot, void* destArray, int length)
{
	memcpy(destArray, robot->GetJointPositions(), length * sizeof(float));
}

void GetRobotLinkIncomingForce(PxwArticulationRobot* robot, int n, PxwSpatialForceData* destSpatialForceData)
{
	*destSpatialForceData = robot->GetLinkIncomingJointForce(n);
}

void GetRobotLinkPoses(PxwArticulationRobot* robot, void* destArray, int length)
{
	PxwTransformData* poseData = robot->GetLinkPoses();
	memcpy(destArray, poseData, length * sizeof(PxwTransformData));
}

void GetRobotForwardKinematics(PxwArticulationRobot* robot, float* q, Matrix4f* destArray)
{
	Matrix4f p = robot->ForwardKinematics(q);
	memcpy(destArray, &p, sizeof(Matrix4f));
}

void GetRobotJacobianBody(PxwArticulationRobot* robot, float* q, float* destArray, int rows, int cols)
{
	Eigen::MatrixXf Jn = robot->ComputeJacobianBody(q);
	memcpy(destArray, Jn.data(), rows * cols * sizeof(float));
}

void GetRobotJacobianSpatial(PxwArticulationRobot* robot, float* q, float* destArray, int rows, int cols)
{
	Eigen::MatrixXf Js = robot->ComputeJacobianSpatial(q);
	memcpy(destArray, Js.data(), rows * cols * sizeof(float));
}

bool GetRobotInverseKinematics(PxwArticulationRobot* robot, float* qInit, PxwTransformData* targetTransformEEJoint, float tolerance, int numIterations, float lambda)
{
	return robot->InverseKinematics(qInit, *targetTransformEEJoint, tolerance, numIterations, lambda);
}

// Utility functions

PxTriangleMesh* CreateBV33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices, bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff, bool buildGpuData, float sdfSpacing, PxU32 sdfSubgridSize, PxSdfBitsPerSubgridPixel::Enum bitsPerSdfSubgridPixel)
{
	return gPhysXWrapper.CreateBV33TriangleMesh(numVertices, vertices, numTriangles, indices, skipMeshCleanup, skipEdgeData, inserted, cookingPerformance, meshSizePerfTradeoff, buildGpuData, sdfSpacing, sdfSubgridSize, bitsPerSdfSubgridPixel);
}

PxConvexMesh* CreateConvexMesh(PxU32 numVertices, const PxVec3* vertices, bool inserted, PxU32 gaussMapLimit)
{
	return gPhysXWrapper.CreateConvexMesh(numVertices, vertices, inserted, gaussMapLimit);
}

void FastCopy(void* src, void* dst, int size) {
	memcpy(dst, src, size);
}

int CreateWeldedMeshIndices(const PxVec3* vertices, int numVertices, int* uniqueVerts, int* originalToUniqueMap, float threshold)
{
	return gPhysXWrapper.CreateWeldedMeshIndices(vertices, numVertices, uniqueVerts, originalToUniqueMap, threshold);
}

PxGeometry* CreatePxGeometry(const PxGeometryType::Enum type, const int numShapeParams, const float* shapeParams, void* shapeRef)
{
	return gPhysXWrapper.CreatePxGeometry(type, numShapeParams, shapeParams, shapeRef);
}

void DeletePxGeometry(PxGeometry* geometry)
{
	delete geometry;
}

PxMaterial* CreatePxMaterial(const float staticFriction, const float dynamicFriction, const float restitution)
{
	return gPhysXWrapper.CreateMaterial(staticFriction, dynamicFriction, restitution);
}

PxFEMSoftBodyMaterial* CreatePxFEMSoftBodyMaterial(const float youngs, const float poissons, const float dynamicFriction, const float damping, const PxFEMSoftBodyMaterialModel::Enum model)
{
	return gPhysXWrapper.CreateFEMSoftBodyMaterial(youngs, poissons, dynamicFriction, damping, model);
}

PxPBDMaterial* CreatePxPBDMaterial(const float friction, const float damping, const float adhesion, const float viscosity, const float vorticityConfinement, const float surfaceTension, const float cohesion, const float lift, const float drag, const float cflCoefficient, const float gravityScale)
{
	return gPhysXWrapper.CreatePBDMaterial(friction, damping, adhesion, viscosity, vorticityConfinement, surfaceTension, cohesion, lift, drag, cflCoefficient, gravityScale);
}

void ReleasePxMaterial(PxBaseMaterial* material)
{
	material->release();
}

// Material physical property setters

void PxPBDMaterialSetFriction(PxPBDMaterial* material, float value)
{
	material->setFriction(value);
}

void PxPBDMaterialSetDamping(PxPBDMaterial* material, float value)
{
	material->setDamping(value);
}

void PxPBDMaterialSetAdhesion(PxPBDMaterial* material, float value)
{
	material->setAdhesion(value);
}

void PxPBDMaterialSetViscosity(PxPBDMaterial* material, float value)
{
	material->setViscosity(value);
}

void PxPBDMaterialSetVorticityConfinement(PxPBDMaterial* material, float value)
{
	material->setVorticityConfinement(value);
}

void PxPBDMaterialSetSurfaceTension(PxPBDMaterial* material, float value)
{
	material->setSurfaceTension(value);
}

void PxPBDMaterialSetCohesion(PxPBDMaterial* material, float value)
{
	material->setCohesion(value);
}

void PxPBDMaterialSetLift(PxPBDMaterial* material, float value)
{
	material->setLift(value);
}

void PxPBDMaterialSetDrag(PxPBDMaterial* material, float value)
{
	material->setDrag(value);
}

void PxPBDMaterialSetCflCoefficient(PxPBDMaterial* material, float value)
{
	material->setCFLCoefficient(value);
}

void PxPBDMaterialSetGravityScale(PxPBDMaterial* material, float value)
{
	material->setGravityScale(value);
}

float PointDistance(const PxVec3* point, const PxGeometry* geom, const PxwTransformData* pose, PxVec3* closestPoint, PxU32* closestIndex)
{
	return PxGeometryQuery::pointDistance(*point, *geom, pose->ToPxTransform(), closestPoint, closestIndex);
}

void ComputeGeomBounds(PxBounds3& bounds, const PxGeometry* geom, const PxwTransformData* pose, float offset, float inflation)
{
	return PxGeometryQuery::computeGeomBounds(bounds, *geom, pose->ToPxTransform(), offset, inflation);
}

// PxRigidActor/Dynamic

void GetRigidActorPose(PxRigidActor* actor, PxwTransformData* destPose)
{
	*destPose = PxwTransformData(actor->getGlobalPose());
}

void SetMass(PxRigidDynamic* rigidDynamic, PxReal mass)
{
	PxRigidBodyExt::setMassAndUpdateInertia(*rigidDynamic, mass);
}

PxReal GetMass(PxRigidDynamic* rigidDynamic)
{
	return rigidDynamic->getMass();
}

void SetLinearVelocity(PxRigidDynamic* rigidDynamic, PxVec3* velocity)
{
	rigidDynamic->setLinearVelocity(*velocity);
}

PxVec3 GetLinearVelocity(PxRigidDynamic* rigidDynamic)
{
	return rigidDynamic->getLinearVelocity();
}

void SetAngularVelocity(PxRigidDynamic* rigidDynamic, PxVec3* velocity)
{
	rigidDynamic->setAngularVelocity(*velocity);
}

PxVec3 GetAngularVelocity(PxRigidDynamic* rigidDynamic)
{
	return rigidDynamic->getAngularVelocity();
}

// D6 joints
PxD6Joint* CreateD6Joint(PxRigidActor* actor0, PxRigidActor* actor1)
{
	return PxD6JointCreate(*gPhysXWrapper.GetPhysics(), actor0, PxTransform(PxIdentity), actor1, PxTransform(PxIdentity));
}

void ReleaseD6Joint(PxD6Joint* joint)
{
	joint->release();
}

void SetD6JointDriveMotion(PxD6Joint* joint, PxD6Axis::Enum axis, PxD6Motion::Enum motionType)
{
	joint->setMotion(axis, motionType);
}

void SetD6JointDrive(PxD6Joint* joint, PxD6Drive::Enum index, PxReal driveStiffness, PxReal driveDamping, PxReal driveForceLimit)
{
	joint->setDrive(index, PxD6JointDrive(driveStiffness, driveDamping, driveForceLimit));
}

void SetD6DriveVelocity(PxD6Joint* joint, PxVec3* linearVelocity, PxVec3* angularVelocity)
{
	joint->setDriveVelocity(*linearVelocity, *angularVelocity);
}

void GetD6DriveVelocity(PxD6Joint* joint, PxVec3* linearVelocity, PxVec3* angularVelocity)
{
	joint->getDriveVelocity(*linearVelocity, *angularVelocity);
}

PxArticulationReducedCoordinate* CreateArticulationRoot(PxArticulationFlag::Enum flag, int solverIterationCount)
{
	PxArticulationReducedCoordinate* articulation = gPhysXWrapper.GetPhysics()->createArticulationReducedCoordinate();
	if (articulation != nullptr)
	{
		articulation->setArticulationFlag(flag, true);
		articulation->setSolverIterationCounts(solverIterationCount, 1);
	}
	return articulation;
}

void AddArticulationRootToScene(PxScene* scene, PxArticulationReducedCoordinate* articulation)
 {
	scene->addArticulation(*articulation);
 }

void RemoveArticulationRootFromScene(PxScene* scene, PxArticulationReducedCoordinate* articulation)
{
	scene->removeArticulation(*articulation);
}

// Articulation
 PxArticulationLink* CreateArticulationLink(PxArticulationReducedCoordinate* articulation, PxArticulationLink* parentLink, PxwTransformData* pose)
 {
	 PxArticulationLink* link = articulation->createLink(parentLink, pose->ToPxTransform());
	 return link;
 }

 void SetArticulationLinkShape(PxArticulationLink* link, PxShape* shape)
 {
	//TODO replace gMaterial
	PxRigidActorExt::createExclusiveShape(*link, shape->getGeometry(), *gMaterial);
 }

 void UpdateArticulationLinkMassAndInertia(PxArticulationLink* link, PxReal density)
 {
	PxRigidBodyExt::updateMassAndInertia(*link, density);
 }

 PxArticulationJointReducedCoordinate* GetArticulationJoint(PxArticulationLink* link)
 {
	return static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
 }

 PxArticulationReducedCoordinate* GetLinkArticulation(PxArticulationLink* link)
 {
	return &link->getArticulation();
 }

 void SetArticulationJointType(PxArticulationJointReducedCoordinate* joint, PxArticulationJointType::Enum type)
 {
	joint->setJointType(type);
 }

 void SetArticulationJointParentPose(PxArticulationJointReducedCoordinate* joint, PxwTransformData* pose)
 {
	joint->setParentPose(pose->ToPxTransform());
 }

 void SetArticulationJointChildPose(PxArticulationJointReducedCoordinate* joint, PxwTransformData* pose)
 {
	joint->setChildPose(pose->ToPxTransform());
 }

 void SetArticulationJointMotion(PxArticulationJointReducedCoordinate* joint, PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
 {
	joint->setMotion(axis, motion);
 }

 void SetArticulationJointLimitParams(PxArticulationJointReducedCoordinate* joint, PxArticulationAxis::Enum axis, PxReal lower, PxReal upper)
 {
	joint->setLimitParams(axis, PxArticulationLimit(lower, upper));
 }

 void SetArticulationJointDriveParams(PxArticulationJointReducedCoordinate* joint, PxArticulationAxis::Enum axis, PxReal stiffness, PxReal damping, PxReal maxForce)
 {
	joint->setDriveParams(axis, PxArticulationDrive(stiffness, damping, maxForce));
 }

 void SetArticulationJointDriveTarget(PxArticulationJointReducedCoordinate* joint, PxArticulationAxis::Enum axis, PxReal target)
 {
	joint->setDriveTarget(axis, target);
 }

 void SetArticulationJointDriveVelocity(PxArticulationJointReducedCoordinate* joint, PxArticulationAxis::Enum axis, PxReal velocity)
 {
	joint->setDriveVelocity(axis, velocity);
 }

 void ReleaseArticulation(PxArticulationReducedCoordinate* articulation)
 {
	articulation->release();
 }

 PxU32 GetArticulationLinkCount(PxArticulationReducedCoordinate* articulation)
 {
	return articulation->getNbLinks();
 }

 void GetArticulationLinks(PxArticulationReducedCoordinate* articulation, PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex)
 {
	articulation->getLinks(userBuffer, bufferSize, startIndex);
 }

 void SetArticulationRootGlobalPose(PxArticulationReducedCoordinate* articulation, PxwTransformData* pose, bool autowake)
 {
	articulation->setRootGlobalPose(pose->ToPxTransform(), autowake);
 }

 void SetArticulationSolverIterationCounts(PxArticulationReducedCoordinate* articulation, PxU32 positionIters, PxU32 velocityIters)
 {
	articulation->setSolverIterationCounts(positionIters, velocityIters);
 }

 void SetArticulationSleepThreshold(PxArticulationReducedCoordinate* articulation, PxReal threshold)
 {
	articulation->setSleepThreshold(threshold);
 }

 void SetArticulationStabilizationThreshold(PxArticulationReducedCoordinate* articulation, PxReal threshold)
 {
	articulation->setStabilizationThreshold(threshold);
 }

 void SetArticulationWakeCounter(PxArticulationReducedCoordinate* articulation, PxReal wakeCounterValue)
 {
	articulation->setWakeCounter(wakeCounterValue);
 }

 void WakeUpArticulation(PxArticulationReducedCoordinate* articulation)
 {
	articulation->wakeUp();
 }

 void PutArticulationToSleep(PxArticulationReducedCoordinate* articulation)
 {
	articulation->putToSleep();
 }

 void SetArticulationMaxCOMLinearVelocity(PxArticulationReducedCoordinate* articulation, PxReal maxLinearVelocity)
 {
	articulation->setMaxCOMLinearVelocity(maxLinearVelocity);
 }

 void SetArticulationMaxCOMAngularVelocity(PxArticulationReducedCoordinate* articulation, PxReal maxAngularVelocity)
 {
	articulation->setMaxCOMAngularVelocity(maxAngularVelocity);
 }

 PxArticulationCache* CreateArticulationCache(PxArticulationReducedCoordinate* articulation)
 {
	return articulation->createCache();
 }

 void ReleaseArticulationCache(PxArticulationCache* cache)
 {
	cache->release();
 }

 void ApplyArticulationCache(PxArticulationReducedCoordinate* articulation, PxArticulationCache& cache, PxArticulationCacheFlags flags)
 {
	articulation->applyCache(cache, flags, true);
 }

 void CopyInternalStateToArticulationCache(PxArticulationReducedCoordinate* articulation, PxArticulationCache& cache, PxArticulationCacheFlags flags)
 {
	articulation->copyInternalStateToCache(cache, flags);
 }

 void SetArticulationLinkJointDriveParams(PxArticulationLink* link, PxArticulationAxis::Enum axis, PxReal stiffness, PxReal damping, PxReal driveMaxForce)
 {
	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	if (joint)
	{
		joint->setDriveParams(axis, PxArticulationDrive(stiffness, damping, driveMaxForce));
	}
 }

 void SetArticulationLinkJointMotion(PxArticulationLink* link, PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)
 {
	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	if (joint)
	{
		joint->setMotion(axis, motion);
	}
 }

void SetArticulationLinkJointLimits(PxArticulationLink* link, PxArticulationAxis::Enum axis, PxReal lower, PxReal upper)
{
	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
	if (joint)
	{
		joint->setLimitParams(axis, PxArticulationLimit(lower, upper));
	}
}

// Articulation Link properties
void GetArticulationLinkGlobalPose(PxArticulationLink* link, PxwTransformData* destPose)
{
	*destPose = PxwTransformData(link->getGlobalPose());
}

void SetArticulationLinkLinearDamping(PxArticulationLink* link, PxReal linearDamping)
{
	link->setLinearDamping(linearDamping);
}

PxReal GetArticulationLinkLinearDamping(PxArticulationLink* link)
{
	return link->getLinearDamping();
}

void SetArticulationLinkAngularDamping(PxArticulationLink* link, PxReal angularDamping)
{
	link->setAngularDamping(angularDamping);
}

PxReal GetArticulationLinkAngularDamping(PxArticulationLink* link)
{
	return link->getAngularDamping();
}

void SetArticulationLinkMaxLinearVelocity(PxArticulationLink* link, PxReal maxLinearVelocity)
{
	link->setMaxLinearVelocity(maxLinearVelocity);
}

PxReal GetArticulationLinkMaxLinearVelocity(PxArticulationLink* link)
{
	return link->getMaxLinearVelocity();
}

void SetArticulationLinkMaxAngularVelocity(PxArticulationLink* link, PxReal maxAngularVelocity)
{
	link->setMaxAngularVelocity(maxAngularVelocity);
}

PxReal GetArticulationLinkMaxAngularVelocity(PxArticulationLink* link)
{
	return link->getMaxAngularVelocity();
}

PxU32 GetArticulationLinkInboundJointDof(PxArticulationLink* link)
{
	return link->getInboundJointDof();
}

// Articulation Cache operations
void GetArticulationJointPositions(PxArticulationReducedCoordinate* articulation, PxArticulationCache* cache, float* positions, PxU32 bufferSize)
{
	articulation->copyInternalStateToCache(*cache, PxArticulationCacheFlag::ePOSITION);
	PxReal* jointPositions = cache->jointPosition;
	PxU32 dofs = articulation->getDofs();
	PxU32 copySize = PxMin(bufferSize, dofs);
	memcpy(positions, jointPositions, copySize * sizeof(float));
}

void SetArticulationJointPositions(PxArticulationReducedCoordinate* articulation, PxArticulationCache* cache, float* positions, PxU32 bufferSize)
{
	PxU32 dofs = articulation->getDofs();
	PxU32 copySize = PxMin(bufferSize, dofs);
	memcpy(cache->jointPosition, positions, copySize * sizeof(float));
	articulation->applyCache(*cache, PxArticulationCacheFlag::ePOSITION, true);
}

void GetArticulationJointVelocities(PxArticulationReducedCoordinate* articulation, PxArticulationCache* cache, float* velocities, PxU32 bufferSize)
{
	articulation->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
	PxReal* jointVelocities = cache->jointVelocity;
	PxU32 dofs = articulation->getDofs();
	PxU32 copySize = PxMin(bufferSize, dofs);
	memcpy(velocities, jointVelocities, copySize * sizeof(float));
}

void SetArticulationJointVelocities(PxArticulationReducedCoordinate* articulation, PxArticulationCache* cache, float* velocities, PxU32 bufferSize)
{
	PxU32 dofs = articulation->getDofs();
	PxU32 copySize = PxMin(bufferSize, dofs);
	memcpy(cache->jointVelocity, velocities, copySize * sizeof(float));
	articulation->applyCache(*cache, PxArticulationCacheFlag::eVELOCITY, true);
}

PxU32 GetArticulationDofs(PxArticulationReducedCoordinate* articulation)
{
	return articulation->getDofs();
}





























