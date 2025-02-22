#pragma once

#include "PxPhysicsAPI.h"
#include "PhysXWrapper.h"

// Macro to export our functions
#ifdef _WIN32
    #ifdef API_EXPORTS
        #define PHYSX_WRAPPER_API __declspec(dllexport)
    #else
        #define PHYSX_WRAPPER_API __declspec(dllimport)
    #endif
#elif defined(__linux__)
    #ifdef API_EXPORTS
        #define PHYSX_WRAPPER_API __attribute__((visibility("default")))
    #else
        #define PHYSX_WRAPPER_API
    #endif
#endif

using namespace physx;
using namespace pxw;

extern "C" {

    // Basics
    
    PHYSX_WRAPPER_API bool InitializePhysX();

    PHYSX_WRAPPER_API bool GetPhysXInitStatus();

    PHYSX_WRAPPER_API PxScene* CreateScene(PxVec3* gravity, PxPruningStructureType::Enum pruningStructureType, PxSolverType::Enum solverType, bool useGpu);

    PHYSX_WRAPPER_API void StepPhysics(PxReal dt);

    PHYSX_WRAPPER_API void StepPhysicsStart(PxReal dt);

    PHYSX_WRAPPER_API void StepPhysicsFetchResults();

    PHYSX_WRAPPER_API void ReleaseScene(PxScene* scene);

    PHYSX_WRAPPER_API void ReleasePhysX();

    PHYSX_WRAPPER_API void ReleaseActor(PxActor* actor);

    // Actor basics

    PHYSX_WRAPPER_API void AddActorToScene(PxScene* scene, PxActor* actor);

    PHYSX_WRAPPER_API void RemoveActorFromScene(PxScene* scene, PxActor* actor);

    PHYSX_WRAPPER_API PxShape* CreateShape(PxGeometry* geometry, PxMaterial* material, bool isExclusive);

    PHYSX_WRAPPER_API void ReleaseShape(PxShape* shape);

    PHYSX_WRAPPER_API void AddSoftActorToScene(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API void RemoveSoftActorFromScene(PxwSoftBodyHelper* softBodyHelper);
    
    PHYSX_WRAPPER_API void AddPBDParticleSystemToScene(PxwPBDParticleSystemHelper* particleSystemHelper);

    PHYSX_WRAPPER_API void RemovePBDParticleSystemFromScene(PxwPBDParticleSystemHelper* particleSystemHelper);

    PHYSX_WRAPPER_API void AddPBDObjectToParticleSystem(PxwParticleSystemObject* particleSystemObject);

    PHYSX_WRAPPER_API void RemovePBDObjectFromParticleSystem(PxwParticleSystemObject* particleSystemObject);

    PHYSX_WRAPPER_API void AddArticulationToScene(PxwArticulationKinematicTree* articulation);

    PHYSX_WRAPPER_API void RemoveArticulationFromScene(PxwArticulationKinematicTree* articulation);

    // Particle system

    PHYSX_WRAPPER_API PxwPBDParticleSystemHelper* CreatePBDParticleSystem(PxScene* scene, PxReal particleSpacing, int maxNumParticlesForAnisotropy);

    PHYSX_WRAPPER_API void ReleasePBDParticleSystem(PxwPBDParticleSystemHelper* particleSystemHelper);

    PHYSX_WRAPPER_API void PBDParticleSystemGetBounds(PxwPBDParticleSystemHelper* particleSystemHelper, PxBounds3& bounds);

    PHYSX_WRAPPER_API PxwPBDBoxFluid* CreateCubeFluid(
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
        const PxReal buoyancy);

    PHYSX_WRAPPER_API PxwPBDFluid* CreateFluid(
        PxScene* scene,
        PxwPBDParticleSystemHelper* particleSystem,
        PxPBDMaterial* material,
        PxVec4* positions,
        PxU32 numParticles,
        PxReal particleSpacing,
        PxReal fluidDensity,
        PxU32 maxDiffuseParticles,
        PxReal buoyancy
    );

    PHYSX_WRAPPER_API PxwPBDTriMeshCloth* CreateTriMeshCloth(
        PxScene* scene,
        PxwPBDParticleSystemHelper* particleSystem,
        PxPBDMaterial* material,
        PxVec3* vertices,
        const int numVertices,
        int* indices,
        const int numIndices,
        const PxVec3* position,
        const PxReal totalMass,
        const bool inflatable,
        const PxReal blendScale,
        const PxReal pressure,
        const PxReal particleSpacing = 0.2f
    );

    PHYSX_WRAPPER_API void ResetParticleSystemObject(PxwParticleSystemObject* object);

    PHYSX_WRAPPER_API PxwParticleData GetParticleData(PxwParticleSystemObject* object);

    PHYSX_WRAPPER_API PxwAnisotropyBuffer GetAnisotropy(PxwPBDBoxFluid* object);

    PHYSX_WRAPPER_API PxwAnisotropyBuffer GetAnisotropyAll(PxwPBDParticleSystemHelper* object);

    PHYSX_WRAPPER_API PxwParticleSpringsData GetParticleSpringData(PxwPBDCloth* object);

    PHYSX_WRAPPER_API void UpdateParticleSprings(PxwPBDCloth* object, PxParticleSpring* springs, int numSprings);

    PHYSX_WRAPPER_API void AttachParticleToRigidBody(PxwPBDCloth* pbdObject, int particleIdx, PxRigidActor* rigidActor, PxVec3* pos);

    PHYSX_WRAPPER_API void DetachParticleFromRigidBody(PxwPBDCloth* pbdObject, int particleIdx, PxRigidActor* rigidActor, PxVec3* pos);

    PHYSX_WRAPPER_API void SyncParticleDataHostToDevice(PxwParticleSystemObject* object, bool copyPhase);

    PHYSX_WRAPPER_API void AddParticleRigidFilter(PxwParticleSystemObject* particleObject, PxRigidActor* rigidActor, int particleIndex);

    PHYSX_WRAPPER_API void RemoveParticleRigidFilter(PxwParticleSystemObject* particleObject, PxRigidActor* rigidActor, int particleIndex);

    PHYSX_WRAPPER_API void ReleaseParticleSystemObject(PxwParticleSystemObject* object);

    // Rigid and soft bodies

    PHYSX_WRAPPER_API PxActor* CreateDynamicRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape);

    PHYSX_WRAPPER_API PxActor* CreateKinematicRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape);

    PHYSX_WRAPPER_API PxActor* CreateStaticRigidActor(PxScene* scene, PxwTransformData* pose, PxShape* shape);

    PHYSX_WRAPPER_API PxwSoftBodyHelper* CreateFEMSoftBody(PxScene* scene, const PxU32 numVertices, const PxVec3* triVerts, const PxU32 numTriangles, const int* triIndices, PxwTransformData* pose, PxFEMSoftBodyMaterial* material, PxReal density, PxU32 iterationCount, bool useCollisionMeshForSimulation, PxU32 numVoxelsAlongLongestAABBAxis);

    PHYSX_WRAPPER_API void ResetFEMSoftBody(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API void SyncFEMSoftBodyCollisionMeshDtoH(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API PxwFEMSoftBodyMeshData GetFEMSoftBodyCollisionMeshData(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API int AttachFEMSoftBodyVertexToRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* actor, PxU32 vertId, const PxVec3* actorSpacePose);

    PHYSX_WRAPPER_API void DetachFEMSoftBodyVertexFromRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* actor, PxU32 attachmentHandle);

    PHYSX_WRAPPER_API int AttachFEMSoftBodyOverlappingAreaToRigidBody(PxwSoftBodyHelper* softBodyHelper, PxRigidActor* rigidActor, PxGeometry* rigidGeometry);
    
    PHYSX_WRAPPER_API void DetachFEMSoftBodyRigidBodyAttachments(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API int AttachFEMSoftBodyOverlappingAreaToSoftBody(PxwSoftBodyHelper* softBodyHelper0, PxwSoftBodyHelper* softBodyHelper1);

    PHYSX_WRAPPER_API void ReleaseFEMSoftBody(PxwSoftBodyHelper* softBodyHelper);

    PHYSX_WRAPPER_API void SetKinematicTarget(PxActor* actor, PxwTransformData* pose);

    PHYSX_WRAPPER_API void ReleaseMesh(PxTriangleMesh* mesh);

    // Dynamic Rigid properties
    PHYSX_WRAPPER_API void GetRigidActorPose(PxRigidActor* actor, PxwTransformData* destPose);

    PHYSX_WRAPPER_API void SetMass(PxRigidDynamic* actor, PxReal mass);

    PHYSX_WRAPPER_API PxReal GetMass(PxRigidDynamic* actor);
    
    PHYSX_WRAPPER_API void SetLinearVelocity(PxRigidDynamic* rigidDynamic, PxVec3* velocity);

    PHYSX_WRAPPER_API PxVec3 GetLinearVelocity(PxRigidDynamic* rigidDynamic);   

    PHYSX_WRAPPER_API void SetAngularVelocity(PxRigidDynamic* rigidDynamic, PxVec3* velocity);

    PHYSX_WRAPPER_API PxVec3 GetAngularVelocity(PxRigidDynamic* rigidDynamic);  

    // Robotics

    PHYSX_WRAPPER_API PxwArticulationKinematicTree* CreateArticulationKinematicTree(PxScene* scene, bool fixBase, bool disableSelfCollision);

    PHYSX_WRAPPER_API PxArticulationLink* CreateArticulationKinematicTreeBase(PxwArticulationKinematicTree* kinematicTree, PxwTransformData* basePose, PxShape* shape, float density);

    PHYSX_WRAPPER_API PxArticulationLink* AddLinkToArticulationKinematicTree(
        PxwArticulationKinematicTree* kinematicTree,
        PxArticulationLink* parentLink,
        PxwTransformData* linkPose,
        PxwRobotJointType::Enum type,
        PxwTransformData* jointPoseParent,
        PxwTransformData* jointPoseChild,
        PxArticulationAxis::Enum dofAxis,
        PxShape* shape,
        float jointLimLower,
        float jointLimUpper,
        bool isDriveJoint,
        float driveGainP,
        float driveGainD,
        float driveMaxForce,
        float density
    );

    PHYSX_WRAPPER_API void ResetArticulationKinematicTree(PxwArticulationKinematicTree* kinematicTree);

    PHYSX_WRAPPER_API void DriveArticulationKinematicTreeJoints(PxwArticulationKinematicTree* kinematicTree, float* targetJointPositions);

    PHYSX_WRAPPER_API void ReleaseArticulationKinematicTree(PxwArticulationKinematicTree* kinematicTree);

    PHYSX_WRAPPER_API void GetArticulationKinematicTreeJointPositions(PxwArticulationKinematicTree* kinematicTree, void* destArray, int length);

    PHYSX_WRAPPER_API void GetArticulationKinematicTreeLinkPoses(PxwArticulationKinematicTree* kinematicTree, void* destArray, int length);

    PHYSX_WRAPPER_API PxwArticulationRobot* CreateArticulationRobot(PxScene* scene, PxwTransformData* pose, float density);

    PHYSX_WRAPPER_API PxArticulationLink* AddLinkToRobot(
        PxwArticulationRobot* robot,
        PxwTransformData* linkPose,
        PxwRobotJointType::Enum type,
        PxwTransformData* jointPoseParent,
        PxwTransformData* jointPoseChild,
        PxShape* shape,
        float jointLimLower,
        float jointLimUpper,
        float driveGainP,
        float driveGainD,
        float driveMaxForce,
        float density
    );

    PHYSX_WRAPPER_API PxArticulationLink* AddEndEffectorLinkToRobot(
        PxwArticulationRobot* robot,
        PxwTransformData* linkPose,
        PxwRobotJointType::Enum type,
        PxwTransformData* jointPoseParent,
        PxwTransformData* jointPoseChild,
        PxShape* shape,
        float jointLimLower,
        float jointLimUpper,
        float driveGainP,
        float driveGainD,
        float driveMaxForce,
        float density
    );

    PHYSX_WRAPPER_API void ResetArticulationRobot(PxwArticulationRobot* robot);

    PHYSX_WRAPPER_API void DriveJoints(PxwArticulationRobot* robot, float* targetJointPositions);

    PHYSX_WRAPPER_API void ReleaseArticulationRobot(PxwArticulationRobot* robot);

    PHYSX_WRAPPER_API void GetRobotJointPositions(PxwArticulationRobot* robot, void* destArray, int length);

    PHYSX_WRAPPER_API void GetRobotLinkIncomingForce(PxwArticulationRobot* robot, int n, PxwSpatialForceData* destSpatialForceData);

    PHYSX_WRAPPER_API void GetRobotLinkPoses(PxwArticulationRobot* robot, void* destArray, int length);

    PHYSX_WRAPPER_API void GetRobotForwardKinematics(PxwArticulationRobot* robot, float* q, Matrix4f* destArray);

    PHYSX_WRAPPER_API void GetRobotJacobianBody(PxwArticulationRobot* robot, float* q, float* destArray, int rows, int cols);

    PHYSX_WRAPPER_API void GetRobotJacobianSpatial(PxwArticulationRobot* robot, float* q, float* destArray, int rows, int cols);

    PHYSX_WRAPPER_API bool GetRobotInverseKinematics(PxwArticulationRobot* robot, float* qInit, PxwTransformData* targetTransformEEJoint, float tolerance, int numIterations, float lambda);

    // Utility functions

    PHYSX_WRAPPER_API PxTriangleMesh* CreateBV33TriangleMesh(
        PxU32 numVertices,
        const PxVec3* vertices,
        PxU32 numTriangles,
        const PxU32* indices,
        bool skipMeshCleanup,
        bool skipEdgeData,
        bool inserted,
        bool cookingPerformance,
        bool meshSizePerfTradeoff,
        bool buildGpuData,
        float sdfSpacing,
        PxU32 sdfSubgridSize,
        PxSdfBitsPerSubgridPixel::Enum bitsPerSdfSubgridPixel
    );

    PHYSX_WRAPPER_API PxConvexMesh* CreateConvexMesh(PxU32 numVertices, const PxVec3* vertices, bool inserted, PxU32 gaussMapLimit);

    PHYSX_WRAPPER_API void FastCopy(void* src, void* dst, int size);

    PHYSX_WRAPPER_API int CreateWeldedMeshIndices(const PxVec3* vertices, int numVertices, int* uniqueVerts, int* originalToUniqueMap, float threshold);

    PHYSX_WRAPPER_API PxGeometry* CreatePxGeometry(const PxGeometryType::Enum type, const int numShapeParams, const float* shapeParams, void* shapeRef);

    PHYSX_WRAPPER_API void DeletePxGeometry(PxGeometry* geometry);

    PHYSX_WRAPPER_API PxMaterial* CreatePxMaterial(const float staticFriction, const float dynamicFriction, const float restitution);

    PHYSX_WRAPPER_API PxFEMSoftBodyMaterial* CreatePxFEMSoftBodyMaterial(const float youngs, const float poissons, const float dynamicFriction, const float damping, const PxFEMSoftBodyMaterialModel::Enum model);

    PHYSX_WRAPPER_API PxPBDMaterial* CreatePxPBDMaterial(const float friction, const float damping, const float adhesion, const float viscosity, const float vorticityConfinement,
        const float surfaceTension, const float cohesion, const float lift, const float drag, const float cflCoefficient, const float gravityScale);

    PHYSX_WRAPPER_API void ReleasePxMaterial(PxBaseMaterial* material);

    // Material physical property setters

    PHYSX_WRAPPER_API void PxPBDMaterialSetFriction(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetDamping(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetAdhesion(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetViscosity(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetVorticityConfinement(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetSurfaceTension(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetCohesion(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetLift(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetDrag(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetCflCoefficient(PxPBDMaterial* material, float value);

    PHYSX_WRAPPER_API void PxPBDMaterialSetGravityScale(PxPBDMaterial* material, float value);

    // Geometry query

    PHYSX_WRAPPER_API float PointDistance(const PxVec3* point, const PxGeometry* geom, const PxwTransformData* pose, PxVec3* closestPoint, PxU32* closestIndex);
    
    PHYSX_WRAPPER_API void ComputeGeomBounds(PxBounds3& bounds, const PxGeometry* geom, const PxwTransformData* pose, float offset, float inflation);
}
