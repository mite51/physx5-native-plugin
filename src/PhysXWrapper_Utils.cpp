#include "PhysXWrapper.h"

using namespace pxw;

namespace pxw
{
	void PhysXWrapper::SetupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
	{
		// we suppress the triangle mesh remap table computation to gain some speed, as we will not need it 
		// in this snippet
		params.suppressTriangleMeshRemapTable = true;

		// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
		// The following conditions are true for a valid triangle mesh :
		//  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
		//  2. There are no large triangles(within specified PxTolerancesScale.)
		// It is recommended to run a separate validation check in debug/checked builds, see below.

		if (!skipMeshCleanup)
			params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
		else
			params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

		// If eDISABLE_ACTIVE_EDGES_PRECOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
		// marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
		// the collision behavior, as all edges of the triangle mesh will now be considered active.
		if (!skipEdgeData)
			params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
		else
			params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	}

	PxTriangleMesh* PhysXWrapper::CreateBV33TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
		bool skipMeshCleanup, bool skipEdgeData, bool inserted, bool cookingPerformance, bool meshSizePerfTradeoff, bool buildGpuData, PxReal sdfSpacing,
		PxU32 sdfSubgridSize, PxSdfBitsPerSubgridPixel::Enum bitsPerSdfSubgridPixel)
	{
		// OutputDebugString(L"Create mesh");
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create triangle mesh\n");
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = numVertices;
		meshDesc.points.data = vertices;
		meshDesc.points.stride = sizeof(PxVec3);
		meshDesc.triangles.count = numTriangles;
		meshDesc.triangles.data = indices;
		meshDesc.triangles.stride = 3 * sizeof(PxU32);

		PxSDFDesc sdfDesc;
		if (sdfSpacing > 0.f)
		{
			sdfDesc.spacing = sdfSpacing;
			sdfDesc.subgridSize = sdfSubgridSize;
			sdfDesc.bitsPerSubgridPixel = bitsPerSdfSubgridPixel;
			sdfDesc.numThreadsForSdfConstruction = 16;

			meshDesc.sdfDesc = &sdfDesc;
		}

		PxTolerancesScale scale;
		PxCookingParams params(scale);

		// Create BVH33 midphase
		params.midphaseDesc = PxMeshMidPhase::eBVH33;

		// setup common cooking params
		SetupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

		// The COOKING_PERFORMANCE flag for BVH33 midphase enables a fast cooking path at the expense of somewhat lower quality BVH construction.	
		if (cookingPerformance)
			params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eCOOKING_PERFORMANCE;
		else
			params.midphaseDesc.mBVH33Desc.meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;

		// If meshSizePerfTradeoff is set to true, smaller mesh cooked mesh is produced. The mesh size/performance trade-off
		// is controlled by setting the meshSizePerformanceTradeOff from 0.0f (smaller mesh) to 1.0f (larger mesh).
		if (meshSizePerfTradeoff)
		{
			params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.0f;
		}
		else
		{
			// using the default value
			params.midphaseDesc.mBVH33Desc.meshSizePerformanceTradeOff = 0.55f;
		}

		if (buildGpuData)
		{
			// GPU data is needed for particle collision detection
			params.buildGPUData = true;
		}

#if defined(PX_CHECKED) || defined(PX_DEBUG)
		// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
		// We should check the validity of provided triangles in debug/checked builds though.
		if (skipMeshCleanup)
		{
			PX_ASSERT(PxValidateTriangleMesh(params, meshDesc));
		}
#endif // DEBUG


		PxTriangleMesh* triMesh = NULL;
		PxU32 meshSize = 0;

		// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
		if (inserted)
		{
			triMesh = PxCreateTriangleMesh(params, meshDesc, mPhysics->getPhysicsInsertionCallback());
		}
		else
		{
			PxDefaultMemoryOutputStream outBuffer;
			PxCookTriangleMesh(params, meshDesc, outBuffer);

			PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
			triMesh = mPhysics->createTriangleMesh(stream);

			meshSize = outBuffer.getSize();
		}

		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create triangle mesh done\n");
		return triMesh;
	}

	struct Vec3Hash {
		float threshold;

		Vec3Hash(float thresh) : threshold(thresh) {}

		size_t operator()(const PxVec3& v) const {
			// Scale down the components by the threshold and hash them
			return std::hash<float>()(std::floor(v.x / threshold)) ^
				std::hash<float>()(std::floor(v.y / threshold)) ^
				std::hash<float>()(std::floor(v.z / threshold));
		}
	};

	struct Vec3Equal {
		float threshold;

		Vec3Equal(float thresh) : threshold(thresh) {}

		bool operator()(const PxVec3& a, const PxVec3& b) const {
			// Check if the distance between a and b is within the threshold
			return (a - b).magnitudeSquared() < threshold * threshold;
		}
	};

	int PhysXWrapper::CreateWeldedMeshIndices(const PxVec3* vertices, int numVertices, int* uniqueVerts, int* originalToUniqueMap, float threshold) {
		std::unordered_map<PxVec3, int, Vec3Hash, Vec3Equal> uniqueVertexMap(10, Vec3Hash(threshold), Vec3Equal(threshold));

		int uniqueCount = 0;
		for (int i = 0; i < numVertices; ++i) {
			const PxVec3& vertex = vertices[i];
			auto it = uniqueVertexMap.find(vertex);
			if (it == uniqueVertexMap.end()) {
				uniqueVertexMap[vertex] = uniqueCount;
				uniqueVerts[uniqueCount] = i;
				originalToUniqueMap[i] = uniqueCount;
				uniqueCount++;
			}
			else {
				originalToUniqueMap[i] = it->second;
			}
		}

		return uniqueCount;
	}

	PxConvexMesh* PhysXWrapper::CreateConvexMesh(PxU32 numVerts, const PxVec3* verts, bool directInsertion, PxU32 gaussMapLimit)
	{
		PxTolerancesScale tolerances;
		PxCookingParams params(tolerances);

		// GPU data is needed for particle collision detection
		params.buildGPUData = true;

		// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
		// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
		params.gaussMapLimit = gaussMapLimit;

		// Setup the convex mesh descriptor
		PxConvexMeshDesc desc;

		// We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
		desc.points.data = verts;
		desc.points.count = numVerts;
		desc.points.stride = sizeof(PxVec3);
		desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxU32 meshSize = 0;
		PxConvexMesh* convex = NULL;

		if (directInsertion)
		{
			// Directly insert mesh into PhysX
			convex = PxCreateConvexMesh(params, desc, mPhysics->getPhysicsInsertionCallback());
			PX_ASSERT(convex);
		}
		else
		{
			// Serialize the cooked mesh into a stream.
			PxDefaultMemoryOutputStream outStream;
			bool res = PxCookConvexMesh(params, desc, outStream);
			PX_UNUSED(res);
			PX_ASSERT(res);
			meshSize = outStream.getSize();

			// Create the mesh from a stream.
			PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
			convex = mPhysics->createConvexMesh(inStream);
			PX_ASSERT(convex);
		}
		return convex;
	}

	/**
	* \brief Creates a geometry given the geometry type.
	*/
	PxGeometry* PhysXWrapper::CreatePxGeometry(const PxGeometryType::Enum type, const int numShapeParams, const float* shapeParams, void* shapeRef)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create geometry\n");
		PxMeshScale scale;
		switch (type)
		{
		case PxGeometryType::eSPHERE:
			if (numShapeParams != 1)
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Incorrect parameter number\n");
				break;
			}
			return new PxSphereGeometry(shapeParams[0]);
		case PxGeometryType::eBOX:
			if (numShapeParams != 3)
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Incorrect parameter number\n");
				break;
			}
			return new PxBoxGeometry(shapeParams[0], shapeParams[1], shapeParams[2]);
		case PxGeometryType::eCAPSULE:
			if (numShapeParams != 2)
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Incorrect parameter number\n");
				break;
			}
			return new PxCapsuleGeometry(shapeParams[0], shapeParams[1]);
		case PxGeometryType::eTRIANGLEMESH:
			if (numShapeParams == 1) scale = PxMeshScale(shapeParams[0]);
			else if (numShapeParams == 3) scale = PxMeshScale(PxVec3(shapeParams[0], shapeParams[1], shapeParams[2]));
			else if (numShapeParams == 7) scale = PxMeshScale(PxVec3(shapeParams[0], shapeParams[1], shapeParams[2]), PxQuat(shapeParams[3], shapeParams[4], shapeParams[5], shapeParams[6]));
			else
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Incorrect parameter number\n");
				break;
			}
			return new PxTriangleMeshGeometry(static_cast<PxTriangleMesh*>(shapeRef), scale);
		case PxGeometryType::eCONVEXMESH:
			if (numShapeParams == 1) scale = PxMeshScale(shapeParams[0]);
			else if (numShapeParams == 3) scale = PxMeshScale(PxVec3(shapeParams[0], shapeParams[1], shapeParams[2]));
			else if (numShapeParams == 7) scale = PxMeshScale(PxVec3(shapeParams[0], shapeParams[1], shapeParams[2]), PxQuat(shapeParams[3], shapeParams[4], shapeParams[5], shapeParams[6]));
			else
			{
				PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Incorrect parameter number\n");
				break;
			}
			return new PxConvexMeshGeometry(static_cast<PxConvexMesh*>(shapeRef), scale); // uses tighter bounds by default
		default:
			PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Geometry type not supported\n");
			break;
		}
		return NULL;
	}

	/*
	* \brief Create a rigid body material
	*/
	PxMaterial* PhysXWrapper::CreateMaterial(const float staticFriction, const float dynamicFriction, const float restitution)
	{
		PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Create rigid material\n");
		return mPhysics->createMaterial(staticFriction, dynamicFriction, restitution);
	}

	/*
	* \brief Create a PBD material
	*/
	PxPBDMaterial* PhysXWrapper::CreatePBDMaterial(const float friction, const float damping, const float adhesion, const float viscosity, const float vorticityConfinement, const float surfaceTension, const float cohesion, const float lift, const float drag, const float cflCoefficient, const float gravityScale)
	{
		return mPhysics->createPBDMaterial(friction, damping, adhesion, viscosity, vorticityConfinement, surfaceTension, cohesion, lift, drag, cflCoefficient, gravityScale);
	}


	PxFEMSoftBodyMaterial* PhysXWrapper::CreateFEMSoftBodyMaterial(const float youngs, const float poissons, const float dynamicFriction, const float damping, const PxFEMSoftBodyMaterialModel::Enum model)
	{
		PxFEMSoftBodyMaterial* material = mPhysics->createFEMSoftBodyMaterial(youngs, poissons, dynamicFriction);
		material->setDamping(damping);
		material->setMaterialModel(model);
		return material;
	}
}