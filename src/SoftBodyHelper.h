#pragma once

#include "PxPhysicsAPI.h"
#include "DataInterop.h"
#include <gpu/PxPhysicsGpu.h>
#include "extensions/PxSoftBodyExt.h"
#include "extensions/PxTetrahedronMeshExt.h"
#include "geometry/PxGeometryQuery.h"
#include <vector>

namespace pxw
{
	class PxwSoftBodyHelper;

	template <class F, class S>
	PX_INLINE bool operator!=(const PxPair<F, S>& a, const PxPair<F, S>& b)
	{
		return !(a == b);
	}

	class PxwSoftBodyFilterTriple {
	public:
		PxwSoftBodyHelper* softBody;
		PxU32 tetIdx0;
		PxU32 tetIdx1;

		PxwSoftBodyFilterTriple(PxwSoftBodyHelper* e1, PxU32 e2, PxU32 e3)
		{
			softBody = e1;
			tetIdx0 = e2;
			tetIdx1 = e3;
		}

		bool operator==(const PxwSoftBodyFilterTriple& other) const {
			return softBody == other.softBody && tetIdx0 == other.tetIdx0 && tetIdx1 == other.tetIdx1;
		}

		bool operator!=(const PxwSoftBodyFilterTriple& other) const {
			return !(*this == other);
		}
	};


	class PxwSoftBodyHelper
	{
	protected:
		PxScene* mScene;
		PxSoftBody* mSoftBody;
		PxTransform mTransform;
		float mScale;
		PxwFEMSoftBodyMeshData* mCollisionMeshData;
		PxCudaContextManager* mCudaContextManager;
		PxVec4* mInitialSimPositionInvMassPinned;
		PxVec4* mInitialSimVelocityPinned;
		PxVec4* mInitialCollPositionInvMassPinned;
		PxVec4* mInitialRestPositionPinned;

		PxArray<PxPair<PxRigidActor*, PxU32>> mRigidAttachments;
		PxArray<PxPair<PxRigidActor*, PxU32>> mTetRigidFilters;

		// <other soft body helper, handle>
		PxArray<PxPair<PxwSoftBodyHelper*, PxU32>> mSoftBodyAttachments;

		// <other soft body helper, a pair in the other soft body's mSoftBodyAttachments>
		PxArray<PxPair<PxwSoftBodyHelper*, PxPair<PxwSoftBodyHelper*, PxU32>>> mSoftBodyAttachmentsToThis;

		PxArray<PxwSoftBodyFilterTriple> mSoftBodyFilters;
		PxArray<PxPair<PxwSoftBodyHelper*, PxwSoftBodyFilterTriple>> mSoftBodyFiltersToThis;
	public:
		PxwSoftBodyHelper(PxCudaContextManager* cudaContextManager)
		{
			mScene = NULL;
			mCollisionMeshData = new PxwFEMSoftBodyMeshData();
			mSoftBody = NULL;
			mCudaContextManager = cudaContextManager;
			mTransform = PxTransform(PxIdentity);
			mScale = 1.0f;
			mInitialSimPositionInvMassPinned = NULL;
			mInitialSimVelocityPinned = NULL;
			mInitialCollPositionInvMassPinned = NULL;
			mInitialRestPositionPinned = NULL;
		}

		~PxwSoftBodyHelper()
		{
		}

		void AttachSoftBodyToDevice(PxScene* scene, PxSoftBody* softBody, const PxFEMParameters& femParams, const PxTransform& transform,
			const PxReal density, const PxReal scale, const PxU32 iterCount);

		void AddToScene();

		void RemoveFromScene();

		PxU32 AttachVertexToRigidBody(PxRigidActor* actor, PxU32 vertId, const PxVec3* actorSpacePose);

		void DetachVertexFromRigidBody(PxRigidActor* actor, PxU32 attachmentHandle);

		PxU32 AttachOverlappingAreaToRigidBody(PxRigidActor* rigidActor, PxGeometry* rigidGeometry);

		PxU32 AttachOverlappingAreaToSoftBody(PxwSoftBodyHelper* otherSoftBodyHelper);

		void RemoveRigidAttachments();

		void RemoveSoftAttachment(PxPair<PxwSoftBodyHelper*, PxU32> attachment);

		void RemoveSoftFilter(PxwSoftBodyFilterTriple filter);

		void RemoveSoftAttachments();

		bool GetBarycentricCoordinatesInsideCollisionMeshTetrahedron(PxVec3 pointPosition, PxU32* tetIndices, PxVec4* outputBaryCoord);

		void ResetObject();

		void Release()
		{
			PX_PINNED_HOST_FREE(mCudaContextManager, mInitialSimPositionInvMassPinned);
			PX_PINNED_HOST_FREE(mCudaContextManager, mInitialSimVelocityPinned);
			PX_PINNED_HOST_FREE(mCudaContextManager, mInitialCollPositionInvMassPinned);
			PX_PINNED_HOST_FREE(mCudaContextManager, mInitialRestPositionPinned);
			mInitialSimPositionInvMassPinned = NULL;
			mInitialSimVelocityPinned = NULL;
			mInitialCollPositionInvMassPinned = NULL;
			mInitialRestPositionPinned = NULL;
			if (mSoftBody)
				mSoftBody->release();
			if (mCollisionMeshData->positionInvMass)
				PX_PINNED_HOST_FREE(mCudaContextManager, mCollisionMeshData->positionInvMass);
			if (mCollisionMeshData->velocity)
				PX_PINNED_HOST_FREE(mCudaContextManager, mCollisionMeshData->positionInvMass);
			delete mCollisionMeshData;
		}

		void SyncCollisionVerticesDtoH()
		{
			PxTetrahedronMesh* tetMesh = mSoftBody->getCollisionMesh();
			mCudaContextManager->getCudaContext()->memcpyDtoH(mCollisionMeshData->positionInvMass, reinterpret_cast<CUdeviceptr>(mSoftBody->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(PxVec4));
		}

		PxwFEMSoftBodyMeshData GetCollisionMesh()
		{
			return *mCollisionMeshData;
		}
	};
}
