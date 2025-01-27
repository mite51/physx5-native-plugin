#include "SoftBodyHelper.h"


namespace pxw
{
	void PxwSoftBodyHelper::AttachSoftBodyToDevice(PxScene* scene, PxSoftBody* softBody, const PxFEMParameters& femParams, const PxTransform& transform, const PxReal density, const PxReal scale, const PxU32 iterCount)
	{
		mScene = scene;

		PxSoftBodyExt::allocateAndInitializeHostMirror(*softBody, mCudaContextManager, mInitialSimPositionInvMassPinned, mInitialSimVelocityPinned, mInitialCollPositionInvMassPinned, mInitialRestPositionPinned);

		const PxReal maxInvMassRatio = 50.f;

		softBody->setParameter(femParams);
		softBody->setSolverIterationCounts(iterCount);

		PxSoftBodyExt::transform(*softBody, transform, scale, mInitialSimPositionInvMassPinned, mInitialSimVelocityPinned, mInitialCollPositionInvMassPinned, mInitialRestPositionPinned);
		PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio, mInitialSimPositionInvMassPinned);
		PxSoftBodyExt::copyToDevice(*softBody, PxSoftBodyDataFlag::eALL, mInitialSimPositionInvMassPinned, mInitialSimVelocityPinned, mInitialCollPositionInvMassPinned, mInitialRestPositionPinned);

		mSoftBody = softBody;
		mTransform = transform;
		mScale = scale;

		mCollisionMeshData->numVertices = softBody->getCollisionMesh()->getNbVertices();
		mCollisionMeshData->positionInvMass = PX_PINNED_HOST_ALLOC_T(physx::PxVec4, mCudaContextManager, softBody->getCollisionMesh()->getNbVertices());

		// Update the device buffer
		SyncCollisionVerticesDtoH();
	}

	void PxwSoftBodyHelper::AddToScene()
	{
		mScene->addActor(*mSoftBody);
	}

	void PxwSoftBodyHelper::RemoveFromScene()
	{
		RemoveRigidAttachments();
		RemoveSoftAttachments();
		mScene->removeActor(*mSoftBody);
	}

	PxU32 PxwSoftBodyHelper::AttachVertexToRigidBody(PxRigidActor* actor, PxU32 vertId, const PxVec3* actorSpacePose)
	{
		PxU32 handle = mSoftBody->addRigidAttachment(actor, vertId, *actorSpacePose);
		mRigidAttachments.pushBack(PxPair<PxRigidActor*, PxU32>(actor, handle));
		return handle;
	}

	void PxwSoftBodyHelper::DetachVertexFromRigidBody(PxRigidActor* actor, PxU32 attachmentHandle)
	{
		mSoftBody->removeRigidAttachment(actor, attachmentHandle);
	}

	PxU32 PxwSoftBodyHelper::AttachOverlappingAreaToRigidBody(PxRigidActor* rigidActor, PxGeometry* rigidGeometry)
	{
		const PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::eDEFAULT;
		PxOverlapThreadContext* threadContext = NULL;

		const int numTets0 = mSoftBody->getCollisionMesh()->getNbTetrahedrons();
		const void* tetrahedrons0 = mSoftBody->getCollisionMesh()->getTetrahedrons();
		//const PxU32* tetrahedronRemapTable0 = mSoftBody->getCollisionMesh()->getTetrahedraRemap();
		bool is16Bit0 = mSoftBody->getCollisionMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

		PxConvexMesh* convexTetMesh = NULL;
		for (int i = 0; i < numTets0; ++i)
		{
			// Retrieve indices of the vertices of the tetrahedron
			PxU32 indices0[4];
			if (is16Bit0) {
				PxU16* tets = (PxU16*)tetrahedrons0;
				indices0[0] = tets[i * 4];
				indices0[1] = tets[i * 4 + 1];
				indices0[2] = tets[i * 4 + 2];
				indices0[3] = tets[i * 4 + 3];
			}
			else {
				PxU32* tets = (PxU32*)tetrahedrons0;
				indices0[0] = tets[i * 4];
				indices0[1] = tets[i * 4 + 1];
				indices0[2] = tets[i * 4 + 2];
				indices0[3] = tets[i * 4 + 3];
			}

			PxVec3 vertices[4];
			for (int j = 0; j < 4; j++)
			{
				vertices[j] = PxVec3(mCollisionMeshData->positionInvMass[indices0[j]].x, mCollisionMeshData->positionInvMass[indices0[j]].y, mCollisionMeshData->positionInvMass[indices0[j]].z);
			}

			PxTolerancesScale tolerances;
			PxCookingParams params(tolerances);

			// GPU data is needed for particle collision detection
			params.buildGPUData = true;
			params.gaussMapLimit = 256;

			// Cook the convex mesh
			PxConvexMeshDesc convexDesc;
			convexDesc.points.count = 4;
			convexDesc.points.stride = sizeof(PxVec3);
			convexDesc.points.data = vertices;
			convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

			// Directly insert mesh into PhysX
			convexTetMesh = PxCreateConvexMesh(params, convexDesc, mScene->getPhysics().getPhysicsInsertionCallback());

			PxMeshScale scale = PxMeshScale(1);
			PxConvexMeshGeometry tetGeom = PxConvexMeshGeometry(convexTetMesh, scale);

			bool isOverlapping = PxGeometryQuery::overlap(tetGeom, PxTransform(PxIDENTITY::PxIdentity), *rigidGeometry, rigidActor->getGlobalPose(), queryFlags, threadContext);
			if (isOverlapping)
			{
				PxVec4 barycentricCoords0;
				for (int j = 0; j < 4; j++)
				{
					PxVec3 vertexPosition = vertices[j];

					// Attach the vertex if it is inside the rigid body
					PxReal distance = PxGeometryQuery::pointDistance(vertexPosition, *rigidGeometry, rigidActor->getGlobalPose());
					if (distance == 0.0)
					{
						PxVec3 vertexpositionInRigidActor = rigidActor->getGlobalPose().transformInv(vertexPosition);
						GetBarycentricCoordinatesInsideCollisionMeshTetrahedron(vertexPosition, indices0, &barycentricCoords0);
						PxU32 handle = mSoftBody->addTetRigidAttachment(rigidActor, i, barycentricCoords0, vertexpositionInRigidActor);
						mRigidAttachments.pushBack(PxPair<PxRigidActor*, PxU32>(rigidActor, handle));
					}
				}

				// Remove the collision between this tetrahedron and the rigid body anyways
				mSoftBody->addTetRigidFilter(rigidActor, i);
				mTetRigidFilters.pushBack(PxPair<PxRigidActor*, PxU32>(rigidActor, i));
			}

			convexTetMesh->release();
		}

		return PxU32();
	}

	PxU32 PxwSoftBodyHelper::AttachOverlappingAreaToSoftBody(PxwSoftBodyHelper* otherSoftBodyHelper) {
		// Initialize a counter for attachments
		PxU32 attachmentCount = 0;

		// Loop over each tetrahedron in this soft body
		const int numTets0 = mSoftBody->getCollisionMesh()->getNbTetrahedrons();
		const void* tetrahedrons0 = mSoftBody->getCollisionMesh()->getTetrahedrons();
		//const PxU32* tetrahedronRemapTable0 = mSoftBody->getCollisionMesh()->getTetrahedraRemap();
		bool is16Bit0 = mSoftBody->getCollisionMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

		std::vector<int> tetAttached0;
		std::vector<int> tetAttached1;
		for (int i = 0; i < numTets0; ++i)
		{
			// Retrieve indices of the vertices of the tetrahedron
			PxU32 indices0[4];
			if (is16Bit0) {
				PxU16* tets = (PxU16*)tetrahedrons0;
				indices0[0] = tets[i * 4];
				indices0[1] = tets[i * 4 + 1];
				indices0[2] = tets[i * 4 + 2];
				indices0[3] = tets[i * 4 + 3];
			}
			else {
				PxU32* tets = (PxU32*)tetrahedrons0;
				indices0[0] = tets[i * 4];
				indices0[1] = tets[i * 4 + 1];
				indices0[2] = tets[i * 4 + 2];
				indices0[3] = tets[i * 4 + 3];
			}
			PxVec4 barycentricCoords0;
			PxVec4 barycentricCoords1;
			for (int nv = 0; nv < 4; ++nv)
			{
				int vertId0 = indices0[nv];
				PxVec4 vertexPosition = mCollisionMeshData->positionInvMass[vertId0];
				PxVec3 p = PxVec3(vertexPosition.x, vertexPosition.y, vertexPosition.z);

				PxVec3 localposition1 = otherSoftBodyHelper->mTransform.getInverse().q.rotate(p) + otherSoftBodyHelper->mTransform.getInverse().p;
				localposition1 /= mScale;

				int j = PxTetrahedronMeshExt::findTetrahedronContainingPoint(otherSoftBodyHelper->mSoftBody->getCollisionMesh(), localposition1, barycentricCoords1);
				if (j > -1)
				{
					GetBarycentricCoordinatesInsideCollisionMeshTetrahedron(p, indices0, &barycentricCoords0);
					PxU32 handle = mSoftBody->addSoftBodyAttachment(otherSoftBodyHelper->mSoftBody, j, barycentricCoords1, i, barycentricCoords0);
					if (std::find(tetAttached0.begin(), tetAttached0.end(), i) == tetAttached0.end()) tetAttached0.push_back(i);
					if (std::find(tetAttached1.begin(), tetAttached1.end(), j) == tetAttached1.end()) tetAttached1.push_back(j);
					PxPair<PxwSoftBodyHelper*, PxU32> attachment = PxPair<PxwSoftBodyHelper*, PxU32>(otherSoftBodyHelper, handle);
					mSoftBodyAttachments.pushBack(attachment);
					otherSoftBodyHelper->mSoftBodyAttachmentsToThis.pushBack(
						PxPair<PxwSoftBodyHelper*, PxPair<PxwSoftBodyHelper*, PxU32>>(this, attachment));
				}
			}
		}

		for (int i = 0; i < tetAttached0.size(); i++)
		{
			// TODO: this is an ugly hack to basically diable collision between tetAttached0 and the other complete soft body
			mSoftBody->addSoftBodyFilter(otherSoftBodyHelper->mSoftBody, PX_MAX_TETID, tetAttached0[i]);
			PxwSoftBodyFilterTriple filter = PxwSoftBodyFilterTriple(otherSoftBodyHelper, PX_MAX_TETID, tetAttached0[i]);
			mSoftBodyFilters.pushBack(filter);
			otherSoftBodyHelper->mSoftBodyFiltersToThis.pushBack(PxPair<PxwSoftBodyHelper*, PxwSoftBodyFilterTriple>(this, filter));
		}

		return attachmentCount;
	}

	void PxwSoftBodyHelper::RemoveRigidAttachments()
	{
		for (PxU32 idx = 0; idx < mRigidAttachments.size(); ++idx)
		{
			mSoftBody->removeRigidAttachment(mRigidAttachments[idx].first, mRigidAttachments[idx].second);
		}
		for (PxU32 idx = 0; idx < mTetRigidFilters.size(); ++idx)
		{
			mSoftBody->removeTetRigidFilter(mTetRigidFilters[idx].first, mTetRigidFilters[idx].second);
		}
		mRigidAttachments.clear();
		mTetRigidFilters.clear();
	}

	void PxwSoftBodyHelper::RemoveSoftAttachment(PxPair<PxwSoftBodyHelper*, PxU32> attachment)
	{
		mSoftBody->removeSoftBodyAttachment(attachment.first->mSoftBody, attachment.second);
		mSoftBodyAttachments.remove(mSoftBodyAttachments.find(attachment) - mSoftBodyAttachments.begin());
		attachment.first->mSoftBodyAttachmentsToThis.remove(
			attachment.first->mSoftBodyAttachmentsToThis.find(PxPair<PxwSoftBodyHelper*, PxPair<PxwSoftBodyHelper*, PxU32>>(this, attachment))
			-attachment.first->mSoftBodyAttachmentsToThis.begin()
		);
	}

	void PxwSoftBodyHelper::RemoveSoftFilter(PxwSoftBodyFilterTriple filter)
	{
		mSoftBody->removeSoftBodyFilter(filter.softBody->mSoftBody, filter.tetIdx0, filter.tetIdx1);
		mSoftBodyFilters.remove(mSoftBodyFilters.find(filter) - mSoftBodyFilters.begin());
		filter.softBody->mSoftBodyFiltersToThis.remove(
			filter.softBody->mSoftBodyFiltersToThis.find(PxPair<PxwSoftBodyHelper*, PxwSoftBodyFilterTriple>(this, filter))
			- filter.softBody->mSoftBodyFiltersToThis.begin()
		);
	}

	void PxwSoftBodyHelper::RemoveSoftAttachments()
	{
		for (int idx = mSoftBodyAttachments.size() - 1; idx >= 0; --idx)
		{
			RemoveSoftAttachment(mSoftBodyAttachments[idx]);
		}
		for (int idx = mSoftBodyFilters.size() - 1; idx >= 0; --idx)
		{
			RemoveSoftFilter(mSoftBodyFilters[idx]);
		}
		for (int idx = mSoftBodyAttachmentsToThis.size() - 1; idx >= 0; --idx)
		{
			mSoftBodyAttachmentsToThis[idx].first->RemoveSoftAttachment(mSoftBodyAttachmentsToThis[idx].second);
		}
		for (int idx = mSoftBodyFiltersToThis.size() - 1; idx >= 0; --idx)
		{
			mSoftBodyFiltersToThis[idx].first->RemoveSoftFilter(mSoftBodyFiltersToThis[idx].second);
		}
		mSoftBodyAttachments.clear();
		mSoftBodyFilters.clear();
		mSoftBodyAttachmentsToThis.clear();
		mSoftBodyFiltersToThis.clear();
	}

	bool PxwSoftBodyHelper::GetBarycentricCoordinatesInsideCollisionMeshTetrahedron(PxVec3 pointPosition, PxU32* tetIndices, PxVec4* outputBaryCoord)
	{
		PxVec3 a = PxVec3(mCollisionMeshData->positionInvMass[tetIndices[0]].x, mCollisionMeshData->positionInvMass[tetIndices[0]].y, mCollisionMeshData->positionInvMass[tetIndices[0]].z);
		PxVec3 b = PxVec3(mCollisionMeshData->positionInvMass[tetIndices[1]].x, mCollisionMeshData->positionInvMass[tetIndices[1]].y, mCollisionMeshData->positionInvMass[tetIndices[1]].z);
		PxVec3 c = PxVec3(mCollisionMeshData->positionInvMass[tetIndices[2]].x, mCollisionMeshData->positionInvMass[tetIndices[2]].y, mCollisionMeshData->positionInvMass[tetIndices[2]].z);
		PxVec3 d = PxVec3(mCollisionMeshData->positionInvMass[tetIndices[3]].x, mCollisionMeshData->positionInvMass[tetIndices[3]].y, mCollisionMeshData->positionInvMass[tetIndices[3]].z);

		PxVec3& p = pointPosition;

		// Check if P coincides with any of the tetrahedron's vertices
		if (p == a || p == b || p == c || p == d)
		{
			outputBaryCoord->x = 0;
			outputBaryCoord->y = 0;
			outputBaryCoord->z = 0;
			outputBaryCoord->w = 0;
			if (p == a) outputBaryCoord->x = 1;
			if (p == b) outputBaryCoord->y = 1;
			if (p == c) outputBaryCoord->z = 1;
			if (p == d) outputBaryCoord->w = 1;
			return true;
		}

		PxVec3 v0 = b - a, v1 = c - a, v2 = d - a, v3 = p - a;

		// For tetrahedron
		double denom = v0.dot(v1.cross(v2));
		double b0 = ((p - b).dot((c - b).cross(d - b))) / denom;
		double b1 = ((p - c).dot((d - c).cross(a - c))) / denom;
		double b2 = ((p - d).dot((a - d).cross(b - d))) / denom;
		double b3 = 1.0 - b0 - b1 - b2;

		if (!(b0 >= 0 && b1 >= 0 && b2 >= 0 && b0 + b1 + b2 <= 1)) return false;

		outputBaryCoord->x = b0;
		outputBaryCoord->y = b1;
		outputBaryCoord->z = b2;
		outputBaryCoord->w = b3;

		return true;
	}

	void PxwSoftBodyHelper::ResetObject()
	{
		if (mInitialSimPositionInvMassPinned)
		{
			PxSoftBodyExt::copyToDevice(*mSoftBody, PxSoftBodyDataFlag::eALL, mInitialSimPositionInvMassPinned, mInitialSimVelocityPinned, mInitialCollPositionInvMassPinned, mInitialRestPositionPinned);
			SyncCollisionVerticesDtoH();
		}
	}

}
