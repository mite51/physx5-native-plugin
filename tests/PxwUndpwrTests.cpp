// Native determinism tests for the UNDPWR support layer.
//
// These run without Unity so the core guarantees can be validated in isolation.
//
// The central result they establish is the shape of the tick function. PhysX cannot
// round-trip a rigid body pose exactly:
//
//   * setGlobalPose stores pose.getNormalized(), and normalisation is not
//     idempotent. Capturing the raw pose produces a two-cycle where restore flips
//     between two neighbouring quaternions forever. PxwUndpwr.cpp normalises at
//     capture time, which lands the stored value on the stable member of the cycle.
//
//   * setGlobalPose stores the pose composed with the centre-of-mass transform and
//     getGlobalPose composes the inverse back. When the centre of mass is offset
//     from the actor origin, that is a subtract-what-you-added round trip that loses
//     the low bits, so capture(restore(x)) drifts slightly away from x every time.
//     No amount of care at capture time fixes this; it is float cancellation.
//
// The consequence is that a tick must be a pure function of a snapshot, not of
// whatever happens to be live in the scene:
//
//     Tick(T):  restore(snapshot[T]); step(); snapshot[T+1] = capture()
//
// Every tick begins by restoring its own snapshot, so a replayed tick starts from
// byte-identical state to the original. Rolling back is then just "put an older
// snapshot back in the variable and keep going", with no special cases. This is why
// the original UNDPWR needed to reapply its state snapshot inside the main loop.
//
// Build with -DBUILD_TESTS=ON, then run PxwUndpwrTests.

#include "PxwUndpwr.h"
#include "PxwAPIs.h"
#include "VehicleHelper.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using namespace physx;
using namespace pxw;

namespace
{
	int gFailures = 0;
	int gChecks = 0;

	void Check(bool condition, const std::string& what)
	{
		++gChecks;
		std::printf("  %s  %s\n", condition ? "ok  " : "FAIL", what.c_str());
		if (!condition)
		{
			++gFailures;
		}
	}

	// Reports an observation without affecting the exit code. Used for the
	// characterisation runs whose purpose is to document PhysX behaviour rather than
	// to assert a required outcome.
	void Observe(bool condition, const std::string& what)
	{
		std::printf("  %s  %s\n", condition ? "yes " : "no  ", what.c_str());
	}

	const float kDt = 1.0f / 60.0f;

	// Framework sleep parameters shared by the sleep tests. A body must stay under
	// both speed thresholds for kSleepTicks consecutive steps before it is slept.
	const float kSleepLinear = 0.05f;
	const float kSleepAngular = 0.05f;
	const PxU32 kSleepTicks = 20u;

	// Set by the PCM characterisation run. Persistent contact manifolds are the main
	// piece of simulation state that survives a step and is not part of any snapshot.
	bool gUsePcm = true;

	// Set by the articulation runs, which ask the same questions of both solvers. PGS is
	// the framework default after the Phase 1 decision, because it is the one measured to
	// make replay transparent to varying rollback depth; TGS stays available for a strictly
	// fixed-horizon session.
	PxSolverType::Enum gSolverType = PxSolverType::eTGS;

	const char* SolverName()
	{
		return gSolverType == PxSolverType::ePGS ? "PGS" : "TGS";
	}

	PxwSceneDesc MakeDeterministicSceneDesc()
	{
		PxwSceneDesc desc;
		desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
		desc.flags = PxwSceneFlag::eENABLE_ENHANCED_DETERMINISM
			| PxwSceneFlag::eDISABLE_PVD;
		if (gUsePcm)
		{
			desc.flags |= PxwSceneFlag::eENABLE_PCM;
		}
		desc.pruningStructureType = PxPruningStructureType::eDYNAMIC_AABB_TREE;
		desc.solverType = gSolverType;
		desc.broadPhaseType = -1;
		desc.cpuWorkerThreads = 0;
		desc.useGpu = 0;
		desc.bounceThresholdVelocity = 0.2f;
		desc.frictionOffsetThreshold = 0.04f;
		desc.ccdMaxPasses = 1;
		return desc;
	}

	// ---------------------------------------------------------------------------

	// A deliberately contact-heavy scene: a stack of boxes plus falling spheres, so
	// persistent manifolds and friction anchors are actually exercised.
	struct TestWorld
	{
		PxwWorld* world;
		std::vector<PxU32> dynamicIds;

		TestWorld() : world(NULL) {}

		// settled:   boxes stacked squarely and no spheres, so the scene can come to
		//            rest and bodies are allowed to fall asleep
		// offsetCoM: shifts the centre of mass away from the actor origin, the
		//            configuration the original UNDPWR notes flagged as problematic
		//            ("works great if center of mass is 0,0,0")
		// sleepTicks: 0 keeps everything awake and pinned (the default). A non-zero
		//            value turns on framework sleeping, so a test that wants bodies to
		//            fall asleep has to ask for it. See PxwWorldSetSleepParams.
		void Build(bool reverseRegistrationOrder, bool settled = false, bool offsetCoM = false,
			PxReal sleepLinearThreshold = 0.0f, PxReal sleepAngularThreshold = 0.0f, PxU32 sleepTicks = 0,
			int boxCount = 8)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);
			PxwWorldSetSleepParams(world, sleepLinearThreshold, sleepAngularThreshold, sleepTicks);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

			{
				PxBoxGeometry groundGeom(50.0f, 1.0f, 50.0f);
				PxShape* shape = physics->createShape(groundGeom, *material, true);
				PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
				ground->attachShape(*shape);
				shape->release();
				PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);
			}

			struct Spawn { PxU32 id; PxVec3 pos; bool sphere; };
			std::vector<Spawn> spawns;

			for (int i = 0; i < boxCount; ++i)
			{
				Spawn s;
				s.id = 100u + static_cast<PxU32>(i);
				s.pos = PxVec3(settled ? 0.0f : 0.02f * i, 0.5f + 1.001f * i, 0.0f);
				s.sphere = false;
				spawns.push_back(s);
			}
			if (!settled)
			{
				for (int i = 0; i < 6; ++i)
				{
					Spawn s;
					s.id = 200u + static_cast<PxU32>(i);
					s.pos = PxVec3(-1.5f + 0.6f * i, 12.0f + 0.35f * i, 0.15f * i);
					s.sphere = true;
					spawns.push_back(s);
				}
			}

			if (reverseRegistrationOrder)
			{
				std::vector<Spawn> reversed(spawns.rbegin(), spawns.rend());
				spawns.swap(reversed);
			}

			for (size_t i = 0; i < spawns.size(); ++i)
			{
				const Spawn& s = spawns[i];
				PxShape* shape;
				if (s.sphere)
				{
					PxSphereGeometry geom(0.5f);
					shape = physics->createShape(geom, *material, true);
				}
				else
				{
					PxBoxGeometry geom(0.5f, 0.5f, 0.5f);
					shape = physics->createShape(geom, *material, true);
				}

				PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(s.pos));
				body->attachShape(*shape);
				shape->release();

				if (offsetCoM)
				{
					const PxVec3 massLocalPose(0.17f, -0.23f, 0.11f);
					PxRigidBodyExt::updateMassAndInertia(*body, 10.0f, &massLocalPose);
				}
				else
				{
					PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
				}
				PxwApplyDeterministicRigidDefaults(body, 8, 2);

				PxwWorldRegister(world, s.id, body, PxwHandleKind::eRIGID_DYNAMIC);
				dynamicIds.push_back(s.id);
			}

			material->release();
			PxwWorldCommitPending(world);
		}

		void Destroy()
		{
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
			}
		}

		PxU64 Hash() { return PxwWorldHashState(world); }

		std::vector<PxU8> Capture()
		{
			std::vector<PxU8> buffer(PxwWorldStateSize(world));
			PxU64 hash = 0;
			const PxU32 written = PxwWorldCaptureState(world, buffer.data(), static_cast<PxU32>(buffer.size()), &hash);
			buffer.resize(written);
			return buffer;
		}

		PxI32 Restore(const std::vector<PxU8>& buffer)
		{
			return PxwWorldRestoreState(world, buffer.data(), static_cast<PxU32>(buffer.size()));
		}

		// Applying a force keeps bodies awake and makes the trajectory sensitive to
		// any state that was not restored correctly.
		void ApplyInput(int tickIndex)
		{
			const float wobble = 0.35f * static_cast<float>((tickIndex % 7) - 3);
			for (size_t i = 0; i < dynamicIds.size(); ++i)
			{
				PxRigidDynamic* body = static_cast<PxRigidDynamic*>(PxwWorldFindHandle(world, dynamicIds[i]));
				if (body != NULL && !body->isSleeping())
				{
					body->addForce(PxVec3(wobble, 0.0f, wobble * 0.5f), PxForceMode::eACCELERATION);
				}
			}
		}

		// Adds and commits a dynamic sphere after the world is already running. Used to
		// drop a body onto a sleeping stack and check that the impact wakes it.
		PxU32 SpawnDynamicSphere(PxU32 id, PxVec3 pos, float radius, float density)
		{
			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);
			PxSphereGeometry geom(radius);
			PxShape* shape = physics->createShape(geom, *material, true);
			PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(pos));
			body->attachShape(*shape);
			shape->release();
			PxRigidBodyExt::updateMassAndInertia(*body, density);
			PxwApplyDeterministicRigidDefaults(body, 8, 2);
			PxwWorldRegister(world, id, body, PxwHandleKind::eRIGID_DYNAMIC);
			material->release();
			dynamicIds.push_back(id);
			PxwWorldCommitPending(world);
			return id;
		}
	};

	// Drives a TestWorld using the snapshot-in, snapshot-out tick function described
	// at the top of this file.
	struct SimRunner
	{
		TestWorld world;
		std::vector<PxU8> snapshot;
		PxU32 resetMode;
		bool restoreEachTick;
		bool applyInput;

		SimRunner() : resetMode(PxwContactResetMode::eNONE), restoreEachTick(true), applyInput(true) {}

		void Build(bool reverseRegistrationOrder, bool settled = false, bool offsetCoM = false,
			PxReal sleepLinearThreshold = 0.0f, PxReal sleepAngularThreshold = 0.0f, PxU32 sleepTicks = 0)
		{
			world.Build(reverseRegistrationOrder, settled, offsetCoM,
				sleepLinearThreshold, sleepAngularThreshold, sleepTicks);
			snapshot = world.Capture();
		}

		void Destroy() { world.Destroy(); }

		// Rewinding means putting an older snapshot back. When the tick does not
		// restore on its own, the rewind has to do it.
		void Rewind(const std::vector<PxU8>& to)
		{
			snapshot = to;
			if (!restoreEachTick)
			{
				RestoreAndReset();
			}
		}

		// eREINSERT resets sleep state as a side effect of re-adding actors, so for
		// that mode the restore has to come afterwards to put it back. The other modes
		// leave actor state alone and want the reset to see the restored poses.
		void RestoreAndReset()
		{
			if (resetMode == PxwContactResetMode::eREINSERT)
			{
				PxwWorldResetContactStateEx(world.world, resetMode);
				world.Restore(snapshot);
			}
			else
			{
				world.Restore(snapshot);
				PxwWorldResetContactStateEx(world.world, resetMode);
			}
		}

		void Tick(int tickIndex)
		{
			if (restoreEachTick)
			{
				RestoreAndReset();
			}
			if (applyInput)
			{
				world.ApplyInput(tickIndex);
			}
			PxwWorldStep(world.world, kDt);
			snapshot = world.Capture();
		}

		PxU64 SnapshotHash() const { return PxwHashBuffer(snapshot.data(), static_cast<PxU32>(snapshot.size())); }
	};

	// ---------------------------------------------------------------------------

	// Walks two state blobs and reports which entry and which field differ. The blob
	// layout mirrors PxwUndpwr.cpp: a 16 byte header, then per entry a 16 byte header
	// followed by the payload.
	void DiffStateBlobs(const char* label, const std::vector<PxU8>& lhs, const std::vector<PxU8>& rhs, int maxReported = 3)
	{
		if (lhs.size() != rhs.size())
		{
			std::printf("        %s: size differs (%zu vs %zu)\n", label, lhs.size(), rhs.size());
			return;
		}
		if (std::memcmp(lhs.data(), rhs.data(), lhs.size()) == 0)
		{
			std::printf("        %s: identical\n", label);
			return;
		}

		struct BlobHeader { PxU32 magic, version, entryCount, totalBytes; };
		struct BlobEntry { PxU32 stableId, kind, payloadBytes, reserved; };
		struct BlobRigid
		{
			PxVec3 position; PxQuat rotation;
			PxVec3 linearVelocity; PxVec3 angularVelocity;
			PxReal wakeCounter; PxU32 flags;
		};

		const BlobHeader* header = reinterpret_cast<const BlobHeader*>(lhs.data());
		size_t offset = sizeof(BlobHeader);
		int reported = 0;

		for (PxU32 i = 0; i < header->entryCount && offset < lhs.size(); ++i)
		{
			const BlobEntry* entry = reinterpret_cast<const BlobEntry*>(lhs.data() + offset);
			const size_t payloadOffset = offset + sizeof(BlobEntry);

			if (entry->kind != PxwHandleKind::eARTICULATION &&
				std::memcmp(lhs.data() + payloadOffset, rhs.data() + payloadOffset, entry->payloadBytes) != 0 &&
				reported < maxReported)
			{
				const BlobRigid* a = reinterpret_cast<const BlobRigid*>(lhs.data() + payloadOffset);
				const BlobRigid* b = reinterpret_cast<const BlobRigid*>(rhs.data() + payloadOffset);
				std::printf("        %s: id %u\n", label, entry->stableId);
				if (a->position != b->position)
					std::printf("           position (%.9g %.9g %.9g) -> (%.9g %.9g %.9g)\n",
						a->position.x, a->position.y, a->position.z, b->position.x, b->position.y, b->position.z);
				if (a->rotation.x != b->rotation.x || a->rotation.y != b->rotation.y ||
					a->rotation.z != b->rotation.z || a->rotation.w != b->rotation.w)
					std::printf("           rotation (%.9g %.9g %.9g %.9g) -> (%.9g %.9g %.9g %.9g)\n",
						a->rotation.x, a->rotation.y, a->rotation.z, a->rotation.w,
						b->rotation.x, b->rotation.y, b->rotation.z, b->rotation.w);
				if (a->linearVelocity != b->linearVelocity)
					std::printf("           linVel   (%.9g %.9g %.9g) -> (%.9g %.9g %.9g)\n",
						a->linearVelocity.x, a->linearVelocity.y, a->linearVelocity.z,
						b->linearVelocity.x, b->linearVelocity.y, b->linearVelocity.z);
				if (a->angularVelocity != b->angularVelocity)
					std::printf("           angVel   (%.9g %.9g %.9g) -> (%.9g %.9g %.9g)\n",
						a->angularVelocity.x, a->angularVelocity.y, a->angularVelocity.z,
						b->angularVelocity.x, b->angularVelocity.y, b->angularVelocity.z);
				if (a->wakeCounter != b->wakeCounter)
					std::printf("           wakeCtr  %.9g -> %.9g\n", a->wakeCounter, b->wakeCounter);
				if (a->flags != b->flags)
					std::printf("           flags    %u -> %u\n", a->flags, b->flags);
				++reported;
			}

			offset = payloadOffset + entry->payloadBytes;
		}
	}

	// ---------------------------------------------------------------------------

	void TestRegistryOrdering()
	{
		std::printf("TestRegistryOrdering\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);
		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.0f);

		const PxU32 ids[] = { 40, 10, 30, 20 };
		for (int i = 0; i < 4; ++i)
		{
			PxBoxGeometry geom(0.5f, 0.5f, 0.5f);
			PxShape* shape = physics->createShape(geom, *material, true);
			PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(PxVec3(static_cast<float>(i) * 3.0f, 5.0f, 0.0f)));
			body->attachShape(*shape);
			shape->release();
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			PxwWorldRegister(world, ids[i], body, PxwHandleKind::eRIGID_DYNAMIC);
		}
		PxwWorldCommitPending(world);

		Check(PxwWorldGetEntryCount(world) == 4, "all four bodies registered");

		std::vector<PxwPoseEntry> poses(8);
		const PxU32 count = PxwWorldReadPoses(world, poses.data(), 8);
		Check(count == 4, "pose readback returned every entry");

		bool ascending = true;
		for (PxU32 i = 1; i < count; ++i)
		{
			if (poses[i].stableId <= poses[i - 1].stableId)
			{
				ascending = false;
			}
		}
		Check(ascending, "entries are reported in ascending stable-ID order regardless of registration order");

		PxBoxGeometry geom(0.5f, 0.5f, 0.5f);
		PxShape* dupShape = physics->createShape(geom, *material, true);
		PxRigidDynamic* dup = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 20.0f, 0.0f)));
		dup->attachShape(*dupShape);
		dupShape->release();
		Check(PxwWorldRegister(world, 10, dup, PxwHandleKind::eRIGID_DYNAMIC) == PxwResult::eDUPLICATE_ID,
			"registering a duplicate stable ID is rejected");
		dup->release();

		Check(PxwWorldUnregister(world, 999) == PxwResult::eUNKNOWN_ID, "unregistering an unknown ID is rejected");

		material->release();
		PxwWorldDestroy(world);
	}

	void TestBaselineDeterminism()
	{
		std::printf("TestBaselineDeterminism\n");

		SimRunner a, b;
		a.Build(false);
		b.Build(false);

		bool identical = true;
		for (int tick = 0; tick < 240; ++tick)
		{
			a.Tick(tick);
			b.Tick(tick);
			if (a.SnapshotHash() != b.SnapshotHash())
			{
				std::printf("        diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}
		Check(identical, "two identically built worlds stay bit-identical for 240 ticks");

		a.Destroy();
		b.Destroy();
	}

	void TestRegistrationOrderIndependence()
	{
		std::printf("TestRegistrationOrderIndependence\n");

		SimRunner a, b;
		a.Build(false);
		b.Build(true); // same bodies, registered in the opposite order

		bool identical = true;
		for (int tick = 0; tick < 240; ++tick)
		{
			a.Tick(tick);
			b.Tick(tick);
			if (a.SnapshotHash() != b.SnapshotHash())
			{
				std::printf("        diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}
		Check(identical, "registration order does not affect the simulation, because insertion is sorted by stable ID");

		a.Destroy();
		b.Destroy();
	}

	// Characterises how faithfully PhysX round-trips a pose. Informational: the tick
	// function does not depend on this being lossless, only on it being consistent.
	void TestRestoreRoundTrip(bool offsetCoM, const char* label)
	{
		std::printf("TestRestoreRoundTrip [%s]\n", label);

		SimRunner a;
		a.Build(false, false, offsetCoM);
		for (int tick = 0; tick < 90; ++tick)
		{
			a.Tick(tick);
		}

		std::vector<PxU8> first = a.world.Capture();
		a.world.Restore(first);
		std::vector<PxU8> second = a.world.Capture();
		a.world.Restore(second);
		std::vector<PxU8> third = a.world.Capture();

		const bool lossless = first.size() == second.size() &&
			std::memcmp(first.data(), second.data(), first.size()) == 0;
		const bool settles = second.size() == third.size() &&
			std::memcmp(second.data(), third.data(), second.size()) == 0;

		Observe(lossless, "capture/restore is lossless");
		Observe(settles, "capture/restore reaches a fixed point");
		DiffStateBlobs("first -> second", first, second, 2);

		a.Destroy();
	}

	int CountSleeping(TestWorld& w);

	// Framework sleeping, not PhysX's, decides when a body rests.
	//
	// PhysX's own sleep timing does not survive rollback: its wake counter resets to
	// a value that includes a body's counted contact interactions, a number kept from
	// touch transitions against the previous step's state, which a restore does not
	// put back. Measured in the native repro, a replayed tick reproduces its pose from
	// all 24 rewind depths and its wake counter from only 22. So the wake counter is
	// pinned while a body is awake and the framework decides sleeping from a rest
	// counter that is in the snapshot.
	//
	// This first test is the base case: a settled stack falls asleep once its bodies
	// have been below the thresholds for kSleepTicks steps, and not before.
	void TestFrameworkSleepThreshold()
	{
		std::printf("TestFrameworkSleepThreshold\n");

		TestWorld a;
		a.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);

		// Long enough to settle and cross the rest threshold, with margin.
		int firstAsleepTick = -1;
		for (int tick = 0; tick < 400; ++tick)
		{
			PxwWorldStep(a.world, kDt);
			if (firstAsleepTick < 0 && CountSleeping(a) > 0)
			{
				firstAsleepTick = tick;
			}
		}

		std::printf("        first body asleep at tick %d; %d of %zu asleep at the end\n",
			firstAsleepTick, CountSleeping(a), a.dynamicIds.size());
		Check(CountSleeping(a) > 0, "a settled stack falls asleep under framework sleep");
		Check(firstAsleepTick >= static_cast<int>(kSleepTicks),
			"no body sleeps before it has rested for kSleepTicks steps");

		a.Destroy();
	}

	// The sleeping flag and the rest counter both have to survive a capture/restore,
	// or a rolled-back peer would disagree with a live one about who is asleep. Sleep
	// state was absent from the original snapshot format and is a genuine desync
	// source: a body awake on one peer and asleep on another answers the next contact
	// differently.
	void TestFrameworkSleepSurvivesRestore()
	{
		std::printf("TestFrameworkSleepSurvivesRestore\n");

		TestWorld a;
		a.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);
		for (int tick = 0; tick < 400; ++tick)
		{
			PxwWorldStep(a.world, kDt);
		}

		const int sleeping = CountSleeping(a);
		Check(sleeping > 0, "at least one body is asleep before the round trip");

		const PxU64 before = a.Hash();
		std::vector<PxU8> snapshot = a.Capture();

		// Disturb everything: wake the sleepers and fling them, so the restore has real
		// work to undo rather than a no-op.
		for (size_t i = 0; i < a.dynamicIds.size(); ++i)
		{
			PxRigidDynamic* body = static_cast<PxRigidDynamic*>(PxwWorldFindHandle(a.world, a.dynamicIds[i]));
			if (body != NULL)
			{
				body->wakeUp();
				body->setLinearVelocity(PxVec3(5.0f, 5.0f, 5.0f));
			}
		}

		a.Restore(snapshot);
		Check(a.Hash() == before, "sleep flag, rest counter and velocities survive capture/restore");
		Check(CountSleeping(a) == sleeping, "the same bodies are asleep after restore");

		// A restore followed by a capture, with no step between, must be a fixed point:
		// the restore reproduces exactly what was captured.
		std::vector<PxU8> afterRestore = a.Capture();
		Check(afterRestore.size() == snapshot.size() &&
			std::memcmp(afterRestore.data(), snapshot.data(), snapshot.size()) == 0,
			"capture after restore is byte-identical to the snapshot");

		a.Destroy();
	}

	// A sleeping body is out of the solver but still in the broadphase, so it must
	// still collide, and PhysX must wake it, when something lands on it. This is the
	// one part of sleeping the framework does not drive; it relies on PhysX's
	// auto-wake, and this test is where that is exercised directly.
	void TestSleepingBodyWakesOnContact()
	{
		std::printf("TestSleepingBodyWakesOnContact\n");

		// A single box resting on the ground, so the sleeper is exposed rather than
		// buried in a stack. A body landing on it makes a fresh contact, which is what
		// PhysX wakes on.
		TestWorld a;
		a.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks, 1);
		for (int tick = 0; tick < 200; ++tick)
		{
			PxwWorldStep(a.world, kDt);
		}

		const PxU32 restingId = 100u;
		PxRigidDynamic* resting = static_cast<PxRigidDynamic*>(PxwWorldFindHandle(a.world, restingId));
		Check(resting != NULL && resting->isSleeping(), "the resting box is asleep before impact");

		// Drop a heavy sphere onto it.
		a.SpawnDynamicSphere(500u, PxVec3(0.0f, 6.0f, 0.0f), 0.6f, 40.0f);

		bool woke = false;
		for (int tick = 0; tick < 240 && !woke; ++tick)
		{
			PxwWorldStep(a.world, kDt);
			PxRigidDynamic* body = static_cast<PxRigidDynamic*>(PxwWorldFindHandle(a.world, restingId));
			if (body != NULL && !body->isSleeping())
			{
				woke = true;
			}
		}

		Check(woke, "a body landing on a sleeper collides with it and wakes it");

		a.Destroy();
	}

	// The gate. A settling scene is run straight through once for a reference, then
	// again with a fixed-depth rollback on every frame. Because each tick restores its
	// own snapshot before stepping, the two runs step from byte-identical state, so
	// the only thing that can pull them apart is state the snapshot does not carry.
	// Framework sleep is designed so it carries all of it -- velocities and the rest
	// counter -- and this checks that it does, including the PhysX auto-wake path when
	// a body settles, sleeps and is left alone.
	//
	// A failure here is load-bearing: it would mean sleep timing does not replay under
	// rollback, and the honest response is to stop and move to a contact-event layer,
	// not to paper over it.
	void TestFrameworkSleepReplays()
	{
		std::printf("TestFrameworkSleepReplays\n");

		const int warmup = 40;
		const int frames = 200;
		const int depth = 6;
		const int historyDepth = 24;

		// Reference: a straight settling run, no rollback.
		SimRunner ref;
		ref.applyInput = false;
		ref.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);

		std::vector<std::vector<PxU8> > reference;
		reference.push_back(ref.snapshot);
		for (int t = 0; t < warmup + frames; ++t)
		{
			ref.Tick(t);
			reference.push_back(ref.snapshot);
		}

		bool everSlept = false;
		if (CountSleeping(ref.world) > 0)
		{
			everSlept = true;
		}
		ref.Destroy();

		// Peer: the same scene, rewound by a fixed depth and replayed to the present on
		// every frame.
		SimRunner peer;
		peer.applyInput = false;
		peer.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);

		std::vector<std::vector<PxU8> > history(historyDepth);
		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			peer.Tick(tick);
			history[tick % historyDepth] = peer.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;
		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			const int from = tick - depth;
			peer.Rewind(history[from % historyDepth]);
			for (int t = from + 1; t <= tick; ++t)
			{
				peer.Tick(t);
				history[t % historyDepth] = peer.snapshot;
			}

			const std::vector<PxU8>& expected = reference[static_cast<size_t>(tick + 1)];
			if (peer.snapshot.size() != expected.size() ||
				std::memcmp(peer.snapshot.data(), expected.data(), expected.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
				DiffStateBlobs("reference -> replayed", expected, peer.snapshot, 2);
			}
		}

		if (CountSleeping(peer.world) > 0)
		{
			everSlept = true;
		}

		std::printf("        %d frames of fixed depth-%d rollback over a settling scene: %s\n",
			matched ? frames : divergedAt, depth, matched ? "identical" : "diverged");
		Check(everSlept, "the gate scene actually put bodies to sleep");
		Check(matched, "framework sleep replays bit-exactly under fixed-depth rollback");

		peer.Destroy();
	}

	// The case TestFrameworkSleepReplays leaves untested, which Architecture.md flags: a
	// body that is already asleep and is woken by a *new* contact inside the rolled-back
	// window. A settled scene that only ever quietens is the easy direction for sleep to
	// replay; the wake is the hard one, because it is a discrete transition -- a sleeper is
	// out of the solver until the contact re-admits it -- and it has to land on exactly the
	// same tick with exactly the same resulting state on every replay, or a rolled-back peer
	// disagrees with a live one about who is awake.
	//
	// Run under PGS. A wake is a high-energy event, not the quiet settling the sibling test
	// leans on, so only PGS's transparent cold-step replay makes a bit-exact assertion
	// honest; PGS is the framework's chosen solver for exactly this reason (§4).
	void TestSleeperWokenUnderRollback()
	{
		std::printf("TestSleeperWokenUnderRollback\n");

		const PxSolverType::Enum savedSolver = gSolverType;
		gSolverType = PxSolverType::ePGS;

		const int warmup = 45;   // long enough for the stack to settle and sleep
		const int frames = 220;
		const int depth = 6;
		const int historyDepth = 24;
		const PxU32 projectileId = 900u;

		// Reference: a straight cold-step run, with a sphere dropped from above that lands
		// on the sleeping stack partway through. Registered before the first capture so the
		// snapshot layout is fixed for the whole run.
		SimRunner ref;
		ref.applyInput = false;
		ref.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);
		ref.world.SpawnDynamicSphere(projectileId, PxVec3(0.0f, 15.0f, 0.0f), 0.6f, 60.0f);
		ref.snapshot = ref.world.Capture();

		std::vector<std::vector<PxU8> > reference;
		reference.push_back(ref.snapshot);

		int sleepPeak = 0;
		int wokeAt = -1;
		for (int t = 0; t < warmup + frames; ++t)
		{
			ref.Tick(t);
			reference.push_back(ref.snapshot);
			const int sleeping = CountSleeping(ref.world);
			if (sleeping > sleepPeak)
			{
				sleepPeak = sleeping;
			}
			// The first time the count drops back below its peak is the impact waking a
			// sleeper. The falling sphere is never asleep, so it does not inflate the count.
			if (wokeAt < 0 && sleepPeak > 0 && sleeping < sleepPeak)
			{
				wokeAt = t;
			}
		}
		ref.Destroy();

		Check(sleepPeak > 0, "the stack fell asleep before the impact");
		Check(wokeAt >= 0, "the falling body woke a sleeper in the reference run");

		// Peer: the same scene, rewound by a fixed depth and replayed to the present every
		// frame, so the wake transition sits inside the replayed window repeatedly.
		SimRunner peer;
		peer.applyInput = false;
		peer.Build(false, true, false, kSleepLinear, kSleepAngular, kSleepTicks);
		peer.world.SpawnDynamicSphere(projectileId, PxVec3(0.0f, 15.0f, 0.0f), 0.6f, 60.0f);
		peer.snapshot = peer.world.Capture();

		std::vector<std::vector<PxU8> > history(historyDepth);
		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			peer.Tick(tick);
			history[tick % historyDepth] = peer.snapshot;
		}

		bool preWakeExact = true;   // every replayed tick strictly before the wake
		bool wakeExact = true;      // the whole run including the wake and after
		int firstDivergeTick = -1;
		for (int frame = 0; frame < frames; ++frame, ++tick)
		{
			const int from = tick - depth;
			peer.Rewind(history[from % historyDepth]);
			for (int t = from + 1; t <= tick; ++t)
			{
				peer.Tick(t);
				history[t % historyDepth] = peer.snapshot;
			}

			const std::vector<PxU8>& expected = reference[static_cast<size_t>(tick + 1)];
			const bool same = peer.snapshot.size() == expected.size() &&
				std::memcmp(peer.snapshot.data(), expected.data(), expected.size()) == 0;
			if (!same)
			{
				wakeExact = false;
				if (firstDivergeTick < 0)
				{
					firstDivergeTick = tick + 1;
					DiffStateBlobs("reference -> replayed at the wake", expected, peer.snapshot, 3);
				}
				// A tick comfortably before the wake that diverges would be the settling
				// replay itself breaking, which is the guaranteed part. The two-tick margin
				// keeps the boundary between "settling" and "wake" from turning an
				// off-by-one in wake detection into a flaky failure.
				if (tick + 1 <= wokeAt - 2)
				{
					preWakeExact = false;
				}
			}
		}

		std::printf("        depth-%d rollback across a wake at tick %d: first divergence at tick %d\n",
			depth, wokeAt, firstDivergeTick);

		// The load-bearing guarantee: the settling that happens *before* the sleeper is
		// woken replays bit-exactly, so the sleep flag and rest counter carried in the
		// snapshot do their job under rollback.
		Check(preWakeExact, "the settling before the wake replays bit-exactly under rollback");

		// The wake transition itself is a characterisation, not a guarantee. Waking a
		// sleeper builds a fresh contact whose solver warm-start the snapshot deliberately
		// does not carry (the same state that makes a contact's point and impulse only
		// approximate across a cold restore, Architecture.md §5). So a wake that lands
		// inside a rolled-back window is *not* bit-exact, and it is measured here rather
		// than asserted, with the first divergence lining up with the wake tick to show it
		// is the wake and not the settling that moves. Gameplay must treat a rollback-
		// spanning wake like a contact impulse: it may branch on the fact that a body woke,
		// never on the exact tick or resulting velocity.
		Observe(wakeExact, "a sleeper woken by a new contact replays bit-exactly under rollback");
		Observe(firstDivergeTick < 0 || firstDivergeTick >= wokeAt,
			"any divergence begins at the wake, not before it");

		peer.Destroy();
		gSolverType = savedSolver;
	}


	int CountSleeping(TestWorld& w)
	{
		int sleeping = 0;
		for (size_t i = 0; i < w.dynamicIds.size(); ++i)
		{
			PxRigidDynamic* body = static_cast<PxRigidDynamic*>(PxwWorldFindHandle(w.world, w.dynamicIds[i]));
			if (body != NULL && body->isSleeping())
			{
				++sleeping;
			}
		}
		return sleeping;
	}

	// Sweeps rewind depth to find out whether exactness depends on how far the live
	// scene has run past the snapshot being restored, and on which tick is replayed
	// first. Divergence here is not a float-precision effect: it shows up whenever a
	// replayed tick creates or loses contacts that the leftover pair bookkeeping
	// disagrees about.
	void TestRewindDepthSweep(PxU32 resetMode, const char* label)
	{
		std::printf("TestRewindDepthSweep [%s]\n", label);

		const int warmup = 60;
		const int horizon = 48;
		const int depths[] = { 1, 2, 4, 8, 16, 32, 48 };
		int exactCount = 0;

		for (size_t d = 0; d < sizeof(depths) / sizeof(depths[0]); ++d)
		{
			const int depth = depths[d];

			SimRunner a;
			a.resetMode = resetMode;
			a.Build(false);
			for (int tick = 0; tick < warmup; ++tick)
			{
				a.Tick(tick);
			}

			std::vector<std::vector<PxU8> > reference;
			reference.push_back(a.snapshot);
			for (int i = 0; i < horizon; ++i)
			{
				a.Tick(warmup + i);
				reference.push_back(a.snapshot);
			}

			const int rewindTo = horizon - depth;

			a.Rewind(reference[static_cast<size_t>(rewindTo)]);
			bool matched = true;
			int divergedAt = -1;
			for (int i = rewindTo; i < horizon; ++i)
			{
				a.Tick(warmup + i);
				const std::vector<PxU8>& expected = reference[static_cast<size_t>(i + 1)];
				if (a.snapshot.size() != expected.size() ||
					std::memcmp(a.snapshot.data(), expected.data(), expected.size()) != 0)
				{
					matched = false;
					divergedAt = i - rewindTo;
					break;
				}
			}

			if (matched)
			{
				++exactCount;
			}
			std::printf("        depth %2d: %s%s\n", depth,
				matched ? "exact" : "diverged at replay step ",
				matched ? "" : std::to_string(divergedAt).c_str());

			a.Destroy();
		}

		Observe(exactCount == static_cast<int>(sizeof(depths) / sizeof(depths[0])),
			std::string("every rewind depth replays bit-exactly [") + label + "]");
	}

	// Largest per-body position and linear-velocity difference between two state blobs.
	void MaxStateError(const std::vector<PxU8>& lhs, const std::vector<PxU8>& rhs,
		float& outMaxPosition, float& outMaxVelocity)
	{
		outMaxPosition = 0.0f;
		outMaxVelocity = 0.0f;
		if (lhs.size() != rhs.size() || lhs.size() < 16)
		{
			return;
		}

		struct BlobHeader { PxU32 magic, version, entryCount, totalBytes; };
		struct BlobEntry { PxU32 stableId, kind, payloadBytes, reserved; };
		struct BlobRigid
		{
			PxVec3 position; PxQuat rotation;
			PxVec3 linearVelocity; PxVec3 angularVelocity;
			PxReal wakeCounter; PxU32 flags;
		};

		const BlobHeader* header = reinterpret_cast<const BlobHeader*>(lhs.data());
		size_t offset = sizeof(BlobHeader);

		for (PxU32 i = 0; i < header->entryCount && offset + sizeof(BlobEntry) <= lhs.size(); ++i)
		{
			const BlobEntry* entry = reinterpret_cast<const BlobEntry*>(lhs.data() + offset);
			const size_t payloadOffset = offset + sizeof(BlobEntry);
			if (entry->kind != PxwHandleKind::eARTICULATION && payloadOffset + sizeof(BlobRigid) <= lhs.size())
			{
				const BlobRigid* a = reinterpret_cast<const BlobRigid*>(lhs.data() + payloadOffset);
				const BlobRigid* b = reinterpret_cast<const BlobRigid*>(rhs.data() + payloadOffset);
				const float dp = (a->position - b->position).magnitude();
				const float dv = (a->linearVelocity - b->linearVelocity).magnitude();
				if (dp > outMaxPosition) outMaxPosition = dp;
				if (dv > outMaxVelocity) outMaxVelocity = dv;
			}
			offset = payloadOffset + entry->payloadBytes;
		}
	}

	// Quantifies how far a replayed trace drifts from the original. Bit-exact replay is
	// not achievable, so what matters is whether the error stays down at the level of
	// float noise or grows into something a player would see.
	//
	// positionBudget is checked at the rollback depth the framework actually uses; a
	// negative budget means the run is characterisation only.
	void TestReplayErrorMagnitude(PxU32 resetMode, const char* label,
		float positionBudget = -1.0f, float velocityBudget = -1.0f, int budgetAtTick = 30)
	{
		std::printf("TestReplayErrorMagnitude [%s]\n", label);

		const int warmup = 60;
		const int horizon = 120;

		SimRunner a;
		a.resetMode = resetMode;
		a.Build(false);
		for (int tick = 0; tick < warmup; ++tick)
		{
			a.Tick(tick);
		}

		const std::vector<PxU8> rewindPoint = a.snapshot;
		std::vector<std::vector<PxU8> > trace;
		for (int i = 0; i < horizon; ++i)
		{
			a.Tick(warmup + i);
			trace.push_back(a.snapshot);
		}

		a.Rewind(rewindPoint);
		float worstPosition = 0.0f;
		float worstVelocity = 0.0f;
		float budgetPosition = 0.0f;
		float budgetVelocity = 0.0f;
		for (int i = 0; i < horizon; ++i)
		{
			a.Tick(warmup + i);
			float dp, dv;
			MaxStateError(trace[static_cast<size_t>(i)], a.snapshot, dp, dv);
			if (dp > worstPosition) worstPosition = dp;
			if (dv > worstVelocity) worstVelocity = dv;
			if (i < budgetAtTick)
			{
				budgetPosition = worstPosition;
				budgetVelocity = worstVelocity;
			}

			if (i == 9 || i == 29 || i == 59 || i == horizon - 1)
			{
				std::printf("        after %3d replayed ticks: max position error %.3e m, max velocity error %.3e m/s\n",
					i + 1, dp, dv);
			}
		}

		std::printf("        worst over the whole replay: %.3e m, %.3e m/s\n", worstPosition, worstVelocity);

		if (positionBudget > 0.0f)
		{
			Check(budgetPosition <= positionBudget && budgetVelocity <= velocityBudget,
				std::string("replay error stays within budget over ") + std::to_string(budgetAtTick) + " ticks [" + label + "]");
		}

		a.Destroy();
	}

	// The decisive experiment.
	//
	// Replays the reference ticks into a world that was just built, so the scene has
	// no history at all. This separates the two possible explanations for a divergent
	// replay:
	//
	//   * If the fresh world reproduces the reference exactly, the snapshot captures
	//     everything that matters and any divergence in a reused scene comes from
	//     leftover state that a reset has to clear.
	//
	//   * If the fresh world also diverges, then the snapshot is missing simulation
	//     state that PhysX does not expose, and bit-exact rollback in a reused scene
	//     is not achievable no matter how the scene is reset.
	// The question that decides whether a single-world rollback can hash bit-exactly.
	//
	// A peer that rewinds and one that does not carry different hidden state into the
	// same tick, so their results differ. But if the step is preceded by a reset that
	// erases that hidden state, the step becomes a pure function of (snapshot, inputs),
	// and every peer computing it must agree bit for bit regardless of how it got
	// there. That trades a little fidelity against the uninterrupted trajectory for
	// something far more valuable in netcode: peers that agree exactly.
	//
	// A world with no history at all is the strongest form of "different history", so
	// if a fresh world can reproduce a used world's trace under a given reset mode,
	// any two peers can.
	void TestFreshWorldReplay(PxU32 resetMode, const char* label)
	{
		std::printf("TestFreshWorldReplay [%s]\n", label);

		const int warmup = 60;
		const int horizon = 16;

		SimRunner reference;
		reference.resetMode = resetMode;
		reference.restoreEachTick = true;
		reference.Build(false);
		for (int tick = 0; tick < warmup; ++tick)
		{
			reference.Tick(tick);
		}

		const std::vector<PxU8> rewindPoint = reference.snapshot;
		std::vector<std::vector<PxU8> > trace;
		for (int i = 0; i < horizon; ++i)
		{
			reference.Tick(warmup + i);
			trace.push_back(reference.snapshot);
		}
		reference.Destroy();

		SimRunner fresh;
		fresh.resetMode = resetMode;
		fresh.restoreEachTick = true;
		fresh.Build(false);
		fresh.snapshot = rewindPoint;

		bool matched = true;
		int divergedAt = -1;
		for (int i = 0; i < horizon; ++i)
		{
			fresh.Tick(warmup + i);
			const std::vector<PxU8>& expected = trace[static_cast<size_t>(i)];
			if (fresh.snapshot.size() != expected.size() ||
				std::memcmp(fresh.snapshot.data(), expected.data(), expected.size()) != 0)
			{
				matched = false;
				divergedAt = i;
				DiffStateBlobs("reference -> fresh world", expected, fresh.snapshot, 2);
				break;
			}
		}

		if (!matched)
		{
			std::printf("        diverged at replay step %d\n", divergedAt);
		}
		std::printf("        replayed %d of %d ticks bit-exactly\n",
			matched ? horizon : divergedAt, horizon);
		Observe(matched, "a world with no history replays the trace exactly [" + std::string(label) + "]");

		fresh.Destroy();
	}

	// A fresh world cannot reproduce a used world's trace, which rules out a late
	// joiner ever matching the peers already running. But that is not the only way to
	// get a joiner in sync: instead of making the joiner match everyone else, every
	// peer can rebuild its world from one agreed snapshot at an agreed tick. All peers
	// then have identical history again, from a common starting point.
	//
	// That only works if two worlds built from scratch and restored from the same
	// snapshot agree with each other bit for bit. This measures exactly that, and it is
	// the hinge the whole single-world design turns on: if it holds, bit-exact hashing
	// and mid-match join can coexist behind a synchronised rebuild.
	void TestRebuiltWorldsAgree(PxU32 resetMode, const char* label)
	{
		std::printf("TestRebuiltWorldsAgree [%s]\n", label);

		const int warmup = 60;
		const int horizon = 32;

		// Produce a snapshot from a world with arbitrary history, standing in for the
		// state a running match would hand to a joiner.
		SimRunner source;
		source.resetMode = resetMode;
		source.restoreEachTick = true;
		source.Build(false);
		for (int tick = 0; tick < warmup; ++tick)
		{
			source.Tick(tick);
		}
		const std::vector<PxU8> handover = source.snapshot;
		source.Destroy();

		// Two peers, both rebuilding from that snapshot. Neither shares the source's
		// history, but they share each other's.
		SimRunner peerA, peerB;
		peerA.resetMode = resetMode;
		peerB.resetMode = resetMode;
		peerA.restoreEachTick = true;
		peerB.restoreEachTick = true;
		peerA.Build(false);
		peerB.Build(true);   // reversed registration order, to also prove ordering holds
		peerA.snapshot = handover;
		peerB.snapshot = handover;

		bool matched = true;
		int divergedAt = -1;
		for (int i = 0; i < horizon; ++i)
		{
			peerA.Tick(warmup + i);
			peerB.Tick(warmup + i);

			if (peerA.snapshot.size() != peerB.snapshot.size() ||
				std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = i;
				DiffStateBlobs("peer A -> peer B", peerA.snapshot, peerB.snapshot, 2);
				break;
			}
		}

		std::printf("        agreed for %d of %d ticks\n", matched ? horizon : divergedAt, horizon);
		Check(matched, "two worlds rebuilt from the same snapshot stay bit-identical ["
			+ std::string(label) + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	// The rebuild above works because both peers started from a world with no history.
	// The tempting shortcut is to skip the world recreation and simply restore the
	// agreed snapshot into the world each peer already has, which is much cheaper and
	// looks equivalent: the snapshot fully determines pose, velocity and sleep state.
	//
	// It is not equivalent, because restore is a partial reset. Whatever PhysX carries
	// between steps and does not expose survives it, and each peer's carried state comes
	// from its own history. This measures the shortcut so the cost of skipping the
	// recreation is on the record rather than assumed.
	void TestRestoreIntoUsedWorldsDisagrees(PxU32 resetMode, const char* label)
	{
		std::printf("TestRestoreIntoUsedWorldsDisagrees [%s]\n", label);

		const int horizon = 32;

		SimRunner source;
		source.resetMode = resetMode;
		source.restoreEachTick = true;
		source.Build(false);
		for (int tick = 0; tick < 60; ++tick)
		{
			source.Tick(tick);
		}
		const std::vector<PxU8> handover = source.snapshot;
		source.Destroy();

		// Two peers that have been running, and that have run for different lengths of
		// time, which is the realistic case: they joined at different moments.
		SimRunner peerA, peerB;
		peerA.resetMode = resetMode;
		peerB.resetMode = resetMode;
		peerA.restoreEachTick = true;
		peerB.restoreEachTick = true;
		peerA.Build(false);
		peerB.Build(false);
		for (int tick = 0; tick < 40; ++tick)
		{
			peerA.Tick(tick);
		}
		for (int tick = 0; tick < 90; ++tick)
		{
			peerB.Tick(tick);
		}

		// The shortcut: hand both peers the same snapshot without recreating the world.
		peerA.snapshot = handover;
		peerB.snapshot = handover;

		bool matched = true;
		int divergedAt = -1;
		for (int i = 0; i < horizon; ++i)
		{
			peerA.Tick(60 + i);
			peerB.Tick(60 + i);

			if (peerA.snapshot.size() != peerB.snapshot.size() ||
				std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = i;
				DiffStateBlobs("peer A -> peer B", peerA.snapshot, peerB.snapshot, 1);
				break;
			}
		}

		std::printf("        agreed for %d of %d ticks\n", matched ? horizon : divergedAt, horizon);
		Observe(matched, "restoring into existing worlds is as good as recreating them ["
			+ std::string(label) + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	// The decisive experiment for the rollback design.
	//
	// The depth sweep above compares a replay against the original trace, and that is
	// not the question netcode asks. No peer ever compares itself against a hypothetical
	// un-rewound version of itself. What matters is whether two peers agree with each
	// other, and in a conventional rollback engine they rewind by whatever their own
	// late inputs happened to demand, which differs.
	//
	// So: two peers running identically, which then roll back by different depths and
	// replay to the same tick. If they agree, rollback depth need not be synchronised
	// and the prediction horizon can adapt to each peer's latency. If they disagree, the
	// horizon has to be fixed so that every peer rewinds by the same amount.
	void TestVariableRewindDepthAgreement(PxU32 resetMode, const char* label)
	{
		std::printf("TestVariableRewindDepthAgreement [%s]\n", label);

		const int warmup = 60;
		const int depthA = 4;
		const int depthB = 16;

		SimRunner peerA, peerB;
		peerA.resetMode = resetMode;
		peerB.resetMode = resetMode;
		peerA.restoreEachTick = true;
		peerB.restoreEachTick = true;
		peerA.Build(false);
		peerB.Build(false);

		std::vector<std::vector<PxU8> > historyA;
		std::vector<std::vector<PxU8> > historyB;
		for (int tick = 0; tick < warmup; ++tick)
		{
			peerA.Tick(tick);
			peerB.Tick(tick);
			historyA.push_back(peerA.snapshot);
			historyB.push_back(peerB.snapshot);
		}

		// They must be identical before the rollback, or the test proves nothing.
		const bool alignedBefore = historyA[warmup - 1].size() == historyB[warmup - 1].size() &&
			std::memcmp(historyA[warmup - 1].data(), historyB[warmup - 1].data(), historyA[warmup - 1].size()) == 0;
		Check(alignedBefore, "the two peers are bit-identical before rolling back");

		// Peer A rewinds a little, peer B rewinds a lot, both replay to the same tick.
		peerA.Rewind(historyA[warmup - 1 - depthA]);
		for (int tick = warmup - depthA; tick < warmup; ++tick)
		{
			peerA.Tick(tick);
		}

		peerB.Rewind(historyB[warmup - 1 - depthB]);
		for (int tick = warmup - depthB; tick < warmup; ++tick)
		{
			peerB.Tick(tick);
		}

		const bool matched = peerA.snapshot.size() == peerB.snapshot.size() &&
			std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) == 0;

		if (!matched)
		{
			DiffStateBlobs("peer A -> peer B", peerA.snapshot, peerB.snapshot, 1);
		}
		std::printf("        rewind %d vs rewind %d, replayed to the same tick: %s\n",
			depthA, depthB, matched ? "identical" : "different");
		Observe(matched, "peers that rewound by different depths agree [" + std::string(label) + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	// A single divergent rollback agreeing proves little. The realistic case is peers
	// rewinding by different amounts every frame for minutes, where any per-rollback
	// error would compound. This drives two peers through sustained, differing rollback
	// for a long run and checks they never part company.
	void TestSustainedDivergentRollback(PxU32 resetMode, const char* label, int frames)
	{
		std::printf("TestSustainedDivergentRollback [%s]\n", label);

		const int warmup = 30;
		const int historyDepth = 24;

		SimRunner peerA, peerB;
		peerA.resetMode = resetMode;
		peerB.resetMode = resetMode;
		peerA.restoreEachTick = true;
		peerB.restoreEachTick = true;
		peerA.Build(false);
		peerB.Build(true);   // opposite registration order, for good measure

		std::vector<std::vector<PxU8> > historyA(historyDepth);
		std::vector<std::vector<PxU8> > historyB(historyDepth);

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			peerA.Tick(tick);
			peerB.Tick(tick);
			historyA[tick % historyDepth] = peerA.snapshot;
			historyB[tick % historyDepth] = peerB.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;

		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			// Different, varying rollback depths, as two peers with different latency
			// and different jitter would produce.
			const int depthA = 1 + (frame * 3) % 7;
			const int depthB = 1 + (frame * 5) % 17;

			const int fromA = tick - depthA;
			const int fromB = tick - depthB;

			peerA.Rewind(historyA[fromA % historyDepth]);
			for (int t = fromA + 1; t <= tick; ++t)
			{
				peerA.Tick(t);
				historyA[t % historyDepth] = peerA.snapshot;
			}

			peerB.Rewind(historyB[fromB % historyDepth]);
			for (int t = fromB + 1; t <= tick; ++t)
			{
				peerB.Tick(t);
				historyB[t % historyDepth] = peerB.snapshot;
			}

			if (peerA.snapshot.size() != peerB.snapshot.size() ||
				std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
				DiffStateBlobs("peer A -> peer B", peerA.snapshot, peerB.snapshot, 1);
			}
		}

		std::printf("        %d frames of differing rollback depth: %s\n",
			matched ? frames : divergedAt, matched ? "still identical" : "diverged");
		Observe(matched, "peers stay identical under sustained divergent rollback ["
			+ std::string(label) + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	// The wrapper registers entities in ascending stable-ID order regardless of the
	// order the application asked for them, precisely so that PhysX assigns the same
	// internal indices on every peer. This checks that the mechanism works, because if
	// it silently stopped working the symptom would be a slow desync with no obvious
	// cause -- the standalone repro measures a reverse-ordered scene diverging within a
	// few hundred steps.
	void TestInternalIdsMatchAcrossRegistrationOrder()
	{
		std::printf("TestInternalIdsMatchAcrossRegistrationOrder\n");

		TestWorld forward, reverse;
		forward.Build(false);
		reverse.Build(true);

		// Indices are handed out on insertion into the scene, and settle after a step.
		PxwWorldStep(forward.world, kDt);
		PxwWorldStep(reverse.world, kDt);

		const PxU64 hashForward = PxwWorldHashInternalIds(forward.world);
		const PxU64 hashReverse = PxwWorldHashInternalIds(reverse.world);

		std::vector<PxwInternalIdEntry> a(64), b(64);
		const PxU32 countA = PxwWorldReadInternalIds(forward.world, a.data(), static_cast<PxU32>(a.size()));
		const PxU32 countB = PxwWorldReadInternalIds(reverse.world, b.data(), static_cast<PxU32>(b.size()));

		Check(countA == countB && countA > 0, "both worlds report the same number of bodies");

		bool allMatch = countA == countB;
		for (PxU32 i = 0; i < countA && i < countB; ++i)
		{
			if (a[i].stableId != b[i].stableId ||
				a[i].internalActorIndex != b[i].internalActorIndex ||
				a[i].islandNodeIndex != b[i].islandNodeIndex)
			{
				allMatch = false;
				std::printf("        stable id %u: actor %u vs %u, island %llu vs %llu\n",
					a[i].stableId, a[i].internalActorIndex, b[i].internalActorIndex,
					static_cast<unsigned long long>(a[i].islandNodeIndex),
					static_cast<unsigned long long>(b[i].islandNodeIndex));
			}
		}

		std::printf("        %u bodies, id-map hash 0x%016llX vs 0x%016llX\n",
			countA,
			static_cast<unsigned long long>(hashForward),
			static_cast<unsigned long long>(hashReverse));

		Check(allMatch, "the same stable id gets the same PhysX indices whatever the registration order");
		Check(hashForward == hashReverse, "the id-map hash agrees across registration orders");

		forward.Destroy();
		reverse.Destroy();
	}

	// ---------------------------------------------------------------------------
	// Deterministic mass properties.
	//
	// PhysX stores a diagonal inertia tensor plus a mass frame, so any body whose
	// inertia is not axis aligned gets an eigenvector rotation baked into its
	// centre-of-mass pose. For a body that is inertially close to a sphere those
	// eigenvectors barely exist, and a last-bit difference in the shape layout can
	// swing them noticeably. Two peers that each compute their own mass properties can
	// therefore end up simulating measurably different bodies.

	struct MassTestShape
	{
		PxVec3 halfExtents;
		PxTransform pose;
	};

	// The problem case: a ball with spikes, inertially almost spherical.
	std::vector<MassTestShape> MakeSpikedBallShapes(int spikeCount, float jitter)
	{
		std::vector<MassTestShape> shapes;
		MassTestShape core;
		core.halfExtents = PxVec3(0.5f, 0.5f, 0.5f);
		core.pose = PxTransform(PxIdentity);
		shapes.push_back(core);

		for (int i = 0; i < spikeCount; ++i)
		{
			const float k = (float(i) + 0.5f) / float(spikeCount);
			const float phi = PxAcos(1.0f - 2.0f * k);
			const float theta = 3.883222f * float(i);
			const PxVec3 dir(PxSin(phi) * PxCos(theta), PxSin(phi) * PxSin(theta), PxCos(phi));

			MassTestShape spike;
			spike.halfExtents = PxVec3(0.25f, 0.06f, 0.06f);
			spike.pose = PxTransform(dir * (0.75f + jitter * float(i)),
				PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), dir).getNormalized());
			shapes.push_back(spike);
		}
		return shapes;
	}

	PxRigidDynamic* MakeMassTestBody(const std::vector<MassTestShape>& shapes, bool reverseAttachOrder)
	{
		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);
		PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));

		for (size_t i = 0; i < shapes.size(); ++i)
		{
			const MassTestShape& s = reverseAttachOrder ? shapes[shapes.size() - 1 - i] : shapes[i];
			PxShape* shape = physics->createShape(PxBoxGeometry(s.halfExtents), *material, true);
			shape->setLocalPose(s.pose);
			body->attachShape(*shape);
			shape->release();
		}

		material->release();
		return body;
	}

	double MassFrameAngle(const PxwMassProperties& a, const PxwMassProperties& b)
	{
		const PxQuat qa = a.cMassLocalPose.ToPxTransform().q;
		const PxQuat qb = b.cMassLocalPose.ToPxTransform().q;
		const PxQuat d = qa.getConjugate() * qb;
		const double axis = PxSqrt(double(d.x) * d.x + double(d.y) * d.y + double(d.z) * d.z);
		return 2.0 * std::atan2(axis, std::fabs(double(d.w)));
	}

	void PrintMass(const char* label, const PxwMassProperties& m)
	{
		const PxTransform com = m.cMassLocalPose.ToPxTransform();
		std::printf("        %-22s mass %.7g  inertia (%.7g %.7g %.7g)  anisotropy %.4f%%%s\n",
			label, m.mass, m.inertia.x, m.inertia.y, m.inertia.z,
			double(m.anisotropy) * 100.0, m.massFrameCollapsed ? "  [frame collapsed]" : "");
		std::printf("        %-22s com p (%.7g %.7g %.7g) q (%.7g %.7g %.7g %.7g)\n",
			"", com.p.x, com.p.y, com.p.z, com.q.x, com.q.y, com.q.z, com.q.w);
	}

	// Every peer must land on the same numbers when handed the same body.
	void TestMassIsReproducible(int spikeCount, const char* label)
	{
		std::printf("TestMassIsReproducible [%s]\n", label);

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(spikeCount, 0.0f);

		PxRigidDynamic* first = MakeMassTestBody(shapes, false);
		PxRigidDynamic* second = MakeMassTestBody(shapes, false);

		PxwMassProperties a, b;
		Check(PxwComputeMassProperties(first, 10.0f, -1.0f, false, &a) == PxwResult::eOK,
			"mass computation succeeded");
		Check(PxwComputeMassProperties(second, 10.0f, -1.0f, false, &b) == PxwResult::eOK,
			"mass computation succeeded for the second body");

		PrintMass("computed", a);
		Check(PxwHashMassProperties(&a) == PxwHashMassProperties(&b),
			"two separately built copies of the same body hash identically");

		// And recomputing on the same body must not wander.
		PxwMassProperties again;
		PxwComputeMassProperties(first, 10.0f, -1.0f, false, &again);
		Check(PxwHashMassProperties(&a) == PxwHashMassProperties(&again),
			"recomputing on the same body gives the same answer");

		first->release();
		second->release();
	}

	// Floating point addition is not associative, so summing the same shapes in a
	// different order rounds differently. Attachment order depends on how a prefab was
	// authored and how a loader walks it, which is not something a peer should have to
	// match bit for bit, so the summation is sorted into a canonical order first.
	void TestMassIgnoresAttachOrder(int spikeCount, const char* label)
	{
		std::printf("TestMassIgnoresAttachOrder [%s]\n", label);

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(spikeCount, 0.0f);

		PxRigidDynamic* forward = MakeMassTestBody(shapes, false);
		PxRigidDynamic* reversed = MakeMassTestBody(shapes, true);

		PxwMassProperties a, b;
		PxwComputeMassProperties(forward, 10.0f, -1.0f, false, &a);
		PxwComputeMassProperties(reversed, 10.0f, -1.0f, false, &b);

		const bool sameHash = PxwHashMassProperties(&a) == PxwHashMassProperties(&b);
		std::printf("        reversing attachment order changes the mass frame by %.3e rad,"
			" centre of mass by %.3e m\n",
			MassFrameAngle(a, b),
			(a.cMassLocalPose.ToPxTransform().p - b.cMassLocalPose.ToPxTransform().p).magnitude());
		Check(sameHash, "reversing attachment order does not change the mass properties");

		forward->release();
		reversed->release();
	}

	// The headline result: collapsing a near-isotropic mass frame removes the
	// amplification that makes the spiked ball fragile.
	void TestIsotropyCollapseStabilisesMassFrame(int spikeCount, const char* label)
	{
		std::printf("TestIsotropyCollapseStabilisesMassFrame [%s]\n", label);

		PxRigidDynamic* reference = MakeMassTestBody(MakeSpikedBallShapes(spikeCount, 0.0f), false);
		PxRigidDynamic* perturbed = MakeMassTestBody(MakeSpikedBallShapes(spikeCount, 1e-6f), false);

		// Exact principal axes, the behaviour PxRigidBodyExt::updateMassAndInertia has.
		PxwMassProperties exactA, exactB;
		PxwComputeMassProperties(reference, 10.0f, 0.0f, false, &exactA);
		PxwComputeMassProperties(perturbed, 10.0f, 0.0f, false, &exactB);
		const double exactSwing = MassFrameAngle(exactA, exactB);

		// Collapsed, the default.
		PxwMassProperties collapsedA, collapsedB;
		PxwComputeMassProperties(reference, 10.0f, -1.0f, false, &collapsedA);
		PxwComputeMassProperties(perturbed, 10.0f, -1.0f, false, &collapsedB);
		const double collapsedSwing = MassFrameAngle(collapsedA, collapsedB);

		PrintMass("exact axes", exactA);
		PrintMass("collapsed", collapsedA);
		std::printf("        a 1e-6 m layout change swings the mass frame by %.3e rad exact,"
			" %.3e rad collapsed\n", exactSwing, collapsedSwing);

		Check(collapsedA.massFrameCollapsed == 1,
			"a near-isotropic body collapses its mass frame to the identity");
		Check(collapsedSwing <= exactSwing,
			"collapsing the mass frame does not make it more sensitive to input noise");
		Check(collapsedSwing == 0.0,
			"a collapsed mass frame is completely insensitive to a last-bit layout change");

		reference->release();
		perturbed->release();
	}

	// Collapsing the mass frame orientation was not the whole story. A near-spherical
	// compound also has a centre of mass that is a sum of per-shape contributions, and
	// that sum lands a last bit apart on a peer whose floating point rounds differently.
	// Even with the frame collapsed and the moments meaned, that residual centre-of-mass
	// difference desyncs the body the moment it shares a solver island. The collapse
	// path therefore snaps a near-origin centre of mass to the actor origin; this pins
	// that it fires and that it makes two last-bit-different builds hash identically.
	void TestIsotropyCollapseCanonicalisesCentreOfMass(int spikeCount, const char* label)
	{
		std::printf("TestIsotropyCollapseCanonicalisesCentreOfMass [%s]\n", label);

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(spikeCount, 0.0f);
		PxRigidDynamic* forward = MakeMassTestBody(shapes, false);
		PxRigidDynamic* reversed = MakeMassTestBody(shapes, true);

		// The exact path (isotropyTolerance 0) keeps the summed centre of mass, which for
		// a Fibonacci spike layout is a small but non-zero offset -- the last-bit-fragile
		// quantity the collapse has to erase.
		PxwMassProperties exact;
		PxwComputeMassProperties(forward, 10.0f, 0.0f, false, &exact);
		const PxVec3 exactCom = exact.cMassLocalPose.ToPxTransform().p;

		// The default path collapses the frame and snaps the near-origin COM to exactly
		// the actor origin.
		PxwMassProperties a, b;
		PxwComputeMassProperties(forward, 10.0f, -1.0f, false, &a);
		PxwComputeMassProperties(reversed, 10.0f, -1.0f, false, &b);
		const PxVec3 comA = a.cMassLocalPose.ToPxTransform().p;

		std::printf("        exact COM %.3e m, collapsed COM %.3e m\n",
			exactCom.magnitude(), comA.magnitude());

		Check(exactCom.magnitude() > 0.0f,
			"the raw centre of mass is a non-zero, layout-dependent offset");
		Check(a.massFrameCollapsed == 1 && b.massFrameCollapsed == 1,
			"a near-isotropic body collapses its mass frame");
		Check(comA == PxVec3(0.0f),
			"a collapsed near-origin centre of mass is snapped to the actor origin");
		Check(PxwHashMassProperties(&a) == PxwHashMassProperties(&b),
			"the snapped mass properties are identical whatever the shape summation order");

		forward->release();
		reversed->release();
	}

	// The snap is deliberately narrow: a body that is inertially near-spherical but has
	// a genuinely off-centre mass -- a weighted die, a hammer head on a light handle --
	// must keep its real centre of mass, or the fix would quietly change how such bodies
	// behave. A single dense box offset well beyond the radius-of-gyration threshold
	// stands in for that case.
	void TestOffCentreMassIsPreserved()
	{
		std::printf("TestOffCentreMassIsPreserved\n");

		// A near-spherical shell of spikes (so the frame is near-isotropic and collapses)
		// plus one box pushed far to the side, giving a real, well-off-origin COM.
		std::vector<MassTestShape> shapes = MakeSpikedBallShapes(24, 0.0f);
		MassTestShape offset;
		offset.halfExtents = PxVec3(0.3f, 0.3f, 0.3f);
		offset.pose = PxTransform(PxVec3(2.0f, 0.0f, 0.0f));
		shapes.push_back(offset);

		PxRigidDynamic* body = MakeMassTestBody(shapes, false);
		PxwMassProperties m;
		PxwComputeMassProperties(body, 10.0f, -1.0f, false, &m);
		const PxVec3 com = m.cMassLocalPose.ToPxTransform().p;

		std::printf("        centre of mass %.4f m from origin\n", com.magnitude());
		Check(com.magnitude() > 0.01f,
			"a genuinely off-centre mass keeps its centre of mass and is not snapped to origin");

		body->release();
	}

	// ---------------------------------------------------------------------------
	// Construction hashing.
	//
	// A snapshot describes a body's state. It says nothing about how the body was
	// built, and neither does any other checksum a session compares: not the state
	// hash, not the per-entry hashes, not the internal-id hash. Yet every solve reads
	// the construction -- the shapes, their local poses and offsets, the materials, the
	// depenetration clamp, the iteration counts -- so two peers that build the same
	// entity even slightly differently agree on every number they exchange and still
	// diverge as soon as the body is loaded hard enough for the difference to matter.
	//
	// That is a nasty failure to diagnose, because the delay between cause and symptom
	// is unbounded: the same one-ULP shape offset that is completely invisible while a
	// ball rolls around on the floor desyncs it within a couple of seconds once it is
	// squeezed between two other bodies. PxwWorldHashConstruction closes the gap by
	// making the construction comparable, so the mismatch is reported at the body that
	// differs rather than inferred from a divergence hundreds of ticks later.
	//
	// The case that most needs it is a compound of offset shapes. A spiked ball has
	// twenty-five geometries, local poses and material bindings that all have to match,
	// and -- because a near-isotropic compound's mass is deliberately canonicalised to
	// an identity frame, mean moments and an origin centre of mass -- its mass hash is
	// specifically designed NOT to reflect small shape differences. The construction
	// hash is what is left to catch them.

	// A world holding one body, built from the given shapes, for comparing how two
	// peers constructed the same entity.
	struct ConstructionTestWorld
	{
		PxwWorld* world;
		PxRigidDynamic* body;

		ConstructionTestWorld() : world(NULL), body(NULL) {}

		void Build(const std::vector<MassTestShape>& shapes, PxReal friction, PxReal maxDepenetration)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(friction, 0.5f, 0.1f);
			body = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));

			for (size_t i = 0; i < shapes.size(); ++i)
			{
				PxShape* shape = physics->createShape(PxBoxGeometry(shapes[i].halfExtents), *material, true);
				shape->setLocalPose(shapes[i].pose);
				body->attachShape(*shape);
				shape->release();
			}
			material->release();

			PxwSetupDeterministicMass(body, 10.0f, -1.0f, false, NULL);
			PxwApplyDeterministicRigidDefaults(body, 8, 2);
			body->setMaxDepenetrationVelocity(maxDepenetration);

			PxwWorldRegister(world, 100, body, PxwHandleKind::eRIGID_DYNAMIC);
			PxwWorldCommitPending(world);
		}

		PxU64 Hash() const { return PxwWorldHashConstruction(world); }

		void Destroy()
		{
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
				body = NULL;
			}
		}
	};

	// Nudges one component of one shape's local pose by a single representable step,
	// the smallest difference two peers can possibly have.
	std::vector<MassTestShape> NudgeOneShapeByOneUlp(const std::vector<MassTestShape>& shapes)
	{
		std::vector<MassTestShape> out = shapes;
		if (out.size() > 1)
		{
			PxU32 bits;
			std::memcpy(&bits, &out[1].pose.p.x, sizeof(bits));
			++bits;
			std::memcpy(&out[1].pose.p.x, &bits, sizeof(bits));
		}
		return out;
	}

	void TestConstructionHashAgreesForIdenticalBuilds()
	{
		std::printf("TestConstructionHashAgreesForIdenticalBuilds\n");

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(24, 0.0f);

		ConstructionTestWorld a, b;
		a.Build(shapes, 0.6f, 3.0f);
		b.Build(shapes, 0.6f, 3.0f);

		Check(a.Hash() == b.Hash(),
			"two worlds whose bodies were built the same way hash identically");

		// The construction does not change as the simulation runs, so it must survive
		// stepping. A hash that drifted would be useless for comparing peers mid-session.
		const PxU64 before = a.Hash();
		for (int i = 0; i < 30; ++i)
		{
			PxwWorldStep(a.world, 1.0f / 60.0f);
		}
		Check(a.Hash() == before, "the construction hash is unchanged by stepping the world");

		a.Destroy();
		b.Destroy();
	}

	// The headline case, and the reason this hash exists. A single ULP in one spike's
	// local pose leaves the mass properties and the whole state blob identical -- the
	// mass is canonicalised precisely so that it does -- while genuinely changing the
	// body PhysX solves.
	void TestConstructionHashCatchesWhatMassAndStateHashesMiss()
	{
		std::printf("TestConstructionHashCatchesWhatMassAndStateHashesMiss\n");

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(24, 0.0f);
		const std::vector<MassTestShape> nudged = NudgeOneShapeByOneUlp(shapes);

		ConstructionTestWorld reference, perturbed;
		reference.Build(shapes, 0.6f, 3.0f);
		perturbed.Build(nudged, 0.6f, 3.0f);

		PxwMassProperties massA, massB;
		PxwGetMassProperties(reference.body, &massA);
		PxwGetMassProperties(perturbed.body, &massB);

		const bool massAgrees = PxwHashMassProperties(&massA) == PxwHashMassProperties(&massB);
		const bool stateAgrees = PxwWorldHashState(reference.world) == PxwWorldHashState(perturbed.world);
		const bool constructionAgrees = reference.Hash() == perturbed.Hash();

		std::printf("        one spike moved by 1 ULP: mass hash %s, state hash %s, construction hash %s\n",
			massAgrees ? "AGREES" : "differs",
			stateAgrees ? "AGREES" : "differs",
			constructionAgrees ? "AGREES" : "differs");

		Check(massAgrees,
			"a one-ULP shape offset leaves the canonicalised mass properties identical");
		Check(stateAgrees,
			"a one-ULP shape offset leaves the state hash identical");
		Check(!constructionAgrees,
			"the construction hash notices a one-ULP shape offset that every other hash misses");

		reference.Destroy();
		perturbed.Destroy();
	}

	// The other properties that reach the solver without ever reaching a snapshot. The
	// depenetration clamp is the sharpest of them: it does nothing at all until bodies
	// are deeply overlapped, so a mismatch stays invisible until something squeezes.
	void TestConstructionHashCatchesSolverProperties()
	{
		std::printf("TestConstructionHashCatchesSolverProperties\n");

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(24, 0.0f);

		ConstructionTestWorld reference, clamp, friction, iterations;
		reference.Build(shapes, 0.6f, 3.0f);
		clamp.Build(shapes, 0.6f, PX_MAX_F32);
		friction.Build(shapes, 0.6000001f, 3.0f);
		iterations.Build(shapes, 0.6f, 3.0f);
		iterations.body->setSolverIterationCounts(9, 2);

		Check(reference.Hash() != clamp.Hash(),
			"the construction hash notices a different max depenetration velocity");
		Check(reference.Hash() != friction.Hash(),
			"the construction hash notices a different material friction");
		Check(reference.Hash() != iterations.Hash(),
			"the construction hash notices different solver iteration counts");

		// Shapes are hashed in attachment order, because PhysX generates contacts in that
		// order: the same shapes attached differently are not the same body.
		ConstructionTestWorld reversed;
		std::vector<MassTestShape> backwards(shapes.rbegin(), shapes.rend());
		reversed.Build(backwards, 0.6f, 3.0f);
		Check(reference.Hash() != reversed.Hash(),
			"the construction hash notices a different shape attachment order");

		reference.Destroy();
		clamp.Destroy();
		friction.Destroy();
		iterations.Destroy();
		reversed.Destroy();
	}

	// A whole-world hash says only that something differs. The per-entry form has to
	// name which body, or a session with fifty actors is no better off.
	void TestConstructionHashPerEntryNamesTheBody()
	{
		std::printf("TestConstructionHashPerEntryNamesTheBody\n");

		const std::vector<MassTestShape> shapes = MakeSpikedBallShapes(24, 0.0f);
		const std::vector<MassTestShape> nudged = NudgeOneShapeByOneUlp(shapes);

		ConstructionTestWorld reference, perturbed;
		reference.Build(shapes, 0.6f, 3.0f);
		perturbed.Build(nudged, 0.6f, 3.0f);

		PxwEntryHash a[8];
		PxwEntryHash b[8];
		const PxU32 countA = PxwWorldHashConstructionPerEntry(reference.world, a, 8);
		const PxU32 countB = PxwWorldHashConstructionPerEntry(perturbed.world, b, 8);

		Check(countA == countB && countA > 0, "both worlds report the same number of entries");

		int mismatches = 0;
		PxU32 mismatchedId = 0;
		for (PxU32 i = 0; i < countA && i < countB; ++i)
		{
			if (a[i].stableId != b[i].stableId || a[i].hash != b[i].hash)
			{
				++mismatches;
				mismatchedId = a[i].stableId;
			}
		}

		std::printf("        %d of %u entries differ, first is stable id %u\n",
			mismatches, countA, mismatchedId);
		Check(mismatches == 1 && mismatchedId == 100,
			"exactly the body that was built differently is reported as differing");

		reference.Destroy();
		perturbed.Destroy();
	}

	// A world holding one body with a single caller-supplied shape, for comparing how the
	// construction hash treats geometries the box-based harness above cannot express.
	struct GeometryTestWorld
	{
		PxwWorld* world;
		PxRigidDynamic* body;

		GeometryTestWorld() : world(NULL), body(NULL) {}

		void Build(const PxGeometry& geometry)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);
			body = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));

			PxShape* shape = physics->createShape(geometry, *material, true);
			body->attachShape(*shape);
			shape->release();
			material->release();

			PxwSetupDeterministicMass(body, 10.0f, -1.0f, false, NULL);
			PxwApplyDeterministicRigidDefaults(body, 8, 2);

			PxwWorldRegister(world, 100, body, PxwHandleKind::eRIGID_DYNAMIC);
			PxwWorldCommitPending(world);
		}

		PxU64 Hash() const { return PxwWorldHashConstruction(world); }

		void Destroy()
		{
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
				body = NULL;
			}
		}
	};

	PxU64 HashOfWorldWith(const PxGeometry& geometry)
	{
		GeometryTestWorld w;
		w.Build(geometry);
		const PxU64 hash = w.Hash();
		w.Destroy();
		return hash;
	}

	// Convex core geometry has to be hashed more carefully than any other geometry type.
	// PxConvexCoreGeometry holds a fixed PxU8[24] core buffer but memcpys only sizeof(Core)
	// bytes into it -- eight, for a cylinder -- and leaves the rest of the buffer at whatever
	// the stack or the allocator happened to contain. Hashing the buffer wholesale would
	// therefore fold uninitialised memory into the construction hash, and two peers holding
	// genuinely identical cylinders would report a construction mismatch that has nothing to do
	// with how either of them was built. Only the active core's bytes may participate.
	void TestConstructionHashDescribesConvexCores()
	{
		std::printf("TestConstructionHashDescribesConvexCores\n");

		const PxConvexCoreGeometry cylinder(PxConvexCore::Cylinder(0.3f, 0.35f), 0.0f);

		Check(HashOfWorldWith(cylinder) == HashOfWorldWith(cylinder),
			"two worlds holding the same cylinder hash identically");

		// Every parameter of the shape has to reach the hash, or a peer that authored a
		// slightly different wheel would look identical.
		Check(HashOfWorldWith(cylinder) != HashOfWorldWith(PxConvexCoreGeometry(PxConvexCore::Cylinder(0.3f, 0.3500001f), 0.0f)),
			"the construction hash notices a cylinder radius that differs by one step");
		Check(HashOfWorldWith(cylinder) != HashOfWorldWith(PxConvexCoreGeometry(PxConvexCore::Cylinder(0.3000001f, 0.35f), 0.0f)),
			"the construction hash notices a cylinder height that differs by one step");
		Check(HashOfWorldWith(cylinder) != HashOfWorldWith(PxConvexCoreGeometry(PxConvexCore::Cylinder(0.3f, 0.35f), 0.01f)),
			"the construction hash notices a different margin");

		// A cone and a cylinder with the same numbers are different shapes whose core bytes are
		// identical, so the core type has to participate in its own right.
		Check(HashOfWorldWith(cylinder) != HashOfWorldWith(PxConvexCoreGeometry(PxConvexCore::Cone(0.3f, 0.35f), 0.0f)),
			"the construction hash distinguishes a cone from a cylinder with the same dimensions");

		// The headline case. Two cylinders that describe the same shape, one with garbage in the
		// core bytes the cylinder core does not use, must hash the same: that garbage is exactly
		// what differs between two processes that did the same thing.
		PxConvexCoreGeometry scribbled(PxConvexCore::Cylinder(0.3f, 0.35f), 0.0f);
		PxU8* coreBytes = const_cast<PxU8*>(static_cast<const PxU8*>(scribbled.getCoreData()));
		for (PxU32 i = sizeof(PxReal) * 2; i < PxConvexCoreGeometry::MAX_CORE_SIZE; ++i)
		{
			coreBytes[i] = static_cast<PxU8>(0xAB);
		}

		Check(HashOfWorldWith(cylinder) == HashOfWorldWith(scribbled),
			"the construction hash ignores the core bytes a cylinder does not use");
	}

	// The collision group table decides whether two shapes are allowed to touch at all, so
	// peers that disagree about it simulate differently while every actor, shape and geometry
	// hashes the same. It is also not scene state: PhysX extensions keeps it process-global, so
	// no other part of the construction hash can stand in for it.
	void TestConstructionHashIncludesCollisionGroupTable()
	{
		std::printf("TestConstructionHashIncludesCollisionGroupTable\n");

		GeometryTestWorld w;
		w.Build(PxBoxGeometry(0.5f, 0.5f, 0.5f));

		const PxU64 allCollide = w.Hash();

		SetGroupCollisionFlag(1, 2, false);
		const PxU64 pairDisabled = w.Hash();
		Check(pairDisabled != allCollide,
			"the construction hash notices a group pair that no longer collides");

		// The table is symmetric, so naming the pair the other way round is the same entry
		// rather than a second one.
		SetGroupCollisionFlag(2, 1, false);
		Check(w.Hash() == pairDisabled,
			"the construction hash treats a group pair as symmetric");

		SetGroupCollisionFlag(3, 4, false);
		Check(w.Hash() != pairDisabled,
			"the construction hash notices a second disabled group pair");

		// Only disabled pairs contribute, so restoring the all-collide default has to restore
		// the original hash exactly. That is what lets filtering be added to a session without
		// invalidating the hash of every world that does not use it.
		ResetGroupCollisionFlags();
		Check(w.Hash() == allCollide,
			"resetting the group table restores the hash a world with no filtering had");

		w.Destroy();
	}

	// Applying replicated properties must be verbatim, otherwise replicating them
	// solves nothing.
	void TestApplyMassIsVerbatim()
	{
		std::printf("TestApplyMassIsVerbatim\n");

		PxRigidDynamic* body = MakeMassTestBody(MakeSpikedBallShapes(12, 0.0f), false);

		PxwMassProperties computed;
		PxwComputeMassProperties(body, 10.0f, -1.0f, false, &computed);

		const PxTransform poseBefore = body->getGlobalPose();
		Check(PxwApplyMassProperties(body, &computed) == PxwResult::eOK, "applying mass properties succeeded");

		PxwMassProperties readBack;
		PxwGetMassProperties(body, &readBack);
		Check(PxwHashMassProperties(&computed) == PxwHashMassProperties(&readBack),
			"mass properties read back exactly as they were applied");

		// The PhysX fix for setCMassLocalPose should hold here too.
		Check(body->getGlobalPose().p == poseBefore.p && body->getGlobalPose().q.x == poseBefore.q.x &&
			body->getGlobalPose().q.y == poseBefore.q.y && body->getGlobalPose().q.z == poseBefore.q.z &&
			body->getGlobalPose().q.w == poseBefore.q.w,
			"applying mass properties leaves the actor pose untouched");

		// Re-applying must be idempotent, since setup code often runs more than once.
		PxwApplyMassProperties(body, &computed);
		PxwMassProperties twice;
		PxwGetMassProperties(body, &twice);
		Check(PxwHashMassProperties(&readBack) == PxwHashMassProperties(&twice),
			"applying the same mass properties twice changes nothing");

		body->release();
	}

	// The hash exists to catch a mismatched peer before the simulation starts, so it
	// has to actually notice the things that would desync one.
	void TestMassHashDetectsMismatch()
	{
		std::printf("TestMassHashDetectsMismatch\n");

		PxRigidDynamic* twelve = MakeMassTestBody(MakeSpikedBallShapes(12, 0.0f), false);
		PxRigidDynamic* thirteen = MakeMassTestBody(MakeSpikedBallShapes(13, 0.0f), false);

		PxwMassProperties a, b, denser;
		PxwComputeMassProperties(twelve, 10.0f, -1.0f, false, &a);
		PxwComputeMassProperties(thirteen, 10.0f, -1.0f, false, &b);
		PxwComputeMassProperties(twelve, 11.0f, -1.0f, false, &denser);

		Check(PxwHashMassProperties(&a) != PxwHashMassProperties(&b),
			"a different shape count produces a different hash");
		Check(PxwHashMassProperties(&a) != PxwHashMassProperties(&denser),
			"a different density produces a different hash");

		PxwMassProperties nudged = a;
		nudged.inertia.y *= 1.0000001f;
		Check(PxwHashMassProperties(&a) != PxwHashMassProperties(&nudged),
			"a one-part-in-ten-million inertia change produces a different hash");

		twelve->release();
		thirteen->release();
	}

	// A collapsed mass frame is the identity, which is exactly the condition under
	// which the actor pose round trip is lossless. Confirm the two fixes compose.
	void TestCollapsedMassKeepsPoseRoundTripExact()
	{
		std::printf("TestCollapsedMassKeepsPoseRoundTripExact\n");

		PxRigidDynamic* body = MakeMassTestBody(MakeSpikedBallShapes(12, 0.0f), false);
		PxwSetupDeterministicMass(body, 10.0f, -1.0f, false, NULL);

		const PxTransform requested(PxVec3(3.5f, 12.25f, -7.125f),
			PxQuat(0.3f, PxVec3(0.267261f, 0.534522f, 0.801784f)).getNormalized());

		PxTransform pose = requested;
		bool exact = true;
		for (int i = 0; i < 240; ++i)
		{
			body->setGlobalPose(pose);
			const PxTransform readBack = body->getGlobalPose();
			if (!(readBack.p.x == pose.p.x && readBack.p.y == pose.p.y && readBack.p.z == pose.p.z &&
				  readBack.q.x == pose.q.x && readBack.q.y == pose.q.y &&
				  readBack.q.z == pose.q.z && readBack.q.w == pose.q.w))
			{
				exact = false;
			}
			pose = readBack;
		}

		Check(exact, "240 capture/restore cycles on a spiked ball are bitwise lossless");

		body->release();
	}

	// The gameplay body API and scene queries were added on top of the determinism core.
	// The most important thing these establish is that PxwWorldReadPoses uses the
	// quaternion-first layout the managed SimTransform marshals into: a transposed pose
	// is a silent corruption that no exception catches, only wrong-looking rendering.
	void TestBodyApiAndReadPoseLayout()
	{
		std::printf("TestBodyApiAndReadPoseLayout\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

		PxBoxGeometry groundGeom(50.0f, 1.0f, 50.0f);
		PxShape* groundShape = physics->createShape(groundGeom, *material, true);
		PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
		ground->attachShape(*groundShape);
		groundShape->release();
		PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);

		PxBoxGeometry boxGeom(0.5f, 0.5f, 0.5f);
		PxShape* boxShape = physics->createShape(boxGeom, *material, true);
		PxRigidDynamic* box = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));
		box->attachShape(*boxShape);
		boxShape->release();
		PxRigidBodyExt::updateMassAndInertia(*box, 10.0f);
		PxwApplyDeterministicRigidDefaults(box, 8, 2);
		PxwWorldRegister(world, 100, box, PxwHandleKind::eRIGID_DYNAMIC);

		PxwWorldCommitPending(world);
		material->release();

		PxwPose pose;
		PxwBodyGetPose(box, &pose);
		Check(PxAbs(pose.ToPxTransform().p.y - 5.0f) < 1.0e-4f, "PxwBodyGetPose returns the body position");
		Check(PxAbs(PxwBodyGetMass(box) - box->getMass()) < 1.0e-4f, "PxwBodyGetMass matches the actor mass");

		// Teleport places the body and, like a restore, re-pins the wake counter so a
		// pooled body spawns awake rather than inheriting the slot's sleep state.
		const PxTransform target(PxVec3(1.0f, 8.0f, -2.0f),
			PxQuat(0.4f, PxVec3(0.267261f, 0.534522f, 0.801784f)).getNormalized());
		PxwPose targetPose(target);
		const PxVec3 zero(0.0f);
		PxwBodyTeleport(box, &targetPose, &zero, &zero);
		Check(!box->isSleeping(), "a teleported body is awake");
		Check(box->getWakeCounter() > 1.0e30f, "teleport pins the wake counter high");

		PxwPoseEntry entries[2];
		const PxU32 poseCount = PxwWorldReadPoses(world, entries, 2);
		Check(poseCount == 2, "pose readback returns both entries");

		const PxwPoseEntry* boxEntry = NULL;
		for (PxU32 i = 0; i < poseCount; ++i)
		{
			if (entries[i].stableId == 100u)
			{
				boxEntry = &entries[i];
			}
		}
		Check(boxEntry != NULL, "the dynamic body appears in the pose readback");
		if (boxEntry != NULL)
		{
			const PxTransform live = box->getGlobalPose();
			const PxTransform readback = boxEntry->pose.ToPxTransform();
			Check(PxAbs(readback.p.x - live.p.x) < 1.0e-5f && PxAbs(readback.p.y - live.p.y) < 1.0e-5f &&
				  PxAbs(readback.p.z - live.p.z) < 1.0e-5f,
				"ReadPoses position matches the live pose");

			// The layout fix itself: a managed SimTransform is quaternion-first, so the
			// first four floats of the pose must be the quaternion and the next three the
			// position. Reading them raw catches a transposed struct that ToPxTransform
			// would hide.
			const float* raw = reinterpret_cast<const float*>(&boxEntry->pose);
			Check(raw[0] == live.q.x && raw[1] == live.q.y && raw[2] == live.q.z && raw[3] == live.q.w,
				"ReadPoses lays the quaternion out first, matching SimTransform");
			Check(raw[4] == live.p.x && raw[5] == live.p.y && raw[6] == live.p.z,
				"ReadPoses lays the position after the quaternion, matching SimTransform");
		}

		const PxVec3 up(0.0f, 3.0f, 0.0f);
		PxwBodySetLinearVelocity(box, &up);
		PxVec3 readVel;
		PxwBodyGetLinearVelocity(box, &readVel);
		Check(PxAbs(readVel.y - 3.0f) < 1.0e-4f, "SetLinearVelocity then GetLinearVelocity round-trips");

		PxwWorldDestroy(world);
	}

	// The managed DeterministicWorld.Register calls PxwApplyDeterministicRigidDefaults on
	// every dynamic body it registers, and the determinism the whole suite measures rests
	// on what that call does. PhysX does not default to these values: speculative CCD keys
	// its contact generation off velocity history, so a restored state would generate
	// different contacts from the state it was captured from, and the default max
	// depenetration velocity is effectively unbounded. This pins both so a change in the
	// helper, or in a PhysX default, is caught here rather than as a slow desync in a game.
	void TestDeterministicRigidDefaults()
	{
		std::printf("TestDeterministicRigidDefaults\n");

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);
		PxShape* shape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);
		PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));
		body->attachShape(*shape);
		shape->release();
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

		// A fresh dynamic starts with speculative CCD off in this PhysX build, so enable it
		// first: the test has to prove the helper clears it, not that it happened to be
		// clear already.
		body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);

		PxwApplyDeterministicRigidDefaults(body, 7, 3);

		PxU32 posIters = 0, velIters = 0;
		body->getSolverIterationCounts(posIters, velIters);
		Check(posIters == 7u && velIters == 3u, "the deterministic defaults apply the requested solver iteration counts");
		Check(!(body->getRigidBodyFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD),
			"the deterministic defaults clear speculative CCD");
		Check(body->getMaxDepenetrationVelocity() < 1.0e6f,
			"the deterministic defaults bound the max depenetration velocity");

		body->release();
		material->release();
	}

	// An entity pool registers every slot up front and parks the ones nobody has spawned,
	// which is what keeps the snapshot layout constant while players come and go. Parking
	// raises PxActorFlag::eDISABLE_SIMULATION, and that tears down the body's simulation
	// object while leaving the actor in the scene -- so a parked slot answers getScene()
	// with the scene it is in but has nothing behind it to drive.
	//
	// The restore path did not distinguish the two and cleared the force and torque
	// accumulators of every dynamic it walked. On a parked slot that reached
	// BodySim::raiseVelocityModFlagAndNotify through a torn-down sim and took the process
	// down. Nothing reported it first: the PX_CHECK_AND_RETURN that says the call is
	// illegal is compiled out of a release PhysX build.
	//
	// Every peer rolls back, and a pool almost always holds an unspawned slot, so this
	// crashed on the first rewind of any session that used one.
	void TestRestoreWithParkedPoolSlot()
	{
		std::printf("TestRestoreWithParkedPoolSlot\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

		PxShape* groundShape = physics->createShape(PxBoxGeometry(50.0f, 1.0f, 50.0f), *material, true);
		PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
		ground->attachShape(*groundShape);
		groundShape->release();
		PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);

		PxShape* boxShape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);

		// One slot in play (id 10) and one nobody has spawned (id 20).
		PxRigidDynamic* spawned = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 4.0f, 0.0f)));
		spawned->attachShape(*boxShape);
		PxRigidBodyExt::updateMassAndInertia(*spawned, 10.0f);
		PxwApplyDeterministicRigidDefaults(spawned, 8, 2);
		PxwWorldRegister(world, 10, spawned, PxwHandleKind::eRIGID_DYNAMIC);

		PxRigidDynamic* parked = physics->createRigidDynamic(PxTransform(PxVec3(3.0f, 4.0f, 0.0f)));
		parked->attachShape(*boxShape);
		PxRigidBodyExt::updateMassAndInertia(*parked, 10.0f);
		PxwApplyDeterministicRigidDefaults(parked, 8, 2);
		PxwWorldRegister(world, 20, parked, PxwHandleKind::eRIGID_DYNAMIC);

		boxShape->release();
		PxwWorldCommitPending(world);
		material->release();

		PxwWorldSetEntryEnabled(world, 20, false);
		Check(parked->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION),
			"parking a pool slot disables simulation on its body");
		Check(parked->getScene() != NULL,
			"a parked slot stays in the scene, so scene membership cannot be used to detect it");

		for (int tick = 0; tick < 10; ++tick)
		{
			PxwWorldStep(world, kDt);
		}

		std::vector<PxU8> snapshot(PxwWorldStateSize(world));
		PxU64 hash = 0;
		const PxU32 written = PxwWorldCaptureState(world, snapshot.data(), static_cast<PxU32>(snapshot.size()), &hash);
		snapshot.resize(written);
		Check(written > 0, "a world holding a parked pool slot captures");

		// The crash.
		const PxI32 restored = PxwWorldRestoreState(world, snapshot.data(), static_cast<PxU32>(snapshot.size()));
		Check(restored == PxwResult::eOK, "a world holding a parked pool slot restores");

		// Capture and restore have to agree on a parked slot too, or a peer that rebuilt
		// the slot from the snapshot reports a different confirmed hash than the peer
		// that parked it.
		std::vector<PxU8> second(PxwWorldStateSize(world));
		PxU64 secondHash = 0;
		const PxU32 secondWritten = PxwWorldCaptureState(world, second.data(), static_cast<PxU32>(second.size()), &secondHash);
		second.resize(secondWritten);
		Check(secondWritten == written && std::memcmp(snapshot.data(), second.data(), written) == 0,
			"capture and restore round-trip a parked pool slot losslessly");

		// Spawning the slot: the pool places the body and brings it back into play.
		PxwPose pose;
		PxwBodyGetPose(parked, &pose);
		PxwBodyTeleport(parked, &pose, NULL, NULL);
		PxwWorldSetEntryEnabled(world, 20, true);
		Check(!parked->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION),
			"spawning a pool slot re-enables simulation on its body");

		PxwWorldStep(world, kDt);
		const PxI32 afterSpawn = PxwWorldRestoreState(world, snapshot.data(), static_cast<PxU32>(snapshot.size()));
		Check(afterSpawn == PxwResult::eOK, "restoring a snapshot that parks a slot the world has spawned succeeds");

		// Rewinding past a despawn: the slot is in play and moving when the snapshot is
		// taken, parked by the time the rollback restores it. Restoring has to bring the
		// whole body back, velocity included, which only works if the parked state is
		// applied before the rest of the state rather than after.
		PxwWorldSetEntryEnabled(world, 20, true);
		const PxVec3 launch(1.5f, 0.0f, -2.5f);
		PxwBodySetLinearVelocity(parked, &launch);
		for (int tick = 0; tick < 3; ++tick)
		{
			PxwWorldStep(world, kDt);
		}

		std::vector<PxU8> inPlay(PxwWorldStateSize(world));
		PxU64 inPlayHash = 0;
		const PxU32 inPlayWritten = PxwWorldCaptureState(world, inPlay.data(), static_cast<PxU32>(inPlay.size()), &inPlayHash);
		inPlay.resize(inPlayWritten);
		PxVec3 expected(0.0f);
		PxwBodyGetLinearVelocity(parked, &expected);
		Check(expected.magnitudeSquared() > 0.0f, "the slot is moving when the snapshot is taken");

		PxwWorldSetEntryEnabled(world, 20, false);
		PxwWorldRestoreState(world, inPlay.data(), static_cast<PxU32>(inPlay.size()));

		PxVec3 actual(0.0f);
		PxwBodyGetLinearVelocity(parked, &actual);
		Check(!parked->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION),
			"restoring a snapshot that had the slot in play brings it back into play");
		Check((actual - expected).magnitudeSquared() == 0.0f,
			"rewinding past a despawn restores the slot's velocity, not just its pose");

		PxwWorldDestroy(world);
	}

	// Contact events must resolve to stable IDs, normalise to idA < idB with the normal
	// oriented A toward B, sort by the ID pair, and -- because SimGameHost drains on replay
	// ticks too -- produce the same sorted set when a tick is replayed. A stack of two boxes
	// on the ground exercises all of it: a ground-box contact and a box-box contact, whose
	// normals point opposite ways once normalised.
	void TestContactEvents()
	{
		std::printf("TestContactEvents\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

		PxShape* groundShape = physics->createShape(PxBoxGeometry(50.0f, 1.0f, 50.0f), *material, true);
		PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
		ground->attachShape(*groundShape);
		groundShape->release();
		PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);

		// Box A (id 100) rests on the ground; box B (id 50) rests on A. B's id is smaller,
		// so the box-box pair normalises to idA = 50, idB = 100.
		PxShape* boxShape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);
		PxRigidDynamic* boxA = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 0.5f, 0.0f)));
		boxA->attachShape(*boxShape);
		PxRigidBodyExt::updateMassAndInertia(*boxA, 10.0f);
		PxwApplyDeterministicRigidDefaults(boxA, 8, 2);
		PxwWorldRegister(world, 100, boxA, PxwHandleKind::eRIGID_DYNAMIC);

		PxRigidDynamic* boxB = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 1.5f, 0.0f)));
		boxB->attachShape(*boxShape);
		boxShape->release();
		PxRigidBodyExt::updateMassAndInertia(*boxB, 10.0f);
		PxwApplyDeterministicRigidDefaults(boxB, 8, 2);
		PxwWorldRegister(world, 50, boxB, PxwHandleKind::eRIGID_DYNAMIC);

		PxwWorldCommitPending(world);
		material->release();

		// Settle so the stack is in persistent contact.
		for (int i = 0; i < 60; ++i) { PxwWorldStep(world, kDt); }

		PxwContactEvent events[32];
		const PxU32 count = PxwWorldDrainContacts(world, events, 32);
		Check(count >= 2, "a settled two-box stack reports at least the ground and box-box contacts");

		bool sorted = true;
		bool normalised = true;
		for (PxU32 i = 0; i < count; ++i)
		{
			if (events[i].idA >= events[i].idB) { normalised = false; }
			if (i > 0 && !(events[i - 1].idA < events[i].idA ||
				(events[i - 1].idA == events[i].idA && events[i - 1].idB <= events[i].idB)))
			{
				sorted = false;
			}
		}
		Check(normalised, "every contact is normalised to idA < idB");
		Check(sorted, "contacts are sorted by (idA, idB)");

		// Find the two contacts and check the normal orientation. Ground (1) -> box A (100)
		// points up; box B top (50) -> box A bottom (100) points down.
		const PxwContactEvent* groundBox = NULL;
		const PxwContactEvent* boxBox = NULL;
		for (PxU32 i = 0; i < count; ++i)
		{
			if (events[i].idA == 1u && events[i].idB == 100u) { groundBox = &events[i]; }
			if (events[i].idA == 50u && events[i].idB == 100u) { boxBox = &events[i]; }
		}
		Check(groundBox != NULL && groundBox->normal.y > 0.9f,
			"the ground-to-box normal points from A (ground) up toward B (box)");
		Check(boxBox != NULL && boxBox->normal.y < -0.9f,
			"the box-to-box normal points from A (upper box) down toward B (lower box)");

		// Truncation keeps the front of the sorted list: the smallest pair, (1, 100).
		PxwContactEvent one[1];
		const PxU32 truncated = PxwWorldDrainContacts(world, one, 1);
		Check(truncated == 1 && one[0].idA == 1u && one[0].idB == 100u,
			"capacity truncates to the front of the sorted list");

		// A replayed tick must produce the same sorted event set. Capture, step and drain
		// once, then restore and step the same tick again and drain: the two sets match.
		std::vector<PxU8> snapshot(PxwWorldStateSize(world));
		PxU64 hash = 0;
		const PxU32 written = PxwWorldCaptureState(world, snapshot.data(), static_cast<PxU32>(snapshot.size()), &hash);
		snapshot.resize(written);

		PxwWorldStep(world, kDt);
		PxwContactEvent first[32];
		const PxU32 firstCount = PxwWorldDrainContacts(world, first, 32);

		PxwWorldRestoreState(world, snapshot.data(), static_cast<PxU32>(snapshot.size()));
		PxwWorldStep(world, kDt);
		PxwContactEvent replay[32];
		const PxU32 replayCount = PxwWorldDrainContacts(world, replay, 32);

		bool pairsMatch = (firstCount == replayCount);
		bool geometryMatches = pairsMatch;
		for (PxU32 i = 0; i < firstCount && pairsMatch; ++i)
		{
			if (first[i].idA != replay[i].idA || first[i].idB != replay[i].idB) { pairsMatch = false; }
			if (std::memcmp(&first[i].point, &replay[i].point, sizeof(PxVec3)) != 0 ||
				std::memcmp(&first[i].normal, &replay[i].normal, sizeof(PxVec3)) != 0 ||
				first[i].impulse != replay[i].impulse) { geometryMatches = false; }
		}
		// The determinism-relevant property: the same contacts, normalised and in the same
		// order. This is what gameplay may branch its hashed state on.
		Check(pairsMatch, "a replayed tick produces the same sorted contact set");
		// The point, normal and impulse are derived from solver warm-start state, which the
		// snapshot deliberately does not carry, so they are only approximate across a
		// cold restore -- the same "as close as possible, not bit-exact" property the pose
		// replay has. Recorded, not asserted; gameplay must not branch hashed state on them.
		Observe(geometryMatches, "a replayed tick reproduces contact point, normal and impulse bit-for-bit");

		PxwWorldDestroy(world);
	}

	// A trigger volume reports a Found when a body enters and a Lost when it leaves, each
	// resolved to the two stable IDs, sorted by (triggerId, otherId). A box falling through
	// a static trigger produces exactly that pair over its passage.
	void TestTriggerEvents()
	{
		std::printf("TestTriggerEvents\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

		// A static trigger box (id 2), centred at y = 5, spanning y in [4, 6].
		PxShape* triggerShape = physics->createShape(PxBoxGeometry(1.0f, 1.0f, 1.0f), *material, false);
		triggerShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		triggerShape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
		PxRigidStatic* trigger = physics->createRigidStatic(PxTransform(PxVec3(0.0f, 5.0f, 0.0f)));
		trigger->attachShape(*triggerShape);
		triggerShape->release();
		PxwWorldRegister(world, 2, trigger, PxwHandleKind::eRIGID_STATIC);

		// A dynamic box (id 200) starting above the trigger, falling straight through it.
		PxShape* boxShape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);
		PxRigidDynamic* box = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, 9.0f, 0.0f)));
		box->attachShape(*boxShape);
		boxShape->release();
		PxRigidBodyExt::updateMassAndInertia(*box, 10.0f);
		PxwApplyDeterministicRigidDefaults(box, 8, 2);
		PxwWorldRegister(world, 200, box, PxwHandleKind::eRIGID_DYNAMIC);

		PxwWorldCommitPending(world);
		material->release();

		bool sawFound = false;
		bool sawLost = false;
		bool idsCorrect = true;
		for (int i = 0; i < 120; ++i)
		{
			PxwWorldStep(world, kDt);
			PxwTriggerEvent triggers[8];
			const PxU32 n = PxwWorldDrainTriggers(world, triggers, 8);
			for (PxU32 t = 0; t < n; ++t)
			{
				if (triggers[t].triggerId != 2u || triggers[t].otherId != 200u) { idsCorrect = false; }
				if (triggers[t].status == static_cast<PxU32>(PxwTriggerStatus::eFOUND)) { sawFound = true; }
				if (triggers[t].status == static_cast<PxU32>(PxwTriggerStatus::eLOST)) { sawLost = true; }
			}
		}

		Check(sawFound, "a body entering a trigger volume reports a Found event");
		Check(sawLost, "a body leaving a trigger volume reports a Lost event");
		Check(idsCorrect, "trigger events resolve to the trigger and the other body's stable IDs");

		PxwWorldDestroy(world);
	}

	// Queries must resolve every hit to a stable ID and return a deterministic order,
	// since two peers iterating the same hits in a different order would desync.
	void TestSceneQueries()
	{
		std::printf("TestSceneQueries\n");

		PxwSceneDesc desc = MakeDeterministicSceneDesc();
		PxwWorld* world = PxwWorldCreate(&desc);

		PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
		PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

		PxBoxGeometry groundGeom(50.0f, 1.0f, 50.0f);
		PxShape* groundShape = physics->createShape(groundGeom, *material, true);
		PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
		ground->attachShape(*groundShape);
		groundShape->release();
		PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);

		// Two boxes on the same vertical column. The lower stable ID sits lower, so a ray
		// from above hits the higher stable ID first: distance order is not ID order,
		// which is what makes the sort worth testing.
		for (int k = 0; k < 2; ++k)
		{
			const PxU32 id = 100u + static_cast<PxU32>(k);
			const float y = (k == 0) ? 2.0f : 5.0f;
			PxBoxGeometry g(0.5f, 0.5f, 0.5f);
			PxShape* s = physics->createShape(g, *material, true);
			PxRigidDynamic* b = physics->createRigidDynamic(PxTransform(PxVec3(0.0f, y, 0.0f)));
			b->attachShape(*s);
			s->release();
			PxRigidBodyExt::updateMassAndInertia(*b, 10.0f);
			PxwApplyDeterministicRigidDefaults(b, 8, 2);
			PxwWorldRegister(world, id, b, PxwHandleKind::eRIGID_DYNAMIC);
		}
		PxwWorldCommitPending(world);
		material->release();

		PxVec3 origin(0.0f, 20.0f, 0.0f);
		PxVec3 down(0.0f, -1.0f, 0.0f);
		PxwRaycastHit hits[8];

		const PxU32 n = PxwWorldRaycast(world, &origin, &down, 30.0f, 0u, hits, 8);
		Check(n == 3u, "a downward ray hits both boxes and the ground");
		Check(n >= 1u && hits[0].stableId == 101u, "the nearest hit is the top box, sorted by distance not stable ID");

		bool ascending = true;
		for (PxU32 i = 1; i < n; ++i)
		{
			if (hits[i].distance < hits[i - 1].distance)
			{
				ascending = false;
			}
		}
		Check(ascending, "ray hits are sorted by ascending distance");

		bool resolved = true;
		for (PxU32 i = 0; i < n; ++i)
		{
			if (hits[i].stableId != 1u && hits[i].stableId != 100u && hits[i].stableId != 101u)
			{
				resolved = false;
			}
		}
		Check(resolved, "every hit resolved to a registered stable ID");

		// A direction that need not be normalised: the native side normalises it, so a
		// longer vector must not change the reported distances.
		PxVec3 downLong(0.0f, -4.0f, 0.0f);
		PxwRaycastHit longHits[8];
		const PxU32 nLong = PxwWorldRaycast(world, &origin, &downLong, 30.0f, 0u, longHits, 8);
		Check(nLong == n && longHits[0].stableId == hits[0].stableId &&
			PxAbs(longHits[0].distance - hits[0].distance) < 1.0e-4f,
			"an unnormalised direction is normalised and does not change the hits");

		const PxU32 nStatic = PxwWorldRaycast(world, &origin, &down, 30.0f,
			1u << PxwHandleKind::eRIGID_STATIC, hits, 8);
		Check(nStatic == 1u && hits[0].stableId == 1u, "a filter mask selecting statics returns only the ground");

		const PxU32 nCapped = PxwWorldRaycast(world, &origin, &down, 30.0f, 0u, hits, 1);
		Check(nCapped == 1u && hits[0].stableId == 101u, "capacity truncates to the nearest hit");

		// An overlap sphere spanning both boxes must come back in ascending stable-ID
		// order, since an overlap has no distance to order by.
		PxVec3 center(0.0f, 3.5f, 0.0f);
		PxVec3 halfExtents(0.0f);
		PxQuat identity(PxIdentity);
		PxwOverlapHit overlaps[8];
		const PxU32 nOverlap = PxwWorldOverlap(world, PxwQueryShape::eSPHERE, &center, &halfExtents, 2.5f,
			&identity, 0u, overlaps, 8);
		Check(nOverlap == 2u, "an overlap sphere finds both boxes");
		Check(nOverlap == 2u && overlaps[0].stableId == 100u && overlaps[1].stableId == 101u,
			"overlap hits are sorted by ascending stable ID");

		PxwWorldDestroy(world);
	}

	// ---------------------------------------------------------------------------
	// Articulations
	//
	// The plugin captures and restores articulations already: root pose and
	// velocities, plus joint positions, velocities and forces through a
	// PxArticulationCache. Nothing exercised any of it until these tests, which ask an
	// articulation the same questions the tests above ask of a stack of boxes. Does a
	// restore reach a fixed point, does a replayed tick reproduce the original, and
	// does either answer depend on the solver.
	//
	// The capture is a subset by necessity: link velocities and accelerations are
	// solver outputs that cannot be written back, so they are excluded. For a
	// fixed-base chain that should still be complete, because the links' motion is a
	// function of the root and the joint state. "Should" is what these measure.

	const PxReal kLinkSpan = 1.0f;
	const PxReal kAnchorHeight = 3.0f;
	const int kChainLinks = 5;

	// A fixed-base chain of boxes bolted to the world at kAnchorHeight, laid out along
	// +X and hinged about Z so the whole thing swings down through the XY plane under
	// gravity. A chain rather than a single link, because what makes an articulation
	// different from a rigid body is that its links constrain each other, and a
	// one-link articulation is a rigid body with extra steps.
	struct ArticulationWorld
	{
		PxwWorld* world;
		PxArticulationReducedCoordinate* articulation;
		std::vector<PxArticulationLink*> links;

		ArticulationWorld() : world(NULL), articulation(NULL) {}

		// withGround: hangs the chain over the ground plane so the lower links pile up
		//             on it. Without it the chain never touches anything, which is the
		//             interesting case: no contacts means no persistent manifolds, so
		//             it isolates whatever the solver carries across a step in the
		//             joints alone.
		void Build(int linkCount = kChainLinks, bool withGround = false)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);
			PxwWorldSetSleepParams(world, 0.0f, 0.0f, 0u);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

			if (withGround)
			{
				PxBoxGeometry groundGeom(50.0f, 1.0f, 50.0f);
				PxShape* shape = physics->createShape(groundGeom, *material, true);
				PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
				ground->attachShape(*shape);
				shape->release();
				PxwWorldRegister(world, 1, ground, PxwHandleKind::eRIGID_STATIC);
			}

			articulation = physics->createArticulationReducedCoordinate();
			articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
			// Neighbouring links share a face at the hinge, so self-collision would
			// bury the joint behaviour under contacts between links that are supposed
			// to be touching.
			articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
			articulation->setSolverIterationCounts(8, 2);

			PxArticulationLink* parent = NULL;
			for (int i = 0; i < linkCount; ++i)
			{
				const PxTransform pose(PxVec3(kLinkSpan * static_cast<PxReal>(i), kAnchorHeight, 0.0f));
				PxArticulationLink* link = articulation->createLink(parent, pose);

				PxShape* shape = physics->createShape(
					PxBoxGeometry(kLinkSpan * 0.5f, 0.15f, 0.15f), *material, true);
				link->attachShape(*shape);
				shape->release();
				PxRigidBodyExt::updateMassAndInertia(*link, 10.0f);

				if (parent != NULL)
				{
					PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
					joint->setJointType(PxArticulationJointType::eREVOLUTE);
					joint->setParentPose(PxTransform(PxVec3(kLinkSpan * 0.5f, 0.0f, 0.0f)));
					joint->setChildPose(PxTransform(PxVec3(-kLinkSpan * 0.5f, 0.0f, 0.0f)));
					joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
				}

				links.push_back(link);
				parent = link;
			}

			material->release();
			PxwWorldRegister(world, 10u, articulation, PxwHandleKind::eARTICULATION);
			PxwWorldCommitPending(world);
		}

		void Destroy()
		{
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
				articulation = NULL;
				links.clear();
			}
		}

		PxU64 Hash() { return PxwWorldHashState(world); }

		std::vector<PxU8> Capture()
		{
			std::vector<PxU8> buffer(PxwWorldStateSize(world));
			PxU64 hash = 0;
			const PxU32 written = PxwWorldCaptureState(world, buffer.data(), static_cast<PxU32>(buffer.size()), &hash);
			buffer.resize(written);
			return buffer;
		}

		PxI32 Restore(const std::vector<PxU8>& buffer)
		{
			return PxwWorldRestoreState(world, buffer.data(), static_cast<PxU32>(buffer.size()));
		}

		// Drives the tip, which is the most sensitive place to push a chain: an error
		// anywhere in the joint state shows up amplified at the far end.
		void ApplyInput(int tickIndex)
		{
			if (links.empty())
			{
				return;
			}
			const float wobble = 0.35f * static_cast<float>((tickIndex % 7) - 3);
			links.back()->addForce(PxVec3(0.0f, wobble, wobble * 0.5f), PxForceMode::eACCELERATION);
		}
	};

	// The same snapshot-in, snapshot-out tick as SimRunner, over an articulation.
	struct ArticulationRunner
	{
		ArticulationWorld world;
		std::vector<PxU8> snapshot;

		void Build(int linkCount = kChainLinks, bool withGround = false)
		{
			world.Build(linkCount, withGround);
			snapshot = world.Capture();
		}

		void Destroy() { world.Destroy(); }

		void Rewind(const std::vector<PxU8>& to) { snapshot = to; }

		void Tick(int tickIndex)
		{
			world.Restore(snapshot);
			world.ApplyInput(tickIndex);
			PxwWorldStep(world.world, kDt);
			snapshot = world.Capture();
		}

		PxU64 SnapshotHash() const { return PxwHashBuffer(snapshot.data(), static_cast<PxU32>(snapshot.size())); }
	};

	void TestArticulationBaselineDeterminism()
	{
		std::printf("TestArticulationBaselineDeterminism [%s]\n", SolverName());

		ArticulationRunner a, b;
		a.Build();
		b.Build();

		bool identical = true;
		for (int tick = 0; tick < 240; ++tick)
		{
			a.Tick(tick);
			b.Tick(tick);
			if (a.SnapshotHash() != b.SnapshotHash())
			{
				std::printf("        diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}
		Check(identical, "two identically built articulation worlds stay bit-identical for 240 ticks ["
			+ std::string(SolverName()) + "]");

		a.Destroy();
		b.Destroy();
	}

	// Restoring a captured state and capturing again has to give the same bytes back,
	// or the tick function is not a function: every rollback would nudge the state
	// even when it replays the same inputs. The rigid-body version of this test found
	// that a pose round trip is only a fixed point after one cycle, so this checks the
	// same shape -- first capture may differ, second and third must agree.
	void TestArticulationRestoreRoundTrip()
	{
		std::printf("TestArticulationRestoreRoundTrip [%s]\n", SolverName());

		ArticulationRunner a;
		a.Build();
		for (int tick = 0; tick < 60; ++tick)
		{
			a.Tick(tick);
		}

		const std::vector<PxU8> first = a.world.Capture();
		a.world.Restore(first);
		const std::vector<PxU8> second = a.world.Capture();
		a.world.Restore(second);
		const std::vector<PxU8> third = a.world.Capture();

		const bool immediate = first.size() == second.size() &&
			std::memcmp(first.data(), second.data(), first.size()) == 0;
		const bool settled = second.size() == third.size() &&
			std::memcmp(second.data(), third.data(), second.size()) == 0;

		Observe(immediate, "an articulation capture is its own fixed point immediately ["
			+ std::string(SolverName()) + "]");
		Check(settled, "an articulation capture is a fixed point after one round trip ["
			+ std::string(SolverName()) + "]");

		a.Destroy();
	}

	// What the shipping framework actually relies on. Every peer runs the same fixed
	// prediction horizon, so every peer rewinds by the same amount on the same tick,
	// and the only question is whether replaying a tick from its own snapshot
	// reproduces it. If this fails, articulations cannot be rolled back at all.
	void TestArticulationFixedDepthRollback(int depth, bool withGround, const char* label)
	{
		std::printf("TestArticulationFixedDepthRollback [%s, %s]\n", label, SolverName());

		const int warmup = 30;
		const int frames = 300;
		const int historyDepth = 32;

		ArticulationRunner straight, rewinding;
		straight.Build(kChainLinks, withGround);
		rewinding.Build(kChainLinks, withGround);

		std::vector<std::vector<PxU8> > history(historyDepth);

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			straight.Tick(tick);
			rewinding.Tick(tick);
			history[tick % historyDepth] = rewinding.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;

		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			straight.Tick(tick);

			const int from = tick - depth;
			rewinding.Rewind(history[from % historyDepth]);
			for (int t = from + 1; t <= tick; ++t)
			{
				rewinding.Tick(t);
				history[t % historyDepth] = rewinding.snapshot;
			}

			if (straight.snapshot.size() != rewinding.snapshot.size() ||
				std::memcmp(straight.snapshot.data(), rewinding.snapshot.data(), straight.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
			}
		}

		std::printf("        %d frames rewinding %d ticks every frame: %s\n",
			matched ? frames : divergedAt, depth, matched ? "still identical" : "diverged");
		Check(matched, "an articulation replays a fixed rewind depth exactly ["
			+ std::string(label) + ", " + SolverName() + "]");

		straight.Destroy();
		rewinding.Destroy();
	}

	// The phase 1 question, asked of articulations. An adaptive prediction horizon
	// means peers rewind by whatever their own latency demands, so their rewind depths
	// differ every frame. Boxes only survive that under PGS, and only when no contact
	// chain runs deeper than eight bodies. A chain of jointed links is a contact chain
	// by another name, so whether the same limit applies is the thing to find out.
	void TestArticulationVariableDepthRollback(bool withGround, const char* label, int frames)
	{
		std::printf("TestArticulationVariableDepthRollback [%s, %s]\n", label, SolverName());

		const int warmup = 30;
		const int historyDepth = 32;

		ArticulationRunner peerA, peerB;
		peerA.Build(kChainLinks, withGround);
		peerB.Build(kChainLinks, withGround);

		std::vector<std::vector<PxU8> > historyA(historyDepth);
		std::vector<std::vector<PxU8> > historyB(historyDepth);

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			peerA.Tick(tick);
			peerB.Tick(tick);
			historyA[tick % historyDepth] = peerA.snapshot;
			historyB[tick % historyDepth] = peerB.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;

		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 7;
			const int depthB = 1 + (frame * 5) % 17;

			const int fromA = tick - depthA;
			peerA.Rewind(historyA[fromA % historyDepth]);
			for (int t = fromA + 1; t <= tick; ++t)
			{
				peerA.Tick(t);
				historyA[t % historyDepth] = peerA.snapshot;
			}

			const int fromB = tick - depthB;
			peerB.Rewind(historyB[fromB % historyDepth]);
			for (int t = fromB + 1; t <= tick; ++t)
			{
				peerB.Tick(t);
				historyB[t % historyDepth] = peerB.snapshot;
			}

			if (peerA.snapshot.size() != peerB.snapshot.size() ||
				std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
			}
		}

		std::printf("        %d frames of differing rollback depth: %s\n",
			matched ? frames : divergedAt, matched ? "still identical" : "diverged");
		Observe(matched, "an articulation survives peers rewinding by different depths ["
			+ std::string(label) + ", " + SolverName() + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	// ---------------------------------------------------------------------------
	// Driven articulation: the basic_articulation sample's actual configuration.
	//
	// The chain tests above are passive -- joints are eFREE and pushed with an
	// external force. That leaves the joint DRIVE constraint (stiffness/damping
	// position servo) completely unexercised, and the sample reports a driven
	// pendulum that spins up under rollback even with an effectively zero target.
	//
	// This reproduces that exact setup in isolation: a fixed base, a driven upper
	// link (Force drive, matching the sample's stiffness/damping), and a passive
	// lower link, then compares a warm run (step continuously) against a cold run
	// (restore its own snapshot before every step, the framework discipline). The
	// drive target is held at zero throughout, so the only excitation is gravity;
	// a transparent cold step settles exactly like the warm one, and a
	// non-transparent one pumps energy into the joint and the tip speed diverges.

	struct DrivenPendulumWorld
	{
		PxwWorld* world;
		PxArticulationReducedCoordinate* articulation;
		PxArticulationLink* base;
		PxArticulationLink* upper;
		PxArticulationLink* lower;
		PxArticulationJointReducedCoordinate* drivenJoint;
		PxArticulationCache* cache;

		DrivenPendulumWorld()
			: world(NULL), articulation(NULL), base(NULL), upper(NULL), lower(NULL),
			  drivenJoint(NULL), cache(NULL) {}

		static PxArticulationLink* AddLink(PxArticulationReducedCoordinate* art, PxArticulationLink* parent,
			PxPhysics* physics, PxMaterial* material, const PxTransform& pose, const PxVec3& halfExtents, PxReal density)
		{
			PxArticulationLink* link = art->createLink(parent, pose);
			PxShape* shape = physics->createShape(PxBoxGeometry(halfExtents), *material, true);
			link->attachShape(*shape);
			shape->release();
			PxRigidBodyExt::updateMassAndInertia(*link, density);
			return link;
		}

		void Build(PxReal stiffness, PxReal damping, PxReal maxForce)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);
			PxwWorldSetSleepParams(world, 0.0f, 0.0f, 0u);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.6f, 0.5f, 0.1f);

			articulation = physics->createArticulationReducedCoordinate();
			articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
			articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
			articulation->setSolverIterationCounts(8, 2);

			const PxReal anchorY = 3.0f;
			base  = AddLink(articulation, NULL,  physics, material, PxTransform(PxVec3(0.0f, anchorY, 0.0f)),        PxVec3(0.15f, 0.15f, 0.15f), 1000.0f);
			upper = AddLink(articulation, base,  physics, material, PxTransform(PxVec3(0.0f, anchorY - 0.6f, 0.0f)), PxVec3(0.075f, 0.45f, 0.075f), 1000.0f);
			lower = AddLink(articulation, upper, physics, material, PxTransform(PxVec3(0.0f, anchorY - 1.45f, 0.0f)),PxVec3(0.06f, 0.40f, 0.06f), 1000.0f);

			// Upper joint: hinge about swing Z, driven by a Force position drive.
			drivenJoint = upper->getInboundJoint();
			drivenJoint->setJointType(PxArticulationJointType::eSPHERICAL);
			drivenJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
			drivenJoint->setParentPose(PxTransform(PxVec3(0.0f, -0.15f, 0.0f)));
			drivenJoint->setChildPose(PxTransform(PxVec3(0.0f, 0.45f, 0.0f)));
			drivenJoint->setDriveParams(PxArticulationAxis::eSWING2,
				PxArticulationDrive(stiffness, damping, maxForce, PxArticulationDriveType::eFORCE));
			drivenJoint->setDriveTarget(PxArticulationAxis::eSWING2, 0.0f, false);

			// Lower joint: passive hinge about swing Z.
			PxArticulationJointReducedCoordinate* lowerJoint = lower->getInboundJoint();
			lowerJoint->setJointType(PxArticulationJointType::eSPHERICAL);
			lowerJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
			lowerJoint->setParentPose(PxTransform(PxVec3(0.0f, -0.45f, 0.0f)));
			lowerJoint->setChildPose(PxTransform(PxVec3(0.0f, 0.40f, 0.0f)));

			material->release();

			PxwWorldRegister(world, 10u, articulation, PxwHandleKind::eARTICULATION);
			PxwWorldCommitPending(world);

			cache = articulation->createCache();
		}

		void Destroy()
		{
			if (cache != NULL)
			{
				cache->release();
				cache = NULL;
			}
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
				articulation = NULL;
			}
		}

		void SetTarget(PxReal target)
		{
			drivenJoint->setDriveTarget(PxArticulationAxis::eSWING2, target, false);
		}

		void Step() { PxwWorldStep(world, kDt); }

		std::vector<PxU8> Capture()
		{
			std::vector<PxU8> buffer(PxwWorldStateSize(world));
			PxU64 hash = 0;
			const PxU32 written = PxwWorldCaptureState(world, buffer.data(), static_cast<PxU32>(buffer.size()), &hash);
			buffer.resize(written);
			return buffer;
		}

		void Restore(const std::vector<PxU8>& buffer)
		{
			PxwWorldRestoreState(world, buffer.data(), static_cast<PxU32>(buffer.size()));
		}

		// Total joint speed magnitude across both dofs, read straight from the
		// articulation -- the quantity that grows when a cold step injects energy.
		PxReal JointSpeed()
		{
			articulation->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
			const PxU32 dofs = articulation->getDofs();
			PxReal sum = 0.0f;
			for (PxU32 i = 0; i < dofs; ++i)
			{
				sum += PxAbs(cache->jointVelocity[i]);
			}
			return sum;
		}
	};

	void MeasureDrivenColdStep(PxReal stiffness, PxReal damping, const char* label)
	{
		const PxReal maxForce  = 1.0e6f;
		const int    steps     = 300;

		DrivenPendulumWorld warm, cold;
		warm.Build(stiffness, damping, maxForce);
		cold.Build(stiffness, damping, maxForce);

		// Displace the driven joint off its rest angle so the drive and gravity both do
		// work from the first step -- a pendulum hanging dead straight sees zero torque
		// about the hinge and never moves, which tests nothing.
		{
			const PxReal startAngle = 0.5f;
			warm.cache->jointPosition[0] = startAngle;
			warm.cache->jointPosition[1] = 0.0f;
			warm.articulation->applyCache(*warm.cache, PxArticulationCacheFlag::ePOSITION);
			cold.cache->jointPosition[0] = startAngle;
			cold.cache->jointPosition[1] = 0.0f;
			cold.articulation->applyCache(*cold.cache, PxArticulationCacheFlag::ePOSITION);
		}

		std::vector<PxU8> coldSnapshot = cold.Capture();

		PxReal warmPeak = 0.0f;
		PxReal coldPeak = 0.0f;
		PxReal maxDelta = 0.0f;

		for (int t = 0; t < steps; ++t)
		{
			// The sample drives target = amplitude * sin(t * 0.05); re-applied every tick
			// on the live and every replayed tick alike, so warm and cold see the same
			// command sequence and any divergence is the cold step's, not the input's.
			const PxReal target = 0.5f * PxSin(static_cast<PxReal>(t) * 0.05f);

			// Warm: continuous simulation.
			warm.SetTarget(target);
			warm.Step();
			const PxReal warmSpeed = warm.JointSpeed();

			// Cold: restore own snapshot, then step -- the framework's confirmed-timeline
			// discipline.
			cold.Restore(coldSnapshot);
			cold.SetTarget(target);
			cold.Step();
			coldSnapshot = cold.Capture();
			const PxReal coldSpeed = cold.JointSpeed();

			warmPeak = PxMax(warmPeak, warmSpeed);
			coldPeak = PxMax(coldPeak, coldSpeed);
			maxDelta = PxMax(maxDelta, PxAbs(coldSpeed - warmSpeed));
		}

		std::printf("        %-22s warm peak %.6f, cold peak %.6f, max |cold-warm| %.6f\n",
			label, warmPeak, coldPeak, maxDelta);

		// The cold timeline is the framework's confirmed timeline, so it has to track a warm
		// continuous run. Tolerance is generous because gravity plus a stiff servo is mildly
		// chaotic and float order-of-operations differs slightly between the two paths; the
		// bug this guards against multiplied the tip speed several-fold, far outside this.
		Check(maxDelta <= 0.05f * warmPeak + 1e-3f,
			"a driven pendulum's cold step tracks the warm run [" + std::string(label)
			+ ", " + SolverName() + "]");

		warm.Destroy();
		cold.Destroy();
	}

	// Does a capture/restore round trip preserve joint velocity, with no step in
	// between? If the cold pendulum freezes, this is where to look.
	void DiagnoseArticulationVelocityRoundTrip()
	{
		DrivenPendulumWorld w;
		w.Build(0.0f, 0.0f, 1.0e6f);

		// Displace and run a few steps so there is a real joint velocity to preserve.
		w.cache->jointPosition[0] = 0.5f;
		w.cache->jointPosition[1] = 0.0f;
		w.articulation->applyCache(*w.cache, PxArticulationCacheFlag::ePOSITION);
		for (int i = 0; i < 5; ++i) { w.Step(); }

		const PxReal before = w.JointSpeed();
		std::vector<PxU8> snap = w.Capture();
		w.Restore(snap);
		const PxReal after = w.JointSpeed();

		std::printf("        velocity round trip (no step): before %.6f, after %.6f\n", before, after);
		Observe(PxAbs(before - after) <= 1e-4f,
			"capture/restore preserves articulation joint velocity ["
			+ std::string(SolverName()) + "]");

		w.Destroy();
	}

	void TestDrivenArticulationColdStepTransparency()
	{
		std::printf("TestDrivenArticulationColdStepTransparency [%s]\n", SolverName());

		DiagnoseArticulationVelocityRoundTrip();
		MeasureDrivenColdStep(0.0f,    0.0f,   "passive (k=0)");
		MeasureDrivenColdStep(150.0f,  12.0f,  "medium (k=150)");
		MeasureDrivenColdStep(1500.0f, 120.0f, "sample (k=1500)");
	}

	// Runs the articulation set under one solver.
	void RunArticulationTests(PxSolverType::Enum solver)
	{
		gSolverType = solver;

		TestArticulationBaselineDeterminism();
		TestArticulationRestoreRoundTrip();
		TestArticulationFixedDepthRollback(4, false, "free-swinging chain");
		TestArticulationFixedDepthRollback(4, true, "chain resting on ground");
		TestArticulationVariableDepthRollback(false, "free-swinging chain", 600);
		TestArticulationVariableDepthRollback(true, "chain resting on ground", 600);
		TestDrivenArticulationColdStepTransparency();

		gSolverType = PxSolverType::eTGS;
	}

	// ---------------------------------------------------------------------------
	// Convex core narrowphase soak
	//
	// Convex core shapes are solved by GJK against a support function rather than by the
	// polygonal paths every other geometry here uses, so they are a different narrowphase code
	// path with its own iteration and termination behaviour. Before wheels are allowed to depend
	// on it, that path has to be shown to be bit-reproducible under rollback: a shape that is
	// merely *nearly* reproducible would desync a session at some unpredictable later moment,
	// and the cause would be almost impossible to attribute back to the wheel geometry.
	//
	// The test drops cylinders onto a triangle mesh and spins them, which is the demanding case
	// for two reasons. Contact against a mesh is resolved per triangle, so a rolling cylinder
	// crosses triangle boundaries constantly and re-derives its contact set as it goes. And a
	// cylinder's contact patch is a line rather than a point or a facet, so which features GJK
	// settles on genuinely depends on the iteration.

	// A gently uneven triangle mesh floor. Deliberately not flat: a flat grid lets every contact
	// land on a coplanar pair of triangles, which is the easy case and not the one that matters.
	PxTriangleMesh* CreateUnevenMeshFloor()
	{
		const PxU32 kCells = 8;
		const PxReal kCellSize = 1.0f;
		const PxU32 kVertsPerSide = kCells + 1;

		std::vector<PxVec3> vertices;
		vertices.reserve(kVertsPerSide * kVertsPerSide);
		for (PxU32 z = 0; z < kVertsPerSide; ++z)
		{
			for (PxU32 x = 0; x < kVertsPerSide; ++x)
			{
				const PxReal fx = (static_cast<PxReal>(x) - kCells * 0.5f) * kCellSize;
				const PxReal fz = (static_cast<PxReal>(z) - kCells * 0.5f) * kCellSize;
				const PxReal fy = 0.03f * PxSin(fx * 0.9f) * PxCos(fz * 0.7f);
				vertices.push_back(PxVec3(fx, fy, fz));
			}
		}

		std::vector<PxU32> indices;
		indices.reserve(kCells * kCells * 6);
		for (PxU32 z = 0; z < kCells; ++z)
		{
			for (PxU32 x = 0; x < kCells; ++x)
			{
				const PxU32 v0 = z * kVertsPerSide + x;
				const PxU32 v1 = v0 + 1;
				const PxU32 v2 = v0 + kVertsPerSide;
				const PxU32 v3 = v2 + 1;
				indices.push_back(v0); indices.push_back(v2); indices.push_back(v1);
				indices.push_back(v1); indices.push_back(v2); indices.push_back(v3);
			}
		}

		return GetGlobalPhysXWrapper().CreateBV33TriangleMesh(
			static_cast<PxU32>(vertices.size()), vertices.data(),
			static_cast<PxU32>(indices.size() / 3), indices.data(),
			false, false, false, false, false, false);
	}

	struct ConvexCoreSoakWorld
	{
		PxwWorld* world;
		PxTriangleMesh* mesh;

		ConvexCoreSoakWorld() : world(NULL), mesh(NULL) {}

		// useCylinder selects the shape under test; false substitutes a box of the same
		// dimensions, which is the control. Whatever the mesh contact path does to
		// reproducibility, it does to both, so comparing them isolates the convex core.
		void Build(bool useCylinder)
		{
			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);
			// Sleeping would end the test early and hide any divergence after it.
			PxwWorldSetSleepParams(world, 0.0f, 0.0f, 0u);

			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(0.7f, 0.7f, 0.05f);

			mesh = CreateUnevenMeshFloor();
			{
				PxTriangleMeshGeometry meshGeom(mesh);
				PxShape* shape = physics->createShape(meshGeom, *material, true);
				PxRigidStatic* floor = physics->createRigidStatic(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
				floor->attachShape(*shape);
				shape->release();
				PxwWorldRegister(world, 1u, floor, PxwHandleKind::eRIGID_STATIC);
			}

			// Four cylinders: two dropped flat to settle and rest, two spun up to roll across
			// the mesh. Resting and rolling stress different things, so both are present.
			for (PxU32 i = 0; i < 4; ++i)
			{
				const bool rolling = (i >= 2);
				const PxReal x = -1.5f + static_cast<PxReal>(i) * 1.0f;

				const PxConvexCoreGeometry cylinder(PxConvexCore::Cylinder(0.3f, 0.35f), 0.0f);
				const PxBoxGeometry box(0.15f, 0.35f, 0.35f);
				PxShape* shape = useCylinder
					? physics->createShape(cylinder, *material, true)
					: physics->createShape(box, *material, true);
				// Rolls about the world X axis, which is already the cylinder core's own axis, so
				// the wheel-like orientation needs no rotation here.
				PxRigidDynamic* body = physics->createRigidDynamic(
					PxTransform(PxVec3(x, 0.9f + 0.1f * static_cast<PxReal>(i), rolling ? -2.0f : 1.0f)));
				body->attachShape(*shape);
				shape->release();

				PxwSetupDeterministicMass(body, 500.0f, -1.0f, false, NULL);
				PxwApplyDeterministicRigidDefaults(body, 8, 2);
				if (rolling)
				{
					body->setAngularVelocity(PxVec3(6.0f, 0.0f, 0.0f));
					body->setLinearVelocity(PxVec3(0.0f, 0.0f, 2.0f));
				}

				PxwWorldRegister(world, 100u + i, body, PxwHandleKind::eRIGID_DYNAMIC);
			}

			material->release();
			PxwWorldCommitPending(world);
		}

		void Destroy()
		{
			if (world != NULL)
			{
				PxwWorldDestroy(world);
				world = NULL;
			}
			if (mesh != NULL)
			{
				mesh->release();
				mesh = NULL;
			}
		}

		std::vector<PxU8> Capture()
		{
			std::vector<PxU8> buffer(PxwWorldStateSize(world));
			PxU64 hash = 0;
			const PxU32 written = PxwWorldCaptureState(world, buffer.data(), static_cast<PxU32>(buffer.size()), &hash);
			buffer.resize(written);
			return buffer;
		}
	};

	PxU64 SoakHash(ConvexCoreSoakWorld& world)
	{
		const std::vector<PxU8> snapshot = world.Capture();
		return PxwHashBuffer(snapshot.data(), static_cast<PxU32>(snapshot.size()));
	}

	// The requirement a networked session actually has: two peers running the same world through
	// the same rollback pattern must agree, tick for tick.
	bool SoakPeersAgree(bool useCylinder, int ticks)
	{
		ConvexCoreSoakWorld a, b;
		a.Build(useCylinder);
		b.Build(useCylinder);

		std::vector<PxU8> snapA = a.Capture();
		std::vector<PxU8> snapB = b.Capture();

		bool identical = true;
		for (int tick = 0; tick < ticks; ++tick)
		{
			PxwWorldRestoreState(a.world, snapA.data(), static_cast<PxU32>(snapA.size()));
			PxwWorldStep(a.world, kDt);
			snapA = a.Capture();

			PxwWorldRestoreState(b.world, snapB.data(), static_cast<PxU32>(snapB.size()));
			PxwWorldStep(b.world, kDt);
			snapB = b.Capture();

			if (PxwHashBuffer(snapA.data(), static_cast<PxU32>(snapA.size()))
				!= PxwHashBuffer(snapB.data(), static_cast<PxU32>(snapB.size())))
			{
				std::printf("        peers diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}

		a.Destroy();
		b.Destroy();
		return identical;
	}

	// Rollback transparency: a run that rewinds and replays has to land on exactly the state a
	// run that never rewound reached.
	//
	// Both sides hold the cold-step discipline the framework requires -- one restore before every
	// step, including the steps nobody rolled back. That is not a detail of the test, it is the
	// whole reason the property holds: a step after a restore narrowphases cold while a step
	// after another step warm-starts from PhysX's persistent contact manifolds, and those
	// manifolds are not in the snapshot because no public API exposes them. Restoring
	// unconditionally makes every step cold, which removes the asymmetry rather than fixing it.
	// `RollbackEngine` does exactly this, so an uninterrupted warm run is not a configuration
	// that occurs at runtime and is deliberately not what this compares against.
	bool SoakReplayMatchesUnrewoundRun(bool useCylinder, int ticks, int depth)
	{
		ConvexCoreSoakWorld reference, subject;
		reference.Build(useCylinder);
		subject.Build(useCylinder);

		// Reference: never rewinds, but still restores before every step.
		std::vector<std::vector<PxU8> > referenceSnapshots;
		std::vector<PxU64> referenceHashes;
		referenceSnapshots.push_back(reference.Capture());
		referenceHashes.push_back(SoakHash(reference));
		for (int tick = 1; tick <= ticks; ++tick)
		{
			const std::vector<PxU8>& previous = referenceSnapshots[static_cast<size_t>(tick) - 1];
			PxwWorldRestoreState(reference.world, previous.data(), static_cast<PxU32>(previous.size()));
			PxwWorldStep(reference.world, kDt);
			referenceSnapshots.push_back(reference.Capture());
			referenceHashes.push_back(SoakHash(reference));
		}

		// Subject: the same run, except that every `depth` ticks it throws away the window it
		// just simulated and resimulates it from its own snapshot, which is what a peer receiving
		// a late input does.
		std::vector<std::vector<PxU8> > snapshots;
		snapshots.push_back(subject.Capture());

		bool matched = true;
		for (int tick = 1; tick <= ticks && matched; ++tick)
		{
			const std::vector<PxU8>& previous = snapshots[static_cast<size_t>(tick) - 1];
			PxwWorldRestoreState(subject.world, previous.data(), static_cast<PxU32>(previous.size()));
			PxwWorldStep(subject.world, kDt);
			snapshots.push_back(subject.Capture());

			if (tick % depth == 0)
			{
				for (int replay = tick - depth + 1; replay <= tick; ++replay)
				{
					const std::vector<PxU8>& from = snapshots[static_cast<size_t>(replay) - 1];
					PxwWorldRestoreState(subject.world, from.data(), static_cast<PxU32>(from.size()));
					PxwWorldStep(subject.world, kDt);
					snapshots[static_cast<size_t>(replay)] = subject.Capture();
				}
			}

			if (SoakHash(subject) != referenceHashes[static_cast<size_t>(tick)])
			{
				std::printf("        replay diverged from the un-rewound run at tick %d\n", tick);
				matched = false;
			}
		}

		reference.Destroy();
		subject.Destroy();
		return matched;
	}

	void TestConvexCoreOnMeshIsReproducibleUnderRollback()
	{
		std::printf("TestConvexCoreOnMeshIsReproducibleUnderRollback [%s]\n", SolverName());

		const int kTicks = 600;
		const int kDepth = 8;
		const std::string suffix = " [" + std::string(SolverName()) + "]";

		Check(SoakPeersAgree(true, kTicks),
			"two peers running cylinders on a triangle mesh agree for " + std::to_string(kTicks)
			+ " rolled-back ticks" + suffix);

		// Bit-exact replay is a PGS property in this framework; TGS is characterised rather than
		// asserted, as it is everywhere else in the suite. The box is carried alongside so that a
		// TGS shortfall can be read as the solver rather than the geometry.
		const bool cylinderTransparent = SoakReplayMatchesUnrewoundRun(true, kTicks, kDepth);
		const bool boxTransparent = SoakReplayMatchesUnrewoundRun(false, kTicks, kDepth);

		if (gSolverType == PxSolverType::ePGS)
		{
			Check(cylinderTransparent,
				"rewinding and replaying cylinders on a triangle mesh lands on the un-rewound state" + suffix);
			Check(boxTransparent,
				"the same holds for boxes, so the cylinder is not being held to a private standard" + suffix);
		}
		else
		{
			Observe(cylinderTransparent, "cylinder replay is bit-exact" + suffix);
			Observe(boxTransparent, "box replay is bit-exact" + suffix);
		}
	}

	// ---------------------------------------------------------------------------
	// Vehicles
	//
	// A vehicle carries integrator state a plain rigid body does not: each wheel has
	// a rotation angle and speed, a suspension jounce, and a sticky-tire timer, and an
	// engine-drive vehicle adds engine, gearbox, autobox and clutch state. Before this
	// suite a vehicle snapshot was just its chassis pose, so all of that was silently
	// reset on every restore. These tests build a four-wheeled vehicle on the ground,
	// drive it, and ask whether a rollback reproduces it -- which it can only do if the
	// snapshot carries that state. The "at rest" variant is the sticky-tire case: the
	// low-speed accumulator only grows once the vehicle has stopped, so a snapshot that
	// drops it diverges precisely there.

	// A four-wheeled vehicle built directly on the pxw classes so the test can set the
	// per-wheel parameters without going through the descriptor-heavy C API.
	PxwVehicle* BuildTestVehicle(PxScene* scene, PxPhysics* physics, PxMaterial* material, bool engineDrive,
		const PxwVehicleWheelShapeDesc* wheelShape = NULL, const PxwVehicleFrameDesc* frame = NULL)
	{
		PxwVehicleChassisDesc chassis;
		chassis.mass = 1500.0f;
		chassis.moi = PxVec3(3625.0f, 3625.0f, 3625.0f);
		chassis.cmassLocalPose = PxwTransformData(PxTransform(PxIdentity));
		chassis.boxHalfExtents = PxVec3(0.9f, 0.35f, 2.2f);
		chassis.shapeLocalPose = PxwTransformData(PxTransform(PxIdentity));

		const PxwVehicleDriveMode::Enum mode =
			engineDrive ? PxwVehicleDriveMode::eENGINE : PxwVehicleDriveMode::eDIRECT;
		// NULL chassis geometry: Finalize builds the fallback box from the descriptor.
		PxwVehicle* v = new PxwVehicle(scene, mode, chassis, NULL, material);

		// A non-default frame turns the wheel axis off the convex-core cylinder's local +X, so
		// the cylinder wheel shapes pick up a non-identity axis-alignment construction pose.
		if (frame != NULL)
			v->SetFrame(*frame);

		int nbWheelsPerAxle[2] = { 2, 2 };
		int wheelIds[4] = { 0, 1, 2, 3 };
		v->SetAxleDescription(2, nbWheelsPerAxle, wheelIds);

		const float wheelRadius = 0.35f;
		const PxVec3 wheelXZ[4] = {
			PxVec3(0.8f, 0.0f, 1.4f),
			PxVec3(-0.8f, 0.0f, 1.4f),
			PxVec3(0.8f, 0.0f, -1.4f),
			PxVec3(-0.8f, 0.0f, -1.4f)
		};

		for (int i = 0; i < 4; ++i)
		{
			PxwVehicleWheelDesc w;
			w.radius = wheelRadius;
			w.halfWidth = 0.15f;
			w.mass = 20.0f;
			w.moi = 0.5f * 20.0f * wheelRadius * wheelRadius;
			w.dampingRate = 0.25f;
			v->SetWheel(i, w);

			// Left alone by default, so the default path is exercised as the shipped
			// configuration rather than as an explicit request for the same thing.
			if (wheelShape != NULL)
			{
				v->SetWheelShape(i, *wheelShape, NULL);
			}

			PxwVehicleSuspensionDesc s;
			s.suspensionAttachment = PxwTransformData(PxTransform(PxVec3(wheelXZ[i].x, 0.3f, wheelXZ[i].z)));
			s.travelDir = PxVec3(0.0f, -1.0f, 0.0f);
			s.travelDist = 0.25f;
			s.wheelAttachment = PxwTransformData(PxTransform(PxIdentity));
			s.stiffness = 35000.0f;
			s.damping = 4500.0f;
			s.sprungMass = 375.0f;
			v->SetSuspension(i, s);

			PxwVehicleSuspensionComplianceDesc c;
			c.toeAngle = 0.0f;
			c.camberAngle = 0.0f;
			c.suspForceAppPoint = PxVec3(0.0f, 0.0f, 0.0f);
			c.tireForceAppPoint = PxVec3(0.0f, 0.0f, 0.0f);
			v->SetSuspensionCompliance(i, c);

			PxwVehicleTireDesc t;
			std::memset(&t, 0, sizeof(t));
			t.latStiffX = 0.01f;
			t.latStiffY = 18.0f;
			t.longStiff = 5000.0f;
			t.camberStiff = 0.0f;
			t.restLoad = 3500.0f;
			t.frictionVsSlip[0][0] = 0.0f; t.frictionVsSlip[0][1] = 1.0f;
			t.frictionVsSlip[1][0] = 0.1f; t.frictionVsSlip[1][1] = 1.0f;
			t.frictionVsSlip[2][0] = 1.0f; t.frictionVsSlip[2][1] = 1.0f;
			t.loadFilter[0][0] = 0.0f; t.loadFilter[0][1] = 0.23f;
			t.loadFilter[1][0] = 3.0f; t.loadFilter[1][1] = 3.0f;
			v->SetTire(i, t);
		}

		if (engineDrive)
		{
			PxwVehicleDifferentialDesc diff;
			std::memset(&diff, 0, sizeof(diff));
			diff.type = PxwVehicleDifferentialType::eMULTIWHEEL;
			for (int i = 0; i < 4; ++i)
			{
				diff.torqueRatios[i] = 0.25f;
				diff.aveWheelSpeedRatios[i] = 0.25f;
			}
			v->SetDifferential(diff);
		}
		else
		{
			float mult[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
			v->SetDirectDriveThrottle(1000.0f, mult, 4);
		}

		PxCookingParams cooking(physics->getTolerancesScale());
		if (!v->Finalize(physics, cooking, material))
		{
			std::printf("        vehicle Finalize FAILED\n");
		}

		// Drop it in just above the ground so the suspension settles onto it.
		PxRigidBody* body = v->GetActor();
		if (body != NULL)
		{
			body->setGlobalPose(PxTransform(PxVec3(0.0f, 1.0f, 0.0f)));
		}

		return v;
	}

	struct VehicleWorld
	{
		PxwWorld* world;
		PxwVehicle* vehicle;
		bool engine;
		float throttle;

		VehicleWorld() : world(NULL), vehicle(NULL), engine(false), throttle(0.0f) {}

		void Build(bool engineDrive, float throttleCmd, const PxwVehicleWheelShapeDesc* wheelShape = NULL,
			const PxwVehicleFrameDesc* frame = NULL)
		{
			engine = engineDrive;
			throttle = throttleCmd;

			PxwSceneDesc desc = MakeDeterministicSceneDesc();
			world = PxwWorldCreate(&desc);
			PxwWorldSetSleepParams(world, 0.0f, 0.0f, 0u);

			PxScene* scene = PxwWorldGetScene(world);
			PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
			PxMaterial* material = physics->createMaterial(1.0f, 1.0f, 0.1f);

			{
				PxBoxGeometry groundGeom(60.0f, 1.0f, 60.0f);
				PxShape* shape = physics->createShape(groundGeom, *material, true);
				PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, -1.0f, 0.0f)));
				ground->attachShape(*shape);
				shape->release();
				PxwWorldRegister(world, 1u, ground, PxwHandleKind::eRIGID_STATIC);
			}

			vehicle = BuildTestVehicle(scene, physics, material, engineDrive, wheelShape, frame);
			PxwWorldRegister(world, 10u, vehicle, PxwHandleKind::eVEHICLE);
			PxwWorldCommitPending(world);

			material->release();
		}

		void Destroy()
		{
			if (world == NULL)
			{
				return;
			}
			if (vehicle != NULL)
			{
				// Managed-style teardown: unregister and commit so the vehicle leaves
				// the scene and the per-scene step list before it is deleted, then let
				// PxwWorldDestroy release the scene.
				PxwWorldUnregister(world, 10u);
				PxwWorldCommitPending(world);
				delete vehicle;
				vehicle = NULL;
			}
			PxwWorldDestroy(world);
			world = NULL;
		}

		void ApplyInput(int /*tickIndex*/)
		{
			if (vehicle == NULL)
			{
				return;
			}
			vehicle->SetCommands(0.0f, 0.0f, throttle, 0.0f);
			if (engine)
			{
				vehicle->SetTransmissionCommand(
					static_cast<int>(PxVehicleEngineDriveTransmissionCommandState::eAUTOMATIC_GEAR), 0.0f);
			}
			else
			{
				vehicle->SetTransmissionCommand(
					static_cast<int>(PxVehicleDirectDriveTransmissionCommandState::eFORWARD), 0.0f);
			}
		}

		std::vector<PxU8> Capture()
		{
			std::vector<PxU8> buffer(PxwWorldStateSize(world));
			PxU64 hash = 0;
			const PxU32 written = PxwWorldCaptureState(world, buffer.data(), static_cast<PxU32>(buffer.size()), &hash);
			buffer.resize(written);
			return buffer;
		}

		PxI32 Restore(const std::vector<PxU8>& buffer)
		{
			return PxwWorldRestoreState(world, buffer.data(), static_cast<PxU32>(buffer.size()));
		}
	};

	struct VehicleRunner
	{
		VehicleWorld world;
		std::vector<PxU8> snapshot;

		void Build(bool engineDrive, float throttleCmd)
		{
			world.Build(engineDrive, throttleCmd);
			snapshot = world.Capture();
		}

		void Destroy() { world.Destroy(); }
		void Rewind(const std::vector<PxU8>& to) { snapshot = to; }

		void Tick(int tickIndex)
		{
			world.Restore(snapshot);
			world.ApplyInput(tickIndex);
			PxwWorldStep(world.world, kDt);
			snapshot = world.Capture();
		}

		PxU64 SnapshotHash() const { return PxwHashBuffer(snapshot.data(), static_cast<PxU32>(snapshot.size())); }
	};

	const char* DriveName(bool engine) { return engine ? "engine drive" : "direct drive"; }

	void TestVehicleConstructionHashSurvivesWheelPoseUpdates(bool engine)
	{
		std::printf("TestVehicleConstructionHashSurvivesWheelPoseUpdates [%s, %s]\n",
			DriveName(engine), SolverName());

		VehicleRunner runner;
		runner.Build(engine, 1.0f);
		const PxU64 before = PxwWorldHashConstruction(runner.world.world);
		for (int tick = 0; tick < 20; ++tick)
			runner.Tick(tick);
		const PxU64 after = PxwWorldHashConstruction(runner.world.world);

		Check(before == after,
			"a vehicle construction hash excludes runtime wheel-shape poses ["
			+ std::string(DriveName(engine)) + ", " + SolverName() + "]");
		runner.Destroy();
	}

	// The plugin builds vehicle wheel shapes itself instead of letting
	// PxVehiclePhysXActorCreate do it, because PxShape::setGeometry cannot change a shape's
	// geometry type, so per-wheel geometry has to be decided when the shape is created. That
	// makes the default path a regression risk: every existing vehicle has to keep the shape it
	// had, or its construction hash moves and peers running different plugin versions cannot
	// agree even though nobody asked for anything new.
	void TestVehicleDefaultWheelShapesAreUnchanged(bool engine)
	{
		std::printf("TestVehicleDefaultWheelShapesAreUnchanged [%s, %s]\n",
			DriveName(engine), SolverName());

		const std::string suffix = " [" + std::string(DriveName(engine)) + ", " + SolverName() + "]";

		PxwVehicleWheelShapeDesc explicitDefault;
		std::memset(&explicitDefault, 0, sizeof(explicitDefault));
		explicitDefault.geometryMode = PxwVehicleWheelGeometryMode::eCOOKED_PRISM;

		VehicleWorld untouched, asked;
		untouched.Build(engine, 0.0f);
		asked.Build(engine, 0.0f, &explicitDefault);

		Check(PxwWorldHashConstruction(untouched.world) == PxwWorldHashConstruction(asked.world),
			"a vehicle left alone is built exactly like one that asks for the defaults" + suffix);

		// The default really is the cooked prism, not a cylinder that happens to be consistent.
		PxRigidBody* body = untouched.vehicle->GetActor();
		bool everyWheelIsAConvexMesh = (body != NULL);
		if (body != NULL)
		{
			const PxU32 shapeCount = body->getNbShapes();
			// One chassis shape plus one per wheel.
			everyWheelIsAConvexMesh = (shapeCount == 5);
			for (PxU32 i = 1; i < shapeCount; ++i)
			{
				PxShape* shape = NULL;
				body->getShapes(&shape, 1, i);
				if (shape == NULL || shape->getGeometry().getType() != PxGeometryType::eCONVEXMESH)
				{
					everyWheelIsAConvexMesh = false;
				}
				// Wheels are raycast driven, so by default they must neither simulate nor be
				// visible to scene queries.
				else if (shape->getFlags() & (PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE))
				{
					everyWheelIsAConvexMesh = false;
				}
			}
		}
		Check(everyWheelIsAConvexMesh,
			"a default vehicle's wheels are still non-colliding cooked convex hulls" + suffix);

		// And the overrides have to actually reach the shapes, or the setting is decoration.
		PxwVehicleWheelShapeDesc cylinderMode = explicitDefault;
		cylinderMode.geometryMode = PxwVehicleWheelGeometryMode::eCYLINDER;

		PxwVehicleWheelShapeDesc queryable = explicitDefault;
		queryable.sceneQueryShape = 1;

		PxwVehicleWheelShapeDesc grouped = explicitDefault;
		grouped.simFilterData[0] = 3u;

		VehicleWorld cylinder, sceneQuery, filtered;
		cylinder.Build(engine, 0.0f, &cylinderMode);
		sceneQuery.Build(engine, 0.0f, &queryable);
		filtered.Build(engine, 0.0f, &grouped);

		const PxU64 baseline = PxwWorldHashConstruction(untouched.world);
		Check(PxwWorldHashConstruction(cylinder.world) != baseline,
			"choosing cylinder wheels changes the construction hash" + suffix);
		Check(PxwWorldHashConstruction(sceneQuery.world) != baseline,
			"making wheels visible to scene queries changes the construction hash" + suffix);
		Check(PxwWorldHashConstruction(filtered.world) != baseline,
			"giving wheels a collision group changes the construction hash" + suffix);

		untouched.Destroy();
		asked.Destroy();
		cylinder.Destroy();
		sceneQuery.Destroy();
		filtered.Destroy();
	}

	// A cylinder wheel is only worth having if it simulates as reproducibly as the hull it
	// replaces: convex core narrowphase is a different code path, and a vehicle that rolls
	// deterministically on cooked hulls but not on cylinders would be worse than no option.
	void TestVehicleCylinderWheelsAreDeterministic(bool engine)
	{
		std::printf("TestVehicleCylinderWheelsAreDeterministic [%s, %s]\n",
			DriveName(engine), SolverName());

		PxwVehicleWheelShapeDesc cylinderMode;
		std::memset(&cylinderMode, 0, sizeof(cylinderMode));
		cylinderMode.geometryMode = PxwVehicleWheelGeometryMode::eCYLINDER;
		cylinderMode.simulationShape = 1;
		// Group 1 for the wheels, and the ground is left in group 0, so the road stays the
		// tire model's job while the wheels are still solid against everything else. Without
		// this the road would be resolved twice.
		cylinderMode.simFilterData[0] = 1u;
		SetGroupCollisionFlag(1, 0, false);

		VehicleRunner a, b;
		a.world.Build(engine, 1.0f, &cylinderMode);
		a.snapshot = a.world.Capture();
		b.world.Build(engine, 1.0f, &cylinderMode);
		b.snapshot = b.world.Capture();

		bool identical = true;
		for (int tick = 0; tick < 200; ++tick)
		{
			a.Tick(tick);
			b.Tick(tick);
			if (a.SnapshotHash() != b.SnapshotHash())
			{
				std::printf("        diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}
		Check(identical,
			"two vehicles on simulating cylinder wheels stay bit-identical for 200 ticks ["
			+ std::string(DriveName(engine)) + ", " + SolverName() + "]");

		a.Destroy();
		b.Destroy();
		ResetGroupCollisionFlags();
	}

	// Reads the diagnostic per-entry construction sub-hash for one registered entry, or 0 if
	// the entry is not reported for that part.
	PxU64 ReadEntryConstructionPart(PxwWorld* world, PxU32 stableId, PxU32 part)
	{
		PxwEntryHash records[8];
		const PxU32 count = PxwWorldHashConstructionPartPerEntry(world, records, 8, part);
		for (PxU32 i = 0; i < count; ++i)
		{
			if (records[i].stableId == stableId)
				return records[i].hash;
		}
		return 0u;
	}

	// The construction hash has to see a wheel's cylinder-axis alignment, because a convex-core
	// cylinder is frame-independent geometry: two peers whose vehicle frames disagree build the
	// same wheel geometry and would hash equal on it alone, yet their wheels point different ways.
	// That difference lives in the construction-time wheel-shape local pose (physxWheelShapeLocalPoses),
	// which is the axis-alignment rotation the vehicle composes with the runtime PxShape pose it
	// rewrites every step. The hash must fold in that construction pose while still ignoring the
	// runtime pose, and must not move a rig whose pose is identity (the shipped Unity frame, where
	// the wheel axis already is the cylinder's local +X). Part 12 exposes this contribution alone.
	void TestVehicleConstructionHashIncludesWheelShapeLocalPoses(bool engine)
	{
		std::printf("TestVehicleConstructionHashIncludesWheelShapeLocalPoses [%s, %s]\n",
			DriveName(engine), SolverName());

		const std::string suffix = " [" + std::string(DriveName(engine)) + ", " + SolverName() + "]";
		const PxU32 vehicleId = 10u;

		PxwVehicleWheelShapeDesc cylinderMode;
		std::memset(&cylinderMode, 0, sizeof(cylinderMode));
		cylinderMode.geometryMode = PxwVehicleWheelGeometryMode::eCYLINDER;

		// A legal right-handed frame whose lateral axis is +Y rather than the shipped +X. The
		// cylinder core runs along its own local +X, so this frame bakes a real 90-degree
		// axis-alignment rotation into every wheel's construction pose.
		PxwVehicleFrameDesc rotatedFrame;
		rotatedFrame.lngAxis = PxwVehicleAxis::ePosX;
		rotatedFrame.latAxis = PxwVehicleAxis::ePosY;
		rotatedFrame.vrtAxis = PxwVehicleAxis::ePosZ;
		rotatedFrame.scale = 1.0f;

		VehicleWorld prism, cylDefault, cylDefaultTwin, cylRotated;
		prism.Build(engine, 0.0f);
		cylDefault.Build(engine, 0.0f, &cylinderMode);
		cylDefaultTwin.Build(engine, 0.0f, &cylinderMode);
		cylRotated.Build(engine, 0.0f, &cylinderMode, &rotatedFrame);

		const PxU64 prism12 = ReadEntryConstructionPart(prism.world, vehicleId, 12u);
		const PxU64 cylDef12 = ReadEntryConstructionPart(cylDefault.world, vehicleId, 12u);
		const PxU64 cylDefTwin12 = ReadEntryConstructionPart(cylDefaultTwin.world, vehicleId, 12u);
		const PxU64 cylRot12 = ReadEntryConstructionPart(cylRotated.world, vehicleId, 12u);

		// Backward compatibility: on the shipped frame the axis pose is identity, so the sparse
		// contract adds nothing and part 12 matches the cooked-prism default it always did.
		Check(cylDef12 == prism12,
			"a default-frame cylinder keeps the prism default's empty wheel-pose hash" + suffix);
		Check(cylDef12 == cylDefTwin12,
			"part 12 is deterministic across identical cylinder builds" + suffix);

		// The point of the change: a frame that rotates the cylinder axis off the wheel axis bakes
		// a non-identity construction pose, and part 12 has to move to catch it.
		Check(cylRot12 != cylDef12,
			"a rotated frame's cylinder axis pose changes part 12" + suffix);

		// The aggregate hash must carry the same separation: two cylinder vehicles that agree on
		// wheel geometry but not on frame are not built the same way.
		const PxU64 cylDefAgg = PxwWorldHashConstruction(cylDefault.world);
		const PxU64 cylRotAgg = PxwWorldHashConstruction(cylRotated.world);
		Check(cylDefAgg != cylRotAgg,
			"the aggregate construction hash separates cylinder frames" + suffix);

		// The point of the sparse contract: the cylinder wheel's runtime PxShape pose moves every
		// step, but the construction pose it is composed with does not. Neither the aggregate hash
		// nor part 12 may move while the vehicle drives.
		VehicleRunner runner;
		runner.world.Build(engine, 1.0f, &cylinderMode);
		runner.snapshot = runner.world.Capture();
		const PxU64 aggBefore = PxwWorldHashConstruction(runner.world.world);
		const PxU64 part12Before = ReadEntryConstructionPart(runner.world.world, vehicleId, 12u);
		for (int tick = 0; tick < 20; ++tick)
			runner.Tick(tick);
		const PxU64 aggAfter = PxwWorldHashConstruction(runner.world.world);
		const PxU64 part12After = ReadEntryConstructionPart(runner.world.world, vehicleId, 12u);

		Check(aggBefore == aggAfter,
			"driving cylinder wheels does not move the construction hash" + suffix);
		Check(part12Before == part12After,
			"driving cylinder wheels does not move part 12" + suffix);

		prism.Destroy();
		cylDefault.Destroy();
		cylDefaultTwin.Destroy();
		cylRotated.Destroy();
		runner.Destroy();
	}

	void TestVehicleBaselineDeterminism(bool engine)
	{
		std::printf("TestVehicleBaselineDeterminism [%s, %s]\n", DriveName(engine), SolverName());

		VehicleRunner a, b;
		a.Build(engine, 1.0f);
		b.Build(engine, 1.0f);

		bool identical = true;
		for (int tick = 0; tick < 200; ++tick)
		{
			a.Tick(tick);
			b.Tick(tick);
			if (a.SnapshotHash() != b.SnapshotHash())
			{
				std::printf("        diverged at tick %d\n", tick);
				identical = false;
				break;
			}
		}
		Check(identical, "two identically built vehicle worlds stay bit-identical for 200 ticks ["
			+ std::string(DriveName(engine)) + ", " + SolverName() + "]");

		a.Destroy();
		b.Destroy();
	}

	// Capturing, restoring and capturing again has to return the same bytes, or a
	// rollback that replays the same inputs would still nudge the state. Same shape as
	// the rigid and articulation versions: the first capture may differ, the settled
	// one must not.
	void TestVehicleRestoreRoundTrip(bool engine)
	{
		std::printf("TestVehicleRestoreRoundTrip [%s, %s]\n", DriveName(engine), SolverName());

		VehicleRunner a;
		a.Build(engine, 1.0f);
		for (int tick = 0; tick < 60; ++tick)
		{
			a.Tick(tick);
		}

		const std::vector<PxU8> first = a.world.Capture();
		a.world.Restore(first);
		const std::vector<PxU8> second = a.world.Capture();
		a.world.Restore(second);
		const std::vector<PxU8> third = a.world.Capture();

		const bool settled = second.size() == third.size() &&
			std::memcmp(second.data(), third.data(), second.size()) == 0;
		Check(settled, "a vehicle capture is a fixed point after one round trip ["
			+ std::string(DriveName(engine)) + ", " + SolverName() + "]");

		a.Destroy();
	}

	// The shipping guarantee: a fixed prediction horizon rewinds every peer by the same
	// amount every frame, and replaying a tick from its own snapshot must reproduce it.
	// throttle 0 is the sticky-tire-at-rest case -- the vehicle settles and its low-speed
	// timer grows, so a snapshot that drops the timer diverges here and nowhere else.
	void TestVehicleFixedDepthRollback(bool engine, float throttleCmd, const char* label)
	{
		std::printf("TestVehicleFixedDepthRollback [%s, %s, %s]\n", label, DriveName(engine), SolverName());

		const int warmup = 40;
		const int frames = 200;
		const int depth = 4;
		const int historyDepth = 32;

		VehicleRunner straight, rewinding;
		straight.Build(engine, throttleCmd);
		rewinding.Build(engine, throttleCmd);

		std::vector<std::vector<PxU8> > history(historyDepth);

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			straight.Tick(tick);
			rewinding.Tick(tick);
			history[tick % historyDepth] = rewinding.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;
		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			straight.Tick(tick);

			const int from = tick - depth;
			rewinding.Rewind(history[from % historyDepth]);
			for (int t = from + 1; t <= tick; ++t)
			{
				rewinding.Tick(t);
				history[t % historyDepth] = rewinding.snapshot;
			}

			if (straight.snapshot.size() != rewinding.snapshot.size() ||
				std::memcmp(straight.snapshot.data(), rewinding.snapshot.data(), straight.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
			}
		}

		std::printf("        %d frames rewinding %d ticks every frame: %s\n",
			matched ? frames : divergedAt, depth, matched ? "still identical" : "diverged");
		Check(matched, "a vehicle replays a fixed rewind depth exactly ["
			+ std::string(label) + ", " + DriveName(engine) + ", " + SolverName() + "]");

		straight.Destroy();
		rewinding.Destroy();
	}

	// The phase 1 question for vehicles: peers on an adaptive horizon rewind by
	// different depths every frame. Characterisation, like the articulation and box
	// versions -- a "no" here would confine vehicles to a fixed horizon, not break them.
	void TestVehicleVariableDepthRollback(bool engine, int frames)
	{
		std::printf("TestVehicleVariableDepthRollback [%s, %s]\n", DriveName(engine), SolverName());

		const int warmup = 40;
		const int historyDepth = 32;

		VehicleRunner peerA, peerB;
		peerA.Build(engine, 1.0f);
		peerB.Build(engine, 1.0f);

		std::vector<std::vector<PxU8> > historyA(historyDepth);
		std::vector<std::vector<PxU8> > historyB(historyDepth);

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			peerA.Tick(tick);
			peerB.Tick(tick);
			historyA[tick % historyDepth] = peerA.snapshot;
			historyB[tick % historyDepth] = peerB.snapshot;
		}

		bool matched = true;
		int divergedAt = -1;
		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 7;
			const int depthB = 1 + (frame * 5) % 17;

			const int fromA = tick - depthA;
			peerA.Rewind(historyA[fromA % historyDepth]);
			for (int t = fromA + 1; t <= tick; ++t)
			{
				peerA.Tick(t);
				historyA[t % historyDepth] = peerA.snapshot;
			}

			const int fromB = tick - depthB;
			peerB.Rewind(historyB[fromB % historyDepth]);
			for (int t = fromB + 1; t <= tick; ++t)
			{
				peerB.Tick(t);
				historyB[t % historyDepth] = peerB.snapshot;
			}

			if (peerA.snapshot.size() != peerB.snapshot.size() ||
				std::memcmp(peerA.snapshot.data(), peerB.snapshot.data(), peerA.snapshot.size()) != 0)
			{
				matched = false;
				divergedAt = frame;
			}
		}

		std::printf("        %d frames of differing rollback depth: %s\n",
			matched ? frames : divergedAt, matched ? "still identical" : "diverged");
		Observe(matched, "a vehicle survives peers rewinding by different depths ["
			+ std::string(DriveName(engine)) + ", " + SolverName() + "]");

		peerA.Destroy();
		peerB.Destroy();
	}

	void RunVehicleTests(PxSolverType::Enum solver)
	{
		gSolverType = solver;

		TestConvexCoreOnMeshIsReproducibleUnderRollback();
		TestVehicleConstructionHashSurvivesWheelPoseUpdates(false);
		TestVehicleConstructionHashSurvivesWheelPoseUpdates(true);
		TestVehicleDefaultWheelShapesAreUnchanged(false);
		TestVehicleDefaultWheelShapesAreUnchanged(true);
		TestVehicleCylinderWheelsAreDeterministic(false);
		TestVehicleCylinderWheelsAreDeterministic(true);
		TestVehicleConstructionHashIncludesWheelShapeLocalPoses(false);
		TestVehicleConstructionHashIncludesWheelShapeLocalPoses(true);
		TestVehicleBaselineDeterminism(false);
		TestVehicleBaselineDeterminism(true);
		TestVehicleRestoreRoundTrip(false);
		TestVehicleRestoreRoundTrip(true);
		TestVehicleFixedDepthRollback(false, 1.0f, "direct drive, driving");
		TestVehicleFixedDepthRollback(false, 0.0f, "direct drive, at rest");
		TestVehicleFixedDepthRollback(true, 1.0f, "engine drive, driving");
		TestVehicleVariableDepthRollback(false, 400);
		TestVehicleVariableDepthRollback(true, 400);

		gSolverType = PxSolverType::eTGS;
	}

	// ---------------------------------------------------------------------------
	// Multi-peer harness
	//
	// Everything above drives one world, or two worlds from one loop. The one thing
	// that cannot be reached that way is the property the whole netcode rests on: two
	// peers, each running its own world and its own rollback loop, fed the same inputs
	// but at different times over a lossy channel, still agree on every confirmed tick.
	//
	// This models exactly that. Each MultiPeer runs the fixed-horizon cold-step loop the
	// managed RollbackEngine runs: every confirmed tick is a single restore+step from the
	// previous confirmed snapshot, so the confirmed state is a pure function of the inputs
	// and never of when they arrived. A SimChannel carries one player's inputs to the
	// other peer with latency, loss and a redundancy window, the same shape the managed
	// SimSession puts on the wire. The assertion is that both peers' PxwWorldHashState at
	// each shared confirmed tick is identical — the native analogue of the confirmed-tick
	// hash exchange.
	// ---------------------------------------------------------------------------

	// A deterministic small force for player p on tick t. Both peers compute it identically,
	// so the only difference between them is the timing with which it is delivered.
	PxVec3 MultiPeerInput(PxU32 player, int tick)
	{
		PxU32 h = (player * 2654435761u) ^ static_cast<PxU32>(tick * 40503 + 12345);
		float fx = (static_cast<int>(h & 0xFFu) - 128) / 128.0f;
		float fz = (static_cast<int>((h >> 8) & 0xFFu) - 128) / 128.0f;
		return PxVec3(fx * 4.0f, 0.0f, fz * 4.0f);
	}

	struct MultiPeer
	{
		static const int kMaxTicks = 4096;

		struct Cmd { PxVec3 force; bool known; };

		TestWorld world;
		int horizon;
		int delay;
		PxU32 localPlayer;
		bool perturb;               // negative control: diverge this peer on purpose

		int confirmedTick;
		int currentTick;
		int lastProduced;
		bool stalled;

		std::vector<PxU8> confirmedSnapshot;
		std::vector<Cmd> inputs[2];
		int lastContig[2];
		PxVec3 lastKnownForce[2];
		int lastKnownTick[2];

		std::vector<PxU64> confirmedHash;
		std::vector<bool> hasConfirmed;

		MultiPeer()
			: horizon(0), delay(0), localPlayer(0), perturb(false),
			  confirmedTick(0), currentTick(0), lastProduced(0), stalled(false) {}

		void Init(PxU32 localPlayer_, int horizon_, int delay_, bool perturb_ = false)
		{
			localPlayer = localPlayer_;
			horizon = horizon_;
			delay = delay_;
			perturb = perturb_;

			world.Build(false);
			confirmedSnapshot = world.Capture();

			for (int p = 0; p < 2; ++p)
			{
				inputs[p].assign(kMaxTicks, Cmd());
				lastContig[p] = 0;          // tick 0 needs no input
				lastKnownForce[p] = PxVec3(0.0f);
				lastKnownTick[p] = -1;
			}

			confirmedHash.assign(kMaxTicks, 0);
			hasConfirmed.assign(kMaxTicks, false);

			confirmedTick = 0;
			currentTick = 0;
			lastProduced = 0;
			stalled = false;

			confirmedHash[0] = world.Hash();
			hasConfirmed[0] = true;
		}

		void Destroy() { world.Destroy(); }

		void Submit(PxU32 player, int tick, PxVec3 force)
		{
			if (tick < 1 || tick >= kMaxTicks || player > 1)
			{
				return;
			}
			inputs[player][tick].force = force;
			inputs[player][tick].known = true;

			while (lastContig[player] + 1 < kMaxTicks && inputs[player][lastContig[player] + 1].known)
			{
				++lastContig[player];
			}
			if (tick >= lastKnownTick[player])
			{
				lastKnownTick[player] = tick;
				lastKnownForce[player] = force;
			}
		}

		int ConfirmedThrough() const
		{
			return lastContig[0] < lastContig[1] ? lastContig[0] : lastContig[1];
		}

		void ApplyFrame(int tick, bool predict)
		{
			for (PxU32 p = 0; p < 2; ++p)
			{
				PxVec3 f;
				const Cmd& c = inputs[p][tick];
				if (c.known)
				{
					f = c.force;
				}
				else if (predict)
				{
					f = lastKnownForce[p];
				}
				else
				{
					f = PxVec3(0.0f);
				}

				if (perturb && p == localPlayer)
				{
					f += PxVec3(0.001f, 0.0f, 0.0f); // a divergence far below captured precision
				}

				if (p < world.dynamicIds.size())
				{
					PxRigidDynamic* body =
						static_cast<PxRigidDynamic*>(PxwWorldFindHandle(world.world, world.dynamicIds[p]));
					if (body != NULL && !body->isSleeping())
					{
						body->addForce(f, PxForceMode::eACCELERATION);
					}
				}
			}
		}

		bool Advance()
		{
			int through = ConfirmedThrough();
			int newConfirmed = through < confirmedTick + 1 ? through : confirmedTick + 1;
			int target = confirmedTick + horizon;

			if (newConfirmed <= confirmedTick && currentTick >= target)
			{
				stalled = true;
				return false;
			}
			stalled = false;

			if (newConfirmed > confirmedTick)
			{
				world.Restore(confirmedSnapshot);
				for (int t = confirmedTick + 1; t <= newConfirmed; ++t)
				{
					ApplyFrame(t, false);
					PxwWorldStep(world.world, kDt);
					confirmedSnapshot = world.Capture();
					if (t < kMaxTicks)
					{
						confirmedHash[t] = world.Hash();
						hasConfirmed[t] = true;
					}
				}
				confirmedTick = newConfirmed;
			}

			std::vector<PxU8> prev = confirmedSnapshot;
			int end = confirmedTick + horizon;
			for (int t = confirmedTick + 1; t <= end; ++t)
			{
				world.Restore(prev);
				ApplyFrame(t, true);
				PxwWorldStep(world.world, kDt);
				prev = world.Capture();
			}
			currentTick = end;
			return true;
		}

		// Produces this peer's local inputs up to LocalInputTick, submits them locally, and
		// returns the ones newly produced so the caller can put them on the channel.
		void ProduceLocal(std::vector<std::pair<int, PxVec3> >& produced)
		{
			int upTo = currentTick + delay;
			if (upTo >= kMaxTicks)
			{
				upTo = kMaxTicks - 1;
			}
			for (int t = lastProduced + 1; t <= upTo; ++t)
			{
				PxVec3 f = MultiPeerInput(localPlayer, t);
				Submit(localPlayer, t, f);
				produced.push_back(std::make_pair(t, f));
			}
			if (upTo > lastProduced)
			{
				lastProduced = upTo;
			}
		}
	};

	// A best-effort channel: whole packets, delivered after a fixed latency, dropped with a
	// loss probability, each carrying a redundancy window of recent inputs so a dropped
	// packet is recovered by the next.
	struct SimChannel
	{
		struct Item { PxU32 player; int tick; PxVec3 force; };
		struct Packet { int targetPeer; int deliverAt; std::vector<Item> items; };

		std::vector<Packet> inFlight;
		int latency;
		int lossPercent;
		PxU32 rng;

		SimChannel() : latency(0), lossPercent(0), rng(0x1234567u) {}

		PxU32 Next() { rng = rng * 1664525u + 1013904223u; return rng; }

		void Send(int frame, int targetPeer, const std::vector<Item>& window)
		{
			if (window.empty())
			{
				return;
			}
			if (lossPercent > 0 && static_cast<int>(Next() % 100u) < lossPercent)
			{
				return; // whole packet lost
			}
			Packet packet;
			packet.targetPeer = targetPeer;
			packet.deliverAt = frame + latency;
			packet.items = window;
			inFlight.push_back(packet);
		}

		void Deliver(int frame, MultiPeer* peers)
		{
			for (size_t i = 0; i < inFlight.size();)
			{
				if (inFlight[i].deliverAt <= frame)
				{
					const Packet& packet = inFlight[i];
					for (size_t j = 0; j < packet.items.size(); ++j)
					{
						const Item& item = packet.items[j];
						peers[packet.targetPeer].Submit(item.player, item.tick, item.force);
					}
					inFlight.erase(inFlight.begin() + i);
				}
				else
				{
					++i;
				}
			}
		}
	};

	// Runs two peers over a channel and compares their confirmed hashes tick for tick.
	// Returns how many shared confirmed ticks matched and how many were compared.
	void RunMultiPeerScenario(int horizon, int delay, int latency, int lossPercent,
		int redundancy, int frames, int minProgress, bool perturbPeer1, const char* label)
	{
		std::printf("MultiPeer [%s, %s] H=%d D=%d L=%d loss=%d%% R=%d\n",
			SolverName(), label, horizon, delay, latency, lossPercent, redundancy);

		MultiPeer peers[2];
		peers[0].Init(0, horizon, delay, false);
		peers[1].Init(1, horizon, delay, perturbPeer1);

		SimChannel channel;
		channel.latency = latency;
		channel.lossPercent = lossPercent;

		// A per-peer history of produced inputs, so each frame can resend a redundancy window.
		std::vector<std::pair<int, PxVec3> > history[2];
		bool anyStall = false;

		for (int frame = 1; frame <= frames; ++frame)
		{
			for (int p = 0; p < 2; ++p)
			{
				std::vector<std::pair<int, PxVec3> > produced;
				peers[p].ProduceLocal(produced);
				for (size_t k = 0; k < produced.size(); ++k)
				{
					history[p].push_back(produced[k]);
				}

				std::vector<SimChannel::Item> window;
				int start = static_cast<int>(history[p].size()) - redundancy;
				if (start < 0)
				{
					start = 0;
				}
				for (size_t k = static_cast<size_t>(start); k < history[p].size(); ++k)
				{
					SimChannel::Item item;
					item.player = peers[p].localPlayer;
					item.tick = history[p][k].first;
					item.force = history[p][k].second;
					window.push_back(item);
				}
				channel.Send(frame, 1 - p, window);
			}

			channel.Deliver(frame, peers);

			peers[0].Advance();
			peers[1].Advance();
			anyStall = anyStall || peers[0].stalled || peers[1].stalled;
		}

		int shared = peers[0].confirmedTick < peers[1].confirmedTick
			? peers[0].confirmedTick : peers[1].confirmedTick;
		int compared = 0;
		int matched = 0;
		for (int t = 0; t <= shared; ++t)
		{
			if (peers[0].hasConfirmed[t] && peers[1].hasConfirmed[t])
			{
				++compared;
				if (peers[0].confirmedHash[t] == peers[1].confirmedHash[t])
				{
					++matched;
				}
			}
		}

		Check(compared > 0, "  peers confirmed a shared run of ticks");
		if (perturbPeer1)
		{
			Check(matched < compared, "  a deliberately diverged peer is caught by the hash");
		}
		else
		{
			Check(matched == compared, "  every shared confirmed tick agrees across peers");
		}

		// A latent channel stalls each peer for roughly the first L frames, before any
		// remote input has arrived; that is correct rollback behaviour, not a fault. What
		// matters is that confirmation then keeps pace, so the metric is progress made, not
		// whether a stall ever happened.
		if (minProgress > 0)
		{
			Check(shared >= minProgress, "  confirmation kept pace with the channel");
		}
		else
		{
			Observe(shared > 0, "  confirmation made some progress despite loss");
		}
		Observe(anyStall, "  a peer stalled at some point (startup latency or loss)");

		peers[0].Destroy();
		peers[1].Destroy();
	}

	void RunMultiPeerTests(PxSolverType::Enum solver)
	{
		gSolverType = solver;

		// The clean case: modest latency well within the horizon, no loss. Confirmation
		// should reach nearly the end after a short startup stall.
		RunMultiPeerScenario(8, 2, 3, 0, 8, 200, 179, false, "latency, no loss");

		// Loss with a redundancy window wide enough to cover it: confirmation still keeps
		// pace because a dropped packet's inputs ride the next one.
		RunMultiPeerScenario(8, 2, 3, 30, 8, 200, 150, false, "loss with redundancy");

		// Negative control: a peer that diverges by a hair is caught by the confirmed hash,
		// while confirmation itself still keeps pace.
		RunMultiPeerScenario(8, 2, 3, 0, 8, 200, 179, true, "negative control");

		// Loss with no redundancy at all: the point of the redundancy window, shown by its
		// absence. Progress is characterised rather than required, but whatever does confirm
		// must still agree, which the matched check covers.
		RunMultiPeerScenario(8, 2, 3, 40, 1, 200, 0, false, "loss without redundancy");

		gSolverType = PxSolverType::eTGS;
	}
}

int main()
{
	if (!InitializePhysX())
	{
		std::printf("Failed to initialise PhysX\n");
		return 1;
	}

	TestRegistryOrdering();
	TestBaselineDeterminism();
	TestRegistrationOrderIndependence();
	TestRestoreRoundTrip(false, "centre of mass at origin");
	TestRestoreRoundTrip(true, "offset centre of mass");
	std::printf("\n--- framework-driven sleep ---\n");
	TestFrameworkSleepThreshold();
	TestFrameworkSleepSurvivesRestore();
	TestSleepingBodyWakesOnContact();
	TestFrameworkSleepReplays();
	TestSleeperWokenUnderRollback();

	// Mass properties are the one piece of setup a peer must not derive on its own,
	// because PhysX bakes an ill-conditioned eigenvector rotation into the mass frame.
	std::printf("\n--- deterministic mass properties ---\n");
	TestMassIsReproducible(0, "single box");
	TestMassIsReproducible(12, "12 spikes");
	TestMassIgnoresAttachOrder(12, "12 spikes");
	TestMassIgnoresAttachOrder(42, "42 spikes");
	TestIsotropyCollapseStabilisesMassFrame(12, "12 spikes");
	TestIsotropyCollapseStabilisesMassFrame(42, "42 spikes");
	TestIsotropyCollapseCanonicalisesCentreOfMass(24, "24 spikes");
	TestOffCentreMassIsPreserved();
	TestConstructionHashAgreesForIdenticalBuilds();
	TestConstructionHashCatchesWhatMassAndStateHashesMiss();
	TestConstructionHashCatchesSolverProperties();
	TestConstructionHashPerEntryNamesTheBody();
	TestConstructionHashDescribesConvexCores();
	TestConstructionHashIncludesCollisionGroupTable();
	TestApplyMassIsVerbatim();
	TestMassHashDetectsMismatch();
	TestCollapsedMassKeepsPoseRoundTripExact();

	// The gameplay body API and scene queries added on top of the core, including the
	// quaternion-first pose layout the managed structs depend on.
	std::printf("\n--- gameplay body api and scene queries ---\n");
	TestBodyApiAndReadPoseLayout();
	TestDeterministicRigidDefaults();
	TestRestoreWithParkedPoolSlot();
	TestContactEvents();
	TestTriggerEvents();
	TestSceneQueries();

	// Is a snapshot enough to reproduce a step exactly? No: PhysX warm-starts the
	// solver from contact impulses held in the persistent manifolds, and nothing in
	// the public API can read or write them. A world with no history cannot reproduce
	// a step taken by a world that had one.
	std::printf("\n--- is the snapshot complete? ---\n");
	TestFreshWorldReplay(PxwContactResetMode::eNONE, "no contact reset");
	TestFreshWorldReplay(PxwContactResetMode::eRESET_FILTERING, "resetFiltering");
	TestFreshWorldReplay(PxwContactResetMode::eREINSERT, "reinsert");

	// Since no reset makes a step independent of history, the way to bring a joiner in
	// is to give every peer the same history rather than none.
	std::printf("\n--- can peers re-sync by rebuilding together? ---\n");
	TestRebuiltWorldsAgree(PxwContactResetMode::eNONE, "no contact reset");
	TestRebuiltWorldsAgree(PxwContactResetMode::eREINSERT, "reinsert");

	// And whether the world recreation is actually load-bearing, or whether restoring
	// into the world a peer already has would do.
	TestRestoreIntoUsedWorldsDisagrees(PxwContactResetMode::eNONE, "no contact reset");
	TestRestoreIntoUsedWorldsDisagrees(PxwContactResetMode::eREINSERT, "reinsert");

	// Does rollback depth have to be synchronised across peers, or can each peer rewind
	// by whatever its own latency demands? This decides whether the prediction horizon
	// must be fixed.
	std::printf("\n--- must rollback depth match across peers? ---\n");
	TestVariableRewindDepthAgreement(PxwContactResetMode::eNONE, "no contact reset");
	TestVariableRewindDepthAgreement(PxwContactResetMode::eRESET_FILTERING, "resetFiltering");
	TestSustainedDivergentRollback(PxwContactResetMode::eNONE, "no contact reset", 600);

	// Do peers actually end up with the same PhysX-side identities?
	std::printf("\n--- do stable ids map to the same PhysX indices? ---\n");
	TestInternalIdsMatchAcrossRegistrationOrder();

	// Since replay cannot be bit-exact, the question that decides the design is how
	// large the error is. Leaving the contact caches alone keeps it at float-noise
	// level; discarding them via resetFiltering throws away the warm-start data the
	// original tick had and makes the error four orders of magnitude worse.
	std::printf("\n--- how big is the replay error? ---\n");
	TestReplayErrorMagnitude(PxwContactResetMode::eNONE, "no contact reset", 1.0e-4f, 1.0e-3f, 30);
	TestReplayErrorMagnitude(PxwContactResetMode::eRESET_FILTERING, "resetFiltering");
	TestReplayErrorMagnitude(PxwContactResetMode::eREINSERT, "reinsert");

	// Bit-exactness by rewind depth, kept as characterisation so the limits stay
	// documented rather than rediscovered.
	std::printf("\n--- bit-exact rewind by depth (characterisation) ---\n");
	TestRewindDepthSweep(PxwContactResetMode::eNONE, "none");
	TestRewindDepthSweep(PxwContactResetMode::eRESET_FILTERING, "resetFiltering");
	TestRewindDepthSweep(PxwContactResetMode::eREINSERT, "reinsert");

	// Articulations. The plugin has captured and restored them since the layer landed,
	// and nothing measured whether that works. Both solvers, because whether PGS is
	// usable for articulations is what gates an adaptive prediction horizon.
	std::printf("\n--- articulations under rollback (TGS) ---\n");
	RunArticulationTests(PxSolverType::eTGS);
	std::printf("\n--- articulations under rollback (PGS) ---\n");
	RunArticulationTests(PxSolverType::ePGS);

	// Vehicles carry drivetrain integrator state a rigid body does not, and until this
	// suite a vehicle snapshot dropped all of it. Both solvers, for the same reason the
	// articulations run both: the vehicle is the sample game's headline dynamic body.
	std::printf("\n--- vehicles under rollback (TGS) ---\n");
	RunVehicleTests(PxSolverType::eTGS);
	std::printf("\n--- vehicles under rollback (PGS) ---\n");
	RunVehicleTests(PxSolverType::ePGS);

	// Two peers, two worlds, one lossy channel: the confirmed state is a pure function of
	// the inputs, never of when they arrived, so the peers' confirmed hashes must agree.
	// Both solvers, because a fixed horizon makes every confirmed tick a single cold
	// restore+step that is identical on both peers regardless of solver.
	std::printf("\n--- multi-peer confirmed-hash agreement (TGS) ---\n");
	RunMultiPeerTests(PxSolverType::eTGS);
	std::printf("\n--- multi-peer confirmed-hash agreement (PGS) ---\n");
	RunMultiPeerTests(PxSolverType::ePGS);

	std::printf("\n%d checks, %d failures\n", gChecks, gFailures);
	return gFailures == 0 ? 0 : 1;
}
