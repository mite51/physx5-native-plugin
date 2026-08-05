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

#include <cstdio>
#include <cstring>
#include <string>
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

	// Set by the articulation runs, which ask the same questions of both solvers. TGS
	// is the framework default; PGS is the candidate for an adaptive rollback depth,
	// because it is the only one measured to make replay transparent.
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

	// Mass properties are the one piece of setup a peer must not derive on its own,
	// because PhysX bakes an ill-conditioned eigenvector rotation into the mass frame.
	std::printf("\n--- deterministic mass properties ---\n");
	TestMassIsReproducible(0, "single box");
	TestMassIsReproducible(12, "12 spikes");
	TestMassIgnoresAttachOrder(12, "12 spikes");
	TestMassIgnoresAttachOrder(42, "42 spikes");
	TestIsotropyCollapseStabilisesMassFrame(12, "12 spikes");
	TestIsotropyCollapseStabilisesMassFrame(42, "42 spikes");
	TestApplyMassIsVerbatim();
	TestMassHashDetectsMismatch();
	TestCollapsedMassKeepsPoseRoundTripExact();

	// The gameplay body API and scene queries added on top of the core, including the
	// quaternion-first pose layout the managed structs depend on.
	std::printf("\n--- gameplay body api and scene queries ---\n");
	TestBodyApiAndReadPoseLayout();
	TestDeterministicRigidDefaults();
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

	std::printf("\n%d checks, %d failures\n", gChecks, gFailures);
	return gFailures == 0 ? 0 : 1;
}
