// ---------------------------------------------------------------------------
// PxwRollbackRepro.cpp
//
// A standalone investigation into bitwise divergence under rollback.
//
// This links PhysX only. No plugin code is compiled in, so nothing in the
// wrapper -- not the stable-ID registry, not the state blob format, not the
// canonical pose handling -- can influence what is measured here. If divergence
// shows up in this file it is PhysX or the way PhysX is being driven, and
// nothing else.
//
// The working assumption is the strict one: with identical inputs, any bitwise
// difference is a bug to be tracked down, not a tolerance to be accepted. The
// tests are arranged as a bisection, each one adding a single ingredient to the
// previous, so that whichever one first goes red names the cause.
//
//   1. two identical scenes, stepped together        -- is PhysX deterministic at all
//   2. rollback with no contacts ever                -- is integration replay-exact
//   3. rollback with contacts                        -- do contacts break replay
//   4. creation order                                -- does actor ordering matter
//   5. internal indices                              -- are PhysX's own ids stable
//
// Bodies are laid out in a grid far from the world origin and spaced so that
// nothing touches at t=0, which keeps the early steps free of contact activity
// that would otherwise muddle the first few comparisons.
// ---------------------------------------------------------------------------

#include "PxPhysicsAPI.h"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

using namespace physx;

namespace
{
	// -----------------------------------------------------------------------
	// Reporting
	// -----------------------------------------------------------------------

	int gChecks = 0;
	int gFailures = 0;

	void Check(bool condition, const std::string& what)
	{
		++gChecks;
		if (!condition)
		{
			++gFailures;
		}
		std::printf("  %s  %s\n", condition ? "ok  " : "FAIL", what.c_str());
	}

	// For properties that are expected to be false and are worked around by
	// construction rather than fixed. Recording them keeps the behaviour visible and
	// would make a change in PhysX obvious, without counting as a regression.
	void Characterise(bool condition, const std::string& what)
	{
		std::printf("  %s  %s\n", condition ? "yes " : "no  ", what.c_str());
	}

	// -----------------------------------------------------------------------
	// PhysX lifetime
	// -----------------------------------------------------------------------

	PxDefaultAllocator gAllocator;
	PxDefaultErrorCallback gErrorCallback;
	PxFoundation* gFoundation = NULL;
	PxPhysics* gPhysics = NULL;
	PxDefaultCpuDispatcher* gDispatcher = NULL;
	PxMaterial* gMaterial = NULL;

	bool StartPhysX()
	{
		gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
		if (gFoundation == NULL)
		{
			return false;
		}

		gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL);
		if (gPhysics == NULL)
		{
			return false;
		}

		// Zero worker threads: the solver runs on the calling thread, which removes
		// task scheduling as a variable before anything else is measured.
		gDispatcher = PxDefaultCpuDispatcherCreate(0);
		gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.05f);
		return gDispatcher != NULL && gMaterial != NULL;
	}

	void StopPhysX()
	{
		if (gMaterial != NULL) { gMaterial->release(); gMaterial = NULL; }
		if (gDispatcher != NULL) { gDispatcher->release(); gDispatcher = NULL; }
		if (gPhysics != NULL) { gPhysics->release(); gPhysics = NULL; }
		if (gFoundation != NULL) { gFoundation->release(); gFoundation = NULL; }
	}

	// -----------------------------------------------------------------------
	// Scene
	// -----------------------------------------------------------------------

	const PxReal kDt = 1.0f / 60.0f;

	// PhysX decays the wake counter by dt every step and only runs its sleep
	// bookkeeping once the counter has dropped below wakeCounterResetTime * 0.5. At
	// this magnitude the decay is far below one ulp, so subtracting dt returns the
	// same float: the counter is a genuine fixed point rather than merely a large
	// number, and a body pinned here never reaches the branch at all.
	const PxReal kNeverSleepWakeCounter = PX_MAX_F32;

	// PhysX's internal wake counter reset value, which is not exposed. A reset lands
	// on this value plus one dt per counted contact interaction.
	const PxReal kWakeCounterResetTime = 20.0f * 0.02f;

	struct Config
	{
		int bodyCount;
		bool withGround;       // contacts at all
		bool spin;             // initial angular velocity
		bool stack;            // one tall column instead of a flat grid
		bool reverseOrder;     // creation order
		PxBroadPhaseType::Enum broadPhase;
		PxSolverType::Enum solver;
		bool enhancedDeterminism;

		// Contact persistence. Both are immutable and must be set at scene creation.
		// PCM keeps a manifold alive across frames and warm-starts from it; the contact
		// cache is a separate generation-time cache. Between them they are the obvious
		// candidates for state that survives a restore.
		bool persistentContactManifolds;   // PxSceneFlag::eENABLE_PCM, on by default
		bool contactCache;                 // inverse of eDISABLE_CONTACT_CACHE

		// Pin the wake counter so that PhysX's sleep bookkeeping never runs at all.
		// See kNeverSleepWakeCounter and section 3j.
		bool neverSleep;

		// Force PhysX to rediscover contact pairs on every restore instead of updating
		// its existing set incrementally. Section 3o: the pair set is carried across a
		// rewind, and this is the only public way to make it a function of the current
		// state rather than of the path taken to reach it.
		bool resetFilteringOnRestore;

		// Height of the lowest body. The default drops everything 39 m onto the ground,
		// which arrives at about 27 m/s; section 3p uses a low value to ask whether the
		// stack's failure needs a violent impact or merely a changing set of contacts.
		PxReal spawnHeight;

		// Grid spacing. The default of 3 m keeps the bodies clear of one another, so
		// every contact in that scene is body-to-static. Section 3r narrows it until
		// they touch, which is the one property the failing scenes share.
		PxReal gridSpacing;

		// Density multiplier for every body except the lowest. A short column of heavy
		// boxes loads its bottom contact like a tall column of light ones without
		// lengthening the chain, which is how section 3t tells the two apart.
		PxReal upperDensityScale;

		// Install a touch-reporting filter shader and simulation event callback so the
		// depth of the resting contact graph can be measured (section 3u). Off by default:
		// every asserted determinism scene keeps PxDefaultSimulationFilterShader untouched,
		// so the numbers those tests defend are unaffected. The shader added here only ORs
		// in notification flags and changes no collision or solve decision, which is the
		// same property the shipping contact-event work depends on.
		bool measureContacts;

		// Use an upright capsule instead of a box, so the representative-workload section
		// can measure the curved, near-point contact a character controller rests on rather
		// than a box's flat four-point manifold.
		bool capsule;

		Config()
			: bodyCount(16)
			, withGround(true)
			, spin(true)
			, stack(false)
			, reverseOrder(false)
			, broadPhase(PxBroadPhaseType::ePABP)
			, solver(PxSolverType::ePGS)
			, enhancedDeterminism(true)
			, persistentContactManifolds(true)
			, contactCache(true)
			, neverSleep(false)
			, resetFilteringOnRestore(false)
			, spawnHeight(40.0f)
			, gridSpacing(3.0f)
			, upperDensityScale(1.0f)
			, measureContacts(false)
			, capsule(false)
		{
		}
	};

	// Far from the origin, so that the float exponent is realistic rather than the
	// unusually forgiving range around zero.
	const PxVec3 kOrigin(500.0f, 40.0f, 500.0f);

	struct BodyState
	{
		PxTransform pose;
		PxVec3 linearVelocity;
		PxVec3 angularVelocity;
		PxReal wakeCounter;
		PxU32 sleeping;
	};

	// Records the actor pairs PhysX reports as touching during a step, so the resting
	// contact graph can be rebuilt and its depth measured. This is the only piece of the
	// suite that reads contacts from PhysX rather than from PxSimulationStatistics, which
	// gives totals but not who-touches-whom.
	struct ContactGraphRecorder : PxSimulationEventCallback
	{
		std::vector<std::pair<const PxActor*, const PxActor*> > touching;

		void Clear() { touching.clear(); }

		void onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 count) override
		{
			for (PxU32 i = 0; i < count; ++i)
			{
				const PxPairFlags e = pairs[i].events;
				if (e & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS))
				{
					touching.push_back(std::make_pair(header.actors[0], header.actors[1]));
				}
			}
		}

		void onTrigger(PxTriggerPair*, PxU32) override {}
		void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
		void onWake(PxActor**, PxU32) override {}
		void onSleep(PxActor**, PxU32) override {}
		void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}
	};

	// The default filter shader with contact-touch notifications ORed on. It changes no
	// collision or solve decision -- it only asks PhysX to report the contacts it was
	// already generating -- so a scene built with it simulates identically to one built
	// with PxDefaultSimulationFilterShader. That is the whole reason contact reporting can
	// be added to the deterministic runtime later without moving the numbers.
	PxFilterFlags ChainDepthFilterShader(
		PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		const PxFilterFlags flags = PxDefaultSimulationFilterShader(
			attributes0, filterData0, attributes1, filterData1, pairFlags, constantBlock, constantBlockSize);
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS;
		return flags;
	}

	struct World
	{
		PxScene* scene;
		std::vector<PxRigidDynamic*> bodies;   // always in logical order, whatever the creation order
		PxRigidStatic* ground;
		Config config;
		ContactGraphRecorder contacts;

		World() : scene(NULL), ground(NULL) {}

		void Build(const Config& cfg)
		{
			config = cfg;

			PxSceneDesc desc(gPhysics->getTolerancesScale());
			desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			desc.cpuDispatcher = gDispatcher;
			desc.filterShader = cfg.measureContacts ? ChainDepthFilterShader : PxDefaultSimulationFilterShader;
			desc.broadPhaseType = cfg.broadPhase;
			desc.solverType = cfg.solver;
			if (cfg.enhancedDeterminism)
			{
				desc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
			}
			if (cfg.persistentContactManifolds)
			{
				desc.flags |= PxSceneFlag::eENABLE_PCM;
			}
			else
			{
				desc.flags &= ~PxSceneFlags(PxSceneFlag::eENABLE_PCM);
			}
			if (!cfg.contactCache)
			{
				desc.flags |= PxSceneFlag::eDISABLE_CONTACT_CACHE;
			}
			scene = gPhysics->createScene(desc);

			if (cfg.measureContacts)
			{
				scene->setSimulationEventCallback(&contacts);
			}

			if (cfg.withGround)
			{
				ground = gPhysics->createRigidStatic(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
				PxRigidActorExt::createExclusiveShape(*ground, PxBoxGeometry(2000.0f, 1.0f, 2000.0f), *gMaterial);
				scene->addActor(*ground);
			}

			bodies.assign(static_cast<size_t>(cfg.bodyCount), static_cast<PxRigidDynamic*>(NULL));

			// The logical layout is identical either way; only the order in which the
			// actors are handed to the scene changes.
			for (int i = 0; i < cfg.bodyCount; ++i)
			{
				const int logical = cfg.reverseOrder ? (cfg.bodyCount - 1 - i) : i;
				bodies[static_cast<size_t>(logical)] = CreateBody(logical, cfg);
			}
		}

		PxRigidDynamic* CreateBody(int logical, const Config& cfg)
		{
			// A 4x4 grid with 3 m spacing and 0.5 m half extents: nothing is in contact
			// with anything at t=0.
			const PxVec3 base(kOrigin.x, cfg.spawnHeight, kOrigin.z);

			PxVec3 position;
			if (cfg.stack)
			{
				// A single column with a small gap, which closes as it settles. This is
				// the case warm starting exists for: the weight of everything above has
				// to propagate down through the contacts every step.
				position = base + PxVec3(0.0f, static_cast<PxReal>(logical) * 1.05f, 0.0f);
			}
			else
			{
				const int gx = logical % 4;
				const int gz = (logical / 4) % 4;
				const int gy = logical / 16;

				position = base + PxVec3(
					static_cast<PxReal>(gx) * cfg.gridSpacing,
					static_cast<PxReal>(gy) * cfg.gridSpacing,
					static_cast<PxReal>(gz) * cfg.gridSpacing);
			}

			PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(position));
			if (cfg.capsule)
			{
				// A 0.5 m radius, 0.5 m half-height capsule stood upright: PhysX capsules
				// lie along local X, so rotate the shape a quarter turn about Z to put the
				// axis on Y and rest it on its rounded end, the way a character controller
				// does.
				PxShape* shape = PxRigidActorExt::createExclusiveShape(*body, PxCapsuleGeometry(0.5f, 0.5f), *gMaterial);
				shape->setLocalPose(PxTransform(PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
			}
			else
			{
				PxRigidActorExt::createExclusiveShape(*body, PxBoxGeometry(0.5f, 0.5f, 0.5f), *gMaterial);
			}
			const PxReal density = (logical > 0) ? 10.0f * cfg.upperDensityScale : 10.0f;
			PxRigidBodyExt::updateMassAndInertia(*body, density);

			if (cfg.neverSleep)
			{
				// PhysX's own sleep threshold is left at its default here, deliberately.
				// The pin alone has to be what keeps the sleep path from running, since
				// that is all the framework does; a body that also needed a doctored
				// threshold would not be evidence about the framework's configuration.
				body->setWakeCounter(kNeverSleepWakeCounter);
			}
			else
			{
				// Never let a body sleep outright, because a sleep transition would mask
				// the numerical question being asked.
				//
				// A zero threshold does not switch the sleep machinery off, though. It
				// makes the energy test pass unconditionally, so the wake counter resets
				// every time it decays past the gate rather than ever reaching zero. The
				// bookkeeping behind that reset still runs, which is what section 3i is
				// about.
				body->setSleepThreshold(0.0f);
			}

			if (cfg.spin)
			{
				// Deterministic but uneven, so the bodies do not move as a rigid block.
				const PxReal s = static_cast<PxReal>(logical + 1);
				body->setAngularVelocity(PxVec3(0.31f * s, -0.17f * s, 0.23f * s));
				body->setLinearVelocity(PxVec3(0.11f * s, 0.0f, -0.07f * s));
			}

			scene->addActor(*body);
			return body;
		}

		void Step()
		{
			// onContact fires during fetchResults, so the touch set is cleared before the
			// step and holds this step's contacts afterwards.
			if (config.measureContacts)
			{
				contacts.Clear();
			}
			scene->simulate(kDt);
			scene->fetchResults(true);
		}

		void Capture(std::vector<BodyState>& out) const
		{
			out.resize(bodies.size());
			for (size_t i = 0; i < bodies.size(); ++i)
			{
				BodyState& s = out[i];
				s.pose = bodies[i]->getGlobalPose();
				s.linearVelocity = bodies[i]->getLinearVelocity();
				s.angularVelocity = bodies[i]->getAngularVelocity();
				s.wakeCounter = bodies[i]->getWakeCounter();
				s.sleeping = bodies[i]->isSleeping() ? 1u : 0u;
			}
		}

		void Restore(const std::vector<BodyState>& in)
		{
			for (size_t i = 0; i < bodies.size() && i < in.size(); ++i)
			{
				const BodyState& s = in[i];
				bodies[i]->setGlobalPose(s.pose, false);
				bodies[i]->setLinearVelocity(s.linearVelocity, false);
				bodies[i]->setAngularVelocity(s.angularVelocity, false);
				bodies[i]->clearForce(PxForceMode::eFORCE);
				bodies[i]->clearForce(PxForceMode::eIMPULSE);
				bodies[i]->clearTorque(PxForceMode::eFORCE);
				bodies[i]->clearTorque(PxForceMode::eIMPULSE);

				// Under the never-sleep policy the counter is not restored from the
				// snapshot but re-pinned, so that it is a constant of the simulation
				// rather than a value the snapshot has to carry correctly.
				bodies[i]->setWakeCounter(config.neverSleep ? kNeverSleepWakeCounter : s.wakeCounter);
			}

			if (config.resetFilteringOnRestore)
			{
				for (size_t i = 0; i < bodies.size(); ++i)
				{
					scene->resetFiltering(*bodies[i]);
				}
			}
		}

		void Destroy()
		{
			if (scene != NULL)
			{
				scene->release();
				scene = NULL;
			}
			bodies.clear();
			ground = NULL;
		}
	};

	// -----------------------------------------------------------------------
	// Contact chain depth
	//
	// The variable-depth failures track a contact chain deeper than eight bodies. That
	// number has been an observation about column height rather than a measured property
	// of the contact graph. These turn it into a measurement: build the graph from the
	// touch set the recorder captured, and report the deepest chain of resting contacts
	// rooted at the ground. A game can carry the same walk to enforce the limit on real
	// content instead of trusting that its stacks are short enough.
	// -----------------------------------------------------------------------

	int ContactNodeOf(const World& w, const PxActor* actor)
	{
		// Node 0 is the ground; nodes 1..N are the dynamic bodies in logical order.
		if (actor == w.ground)
		{
			return 0;
		}
		for (size_t i = 0; i < w.bodies.size(); ++i)
		{
			if (w.bodies[i] == actor)
			{
				return static_cast<int>(i) + 1;
			}
		}
		return -1;
	}

	// Longest simple path from `node`, counting the dynamic bodies on it. The ground
	// (node 0) anchors the chain but is not itself counted. N is small and the measured
	// graphs are essentially trees, so the exponential worst case never bites.
	int LongestContactChain(const std::vector<std::vector<int> >& adjacency, int node, std::vector<char>& onPath)
	{
		onPath[static_cast<size_t>(node)] = 1;
		int deepest = 0;
		for (size_t i = 0; i < adjacency[static_cast<size_t>(node)].size(); ++i)
		{
			const int next = adjacency[static_cast<size_t>(node)][i];
			if (!onPath[static_cast<size_t>(next)])
			{
				const int sub = LongestContactChain(adjacency, next, onPath);
				if (sub > deepest)
				{
					deepest = sub;
				}
			}
		}
		onPath[static_cast<size_t>(node)] = 0;
		return (node == 0 ? 0 : 1) + deepest;
	}

	// The number of dynamic bodies on the deepest chain of resting contacts rooted at the
	// ground, built from the touch set of the last Step(). Step() must have run with
	// Config::measureContacts on. A flat grid returns 1 (every box touches only the
	// ground); a settled column of height H returns H.
	int MaxContactChainDepth(const World& w)
	{
		if (w.ground == NULL)
		{
			return 0;
		}

		const int nodeCount = static_cast<int>(w.bodies.size()) + 1;
		std::vector<std::vector<int> > adjacency(static_cast<size_t>(nodeCount));

		for (size_t k = 0; k < w.contacts.touching.size(); ++k)
		{
			const int a = ContactNodeOf(w, w.contacts.touching[k].first);
			const int b = ContactNodeOf(w, w.contacts.touching[k].second);
			if (a < 0 || b < 0 || a == b)
			{
				continue;
			}
			adjacency[static_cast<size_t>(a)].push_back(b);
			adjacency[static_cast<size_t>(b)].push_back(a);
		}

		std::vector<char> onPath(static_cast<size_t>(nodeCount), 0);
		return LongestContactChain(adjacency, 0, onPath);
	}

	// -----------------------------------------------------------------------
	// Comparison
	// -----------------------------------------------------------------------

	bool StatesEqual(const std::vector<BodyState>& a, const std::vector<BodyState>& b)
	{
		if (a.size() != b.size())
		{
			return false;
		}
		return a.empty() || std::memcmp(&a[0], &b[0], a.size() * sizeof(BodyState)) == 0;
	}

	// Reports the first body that differs, and by how much, so that a failure says
	// something about its own cause rather than just "not equal".
	void ReportFirstDifference(const std::vector<BodyState>& a, const std::vector<BodyState>& b)
	{
		for (size_t i = 0; i < a.size() && i < b.size(); ++i)
		{
			if (std::memcmp(&a[i], &b[i], sizeof(BodyState)) == 0)
			{
				continue;
			}

			const PxVec3 dp = b[i].pose.p - a[i].pose.p;
			const PxVec3 dv = b[i].linearVelocity - a[i].linearVelocity;
			const PxVec3 dw = b[i].angularVelocity - a[i].angularVelocity;
			const PxQuat& qa = a[i].pose.q;
			const PxQuat& qb = b[i].pose.q;

			std::printf("        first difference at body %d\n", static_cast<int>(i));
			std::printf("           position delta %.9g  (%.9g %.9g %.9g)\n",
				dp.magnitude(), dp.x, dp.y, dp.z);
			std::printf("           rotation delta (%.9g %.9g %.9g %.9g)\n",
				qb.x - qa.x, qb.y - qa.y, qb.z - qa.z, qb.w - qa.w);
			std::printf("           linVel   delta %.9g\n", dv.magnitude());
			std::printf("           angVel   delta %.9g\n", dw.magnitude());
			std::printf("           wake %.9g -> %.9g, sleeping %u -> %u\n",
				a[i].wakeCounter, b[i].wakeCounter, a[i].sleeping, b[i].sleeping);

			// A difference that no field explains is a difference in the bytes rather
			// than the values: an unnormalised duplicate, a signed zero, or padding.
			if (dp.isZero() && dv.isZero() && dw.isZero() &&
				qa.x == qb.x && qa.y == qb.y && qa.z == qb.z && qa.w == qb.w &&
				a[i].wakeCounter == b[i].wakeCounter && a[i].sleeping == b[i].sleeping)
			{
				const PxU8* pa = reinterpret_cast<const PxU8*>(&a[i]);
				const PxU8* pb = reinterpret_cast<const PxU8*>(&b[i]);
				std::printf("           every field compares equal; differing bytes at offsets:");
				for (size_t byte = 0; byte < sizeof(BodyState); ++byte)
				{
					if (pa[byte] != pb[byte])
					{
						std::printf(" %d(%02x/%02x)", static_cast<int>(byte), pa[byte], pb[byte]);
					}
				}
				std::printf("\n");
			}
			return;
		}
	}

	// -----------------------------------------------------------------------
	// 1. Is PhysX deterministic at all, given identical construction?
	// -----------------------------------------------------------------------

	void TestPairedScenesAgree(const Config& base, const char* label, int steps)
	{
		std::printf("TestPairedScenesAgree [%s]\n", label);

		World a, b;
		a.Build(base);
		b.Build(base);

		std::vector<BodyState> sa, sb;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < steps; ++i)
		{
			a.Step();
			b.Step();
			a.Capture(sa);
			b.Capture(sb);
			if (!StatesEqual(sa, sb))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(sa, sb);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d steps, identical\n", steps);
		}
		else
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, steps);
		}
		Check(matched, std::string("two identically built scenes agree [") + label + "]");

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// 2/3. Is a rollback transparent?
	//
	// The reference runs straight through with no restores at all, which is the
	// "nothing ever rolled back" timeline. The subject runs to the same point,
	// rewinds, and replays. If rollback is transparent the two agree bitwise.
	// -----------------------------------------------------------------------

	// `expectTransparent` is false for the configurations we now understand to be
	// asymmetric: the reference steps warm because it never restored, the subject steps
	// cold because it just did, and no scene flag closes that gap. Section 3f shows the
	// fix is to make the reference cold too, rather than to make the restore warmer.
	void TestRollbackIsTransparent(const Config& base, const char* label, int warmup, int replay,
		bool expectTransparent = true)
	{
		std::printf("TestRollbackIsTransparent [%s]\n", label);

		// Reference: no rollback, ever.
		World reference;
		reference.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			reference.Step();
		}

		std::vector<std::vector<BodyState> > trace(static_cast<size_t>(replay));
		for (int i = 0; i < replay; ++i)
		{
			reference.Step();
			reference.Capture(trace[static_cast<size_t>(i)]);
		}
		reference.Destroy();

		// Subject: same warmup, then rewind one snapshot and replay the same ticks.
		World subject;
		subject.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			subject.Step();
		}

		std::vector<BodyState> rewindPoint;
		subject.Capture(rewindPoint);

		// Burn the same ticks once, so the rollback has something to undo.
		for (int i = 0; i < replay; ++i)
		{
			subject.Step();
		}

		subject.Restore(rewindPoint);

		std::vector<BodyState> current;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < replay; ++i)
		{
			subject.Step();
			subject.Capture(current);
			if (!StatesEqual(current, trace[static_cast<size_t>(i)]))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(trace[static_cast<size_t>(i)], current);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d replayed steps, identical to the un-rolled-back run\n", replay);
		}
		else
		{
			std::printf("        diverged at replayed step %d of %d\n", divergedAt, replay);
		}
		if (expectTransparent)
		{
			Check(matched, std::string("rollback is bitwise transparent [") + label + "]");
		}
		else
		{
			Characterise(matched, std::string("rollback is bitwise transparent [") + label + "]");
		}

		subject.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3b. Is it the history, or the restore call itself?
	//
	// A rollback does two things at once: it discards ticks the world already
	// simulated, and it writes state back through the API. Only the first is
	// supposed to matter. This separates them by restoring state the world is
	// already in -- a logical no-op -- without discarding anything.
	//
	// If a redundant restore perturbs the simulation, then the problem is the
	// setters, not any hidden contact history, and it is fixable by not calling
	// the ones that do damage.
	// -----------------------------------------------------------------------

	struct RestoreMask
	{
		enum Enum
		{
			eNothing = 0,
			ePose = 1 << 0,
			eVelocity = 1 << 1,
			eWakeCounter = 1 << 2,
			eClearForces = 1 << 3,
			eEverything = ePose | eVelocity | eWakeCounter | eClearForces
		};
	};

	void RestoreWithMask(World& world, const std::vector<BodyState>& in, PxU32 mask)
	{
		for (size_t i = 0; i < world.bodies.size() && i < in.size(); ++i)
		{
			const BodyState& s = in[i];
			PxRigidDynamic* body = world.bodies[i];

			if (mask & RestoreMask::ePose)
			{
				body->setGlobalPose(s.pose, false);
			}
			if (mask & RestoreMask::eVelocity)
			{
				body->setLinearVelocity(s.linearVelocity, false);
				body->setAngularVelocity(s.angularVelocity, false);
			}
			if (mask & RestoreMask::eClearForces)
			{
				body->clearForce(PxForceMode::eFORCE);
				body->clearForce(PxForceMode::eIMPULSE);
				body->clearTorque(PxForceMode::eFORCE);
				body->clearTorque(PxForceMode::eIMPULSE);
			}
			if (mask & RestoreMask::eWakeCounter)
			{
				body->setWakeCounter(s.wakeCounter);
			}
		}
	}

	void TestRedundantRestoreIsHarmless(const Config& base, const char* label,
		PxU32 mask, int warmup, int steps, bool expectHarmless = true)
	{
		std::printf("TestRedundantRestoreIsHarmless [%s]\n", label);

		World reference;
		reference.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			reference.Step();
		}
		std::vector<std::vector<BodyState> > trace(static_cast<size_t>(steps));
		for (int i = 0; i < steps; ++i)
		{
			reference.Step();
			reference.Capture(trace[static_cast<size_t>(i)]);
		}
		reference.Destroy();

		World subject;
		subject.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			subject.Step();
		}

		// Write back exactly what is already there. Nothing is being undone.
		std::vector<BodyState> here;
		subject.Capture(here);
		RestoreWithMask(subject, here, mask);

		std::vector<BodyState> current;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < steps; ++i)
		{
			subject.Step();
			subject.Capture(current);
			if (!StatesEqual(current, trace[static_cast<size_t>(i)]))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(trace[static_cast<size_t>(i)], current);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d steps, unaffected by the redundant restore\n", steps);
		}
		else
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, steps);
		}
		if (expectHarmless)
		{
			Check(matched, std::string("a redundant restore changes nothing [") + label + "]");
		}
		else
		{
			Characterise(matched, std::string("a redundant restore changes nothing [") + label + "]");
		}

		subject.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3c. Why does writing back an unchanged pose change anything?
	//
	// PhysX simulates the centre-of-mass frame, not the actor frame. getGlobalPose
	// returns bodyPose * cMassLocalPose^-1 and setGlobalPose stores
	// pose * cMassLocalPose, so a round trip multiplies by an inverse and then by
	// the original. That is the identity in exact arithmetic and merely very close
	// to it in floats.
	//
	// The size of the error depends on cMassLocalPose. If it is the identity the
	// conversion is skipped or trivial; the further it is from the identity, and the
	// further the body is from the origin, the more bits are lost.
	// -----------------------------------------------------------------------

	void TestPoseRoundTripMechanism()
	{
		std::printf("TestPoseRoundTripMechanism\n");

		Config cfg;
		World world;
		world.Build(cfg);

		PxRigidDynamic* body = world.bodies[3];
		const PxTransform com = body->getCMassLocalPose();
		std::printf("        cMassLocalPose p (%.9g %.9g %.9g)\n", com.p.x, com.p.y, com.p.z);
		std::printf("        cMassLocalPose q (%.9g %.9g %.9g %.9g)\n", com.q.x, com.q.y, com.q.z, com.q.w);
		std::printf("        (a cube has an isotropic inertia tensor, so its eigenbasis is\n");
		std::printf("         degenerate and the diagonalisation may return any rotation)\n");

		const bool comIsIdentity =
			com.p.x == 0.0f && com.p.y == 0.0f && com.p.z == 0.0f &&
			com.q.x == 0.0f && com.q.y == 0.0f && com.q.z == 0.0f && com.q.w == 1.0f;
		std::printf("        centre of mass frame is %s\n",
			comIsIdentity ? "the identity" : "NOT the identity");

		// Hammer the round trip with no simulation at all, so only the conversion is
		// being measured.
		const PxTransform start = body->getGlobalPose();
		PxTransform current = start;
		int firstDrift = -1;
		for (int i = 0; i < 64; ++i)
		{
			body->setGlobalPose(current, false);
			const PxTransform after = body->getGlobalPose();
			if (firstDrift < 0 &&
				(after.p.x != current.p.x || after.p.y != current.p.y || after.p.z != current.p.z ||
				 after.q.x != current.q.x || after.q.y != current.q.y ||
				 after.q.z != current.q.z || after.q.w != current.q.w))
			{
				firstDrift = i;
			}
			current = after;
		}

		const PxVec3 drift = current.p - start.p;
		std::printf("        after 64 set/get cycles the reported pose moved %.9g m\n", drift.magnitude());
		std::printf("        first reported change at cycle %d\n", firstDrift);
		Check(firstDrift < 0, "getGlobalPose(setGlobalPose(p)) reports p unchanged");

		world.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3d. Can a restore be made to erase history entirely?
	//
	// The pose write is exact, so the damage is a side effect: teleporting an actor
	// invalidates the cached contact data for its pairs, and the next step then
	// solves from a cold cache instead of a warm one. That is why a restored world
	// differs from one that simply kept running.
	//
	// But it points at a way out. If the invalidation is total and deterministic,
	// then restoring puts every world into the same cold state regardless of what it
	// did before, and "restore then step" becomes a pure function of the snapshot --
	// which is all rollback actually needs.
	//
	// This drives two worlds along deliberately different histories, hands them the
	// same snapshot, and steps them together for a long run.
	// -----------------------------------------------------------------------

	void TestRestoreMakesStepPure(const Config& base, const char* label,
		bool restoreEveryStep, int compareSteps)
	{
		std::printf("TestRestoreMakesStepPure [%s]\n", label);

		// Two worlds with deliberately different amounts of history.
		World a, b;
		a.Build(base);
		b.Build(base);
		for (int i = 0; i < 120; ++i) { a.Step(); }
		for (int i = 0; i < 313; ++i) { b.Step(); }

		// A third world supplies a snapshot neither of them produced.
		World source;
		source.Build(base);
		for (int i = 0; i < 200; ++i) { source.Step(); }
		std::vector<BodyState> handover;
		source.Capture(handover);
		source.Destroy();

		a.Restore(handover);
		b.Restore(handover);

		std::vector<BodyState> sa, sb;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < compareSteps; ++i)
		{
			if (restoreEveryStep && i > 0)
			{
				// Each world re-restores its own state, so both perform the identical
				// sequence of API calls whatever their history.
				a.Capture(sa);
				b.Capture(sb);
				a.Restore(sa);
				b.Restore(sb);
			}

			a.Step();
			b.Step();
			a.Capture(sa);
			b.Capture(sb);

			if (!StatesEqual(sa, sb))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(sa, sb);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d steps, two different histories stayed identical\n", compareSteps);
		}
		else
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, compareSteps);
		}
		Check(matched, std::string("restore erases history [") + label + "]");

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3f. Can transparency be bought by throwing the cache away every step?
	//
	// The reason a replayed tick differs from the original is that the original ran
	// with a warm contact cache and the replay, having just been restored, ran with a
	// cold one. The asymmetry is not that restoring is lossy -- it is that restoring
	// is *different* from not restoring.
	//
	// So remove the asymmetry. If every step is preceded by a restore, including the
	// steps nobody rolled back, then every step is cold and a replayed tick should be
	// indistinguishable from the original. That would make rollback transparent and
	// remove the need to synchronise rollback depth across peers.
	//
	// The cost is whatever warm starting was buying in solver quality.
	// -----------------------------------------------------------------------

	// `expectTransparent` is false for TGS, which does not achieve transparency under
	// the cold-step discipline and is recorded rather than asserted. See section 3k.
	void TestColdStepsGiveTransparency(const Config& base, const char* label, int warmup, int replay,
		bool expectTransparent = true)
	{
		std::printf("TestColdStepsGiveTransparency [%s]\n", label);

		std::vector<BodyState> scratch;

		// Reference: never rolls back, but restores its own state before every step so
		// that every step starts cold.
		World reference;
		reference.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			reference.Capture(scratch);
			reference.Restore(scratch);
			reference.Step();
		}

		std::vector<std::vector<BodyState> > trace(static_cast<size_t>(replay));
		for (int i = 0; i < replay; ++i)
		{
			reference.Capture(scratch);
			reference.Restore(scratch);
			reference.Step();
			reference.Capture(trace[static_cast<size_t>(i)]);
		}
		reference.Destroy();

		// Subject: identical discipline, but rolls back partway and replays.
		World subject;
		subject.Build(base);
		for (int i = 0; i < warmup; ++i)
		{
			subject.Capture(scratch);
			subject.Restore(scratch);
			subject.Step();
		}

		std::vector<BodyState> rewindPoint;
		subject.Capture(rewindPoint);

		for (int i = 0; i < replay; ++i)
		{
			subject.Capture(scratch);
			subject.Restore(scratch);
			subject.Step();
		}

		subject.Restore(rewindPoint);

		std::vector<BodyState> current;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < replay; ++i)
		{
			// Exactly one restore precedes every step, here as in the reference. The
			// rewind above already provided the first one, and doing another would
			// normalise the rotation twice, which is not the same as normalising once.
			if (i > 0)
			{
				subject.Capture(scratch);
				subject.Restore(scratch);
			}
			subject.Step();
			subject.Capture(current);
			if (!StatesEqual(current, trace[static_cast<size_t>(i)]))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(trace[static_cast<size_t>(i)], current);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d replayed steps, identical to the un-rolled-back run\n", replay);
		}
		else
		{
			std::printf("        diverged at replayed step %d of %d\n", divergedAt, replay);
		}
		const std::string what = std::string("cold steps make rollback transparent [") + label + "]";
		if (expectTransparent)
		{
			Check(matched, what);
		}
		else
		{
			Characterise(matched, what);
		}

		subject.Destroy();
	}

	// If cold steps do deliver transparency, then peers no longer have to agree on how
	// far they rewound. This is the sustained version of that claim: two peers rolling
	// back by different, varying amounts every single frame for a long run.
	//
	// Returns true if they stayed identical throughout. On failure `divergedAtOut`
	// receives the frame index, and `report` selects whether the first differing body is
	// printed -- the phase sweep in section 3l runs this many times over and wants only
	// the frame numbers.
	// The deepest rewind either peer performs. A run must warm up at least this far or
	// the rewind would index history that was never written.
	const int kMaxRewindDepth = 23;

	bool RunVariableDepth(const Config& base, int warmup, int frames, int* divergedAtOut, bool report)
	{
		const int historyDepth = 32;
		if (warmup < kMaxRewindDepth) { warmup = kMaxRewindDepth; }

		World a, b;
		a.Build(base);
		b.Build(base);

		std::vector<std::vector<BodyState> > historyA(historyDepth);
		std::vector<std::vector<BodyState> > historyB(historyDepth);
		std::vector<BodyState> scratch;

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
			a.Capture(historyA[tick % historyDepth]);
			b.Capture(historyB[tick % historyDepth]);
		}

		bool matched = true;
		int divergedAt = -1;

		for (int frame = 0; frame < frames && matched; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 11;
			const int depthB = 1 + (frame * 7) % 23;

			// As above: the rewind is the restore for the first replayed step, so only
			// the subsequent ones restore again.
			a.Restore(historyA[(tick - depthA) % historyDepth]);
			for (int t = tick - depthA + 1; t <= tick; ++t)
			{
				if (t != tick - depthA + 1) { a.Capture(scratch); a.Restore(scratch); }
				a.Step();
				a.Capture(historyA[t % historyDepth]);
			}

			b.Restore(historyB[(tick - depthB) % historyDepth]);
			for (int t = tick - depthB + 1; t <= tick; ++t)
			{
				if (t != tick - depthB + 1) { b.Capture(scratch); b.Restore(scratch); }
				b.Step();
				b.Capture(historyB[t % historyDepth]);
			}

			if (!StatesEqual(historyA[tick % historyDepth], historyB[tick % historyDepth]))
			{
				matched = false;
				divergedAt = frame;
				if (report)
				{
					ReportFirstDifference(historyA[tick % historyDepth], historyB[tick % historyDepth]);
				}
			}
		}

		a.Destroy();
		b.Destroy();

		if (divergedAtOut) { *divergedAtOut = divergedAt; }
		return matched;
	}

	// -----------------------------------------------------------------------
	// 3l. Is the 16-high stack's failure about stacking, or about impact?
	//
	// The stack is dropped from 39 m and the run that fails starts its variable-depth
	// phase at tick 60, deep in free fall. First contact is around tick 168, and the
	// reported divergence lands at tick 175 -- seven ticks into a cascade of sixteen
	// impacts at 27 m/s. That is suggestive enough to test directly: start the
	// variable-depth phase at a range of ticks and see whether the failure follows the
	// impact or is a property of the settled column.
	//
	// Prints the tick at which the column first meets the ground, so the sweep can be
	// read against it.
	void TestVariableDepthPhaseSweep(const Config& base, const char* label, int frames)
	{
		std::printf("TestVariableDepthPhaseSweep [%s]\n", label);

		// Find first contact, by watching for the first tick at which any body stops
		// accelerating downwards.
		int firstContact = -1;
		{
			World probe;
			probe.Build(base);
			std::vector<BodyState> previous, current;
			probe.Capture(previous);
			for (int tick = 0; tick < 400 && firstContact < 0; ++tick)
			{
				probe.Capture(current); probe.Restore(current); probe.Step();
				probe.Capture(current);
				for (size_t i = 0; i < current.size(); ++i)
				{
					if (current[i].linearVelocity.y > previous[i].linearVelocity.y)
					{
						firstContact = tick;
						break;
					}
				}
				previous = current;
			}
			probe.Destroy();
		}
		std::printf("        first contact at tick %d\n", firstContact);
		std::printf("        start tick   result\n");

		static const int kStarts[] = { 60, 120, 150, 165, 175, 185, 200, 230, 260, 300, 360 };
		const int startCount = static_cast<int>(sizeof(kStarts) / sizeof(kStarts[0]));

		for (int i = 0; i < startCount; ++i)
		{
			const int warmup = kStarts[i];
			int divergedAt = -1;
			const bool matched = RunVariableDepth(base, warmup, frames, &divergedAt, false);

			if (matched)
			{
				std::printf("        %10d   identical for %d frames\n", warmup, frames);
			}
			else
			{
				std::printf("        %10d   diverged at frame %d (tick %d)\n",
					warmup, divergedAt, warmup + divergedAt);
			}
		}
	}

	// -----------------------------------------------------------------------
	// 3m. Does the contact bookkeeping differ before the state does?
	//
	// The sweep above puts the failure inside the impact cascade and nowhere else,
	// which points at the same class of cause as the wake counter in section 3i:
	// PhysX maintains its touch set from found/lost *transitions* against the previous
	// step, and a restore rewrites body state without rewriting that set. While the
	// column is settled the touch set is constant, so no peer can disagree about it
	// however far it rewound; while bodies are arriving it changes every step.
	//
	// PxSimulationStatistics exposes exactly the quantities that would show it.
	struct StepStats
	{
		PxU32 pairsTotal;
		PxU32 pairsWithContacts;
		PxU32 cacheHits;
		PxU32 newPairs;
		PxU32 lostPairs;
		PxU32 newTouches;
		PxU32 lostTouches;
		PxU32 activeConstraints;

		bool operator!=(const StepStats& o) const
		{
			return pairsTotal != o.pairsTotal
				|| pairsWithContacts != o.pairsWithContacts
				|| cacheHits != o.cacheHits
				|| newPairs != o.newPairs
				|| lostPairs != o.lostPairs
				|| newTouches != o.newTouches
				|| lostTouches != o.lostTouches
				|| activeConstraints != o.activeConstraints;
		}
	};

	StepStats ReadStats(const World& w)
	{
		PxSimulationStatistics s;
		w.scene->getSimulationStatistics(s);

		StepStats out;
		out.pairsTotal = s.nbDiscreteContactPairsTotal;
		out.pairsWithContacts = s.nbDiscreteContactPairsWithContacts;
		out.cacheHits = s.nbDiscreteContactPairsWithCacheHits;
		out.newPairs = s.nbNewPairs;
		out.lostPairs = s.nbLostPairs;
		out.newTouches = s.nbNewTouches;
		out.lostTouches = s.nbLostTouches;
		out.activeConstraints = s.nbActiveConstraints;
		return out;
	}

	void TestContactBookkeepingUnderVariableDepth(const Config& base, const char* label,
		int warmup, int frames)
	{
		std::printf("TestContactBookkeepingUnderVariableDepth [%s]\n", label);

		const int historyDepth = 32;

		World a, b;
		a.Build(base);
		b.Build(base);

		std::vector<std::vector<BodyState> > historyA(historyDepth);
		std::vector<std::vector<BodyState> > historyB(historyDepth);
		std::vector<BodyState> scratch;

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
			a.Capture(historyA[tick % historyDepth]);
			b.Capture(historyB[tick % historyDepth]);
		}

		std::printf("        tick   pairs  contacts  cached   new/lost pair   new/lost touch  constraints\n");

		int firstStatsDiffer = -1;
		int firstStateDiffers = -1;

		for (int frame = 0; frame < frames; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 11;
			const int depthB = 1 + (frame * 7) % 23;

			a.Restore(historyA[(tick - depthA) % historyDepth]);
			for (int t = tick - depthA + 1; t <= tick; ++t)
			{
				if (t != tick - depthA + 1) { a.Capture(scratch); a.Restore(scratch); }
				a.Step();
				a.Capture(historyA[t % historyDepth]);
			}
			const StepStats statsA = ReadStats(a);

			b.Restore(historyB[(tick - depthB) % historyDepth]);
			for (int t = tick - depthB + 1; t <= tick; ++t)
			{
				if (t != tick - depthB + 1) { b.Capture(scratch); b.Restore(scratch); }
				b.Step();
				b.Capture(historyB[t % historyDepth]);
			}
			const StepStats statsB = ReadStats(b);

			const bool statsDiffer = statsA != statsB;
			const bool stateDiffers =
				!StatesEqual(historyA[tick % historyDepth], historyB[tick % historyDepth]);

			if (statsDiffer && firstStatsDiffer < 0) { firstStatsDiffer = tick; }
			if (stateDiffers && firstStateDiffers < 0) { firstStateDiffers = tick; }

			// Print the run-up to the first state divergence and a few ticks past it.
			if (firstStateDiffers < 0 || tick <= firstStateDiffers + 2)
			{
				std::printf("        %4d   %3u/%-3u %3u/%-3u  %3u/%-3u   %2u,%-2u / %2u,%-2u   %2u,%-2u / %2u,%-2u   %3u/%-3u %s%s\n",
					tick,
					statsA.pairsTotal, statsB.pairsTotal,
					statsA.pairsWithContacts, statsB.pairsWithContacts,
					statsA.cacheHits, statsB.cacheHits,
					statsA.newPairs, statsA.lostPairs, statsB.newPairs, statsB.lostPairs,
					statsA.newTouches, statsA.lostTouches, statsB.newTouches, statsB.lostTouches,
					statsA.activeConstraints, statsB.activeConstraints,
					statsDiffer ? " stats" : "",
					stateDiffers ? " STATE" : "");
			}

			if (firstStateDiffers >= 0 && tick > firstStateDiffers + 2) { break; }
		}

		if (firstStatsDiffer < 0)
		{
			std::printf("        contact bookkeeping never differed\n");
		}
		else
		{
			std::printf("        bookkeeping first differs at tick %d, state at tick %d\n",
				firstStatsDiffer, firstStateDiffers);
		}

		Characterise(firstStatsDiffer >= 0 && firstStatsDiffer <= firstStateDiffers,
			std::string("contact bookkeeping diverges no later than state [") + label + "]");

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3n. Is it the broadphase's incremental state?
	//
	// At the diverging tick both peers step from bitwise identical bodies, and one of
	// them loses a broadphase pair while the other does not. Body state cannot explain
	// that; carried state can. PhysX's broadphases update incrementally from the
	// previous frame's bounds, and two peers that reached the same poses through 3
	// steps and through 23 steps did not present the same sequence of bounds.
	//
	// If that is the cause, the choice of broadphase should move the result, since the
	// four implementations differ precisely in how much they retain between frames.
	const char* BroadPhaseName(PxBroadPhaseType::Enum type)
	{
		switch (type)
		{
		case PxBroadPhaseType::eSAP:  return "eSAP  (sweep and prune, fully incremental)";
		case PxBroadPhaseType::eMBP:  return "eMBP  (multi box pruning)";
		case PxBroadPhaseType::eABP:  return "eABP  (automatic box pruning)";
		case PxBroadPhaseType::ePABP: return "ePABP (parallel ABP, the default)";
		default:                      return "unknown";
		}
	}

	void TestBroadPhaseSweep(const Config& base, const char* label, int warmup, int frames)
	{
		std::printf("TestBroadPhaseSweep [%s]\n", label);

		static const PxBroadPhaseType::Enum kTypes[] =
		{
			PxBroadPhaseType::eSAP,
			PxBroadPhaseType::eMBP,
			PxBroadPhaseType::eABP,
			PxBroadPhaseType::ePABP
		};
		const int typeCount = static_cast<int>(sizeof(kTypes) / sizeof(kTypes[0]));

		for (int i = 0; i < typeCount; ++i)
		{
			// eMBP needs explicit regions and silently degrades without them, so it is
			// named for completeness and skipped rather than measured wrongly.
			if (kTypes[i] == PxBroadPhaseType::eMBP)
			{
				std::printf("        %-42s skipped, needs explicit regions\n", BroadPhaseName(kTypes[i]));
				continue;
			}

			Config cfg = base;
			cfg.broadPhase = kTypes[i];

			int divergedAt = -1;
			const bool matched = RunVariableDepth(cfg, warmup, frames, &divergedAt, false);

			if (matched)
			{
				std::printf("        %-42s identical for %d frames\n", BroadPhaseName(kTypes[i]), frames);
			}
			else
			{
				std::printf("        %-42s diverged at frame %d (tick %d)\n",
					BroadPhaseName(kTypes[i]), divergedAt, warmup + divergedAt);
			}
		}
	}

	// -----------------------------------------------------------------------
	// 3o. If the pair set is the carried state, forcing it to be rebuilt should fix it.
	//
	// Every broadphase fails at the same tick, so the failure is not in any one
	// implementation; it is that the pair set is carried across the rewind at all. A
	// peer that rewinds to tick 162 still holds the pairs it had at tick 173, and then
	// updates them incrementally from there, so how far it rewound is an input to the
	// result. `resetFiltering` is the one public call that discards a pair and forces
	// rediscovery, which would make the set a function of state rather than of path.
	//
	// This is a diagnosis, not a recommendation -- see what it costs, further down.
	void TestPairRediscoveryClosesIt(const Config& base, const char* label, int warmup, int frames)
	{
		std::printf("TestPairRediscoveryClosesIt [%s]\n", label);

		Config withReset = base;
		withReset.resetFilteringOnRestore = true;

		int divergedAt = -1;
		const bool matched = RunVariableDepth(withReset, warmup, frames, &divergedAt, true);

		if (matched)
		{
			std::printf("        %d frames of differing rollback depth, still identical\n", frames);
		}
		else
		{
			std::printf("        diverged at frame %d of %d\n", divergedAt, frames);
		}

		Characterise(matched,
			std::string("rediscovering pairs on restore survives varying depth [") + label + "]");
	}

	// -----------------------------------------------------------------------
	// 3p. Which carried state is it, and does the impact have to be violent?
	//
	// resetFiltering did not close it, so the pair set is downstream of the cause
	// rather than the cause. The remaining carried candidates are the narrow phase's:
	// the persistent manifold PCM keeps alive across frames, and the contact cache.
	// Both are scene-creation flags, so they can only be swept by rebuilding.
	//
	// The second question is separate. The default scene drops the column 39 m and it
	// arrives at 27 m/s, penetrating up to half a body per step. If a gentle drop with
	// the same changing contact set is clean, the cause is the violence rather than
	// the bookkeeping; if it fails too, the reverse.
	void TestCarriedStateSweep(const Config& base, const char* label, int warmup, int frames)
	{
		std::printf("TestCarriedStateSweep [%s]\n", label);
		std::printf("        %-38s  %s\n", "variant", "result");

		struct Variant
		{
			const char* name;
			bool pcm;
			bool cache;
		};

		static const Variant kVariants[] =
		{
			{ "PCM on,  contact cache on (default)", true,  true  },
			{ "PCM on,  contact cache off",          true,  false },
			{ "PCM off, contact cache on",           false, true  },
			{ "PCM off, contact cache off",          false, false }
		};
		const int variantCount = static_cast<int>(sizeof(kVariants) / sizeof(kVariants[0]));

		for (int i = 0; i < variantCount; ++i)
		{
			Config cfg = base;
			cfg.persistentContactManifolds = kVariants[i].pcm;
			cfg.contactCache = kVariants[i].cache;

			int divergedAt = -1;
			const bool matched = RunVariableDepth(cfg, warmup, frames, &divergedAt, false);

			if (matched)
			{
				std::printf("        %-38s  identical for %d frames\n", kVariants[i].name, frames);
			}
			else
			{
				std::printf("        %-38s  diverged at frame %d (tick %d)\n",
					kVariants[i].name, divergedAt, warmup + divergedAt);
			}
		}
	}

	// -----------------------------------------------------------------------
	// 3q. Does the failure track the touch set changing?
	//
	// Everything so far says: not the broadphase implementation, not the pair set, not
	// PCM, not the contact cache, not the violence of the impact, not sleep. What is
	// left is when it happens. Every failing window is one in which contacts are being
	// created and destroyed, and every clean window is one in which the touch set has
	// gone quiet.
	//
	// This measures the two together, so the correlation is a number rather than an
	// impression: total touches found and lost during the run, against whether the
	// peers stayed identical.
	// Returns true if the peers stayed identical, so a caller that wants to assert on
	// the result can. Prints the touch churn either way, which is what makes the table
	// readable as evidence rather than a list of verdicts.
	bool TestChurnPredictsDivergence(const Config& base, const char* label, int warmup, int frames)
	{
		const int historyDepth = 32;

		if (warmup < kMaxRewindDepth) { warmup = kMaxRewindDepth; }

		World a, b;
		a.Build(base);
		b.Build(base);

		std::vector<std::vector<BodyState> > historyA(historyDepth);
		std::vector<std::vector<BodyState> > historyB(historyDepth);
		std::vector<BodyState> scratch;

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
			a.Capture(historyA[tick % historyDepth]);
			b.Capture(historyB[tick % historyDepth]);
		}

		PxU32 churn = 0;
		int divergedAt = -1;

		for (int frame = 0; frame < frames; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 11;
			const int depthB = 1 + (frame * 7) % 23;

			a.Restore(historyA[(tick - depthA) % historyDepth]);
			for (int t = tick - depthA + 1; t <= tick; ++t)
			{
				if (t != tick - depthA + 1) { a.Capture(scratch); a.Restore(scratch); }
				a.Step();
				a.Capture(historyA[t % historyDepth]);
			}

			b.Restore(historyB[(tick - depthB) % historyDepth]);
			for (int t = tick - depthB + 1; t <= tick; ++t)
			{
				if (t != tick - depthB + 1) { b.Capture(scratch); b.Restore(scratch); }
				b.Step();
				b.Capture(historyB[t % historyDepth]);
			}

			const StepStats s = ReadStats(a);
			churn += s.newTouches + s.lostTouches;

			if (divergedAt < 0 &&
				!StatesEqual(historyA[tick % historyDepth], historyB[tick % historyDepth]))
			{
				divergedAt = frame;
				// Keep going, so the churn figure covers the whole window either way.
			}
		}

		std::printf("        %-46s churn %5u   %s\n", label, churn,
			divergedAt < 0 ? "identical" : "DIVERGED");

		a.Destroy();
		b.Destroy();
		return divergedAt < 0;
	}

	// `expectExact` is false while PhysX's sleep bookkeeping is running. Pose and
	// velocity replay bitwise from any depth, but the wake counter does not, for the
	// reason section 3i measures, and that is not something the public API can fix.
	// The framework's answer is the never-sleep policy rather than a repair, so those
	// configurations are recorded rather than asserted.
	void TestVariableDepthUnderColdSteps(const Config& base, const char* label, int warmup, int frames,
		bool expectExact = true)
	{
		std::printf("TestVariableDepthUnderColdSteps [%s]\n", label);

		int divergedAt = -1;
		const bool matched = RunVariableDepth(base, warmup, frames, &divergedAt, true);

		if (matched)
		{
			std::printf("        %d frames of differing rollback depth, still identical\n", frames);
		}
		else
		{
			std::printf("        diverged at frame %d of %d\n", divergedAt, frames);
		}
		const std::string what =
			std::string("peers may roll back by different depths when steps are cold [") + label + "]";
		if (expectExact)
		{
			Check(matched, what);
		}
		else
		{
			Characterise(matched, what);
		}
	}

	// -----------------------------------------------------------------------
	// 3i. Why does one peer's wake counter reset a tick before the other's?
	//
	// At the point the two peers diverge both counters are well above the gate the
	// sleep bookkeeping is behind, so neither body is accumulating or resetting.
	// Both are in the plain decay path, wc = max(wc - dt, 0), one tick out of phase
	// from a reset that happened earlier. The counter is not drifting; a reset fired
	// at different times.
	//
	// PhysX picks the reset value as
	//
	//     wc = factor * 0.5 * wakeCounterResetTime + dt * (clusterFactor - 1)
	//
	// where clusterFactor is one plus the body's counted contact interactions, and
	// factor is pinned at 2 when the sleep threshold is zero. So a reset lands on
	// wakeCounterResetTime, plus exactly one dt for every counted interaction.
	//
	// The interaction count is internal and unreachable. The reset value is neither:
	// it is the wake counter, and it carries the count. Watching for the frames on
	// which the counter goes up, and reading the count back out of the value it went
	// up to, turns the standing hypothesis into a measurement without touching
	// PhysX at all.
	// -----------------------------------------------------------------------

	// Recovers the interaction count from a reset value. Exact rather than
	// approximate: the two are related by a whole number of timesteps.
	int ImpliedInteractionCount(PxReal resetValue)
	{
		const PxReal steps = (resetValue - kWakeCounterResetTime) / kDt;
		return static_cast<int>(steps < 0.0f ? steps - 0.5f : steps + 0.5f);
	}

	void TestWakeResetPhase(const Config& base, int warmup, int frames)
	{
		std::printf("TestWakeResetPhase\n");

		const int historyDepth = 32;
		const int totalTicks = warmup + frames;

		World a, b;
		a.Build(base);
		b.Build(base);

		const size_t bodyCount = a.bodies.size();

		std::vector<std::vector<BodyState> > historyA(historyDepth);
		std::vector<std::vector<BodyState> > historyB(historyDepth);
		std::vector<BodyState> scratch;

		// The wake counter of every body on every committed tick. Only the committed
		// value matters: intermediate replays of a tick are overwritten by the last
		// one, which is the value that tick is finally recorded as having.
		std::vector<std::vector<PxReal> > traceA(static_cast<size_t>(totalTicks));
		std::vector<std::vector<PxReal> > traceB(static_cast<size_t>(totalTicks));

		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
			a.Capture(historyA[tick % historyDepth]);
			b.Capture(historyB[tick % historyDepth]);

			traceA[static_cast<size_t>(tick)].resize(bodyCount);
			traceB[static_cast<size_t>(tick)].resize(bodyCount);
			for (size_t i = 0; i < bodyCount; ++i)
			{
				traceA[static_cast<size_t>(tick)][i] = historyA[tick % historyDepth][i].wakeCounter;
				traceB[static_cast<size_t>(tick)][i] = historyB[tick % historyDepth][i].wakeCounter;
			}
		}

		// The same varying rollback depths as the failing test, so this observes that
		// run rather than a different one. Divergence is recorded and the run
		// continues, because whatever caused it happened before it was visible.
		int divergedAt = -1;

		for (int frame = 0; frame < frames; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 11;
			const int depthB = 1 + (frame * 7) % 23;

			a.Restore(historyA[(tick - depthA) % historyDepth]);
			for (int t = tick - depthA + 1; t <= tick; ++t)
			{
				if (t != tick - depthA + 1) { a.Capture(scratch); a.Restore(scratch); }
				a.Step();
				a.Capture(historyA[t % historyDepth]);
			}

			b.Restore(historyB[(tick - depthB) % historyDepth]);
			for (int t = tick - depthB + 1; t <= tick; ++t)
			{
				if (t != tick - depthB + 1) { b.Capture(scratch); b.Restore(scratch); }
				b.Step();
				b.Capture(historyB[t % historyDepth]);
			}

			traceA[static_cast<size_t>(tick)].resize(bodyCount);
			traceB[static_cast<size_t>(tick)].resize(bodyCount);
			for (size_t i = 0; i < bodyCount; ++i)
			{
				traceA[static_cast<size_t>(tick)][i] = historyA[tick % historyDepth][i].wakeCounter;
				traceB[static_cast<size_t>(tick)][i] = historyB[tick % historyDepth][i].wakeCounter;
			}

			if (divergedAt < 0 && !StatesEqual(historyA[tick % historyDepth], historyB[tick % historyDepth]))
			{
				divergedAt = frame;
			}
		}

		std::printf("        state divergence first seen at frame %d (tick %d)\n",
			divergedAt, divergedAt < 0 ? -1 : warmup + divergedAt);

		// The first tick on which any body's counter differs, which is where the phase
		// parted rather than where it became visible.
		int firstTick = -1;
		size_t firstBody = 0;
		for (int t = 0; t < totalTicks && firstTick < 0; ++t)
		{
			for (size_t i = 0; i < bodyCount; ++i)
			{
				if (traceA[static_cast<size_t>(t)][i] != traceB[static_cast<size_t>(t)][i])
				{
					firstTick = t;
					firstBody = i;
					break;
				}
			}
		}

		if (firstTick < 0)
		{
			std::printf("        the two peers' wake counters agree on every tick\n");
		}
		else
		{
			std::printf("        wake counters first differ at tick %d, body %d\n",
				firstTick, static_cast<int>(firstBody));
			std::printf("        tick        peer A        peer B   note\n");

			const int from = firstTick > 4 ? firstTick - 4 : 0;
			for (int t = from; t < firstTick + 4 && t < totalTicks; ++t)
			{
				const PxReal va = traceA[static_cast<size_t>(t)][firstBody];
				const PxReal vb = traceB[static_cast<size_t>(t)][firstBody];

				// A reset is the only thing that raises the counter, and the value it
				// rises to encodes the body's counted interaction count.
				std::string note;
				if (t > 0)
				{
					const PxReal pa = traceA[static_cast<size_t>(t - 1)][firstBody];
					const PxReal pb = traceB[static_cast<size_t>(t - 1)][firstBody];
					char buffer[96];
					if (va > pa && vb > pb)
					{
						std::snprintf(buffer, sizeof(buffer), "both reset, %d and %d interactions",
							ImpliedInteractionCount(va), ImpliedInteractionCount(vb));
						note = buffer;
					}
					else if (va > pa)
					{
						std::snprintf(buffer, sizeof(buffer), "only A reset, %d interactions",
							ImpliedInteractionCount(va));
						note = buffer;
					}
					else if (vb > pb)
					{
						std::snprintf(buffer, sizeof(buffer), "only B reset, %d interactions",
							ImpliedInteractionCount(vb));
						note = buffer;
					}
				}

				std::printf("        %4d  %12.9g  %12.9g   %s%s\n", t, va, vb,
					note.c_str(), va != vb ? "  <-- differs" : "");
			}
		}

		Characterise(firstTick < 0, "both peers' wake counters agree on every tick");

		a.Destroy();
		b.Destroy();
	}

	// The trace above shows the two peers parting for exactly one tick and then
	// coming back together, which is not a drift and not a persistent phase offset.
	// It is one replayed tick disagreeing with the original pass of that same tick.
	//
	// So ask that directly, against one world and with no second peer involved:
	// simulate a stretch, then replay each tick of it from every rewind depth and
	// compare what comes out against what came out the first time. Pose and velocity
	// are compared alongside, because the interesting result is not that a replay
	// differs but that it differs *only* here.
	void TestReplayedTickMatchesOriginal(const Config& base, int warmup)
	{
		std::printf("TestReplayedTickMatchesOriginal\n");

		const int historyDepth = 32;
		const int maxDepth = 24;

		World world;
		world.Build(base);

		std::vector<std::vector<BodyState> > history(historyDepth);
		std::vector<BodyState> scratch;

		for (int tick = 0; tick < warmup; ++tick)
		{
			world.Capture(scratch);
			world.Restore(scratch);
			world.Step();
			world.Capture(history[tick % historyDepth]);
		}

		const int target = warmup - 1;
		const std::vector<BodyState>& original = history[target % historyDepth];

		int depthsWithWakeDifference = 0;
		int depthsWithPoseDifference = 0;
		int reported = 0;

		for (int depth = 1; depth <= maxDepth; ++depth)
		{
			// Replaying does not disturb the recorded history, and a restore erases
			// whatever the world did on the previous probe, so the probes are
			// independent of each other and of their own order.
			world.Restore(history[(target - depth) % historyDepth]);

			std::vector<BodyState> replayed = history[(target - depth) % historyDepth];
			std::vector<BodyState> previousReplayed;
			bool wakeDiffers = false;

			for (int t = target - depth + 1; t <= target; ++t)
			{
				if (t != target - depth + 1) { world.Capture(scratch); world.Restore(scratch); }
				world.Step();

				previousReplayed = replayed;
				world.Capture(replayed);

				// Every tick of the window is checked, not just the last one, so that
				// the report names the tick the two runs actually parted on rather
				// than the tick the difference was still visible at.
				const std::vector<BodyState>& before = history[(t - 1) % historyDepth];
				const std::vector<BodyState>& after = history[t % historyDepth];

				for (size_t i = 0; i < replayed.size() && reported < 6; ++i)
				{
					if (replayed[i].wakeCounter == after[i].wakeCounter)
					{
						continue;
					}

					// A reset is the only thing that raises the counter, and the value
					// it rises to is wakeCounterResetTime plus one dt per counted
					// contact interaction, so a reset that lands somewhere else says
					// how many interactions the body was judged to have.
					const bool resetOriginally = after[i].wakeCounter > before[i].wakeCounter;
					const bool resetOnReplay = replayed[i].wakeCounter > previousReplayed[i].wakeCounter;

					std::printf("        depth %2d, tick %d, body %2d: %.9g vs %.9g replayed",
						depth, t, static_cast<int>(i), after[i].wakeCounter, replayed[i].wakeCounter);
					if (resetOriginally && resetOnReplay)
					{
						std::printf("   reset both times, to %d and %d interactions\n",
							ImpliedInteractionCount(after[i].wakeCounter),
							ImpliedInteractionCount(replayed[i].wakeCounter));
					}
					else if (resetOriginally || resetOnReplay)
					{
						std::printf("   reset on only one of the two runs\n");
					}
					else
					{
						std::printf("   carried forward from an earlier tick\n");
					}
					++reported;
				}

				for (size_t i = 0; i < replayed.size(); ++i)
				{
					if (replayed[i].wakeCounter != after[i].wakeCounter) { wakeDiffers = true; }
				}
			}

			bool poseDiffers = false;
			for (size_t i = 0; i < replayed.size(); ++i)
			{
				const PxTransform& po = original[i].pose;
				const PxTransform& pr = replayed[i].pose;
				if (po.p.x != pr.p.x || po.p.y != pr.p.y || po.p.z != pr.p.z ||
					po.q.x != pr.q.x || po.q.y != pr.q.y || po.q.z != pr.q.z || po.q.w != pr.q.w ||
					original[i].linearVelocity != replayed[i].linearVelocity ||
					original[i].angularVelocity != replayed[i].angularVelocity)
				{
					poseDiffers = true;
				}
			}

			if (wakeDiffers) { ++depthsWithWakeDifference; }
			if (poseDiffers) { ++depthsWithPoseDifference; }
		}

		std::printf("        of %d rewind depths, %d reproduced tick %d's pose and velocity exactly,\n",
			maxDepth, maxDepth - depthsWithPoseDifference, target);
		std::printf("        and %d reproduced its wake counter\n", maxDepth - depthsWithWakeDifference);

		Check(depthsWithPoseDifference == 0, "a replayed tick reproduces the original pose and velocity");

		// The wake counter's reset value is wakeCounterResetTime plus one dt per
		// counted contact interaction, so a reset landing on a different value means
		// the body had a different number of counted interactions at that instant.
		// Those counts are maintained from touch-found and touch-lost transitions,
		// which are edges against the previous step's touch state, and a restore
		// leaves that state as whatever the world last had rather than as whatever it
		// had the first time through. The edge is therefore not a function of the
		// snapshot, and no snapshot the public API can build would make it one.
		//
		// Recorded rather than asserted, because the framework's answer is to stop
		// running this bookkeeping rather than to make it replay. See section 3j.
		Characterise(depthsWithWakeDifference == 0,
			"a replayed tick reproduces the original wake counter");

		world.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3g. The wake counter.
	//
	// It is readable and writable, and it is captured and restored like everything
	// else, so it ought to replay exactly. It does not: under differing rewind depths
	// it ends up one timestep out while pose and velocity stay bitwise identical.
	//
	// The suspicion is that the counter is an output of sleep bookkeeping rather than
	// the whole of it. PhysX accumulates velocity history to decide when a body may
	// sleep, and if those accumulators are not reset by a restore then the counter's
	// *trajectory* depends on how many steps the body has actually taken, not on the
	// value that was written into it.
	//
	// This replays the same tick from two different depths and prints both.
	// -----------------------------------------------------------------------

	void TestWakeCounterReplaysFromAnyDepth(const Config& base, int warmup)
	{
		std::printf("TestWakeCounterReplaysFromAnyDepth\n");

		const int shallow = 3;
		const int deep = 17;
		const int historyDepth = 32;

		World world;
		world.Build(base);

		std::vector<std::vector<BodyState> > history(historyDepth);
		std::vector<BodyState> scratch;

		for (int tick = 0; tick < warmup; ++tick)
		{
			world.Capture(scratch);
			world.Restore(scratch);
			world.Step();
			world.Capture(history[tick % historyDepth]);
		}

		const int target = warmup - 1;

		// Replay the target tick from a shallow rewind.
		world.Restore(history[(target - shallow) % historyDepth]);
		for (int t = target - shallow + 1; t <= target; ++t)
		{
			if (t != target - shallow + 1) { world.Capture(scratch); world.Restore(scratch); }
			world.Step();
		}
		std::vector<BodyState> fromShallow;
		world.Capture(fromShallow);

		// And from a deep one.
		world.Restore(history[(target - deep) % historyDepth]);
		for (int t = target - deep + 1; t <= target; ++t)
		{
			if (t != target - deep + 1) { world.Capture(scratch); world.Restore(scratch); }
			world.Step();
		}
		std::vector<BodyState> fromDeep;
		world.Capture(fromDeep);

		int poseDifferences = 0;
		int wakeDifferences = 0;
		for (size_t i = 0; i < fromShallow.size(); ++i)
		{
			const bool poseSame =
				fromShallow[i].pose.p.x == fromDeep[i].pose.p.x &&
				fromShallow[i].pose.p.y == fromDeep[i].pose.p.y &&
				fromShallow[i].pose.p.z == fromDeep[i].pose.p.z &&
				fromShallow[i].pose.q.x == fromDeep[i].pose.q.x &&
				fromShallow[i].pose.q.y == fromDeep[i].pose.q.y &&
				fromShallow[i].pose.q.z == fromDeep[i].pose.q.z &&
				fromShallow[i].pose.q.w == fromDeep[i].pose.q.w;
			if (!poseSame) { ++poseDifferences; }

			if (fromShallow[i].wakeCounter != fromDeep[i].wakeCounter)
			{
				if (wakeDifferences < 3)
				{
					std::printf("        body %d wake counter %.9g (depth %d) vs %.9g (depth %d), delta %.9g = %.3g steps\n",
						static_cast<int>(i),
						fromShallow[i].wakeCounter, shallow,
						fromDeep[i].wakeCounter, deep,
						fromDeep[i].wakeCounter - fromShallow[i].wakeCounter,
						(fromDeep[i].wakeCounter - fromShallow[i].wakeCounter) / kDt);
				}
				++wakeDifferences;
			}
		}

		std::printf("        %d of %d bodies differ in pose, %d in wake counter\n",
			poseDifferences, static_cast<int>(fromShallow.size()), wakeDifferences);

		Check(poseDifferences == 0, "pose replays identically from any rewind depth");
		Check(wakeDifferences == 0, "the wake counter replays identically from any rewind depth");

		world.Destroy();
	}

	// If the counter's trajectory really does depend on hidden sleep accumulators, then
	// a scene whose bodies can never sleep should not be affected, because the
	// bookkeeping never runs. That would make "disable sleeping" a complete answer
	// rather than a workaround.
	void TestWakeCounterWithSleepDisabled(const Config& base, int warmup)
	{
		std::printf("TestWakeCounterWithSleepDisabled\n");

		World world;
		world.Build(base);

		// A wake counter that never decays below the threshold cannot trigger the
		// sleep path at all.
		for (size_t i = 0; i < world.bodies.size(); ++i)
		{
			world.bodies[i]->setWakeCounter(PX_MAX_F32);
		}

		const int shallow = 3;
		const int deep = 17;
		const int historyDepth = 32;

		std::vector<std::vector<BodyState> > history(historyDepth);
		std::vector<BodyState> scratch;

		for (int tick = 0; tick < warmup; ++tick)
		{
			world.Capture(scratch);
			world.Restore(scratch);
			world.Step();
			world.Capture(history[tick % historyDepth]);
		}

		const int target = warmup - 1;

		world.Restore(history[(target - shallow) % historyDepth]);
		for (int t = target - shallow + 1; t <= target; ++t)
		{
			if (t != target - shallow + 1) { world.Capture(scratch); world.Restore(scratch); }
			world.Step();
		}
		std::vector<BodyState> fromShallow;
		world.Capture(fromShallow);

		world.Restore(history[(target - deep) % historyDepth]);
		for (int t = target - deep + 1; t <= target; ++t)
		{
			if (t != target - deep + 1) { world.Capture(scratch); world.Restore(scratch); }
			world.Step();
		}
		std::vector<BodyState> fromDeep;
		world.Capture(fromDeep);

		const bool identical = StatesEqual(fromShallow, fromDeep);
		if (!identical)
		{
			ReportFirstDifference(fromShallow, fromDeep);
		}
		std::printf("        replay from depth %d and depth %d: %s\n",
			shallow, deep, identical ? "identical" : "different");
		Check(identical, "with sleeping disabled, replay is depth-independent");

		world.Destroy();
	}

	// -----------------------------------------------------------------------
	// 3h. What does cold stepping cost?
	//
	// Warm starting exists because it helps the solver converge: seeding this frame's
	// impulses from last frame's is most of what keeps a stack from sinking into
	// itself. Throwing it away every step buys determinism, and the bill is paid in
	// contact quality.
	//
	// This settles a stack and measures how far it sags, warm against cold. It is a
	// measurement rather than a pass/fail: what counts as acceptable is a judgement
	// about the game, not about the physics.
	// -----------------------------------------------------------------------

	void MeasureColdStepCost(const Config& base, const char* label, int steps)
	{
		std::vector<BodyState> scratch;

		PxReal warmLowest = 0.0f;
		PxReal coldLowest = 0.0f;
		PxReal warmSpeed = 0.0f;
		PxReal coldSpeed = 0.0f;

		for (int cold = 0; cold < 2; ++cold)
		{
			World world;
			world.Build(base);

			for (int i = 0; i < steps; ++i)
			{
				if (cold != 0)
				{
					world.Capture(scratch);
					world.Restore(scratch);
				}
				world.Step();
			}

			world.Capture(scratch);

			// How far the stack settled, and how much it is still jittering once it
			// should have come to rest.
			PxReal lowest = PX_MAX_F32;
			PxReal fastest = 0.0f;
			for (size_t i = 0; i < scratch.size(); ++i)
			{
				lowest = PxMin(lowest, scratch[i].pose.p.y);
				fastest = PxMax(fastest, scratch[i].linearVelocity.magnitude());
			}

			if (cold == 0) { warmLowest = lowest; warmSpeed = fastest; }
			else { coldLowest = lowest; coldSpeed = fastest; }

			world.Destroy();
		}

		std::printf("        %-28s warm y %.6f, cold y %.6f, sag %+.6f m\n",
			label, warmLowest, coldLowest, coldLowest - warmLowest);
		std::printf("        %-28s warm |v| %.6f, cold |v| %.6f m/s\n",
			"", warmSpeed, coldSpeed);
	}

	// -----------------------------------------------------------------------
	// 4. Does the order actors were created in change the simulation?
	// -----------------------------------------------------------------------

	void TestCreationOrderAgrees(const Config& base, const char* label, int steps)
	{
		std::printf("TestCreationOrderAgrees [%s]\n", label);

		Config forward = base;
		forward.reverseOrder = false;
		Config reverse = base;
		reverse.reverseOrder = true;

		World a, b;
		a.Build(forward);
		b.Build(reverse);

		std::vector<BodyState> sa, sb;
		bool matched = true;
		int divergedAt = -1;

		for (int i = 0; i < steps; ++i)
		{
			a.Step();
			b.Step();
			a.Capture(sa);
			b.Capture(sb);
			if (!StatesEqual(sa, sb))
			{
				matched = false;
				divergedAt = i;
				ReportFirstDifference(sa, sb);
				break;
			}
		}

		if (matched)
		{
			std::printf("        %d steps, identical\n", steps);
		}
		else
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, steps);
		}
		// Expected to fail once contacts are involved: insertion order sets the solver's
		// processing order, and a different summation order rounds differently. The
		// framework's answer is to make every peer insert in stable-id order, not to
		// make PhysX order-independent.
		Characterise(matched, std::string("creation order does not change the simulation [") + label + "]");

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// 5. Does PhysX give the same actor the same internal identity regardless of
	//    the order it was added in?
	// -----------------------------------------------------------------------

	void TestInternalIndicesMatchAcrossOrder(const Config& base)
	{
		std::printf("TestInternalIndicesMatchAcrossOrder\n");

		Config forward = base;
		forward.reverseOrder = false;
		Config reverse = base;
		reverse.reverseOrder = true;

		World a, b;
		a.Build(forward);
		b.Build(reverse);

		// One step, so that anything assigned lazily on insertion has been assigned.
		a.Step();
		b.Step();

		bool actorIndicesMatch = true;
		bool nodeIndicesMatch = true;

		std::printf("        body  actorIndex(fwd/rev)  islandNode(fwd/rev)\n");
		for (size_t i = 0; i < a.bodies.size(); ++i)
		{
			const PxU32 actorA = a.bodies[i]->getInternalActorIndex();
			const PxU32 actorB = b.bodies[i]->getInternalActorIndex();
			const PxU64 nodeA = a.bodies[i]->getInternalIslandNodeIndex().index();
			const PxU64 nodeB = b.bodies[i]->getInternalIslandNodeIndex().index();

			if (actorA != actorB) { actorIndicesMatch = false; }
			if (nodeA != nodeB) { nodeIndicesMatch = false; }

			if (i < 8)
			{
				std::printf("        %4d  %10u/%-10u %10llu/%-10llu%s\n",
					static_cast<int>(i), actorA, actorB,
					static_cast<unsigned long long>(nodeA),
					static_cast<unsigned long long>(nodeB),
					(actorA != actorB || nodeA != nodeB) ? "   <-- differs" : "");
			}
		}

		// Both are assigned from insertion order, so they only line up across peers if
		// the peers inserted in the same order. This is the mechanism behind the test
		// above, and the reason stable-id ordered insertion is not optional.
		Characterise(actorIndicesMatch, "the same logical body gets the same internal actor index");
		Characterise(nodeIndicesMatch, "the same logical body gets the same island node index");

		a.Destroy();
		b.Destroy();
	}
}

int main()
{
	if (!StartPhysX())
	{
		std::printf("failed to start PhysX\n");
		return 1;
	}

	std::printf("\n=== rollback divergence bisection ===\n");
	std::printf("dt %.9g, %d bodies, origin (%g %g %g)\n\n",
		kDt, Config().bodyCount, kOrigin.x, kOrigin.y, kOrigin.z);

	// --- 1. baseline -----------------------------------------------------
	std::printf("--- is PhysX deterministic given identical construction? ---\n");
	{
		Config cfg;
		cfg.withGround = false;
		TestPairedScenesAgree(cfg, "free flight", 600);
	}
	{
		Config cfg;
		TestPairedScenesAgree(cfg, "with contacts", 600);
	}

	// --- 2. rollback without contacts ------------------------------------
	std::printf("\n--- is replay exact with no contacts in play? ---\n");
	{
		Config cfg;
		cfg.withGround = false;
		cfg.spin = true;
		TestRollbackIsTransparent(cfg, "free flight", 60, 30);
	}

	// --- 3. rollback with contacts ---------------------------------------
	std::printf("\n--- is replay exact once bodies are touching? ---\n");
	{
		Config cfg;
		// 60 steps of falling from 40 m does not reach the ground; 200 does, so by
		// the rewind point the stack is genuinely in contact.
		TestRollbackIsTransparent(cfg, "resting on ground", 200, 30, false);
	}

	// --- 3b. which part of the restore does the damage? ------------------
	std::printf("\n--- is it the history, or the restore call itself? ---\n");
	{
		Config cfg;
		TestRedundantRestoreIsHarmless(cfg, "restore nothing (control)", RestoreMask::eNothing, 200, 30);
		// Pose is the one that bites, and it drags "everything" down with it. Both are
		// recorded rather than asserted: this is the mechanism being demonstrated, not
		// a regression.
		TestRedundantRestoreIsHarmless(cfg, "pose only", RestoreMask::ePose, 200, 30, false);
		TestRedundantRestoreIsHarmless(cfg, "velocity only", RestoreMask::eVelocity, 200, 30);
		TestRedundantRestoreIsHarmless(cfg, "wake counter only", RestoreMask::eWakeCounter, 200, 30);
		TestRedundantRestoreIsHarmless(cfg, "clear forces only", RestoreMask::eClearForces, 200, 30);
		TestRedundantRestoreIsHarmless(cfg, "everything", RestoreMask::eEverything, 200, 30, false);
	}

	// --- 3c. mechanism, and whether removing it removes the divergence ---
	std::printf("\n--- why does an unchanged pose change anything? ---\n");
	{
		TestPoseRoundTripMechanism();
	}

	// --- 3e. can the cached contact state simply be turned off? ----------
	std::printf("\n--- does disabling contact persistence make rollback exact? ---\n");
	{
		const bool pcmOptions[2] = { true, false };
		const bool cacheOptions[2] = { true, false };
		for (int p = 0; p < 2; ++p)
		{
			for (int c = 0; c < 2; ++c)
			{
				Config cfg;
				cfg.persistentContactManifolds = pcmOptions[p];
				cfg.contactCache = cacheOptions[c];

				char label[96];
				std::snprintf(label, sizeof(label), "PCM %s, contact cache %s",
					pcmOptions[p] ? "on" : "off", cacheOptions[c] ? "on" : "off");

				TestRollbackIsTransparent(cfg, label, 200, 30, false);
			}
		}
	}
	{
		// The solver also warm-starts, independently of contact generation.
		Config cfg;
		cfg.solver = PxSolverType::eTGS;
		TestRollbackIsTransparent(cfg, "TGS solver", 200, 30, false);

		Config both;
		both.solver = PxSolverType::eTGS;
		both.persistentContactManifolds = false;
		both.contactCache = false;
		TestRollbackIsTransparent(both, "TGS solver, no contact persistence", 200, 30, false);
	}

	// --- 3d. is restore able to erase history? ---------------------------
	std::printf("\n--- can a restore erase history outright? ---\n");
	{
		Config cfg;
		TestRestoreMakesStepPure(cfg, "restore once", false, 600);
		TestRestoreMakesStepPure(cfg, "restore every step", true, 600);

		Config noCache;
		noCache.persistentContactManifolds = false;
		noCache.contactCache = false;
		TestRestoreMakesStepPure(noCache, "restore every step, no contact persistence", true, 600);
	}

	// --- 3f. does making every step cold buy transparency? ---------------
	std::printf("\n--- does restoring before every step make rollback transparent? ---\n");
	{
		Config cfg;
		TestColdStepsGiveTransparency(cfg, "PCM on", 200, 30);

		Config noPcm;
		noPcm.persistentContactManifolds = false;
		TestColdStepsGiveTransparency(noPcm, "PCM off", 200, 30);

		TestVariableDepthUnderColdSteps(cfg, "PhysX sleep bookkeeping active", 60, 600, false);
	}

	// --- 3k. does transparency survive the solver the framework actually uses? ----
	//
	// The result above was measured on a loose grid of boxes under PGS. Neither is
	// what the framework runs, and transparency is the property everything else would
	// rest on, so it has to be established for the harder cases rather than assumed to
	// carry over. Never-sleep throughout, to keep the wake counter of section 3i out
	// of the comparison.
	std::printf("\n--- does cold-step transparency survive a harder scene? ---\n");
	{
		Config grid;
		grid.neverSleep = true;
		TestColdStepsGiveTransparency(grid, "grid, PGS", 200, 30);

		// TGS never reaches transparency, by a few ulps of velocity on the very first
		// replayed step. It warm-starts from state the cold-step discipline does not
		// reach, and unlike the contact cache there is no restore that clears it.
		Config gridTgs = grid;
		gridTgs.solver = PxSolverType::eTGS;
		TestColdStepsGiveTransparency(gridTgs, "grid, TGS", 200, 30, false);

		// A single column, which is the case warm starting exists for: every body
		// carries the weight of everything above it, and the solve is a chain rather
		// than sixteen independent problems.
		Config tower = grid;
		tower.stack = true;
		tower.spin = false;
		TestColdStepsGiveTransparency(tower, "16-high stack, PGS", 200, 30);

		Config towerTgs = tower;
		towerTgs.solver = PxSolverType::eTGS;
		TestColdStepsGiveTransparency(towerTgs, "16-high stack, TGS", 200, 30, false);

		// The same column caught mid-impact rather than settled, which is where the
		// solver is at its most sensitive.
		TestColdStepsGiveTransparency(tower, "16-high stack landing, PGS", 170, 20);
	}

	// --- 3g. the wake counter ---------------------------------------------
	std::printf("\n--- does the wake counter replay from any depth? ---\n");
	{
		Config cfg;
		TestWakeCounterReplaysFromAnyDepth(cfg, 200);
		TestWakeCounterWithSleepDisabled(cfg, 200);
	}

	// --- 3i. where the wake counter's phase comes from --------------------
	std::printf("\n--- why does one peer reset the wake counter first? ---\n");
	{
		Config cfg;
		TestWakeResetPhase(cfg, 60, 600);
		TestReplayedTickMatchesOriginal(cfg, 200);
	}

	// --- 3j. does pinning the wake counter close the gap? -----------------
	std::printf("\n--- does never sleeping remove the last divergence? ---\n");
	{
		// The configuration section 3i's failure was measured in, with the one change
		// that closes it. This is the assertion that pinning the wake counter works.
		Config cfg;
		cfg.neverSleep = true;
		TestVariableDepthUnderColdSteps(cfg, "wake counter pinned, grid, PGS", 60, 600);

		// It does not follow that variable depth is safe generally, and it is not.
		// TGS is not transparent to begin with (3k), so it cannot survive peers
		// rewinding by different amounts, and a falling 16-high stack diverges even
		// under PGS: transparency holds tick by tick there, but the column amplifies
		// whatever it does not hold by, and an impact is where that shows.
		//
		// Recorded, because the fixed prediction horizon is what these are worked
		// around by, and it stays until they are understood.
		Config tgs = cfg;
		tgs.solver = PxSolverType::eTGS;
		TestVariableDepthUnderColdSteps(tgs, "wake counter pinned, grid, TGS", 60, 600, false);

		Config tower = cfg;
		tower.stack = true;
		tower.spin = false;
		TestVariableDepthUnderColdSteps(tower, "wake counter pinned, 16-high stack, PGS", 60, 600, false);

		Config towerTgs = tower;
		towerTgs.solver = PxSolverType::eTGS;
		TestVariableDepthUnderColdSteps(towerTgs, "wake counter pinned, 16-high stack, TGS", 60, 600, false);
	}

	// --- 3l. is the stack's failure about stacking, or about impact? ------
	std::printf("\n--- when in the stack's life does variable depth fail? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;
		TestVariableDepthPhaseSweep(tower, "16-high stack, PGS", 200);

		Config grid;
		grid.neverSleep = true;
		TestVariableDepthPhaseSweep(grid, "grid, PGS (control)", 200);
	}

	// --- 3m. what differs first, the contacts or the state? ---------------
	std::printf("\n--- does the touch set diverge before the bodies do? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;
		TestContactBookkeepingUnderVariableDepth(tower, "16-high stack, PGS", 165, 30);
	}

	// --- 3n. is the carried state the broadphase's? -----------------------
	std::printf("\n--- does the choice of broadphase move the failure? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;
		TestBroadPhaseSweep(tower, "16-high stack, PGS, through the impact", 60, 600);
	}

	// --- 3o. does forcing pair rediscovery close it? ----------------------
	std::printf("\n--- does rediscovering pairs on every restore fix the stack? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;
		TestPairRediscoveryClosesIt(tower, "16-high stack, PGS", 60, 600);

		Config grid;
		grid.neverSleep = true;
		TestPairRediscoveryClosesIt(grid, "grid, PGS", 60, 600);
	}

	// --- 3p. which carried state, and does it need a violent impact? ------
	std::printf("\n--- which carried state is it? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;
		TestCarriedStateSweep(tower, "16-high stack, PGS, 39 m drop", 60, 600);

		// The same column released just above the ground. Contacts are still created
		// and lost as it settles, but nothing arrives faster than about 1 m/s.
		Config gentle = tower;
		gentle.spawnHeight = 1.6f;
		TestCarriedStateSweep(gentle, "16-high stack, PGS, released at rest", 60, 600);
		TestVariableDepthPhaseSweep(gentle, "16-high stack, PGS, released at rest", 200);
	}

	// --- 3s. does it scale with the depth of the contact chain? -----------
	//
	// Not churn, not body-to-body contact on its own: a 4x4 slab of touching boxes is
	// one island with contacts being made, and it is clean. What the slab does not
	// have is depth. Every box in it rests on the ground directly, so nothing
	// propagates further than one contact, whereas the bottom of a 16-high column
	// carries fifteen bodies through fifteen contacts.
	//
	// If the divergence is being amplified rather than caused by the column, its onset
	// should move with the column's height.
	std::printf("\n--- does it scale with the height of the column? ---\n");
	{
		static const int kHeights[] = { 2, 4, 6, 8, 9, 10, 11, 12, 16 };
		const int heightCount = static_cast<int>(sizeof(kHeights) / sizeof(kHeights[0]));

		std::printf("        %-46s %-11s  %s\n", "scene and window", "touch churn", "result");
		for (int i = 0; i < heightCount; ++i)
		{
			Config column;
			column.neverSleep = true;
			column.stack = true;
			column.spin = false;
			column.spawnHeight = 1.6f;
			column.bodyCount = kHeights[i];

			char label[64];
			std::snprintf(label, sizeof(label), "%d-high column, window from tick 60", kHeights[i]);
			TestChurnPredictsDivergence(column, label, 60, 400);
		}

		// The short columns above have settled by tick 60, so their windows contain no
		// churn at all and height is confounded with opportunity. Starting the window
		// at release puts the whole of the settling inside it, so every height gets its
		// churn and only the height varies.
		static const int kSpotChecks[] = { 8, 10, 12 };
		const int spotCount = static_cast<int>(sizeof(kSpotChecks) / sizeof(kSpotChecks[0]));

		std::printf("\n");
		bool shallowHeld = true;
		for (int i = 0; i < spotCount; ++i)
		{
			Config column;
			column.neverSleep = true;
			column.stack = true;
			column.spin = false;
			column.spawnHeight = 1.6f;
			column.bodyCount = kSpotChecks[i];

			char label[64];
			std::snprintf(label, sizeof(label), "%d-high column, window from release", kSpotChecks[i]);
			const bool held = TestChurnPredictsDivergence(column, label, kMaxRewindDepth, 400);
			if (kSpotChecks[i] <= 8 && !held) { shallowHeld = false; }
		}

		// The useful half of the result, and the one worth noticing if it changes: a
		// contact chain of eight or fewer survives peers rewinding by different
		// amounts, in the scene and over the window where nine does not.
		Check(shallowHeld, "a contact chain up to 8 deep survives varying rollback depth");
	}

	// --- 3u. what is the measured contact chain depth? --------------------
	//
	// Everything above reads the chain depth off the column height by construction. This
	// closes the loop by measuring it from the contact graph PhysX actually built, so the
	// eight-deep limit becomes a number a diagnostic reports rather than one a comment
	// remembers. A settled column of height H must measure depth H; a flat grid, where
	// every box rests only on the ground, must measure depth 1. The same walk can run in a
	// game to enforce the limit on real content.
	std::printf("\n--- what is the measured contact chain depth? ---\n");
	{
		std::printf("        %-40s %-10s  %s\n", "settled scene", "measured", "expected");
		bool allMatched = true;

		static const int kColumns[] = { 2, 4, 8, 9, 12 };
		const int columnCount = static_cast<int>(sizeof(kColumns) / sizeof(kColumns[0]));
		for (int i = 0; i < columnCount; ++i)
		{
			Config column;
			column.measureContacts = true;
			column.neverSleep = true;
			column.stack = true;
			column.spin = false;
			column.spawnHeight = 1.6f;
			column.bodyCount = kColumns[i];

			World w;
			w.Build(column);
			for (int t = 0; t < 200; ++t) { w.Step(); }
			const int depth = MaxContactChainDepth(w);
			w.Destroy();

			char label[64];
			std::snprintf(label, sizeof(label), "%d-high column", kColumns[i]);
			std::printf("        %-40s %-10d  %d\n", label, depth, kColumns[i]);
			if (depth != kColumns[i]) { allMatched = false; }
		}

		// The flat grid: sixteen bodies, each resting only on the ground, so one contact
		// deep however many bodies there are.
		Config grid;
		grid.measureContacts = true;
		grid.neverSleep = true;
		grid.spin = false;
		grid.spawnHeight = 1.6f;

		World g;
		g.Build(grid);
		for (int t = 0; t < 200; ++t) { g.Step(); }
		const int gridDepth = MaxContactChainDepth(g);
		g.Destroy();
		std::printf("        %-40s %-10d  %d\n", "4x4 grid, 3 m apart", gridDepth, 1);
		if (gridDepth != 1) { allMatched = false; }

		Check(allMatched, "the contact-graph walk measures the chain depth each scene was built with");
	}

	// --- 3t. chain depth, or the load at the bottom of it? ----------------
	//
	// The threshold sits between 8 and 9 bodies. Two readings of that: the solve has
	// to propagate through 9 contacts, or the lowest body has to carry 8 others and
	// PGS conditions badly under load. A short column of heavy boxes separates them,
	// because it loads the bottom contact without lengthening the chain.
	std::printf("\n--- chain depth, or the load carried at the bottom? ---\n");
	{
		struct Case { int height; PxReal scale; const char* name; };
		static const Case kCases[] =
		{
			{ 4,  1.0f, "4-high, uniform          (load  3x, chain  4)" },
			{ 4, 10.0f, "4-high, upper bodies 10x (load 30x, chain  4)" },
			{ 4, 40.0f, "4-high, upper bodies 40x (load120x, chain  4)" },
			{ 8,  1.0f, "8-high, uniform          (load  7x, chain  8)" },
			{ 12, 1.0f, "12-high, uniform         (load 11x, chain 12)" }
		};
		const int caseCount = static_cast<int>(sizeof(kCases) / sizeof(kCases[0]));

		std::printf("        %-46s %-11s  %s\n", "scene and window", "touch churn", "result");
		for (int i = 0; i < caseCount; ++i)
		{
			Config column;
			column.neverSleep = true;
			column.stack = true;
			column.spin = false;
			column.spawnHeight = 1.6f;
			column.bodyCount = kCases[i].height;
			column.upperDensityScale = kCases[i].scale;

			TestChurnPredictsDivergence(column, kCases[i].name, 60, 400);
		}
	}

	// --- 3r. is it body-to-body contact rather than churn? ----------------
	//
	// The grid and the stack differ in exactly one structural way: in the grid every
	// contact is against the static ground, so the sixteen bodies are sixteen separate
	// islands and no island ever merges or splits. In the stack they are one island
	// whose membership changes as the column arrives. Narrowing the grid's spacing
	// until the boxes touch changes that and nothing else.
	std::printf("\n--- is it body-to-body contact, not churn? ---\n");
	{
		Config loose;
		loose.neverSleep = true;
		loose.spin = false;

		// Boxes are 1 m across, so this leaves a 5 mm gap: inside the default contact
		// offset, and touching once the slab settles. Sixteen bodies in one island,
		// every one of them resting on the ground, so the chain is one contact deep.
		//
		// Dropped from the default 39 m, so that the window from tick 60 covers the
		// same arrival and the same churn as the column that fails.
		Config tight = loose;
		tight.gridSpacing = 1.005f;

		std::printf("        %-46s %-11s  %s\n", "scene and window", "touch churn", "result");
		TestChurnPredictsDivergence(loose, "4x4 grid, 3 m apart, no body-to-body contact", 60, 400);
		TestChurnPredictsDivergence(tight, "4x4 slab, touching, one island of 16",         60, 400);
	}

	// --- 3q. does touch churn predict the failure? ------------------------
	std::printf("\n--- does the failure track contacts being made and broken? ---\n");
	{
		Config tower;
		tower.neverSleep = true;
		tower.stack = true;
		tower.spin = false;

		Config gentle = tower;
		gentle.spawnHeight = 1.6f;

		Config grid;
		grid.neverSleep = true;

		std::printf("        %-46s %-11s  %s\n", "scene and window", "touch churn", "result");
		TestChurnPredictsDivergence(tower,  "stack, 39 m drop, from tick 60 (in flight)",  60, 200);
		TestChurnPredictsDivergence(tower,  "stack, 39 m drop, from tick 165 (impact)",   165, 200);
		TestChurnPredictsDivergence(tower,  "stack, 39 m drop, from tick 300 (settled)",  300, 200);
		TestChurnPredictsDivergence(gentle, "stack, released at rest, from tick 60",       60, 200);
		TestChurnPredictsDivergence(gentle, "stack, released at rest, from tick 360",     360, 200);
		TestChurnPredictsDivergence(grid,   "grid, from tick 165 (its own impact)",       165, 200);
		TestChurnPredictsDivergence(grid,   "grid, from tick 300 (settled)",              300, 200);
	}

	// --- 3h. what does cold stepping cost? --------------------------------
	std::printf("\n--- what does cold stepping cost in contact quality? ---\n");
	{
		Config cfg;
		cfg.spin = false;   // let the stack settle rather than scatter
		MeasureColdStepCost(cfg, "after 300 steps", 300);
		MeasureColdStepCost(cfg, "after 900 steps", 900);

		Config tgs;
		tgs.spin = false;
		tgs.solver = PxSolverType::eTGS;
		MeasureColdStepCost(tgs, "TGS, after 900 steps", 900);

		// A 16-high stack, which is what warm starting is really for.
		Config tower;
		tower.spin = false;
		tower.stack = true;
		MeasureColdStepCost(tower, "16-high stack, PGS", 900);

		Config towerTgs = tower;
		towerTgs.solver = PxSolverType::eTGS;
		MeasureColdStepCost(towerTgs, "16-high stack, TGS", 900);
	}

	// --- 3v. Phase 1: representative workloads under both solvers ---------
	//
	// The solver decision (AdaptiveRollbackPlan section 4) turns on whether PGS holds
	// variable depth on the shapes the game actually has, not just on box grids and
	// columns. These are the ones a character game leans on: a capsule character resting
	// on terrain, and a high mass ratio, both under PGS (asserted) and TGS (recorded). The
	// jointed-pile case lives in PxwUndpwrTests as the articulation battery, which both
	// solvers pass. Vehicles cannot be measured until their integrator state is in the
	// snapshot (stage 3b).
	std::printf("\n--- Phase 1: representative workloads, both solvers ---\n");
	{
		// A single upright capsule settling onto the ground: one curved, near-point
		// contact, chain depth 1. This is the character-controller case.
		Config capsule;
		capsule.neverSleep = true;
		capsule.spin = false;
		capsule.capsule = true;
		capsule.bodyCount = 1;
		capsule.spawnHeight = 1.6f;
		TestVariableDepthUnderColdSteps(capsule, "capsule on terrain, PGS", 60, 600);

		Config capsuleTgs = capsule;
		capsuleTgs.solver = PxSolverType::eTGS;
		TestVariableDepthUnderColdSteps(capsuleTgs, "capsule on terrain, TGS", 60, 600, false);

		// High mass ratio: a 40x-density box resting on a normal one, chain depth 2. This
		// loads the bottom contact the way a heavy prop on a light platform would, without
		// lengthening the chain past the limit.
		Config heavy;
		heavy.neverSleep = true;
		heavy.spin = false;
		heavy.stack = true;
		heavy.bodyCount = 2;
		heavy.spawnHeight = 1.6f;
		heavy.upperDensityScale = 40.0f;
		TestVariableDepthUnderColdSteps(heavy, "40x mass ratio, 2-high, PGS", 60, 600);

		Config heavyTgs = heavy;
		heavyTgs.solver = PxSolverType::eTGS;
		TestVariableDepthUnderColdSteps(heavyTgs, "40x mass ratio, 2-high, TGS", 60, 600, false);
	}

	// --- 4. creation order -----------------------------------------------
	std::printf("\n--- does actor creation order matter? ---\n");
	{
		Config cfg;
		cfg.withGround = false;
		TestCreationOrderAgrees(cfg, "free flight", 300);
	}
	{
		Config cfg;
		TestCreationOrderAgrees(cfg, "with contacts", 300);
	}

	// --- 5. internal identity --------------------------------------------
	std::printf("\n--- does PhysX assign stable internal ids? ---\n");
	{
		Config cfg;
		TestInternalIndicesMatchAcrossOrder(cfg);
	}

	std::printf("\n%d checks, %d failures\n", gChecks, gFailures);

	StopPhysX();
	return gFailures == 0 ? 0 : 1;
}
