// ---------------------------------------------------------------------------
// PxwOffsetShapeRepro.cpp
//
// A standalone investigation into the spiked-ball desync.
//
// This links PhysX only. No plugin code is compiled in, so nothing in the
// wrapper -- not the stable-ID registry, not the state blob format, not the
// canonical pose handling, not PxwComputeMassProperties -- can influence what
// is measured here. If divergence shows up in this file it is PhysX, or the
// way PhysX is being driven, and nothing else.
//
// The one hypothesis under test: attaching sphere shapes at a translated
// setLocalPose and letting PhysX compute the compound mass produces a
// near-isotropic, off-centre mass frame. That mass frame makes the actor-pose
// round trip through setGlobalPose lossy, so a rollback -- which restores a
// body by writing its pose back -- diverges from the never-rolled-back
// reference, where a single-shape (or authored-isotropic) body stays exact.
//
// The file is arranged as a bisection. A plain sphere is the control and must
// stay exact in every check. Then, for the spiked ball, each knob is swept in
// isolation so that whichever one first flips exact -> diverges names the
// cause:
//
//   shapeMode  Plain vs Spiked            -- do the offset shapes matter at all
//   massMode   Computed vs Authored       -- is it the PhysX-computed mass frame
//   offset     translation vs rotation    -- COM offset or mass-frame rotation
//   spikeCount 0 / 1 / 6 / 24             -- how few offset shapes are enough
//
// Two checks are run per configuration, mirrored in miniature from
// PxwRollbackRepro:
//
//   PairedScenesAgree   two identically built scenes, stepped together. Within
//                       one process both run the same code, so this is a pure
//                       base-determinism check and is expected to hold for every
//                       configuration.
//   CrossPeerScenesAgree   two scenes built as the two peers would -- identical
//                       apart from the floating-point association of the
//                       compound-mass sum, a faithful stand-in for the last-bit
//                       differences a heterogeneous peer sees. This is the check
//                       that reproduces the desync.
//
// The controls (plain sphere, authored-isotropic ball) must agree across peers
// and are asserted. The computed-mass rows are *expected* to desync -- that is
// the bug -- so they are recorded rather than counted as failures, and the
// process exits clean when the world behaves exactly as diagnosed.
// ---------------------------------------------------------------------------

#include "PxPhysicsAPI.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>

using namespace physx;

namespace
{
	// -----------------------------------------------------------------------
	// Reporting
	// -----------------------------------------------------------------------

	int gChecks = 0;
	int gFailures = 0;

	// An invariant that must hold: the controls (plain sphere, authored-isotropic
	// ball) agree across peers, and base determinism holds. A failure here is a
	// genuine regression and fails the process.
	void Check(bool condition, const std::string& what)
	{
		++gChecks;
		if (!condition)
		{
			++gFailures;
		}
		std::printf("  %s  %s\n", condition ? "ok  " : "FAIL", what.c_str());
	}

	// An expected reproduction of the bug. Diverging is the whole point, so it is
	// recorded rather than counted as a failure: the process still exits clean when
	// the world behaves exactly as diagnosed. It only draws attention if a row that
	// was expected to reproduce the desync unexpectedly stops doing so.
	void Observe(bool diverged, const std::string& what)
	{
		std::printf("  %s  %s\n", diverged ? "REPRO" : "note ", what.c_str());
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
		gMaterial = gPhysics->createMaterial(2.0f, 2.0f, 0.0f);
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
	// Configuration
	// -----------------------------------------------------------------------

	const PxReal kDt = 1.0f / 60.0f;

	// Far from the origin, so that the float exponent is realistic rather than the
	// unusually forgiving range around zero.
	const PxVec3 kOrigin(500.0f, 0.0f, 500.0f);

	// The ball, matching SoccerBallEntity / SampleActors.CreateSpikedSphere.
	const PxReal kCoreRadius = 0.5f;
	const PxReal kBallDensity = 60.0f;
	const PxReal kSpikeRadiusFraction = 0.28f;
	const PxReal kSpikeProtrusionFraction = 0.45f;

	// The capsule pushers, matching SoccerPlayerEntity.
	const PxReal kCapsuleRadius = 0.45f;
	const PxReal kCapsuleHalfHeight = 0.5f;
	const PxReal kCapsuleDensity = 400.0f;

	enum ShapeMode
	{
		ePlain,   // one PxSphereGeometry at identity (the control)
		eSpiked   // core sphere + spikeCount offset spheres
	};

	enum MassMode
	{
		eComputed,          // PxRigidBodyExt::updateMassAndInertia (the compound path)
		eCollapsedFrame,    // computed, but the near-isotropic frame collapsed to identity
		                    // (the mitigation already in PxwComputeMassProperties)
		eAuthoredIsotropic  // closed-form isotropic mass, centred, identity frame
	};

	enum OffsetMode
	{
		eTranslation,   // spikes at dir*offset (the suspected trigger)
		eRotationOnly   // spikes at the origin, rotated only -- COM stays centred
	};

	struct BallConfig
	{
		ShapeMode shapeMode;
		int spikeCount;
		MassMode massMode;
		OffsetMode offsetMode;

		// Sum the shapes' mass contributions back to front when computing the
		// compound mass. The shapes themselves are still attached in the same order
		// on both peers -- real peers run identical source, so their shape order and
		// therefore their contact-generation order match. In exact arithmetic the
		// reassociated sum is identical; in floating point it rounds differently,
		// which is a faithful, in-process stand-in for the last-bit differences a
		// heterogeneous second peer sees from a different compiler, FPU, or math
		// library. This is the *only* difference between the two peers, so it
		// isolates the mass computation as the cause. Has no effect on an authored
		// mass, which never sums anything.
		bool reverseMassSum;

		// Stage H. A single per-body or per-shape property changed by the smallest amount
		// that changes it at all. None of these are carried in a snapshot -- they are the
		// body's construction, not its state -- so two peers that build them differently
		// stay bitwise identical on every hash the session compares and still solve
		// differently. 0 leaves the body exactly as the other peer builds it.
		int perturbation;

		BallConfig()
			: shapeMode(eSpiked)
			, spikeCount(24)
			, massMode(eComputed)
			, offsetMode(eTranslation)
			, reverseMassSum(false)
			, perturbation(0)
		{
		}
	};

	struct Scenario
	{
		BallConfig ball;
		bool squeeze;   // Stage B: two capsules drive into the ball each tick
		bool spin;      // give the ball an initial spin

		// Stage C rollback knobs, mirrored from PxwRollbackRepro. All three are
		// immutable scene state that a Restore does not rewrite, so they are the
		// obvious candidates for what a rewind fails to reproduce.
		bool persistentContactManifolds;   // PxSceneFlag::eENABLE_PCM, on by default
		bool contactCache;                  // inverse of eDISABLE_CONTACT_CACHE
		// Force PhysX to rediscover contact pairs on every restore instead of carrying
		// the existing set across a rewind. The one public lever that makes the pair set
		// a function of the current state rather than of the path taken to reach it.
		bool resetFilteringOnRestore;

		// Stage D knobs. PhysX assigns an actor index from scene-insertion order and the
		// solver sums contact impulses in that order, so two peers that register the same
		// bodies in a different order can round differently and drift the moment anything
		// touches -- a physics-only desync with the managed channels still agreeing, which
		// is exactly the demo's signature. SimRegistrationCheck exists to catch it.
		bool reverseRegistration;   // add actors to the scene back to front (the "other peer")
		int parkedFillers;          // extra eDISABLE_SIMULATION bodies, standing in for the
		                            // inactive pooled players that occupy actor indices

		// Turn on contact reports so a scenario can be shown to be exercising the
		// contacts it claims to, and so the demo's contact digest can be reproduced.
		bool reportContacts;

		// Stage F. A pool of parked slots that are registered, captured and restored the
		// way the framework treats an inactive pooled player, one of which is brought into
		// play part way through the run. Unparking clears eDISABLE_SIMULATION, which makes
		// PhysX build a fresh simulation object and allocate a fresh island node -- state
		// that is created at the moment the call happens, not derived from the snapshot,
		// and so is the one thing in the world whose identity depends on when a peer got
		// there rather than on what the state was.
		int poolSlots;
		int spawnAtFrame;   // confirmed frame the first pool slot enters play, -1 for never

		Scenario()
			: squeeze(false)
			, spin(true)
			, persistentContactManifolds(true)
			, contactCache(true)
			, resetFilteringOnRestore(false)
			, reverseRegistration(false)
			, parkedFillers(0)
			, reportContacts(false)
			, poolSlots(0)
			, spawnAtFrame(-1)
		{
		}
	};

	// -----------------------------------------------------------------------
	// Deterministic spike directions (Fibonacci lattice), matching
	// SampleActors.SpikeDirections so the native body is built the same way as
	// the one in the failing sample.
	// -----------------------------------------------------------------------

	std::vector<PxVec3> SpikeDirections(int count)
	{
		std::vector<PxVec3> dirs;
		if (count < 0)
		{
			count = 0;
		}
		dirs.reserve(static_cast<size_t>(count));
		const PxReal golden = PxPi * (3.0f - std::sqrt(5.0f));
		for (int i = 0; i < count; ++i)
		{
			const PxReal y = 1.0f - (static_cast<PxReal>(i) + 0.5f) / static_cast<PxReal>(count) * 2.0f;
			const PxReal r = std::sqrt(PxMax(0.0f, 1.0f - y * y));
			const PxReal theta = golden * static_cast<PxReal>(i);
			dirs.push_back(PxVec3(std::cos(theta) * r, y, std::sin(theta) * r));
		}
		return dirs;
	}

	// A unit direction turned into a rotation that maps +X onto it, for the
	// rotation-only control: the spike is placed at the actor origin but rotated,
	// so the compound mass frame can rotate without the centre of mass moving.
	PxQuat DirectionToRotation(const PxVec3& dir)
	{
		const PxVec3 from(1.0f, 0.0f, 0.0f);
		const PxVec3 to = dir.getNormalized();
		const PxVec3 axis = from.cross(to);
		const PxReal len = axis.magnitude();
		if (len < 1e-6f)
		{
			return PxQuat(PxIdentity);
		}
		const PxReal angle = std::acos(PxClamp(from.dot(to), -1.0f, 1.0f));
		return PxQuat(angle, axis / len);
	}

	// -----------------------------------------------------------------------
	// Body builders
	// -----------------------------------------------------------------------

	// The compound-mass path, computed explicitly so its floating-point
	// association can be controlled. This is what PxRigidBodyExt::updateMassAndInertia
	// does internally: form each shape's mass properties, sum them, then diagonalise
	// the summed inertia tensor to get the mass-frame orientation and the principal
	// moments. `reverse` flips only the summation order.
	// The isotropy tolerance below which PxwComputeMassProperties collapses the mass
	// frame. Matches the 5% figure its warning path uses.
	const PxReal kIsotropyTolerance = 0.05f;

	// One shape's contribution, paired so it can be put in a canonical order. Mirrors
	// the ShapeContribution the framework's PxwComputeMassProperties sorts on.
	struct MassContribution
	{
		PxMassProperties props;
		PxTransform pose;

		bool operator<(const MassContribution& other) const
		{
			return std::memcmp(this, &other, sizeof(MassContribution)) < 0;
		}
	};

	void ApplyComputedMass(PxRigidDynamic* body, bool reverse, bool collapseFrame)
	{
		const PxU32 count = body->getNbShapes();
		std::vector<PxShape*> shapes(count, static_cast<PxShape*>(NULL));
		body->getShapes(shapes.data(), count);

		std::vector<MassContribution> contributions;
		contributions.reserve(count);
		for (PxU32 i = 0; i < count; ++i)
		{
			MassContribution c;
			std::memset(&c, 0, sizeof(c));
			c.props = PxMassProperties(shapes[i]->getGeometry()) * kBallDensity;
			c.pose = shapes[i]->getLocalPose();
			contributions.push_back(c);
		}

		// The peers attach shapes in the same order; only the association of the sum
		// differs, standing in for cross-peer floating-point differences.
		if (reverse)
		{
			std::reverse(contributions.begin(), contributions.end());
		}

		// The framework path (collapseFrame) sums in a canonical order, exactly as
		// PxwComputeMassProperties does, so the reassociation above cannot change the
		// result: mass, centre of mass and inertia are all order-independent. The raw
		// path (eComputed) skips this, which is what leaves it exposed to the sum order.
		if (collapseFrame)
		{
			std::sort(contributions.begin(), contributions.end());
		}

		std::vector<PxMassProperties> props;
		std::vector<PxTransform> poses;
		props.reserve(count);
		poses.reserve(count);
		for (size_t i = 0; i < contributions.size(); ++i)
		{
			props.push_back(contributions[i].props);
			poses.push_back(contributions[i].pose);
		}

		const PxMassProperties total = PxMassProperties::sum(props.data(), poses.data(), count);
		PxQuat massFrame(PxIdentity);
		PxVec3 diagonal = PxMassProperties::getMassSpaceInertia(total.inertiaTensor, massFrame);
		PxVec3 centerOfMass = total.centerOfMass;

		if (collapseFrame)
		{
			// Exactly what PxwComputeMassProperties does for a near-isotropic body:
			// snap the frame to identity and the moments to their mean, since the
			// eigenvectors carry only noise, and -- the completing step -- snap a
			// near-origin centre of mass to the actor origin, since the summed COM is
			// otherwise still per-peer floating point and desyncs on its own.
			const PxReal largest = PxMax(diagonal.x, PxMax(diagonal.y, diagonal.z));
			const PxReal smallest = PxMin(diagonal.x, PxMin(diagonal.y, diagonal.z));
			const PxReal anisotropy = largest > 0.0f ? (largest - smallest) / largest : 0.0f;
			if (anisotropy <= kIsotropyTolerance)
			{
				const PxReal mean = (diagonal.x + diagonal.y + diagonal.z) / 3.0f;
				diagonal = PxVec3(mean, mean, mean);
				massFrame = PxQuat(PxIdentity);

				const PxReal radiusOfGyration = (mean > 0.0f && total.mass > 0.0f)
					? PxSqrt(mean / total.mass) : 0.0f;
				if (centerOfMass.magnitude() <= 0.001f * radiusOfGyration)
				{
					centerOfMass = PxVec3(0.0f);
				}
			}
		}

		body->setMass(total.mass);
		body->setMassSpaceInertiaTensor(diagonal);
		body->setCMassLocalPose(PxTransform(centerOfMass, massFrame));
	}

	// What DeterministicWorld.Register applies to every dynamic body it registers, via
	// PxwApplyDeterministicRigidDefaults. PhysX does not default to any of these, and the
	// framework's determinism is measured with them on, so the repro has to match or it is
	// not driving PhysX the way the demo does.
	void ApplyDeterministicDefaults(PxRigidDynamic* body)
	{
		body->setSolverIterationCounts(8, 2);   // SimConfig's hashed defaults
		body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
		body->setMaxDepenetrationVelocity(3.0f);
	}

	// The next representable float above x, used to make the smallest possible
	// construction difference between two peers.
	PxReal NextUp(PxReal x)
	{
		PxU32 bits;
		std::memcpy(&bits, &x, sizeof(bits));
		++bits;
		PxReal out;
		std::memcpy(&out, &bits, sizeof(out));
		return out;
	}

	PxRigidDynamic* MakeBall(const PxVec3& position, const BallConfig& cfg)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(position));

		// Perturbation 3 gives this peer its own material, one ULP of friction away from
		// the other peer's. Materials are shared construction, never snapshot state.
		PxMaterial* material = gMaterial;
		if (cfg.perturbation == 3)
		{
			material = gPhysics->createMaterial(NextUp(2.0f), 2.0f, 0.0f);
		}

		// The core sphere, always at identity: this alone is the plain control.
		PxRigidActorExt::createExclusiveShape(*body, PxSphereGeometry(kCoreRadius), *material);

		if (cfg.shapeMode == eSpiked)
		{
			const PxReal spikeRadius = kCoreRadius * kSpikeRadiusFraction;
			const PxReal spikeProtrusion = kCoreRadius * kSpikeProtrusionFraction;
			const PxReal centreDistance = kCoreRadius + spikeProtrusion - spikeRadius;

			const std::vector<PxVec3> dirs = SpikeDirections(cfg.spikeCount);
			for (size_t i = 0; i < dirs.size(); ++i)
			{
				PxShape* spike = PxRigidActorExt::createExclusiveShape(
					*body, PxSphereGeometry(spikeRadius), *material);

				if (cfg.offsetMode == eTranslation)
				{
					// Translation-only offset: the actual sample layout, and the
					// suspected trigger. The compound COM drifts off origin.
					PxVec3 offset = dirs[i] * centreDistance;
					if (cfg.perturbation == 1 && i == 0)
					{
						// One spike, one ULP. The smallest construction difference an
						// offset-shape compound can have between two peers.
						offset.x = NextUp(offset.x);
					}
					spike->setLocalPose(PxTransform(offset));
				}
				else
				{
					// Rotation-only control: same shapes, placed at the origin but
					// rotated. The COM stays centred; only the mass-frame orientation
					// can move. This separates "off-centre COM" from "rotated frame".
					spike->setLocalPose(PxTransform(PxVec3(0.0f), DirectionToRotation(dirs[i])));
				}
			}
		}

		if (cfg.massMode == eComputed || cfg.massMode == eCollapsedFrame)
		{
			// The ill-conditioned compound path: diagonalise the summed inertia
			// tensor and store the eigenvectors as the mass-frame orientation, plus
			// the summed centre of mass. eCollapsedFrame additionally snaps a
			// near-isotropic frame to identity, as the wrapper already does.
			ApplyComputedMass(body, cfg.reverseMassSum, cfg.massMode == eCollapsedFrame);
		}
		else
		{
			// The closed-form fix shipping in SampleActors: treat the ball as the
			// uniform sphere it is meant to be. Centred, identity frame, isotropic
			// inertia -- byte-identical on every peer and never touching PhysX's
			// compound integration.
			const PxReal spikeProtrusion = kCoreRadius * kSpikeProtrusionFraction;
			const PxReal effectiveRadius = kCoreRadius + spikeProtrusion * 0.5f;
			const PxReal volume = (4.0f / 3.0f) * PxPi * effectiveRadius * effectiveRadius * effectiveRadius;
			const PxReal mass = kBallDensity * volume;
			const PxReal inertiaScalar = 0.4f * mass * effectiveRadius * effectiveRadius;
			body->setMass(mass);
			body->setMassSpaceInertiaTensor(PxVec3(inertiaScalar, inertiaScalar, inertiaScalar));
			body->setCMassLocalPose(PxTransform(PxIdentity));
		}

		// Never let the ball sleep, so a sleep transition cannot mask the numerical
		// question being asked.
		body->setSleepThreshold(0.0f);
		ApplyDeterministicDefaults(body);

		// Perturbation 2 leaves the depenetration clamp at PhysX's default instead of the
		// framework's 3 m/s. The clamp only does anything while bodies are deeply
		// overlapping, so this is invisible until the ball is squeezed -- which is exactly
		// the shape of the demo's report.
		if (cfg.perturbation == 2)
		{
			body->setMaxDepenetrationVelocity(PX_MAX_F32);
		}
		return body;
	}

	// An upright capsule pusher, matching SoccerPlayerEntity: PhysX capsules lie
	// along local X, so a quarter turn about Z stands it on its rounded end.
	PxRigidDynamic* MakeCapsule(const PxVec3& position)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(position));
		PxShape* shape = PxRigidActorExt::createExclusiveShape(
			*body, PxCapsuleGeometry(kCapsuleRadius, kCapsuleHalfHeight), *gMaterial);
		shape->setLocalPose(PxTransform(PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))));
		PxRigidBodyExt::updateMassAndInertia(*body, kCapsuleDensity);
		body->setSleepThreshold(0.0f);
		ApplyDeterministicDefaults(body);
		return body;
	}

	// A parked pool slot: a dynamic body added to the scene but with simulation
	// disabled, exactly as the framework parks an inactive pooled player. It never
	// moves or generates a contact, but it does occupy an actor index, so it changes
	// which indices the active bodies receive -- the thing that must still line up
	// across peers. Placed well clear of the action so nothing depends on where it is.
	PxRigidDynamic* MakeFiller(const PxVec3& position)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(position));
		PxRigidActorExt::createExclusiveShape(*body, PxSphereGeometry(kCapsuleRadius), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*body, kCapsuleDensity);
		body->setSleepThreshold(0.0f);
		ApplyDeterministicDefaults(body);
		return body;
	}

	// -----------------------------------------------------------------------
	// Snapshot
	// -----------------------------------------------------------------------

	struct BodyState
	{
		PxTransform pose;
		PxVec3 lin;
		PxVec3 ang;
	};

	// -----------------------------------------------------------------------
	// Contact instrumentation
	//
	// Two jobs. First, proof that a scenario is actually exercising the condition
	// it claims to: a squeeze test that never generates a contact measures nothing,
	// and every "no divergence" result from it would be worthless. Second, the
	// demo's oracle, reproduced here -- the demo caught its fork as a contact digest
	// that changed while the contact count stayed the same.
	//
	// Two digests are kept, and the difference between them is the diagnosis. The
	// report-order digest hashes pairs in the order PhysX reports them, which follows
	// its internal contact-manager order; the sorted digest hashes the same pairs in a
	// canonical order. If the sorted digests agree while the report-order ones differ,
	// the peers found the same contacts and PhysX ordered them differently. If both
	// differ, the contacts themselves were solved differently. The demo only had the
	// sorted one, which is why it could not tell these apart.
	// -----------------------------------------------------------------------

	struct ContactRecord
	{
		int bodyA;          // logical index, -1 for the static ground
		int bodyB;
		PxU32 pointCount;
		PxReal impulse;     // summed contact impulse magnitude over the pair's points

		bool operator<(const ContactRecord& other) const
		{
			if (bodyA != other.bodyA) return bodyA < other.bodyA;
			if (bodyB != other.bodyB) return bodyB < other.bodyB;
			return pointCount < other.pointCount;
		}
	};

	PxU64 FnvAppend(PxU64 hash, const void* data, size_t bytes)
	{
		const unsigned char* p = static_cast<const unsigned char*>(data);
		for (size_t i = 0; i < bytes; ++i)
		{
			hash ^= static_cast<PxU64>(p[i]);
			hash *= 1099511628211ULL;
		}
		return hash;
	}

	struct World;

	class ContactRecorder : public PxSimulationEventCallback
	{
	public:
		ContactRecorder() : owner(NULL) {}

		const World* owner;
		std::vector<ContactRecord> records;

		void Clear() { records.clear(); }

		void onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 nbPairs) PX_OVERRIDE;

		void onConstraintBreak(PxConstraintInfo*, PxU32) PX_OVERRIDE {}
		void onWake(PxActor**, PxU32) PX_OVERRIDE {}
		void onSleep(PxActor**, PxU32) PX_OVERRIDE {}
		void onTrigger(PxTriggerPair*, PxU32) PX_OVERRIDE {}
		void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) PX_OVERRIDE {}

		// The order PhysX handed the pairs over, which follows its internal
		// contact-manager list.
		PxU64 ReportOrderDigest() const
		{
			PxU64 hash = 1469598103934665603ULL;
			for (size_t i = 0; i < records.size(); ++i)
			{
				hash = FnvAppend(hash, &records[i], sizeof(ContactRecord));
			}
			return hash;
		}

		// The same pairs in a canonical order, which is what the demo's digest did.
		PxU64 SortedDigest() const
		{
			std::vector<ContactRecord> sorted = records;
			std::sort(sorted.begin(), sorted.end());
			PxU64 hash = 1469598103934665603ULL;
			for (size_t i = 0; i < sorted.size(); ++i)
			{
				hash = FnvAppend(hash, &sorted[i], sizeof(ContactRecord));
			}
			return hash;
		}

		// Pair identity and ordering only, with the solved impulses left out, so a
		// difference here is the pair set or its order rather than the solve.
		PxU64 PairOrderDigest() const
		{
			PxU64 hash = 1469598103934665603ULL;
			for (size_t i = 0; i < records.size(); ++i)
			{
				hash = FnvAppend(hash, &records[i].bodyA, sizeof(int));
				hash = FnvAppend(hash, &records[i].bodyB, sizeof(int));
			}
			return hash;
		}
	};

	// Contact reports are off in PxDefaultSimulationFilterShader, so the instrumented
	// scenarios use this instead. Identical on both peers, so it cannot itself be a
	// source of asymmetry.
	PxFilterFlags ReportingFilterShader(
		PxFilterObjectAttributes attributes0, PxFilterData filterData0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1,
		PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
	{
		PX_UNUSED(attributes0); PX_UNUSED(attributes1);
		PX_UNUSED(filterData0); PX_UNUSED(filterData1);
		PX_UNUSED(constantBlock); PX_UNUSED(constantBlockSize);

		pairFlags = PxPairFlag::eCONTACT_DEFAULT
			| PxPairFlag::eNOTIFY_TOUCH_FOUND
			| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
			| PxPairFlag::eNOTIFY_CONTACT_POINTS;
		return PxFilterFlag::eDEFAULT;
	}

	// -----------------------------------------------------------------------
	// World
	// -----------------------------------------------------------------------

	struct World
	{
		PxScene* scene;
		PxRigidStatic* ground;
		PxRigidDynamic* ball;
		PxRigidDynamic* capsuleA;
		PxRigidDynamic* capsuleB;
		std::vector<PxRigidDynamic*> bodies;    // fixed logical order for capture/compare
		std::vector<PxRigidDynamic*> fillers;   // parked pool slots, never captured
		Scenario scen;
		ContactRecorder contacts;

		// Aligned with `bodies`. A parked slot has no simulation object, so only its pose
		// is captured and restored, exactly as CaptureRigid / RestoreRigid do.
		std::vector<unsigned char> parked;
		std::vector<size_t> pool;   // indices into `bodies` of the spawnable slots

		World() : scene(NULL), ground(NULL), ball(NULL), capsuleA(NULL), capsuleB(NULL) {}

		// The logical index used in a contact record, matching the capture order, with
		// -1 standing for the static ground.
		int IndexOf(const PxActor* actor) const
		{
			for (size_t i = 0; i < bodies.size(); ++i)
			{
				if (bodies[i] == actor)
				{
					return static_cast<int>(i);
				}
			}
			return -1;
		}

		void Build(const Scenario& s)
		{
			scen = s;

			PxSceneDesc desc(gPhysics->getTolerancesScale());
			desc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
			desc.cpuDispatcher = gDispatcher;
			desc.filterShader = scen.reportContacts ? ReportingFilterShader : PxDefaultSimulationFilterShader;
			if (scen.reportContacts)
			{
				contacts.owner = this;
				desc.simulationEventCallback = &contacts;
			}
			desc.broadPhaseType = PxBroadPhaseType::ePABP;
			// PGS, matching the framework: a networked SimConfig requires it, and it is
			// the only solver measured replay-transparent under the cold-step discipline.
			desc.solverType = PxSolverType::ePGS;
			desc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
			if (scen.persistentContactManifolds)
			{
				desc.flags |= PxSceneFlag::eENABLE_PCM;
			}
			else
			{
				desc.flags &= ~PxSceneFlags(PxSceneFlag::eENABLE_PCM);
			}
			if (!scen.contactCache)
			{
				desc.flags |= PxSceneFlag::eDISABLE_CONTACT_CACHE;
			}
			scene = gPhysics->createScene(desc);

			// A large floor with its top surface at y = kOrigin.y. Always added first, and
			// static, so it sits outside the dynamic actor-index ordering under test.
			ground = gPhysics->createRigidStatic(PxTransform(PxVec3(kOrigin.x, kOrigin.y - 1.0f, kOrigin.z)));
			PxRigidActorExt::createExclusiveShape(*ground, PxBoxGeometry(2000.0f, 1.0f, 2000.0f), *gMaterial);
			scene->addActor(*ground);

			const PxReal restHeight = kOrigin.y + kCoreRadius;

			// Build the dynamic bodies without adding them yet, so the scene-insertion
			// (registration) order can be chosen below.
			const PxReal ballHeight = scen.squeeze ? restHeight : (restHeight + 2.0f);
			ball = MakeBall(PxVec3(kOrigin.x, ballHeight, kOrigin.z), scen.ball);
			if (scen.spin)
			{
				ball->setAngularVelocity(PxVec3(0.93f, -0.51f, 0.69f));
				ball->setLinearVelocity(PxVec3(0.11f, 0.0f, -0.07f));
			}
			bodies.push_back(ball);
			parked.push_back(0);

			if (scen.squeeze)
			{
				const PxReal capsuleHeight = kOrigin.y + kCapsuleRadius + kCapsuleHalfHeight;
				const PxReal reach = 2.0f;
				capsuleA = MakeCapsule(PxVec3(kOrigin.x - reach, capsuleHeight, kOrigin.z));
				capsuleB = MakeCapsule(PxVec3(kOrigin.x + reach, capsuleHeight, kOrigin.z));
				bodies.push_back(capsuleA);
				bodies.push_back(capsuleB);
				parked.push_back(0);
				parked.push_back(0);
			}

			// Spawnable pool slots. Unlike the Stage D fillers these are registered bodies:
			// captured, restored and compared, which is how the framework treats a pooled
			// player that is merely parked rather than absent.
			for (int i = 0; i < scen.poolSlots; ++i)
			{
				PxRigidDynamic* slot = MakeCapsule(PxVec3(
					kOrigin.x + 100.0f + static_cast<PxReal>(i) * 3.0f, kOrigin.y + 1.0f, kOrigin.z + 100.0f));
				pool.push_back(bodies.size());
				bodies.push_back(slot);
				parked.push_back(1);
			}

			// Parked pool slots, parked well clear of the ball on the +Y/+X diagonal.
			for (int i = 0; i < scen.parkedFillers; ++i)
			{
				fillers.push_back(MakeFiller(PxVec3(
					kOrigin.x + 50.0f + static_cast<PxReal>(i) * 3.0f, kOrigin.y + 50.0f, kOrigin.z + 50.0f)));
			}

			// The registration order under test: the active bodies in logical order, then
			// the parked slots. reverseRegistration models the second peer building the same
			// set in the opposite order, so every body gets a different PhysX actor index
			// while the captured logical order (bodies) stays the same on both peers.
			std::vector<PxRigidDynamic*> insertion;
			insertion.insert(insertion.end(), bodies.begin(), bodies.end());
			insertion.insert(insertion.end(), fillers.begin(), fillers.end());
			if (scen.reverseRegistration)
			{
				std::reverse(insertion.begin(), insertion.end());
			}
			for (size_t i = 0; i < insertion.size(); ++i)
			{
				scene->addActor(*insertion[i]);
			}

			// Park the fillers only after they are in the scene: disabling simulation tears
			// down the sim object, so it must exist first. They keep their actor index but
			// never move or contact anything.
			for (size_t i = 0; i < fillers.size(); ++i)
			{
				fillers[i]->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true);
			}
			for (size_t i = 0; i < pool.size(); ++i)
			{
				bodies[pool[i]]->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, true);
			}
		}

		// Bring a parked pool slot into play, as the framework unparks a pooled player:
		// clear the flag, place it, and give it a clean velocity. PhysX builds a new
		// simulation object here and takes an island node index off its free list.
		void Unpark(size_t slot, const PxVec3& position, const PxVec3& velocity)
		{
			if (slot >= pool.size())
			{
				return;
			}
			const size_t index = pool[slot];
			if (!parked[index])
			{
				return;
			}

			PxRigidDynamic* body = bodies[index];
			body->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, false);
			body->setGlobalPose(PxTransform(position), false);
			body->setLinearVelocity(velocity, false);
			body->setAngularVelocity(PxVec3(0.0f), false);
			parked[index] = 0;
		}

		// PhysX's own identity for every registered body: the actor index the solver
		// orders by, and the island node index allocated when the simulation object was
		// built. Neither is carried in a snapshot, so if two peers disagree here they are
		// running structurally different scenes no restore can reconcile.
		PxU64 InternalIdDigest() const
		{
			PxU64 hash = 1469598103934665603ULL;
			for (size_t i = 0; i < bodies.size(); ++i)
			{
				const PxU32 actorIndex = bodies[i]->getInternalActorIndex();
				const PxU64 nodeIndex = bodies[i]->getInternalIslandNodeIndex().index();
				hash = FnvAppend(hash, &actorIndex, sizeof(actorIndex));
				hash = FnvAppend(hash, &nodeIndex, sizeof(nodeIndex));
			}
			return hash;
		}

		void ReportInternalIds(const char* label) const
		{
			std::printf("        %s internal ids:", label);
			for (size_t i = 0; i < bodies.size(); ++i)
			{
				std::printf(" [%d: actor %u node %llu]", static_cast<int>(i),
					bodies[i]->getInternalActorIndex(),
					static_cast<unsigned long long>(bodies[i]->getInternalIslandNodeIndex().index()));
			}
			std::printf("\n");
		}

		// Drive the pushers toward the ball each tick, so both "players touch the
		// ball" and it is squeezed inside a shared solver island. Purely a function
		// of the current state, so it stays deterministic and replay-safe.
		void DriveSqueeze()
		{
			const PxReal squeezeSpeed = 3.0f;
			const PxReal uprightStiffness = 40.0f;
			PxRigidDynamic* pushers[2] = { capsuleA, capsuleB };
			const PxVec3 ballPos = ball->getGlobalPose().p;
			for (int i = 0; i < 2; ++i)
			{
				PxRigidDynamic* c = pushers[i];
				const PxVec3 p = c->getGlobalPose().p;
				PxVec3 toBall(ballPos.x - p.x, 0.0f, ballPos.z - p.z);
				if (toBall.magnitudeSquared() > 1e-8f)
				{
					toBall.normalize();
				}
				const PxVec3 v = c->getLinearVelocity();
				c->setLinearVelocity(PxVec3(toBall.x * squeezeSpeed, v.y, toBall.z * squeezeSpeed));

				// Keep the capsule upright without teleporting it, as the sample does.
				const PxVec3 up = c->getGlobalPose().q.rotate(PxVec3(0.0f, 1.0f, 0.0f));
				const PxVec3 tiltAxis = up.cross(PxVec3(0.0f, 1.0f, 0.0f));
				c->setAngularVelocity(tiltAxis * uprightStiffness);
			}
		}

		void Step()
		{
			if (scen.squeeze)
			{
				DriveSqueeze();
			}
			contacts.Clear();
			scene->simulate(kDt);
			scene->fetchResults(true);
		}

		void Capture(std::vector<BodyState>& out) const
		{
			out.resize(bodies.size());
			for (size_t i = 0; i < bodies.size(); ++i)
			{
				out[i].pose = bodies[i]->getGlobalPose();
				if (parked[i])
				{
					// A parked slot has no simulation object, so its pose is all there is
					// to capture. Zeroing the rest keeps capture and restore symmetric.
					out[i].lin = PxVec3(0.0f);
					out[i].ang = PxVec3(0.0f);
					continue;
				}
				out[i].lin = bodies[i]->getLinearVelocity();
				out[i].ang = bodies[i]->getAngularVelocity();
			}
		}

		// Write a snapshot back onto the bodies, exactly as the rollback engine does:
		// pose, velocities, and a force/torque clear. This is all a public API can put
		// back; whatever solver or pair-discovery state PhysX also carries is not reached,
		// which is precisely the question Stage C asks.
		void Restore(const std::vector<BodyState>& in)
		{
			for (size_t i = 0; i < bodies.size() && i < in.size(); ++i)
			{
				// Legal on a parked slot, and the only part of its state that is.
				bodies[i]->setGlobalPose(in[i].pose, false);
				if (parked[i])
				{
					continue;
				}
				bodies[i]->setLinearVelocity(in[i].lin, false);
				bodies[i]->setAngularVelocity(in[i].ang, false);
				bodies[i]->clearForce(PxForceMode::eFORCE);
				bodies[i]->clearForce(PxForceMode::eIMPULSE);
				bodies[i]->clearTorque(PxForceMode::eFORCE);
				bodies[i]->clearTorque(PxForceMode::eIMPULSE);
			}

			if (scen.resetFilteringOnRestore)
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
			fillers.clear();
			parked.clear();
			pool.clear();
			ground = NULL;
			ball = NULL;
			capsuleA = NULL;
			capsuleB = NULL;
		}
	};

	void ContactRecorder::onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 nbPairs)
	{
		for (PxU32 i = 0; i < nbPairs; ++i)
		{
			const PxContactPair& pair = pairs[i];

			ContactRecord record;
			record.bodyA = owner != NULL ? owner->IndexOf(header.actors[0]) : -1;
			record.bodyB = owner != NULL ? owner->IndexOf(header.actors[1]) : -1;
			record.pointCount = pair.contactCount;
			record.impulse = 0.0f;

			PxContactPairPoint points[64];
			const PxU32 extracted = pair.extractContacts(points, 64);
			for (PxU32 k = 0; k < extracted; ++k)
			{
				record.impulse += points[k].impulse.magnitude();
			}

			records.push_back(record);
		}
	}

	// How many contacts a scenario actually generates, and on the ball specifically.
	// A squeeze test that reports zero here is not testing a squeeze.
	struct ContactCensus
	{
		int steps;
		int totalPairs;
		int ballPairs;
		int peakBallPairs;
		int stepsWithBallContact;

		ContactCensus() : steps(0), totalPairs(0), ballPairs(0), peakBallPairs(0), stepsWithBallContact(0) {}

		void Observe(const ContactRecorder& recorder)
		{
			++steps;
			totalPairs += static_cast<int>(recorder.records.size());
			int onBall = 0;
			for (size_t i = 0; i < recorder.records.size(); ++i)
			{
				if (recorder.records[i].bodyA == 0 || recorder.records[i].bodyB == 0)
				{
					++onBall;
				}
			}
			ballPairs += onBall;
			if (onBall > peakBallPairs) { peakBallPairs = onBall; }
			if (onBall > 0) { ++stepsWithBallContact; }
		}

		void Report(const char* label) const
		{
			std::printf("  %-46s %d steps, %.2f pairs/step, ball %.2f/step (peak %d), ball touched on %d%% of steps\n",
				label, steps,
				steps > 0 ? double(totalPairs) / steps : 0.0,
				steps > 0 ? double(ballPairs) / steps : 0.0,
				peakBallPairs,
				steps > 0 ? (100 * stepsWithBallContact) / steps : 0);
		}
	};

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

	void ReportFirstDifference(const std::vector<BodyState>& a, const std::vector<BodyState>& b)
	{
		for (size_t i = 0; i < a.size() && i < b.size(); ++i)
		{
			if (std::memcmp(&a[i], &b[i], sizeof(BodyState)) == 0)
			{
				continue;
			}

			const PxVec3 dp = b[i].pose.p - a[i].pose.p;
			const PxVec3 dv = b[i].lin - a[i].lin;
			const PxVec3 dw = b[i].ang - a[i].ang;
			const PxQuat& qa = a[i].pose.q;
			const PxQuat& qb = b[i].pose.q;

			std::printf("        first difference at body %d\n", static_cast<int>(i));
			std::printf("           position delta %.9g  (%.9g %.9g %.9g)\n",
				dp.magnitude(), dp.x, dp.y, dp.z);
			std::printf("           rotation delta (%.9g %.9g %.9g %.9g)\n",
				qb.x - qa.x, qb.y - qa.y, qb.z - qa.z, qb.w - qa.w);
			std::printf("           linVel   delta %.9g\n", dv.magnitude());
			std::printf("           angVel   delta %.9g\n", dw.magnitude());
			return;
		}
	}

	// -----------------------------------------------------------------------
	// A direct probe of the one mechanism under suspicion: is the actor-pose
	// round trip through the mass frame bitwise exact? getGlobalPose returns the
	// actor pose, but PhysX stores the body pose (centre of mass). setGlobalPose
	// converts back. For a non-identity mass frame that conversion is a quaternion
	// multiply and need not be its own inverse to the last bit.
	// -----------------------------------------------------------------------

	void ReportMassFrame(const char* label, const BallConfig& cfg)
	{
		PxRigidDynamic* body = MakeBall(PxVec3(kOrigin.x, kOrigin.y + 5.0f, kOrigin.z), cfg);

		const PxTransform cmass = body->getCMassLocalPose();
		const PxVec3 inertia = body->getMassSpaceInertiaTensor();
		const PxReal largest = PxMax(inertia.x, PxMax(inertia.y, inertia.z));
		const PxReal smallest = PxMin(inertia.x, PxMin(inertia.y, inertia.z));
		const PxReal anisotropy = largest > 0.0f ? (largest - smallest) / largest : 0.0f;

		// The round trip: read the actor pose, write it straight back, read it
		// again. Any drift is the mass frame failing to invert exactly.
		const PxTransform before = body->getGlobalPose();
		body->setGlobalPose(before, false);
		const PxTransform after = body->getGlobalPose();
		const PxVec3 dp = after.p - before.p;
		const PxReal dq = PxMax(PxMax(PxAbs(after.q.x - before.q.x), PxAbs(after.q.y - before.q.y)),
			PxMax(PxAbs(after.q.z - before.q.z), PxAbs(after.q.w - before.q.w)));
		const bool exact = std::memcmp(&before, &after, sizeof(PxTransform)) == 0;

		std::printf("  %-40s COM (%.6g %.6g %.6g) q (%.6g %.6g %.6g %.6g)\n",
			label, cmass.p.x, cmass.p.y, cmass.p.z, cmass.q.x, cmass.q.y, cmass.q.z, cmass.q.w);
		std::printf("  %-40s inertia (%.6g %.6g %.6g) anisotropy %.4f%%\n",
			"", inertia.x, inertia.y, inertia.z, double(anisotropy) * 100.0);
		std::printf("  %-40s pose round trip %s (dp %.3g, dq %.3g)\n",
			"", exact ? "EXACT" : "LOSSY", dp.magnitude(), dq);

		body->release();
	}

	// -----------------------------------------------------------------------
	// The crux. Build the body twice -- once as each peer would -- and report
	// whether the two peers so much as agree on the mass frame before a single
	// step is taken. The only difference between them is the order the identical
	// spikes were attached, which stands in for cross-peer float differences.
	// -----------------------------------------------------------------------

	void ReportPeerMassFrameAgreement(const char* label, BallConfig cfg)
	{
		BallConfig peerA = cfg; peerA.reverseMassSum = false;
		BallConfig peerB = cfg; peerB.reverseMassSum = true;

		PxRigidDynamic* a = MakeBall(PxVec3(kOrigin.x, kOrigin.y + 5.0f, kOrigin.z), peerA);
		PxRigidDynamic* b = MakeBall(PxVec3(kOrigin.x, kOrigin.y + 5.0f, kOrigin.z), peerB);

		const PxTransform ca = a->getCMassLocalPose();
		const PxTransform cb = b->getCMassLocalPose();
		const PxVec3 ia = a->getMassSpaceInertiaTensor();
		const PxVec3 ib = b->getMassSpaceInertiaTensor();

		const PxReal dCom = (cb.p - ca.p).magnitude();
		const PxReal dQuat = PxMax(PxMax(PxAbs(cb.q.x - ca.q.x), PxAbs(cb.q.y - ca.q.y)),
			PxMax(PxAbs(cb.q.z - ca.q.z), PxAbs(cb.q.w - ca.q.w)));
		const PxReal dInertia = (ib - ia).magnitude();
		const bool identical =
			std::memcmp(&ca, &cb, sizeof(PxTransform)) == 0 &&
			std::memcmp(&ia, &ib, sizeof(PxVec3)) == 0;

		std::printf("  %-40s peers %s (dCOM %.3g, dQuat %.3g, dInertia %.3g)\n",
			label, identical ? "AGREE" : "DIFFER", dCom, dQuat, dInertia);

		a->release();
		b->release();
	}

	// -----------------------------------------------------------------------
	// Check 1: two independently built scenes agree, stepped together.
	// -----------------------------------------------------------------------

	void PairedScenesAgree(const Scenario& scen, const char* label, int steps)
	{
		World a, b;
		a.Build(scen);
		b.Build(scen);

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

		if (!matched)
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, steps);
		}
		Check(matched, std::string("paired scenes agree      [") + label + "]");

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// Check 1b: the two-peers test. Two scenes built as the two peers would --
	// identical apart from the spike attachment order -- stepped together. This
	// is the one that reproduces the reported desync: a plain sphere or an
	// authored-isotropic ball agrees, but the PhysX-computed spiked ball, whose
	// mass frame is the eigenvectors of a near-isotropic tensor, diverges.
	// -----------------------------------------------------------------------

	void CrossPeerScenesAgree(const Scenario& scen, const char* label, int steps, bool mustAgree)
	{
		Scenario peerA = scen; peerA.ball.reverseMassSum = false;
		Scenario peerB = scen; peerB.ball.reverseMassSum = true;

		World a, b;
		a.Build(peerA);
		b.Build(peerB);

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

		if (!matched)
		{
			std::printf("        diverged at step %d of %d\n", divergedAt, steps);
		}

		if (mustAgree)
		{
			Check(matched, std::string("two peers agree          [") + label + "]");
		}
		else
		{
			Observe(!matched, std::string("two peers desync         [") + label + "]");
		}

		a.Destroy();
		b.Destroy();
	}

	// -----------------------------------------------------------------------
	// Check 2: variable-depth rollback across two identical-binary peers.
	//
	// This is the Stage C check, and the one that models the demo faithfully once
	// the mass frame is ruled out. Both peers run identical construction (no
	// reverseMassSum, so no per-peer float difference at all -- exactly what MPPM
	// virtual players and same-arch clients are), and both cold-step: every step,
	// forward or replayed, is preceded by a restore of the body's own state, so a
	// forward step and a replayed step are the same operation. The framework's PGS
	// path is only replay-transparent under this discipline.
	//
	// The single difference between the peers is rollback DEPTH. Each confirmed
	// tick, peer A rewinds by one amount and peer B by another, and both replay to
	// the same tick -- which is what distinguishes two networked peers, whose
	// mispredictions reach back by different amounts. If the confirmed states then
	// disagree, replay is not transparent for this island: the rollback engine
	// rewinds by "whatever depth a misprediction reaches" and only lands on the same
	// state a full re-simulation would when replay is bitwise transparent. With a
	// plain sphere this involves no spikes and no computed mass, so a divergence here
	// is the remaining demo desync, isolated to the rollback layer.
	//
	// Mirrors PxwRollbackRepro's TestContactBookkeepingUnderVariableDepth state check.
	// -----------------------------------------------------------------------

	bool TwoPeersVaryingDepth(const Scenario& scen, const char* label,
		int warmup, int frames, bool sameDepth, bool reverseB = false)
	{
		std::printf("  TwoPeersVaryingDepth [%s]%s%s\n", label,
			sameDepth ? " (equal depth)" : "", reverseB ? " (peer B reversed registration)" : "");
		const int historyDepth = 32;   // exceeds the deepest rewind (23) below

		// Stage D: peer B registers the same bodies in the opposite scene-insertion order,
		// so every actor gets a different PhysX actor index while the captured logical order
		// is unchanged. Everything else about the two peers is identical.
		Scenario scenA = scen;
		Scenario scenB = scen;
		if (reverseB)
		{
			scenB.reverseRegistration = !scenB.reverseRegistration;
		}

		World a, b;
		a.Build(scenA);
		b.Build(scenB);

		std::vector<std::vector<BodyState> > historyA(historyDepth);
		std::vector<std::vector<BodyState> > historyB(historyDepth);
		std::vector<BodyState> scratch;

		// Warm up under the cold-step discipline, recording each tick's state.
		int tick = 0;
		for (; tick < warmup; ++tick)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
			a.Capture(historyA[tick % historyDepth]);
			b.Capture(historyB[tick % historyDepth]);
		}

		int divergedAt = -1;
		std::vector<BodyState> lastA, lastB;

		for (int frame = 0; frame < frames && divergedAt < 0; ++frame, ++tick)
		{
			const int depthA = 1 + (frame * 3) % 11;
			const int depthB = sameDepth ? depthA : (1 + (frame * 7) % 23);

			// Peer A rewinds depthA ticks and replays to tick, cold-stepping each
			// replayed step after the first (the rewind is the restore for that one).
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

			const std::vector<BodyState>& ca = historyA[tick % historyDepth];
			const std::vector<BodyState>& cb = historyB[tick % historyDepth];
			if (!StatesEqual(ca, cb))
			{
				divergedAt = tick;
				lastA = ca;
				lastB = cb;
			}
		}

		if (divergedAt >= 0)
		{
			std::printf("        diverged at confirmed tick %d\n", divergedAt);
			ReportFirstDifference(lastA, lastB);
		}

		a.Destroy();
		b.Destroy();
		return divergedAt >= 0;
	}

	// -----------------------------------------------------------------------
	// Check 3: the faithful engine loop -- asymmetric prediction lead.
	//
	// Stage C varies how far each peer rewinds, and finds nothing: under the
	// cold-step discipline a replayed step is the same operation as a forward one,
	// so two peers that replay to the same tick land on the same state. That is the
	// property the rollback engine is built on, and it holds.
	//
	// But it is not what the engine actually does. Every frame RollbackEngine
	// advances the confirmed timeline by one cold restore-and-step, and then runs a
	// PREDICTION WINDOW of further cold steps past the confirmed frontier, whose
	// results are thrown away. The width of that window is the peer's lead over
	// confirmation -- a property of the network, not of the simulation. A host
	// confirming its own input leads by almost nothing; a client a hundred
	// milliseconds away leads by six ticks. So between one confirmed step and the
	// next, the two peers push PhysX through a different number of steps, into
	// different future states, and then both restore the same confirmed snapshot and
	// take what is meant to be the same step.
	//
	// Everything the snapshot carries is identical at that moment: pose, velocity,
	// sleep. If the confirmed step still disagrees, PhysX is carrying something out
	// of the discarded prediction window that the restore does not reach -- and that
	// is the demo's desync, because the demo's peers agreed on every hash right up
	// to the tick the contact digest forked.
	//
	// This is the check Stage C should have been.
	// -----------------------------------------------------------------------

	// How far past confirmation a peer predicts on a given frame. Constant leads model
	// two peers at different, steady RTTs; the jitter term models the window breathing
	// as packets arrive early or late, which is what a real session does.
	int LeadFor(int baseLead, bool jitter, int frame)
	{
		if (!jitter)
		{
			return baseLead;
		}
		const int wobble = (frame * 5) % 4;   // 0..3, deterministic
		const int lead = baseLead + wobble - 1;
		return lead < 0 ? 0 : lead;
	}

	bool TwoPeersWithPredictionLead(const Scenario& scen, const char* label,
		int warmup, int frames, int leadA, int leadB, bool jitter)
	{
		std::printf("  TwoPeersWithPredictionLead [%s]  lead A=%d B=%d%s\n",
			label, leadA, leadB, jitter ? " (jittered)" : "");

		World a, b;
		a.Build(scen);
		b.Build(scen);

		// The confirmed state of each peer: the only thing the two ever compare, and
		// the only thing either one restores from when advancing the confirmed tick.
		std::vector<BodyState> confirmedA, confirmedB, scratch;

		// Bring both peers to the same confirmed state. Identical work on both, so they
		// are bitwise equal entering the measured frames.
		for (int i = 0; i < warmup; ++i)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
		}
		a.Capture(confirmedA);
		b.Capture(confirmedB);

		// One peer's frame: advance the confirmed tick by a single cold restore-and-step,
		// then run a prediction window of `lead` further cold steps and discard it.
		// Mirrors RollbackEngine.AdvanceConfirmed followed by RunPredictionConditional.
		struct Frame
		{
			static void Run(World& w, std::vector<BodyState>& confirmed,
				std::vector<BodyState>& scratch, int lead)
			{
				w.Restore(confirmed);
				w.Step();
				w.Capture(confirmed);

				for (int t = 0; t < lead; ++t)
				{
					// The first prediction step restores the confirmed snapshot; each
					// later one restores the prediction it just produced. Never two
					// restores in a row, exactly as the engine documents.
					if (t == 0) { w.Restore(confirmed); }
					else { w.Capture(scratch); w.Restore(scratch); }
					w.Step();
				}
			}
		};

		int divergedAt = -1;
		for (int frame = 0; frame < frames && divergedAt < 0; ++frame)
		{
			Frame::Run(a, confirmedA, scratch, LeadFor(leadA, jitter, frame));
			Frame::Run(b, confirmedB, scratch, LeadFor(leadB, jitter, frame));

			if (!StatesEqual(confirmedA, confirmedB))
			{
				divergedAt = frame;
			}
		}

		if (divergedAt >= 0)
		{
			std::printf("        diverged at confirmed frame %d\n", divergedAt);
			ReportFirstDifference(confirmedA, confirmedB);
		}

		a.Destroy();
		b.Destroy();
		return divergedAt >= 0;
	}

	// -----------------------------------------------------------------------
	// Check 4: a pooled body entering play, under asymmetric prediction lead.
	//
	// Stages C and E both come back clean, and the contact census shows they were
	// genuinely squeezing the ball while they did, so PhysX really is a pure function
	// of the restored state: neither rewind depth nor prediction-window width can move
	// it. That exhausts the "state PhysX carries between steps" family of explanations
	// and leaves only state PhysX carries that is not per-step at all.
	//
	// Unparking is exactly that. Clearing eDISABLE_SIMULATION makes PhysX construct a
	// new simulation object and take an island node index off a free list, and the
	// value it gets depends on what else the scene has built and torn down, not on the
	// snapshot. Two peers that unpark the same player at the same tick but from
	// different points in their own step sequence can therefore end up with different
	// internal identities for the same body -- and Stage D already established that
	// different identities desync a shared island even with enhanced determinism on.
	//
	// The demo's evidence points straight here: the only two bodies that forked were
	// the ball and the capsule of the player who had just JOINED, while the long-lived
	// player's capsule and all fourteen still-parked slots stayed bit-identical.
	// -----------------------------------------------------------------------

	bool TwoPeersSpawnUnderLead(const Scenario& scen, const char* label,
		int warmup, int frames, int leadA, int leadB, bool jitter)
	{
		std::printf("  TwoPeersSpawnUnderLead [%s]  lead A=%d B=%d%s\n",
			label, leadA, leadB, jitter ? " (jittered)" : "");

		World a, b;
		a.Build(scen);
		b.Build(scen);

		std::vector<BodyState> confirmedA, confirmedB, scratch;

		for (int i = 0; i < warmup; ++i)
		{
			a.Capture(scratch); a.Restore(scratch); a.Step();
			b.Capture(scratch); b.Restore(scratch); b.Step();
		}
		a.Capture(confirmedA);
		b.Capture(confirmedB);

		// Where the joining player arrives: right against the ball, so it lands inside
		// the contested island rather than somewhere harmless.
		const PxVec3 spawnAt(kOrigin.x, kOrigin.y + kCapsuleRadius + kCapsuleHalfHeight, kOrigin.z - 1.6f);
		const PxVec3 spawnVelocity(0.0f, 0.0f, 2.5f);

		struct Frame
		{
			static void Run(World& w, std::vector<BodyState>& confirmed,
				std::vector<BodyState>& scratch, int lead, bool spawnNow,
				const PxVec3& spawnAt, const PxVec3& spawnVelocity)
			{
				w.Restore(confirmed);
				// Gameplay brings the player into play as part of the tick, so the unpark
				// happens between the restore and the step, where a step handler would put it.
				if (spawnNow)
				{
					w.Unpark(0, spawnAt, spawnVelocity);
				}
				w.Step();
				w.Capture(confirmed);

				for (int t = 0; t < lead; ++t)
				{
					if (t == 0) { w.Restore(confirmed); }
					else { w.Capture(scratch); w.Restore(scratch); }
					w.Step();
				}
			}
		};

		int divergedAt = -1;
		bool idsForked = false;
		int idsForkedAt = -1;

		for (int frame = 0; frame < frames && divergedAt < 0; ++frame)
		{
			const bool spawnNow = (scen.spawnAtFrame >= 0 && frame == scen.spawnAtFrame);

			Frame::Run(a, confirmedA, scratch, LeadFor(leadA, jitter, frame), spawnNow, spawnAt, spawnVelocity);
			Frame::Run(b, confirmedB, scratch, LeadFor(leadB, jitter, frame), spawnNow, spawnAt, spawnVelocity);

			if (!idsForked && a.InternalIdDigest() != b.InternalIdDigest())
			{
				idsForked = true;
				idsForkedAt = frame;
			}

			if (!StatesEqual(confirmedA, confirmedB))
			{
				divergedAt = frame;
			}
		}

		if (idsForked)
		{
			std::printf("        internal ids forked at frame %d\n", idsForkedAt);
			a.ReportInternalIds("peer A");
			b.ReportInternalIds("peer B");
		}
		else
		{
			std::printf("        internal ids agree throughout\n");
		}

		if (divergedAt >= 0)
		{
			std::printf("        diverged at confirmed frame %d\n", divergedAt);
			ReportFirstDifference(confirmedA, confirmedB);
		}

		a.Destroy();
		b.Destroy();
		return divergedAt >= 0;
	}

	// -----------------------------------------------------------------------
	// Check 5: is restore-and-step a pure function of the restored state?
	//
	// Stages C, E and F each model one particular way two peers' histories can differ,
	// and each comes back clean. Rather than keep guessing at histories, ask the
	// general question directly, because every desync theory in this family reduces to
	// it: if a restore followed by a step always lands on the same bytes no matter what
	// the scene did beforehand, then no difference in how two peers reached a confirmed
	// tick can possibly matter, and the whole "PhysX carries un-snapshottable residue"
	// explanation is dead regardless of which history is the real one.
	//
	// One reference result is computed from a clean world: restore S, step, record.
	// Then the same world is put through a deliberately hostile history -- free
	// running, teleporting every body a kilometre away so the broadphase drops every
	// pair and has to rediscover it, sleeping and waking, resetting filtering -- and
	// then asked to restore the very same S and step. Any difference is residue.
	// -----------------------------------------------------------------------

	const char* ScrambleName(int kind)
	{
		switch (kind)
		{
		case 0:  return "nothing (control)";
		case 1:  return "5 free steps";
		case 2:  return "teleported 1km away, 4 steps (all pairs lost)";
		case 3:  return "teleport, step, restore, step, teleport again";
		case 4:  return "slept and woken";
		case 5:  return "filtering reset";
		default: return "deep free run (40 steps)";
		}
	}

	void Scramble(World& w, int kind, const std::vector<BodyState>& s)
	{
		switch (kind)
		{
		case 0:
			break;
		case 1:
			for (int i = 0; i < 5; ++i) { w.Step(); }
			break;
		case 2:
		{
			// Far enough that every broadphase pair is dropped and has to be found again.
			std::vector<BodyState> away = s;
			for (size_t i = 0; i < away.size(); ++i) { away[i].pose.p.y += 1000.0f; }
			w.Restore(away);
			for (int i = 0; i < 4; ++i) { w.Step(); }
			break;
		}
		case 3:
		{
			std::vector<BodyState> away = s;
			for (size_t i = 0; i < away.size(); ++i) { away[i].pose.p.y += 1000.0f; }
			w.Restore(away); w.Step();
			w.Restore(s); w.Step();
			w.Restore(away); w.Step();
			break;
		}
		case 4:
			for (size_t i = 0; i < w.bodies.size(); ++i)
			{
				if (!w.parked[i] && w.bodies[i]->getScene() != NULL) { w.bodies[i]->putToSleep(); }
			}
			w.Step();
			for (size_t i = 0; i < w.bodies.size(); ++i)
			{
				if (!w.parked[i] && w.bodies[i]->getScene() != NULL) { w.bodies[i]->wakeUp(); }
			}
			w.Step();
			break;
		case 5:
			for (size_t i = 0; i < w.bodies.size(); ++i)
			{
				w.scene->resetFiltering(*w.bodies[i]);
			}
			w.Step();
			break;
		default:
			for (int i = 0; i < 40; ++i) { w.Step(); }
			break;
		}
	}

	bool RestoreIsPure(const Scenario& scen, const char* label, int warmup)
	{
		std::printf("  RestoreIsPure [%s]\n", label);

		World reference;
		reference.Build(scen);
		for (int i = 0; i < warmup; ++i)
		{
			std::vector<BodyState> scratch;
			reference.Capture(scratch); reference.Restore(scratch); reference.Step();
		}

		// S, the confirmed snapshot every trial restores, and the reference answer:
		// what one step from S produces in a world that did nothing else first.
		std::vector<BodyState> s;
		reference.Capture(s);
		reference.Restore(s);
		reference.Step();
		std::vector<BodyState> expected;
		reference.Capture(expected);
		reference.Destroy();

		bool anyImpure = false;
		for (int kind = 0; kind <= 6; ++kind)
		{
			World w;
			w.Build(scen);
			std::vector<BodyState> scratch;
			for (int i = 0; i < warmup; ++i)
			{
				w.Capture(scratch); w.Restore(scratch); w.Step();
			}

			Scramble(w, kind, s);

			w.Restore(s);
			w.Step();
			std::vector<BodyState> actual;
			w.Capture(actual);

			const bool pure = StatesEqual(expected, actual);
			if (!pure)
			{
				anyImpure = true;
			}
			std::printf("        %-46s %s\n", ScrambleName(kind), pure ? "pure" : "IMPURE");
			if (!pure)
			{
				ReportFirstDifference(expected, actual);
			}
			w.Destroy();
		}

		return anyImpure;
	}

	// -----------------------------------------------------------------------
	// Check 6: construction differences the snapshot never carries.
	//
	// Stage G leaves only one place a desync can come from. A confirmed step is a pure
	// function of the restored state, and the peers' restored states hash equal, so
	// what differs has to be something the step reads that the snapshot does not
	// write: the body's construction rather than its state. Shape offsets, material
	// coefficients, the depenetration clamp, the solver iteration counts -- all of them
	// are set once when the body is built, all of them are read by every solve, and
	// none of them appear in a snapshot or in any hash a session compares.
	//
	// A compound with two dozen offset shapes has far more of this surface than a
	// single sphere: twenty-five geometries, twenty-five local poses and twenty-five
	// material bindings, every one of which has to be built identically on every peer.
	//
	// What this measures is how much a difference has to be to matter, and when. Each
	// row perturbs one property by the smallest amount that changes it at all, and runs
	// it twice: once with the ball merely dropping and rolling, and once with it
	// squeezed between both pushers. A row that survives the drop and fails the squeeze
	// has the demo's exact signature -- fine until both players touch the ball.
	// -----------------------------------------------------------------------

	const char* PerturbationName(int kind)
	{
		switch (kind)
		{
		case 0:  return "none (control)";
		case 1:  return "one spike local pose, 1 ULP";
		case 2:  return "max depenetration velocity unclamped";
		case 3:  return "material friction, 1 ULP";
		default: return "unknown";
		}
	}

	// Two peers built from the same source apart from one construction property, run
	// forward in lockstep. No rollback and no prediction: this is the plainest possible
	// setting, so anything that shows up here is the construction alone.
	int FirstDivergence(const Scenario& scen, int perturbation, int steps)
	{
		Scenario peerA = scen; peerA.ball.perturbation = 0;
		Scenario peerB = scen; peerB.ball.perturbation = perturbation;

		World a, b;
		a.Build(peerA);
		b.Build(peerB);

		std::vector<BodyState> sa, sb;
		int divergedAt = -1;
		for (int i = 0; i < steps && divergedAt < 0; ++i)
		{
			a.Step();
			b.Step();
			a.Capture(sa);
			b.Capture(sb);
			if (!StatesEqual(sa, sb))
			{
				divergedAt = i;
			}
		}

		a.Destroy();
		b.Destroy();
		return divergedAt;
	}

	void ReportPerturbationSensitivity(const Scenario& base, int steps)
	{
		Scenario drop = base;    drop.squeeze = false;
		Scenario squeeze = base; squeeze.squeeze = true;

		std::printf("  %-38s %14s  %14s\n", "perturbation", "drop only", "squeezed");
		for (int kind = 0; kind <= 3; ++kind)
		{
			const int dropAt = FirstDivergence(drop, kind, steps);
			const int squeezeAt = FirstDivergence(squeeze, kind, steps);

			char dropText[32];
			char squeezeText[32];
			if (dropAt < 0) { std::snprintf(dropText, sizeof(dropText), "agrees"); }
			else { std::snprintf(dropText, sizeof(dropText), "step %d", dropAt); }
			if (squeezeAt < 0) { std::snprintf(squeezeText, sizeof(squeezeText), "agrees"); }
			else { std::snprintf(squeezeText, sizeof(squeezeText), "step %d", squeezeAt); }

			std::printf("  %-38s %14s  %14s\n", PerturbationName(kind), dropText, squeezeText);
		}
	}

	// -----------------------------------------------------------------------
	// Drivers
	// -----------------------------------------------------------------------

	const char* ShapeName(ShapeMode m) { return m == ePlain ? "plain" : "spiked"; }
	const char* MassName(MassMode m)
	{
		switch (m)
		{
		case eComputed: return "computed";
		case eCollapsedFrame: return "collapsed";
		default: return "authored";
		}
	}
	const char* OffsetName(OffsetMode m) { return m == eTranslation ? "translate" : "rotate"; }

	std::string Label(const BallConfig& cfg)
	{
		char buf[128];
		std::snprintf(buf, sizeof(buf), "%s x%d %s %s",
			ShapeName(cfg.shapeMode), cfg.spikeCount, MassName(cfg.massMode), OffsetName(cfg.offsetMode));
		return buf;
	}

	void RunBothChecks(const Scenario& scen, int steps)
	{
		const std::string label = Label(scen.ball);

		// A plain sphere, an authored-isotropic ball, and the collapsed-frame path (the
		// framework fix: identity frame plus a snapped centre of mass) all have a
		// peer-identical mass frame, so the two peers must agree. Only the raw computed
		// compound -- the unrepaired PhysX path -- is expected to desync: that is the
		// bug being reproduced, not a regression.
		const bool mustAgree =
			scen.ball.shapeMode == ePlain ||
			scen.ball.massMode == eAuthoredIsotropic ||
			scen.ball.massMode == eCollapsedFrame;

		// The control: identical construction must agree, proving base determinism.
		PairedScenesAgree(scen, label.c_str(), steps);
		// The discriminator: the two peers built the same body two ways.
		CrossPeerScenesAgree(scen, label.c_str(), steps, mustAgree);
	}
}

int main()
{
	if (!StartPhysX())
	{
		std::printf("failed to start PhysX\n");
		return 1;
	}

	std::printf("\n=== offset-shape desync repro ===\n");
	std::printf("dt %.9g, origin (%g %g %g)\n", kDt, kOrigin.x, kOrigin.y, kOrigin.z);

	// --- mass frame -------------------------------------------------------
	// Before any simulation, show what each construction does to the mass frame
	// and whether that frame makes the pose round trip lossy. This is the
	// mechanism the checks below then confirm end to end.
	std::printf("\n--- mass frame and pose round trip ---\n");
	{
		BallConfig plain; plain.shapeMode = ePlain; plain.spikeCount = 0;
		ReportMassFrame("plain sphere, computed", plain);

		BallConfig spiked; // spiked x24 computed translate
		ReportMassFrame("spiked x24, computed, translate", spiked);

		BallConfig rot = spiked; rot.offsetMode = eRotationOnly;
		ReportMassFrame("spiked x24, computed, rotate-only", rot);

		BallConfig authored = spiked; authored.massMode = eAuthoredIsotropic;
		ReportMassFrame("spiked x24, authored isotropic", authored);
	}

	std::printf("\n--- do two peers agree on the mass frame? ---\n");
	{
		BallConfig plain; plain.shapeMode = ePlain; plain.spikeCount = 0;
		ReportPeerMassFrameAgreement("plain sphere, computed", plain);

		BallConfig spiked;
		ReportPeerMassFrameAgreement("spiked x24, computed, translate", spiked);

		BallConfig rot = spiked; rot.offsetMode = eRotationOnly;
		ReportPeerMassFrameAgreement("spiked x24, computed, rotate-only", rot);

		BallConfig collapsed = spiked; collapsed.massMode = eCollapsedFrame;
		ReportPeerMassFrameAgreement("spiked x24, collapsed frame", collapsed);

		BallConfig authored = spiked; authored.massMode = eAuthoredIsotropic;
		ReportPeerMassFrameAgreement("spiked x24, authored isotropic", authored);
	}

	// --- Stage A: spiked ball alone on a floor (spin + drop) --------------
	std::printf("\n--- Stage A: ball spins and drops onto a floor ---\n");
	{
		// The control: a plain sphere must stay exact in both checks.
		Scenario control;
		control.ball.shapeMode = ePlain;
		control.ball.spikeCount = 0;
		RunBothChecks(control, 400);
	}
	{
		// Bisect massMode with the full 24-spike ball and translation offset.
		Scenario computed;
		computed.ball.massMode = eComputed;
		RunBothChecks(computed, 400);

		// The wrapper's existing mitigation: collapse the near-isotropic frame to
		// identity but keep the summed COM. The sample's comment says this was not
		// enough; this row is where that claim is put to the test.
		Scenario collapsed;
		collapsed.ball.massMode = eCollapsedFrame;
		RunBothChecks(collapsed, 400);

		Scenario authored;
		authored.ball.massMode = eAuthoredIsotropic;
		RunBothChecks(authored, 400);
	}
	{
		// Bisect offset: translation vs rotation-only, both with computed mass, to
		// separate an off-centre COM from a rotated mass frame.
		Scenario rot;
		rot.ball.massMode = eComputed;
		rot.ball.offsetMode = eRotationOnly;
		RunBothChecks(rot, 400);
	}
	{
		// Bisect spikeCount: how few offset shapes are enough to flip it.
		const int counts[] = { 1, 6, 24 };
		for (int i = 0; i < 3; ++i)
		{
			Scenario s;
			s.ball.massMode = eComputed;
			s.ball.spikeCount = counts[i];
			RunBothChecks(s, 400);
		}
	}

	// --- Stage B: two capsules squeeze the ball on the floor --------------
	std::printf("\n--- Stage B: two pushers squeeze the ball ---\n");
	{
		Scenario control;
		control.squeeze = true;
		control.ball.shapeMode = ePlain;
		control.ball.spikeCount = 0;
		RunBothChecks(control, 400);

		Scenario computed;
		computed.squeeze = true;
		computed.ball.massMode = eComputed;
		RunBothChecks(computed, 400);

		Scenario authored;
		authored.squeeze = true;
		authored.ball.massMode = eAuthoredIsotropic;
		RunBothChecks(authored, 400);
	}

	// --- Stage C: variable-depth rollback while the ball is contested ------
	// Everything above steps only forward: it can find a mass-frame difference
	// between heterogeneous peers, but not a rollback bug, because it never rolls
	// back. This stage does. Two identical-binary peers (no float difference at all)
	// rewind to different depths each confirmed tick and replay under the cold-step
	// discipline, while two pushers squeeze the ball. The ball here is a PLAIN sphere
	// unless noted, so any divergence is the rollback/replay layer alone -- no spikes,
	// no computed mass. This is the faithful model of the "both players touch the ball"
	// desync on MPPM / same-arch clients.
	std::printf("\n--- Stage C: variable-depth rollback while two pushers squeeze the ball ---\n");
	{
		Scenario plain;
		plain.squeeze = true;
		plain.ball.shapeMode = ePlain;
		plain.ball.spikeCount = 0;

		// Control: when both peers rewind to the SAME depth, replay is symmetric and
		// they must agree. This proves rollback itself is transparent here and pins any
		// failure below on the depth difference, not on rolling back at all.
		const bool equalDepthDiverged = TwoPeersVaryingDepth(plain, "plain, equal depth (control)", 120, 120, true);
		Check(!equalDepthDiverged, "equal-depth rollback agrees   [plain, PCM on]");

		// The discriminator: differing rewind depths, the demo's actual condition.
		const bool carryDiverged = TwoPeersVaryingDepth(plain, "plain, differing depth, carry pairs", 120, 120, false);
		Observe(carryDiverged, "varying-depth rollback desyncs [plain, PCM on, carry pairs]");

		// Rediscover contact pairs on every restore: the pair set carried across a
		// rewind was PxwRollbackRepro's culprit. If this removes the divergence, that is
		// the mechanism and resetFiltering-on-rollback is the lever.
		Scenario reset = plain;
		reset.resetFilteringOnRestore = true;
		const bool resetDiverged = TwoPeersVaryingDepth(reset, "plain, differing depth, reset filtering", 120, 120, false);
		Observe(resetDiverged, "varying-depth rollback desyncs [plain, PCM on, reset filtering]");

		// Turn off both contact-persistence caches, the other thing a restore cannot
		// reach.
		Scenario noPersist = plain;
		noPersist.persistentContactManifolds = false;
		noPersist.contactCache = false;
		const bool noPersistDiverged = TwoPeersVaryingDepth(noPersist, "plain, differing depth, no persistence", 120, 120, false);
		Observe(noPersistDiverged, "varying-depth rollback desyncs [plain, no contact persistence]");

		// The full spiked + computed ball, for completeness: if the plain sphere already
		// diverges, the spikes were never the cause of this layer.
		Scenario spiked = plain;
		spiked.ball.shapeMode = eSpiked;
		spiked.ball.spikeCount = 24;
		spiked.ball.massMode = eComputed;
		const bool spikedDiverged = TwoPeersVaryingDepth(spiked, "spiked computed, differing depth", 120, 120, false);
		Observe(spikedDiverged, "varying-depth rollback desyncs [spiked computed]");
	}

	// --- Stage D: registration order across peers with a parked pool -------
	// The demo's remaining desync is physics-only (its entity and game channels agree),
	// which rules out the managed pool/roster logic and points at native state. With the
	// mass frame fixed, framework sleeping off (SleepTicks = 0) and rollback shown
	// transparent above, the one native mechanism left that fits "physics-only, confined
	// to the contacting bodies, invisible until they touch" is registration order: PhysX
	// numbers actors by scene-insertion order and sums contact impulses in that order, so
	// two peers that register the same bodies differently round differently on first
	// contact. This stage builds peer B with the reversed insertion order (and a parked
	// pool occupying actor indices, as the inactive players do) and asks whether that
	// alone forks the squeeze, with enhanced determinism on. If it agrees, order is
	// eliminated too and the demo fork must enter at the rebuild itself (compare each
	// peer's "Rebuild complete; state hash" and the SimRegistrationCheck line).
	std::printf("\n--- Stage D: differing registration order, parked pool, squeeze ---\n");
	{
		Scenario pooled;
		pooled.squeeze = true;
		pooled.ball.shapeMode = ePlain;
		pooled.ball.spikeCount = 0;
		pooled.parkedFillers = 14;   // ~ the inactive soccer player slots

		// Control: identical order on both peers (same depths too) must agree, proving the
		// parked pool by itself changes nothing.
		const bool sameOrderDiverged = TwoPeersVaryingDepth(pooled, "pooled, same order (control)", 120, 120, true, false);
		Check(!sameOrderDiverged, "same registration order agrees [pooled, equal depth]");

		// The discriminator: peer B registers every body in the opposite order, so the ball
		// and both pushers get different PhysX actor indices. Lockstep first (equal depth),
		// to isolate registration order from rollback depth.
		const bool revLockstepDiverged = TwoPeersVaryingDepth(pooled, "pooled, reversed order, lockstep", 120, 120, true, true);
		Observe(revLockstepDiverged, "reversed registration desyncs  [pooled, equal depth]");

		// Then reversed order together with differing rollback depth, the full demo
		// condition: a mid-match joiner that built the world in a different order and rewinds
		// by a different amount.
		const bool revVaryingDiverged = TwoPeersVaryingDepth(pooled, "pooled, reversed order, varying depth", 120, 120, false, true);
		Observe(revVaryingDiverged, "reversed registration desyncs  [pooled, varying depth]");

		// And with the real spiked + computed ball, for completeness.
		Scenario spikedPooled = pooled;
		spikedPooled.ball.shapeMode = eSpiked;
		spikedPooled.ball.spikeCount = 24;
		spikedPooled.ball.massMode = eComputed;
		const bool spikedRevDiverged = TwoPeersVaryingDepth(spikedPooled, "spiked pooled, reversed order, varying depth", 120, 120, false, true);
		Observe(spikedRevDiverged, "reversed registration desyncs  [spiked pooled, varying depth]");
	}

	// --- Contact census: is the squeeze scenario exercising anything? -------
	// Every "no divergence" result above is only worth as much as the contact load
	// behind it. If the pushers never actually reach the ball, the rollback stages
	// were measuring an empty scene and proved nothing.
	std::printf("\n--- contact census (is the squeeze real?) ---\n");
	{
		const int census = 400;

		Scenario plain;
		plain.squeeze = true;
		plain.reportContacts = true;
		plain.ball.shapeMode = ePlain;
		plain.ball.spikeCount = 0;

		Scenario spiked = plain;
		spiked.ball.shapeMode = eSpiked;
		spiked.ball.spikeCount = 24;
		spiked.ball.massMode = eCollapsedFrame;

		Scenario drop = plain;
		drop.squeeze = false;

		const Scenario* scenarios[3] = { &drop, &plain, &spiked };
		const char* names[3] = { "plain, no squeeze (drop only)", "plain, squeeze", "spiked x24, squeeze" };

		for (int s = 0; s < 3; ++s)
		{
			World w;
			w.Build(*scenarios[s]);
			ContactCensus counts;
			for (int i = 0; i < census; ++i)
			{
				w.Step();
				counts.Observe(w.contacts);
			}
			counts.Report(names[s]);
			w.Destroy();
		}
	}

	// --- Stage E: asymmetric prediction lead, the real engine loop ---------
	// Stage C found nothing because it never predicts past the confirmed tick, which
	// is most of what the engine actually asks PhysX to do. This stage runs the loop
	// as written: one cold confirmed step per frame, then a prediction window that is
	// thrown away, with the two peers leading confirmation by different amounts. Only
	// the confirmed states are compared, and both peers reach every confirmed step
	// from a bitwise identical snapshot, so a divergence here is PhysX carrying state
	// out of the discarded window.
	std::printf("\n--- Stage E: asymmetric prediction lead while two pushers squeeze the ball ---\n");
	{
		Scenario plain;
		plain.squeeze = true;
		plain.ball.shapeMode = ePlain;
		plain.ball.spikeCount = 0;

		// Control: equal lead on both peers. The two run identical work, so they must
		// agree; this proves predicting at all is not the problem and pins any failure
		// below on the lead DIFFERENCE.
		const bool equalLeadDiverged = TwoPeersWithPredictionLead(plain, "plain, equal lead (control)", 120, 200, 4, 4, false);
		Check(!equalLeadDiverged, "equal prediction lead agrees   [plain]");

		// The discriminator: a host that barely predicts against a client six ticks out.
		const bool asymDiverged = TwoPeersWithPredictionLead(plain, "plain, host vs client lead", 120, 200, 0, 6, false);
		Observe(asymDiverged, "asymmetric lead desyncs        [plain]");

		// The same with the window breathing, which is what a live session does.
		const bool jitterDiverged = TwoPeersWithPredictionLead(plain, "plain, jittered leads", 120, 200, 1, 6, true);
		Observe(jitterDiverged, "asymmetric jittered lead desyncs [plain]");

		// And the real ball: the 25-shape compound with the shipping collapsed-frame
		// mass, which is what the demo runs.
		Scenario spiked = plain;
		spiked.ball.shapeMode = eSpiked;
		spiked.ball.spikeCount = 24;
		spiked.ball.massMode = eCollapsedFrame;
		const bool spikedDiverged = TwoPeersWithPredictionLead(spiked, "spiked collapsed, host vs client lead", 120, 200, 0, 6, false);
		Observe(spikedDiverged, "asymmetric lead desyncs        [spiked collapsed]");
	}

	// --- Stage F: a pooled player joins the squeeze -------------------------
	// The demo's fork was confined to the ball and the capsule of the player who had
	// just joined. This stage reproduces that shape: a pool of parked slots, one of
	// which is unparked into the contested island part way through, with the two peers
	// running different prediction leads so they reach the unpark from different points
	// in their own step sequences.
	std::printf("\n--- Stage F: a pooled player joins the contested ball ---\n");
	{
		Scenario pooled;
		pooled.squeeze = true;
		pooled.reportContacts = true;
		pooled.ball.shapeMode = eSpiked;
		pooled.ball.spikeCount = 24;
		pooled.ball.massMode = eCollapsedFrame;
		pooled.poolSlots = 14;      // the inactive soccer player slots
		pooled.spawnAtFrame = 20;

		// Control: equal lead. Both peers do identical work, so the unpark happens at the
		// same point in the same sequence and they must agree.
		const bool equalDiverged = TwoPeersSpawnUnderLead(pooled, "spawn, equal lead (control)", 120, 200, 4, 4, false);
		Check(!equalDiverged, "pooled spawn agrees            [equal lead]");

		// The discriminator: a host that barely predicts against a client six ticks out,
		// both unparking the same slot on the same confirmed frame.
		const bool asymDiverged = TwoPeersSpawnUnderLead(pooled, "spawn, host vs client lead", 120, 200, 0, 6, false);
		Observe(asymDiverged, "pooled spawn desyncs           [asymmetric lead]");

		// And with the window breathing, which is what a live session does.
		const bool jitterDiverged = TwoPeersSpawnUnderLead(pooled, "spawn, jittered leads", 120, 200, 1, 6, true);
		Observe(jitterDiverged, "pooled spawn desyncs           [jittered lead]");

		// Never spawning, as the baseline: the same pooled world left alone must agree,
		// so any failure above belongs to the unpark and not to having a pool at all.
		Scenario noSpawn = pooled;
		noSpawn.spawnAtFrame = -1;
		const bool noSpawnDiverged = TwoPeersSpawnUnderLead(noSpawn, "no spawn, host vs client lead", 120, 200, 0, 6, false);
		Check(!noSpawnDiverged, "pooled world agrees            [no spawn, asymmetric lead]");
	}

	// --- Stage G: is restore-and-step pure? ---------------------------------
	// The general form of every question the stages above ask one case of. If this
	// holds, no difference between two peers' histories can move a confirmed tick, and
	// the desync cannot be PhysX carrying residue across a restore.
	std::printf("\n--- Stage G: is restore-and-step a pure function of the restored state? ---\n");
	{
		Scenario plain;
		plain.squeeze = true;
		plain.ball.shapeMode = ePlain;
		plain.ball.spikeCount = 0;
		const bool plainImpure = RestoreIsPure(plain, "plain, squeezed", 120);
		Observe(plainImpure, "restore is impure              [plain, squeezed]");

		Scenario spiked = plain;
		spiked.ball.shapeMode = eSpiked;
		spiked.ball.spikeCount = 24;
		spiked.ball.massMode = eCollapsedFrame;
		const bool spikedImpure = RestoreIsPure(spiked, "spiked x24 collapsed, squeezed", 120);
		Observe(spikedImpure, "restore is impure              [spiked collapsed, squeezed]");

		Scenario pooled = spiked;
		pooled.poolSlots = 14;
		const bool pooledImpure = RestoreIsPure(pooled, "spiked x24 collapsed, squeezed, pooled", 120);
		Observe(pooledImpure, "restore is impure              [spiked collapsed, pooled]");
	}

	// --- Stage H: construction differences, drop vs squeeze -----------------
	// With residue ruled out by Stage G, what remains is the body's construction: the
	// properties every solve reads and no snapshot writes. This measures how small a
	// construction difference has to be before it stops mattering, and whether the
	// squeeze is what makes it visible.
	std::printf("\n--- Stage H: construction differences the snapshot never carries ---\n");
	{
		Scenario spiked;
		spiked.ball.shapeMode = eSpiked;
		spiked.ball.spikeCount = 24;
		spiked.ball.massMode = eCollapsedFrame;
		ReportPerturbationSensitivity(spiked, 400);
	}

	std::printf("\n%d checks, %d failures\n", gChecks, gFailures);

	StopPhysX();
	return gFailures == 0 ? 0 : 1;
}
