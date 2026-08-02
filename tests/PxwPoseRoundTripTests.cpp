// Isolates the numerical issue behind the "spiked ball" desync.
//
// PhysX stores a rigid body's centre-of-mass frame, not its actor frame:
//
//     Sc::BodyCore::mBody2World    the CoM frame in world space
//     Sc::BodyCore::mBody2Actor    the CoM frame relative to the actor origin
//
// so the actor pose is always a derived quantity:
//
//     NpRigidDynamic::setGlobalPose(P)   body2World = P.getNormalized() * body2Actor
//     NpRigidDynamic::getGlobalPoseFast()   body2World * body2Actor.getInverse()
//
// and Sc::BodyCore::setCMassLocalPose decomposes and reintegrates the world pose:
//
//     oldActor2World = body2World * oldBody2Actor.getInverse()
//     body2World     = oldActor2World * newBody2Actor
//
// even though every caller documents that changing the mass frame must not move the
// actor. Whenever body2Actor is not the identity, none of these round trips return
// what was put in.
//
// A body only gets a non-identity body2Actor if its shapes are off-centre, which is
// exactly what a spiked ball is. This file measures each suspect operation in
// isolation and reports which ones lose precision and which ones accumulate.

#include "PxPhysicsAPI.h"
#include "extensions/PxShapeExt.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using namespace physx;

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

	// Reports a measurement without passing judgement, for behaviour we are
	// characterising rather than asserting.
	void Observe(const char* what, double value, const char* unit)
	{
		std::printf("        %-52s %.6e %s\n", what, value, unit);
	}

	PxDefaultAllocator gAllocator;
	PxDefaultErrorCallback gErrorCallback;

	float PositionError(const PxTransform& a, const PxTransform& b)
	{
		return (a.p - b.p).magnitude();
	}

	// Angle between two orientations. Computed from the relative quaternion rather
	// than acos(dot), because acos loses roughly half its significant digits near
	// dot == 1 and that is the whole range we care about here: a plain
	// 2*acos(dot) has a noise floor around 6e-4 rad in single precision, which is
	// larger than most of the errors being measured.
	double RotationError(const PxTransform& a, const PxTransform& b)
	{
		const PxQuat d = a.q.getConjugate() * b.q;
		const double axis = PxSqrt(double(d.x) * d.x + double(d.y) * d.y + double(d.z) * d.z);
		return 2.0 * std::atan2(axis, std::fabs(double(d.w)));
	}

	// Distance in representable floats, which shows whether a difference is a single
	// rounding step or something structurally larger.
	int UlpDistance(float a, float b)
	{
		if (a == b)
		{
			return 0;
		}
		int ia, ib;
		std::memcpy(&ia, &a, sizeof(int));
		std::memcpy(&ib, &b, sizeof(int));
		if (ia < 0) ia = int(0x80000000u) - ia;
		if (ib < 0) ib = int(0x80000000u) - ib;
		const int d = ia - ib;
		return d < 0 ? -d : d;
	}

	int MaxQuatUlps(const PxTransform& a, const PxTransform& b)
	{
		int worst = UlpDistance(a.q.x, b.q.x);
		worst = PxMax(worst, UlpDistance(a.q.y, b.q.y));
		worst = PxMax(worst, UlpDistance(a.q.z, b.q.z));
		worst = PxMax(worst, UlpDistance(a.q.w, b.q.w));
		return worst;
	}

	int MaxPosUlps(const PxTransform& a, const PxTransform& b)
	{
		int worst = UlpDistance(a.p.x, b.p.x);
		worst = PxMax(worst, UlpDistance(a.p.y, b.p.y));
		worst = PxMax(worst, UlpDistance(a.p.z, b.p.z));
		return worst;
	}

	bool BitwiseEqual(const PxTransform& a, const PxTransform& b)
	{
		return a.p.x == b.p.x && a.p.y == b.p.y && a.p.z == b.p.z &&
			a.q.x == b.q.x && a.q.y == b.q.y && a.q.z == b.q.z && a.q.w == b.q.w;
	}

	const char* ExactTag(const PxTransform& a, const PxTransform& b)
	{
		return BitwiseEqual(a, b) ? "  (bitwise exact)" : "  (DIFFERS)";
	}

	// -----------------------------------------------------------------------------
	// Suspect 1: PxShape::setLocalPose normalises, so it does not store what it is given.

	void TestShapeLocalPoseIsStoredVerbatim(PxPhysics* physics)
	{
		std::printf("TestShapeLocalPoseIsStoredVerbatim\n");

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.0f);
		PxShape* shape = physics->createShape(PxBoxGeometry(0.5f, 0.5f, 0.5f), *material, true);

		// Already unit length as far as the caller is concerned.
		const PxTransform requested(PxVec3(0.75f, 0.25f, -0.4f),
			PxQuat(0.7f, PxVec3(0.0f, 0.0f, 1.0f)).getNormalized());

		shape->setLocalPose(requested);
		const PxTransform readBack = shape->getLocalPose();

		std::printf("        set/get local pose%s\n", ExactTag(requested, readBack));
		Observe("local pose rotation error", RotationError(requested, readBack), "rad");

		// Repeated set(get()) is what a snapshot restore does.
		PxTransform walked = requested;
		bool settled = false;
		for (int i = 0; i < 16 && !settled; ++i)
		{
			shape->setLocalPose(walked);
			const PxTransform next = shape->getLocalPose();
			settled = BitwiseEqual(next, walked);
			walked = next;
		}
		Check(settled, "shape local pose reaches a fixed point under repeated set/get");

		shape->release();
		material->release();
	}

	// -----------------------------------------------------------------------------
	// Suspect 2: setCMassLocalPose promises not to move the actor, but reintegrates.

	struct Spike
	{
		PxTransform pose;
		PxVec3 halfExtents;
	};

	// A ball with spikes around it: nearly spherically symmetric, so its inertia
	// tensor has near-degenerate eigenvalues.
	std::vector<Spike> MakeSpikedBall(int spikeCount, float jitter)
	{
		std::vector<Spike> spikes;
		spikes.push_back(Spike{ PxTransform(PxIdentity), PxVec3(0.5f, 0.5f, 0.5f) });

		for (int i = 0; i < spikeCount; ++i)
		{
			// Fibonacci sphere, so the spikes are near-uniformly distributed.
			const float k = (float(i) + 0.5f) / float(spikeCount);
			const float phi = PxAcos(1.0f - 2.0f * k);
			const float theta = 3.883222f * float(i);
			const PxVec3 dir(PxSin(phi) * PxCos(theta), PxSin(phi) * PxSin(theta), PxCos(phi));

			const PxVec3 offset = dir * (0.75f + jitter * float(i));
			PxQuat rot = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), dir);
			spikes.push_back(Spike{ PxTransform(offset, rot.getNormalized()), PxVec3(0.25f, 0.06f, 0.06f) });
		}
		return spikes;
	}

	PxRigidDynamic* BuildBody(PxPhysics* physics, PxMaterial* material, const std::vector<Spike>& spikes,
		const PxTransform& pose)
	{
		PxRigidDynamic* body = physics->createRigidDynamic(pose);
		for (size_t i = 0; i < spikes.size(); ++i)
		{
			PxShape* shape = physics->createShape(PxBoxGeometry(spikes[i].halfExtents), *material, true);
			shape->setLocalPose(spikes[i].pose);
			body->attachShape(*shape);
			shape->release();
		}
		return body;
	}

	void TestCMassLocalPoseDoesNotMoveActor(PxPhysics* physics, int spikeCount, const char* label)
	{
		std::printf("TestCMassLocalPoseDoesNotMoveActor [%s]\n", label);

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.0f);
		const std::vector<Spike> spikes = MakeSpikedBall(spikeCount, 0.0f);

		const PxTransform placed(PxVec3(3.5f, 12.25f, -7.125f),
			PxQuat(0.3f, PxVec3(0.267261f, 0.534522f, 0.801784f)).getNormalized());

		PxRigidDynamic* body = BuildBody(physics, material, spikes, placed);

		const PxTransform beforeMass = body->getGlobalPose();
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
		const PxTransform afterFirst = body->getGlobalPose();

		const PxTransform com = body->getCMassLocalPose();
		std::printf("        centre of mass: p (%.7g %.7g %.7g) q (%.7g %.7g %.7g %.7g)\n",
			com.p.x, com.p.y, com.p.z, com.q.x, com.q.y, com.q.z, com.q.w);
		std::printf("        first updateMassAndInertia%s\n", ExactTag(beforeMass, afterFirst));
		Observe("actor moved by", PositionError(beforeMass, afterFirst), "m");

		// Recomputing mass properties is idempotent in exact arithmetic: same shapes,
		// same density, same answer. Any movement here is pure round-trip loss.
		// Whether it settles or keeps walking is what decides if it matters.
		PxTransform previous = afterFirst;
		double worstStep = 0.0;
		const int recomputes = 256;
		for (int i = 0; i < recomputes; ++i)
		{
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			const PxTransform now = body->getGlobalPose();
			worstStep = PxMax(worstStep, RotationError(previous, now));
			previous = now;

			if (i == 0 || i == 15 || i == 63 || i == recomputes - 1)
			{
				std::printf("        after %3d recomputes: %.6e rad, %d quat ulps, %d pos ulps\n",
					i + 1, RotationError(afterFirst, now),
					MaxQuatUlps(afterFirst, now), MaxPosUlps(afterFirst, now));
			}
		}
		Observe("worst single-call rotation step", worstStep, "rad");
		Observe("total rotation drift", RotationError(afterFirst, previous), "rad");
		Observe("total position drift", PositionError(afterFirst, previous), "m");

		Check(BitwiseEqual(afterFirst, previous),
			"repeated updateMassAndInertia leaves the actor pose untouched");

		body->release();
		material->release();
	}

	// -----------------------------------------------------------------------------
	// Suspect 3: PxDiagonalize on a near-degenerate inertia tensor.
	//
	// A spiked ball is close to spherically symmetric, so its principal axes are
	// almost arbitrary. If a vanishingly small change to the shape layout swings the
	// resulting orientation wildly, then two peers that build the same body from
	// slightly different float inputs get different mass frames, and every later
	// pose round trip inherits that difference.

	void TestInertiaDiagonalisationConditioning(PxPhysics* physics, int spikeCount, const char* label)
	{
		std::printf("TestInertiaDiagonalisationConditioning [%s]\n", label);

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.0f);
		const PxTransform placed(PxVec3(0.0f, 0.0f, 0.0f), PxQuat(PxIdentity));

		PxRigidDynamic* reference = BuildBody(physics, material, MakeSpikedBall(spikeCount, 0.0f), placed);
		PxRigidBodyExt::updateMassAndInertia(*reference, 10.0f);
		const PxTransform referenceCom = reference->getCMassLocalPose();
		const PxVec3 referenceInertia = reference->getMassSpaceInertiaTensor();

		std::printf("        principal moments: (%.7g %.7g %.7g)\n",
			referenceInertia.x, referenceInertia.y, referenceInertia.z);
		const float spread = (PxMax(referenceInertia.x, PxMax(referenceInertia.y, referenceInertia.z)) -
			PxMin(referenceInertia.x, PxMin(referenceInertia.y, referenceInertia.z)));
		Observe("spread between largest and smallest moment", spread, "kg m^2");
		Observe("relative spread", spread / referenceInertia.magnitude(), "");

		// Perturb the spike layout by an amount far below anything gameplay cares about.
		const float jitter = 1e-6f;
		PxRigidDynamic* perturbed = BuildBody(physics, material, MakeSpikedBall(spikeCount, jitter), placed);
		PxRigidBodyExt::updateMassAndInertia(*perturbed, 10.0f);
		const PxTransform perturbedCom = perturbed->getCMassLocalPose();

		Observe("spike layout perturbation", jitter, "m");
		Observe("resulting centre-of-mass position change", PositionError(referenceCom, perturbedCom), "m");
		Observe("resulting mass-frame rotation change", RotationError(referenceCom, perturbedCom), "rad");

		Check(RotationError(referenceCom, perturbedCom) < 1e-3f,
			"a 1e-6 m layout change moves the principal axes by less than 1e-3 rad");

		perturbed->release();
		reference->release();
		material->release();
	}

	// -----------------------------------------------------------------------------
	// Suspect 4: the capture/restore round trip a rollback performs every tick.

	void TestGlobalPoseRoundTrip(PxPhysics* physics, int spikeCount, const char* label)
	{
		std::printf("TestGlobalPoseRoundTrip [%s]\n", label);

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.0f);
		const PxTransform placed(PxVec3(3.5f, 12.25f, -7.125f),
			PxQuat(0.3f, PxVec3(0.267261f, 0.534522f, 0.801784f)).getNormalized());

		PxRigidDynamic* body = BuildBody(physics, material, MakeSpikedBall(spikeCount, 0.0f), placed);
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

		const PxTransform requested = placed;
		body->setGlobalPose(requested);
		const PxTransform readBack = body->getGlobalPose();
		std::printf("        setGlobalPose then getGlobalPose%s\n", ExactTag(requested, readBack));
		Observe("actor pose round-trip rotation error", RotationError(requested, readBack), "rad");
		Observe("actor pose round-trip position error", PositionError(requested, readBack), "m");

		// Whether the error settles, oscillates or walks decides whether a rollback
		// snapshot can rely on it. A fixed point is fine, a 2-cycle is survivable, an
		// unbounded walk is not.
		PxTransform previous = readBack;
		PxTransform beforePrevious = readBack;
		bool fixedPoint = false;
		bool twoCycle = false;
		const int cycles = 600;
		for (int i = 0; i < cycles; ++i)
		{
			body->setGlobalPose(previous);
			const PxTransform next = body->getGlobalPose();

			if (BitwiseEqual(next, previous))
			{
				fixedPoint = true;
			}
			else if (BitwiseEqual(next, beforePrevious))
			{
				twoCycle = true;
			}

			if (i == 0 || i == 59 || i == cycles - 1)
			{
				std::printf("        after %3d cycles: %.6e rad, %d quat ulps from the start\n",
					i + 1, RotationError(readBack, next), MaxQuatUlps(readBack, next));
			}

			beforePrevious = previous;
			previous = next;
		}
		std::printf("        settles to a %s\n",
			fixedPoint ? "fixed point" : (twoCycle ? "2-cycle" : "NEITHER - it keeps walking"));
		Observe("total rotation drift", RotationError(readBack, previous), "rad");
		Check(fixedPoint || twoCycle, "capture/restore is bounded rather than walking");

		body->release();
		material->release();
	}

	// -----------------------------------------------------------------------------
	// Regression guard for the fix.
	//
	// getGlobalPose now answers from a cache when the core still holds exactly the
	// state the cache was built from. If that check were ever too permissive the body
	// would appear frozen, or would report a pose from before the last step, which is
	// far worse than the few ulps the cache exists to remove. Simulate and confirm the
	// reported pose tracks the body every single step.

	void TestSimulationIsReportedFaithfully(PxPhysics* physics, int spikeCount, const char* label)
	{
		std::printf("TestSimulationIsReportedFaithfully [%s]\n", label);

		PxSceneDesc sceneDesc(physics->getTolerancesScale());
		sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
		PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(0);
		sceneDesc.cpuDispatcher = dispatcher;
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;
		PxScene* scene = physics->createScene(sceneDesc);

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.2f);
		PxRigidStatic* ground = PxCreatePlane(*physics, PxPlane(0.0f, 1.0f, 0.0f, 0.0f), *material);
		scene->addActor(*ground);

		const PxTransform start(PxVec3(0.0f, 8.0f, 0.0f),
			PxQuat(0.3f, PxVec3(0.267261f, 0.534522f, 0.801784f)).getNormalized());
		PxRigidDynamic* body = BuildBody(physics, material, MakeSpikedBall(spikeCount, 0.0f), start);
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
		scene->addActor(*body);

		PxTransform previous = body->getGlobalPose();
		Check(BitwiseEqual(previous, start), "the pose reported before stepping is the pose it was built with");

		int stationaryWhileFalling = 0;
		const int steps = 120;
		for (int i = 0; i < steps; ++i)
		{
			scene->simulate(1.0f / 60.0f);
			scene->fetchResults(true);

			const PxTransform now = body->getGlobalPose();

			// While the ball is still in the air it must move every step. A cache that
			// failed to notice the solver writing body2World would show up right here.
			const bool inAir = now.p.y > 1.5f;
			if (inAir && BitwiseEqual(now, previous))
			{
				++stationaryWhileFalling;
			}
			previous = now;
		}

		std::printf("        final height %.4f m after %d steps\n", previous.p.y, steps);
		Check(stationaryWhileFalling == 0, "the reported pose changes on every step while falling");
		Check(previous.p.y < start.p.y - 1.0f, "the body actually fell");
		Check(previous.p.y > 0.0f, "the body came to rest above the ground plane");

		scene->removeActor(*body);
		body->release();
		ground->release();
		scene->release();
		dispatcher->release();
		material->release();
	}

	// Setting a pose mid-simulation must win over whatever the solver last wrote.
	void TestSetGlobalPoseAfterSimulation(PxPhysics* physics, int spikeCount, const char* label)
	{
		std::printf("TestSetGlobalPoseAfterSimulation [%s]\n", label);

		PxSceneDesc sceneDesc(physics->getTolerancesScale());
		sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
		PxDefaultCpuDispatcher* dispatcher = PxDefaultCpuDispatcherCreate(0);
		sceneDesc.cpuDispatcher = dispatcher;
		sceneDesc.filterShader = PxDefaultSimulationFilterShader;
		PxScene* scene = physics->createScene(sceneDesc);

		PxMaterial* material = physics->createMaterial(0.5f, 0.5f, 0.2f);
		PxRigidDynamic* body = BuildBody(physics, material, MakeSpikedBall(spikeCount, 0.0f),
			PxTransform(PxVec3(0.0f, 8.0f, 0.0f)));
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
		scene->addActor(*body);

		for (int i = 0; i < 10; ++i)
		{
			scene->simulate(1.0f / 60.0f);
			scene->fetchResults(true);
		}

		const PxTransform teleport(PxVec3(-4.25f, 3.75f, 2.5f),
			PxQuat(1.1f, PxVec3(0.0f, 1.0f, 0.0f)).getNormalized());
		body->setGlobalPose(teleport);
		Check(BitwiseEqual(body->getGlobalPose(), teleport),
			"a pose set after simulation reads back exactly");

		scene->simulate(1.0f / 60.0f);
		scene->fetchResults(true);
		Check(!BitwiseEqual(body->getGlobalPose(), teleport),
			"the next step moves the body away from the pose that was set");

		scene->removeActor(*body);
		body->release();
		scene->release();
		dispatcher->release();
		material->release();
	}
}

int main()
{
	PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, NULL);
	PxInitExtensions(*physics, NULL);

	TestShapeLocalPoseIsStoredVerbatim(physics);

	TestCMassLocalPoseDoesNotMoveActor(physics, 0, "single centred box");
	TestCMassLocalPoseDoesNotMoveActor(physics, 1, "one spike");
	TestCMassLocalPoseDoesNotMoveActor(physics, 12, "12 spikes");

	TestInertiaDiagonalisationConditioning(physics, 6, "6 spikes");
	TestInertiaDiagonalisationConditioning(physics, 12, "12 spikes");
	TestInertiaDiagonalisationConditioning(physics, 42, "42 spikes");

	TestGlobalPoseRoundTrip(physics, 0, "single centred box");
	TestGlobalPoseRoundTrip(physics, 12, "12 spikes");

	TestSimulationIsReportedFaithfully(physics, 0, "single centred box");
	TestSimulationIsReportedFaithfully(physics, 12, "12 spikes");
	TestSetGlobalPoseAfterSimulation(physics, 12, "12 spikes");

	std::printf("\n%d checks, %d failures\n", gChecks, gFailures);

	PxCloseExtensions();
	physics->release();
	foundation->release();
	return gFailures == 0 ? 0 : 1;
}
