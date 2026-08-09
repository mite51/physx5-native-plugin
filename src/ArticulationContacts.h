#pragma once

#include "PhysXWrapper.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace pxw
{
	// Per-link contact booleans for articulations.
	//
	// PhysX reports contacts as pair events during fetchResults, but a policy that runs
	// at a lower rate than physics needs to know whether a link touched anything at any
	// point during the whole control step. So this accumulates: onContact only ever sets
	// bits, and the caller decides where the accumulation window starts by calling
	// Clear(). A window that spans several simulate/fetchResults pairs therefore reports
	// the OR over all of its substeps, which is what a decimated control loop wants.
	//
	// Flags are keyed by articulation and indexed by PxArticulationLink::getLinkIndex(),
	// the same low-level index the articulation cache uses, so a caller that already
	// knows its link indices needs no extra mapping.
	//
	// onContact can be invoked from several PhysX worker threads at once, so every entry
	// point is guarded. The lock is uncontended for a single-threaded dispatcher, which
	// is the configuration this plugin uses for reproducibility.
	class ArticulationContactTracker : public PxSimulationEventCallback
	{
	public:
		// Starts a new accumulation window. Existing per-articulation buffers are kept
		// and zeroed rather than freed, so a steady-state loop does not allocate.
		void Clear();

		// Drops an articulation's buffer. Call before releasing the articulation so a
		// later allocation at the same address cannot inherit stale flags.
		void Forget(PxArticulationReducedCoordinate* articulation);

		// Writes one byte per link (0 or 1) into dst, indexed by link index, and zeroes
		// any remaining capacity. Returns the number of links written. Returns 0 when the
		// articulation has never reported a contact, leaving dst zeroed, so a caller
		// cannot tell "no contacts yet" from "no contacts this window" -- both mean the
		// same thing to an observer.
		PxU32 Read(PxArticulationReducedCoordinate* articulation, PxU8* dst, PxU32 capacity) const;

		void onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 count) override;

		void onTrigger(PxTriggerPair*, PxU32) override {}
		void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
		void onWake(PxActor**, PxU32) override {}
		void onSleep(PxActor**, PxU32) override {}
		void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}

	private:
		// Caller must hold mMutex.
		void MarkActor(PxActor* actor);

		mutable std::mutex mMutex;
		std::unordered_map<PxArticulationReducedCoordinate*, std::vector<PxU8> > mFlags;
	};

	// The process-wide tracker. Scenes created with PxwSceneFlag::eENABLE_CONTACT_EVENTS
	// and no callback of their own get this one installed.
	ArticulationContactTracker& GetArticulationContactTracker();
}
