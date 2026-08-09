#include "ArticulationContacts.h"

namespace pxw
{
	ArticulationContactTracker& GetArticulationContactTracker()
	{
		static ArticulationContactTracker tracker;
		return tracker;
	}

	void ArticulationContactTracker::Clear()
	{
		std::lock_guard<std::mutex> lock(mMutex);
		for (std::unordered_map<PxArticulationReducedCoordinate*, std::vector<PxU8> >::iterator it = mFlags.begin();
			it != mFlags.end(); ++it)
		{
			std::fill(it->second.begin(), it->second.end(), (PxU8)0);
		}
	}

	void ArticulationContactTracker::Forget(PxArticulationReducedCoordinate* articulation)
	{
		std::lock_guard<std::mutex> lock(mMutex);
		mFlags.erase(articulation);
	}

	PxU32 ArticulationContactTracker::Read(PxArticulationReducedCoordinate* articulation, PxU8* dst, PxU32 capacity) const
	{
		if (dst == NULL || capacity == 0)
		{
			return 0;
		}

		std::lock_guard<std::mutex> lock(mMutex);

		memset(dst, 0, capacity);

		std::unordered_map<PxArticulationReducedCoordinate*, std::vector<PxU8> >::const_iterator it = mFlags.find(articulation);
		if (it == mFlags.end())
		{
			return 0;
		}

		const PxU32 count = PxMin(capacity, (PxU32)it->second.size());
		if (count > 0)
		{
			memcpy(dst, &it->second[0], count);
		}
		return count;
	}

	void ArticulationContactTracker::MarkActor(PxActor* actor)
	{
		if (actor == NULL || actor->getType() != PxActorType::eARTICULATION_LINK)
		{
			return;
		}

		PxArticulationLink* link = static_cast<PxArticulationLink*>(actor);
		PxArticulationReducedCoordinate* articulation = &link->getArticulation();
		const PxU32 linkIndex = link->getLinkIndex();

		std::vector<PxU8>& flags = mFlags[articulation];
		if (flags.size() <= linkIndex)
		{
			// Size to the whole articulation rather than just this link so Read reports a
			// consistent link count no matter which link happened to touch first.
			const PxU32 nbLinks = articulation->getNbLinks();
			flags.resize(PxMax(nbLinks, linkIndex + 1), (PxU8)0);
		}
		flags[linkIndex] = 1;
	}

	void ArticulationContactTracker::onContact(const PxContactPairHeader& header, const PxContactPair* pairs, PxU32 count)
	{
		// An actor deleted mid-step leaves a pointer in the header that must not be
		// dereferenced, and its contacts are of no interest to an observer anyway.
		if (header.flags & (PxContactPairHeaderFlag::eREMOVED_ACTOR_0 | PxContactPairHeaderFlag::eREMOVED_ACTOR_1))
		{
			return;
		}

		bool anyTouch = false;
		for (PxU32 i = 0; i < count; ++i)
		{
			const PxContactPair& pair = pairs[i];
			if (!(pair.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS)))
			{
				continue;
			}
			if (pair.flags & (PxContactPairFlag::eREMOVED_SHAPE_0 | PxContactPairFlag::eREMOVED_SHAPE_1))
			{
				continue;
			}
			anyTouch = true;
			break;
		}

		if (!anyTouch)
		{
			return;
		}

		std::lock_guard<std::mutex> lock(mMutex);
		MarkActor(header.actors[0]);
		MarkActor(header.actors[1]);
	}
}
