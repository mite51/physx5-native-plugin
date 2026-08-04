// UNDPWR deterministic simulation support layer. See include/PxwUndpwr.h for the
// rationale behind the stable-ID registry and the contact cache reset.

#include "PxwUndpwr.h"
#include "VehicleHelper.h"
#include "VehicleModule.h"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace physx;

namespace pxw
{
	PxwLogSink gPxwLogSink = NULL;

	namespace
	{
		const PxU32 kStateMagic = 0x57444E55u;   // 'UNDW'
		const PxU32 kStateVersion = 2u;

		const PxU64 kFnvOffsetBasis = 0xcbf29ce484222325ULL;
		const PxU64 kFnvPrime = 0x100000001b3ULL;

		// The wake counter value that means "this body does not sleep".
		//
		// PhysX decays the counter by dt every step and only runs its sleep
		// bookkeeping once the counter has fallen below half the reset time. At this
		// magnitude the decay is far below one ulp, so subtracting dt returns the same
		// float: the counter is a fixed point rather than merely a large number, and a
		// body pinned here never reaches the sleep branch at all, for any timestep.
		const PxReal kNeverSleepWakeCounter = PX_MAX_F32;

		// Articulation cache subset that is both readable and writable. Link
		// velocities and accelerations are outputs and cannot be applied, so they are
		// deliberately excluded.
		const PxArticulationCacheFlags kArticulationCacheFlags =
			PxArticulationCacheFlag::ePOSITION |
			PxArticulationCacheFlag::eVELOCITY |
			PxArticulationCacheFlag::eFORCE;

		struct StateFlag
		{
			enum Enum : PxU32
			{
				eSLEEPING = 1u << 0,
				eDISABLED = 1u << 1
			};
		};

		struct StateHeader
		{
			PxU32 magic;
			PxU32 version;
			PxU32 entryCount;
			PxU32 totalBytes;
		};

		struct EntryHeader
		{
			PxU32 stableId;
			PxU32 kind;
			PxU32 payloadBytes;
			PxU32 reserved;
		};

		struct RigidPayload
		{
			PxwTransformData pose;
			PxVec3 linearVelocity;
			PxVec3 angularVelocity;
			PxReal wakeCounter;
			PxU32  flags;
			PxU32  restTicks;
		};

		struct ArticulationPayload
		{
			PxwTransformData rootPose;
			PxVec3 rootLinearVelocity;
			PxVec3 rootAngularVelocity;
			PxReal wakeCounter;
			PxU32  flags;
			PxU32  dofCount;
			PxU32  linkCount;
			PxU32  restTicks;
			// Followed by 3 * dofCount floats: joint positions, velocities, forces.
		};

		// Canonical form of a pose for snapshot purposes.
		//
		// PxRigidActor::setGlobalPose stores pose.getNormalized(), and normalisation is
		// not idempotent: for a quaternion whose magnitude is a hair below one it
		// scales up, and for the result, which is now a hair above one, it scales back
		// down. Capturing the raw pose therefore produces a two-cycle in which
		// capture/restore never settles, and any rollback replay diverges immediately.
		//
		// Normalising here breaks the cycle. The stored value becomes the fixed point
		// of capture, and the live state becomes the fixed point of restore, so a
		// resimulation starts from exactly the state the original simulation did.
		inline PxTransform CanonicalPose(const PxTransform& pose)
		{
			return pose.getNormalized();
		}

		// Reports a diagnostic through the installed sink, if any. Silent otherwise,
		// so a caller that has not opted in pays nothing.
		inline void PxwLog(PxI32 severity, const char* message)
		{
			const PxwLogSink sink = gPxwLogSink;
			if (sink != NULL)
			{
				sink(static_cast<int>(severity), message);
			}
		}

		inline PxU64 FnvAccumulate(PxU64 hash, const void* data, size_t byteCount)
		{
			const PxU8* bytes = static_cast<const PxU8*>(data);
			for (size_t i = 0; i < byteCount; ++i)
			{
				hash ^= static_cast<PxU64>(bytes[i]);
				hash *= kFnvPrime;
			}
			return hash;
		}

		void LogMessage(PxErrorCode::Enum code, const char* message)
		{
			PxGetFoundation().error(code, __FILE__, __LINE__, "%s", message);
		}
	}

	// A registry entry. Entries are kept sorted by stableId at all times so that
	// iteration order, scene insertion order and snapshot layout are all identical on
	// every peer.
	struct PxwWorldEntry
	{
		PxU32 stableId;
		PxU32 kind;
		void* handle;
		bool  inScene;
		bool  pendingAdd;
		bool  pendingRemove;
		bool  enabled;

		// Consecutive steps this body's speed has stayed below the world's sleep
		// thresholds. Drives framework sleeping, and is carried in the snapshot so the
		// decision replays. Meaningless for statics and kinematics.
		PxU32 restTicks;

		// Articulation bookkeeping, unused for other kinds.
		PxArticulationCache* cache;
		PxU32 dofCount;
		PxU32 linkCount;

		PxwWorldEntry()
			: stableId(0), kind(0), handle(NULL), inScene(false), pendingAdd(false),
			  pendingRemove(false), enabled(true), restTicks(0), cache(NULL), dofCount(0), linkCount(0)
		{
		}
	};

	class PxwWorld
	{
	public:
		PxScene* scene;
		std::vector<PxwWorldEntry> entries;
		bool simulating;
		std::vector<PxU8> scratch;

		// Framework sleep parameters. Thresholds are stored squared, to compare
		// against magnitudeSquared() without a root. sleepTicks of 0 disables
		// framework sleeping, so bodies stay awake and pinned; this is the default.
		PxReal sleepLinThresholdSq;
		PxReal sleepAngThresholdSq;
		PxU32 sleepTicks;

		// Reverse index from a scene actor pointer back to the identity every peer knows
		// it by. PhysX reports a scene-query hit as a PxRigidActor*, but the registry is
		// keyed by stable ID, so a query needs this to turn a hit into a stable ID and
		// drop hits on actors it does not own. Rebuilt from the committed entries on every
		// PxwWorldCommitPending; enabling or disabling an entry keeps the same actor
		// pointer, so it does not need rebuilding there. Kept sorted by pointer so a hit
		// resolves with a binary search rather than a scan of every registered body.
		struct ActorLookup
		{
			PxRigidActor* actor;
			PxU32 stableId;
			PxU32 kind;
		};
		std::vector<ActorLookup> actorLookup;

		PxwWorld()
			: scene(NULL), simulating(false),
			  sleepLinThresholdSq(0.0f), sleepAngThresholdSq(0.0f), sleepTicks(0) {}

		// Resolves a scene actor to its stable ID and kind, or returns false when the
		// actor is not one this world registered.
		bool ResolveActor(PxRigidActor* actor, PxU32& stableId, PxU32& kind) const
		{
			size_t lo = 0;
			size_t hi = actorLookup.size();
			while (lo < hi)
			{
				size_t mid = lo + ((hi - lo) >> 1);
				if (actorLookup[mid].actor < actor)
				{
					lo = mid + 1;
				}
				else
				{
					hi = mid;
				}
			}
			if (lo < actorLookup.size() && actorLookup[lo].actor == actor)
			{
				stableId = actorLookup[lo].stableId;
				kind = actorLookup[lo].kind;
				return true;
			}
			return false;
		}

		// Index of stableId, or -1. Entries are sorted so this is a binary search.
		PxI32 Find(PxU32 stableId) const
		{
			size_t lo = 0;
			size_t hi = entries.size();
			while (lo < hi)
			{
				size_t mid = lo + ((hi - lo) >> 1);
				if (entries[mid].stableId < stableId)
				{
					lo = mid + 1;
				}
				else
				{
					hi = mid;
				}
			}
			if (lo < entries.size() && entries[lo].stableId == stableId)
			{
				return static_cast<PxI32>(lo);
			}
			return -1;
		}

		size_t LowerBound(PxU32 stableId) const
		{
			size_t lo = 0;
			size_t hi = entries.size();
			while (lo < hi)
			{
				size_t mid = lo + ((hi - lo) >> 1);
				if (entries[mid].stableId < stableId)
				{
					lo = mid + 1;
				}
				else
				{
					hi = mid;
				}
			}
			return lo;
		}
	};

	namespace
	{
		inline PxRigidDynamic* AsDynamic(const PxwWorldEntry& e)
		{
			if (e.kind == PxwHandleKind::eRIGID_DYNAMIC || e.kind == PxwHandleKind::eRIGID_KINEMATIC)
			{
				return static_cast<PxRigidDynamic*>(e.handle);
			}
			return NULL;
		}

		inline PxArticulationReducedCoordinate* AsArticulation(const PxwWorldEntry& e)
		{
			if (e.kind == PxwHandleKind::eARTICULATION)
			{
				return static_cast<PxArticulationReducedCoordinate*>(e.handle);
			}
			return NULL;
		}

		inline PxwVehicle* AsVehicle(const PxwWorldEntry& e)
		{
			if (e.kind == PxwHandleKind::eVEHICLE)
			{
				return static_cast<PxwVehicle*>(e.handle);
			}
			return NULL;
		}

		// Rigid body backing an entry, whatever kind it is. Vehicles resolve to their
		// chassis actor so their pose participates in snapshots and hashes.
		PxRigidActor* AsRigidActor(const PxwWorldEntry& e)
		{
			switch (e.kind)
			{
			case PxwHandleKind::eRIGID_DYNAMIC:
			case PxwHandleKind::eRIGID_KINEMATIC:
			case PxwHandleKind::eRIGID_STATIC:
				return static_cast<PxRigidActor*>(e.handle);
			case PxwHandleKind::eVEHICLE:
			{
				PxwVehicle* vehicle = static_cast<PxwVehicle*>(e.handle);
				return vehicle != NULL ? vehicle->GetActor() : NULL;
			}
			default:
				return NULL;
			}
		}

		// Rebuilds the actor-to-stable-ID reverse index from the entries currently in the
		// scene. Called after every commit, where the set of scene actors changes.
		void RebuildActorLookup(PxwWorld& world)
		{
			world.actorLookup.clear();
			for (size_t i = 0; i < world.entries.size(); ++i)
			{
				const PxwWorldEntry& e = world.entries[i];
				if (!e.inScene)
				{
					continue;
				}
				PxRigidActor* actor = AsRigidActor(e);
				if (actor == NULL)
				{
					continue;
				}
				PxwWorld::ActorLookup record;
				record.actor = actor;
				record.stableId = e.stableId;
				record.kind = e.kind;
				world.actorLookup.push_back(record);
			}

			std::sort(world.actorLookup.begin(), world.actorLookup.end(),
				[](const PxwWorld::ActorLookup& a, const PxwWorld::ActorLookup& b)
				{
					return a.actor < b.actor;
				});
		}

		// Largest number of touching hits a single query gathers before sorting and
		// truncating to the caller's capacity. Hits beyond this are dropped, which is a
		// deterministic outcome as long as every peer uses the same limit; it is set well
		// above anything a normal scene produces from one query.
		const PxU32 kMaxQueryTouches = 256u;

		// Maps a managed SimForceMode onto PxForceMode. The enums share values by design,
		// so this only clamps a malformed value onto eFORCE rather than translating.
		inline PxForceMode::Enum ToForceMode(PxU32 mode)
		{
			switch (mode)
			{
			case PxForceMode::eIMPULSE:         return PxForceMode::eIMPULSE;
			case PxForceMode::eVELOCITY_CHANGE: return PxForceMode::eVELOCITY_CHANGE;
			case PxForceMode::eACCELERATION:    return PxForceMode::eACCELERATION;
			case PxForceMode::eFORCE:
			default:                            return PxForceMode::eFORCE;
			}
		}

		// Builds the query geometry for an overlap or sweep from the managed shape enum.
		// Returns false for an unknown shape. PxwQueryShape mirrors SimQueryShape: 0 sphere,
		// 1 box, 2 capsule. A capsule's axis is PhysX's local X, as the SDK defines it.
		inline bool BuildQueryGeometry(PxU32 shape, const PxVec3& halfExtents, PxReal radius,
			PxSphereGeometry& sphere, PxBoxGeometry& box, PxCapsuleGeometry& capsule,
			PxGeometry** out)
		{
			switch (shape)
			{
			case 0u: // sphere
				sphere = PxSphereGeometry(radius);
				*out = &sphere;
				return true;
			case 1u: // box
				box = PxBoxGeometry(halfExtents.x, halfExtents.y, halfExtents.z);
				*out = &box;
				return true;
			case 2u: // capsule
				capsule = PxCapsuleGeometry(radius, halfExtents.y);
				*out = &capsule;
				return true;
			default:
				return false;
			}
		}

		// Static actors never move, so they are registered for identity resolution but
		// contribute nothing to snapshots. Excluding them keeps the rollback ring
		// buffer proportional to the number of moving bodies rather than level size.
		inline bool IsCaptured(const PxwWorldEntry& e)
		{
			return e.kind != PxwHandleKind::eRIGID_STATIC && e.handle != NULL;
		}

		PxU32 PayloadSize(const PxwWorldEntry& e)
		{
			switch (e.kind)
			{
			case PxwHandleKind::eRIGID_DYNAMIC:
			case PxwHandleKind::eRIGID_KINEMATIC:
			case PxwHandleKind::eVEHICLE:
				return static_cast<PxU32>(sizeof(RigidPayload));
			case PxwHandleKind::eARTICULATION:
				return static_cast<PxU32>(sizeof(ArticulationPayload) + sizeof(PxReal) * 3 * e.dofCount);
			default:
				return 0;
			}
		}

		void EnsureArticulationCache(PxwWorldEntry& entry)
		{
			PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
			if (articulation == NULL || entry.cache != NULL)
			{
				return;
			}
			entry.cache = articulation->createCache();
			entry.dofCount = articulation->getDofs();
			entry.linkCount = articulation->getNbLinks();
			if (entry.cache == NULL)
			{
				LogMessage(PxErrorCode::eINTERNAL_ERROR, "UNDPWR: failed to create an articulation cache; that articulation cannot be rolled back.\n");
			}
		}

		void ReleaseArticulationCache(PxwWorldEntry& entry)
		{
			if (entry.cache != NULL)
			{
				entry.cache->release();
				entry.cache = NULL;
			}
		}

		// Pins the wake counter of an awake body high, so PhysX's own sleep
		// bookkeeping -- the part that does not survive rollback -- never runs for it.
		// The framework decides sleeping instead, in UpdateSleepForWorld.
		//
		// A no-op for kinematics and statics, which have no wake counter that matters,
		// for parked entries, where PhysX rejects the call, and deliberately for a
		// sleeping body: pinning one would wake it, and whether a body sleeps is the
		// framework's decision to make, not a side effect of pinning.
		//
		// Has to be reapplied after anything that can lower the counter: on scene
		// entry, on unparking, on every restore of an awake body, and after PhysX
		// auto-wakes a sleeper. Setting it once is not enough.
		void PinWakeCounter(PxwWorldEntry& entry)
		{
			if (!entry.enabled)
			{
				return;
			}

			PxRigidActor* actor = AsRigidActor(entry);
			if (actor != NULL)
			{
				PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
				if (dynamic != NULL && !(dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) &&
					dynamic->getScene() != NULL && !dynamic->isSleeping())
				{
					dynamic->setWakeCounter(kNeverSleepWakeCounter);
				}
				return;
			}

			PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
			if (articulation != NULL && articulation->getScene() != NULL && !articulation->isSleeping())
			{
				articulation->setWakeCounter(kNeverSleepWakeCounter);
			}
		}

		// Whether a body's linear and angular speeds are both below the world's sleep
		// thresholds. A pure function of restored velocity, so it replays.
		bool BelowSleepThresholds(const PxVec3& linear, const PxVec3& angular, const PxwWorld& world)
		{
			return linear.magnitudeSquared() <= world.sleepLinThresholdSq
				&& angular.magnitudeSquared() <= world.sleepAngThresholdSq;
		}

		// Advances framework sleeping by one step. Called after fetchResults, so the
		// decision it makes is part of the state the next capture records.
		//
		// While a body is awake its wake counter is pinned, so PhysX never sleeps it
		// on its own. Once its speed has stayed below the thresholds for sleepTicks
		// steps the framework sleeps it. When PhysX later auto-wakes a sleeper on
		// contact, the rest counter is cleared and the pin restored. Every input --
		// velocity and the rest counter -- is snapshotted, so the decision replays.
		//
		// The one part that is PhysX's and not snapshotted is the tick at which a
		// sleeper is auto-woken by a new contact. Whether that replays is the property
		// TestFrameworkSleepReplays exists to hold.
		void UpdateSleepForWorld(PxwWorld& world)
		{
			if (world.sleepTicks == 0)
			{
				return;   // framework sleeping disabled; bodies stay awake and pinned
			}

			for (size_t i = 0; i < world.entries.size(); ++i)
			{
				PxwWorldEntry& entry = world.entries[i];
				if (!entry.inScene || !entry.enabled)
				{
					continue;
				}

				PxRigidActor* actor = AsRigidActor(entry);
				if (actor != NULL)
				{
					PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
					if (dynamic == NULL || (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
					{
						continue;
					}
					if (dynamic->isSleeping())
					{
						continue;   // asleep: nothing to do until PhysX wakes it
					}
					if (entry.restTicks >= world.sleepTicks)
					{
						// Was asleep, and PhysX auto-woke it; the counter is stale.
						entry.restTicks = 0;
						PinWakeCounter(entry);
					}
					else if (BelowSleepThresholds(dynamic->getLinearVelocity(), dynamic->getAngularVelocity(), world))
					{
						if (++entry.restTicks >= world.sleepTicks)
						{
							dynamic->putToSleep();
						}
					}
					else
					{
						entry.restTicks = 0;
					}
					continue;
				}

				PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
				if (articulation != NULL)
				{
					if (articulation->isSleeping())
					{
						continue;
					}
					if (entry.restTicks >= world.sleepTicks)
					{
						entry.restTicks = 0;
						PinWakeCounter(entry);
					}
					else if (BelowSleepThresholds(articulation->getRootLinearVelocity(),
						articulation->getRootAngularVelocity(), world))
					{
						if (++entry.restTicks >= world.sleepTicks)
						{
							articulation->putToSleep();
						}
					}
					else
					{
						entry.restTicks = 0;
					}
				}
			}
		}

		void AddEntryToScene(PxScene* scene, PxwWorldEntry& entry)
		{
			switch (entry.kind)
			{
			case PxwHandleKind::eRIGID_DYNAMIC:
			case PxwHandleKind::eRIGID_KINEMATIC:
			case PxwHandleKind::eRIGID_STATIC:
				scene->addActor(*static_cast<PxActor*>(entry.handle));
				break;
			case PxwHandleKind::eARTICULATION:
				scene->addArticulation(*static_cast<PxArticulationReducedCoordinate*>(entry.handle));
				EnsureArticulationCache(entry);
				break;
			case PxwHandleKind::eVEHICLE:
			{
				PxwVehicle* vehicle = AsVehicle(entry);
				if (vehicle != NULL)
				{
					vehicle->AddToScene();
				}
				break;
			}
			default:
				LogMessage(PxErrorCode::eINVALID_PARAMETER, "UNDPWR: cannot add an entry with an unknown handle kind.\n");
				return;
			}
			entry.inScene = true;
		}

		void RemoveEntryFromScene(PxScene* scene, PxwWorldEntry& entry)
		{
			if (!entry.inScene)
			{
				return;
			}
			switch (entry.kind)
			{
			case PxwHandleKind::eRIGID_DYNAMIC:
			case PxwHandleKind::eRIGID_KINEMATIC:
			case PxwHandleKind::eRIGID_STATIC:
				scene->removeActor(*static_cast<PxActor*>(entry.handle));
				break;
			case PxwHandleKind::eARTICULATION:
				ReleaseArticulationCache(entry);
				scene->removeArticulation(*static_cast<PxArticulationReducedCoordinate*>(entry.handle));
				break;
			case PxwHandleKind::eVEHICLE:
			{
				PxwVehicle* vehicle = AsVehicle(entry);
				if (vehicle != NULL)
				{
					vehicle->RemoveFromScene();
				}
				break;
			}
			default:
				break;
			}
			entry.inScene = false;
		}

		void CaptureRigid(const PxwWorldEntry& entry, RigidPayload& out)
		{
			std::memset(&out, 0, sizeof(out));

			PxRigidActor* actor = AsRigidActor(entry);
			if (actor == NULL)
			{
				return;
			}

			out.pose = PxwTransformData(CanonicalPose(actor->getGlobalPose()));
			out.restTicks = entry.restTicks;

			PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
			if (dynamic != NULL)
			{
				const bool kinematic = (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC);
				if (kinematic)
				{
					out.linearVelocity = dynamic->getLinearVelocity();
					out.angularVelocity = dynamic->getAngularVelocity();
				}
				else
				{
					out.wakeCounter = dynamic->getWakeCounter();
					if (dynamic->getScene() != NULL && dynamic->isSleeping())
					{
						// A sleeping body is at rest by definition, and the restore
						// path sleeps it, which zeroes its velocity. Capturing whatever
						// residual velocity PhysX still reports for a body kept in an
						// active island would make capture and restore disagree, so it
						// is left at zero to match.
						out.flags |= StateFlag::eSLEEPING;
					}
					else
					{
						out.linearVelocity = dynamic->getLinearVelocity();
						out.angularVelocity = dynamic->getAngularVelocity();
					}
				}
			}

			if (!entry.enabled)
			{
				out.flags |= StateFlag::eDISABLED;
			}
		}

		void RestoreRigid(PxwWorldEntry& entry, const RigidPayload& in)
		{
			PxRigidActor* actor = AsRigidActor(entry);
			if (actor == NULL)
			{
				return;
			}

			actor->setGlobalPose(in.pose.ToPxTransform(), false);
			entry.restTicks = in.restTicks;

			PxRigidDynamic* dynamic = actor->is<PxRigidDynamic>();
			if (dynamic == NULL)
			{
				return;
			}

			const bool kinematic = (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC);
			if (kinematic)
			{
				return;
			}

			// autowake is false throughout: waking a body that was asleep in the
			// snapshot would silently change the simulation being restored.
			dynamic->setLinearVelocity(in.linearVelocity, false);
			dynamic->setAngularVelocity(in.angularVelocity, false);
			dynamic->clearForce(PxForceMode::eFORCE);
			dynamic->clearForce(PxForceMode::eIMPULSE);
			dynamic->clearTorque(PxForceMode::eFORCE);
			dynamic->clearTorque(PxForceMode::eIMPULSE);

			if (dynamic->getScene() == NULL)
			{
				return;
			}

			// The sleeping flag is snapshotted, so it is restored directly. An awake
			// body has its wake counter re-pinned rather than restored: the pin is a
			// constant of the world, not a value the snapshot has to carry, and it is
			// what keeps PhysX's own sleep bookkeeping from running.
			if (in.flags & StateFlag::eSLEEPING)
			{
				if (!dynamic->isSleeping())
				{
					dynamic->putToSleep();
				}
			}
			else
			{
				if (dynamic->isSleeping())
				{
					dynamic->wakeUp();
				}
				dynamic->setWakeCounter(kNeverSleepWakeCounter);
			}
		}

		void CaptureArticulation(const PxwWorldEntry& entry, ArticulationPayload& header, PxReal* joints)
		{
			std::memset(&header, 0, sizeof(header));

			PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
			if (articulation == NULL)
			{
				return;
			}

			header.rootPose = PxwTransformData(CanonicalPose(articulation->getRootGlobalPose()));
			header.rootLinearVelocity = articulation->getRootLinearVelocity();
			header.rootAngularVelocity = articulation->getRootAngularVelocity();
			header.wakeCounter = articulation->getWakeCounter();
			header.dofCount = entry.dofCount;
			header.linkCount = entry.linkCount;
			header.restTicks = entry.restTicks;

			if (articulation->getScene() != NULL && articulation->isSleeping())
			{
				header.flags |= StateFlag::eSLEEPING;
			}
			if (!entry.enabled)
			{
				header.flags |= StateFlag::eDISABLED;
			}

			if (entry.cache == NULL || entry.dofCount == 0)
			{
				return;
			}

			articulation->copyInternalStateToCache(*entry.cache, kArticulationCacheFlags);

			const size_t dofBytes = sizeof(PxReal) * entry.dofCount;
			std::memcpy(joints, entry.cache->jointPosition, dofBytes);
			std::memcpy(joints + entry.dofCount, entry.cache->jointVelocity, dofBytes);
			std::memcpy(joints + entry.dofCount * 2, entry.cache->jointForce, dofBytes);
		}

		void RestoreArticulation(PxwWorldEntry& entry, const ArticulationPayload& header, const PxReal* joints)
		{
			PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
			if (articulation == NULL)
			{
				return;
			}

			entry.restTicks = header.restTicks;

			if (header.dofCount != entry.dofCount)
			{
				LogMessage(PxErrorCode::eINVALID_PARAMETER,
					"UNDPWR: articulation DOF count changed between capture and restore; skipping joint state.\n");
			}
			else if (entry.cache != NULL && entry.dofCount > 0)
			{
				const size_t dofBytes = sizeof(PxReal) * entry.dofCount;
				std::memcpy(entry.cache->jointPosition, joints, dofBytes);
				std::memcpy(entry.cache->jointVelocity, joints + entry.dofCount, dofBytes);
				std::memcpy(entry.cache->jointForce, joints + entry.dofCount * 2, dofBytes);
				articulation->applyCache(*entry.cache, kArticulationCacheFlags, false);
			}

			articulation->setRootGlobalPose(header.rootPose.ToPxTransform(), false);
			articulation->setRootLinearVelocity(header.rootLinearVelocity, false);
			articulation->setRootAngularVelocity(header.rootAngularVelocity, false);

			if (articulation->getScene() == NULL)
			{
				return;
			}

			// An articulation's links carry the same per-body sleep bookkeeping as a
			// free rigid body, so the same reasoning applies: the sleeping flag is
			// restored, and an awake articulation has its wake counter re-pinned.
			if (header.flags & StateFlag::eSLEEPING)
			{
				if (!articulation->isSleeping())
				{
					articulation->putToSleep();
				}
			}
			else
			{
				if (articulation->isSleeping())
				{
					articulation->wakeUp();
				}
				articulation->setWakeCounter(kNeverSleepWakeCounter);
			}
		}

		void ApplyEnabled(PxwWorldEntry& entry, bool enabled)
		{
			const bool wasEnabled = entry.enabled;
			entry.enabled = enabled;

			switch (entry.kind)
			{
			case PxwHandleKind::eRIGID_DYNAMIC:
			case PxwHandleKind::eRIGID_KINEMATIC:
			case PxwHandleKind::eRIGID_STATIC:
			{
				PxActor* actor = static_cast<PxActor*>(entry.handle);
				if (actor != NULL)
				{
					actor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, !enabled);
				}
				break;
			}
			case PxwHandleKind::eARTICULATION:
			{
				// PhysX has no per-articulation simulation switch, so a parked
				// articulation is put to sleep instead. Callers are expected to move it
				// out of play as well.
				PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
				if (articulation != NULL && articulation->getScene() != NULL)
				{
					if (!enabled)
					{
						articulation->putToSleep();
					}
					else
					{
						articulation->wakeUp();
					}
				}
				break;
			}
			default:
				break;
			}

			// Parking clears the wake counter, and PhysX refuses to set it while an
			// actor is parked, so the pin has to be reapplied on the way back in. The
			// rest counter is left as it was: it only advances in UpdateSleepForWorld
			// and is otherwise carried in the snapshot, so a restore's value must not
			// be overwritten here.
			if (enabled && !wasEnabled)
			{
				PinWakeCounter(entry);
			}
		}
	}
}

using namespace pxw;

// ------------------------------------------------------------------ logging ----

void PxwSetLogCallback(PxwLogCallbackFn callback)
{
	gPxwLogSink = reinterpret_cast<PxwLogSink>(callback);
}

// -------------------------------------------------------------------- scene ----

PxScene* PxwCreateSceneEx(const PxwSceneDesc* desc)
{
	if (desc == NULL)
	{
		return NULL;
	}
	return GetGlobalPhysXWrapper().CreateSceneEx(*desc);
}

void PxwSceneSimulate(PxScene* scene, PxReal dt)
{
	if (scene == NULL)
	{
		return;
	}
	VehicleStepScene(scene, dt);
	scene->simulate(dt);
}

void PxwSceneFetchResults(PxScene* scene)
{
	if (scene == NULL)
	{
		return;
	}
	scene->fetchResults(true);
	scene->fetchResultsParticleSystem();
}

void PxwSceneStep(PxScene* scene, PxReal dt)
{
	PxwSceneSimulate(scene, dt);
	PxwSceneFetchResults(scene);
}

void PxwSceneResetContactState(PxScene* scene)
{
	if (scene == NULL)
	{
		return;
	}

	// PxScene::flushSimulation is only memory housekeeping; it does not drop the
	// persistent contact manifolds. resetFiltering is the documented way to make
	// PhysX discard existing contact pairs and regenerate them from scratch, which is
	// what a resimulated tick needs in order to see the same contacts the original
	// tick saw.
	const PxU32 actorCount = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if (actorCount == 0)
	{
		return;
	}

	std::vector<PxActor*> actors(actorCount);
	scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, actors.data(), actorCount, 0);

	for (PxU32 i = 0; i < actorCount; ++i)
	{
		scene->resetFiltering(*actors[i]);
	}
}

// ------------------------------------------------------- rigid body extras ----

void PxwSetRigidDynamicSolverIterations(PxRigidDynamic* actor, PxU32 positionIters, PxU32 velocityIters)
{
	if (actor != NULL)
	{
		actor->setSolverIterationCounts(positionIters, velocityIters);
	}
}

void PxwSetRigidDynamicSleepThreshold(PxRigidDynamic* actor, PxReal threshold)
{
	if (actor != NULL)
	{
		actor->setSleepThreshold(threshold);
	}
}

PxReal PxwGetRigidDynamicSleepThreshold(PxRigidDynamic* actor)
{
	return actor != NULL ? actor->getSleepThreshold() : 0.0f;
}

void PxwSetRigidDynamicWakeCounter(PxRigidDynamic* actor, PxReal wakeCounter)
{
	if (actor != NULL && actor->getScene() != NULL)
	{
		actor->setWakeCounter(wakeCounter);
	}
}

PxReal PxwGetRigidDynamicWakeCounter(PxRigidDynamic* actor)
{
	return actor != NULL ? actor->getWakeCounter() : 0.0f;
}

bool PxwIsRigidDynamicSleeping(PxRigidDynamic* actor)
{
	return actor != NULL && actor->getScene() != NULL && actor->isSleeping();
}

void PxwSetRigidBodyMassSpaceInertiaTensor(PxRigidBody* actor, const PxVec3* inertia)
{
	if (actor != NULL && inertia != NULL)
	{
		actor->setMassSpaceInertiaTensor(*inertia);
	}
}

void PxwGetRigidBodyMassSpaceInertiaTensor(PxRigidBody* actor, PxVec3* outInertia)
{
	if (outInertia == NULL)
	{
		return;
	}
	*outInertia = actor != NULL ? actor->getMassSpaceInertiaTensor() : PxVec3(0.0f);
}

void PxwSetRigidBodyCMassLocalPose(PxRigidBody* actor, const PxwTransformData* pose)
{
	if (actor != NULL && pose != NULL)
	{
		actor->setCMassLocalPose(pose->ToPxTransform());
	}
}

void PxwSetActorSimulationEnabled(PxActor* actor, bool enabled)
{
	if (actor != NULL)
	{
		actor->setActorFlag(PxActorFlag::eDISABLE_SIMULATION, !enabled);
	}
}

void PxwApplyDeterministicRigidDefaults(PxRigidDynamic* actor, PxU32 positionIters, PxU32 velocityIters)
{
	if (actor == NULL)
	{
		return;
	}
	actor->setSolverIterationCounts(positionIters, velocityIters);

	// Speculative CCD changes contact generation based on velocity history, which
	// makes a restored state behave differently from the state it was captured from.
	actor->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
	actor->setMaxDepenetrationVelocity(3.0f);
}

// -------------------------------------------------------- body gameplay I/O ----
//
// The reads and writes gameplay applies to a single body inside a step handler. They
// take a PxActor* directly, which is the handle the registry stored, and forward to
// the matching PxRigidBody / PxRigidActor call. See Documentation/NativeGameplayApi.md
// in the managed package for the contract. A force applied here is accumulated by
// PhysX and cleared on the next step, so a replayed step that applies the same force to
// the same restored state reproduces the original; nothing else is needed to make them
// deterministic, because the framework fixes the order the step handlers run in.

void PxwBodyAddForce(PxActor* actor, const PxVec3* force, PxU32 mode)
{
	if (actor == NULL || force == NULL)
	{
		return;
	}
	PxRigidBody* body = actor->is<PxRigidBody>();
	if (body == NULL || body->getScene() == NULL)
	{
		return;
	}
	if (body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		return;
	}
	body->addForce(*force, ToForceMode(mode));
}

void PxwBodyAddTorque(PxActor* actor, const PxVec3* torque, PxU32 mode)
{
	if (actor == NULL || torque == NULL)
	{
		return;
	}
	PxRigidBody* body = actor->is<PxRigidBody>();
	if (body == NULL || body->getScene() == NULL)
	{
		return;
	}
	if (body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		return;
	}
	body->addTorque(*torque, ToForceMode(mode));
}

void PxwBodyGetPose(PxActor* actor, PxwPose* outPose)
{
	if (outPose == NULL)
	{
		return;
	}
	PxRigidActor* rigid = actor != NULL ? actor->is<PxRigidActor>() : NULL;
	*outPose = PxwPose(rigid != NULL ? rigid->getGlobalPose() : PxTransform(PxIdentity));
}

void PxwBodyTeleport(PxActor* actor, const PxwPose* pose, const PxVec3* velocity, const PxVec3* angularVelocity)
{
	if (actor == NULL || pose == NULL)
	{
		return;
	}
	PxRigidActor* rigid = actor->is<PxRigidActor>();
	if (rigid == NULL)
	{
		return;
	}

	// A placement, not a physical move: the pose is set directly rather than integrated,
	// which is what bringing a pooled body into play needs.
	rigid->setGlobalPose(pose->ToPxTransform(), false);

	PxRigidDynamic* dynamic = rigid->is<PxRigidDynamic>();
	if (dynamic == NULL)
	{
		return;
	}
	if (dynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		return;
	}

	dynamic->setLinearVelocity(velocity != NULL ? *velocity : PxVec3(0.0f), false);
	dynamic->setAngularVelocity(angularVelocity != NULL ? *angularVelocity : PxVec3(0.0f), false);
	dynamic->clearForce(PxForceMode::eFORCE);
	dynamic->clearForce(PxForceMode::eIMPULSE);
	dynamic->clearTorque(PxForceMode::eFORCE);
	dynamic->clearTorque(PxForceMode::eIMPULSE);

	// Re-pin the wake counter the way a restore does, so a body brought out of a pool is
	// awake and simulated rather than inheriting whatever sleep state the slot last held.
	// A body only carries a wake counter while it is in a scene.
	if (dynamic->getScene() != NULL)
	{
		dynamic->setWakeCounter(kNeverSleepWakeCounter);
	}
}

void PxwBodyGetLinearVelocity(PxActor* actor, PxVec3* outVelocity)
{
	if (outVelocity == NULL)
	{
		return;
	}
	PxRigidBody* body = actor != NULL ? actor->is<PxRigidBody>() : NULL;
	*outVelocity = body != NULL ? body->getLinearVelocity() : PxVec3(0.0f);
}

void PxwBodySetLinearVelocity(PxActor* actor, const PxVec3* velocity)
{
	if (actor == NULL || velocity == NULL)
	{
		return;
	}
	// The velocity setters live on PxRigidDynamic, not PxRigidBody: an articulation link
	// is a body but its velocity is a solver output that cannot be written directly.
	PxRigidDynamic* body = actor->is<PxRigidDynamic>();
	if (body == NULL || body->getScene() == NULL)
	{
		return;
	}
	if (body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		return;
	}
	// autowake false: waking a body that was asleep would change the simulation being
	// driven; the framework owns wake state through its pinned wake counter.
	body->setLinearVelocity(*velocity, false);
}

void PxwBodyGetAngularVelocity(PxActor* actor, PxVec3* outVelocity)
{
	if (outVelocity == NULL)
	{
		return;
	}
	PxRigidBody* body = actor != NULL ? actor->is<PxRigidBody>() : NULL;
	*outVelocity = body != NULL ? body->getAngularVelocity() : PxVec3(0.0f);
}

void PxwBodySetAngularVelocity(PxActor* actor, const PxVec3* velocity)
{
	if (actor == NULL || velocity == NULL)
	{
		return;
	}
	// See PxwBodySetLinearVelocity: the setter is a PxRigidDynamic member.
	PxRigidDynamic* body = actor->is<PxRigidDynamic>();
	if (body == NULL || body->getScene() == NULL)
	{
		return;
	}
	if (body->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
	{
		return;
	}
	body->setAngularVelocity(*velocity, false);
}

float PxwBodyGetMass(PxActor* actor)
{
	PxRigidBody* body = actor != NULL ? actor->is<PxRigidBody>() : NULL;
	return body != NULL ? body->getMass() : 0.0f;
}

// --------------------------------------------------------------------- mass ----

namespace
{
	// A diagonalisation is free to return either q or -q for the same rotation, and
	// which one comes out depends on the last bits of the input. Pick one, so that two
	// peers whose tensors differ imperceptibly cannot end up storing opposite
	// quaternions and hashing differently.
	PxQuat CanonicalSign(const PxQuat& q)
	{
		if (q.w > 0.0f) return q;
		if (q.w < 0.0f) return PxQuat(-q.x, -q.y, -q.z, -q.w);

		// w is exactly zero, a half turn. Fall through the axis components in a fixed
		// order so the choice is still total.
		if (q.x != 0.0f) return q.x > 0.0f ? q : PxQuat(-q.x, -q.y, -q.z, -q.w);
		if (q.y != 0.0f) return q.y > 0.0f ? q : PxQuat(-q.x, -q.y, -q.z, -q.w);
		if (q.z != 0.0f) return q.z > 0.0f ? q : PxQuat(-q.x, -q.y, -q.z, -q.w);
		return q;
	}

	// One shape's contribution to the body, in a form that can be ordered.
	//
	// Summing has to happen in a fixed order because floating point addition is not
	// associative, and attachment order is a fragile thing to rely on: it depends on
	// how a prefab was authored and on the order a loader happens to walk a hierarchy.
	// Sorting on the contribution itself removes the dependency, and it does so without
	// looking at mesh pointers, which differ between peers anyway.
	//
	// Two entries that compare equal are byte-identical, so their relative order cannot
	// change the sum. The comparison orders by raw bytes: the ordering it produces is
	// arbitrary but identical on every peer of the same architecture, which is the same
	// condition PhysX already requires for determinism.
	struct ShapeContribution
	{
		PxMassProperties props;
		PxTransform pose;

		bool operator<(const ShapeContribution& other) const
		{
			return std::memcmp(this, &other, sizeof(ShapeContribution)) < 0;
		}
	};
}

PxI32 PxwComputeMassProperties(PxRigidBody* actor, PxReal density, PxReal isotropyTolerance,
                               bool includeNonSimShapes, PxwMassProperties* out)
{
	if (actor == NULL || out == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	if (!(density > 0.0f))
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwComputeMassProperties: density must be positive");
		return PxwResult::eBAD_FORMAT;
	}
	if (isotropyTolerance < 0.0f)
	{
		isotropyTolerance = PXW_DEFAULT_ISOTROPY_TOLERANCE;
	}

	*out = PxwMassProperties();

	const PxU32 shapeCount = actor->getNbShapes();
	if (shapeCount == 0)
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwComputeMassProperties: actor has no shapes");
		return PxwResult::eBAD_FORMAT;
	}

	std::vector<PxShape*> shapes(shapeCount, static_cast<PxShape*>(NULL));
	actor->getShapes(shapes.data(), shapeCount);

	std::vector<ShapeContribution> contributions;
	contributions.reserve(shapeCount);

	for (PxU32 i = 0; i < shapeCount; ++i)
	{
		PxShape* shape = shapes[i];
		if (shape == NULL)
		{
			continue;
		}
		if (!includeNonSimShapes && !(shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE))
		{
			continue;
		}

		ShapeContribution contribution;
		std::memset(&contribution, 0, sizeof(contribution));
		contribution.props = PxMassProperties(shape->getGeometry()) * density;
		contribution.pose = shape->getLocalPose();
		contributions.push_back(contribution);
	}

	if (contributions.empty())
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwComputeMassProperties: actor has no shapes that contribute mass");
		return PxwResult::eBAD_FORMAT;
	}

	// Canonical summation order, so the answer does not depend on how the shapes
	// happened to be attached. See ShapeContribution.
	std::sort(contributions.begin(), contributions.end());

	std::vector<PxMassProperties> props;
	std::vector<PxTransform> poses;
	props.reserve(contributions.size());
	poses.reserve(contributions.size());
	for (size_t i = 0; i < contributions.size(); ++i)
	{
		props.push_back(contributions[i].props);
		poses.push_back(contributions[i].pose);
	}

	const PxMassProperties total = PxMassProperties::sum(props.data(), poses.data(), static_cast<PxU32>(props.size()));
	if (!(total.mass > 0.0f))
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwComputeMassProperties: computed mass is not positive");
		return PxwResult::eBAD_FORMAT;
	}

	PxQuat massFrame(PxIdentity);
	PxVec3 diagonal = PxMassProperties::getMassSpaceInertia(total.inertiaTensor, massFrame);

	const PxReal largest = PxMax(diagonal.x, PxMax(diagonal.y, diagonal.z));
	const PxReal smallest = PxMin(diagonal.x, PxMin(diagonal.y, diagonal.z));
	out->anisotropy = largest > 0.0f ? (largest - smallest) / largest : 0.0f;

	if (!(diagonal.x > 0.0f) || !(diagonal.y > 0.0f) || !(diagonal.z > 0.0f))
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwComputeMassProperties: inertia tensor has non-positive principal moments");
		return PxwResult::eBAD_FORMAT;
	}

	if (out->anisotropy <= isotropyTolerance)
	{
		// The body is inertially close enough to a sphere that its principal axes carry
		// no information, only noise. Collapsing to the identity costs at most
		// `anisotropy` in each moment, and buys a mass frame that cannot swing under a
		// last-bit change in the inputs. It also leaves the actor pose round trip exact,
		// since a rotated mass frame is what makes it lossy.
		const PxReal mean = (diagonal.x + diagonal.y + diagonal.z) / 3.0f;
		diagonal = PxVec3(mean, mean, mean);
		massFrame = PxQuat(PxIdentity);
		out->massFrameCollapsed = 1;
	}
	else
	{
		massFrame = CanonicalSign(massFrame.getNormalized());

		if (out->anisotropy < 0.05f)
		{
			char message[256];
			snprintf(message, sizeof(message),
				"PxwComputeMassProperties: principal moments differ by only %.3f%%, so the mass frame is "
				"ill conditioned. Replicate PxwMassProperties rather than recomputing it per peer, or raise "
				"the isotropy tolerance above %.4f to collapse the frame.",
				double(out->anisotropy) * 100.0, double(out->anisotropy));
			PxwLog(PxwLogSeverity::eWARNING, message);
		}
	}

	out->mass = total.mass;
	out->inertia = diagonal;
	out->cMassLocalPose = PxwPose(PxTransform(total.centerOfMass, massFrame));
	out->shapeCount = static_cast<PxU32>(props.size());

	return PxwResult::eOK;
}

PxI32 PxwApplyMassProperties(PxRigidBody* actor, const PxwMassProperties* props)
{
	if (actor == NULL || props == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	if (!(props->mass > 0.0f) || !(props->inertia.x > 0.0f) || !(props->inertia.y > 0.0f) || !(props->inertia.z > 0.0f))
	{
		PxwLog(PxwLogSeverity::eERROR, "PxwApplyMassProperties: mass and inertia must all be positive");
		return PxwResult::eBAD_FORMAT;
	}

	// Order matters only in that the mass frame must be set through setCMassLocalPose,
	// which preserves the actor pose. Mass and inertia are independent of it.
	actor->setMass(props->mass);
	actor->setMassSpaceInertiaTensor(props->inertia);
	actor->setCMassLocalPose(props->cMassLocalPose.ToPxTransform());

	return PxwResult::eOK;
}

PxI32 PxwSetupDeterministicMass(PxRigidBody* actor, PxReal density, PxReal isotropyTolerance,
                                bool includeNonSimShapes, PxwMassProperties* out)
{
	PxwMassProperties computed;
	const PxI32 result = PxwComputeMassProperties(actor, density, isotropyTolerance, includeNonSimShapes, &computed);
	if (result != PxwResult::eOK)
	{
		return result;
	}

	const PxI32 applied = PxwApplyMassProperties(actor, &computed);
	if (out != NULL)
	{
		*out = computed;
	}
	return applied;
}

PxU64 PxwHashMassProperties(const PxwMassProperties* props)
{
	if (props == NULL)
	{
		return 0;
	}

	// Only the fields that change the simulation. anisotropy and massFrameCollapsed are
	// diagnostics derived from them, so including them would add nothing but would make
	// the hash sensitive to a tolerance change that did not alter the outcome.
	PxU64 hash = kFnvOffsetBasis;
	hash = FnvAccumulate(hash, &props->mass, sizeof(props->mass));
	hash = FnvAccumulate(hash, &props->inertia, sizeof(props->inertia));
	hash = FnvAccumulate(hash, &props->cMassLocalPose, sizeof(props->cMassLocalPose));
	hash = FnvAccumulate(hash, &props->shapeCount, sizeof(props->shapeCount));
	return hash;
}

PxI32 PxwGetMassProperties(PxRigidBody* actor, PxwMassProperties* out)
{
	if (actor == NULL || out == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}

	*out = PxwMassProperties();
	out->mass = actor->getMass();
	out->inertia = actor->getMassSpaceInertiaTensor();
	out->cMassLocalPose = PxwPose(actor->getCMassLocalPose());
	out->shapeCount = actor->getNbShapes();

	const PxVec3& d = out->inertia;
	const PxReal largest = PxMax(d.x, PxMax(d.y, d.z));
	const PxReal smallest = PxMin(d.x, PxMin(d.y, d.z));
	out->anisotropy = largest > 0.0f ? (largest - smallest) / largest : 0.0f;

	return PxwResult::eOK;
}

// -------------------------------------------------------------------- world ----

PxwWorld* PxwWorldCreate(const PxwSceneDesc* desc)
{
	if (desc == NULL)
	{
		return NULL;
	}

	PxScene* scene = GetGlobalPhysXWrapper().CreateSceneEx(*desc);
	if (scene == NULL)
	{
		return NULL;
	}

	PxwWorld* world = new PxwWorld();
	world->scene = scene;
	return world;
}

void PxwWorldDestroy(PxwWorld* world)
{
	if (world == NULL)
	{
		return;
	}

	if (world->simulating)
	{
		PxwWorldFetchResults(world);
	}

	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		ReleaseArticulationCache(world->entries[i]);
	}
	world->entries.clear();

	if (world->scene != NULL)
	{
		GetGlobalPhysXWrapper().ReleaseScene(world->scene);
		world->scene = NULL;
	}

	delete world;
}

PxScene* PxwWorldGetScene(PxwWorld* world)
{
	return world != NULL ? world->scene : NULL;
}

PxI32 PxwWorldRegister(PxwWorld* world, PxU32 stableId, void* handle, PxU32 kind)
{
	if (world == NULL || handle == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	if (kind > PxwHandleKind::eVEHICLE)
	{
		return PxwResult::eNULL_ARGUMENT;
	}

	const size_t insertAt = world->LowerBound(stableId);
	if (insertAt < world->entries.size() && world->entries[insertAt].stableId == stableId)
	{
		// Re-registering an ID that is queued for removal simply revives it, which is
		// what a pool does when an object is recycled in the same tick it was freed.
		PxwWorldEntry& existing = world->entries[insertAt];
		if (existing.pendingRemove && existing.handle == handle)
		{
			existing.pendingRemove = false;
			return PxwResult::eOK;
		}
		return PxwResult::eDUPLICATE_ID;
	}

	PxwWorldEntry entry;
	entry.stableId = stableId;
	entry.kind = kind;
	entry.handle = handle;
	entry.pendingAdd = true;

	if (kind == PxwHandleKind::eARTICULATION)
	{
		PxArticulationReducedCoordinate* articulation = static_cast<PxArticulationReducedCoordinate*>(handle);
		entry.dofCount = articulation->getDofs();
		entry.linkCount = articulation->getNbLinks();
	}

	world->entries.insert(world->entries.begin() + static_cast<std::ptrdiff_t>(insertAt), entry);
	return PxwResult::eOK;
}

PxI32 PxwWorldUnregister(PxwWorld* world, PxU32 stableId)
{
	if (world == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}

	const PxI32 index = world->Find(stableId);
	if (index < 0)
	{
		return PxwResult::eUNKNOWN_ID;
	}

	PxwWorldEntry& entry = world->entries[static_cast<size_t>(index)];
	entry.pendingRemove = true;
	entry.pendingAdd = false;
	return PxwResult::eOK;
}

PxI32 PxwWorldCommitPending(PxwWorld* world)
{
	if (world == NULL || world->scene == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	if (world->simulating)
	{
		return PxwResult::eBUSY;
	}

	// Removals first, then additions, both in ascending stable-ID order. Every peer
	// therefore issues an identical sequence of scene mutations regardless of the
	// order in which gameplay code requested them.
	for (size_t i = world->entries.size(); i-- > 0; )
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!entry.pendingRemove)
		{
			continue;
		}
		RemoveEntryFromScene(world->scene, entry);
		ReleaseArticulationCache(entry);
		world->entries.erase(world->entries.begin() + static_cast<std::ptrdiff_t>(i));
	}

	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!entry.pendingAdd)
		{
			continue;
		}
		AddEntryToScene(world->scene, entry);
		entry.pendingAdd = false;

		if (!entry.enabled)
		{
			ApplyEnabled(entry, false);
		}
		else
		{
			// A body only has a wake counter worth setting once it is in a scene, so
			// the pin is applied here rather than at registration. It stays pinned
			// while awake; framework sleeping is what takes it down again.
			PinWakeCounter(entry);
		}
	}

	// The scene actor set just changed, so the query reverse index is rebuilt to match.
	RebuildActorLookup(*world);

	return PxwResult::eOK;
}

PxU32 PxwWorldGetEntryCount(PxwWorld* world)
{
	return world != NULL ? static_cast<PxU32>(world->entries.size()) : 0u;
}

void* PxwWorldFindHandle(PxwWorld* world, PxU32 stableId)
{
	if (world == NULL)
	{
		return NULL;
	}
	const PxI32 index = world->Find(stableId);
	return index >= 0 ? world->entries[static_cast<size_t>(index)].handle : NULL;
}

PxI32 PxwWorldSetEntryEnabled(PxwWorld* world, PxU32 stableId, bool enabled)
{
	if (world == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	const PxI32 index = world->Find(stableId);
	if (index < 0)
	{
		return PxwResult::eUNKNOWN_ID;
	}
	ApplyEnabled(world->entries[static_cast<size_t>(index)], enabled);
	return PxwResult::eOK;
}

PxI32 PxwWorldSetSleepParams(PxwWorld* world, PxReal linearThreshold, PxReal angularThreshold, PxU32 sleepTicks)
{
	if (world == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}

	world->sleepLinThresholdSq = linearThreshold * linearThreshold;
	world->sleepAngThresholdSq = angularThreshold * angularThreshold;
	world->sleepTicks = sleepTicks;

	// Intended to be called once at world creation, before anything is committed, so
	// this loop usually does nothing. It exists so a later call is still well
	// defined: awake bodies keep their pin and restart their rest counter, and
	// anything the framework had already slept is left alone until it is next touched.
	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!entry.inScene || !entry.enabled)
		{
			continue;
		}
		entry.restTicks = 0;
		PinWakeCounter(entry);
	}

	return PxwResult::eOK;
}

void PxwWorldSimulate(PxwWorld* world, PxReal dt)
{
	if (world == NULL || world->scene == NULL || world->simulating)
	{
		return;
	}
	world->simulating = true;
	PxwSceneSimulate(world->scene, dt);
}

void PxwWorldFetchResults(PxwWorld* world)
{
	if (world == NULL || world->scene == NULL || !world->simulating)
	{
		return;
	}
	PxwSceneFetchResults(world->scene);
	world->simulating = false;

	// After fetchResults so it sees post-step velocities, and before the caller
	// captures, so the sleep decision it makes is part of the recorded state.
	UpdateSleepForWorld(*world);
}

void PxwWorldStep(PxwWorld* world, PxReal dt)
{
	PxwWorldSimulate(world, dt);
	PxwWorldFetchResults(world);
}

void PxwWorldResetContactState(PxwWorld* world)
{
	PxwWorldResetContactStateEx(world, PxwContactResetMode::eRESET_FILTERING);
}

void PxwWorldResetContactStateEx(PxwWorld* world, PxU32 mode)
{
	if (world == NULL || world->scene == NULL || world->simulating || mode == PxwContactResetMode::eNONE)
	{
		return;
	}

	PxScene* scene = world->scene;

	// Both modes iterate the registry rather than the scene's actor list, so the
	// order is the stable-ID order every peer shares.
	if (mode == PxwContactResetMode::eRESET_FILTERING)
	{
		for (size_t i = 0; i < world->entries.size(); ++i)
		{
			PxwWorldEntry& entry = world->entries[i];
			if (!entry.inScene)
			{
				continue;
			}

			if (entry.kind == PxwHandleKind::eARTICULATION)
			{
				PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
				if (articulation == NULL)
				{
					continue;
				}
				const PxU32 linkCount = articulation->getNbLinks();
				std::vector<PxArticulationLink*> links(linkCount);
				articulation->getLinks(links.data(), linkCount, 0);
				for (PxU32 link = 0; link < linkCount; ++link)
				{
					scene->resetFiltering(*links[link]);
				}
				continue;
			}

			PxRigidActor* actor = AsRigidActor(entry);
			if (actor != NULL)
			{
				scene->resetFiltering(*actor);
			}
		}
		return;
	}

	// eREINSERT: tear the scene down and rebuild it. Removals run in descending
	// stable-ID order and additions in ascending order, mirroring PxwWorldCommitPending
	// so the resulting scene composition is identical to a freshly built world.
	for (size_t i = world->entries.size(); i-- > 0; )
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!entry.inScene)
		{
			continue;
		}
		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			// The cache is tied to the articulation, not the scene, so it survives.
			scene->removeArticulation(*static_cast<PxArticulationReducedCoordinate*>(entry.handle));
		}
		else
		{
			PxRigidActor* actor = AsRigidActor(entry);
			if (actor != NULL)
			{
				// wakeOnLostTouch would perturb the very sleep state being restored.
				scene->removeActor(*actor, false);
			}
		}
	}

	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!entry.inScene)
		{
			continue;
		}
		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			scene->addArticulation(*static_cast<PxArticulationReducedCoordinate*>(entry.handle));
		}
		else
		{
			PxRigidActor* actor = AsRigidActor(entry);
			if (actor != NULL)
			{
				scene->addActor(*actor);
			}
		}
	}
}

// -------------------------------------------------------------------- state ----

PxU32 PxwWorldStateSize(PxwWorld* world)
{
	if (world == NULL)
	{
		return 0;
	}

	PxU32 total = static_cast<PxU32>(sizeof(StateHeader));
	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		const PxwWorldEntry& entry = world->entries[i];
		if (!IsCaptured(entry))
		{
			continue;
		}
		total += static_cast<PxU32>(sizeof(EntryHeader)) + PayloadSize(entry);
	}
	return total;
}

PxU32 PxwWorldCaptureState(PxwWorld* world, void* dst, PxU32 capacity, PxU64* outHash)
{
	if (world == NULL || dst == NULL)
	{
		return 0;
	}
	if (world->simulating)
	{
		LogMessage(PxErrorCode::eINVALID_OPERATION, "UNDPWR: cannot capture state while the scene is simulating.\n");
		return 0;
	}

	const PxU32 required = PxwWorldStateSize(world);
	if (capacity < required)
	{
		return 0;
	}

	PxU8* cursor = static_cast<PxU8*>(dst);
	StateHeader* header = reinterpret_cast<StateHeader*>(cursor);
	cursor += sizeof(StateHeader);

	PxU32 captured = 0;
	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!IsCaptured(entry))
		{
			continue;
		}

		EntryHeader* entryHeader = reinterpret_cast<EntryHeader*>(cursor);
		entryHeader->stableId = entry.stableId;
		entryHeader->kind = entry.kind;
		entryHeader->payloadBytes = PayloadSize(entry);
		entryHeader->reserved = 0;
		cursor += sizeof(EntryHeader);

		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			ArticulationPayload* payload = reinterpret_cast<ArticulationPayload*>(cursor);
			PxReal* joints = reinterpret_cast<PxReal*>(cursor + sizeof(ArticulationPayload));
			CaptureArticulation(entry, *payload, joints);
		}
		else
		{
			RigidPayload* payload = reinterpret_cast<RigidPayload*>(cursor);
			CaptureRigid(entry, *payload);
		}

		cursor += entryHeader->payloadBytes;
		++captured;
	}

	const PxU32 written = static_cast<PxU32>(cursor - static_cast<PxU8*>(dst));
	header->magic = kStateMagic;
	header->version = kStateVersion;
	header->entryCount = captured;
	header->totalBytes = written;

	if (outHash != NULL)
	{
		// The header is included so a changed entry count is itself a hash change.
		*outHash = FnvAccumulate(kFnvOffsetBasis, dst, written);
	}

	return written;
}

PxI32 PxwWorldRestoreState(PxwWorld* world, const void* src, PxU32 size)
{
	if (world == NULL || src == NULL)
	{
		return PxwResult::eNULL_ARGUMENT;
	}
	if (world->simulating)
	{
		return PxwResult::eBUSY;
	}
	if (size < sizeof(StateHeader))
	{
		return PxwResult::eBAD_FORMAT;
	}

	const PxU8* cursor = static_cast<const PxU8*>(src);
	const PxU8* end = cursor + size;

	const StateHeader* header = reinterpret_cast<const StateHeader*>(cursor);
	cursor += sizeof(StateHeader);

	if (header->magic != kStateMagic)
	{
		return PxwResult::eBAD_FORMAT;
	}
	if (header->version != kStateVersion)
	{
		return PxwResult::eVERSION_MISMATCH;
	}
	if (header->totalBytes > size)
	{
		return PxwResult::eBAD_FORMAT;
	}

	bool sawUnknownEntry = false;

	for (PxU32 i = 0; i < header->entryCount; ++i)
	{
		if (cursor + sizeof(EntryHeader) > end)
		{
			return PxwResult::eBAD_FORMAT;
		}

		const EntryHeader* entryHeader = reinterpret_cast<const EntryHeader*>(cursor);
		cursor += sizeof(EntryHeader);

		if (cursor + entryHeader->payloadBytes > end)
		{
			return PxwResult::eBAD_FORMAT;
		}

		const PxI32 index = world->Find(entryHeader->stableId);
		if (index < 0)
		{
			// The snapshot references a body this peer does not have. Keep restoring
			// the rest so the caller can diff, but report the mismatch.
			sawUnknownEntry = true;
			cursor += entryHeader->payloadBytes;
			continue;
		}

		PxwWorldEntry& entry = world->entries[static_cast<size_t>(index)];
		if (entry.kind != entryHeader->kind)
		{
			sawUnknownEntry = true;
			cursor += entryHeader->payloadBytes;
			continue;
		}

		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			const ArticulationPayload* payload = reinterpret_cast<const ArticulationPayload*>(cursor);
			const PxReal* joints = reinterpret_cast<const PxReal*>(cursor + sizeof(ArticulationPayload));
			RestoreArticulation(entry, *payload, joints);
			ApplyEnabled(entry, (payload->flags & StateFlag::eDISABLED) == 0);
		}
		else
		{
			const RigidPayload* payload = reinterpret_cast<const RigidPayload*>(cursor);
			RestoreRigid(entry, *payload);
			ApplyEnabled(entry, (payload->flags & StateFlag::eDISABLED) == 0);
		}

		cursor += entryHeader->payloadBytes;
	}

	return sawUnknownEntry ? PxwResult::eENTRY_MISMATCH : PxwResult::eOK;
}

PxU64 PxwWorldHashState(PxwWorld* world)
{
	if (world == NULL)
	{
		return 0;
	}

	const PxU32 required = PxwWorldStateSize(world);
	if (world->scratch.size() < required)
	{
		world->scratch.resize(required);
	}

	PxU64 hash = 0;
	const PxU32 written = PxwWorldCaptureState(world, world->scratch.data(), static_cast<PxU32>(world->scratch.size()), &hash);
	return written > 0 ? hash : 0;
}

PxU32 PxwWorldHashPerEntry(PxwWorld* world, PxwEntryHash* dst, PxU32 capacity)
{
	if (world == NULL || dst == NULL || world->simulating)
	{
		return 0;
	}

	PxU32 count = 0;
	for (size_t i = 0; i < world->entries.size() && count < capacity; ++i)
	{
		PxwWorldEntry& entry = world->entries[i];
		if (!IsCaptured(entry))
		{
			continue;
		}

		PxU64 hash = kFnvOffsetBasis;
		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			const size_t jointFloats = static_cast<size_t>(entry.dofCount) * 3;
			if (world->scratch.size() < sizeof(ArticulationPayload) + sizeof(PxReal) * jointFloats)
			{
				world->scratch.resize(sizeof(ArticulationPayload) + sizeof(PxReal) * jointFloats);
			}
			ArticulationPayload* payload = reinterpret_cast<ArticulationPayload*>(world->scratch.data());
			PxReal* joints = reinterpret_cast<PxReal*>(world->scratch.data() + sizeof(ArticulationPayload));
			CaptureArticulation(entry, *payload, joints);
			hash = FnvAccumulate(hash, payload, sizeof(ArticulationPayload) + sizeof(PxReal) * jointFloats);
		}
		else
		{
			RigidPayload payload;
			CaptureRigid(entry, payload);
			hash = FnvAccumulate(hash, &payload, sizeof(payload));
		}

		dst[count].stableId = entry.stableId;
		dst[count].kind = entry.kind;
		dst[count].hash = hash;
		++count;
	}

	return count;
}

PxU32 PxwWorldReadPoses(PxwWorld* world, PxwPoseEntry* dst, PxU32 capacity)
{
	if (world == NULL || dst == NULL)
	{
		return 0;
	}

	PxU32 count = 0;
	for (size_t i = 0; i < world->entries.size() && count < capacity; ++i)
	{
		const PxwWorldEntry& entry = world->entries[i];
		if (entry.handle == NULL)
		{
			continue;
		}

		PxTransform pose(PxIdentity);
		if (entry.kind == PxwHandleKind::eARTICULATION)
		{
			PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
			if (articulation == NULL)
			{
				continue;
			}
			pose = articulation->getRootGlobalPose();
		}
		else
		{
			PxRigidActor* actor = AsRigidActor(entry);
			if (actor == NULL)
			{
				continue;
			}
			pose = actor->getGlobalPose();
		}

		dst[count].stableId = entry.stableId;
		dst[count].kind = entry.kind;
		dst[count].pose = PxwPose(pose);
		++count;
	}

	return count;
}

static void FillInternalIdEntry(const PxwWorldEntry& entry, PxwInternalIdEntry& out)
{
	out.stableId = entry.stableId;
	out.kind = entry.kind;
	out.internalActorIndex = 0xFFFFFFFFu;
	out.padding = 0;
	out.islandNodeIndex = 0xFFFFFFFFFFFFFFFFull;

	if (entry.kind == PxwHandleKind::eARTICULATION)
	{
		// An articulation has no actor index of its own; its root link carries the
		// identity that matters for solver ordering.
		PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
		if (articulation != NULL && articulation->getNbLinks() > 0)
		{
			PxArticulationLink* root = NULL;
			articulation->getLinks(&root, 1, 0);
			if (root != NULL)
			{
				out.internalActorIndex = root->getInternalActorIndex();
				out.islandNodeIndex = root->getInternalIslandNodeIndex().index();
			}
		}
		return;
	}

	PxRigidActor* actor = AsRigidActor(entry);
	if (actor == NULL)
	{
		return;
	}

	out.internalActorIndex = actor->getInternalActorIndex();

	// Statics have no island node; only bodies participate in islands.
	PxRigidBody* body = actor->is<PxRigidBody>();
	if (body != NULL)
	{
		out.islandNodeIndex = body->getInternalIslandNodeIndex().index();
	}
}

PxU32 PxwWorldReadInternalIds(PxwWorld* world, PxwInternalIdEntry* dst, PxU32 capacity)
{
	if (world == NULL || dst == NULL)
	{
		return 0;
	}

	PxU32 count = 0;
	for (size_t i = 0; i < world->entries.size() && count < capacity; ++i)
	{
		const PxwWorldEntry& entry = world->entries[i];
		if (entry.handle == NULL)
		{
			continue;
		}

		FillInternalIdEntry(entry, dst[count]);
		++count;
	}

	return count;
}

PxU64 PxwWorldHashInternalIds(PxwWorld* world)
{
	if (world == NULL)
	{
		return 0;
	}

	PxU64 hash = kFnvOffsetBasis;
	for (size_t i = 0; i < world->entries.size(); ++i)
	{
		const PxwWorldEntry& entry = world->entries[i];
		if (entry.handle == NULL)
		{
			continue;
		}

		PxwInternalIdEntry record;
		FillInternalIdEntry(entry, record);
		hash = FnvAccumulate(hash, &record, sizeof(record));
	}

	return hash;
}

PxU32 PxwWorldReadArticulationLinkPoses(PxwWorld* world, PxU32 stableId, PxwTransformData* dst, PxU32 capacity)
{
	if (world == NULL || dst == NULL)
	{
		return 0;
	}

	const PxI32 index = world->Find(stableId);
	if (index < 0)
	{
		return 0;
	}

	const PxwWorldEntry& entry = world->entries[static_cast<size_t>(index)];
	PxArticulationReducedCoordinate* articulation = AsArticulation(entry);
	if (articulation == NULL)
	{
		return 0;
	}

	const PxU32 linkCount = articulation->getNbLinks();
	if (linkCount == 0)
	{
		return 0;
	}

	std::vector<PxArticulationLink*> links(linkCount);
	articulation->getLinks(links.data(), linkCount, 0);

	const PxU32 count = linkCount < capacity ? linkCount : capacity;
	for (PxU32 i = 0; i < count; ++i)
	{
		// Written at the link's own index so the order matches GetArticulationLinkIndex
		// on the managed side rather than the order getLinks happened to return.
		const PxU32 linkIndex = links[i]->getLinkIndex();
		if (linkIndex < capacity)
		{
			dst[linkIndex] = PxwTransformData(links[i]->getGlobalPose());
		}
	}

	return count;
}

PxU64 PxwHashBuffer(const void* src, PxU32 size)
{
	if (src == NULL || size == 0)
	{
		return 0;
	}
	return FnvAccumulate(kFnvOffsetBasis, src, size);
}

// ------------------------------------------------------------- scene queries ----

namespace
{
	// True when a hit on a body of the given kind passes the caller's filter. A zero mask
	// matches everything; otherwise the mask is ANDed against a single bit per kind.
	inline bool PassesFilter(PxU32 filterMask, PxU32 kind)
	{
		return filterMask == 0u || (filterMask & (1u << kind)) != 0u;
	}

	// Resolves, filters and sorts the touches from a raycast or sweep into the caller's
	// buffer. PxRaycastHit and PxSweepHit share the fields this reads, so one template
	// serves both. Hits on unregistered actors are dropped; the rest are sorted by
	// distance then stable ID and truncated to capacity from the front.
	template <typename HitT>
	PxU32 EmitSortedLineHits(const PxwWorld& world, const HitT* touches, PxU32 touchCount,
		PxU32 filterMask, PxwRaycastHit* hits, PxU32 capacity)
	{
		std::vector<PxwRaycastHit> resolved;
		resolved.reserve(touchCount);

		for (PxU32 i = 0; i < touchCount; ++i)
		{
			const HitT& t = touches[i];
			PxU32 stableId = 0;
			PxU32 kind = 0;
			if (t.actor == NULL || !world.ResolveActor(t.actor, stableId, kind))
			{
				continue;
			}
			if (!PassesFilter(filterMask, kind))
			{
				continue;
			}

			PxwRaycastHit h;
			h.stableId = stableId;
			h.kind = kind;
			h.point = t.position;
			h.normal = t.normal;
			h.distance = t.distance;
			h.faceIndex = t.faceIndex;
			resolved.push_back(h);
		}

		std::sort(resolved.begin(), resolved.end(),
			[](const PxwRaycastHit& a, const PxwRaycastHit& b)
			{
				if (a.distance != b.distance)
				{
					return a.distance < b.distance;
				}
				return a.stableId < b.stableId;
			});

		PxU32 count = static_cast<PxU32>(resolved.size());
		if (count > capacity)
		{
			count = capacity;
		}
		for (PxU32 i = 0; i < count; ++i)
		{
			hits[i] = resolved[i];
		}
		return count;
	}
}

PxU32 PxwWorldRaycast(PxwWorld* world, const PxVec3* origin, const PxVec3* direction, PxReal maxDistance,
	PxU32 filterMask, PxwRaycastHit* hits, PxU32 capacity)
{
	if (world == NULL || world->scene == NULL || origin == NULL || direction == NULL ||
		hits == NULL || capacity == 0u || maxDistance <= 0.0f)
	{
		return 0;
	}

	PxVec3 unitDir = *direction;
	const PxReal length = unitDir.magnitude();
	if (length <= 0.0f)
	{
		return 0;
	}
	unitDir /= length;

	PxRaycastHit touches[kMaxQueryTouches];
	PxRaycastBuffer buffer(touches, kMaxQueryTouches);
	const PxHitFlags hitFlags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
	// eNO_BLOCK turns every hit into a touch, so the whole set is gathered and then sorted
	// here rather than letting PhysX pick a single blocking hit in an unspecified order.
	const PxQueryFilterData filterData(PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::eNO_BLOCK));

	world->scene->raycast(*origin, unitDir, maxDistance, buffer, hitFlags, filterData);

	return EmitSortedLineHits(*world, buffer.touches, buffer.nbTouches, filterMask, hits, capacity);
}

PxU32 PxwWorldOverlap(PxwWorld* world, PxU32 shape, const PxVec3* center, const PxVec3* halfExtents, PxReal radius,
	const PxQuat* rotation, PxU32 filterMask, PxwOverlapHit* hits, PxU32 capacity)
{
	if (world == NULL || world->scene == NULL || center == NULL || hits == NULL || capacity == 0u)
	{
		return 0;
	}

	const PxVec3 extents = halfExtents != NULL ? *halfExtents : PxVec3(0.0f);
	PxSphereGeometry sphere(1.0f);
	PxBoxGeometry box(1.0f, 1.0f, 1.0f);
	PxCapsuleGeometry capsule(1.0f, 1.0f);
	PxGeometry* geometry = NULL;
	if (!BuildQueryGeometry(shape, extents, radius, sphere, box, capsule, &geometry))
	{
		return 0;
	}

	const PxQuat q = rotation != NULL ? *rotation : PxQuat(PxIdentity);
	const PxTransform pose(*center, q);

	PxOverlapHit touches[kMaxQueryTouches];
	PxOverlapBuffer buffer(touches, kMaxQueryTouches);
	const PxQueryFilterData filterData(PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::eNO_BLOCK));

	world->scene->overlap(*geometry, pose, buffer, filterData);

	std::vector<PxwOverlapHit> resolved;
	resolved.reserve(buffer.nbTouches);
	for (PxU32 i = 0; i < buffer.nbTouches; ++i)
	{
		const PxOverlapHit& t = buffer.touches[i];
		PxU32 stableId = 0;
		PxU32 kind = 0;
		if (t.actor == NULL || !world->ResolveActor(t.actor, stableId, kind))
		{
			continue;
		}
		if (!PassesFilter(filterMask, kind))
		{
			continue;
		}
		PxwOverlapHit h;
		h.stableId = stableId;
		h.kind = kind;
		resolved.push_back(h);
	}

	std::sort(resolved.begin(), resolved.end(),
		[](const PxwOverlapHit& a, const PxwOverlapHit& b)
		{
			return a.stableId < b.stableId;
		});

	PxU32 count = static_cast<PxU32>(resolved.size());
	if (count > capacity)
	{
		count = capacity;
	}
	for (PxU32 i = 0; i < count; ++i)
	{
		hits[i] = resolved[i];
	}
	return count;
}

PxU32 PxwWorldSweep(PxwWorld* world, PxU32 shape, const PxVec3* origin, const PxVec3* halfExtents, PxReal radius,
	const PxQuat* rotation, const PxVec3* direction, PxReal maxDistance,
	PxU32 filterMask, PxwRaycastHit* hits, PxU32 capacity)
{
	if (world == NULL || world->scene == NULL || origin == NULL || direction == NULL ||
		hits == NULL || capacity == 0u || maxDistance <= 0.0f)
	{
		return 0;
	}

	const PxVec3 extents = halfExtents != NULL ? *halfExtents : PxVec3(0.0f);
	PxSphereGeometry sphere(1.0f);
	PxBoxGeometry box(1.0f, 1.0f, 1.0f);
	PxCapsuleGeometry capsule(1.0f, 1.0f);
	PxGeometry* geometry = NULL;
	if (!BuildQueryGeometry(shape, extents, radius, sphere, box, capsule, &geometry))
	{
		return 0;
	}

	PxVec3 unitDir = *direction;
	const PxReal length = unitDir.magnitude();
	if (length <= 0.0f)
	{
		return 0;
	}
	unitDir /= length;

	const PxQuat q = rotation != NULL ? *rotation : PxQuat(PxIdentity);
	const PxTransform pose(*origin, q);

	PxSweepHit touches[kMaxQueryTouches];
	PxSweepBuffer buffer(touches, kMaxQueryTouches);
	const PxHitFlags hitFlags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
	const PxQueryFilterData filterData(PxQueryFlags(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::eNO_BLOCK));

	world->scene->sweep(*geometry, pose, unitDir, maxDistance, buffer, hitFlags, filterData);

	return EmitSortedLineHits(*world, buffer.touches, buffer.nbTouches, filterMask, hits, capacity);
}

// --------------------------------------------------------- contact draining ----

PxU32 PxwWorldDrainContacts(PxwWorld* /*world*/, void* /*dst*/, PxU32 /*capacity*/)
{
	// Intentional no-op: no PxSimulationEventCallback is installed, so there is nothing to
	// drain. Present so the managed host's per-tick drain resolves. See the header.
	return 0;
}

PxU32 PxwWorldDrainTriggers(PxwWorld* /*world*/, void* /*dst*/, PxU32 /*capacity*/)
{
	return 0;
}
