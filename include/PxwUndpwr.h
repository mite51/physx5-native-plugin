#pragma once

// UNDPWR - deterministic simulation support layer.
//
// This header exposes everything the managed rollback framework needs that the
// original wrapper did not provide:
//
//   * explicit scene configuration, including enhanced determinism
//   * per-scene stepping, so several independent worlds can advance in one process
//   * a stable-ID registry that inserts actors into the scene in ID order
//   * bulk state capture / restore / hashing for rollback snapshots
//   * an explicit contact and solver cache reset for hard resynchronisation
//
// Why the registry matters: PhysX only guarantees reproducible results for the same
// scene with the same actors created and inserted in the same order. Gameplay code
// spawns things in whatever order it likes, so the registry defers scene insertion
// and flushes it sorted by a caller-supplied stable ID. Every peer therefore issues
// the identical sequence of PxScene::addActor calls.
//
// What this layer can and cannot guarantee, as measured by tests/PxwUndpwrTests.cpp:
//
//   * Two peers that run the same ticks from the same starting state stay
//     bit-identical indefinitely, regardless of the order gameplay code registered
//     the actors in. This is the guarantee the netcode is built on.
//
//   * A peer that rewinds and resimulates does NOT reproduce its own earlier trace
//     bit-for-bit. PhysX warm-starts the solver from contact impulses stored in the
//     persistent manifolds, and the public API exposes no way to read or write them,
//     so they cannot be part of a snapshot. Over a 30 tick replay the resulting error
//     stays around 2e-06 m, far below anything observable, but it is not zero.
//
//   * No contact reset mode fixes that. Replaying 16 ticks into a world with no
//     history reproduces 0 of them with eNONE, 0 with eRESET_FILTERING and 1 with
//     eREINSERT, so a step can never be made a pure function of the snapshot.
//
//   * Two worlds built from scratch and restored from the same snapshot DO agree
//     bit-for-bit, even when their actors were registered in opposite orders.
//
// Together those last two points decide the design. Bit-exactness does not require a
// shared past, it requires an identical one, so:
//
//   * every peer runs the identical sequence of operations each tick, with a fixed
//     prediction horizon, rather than rewinding by however much its own late inputs
//     happen to demand. A peer that cannot keep up stalls instead of drifting.
//
//   * a peer joining mid-match does not try to match the others. Every peer rebuilds
//     its world from one agreed snapshot at an agreed tick, which puts them all back
//     on an identical history. The same barrier doubles as desync recovery.
//
// Under those two rules confirmed-tick hashes can be compared bit-exactly.

#include "PxPhysicsAPI.h"
#include "PhysXWrapper.h"
#include "DataInterop.h"

#ifndef PHYSX_WRAPPER_API
    #ifdef _WIN32
        #ifdef API_EXPORTS
            #define PHYSX_WRAPPER_API __declspec(dllexport)
        #else
            #define PHYSX_WRAPPER_API __declspec(dllimport)
        #endif
    #elif defined(__linux__)
        #ifdef API_EXPORTS
            #define PHYSX_WRAPPER_API __attribute__((visibility("default")))
        #else
            #define PHYSX_WRAPPER_API
        #endif
    #endif
#endif

using namespace physx;

namespace pxw
{
    class PxwWorld;

    /// Kind discriminator for a registry entry. Determines how state is captured.
    struct PxwHandleKind
    {
        enum Enum : PxU32
        {
            eRIGID_DYNAMIC = 0,   //!< PxRigidDynamic, non-kinematic
            eRIGID_STATIC = 1,    //!< PxRigidStatic; never captured, registered only so queries can resolve its ID
            eRIGID_KINEMATIC = 2, //!< PxRigidDynamic with the kinematic flag set
            eARTICULATION = 3,    //!< PxArticulationReducedCoordinate
            eVEHICLE = 4          //!< PxwVehicle
        };
    };

    /// Result codes returned by the registry and state functions.
    struct PxwResult
    {
        enum Enum : PxI32
        {
            eOK = 0,
            eNULL_ARGUMENT = -1,
            eDUPLICATE_ID = -2,
            eUNKNOWN_ID = -3,
            eBUFFER_TOO_SMALL = -4,
            eBAD_FORMAT = -5,
            eVERSION_MISMATCH = -6,
            eENTRY_MISMATCH = -7,
            eBUSY = -8
        };
    };

    /// Per-entry checksum, used to pinpoint which body diverged during a desync.
    struct PxwEntryHash
    {
        PxU32 stableId;
        PxU32 kind;
        PxU64 hash;
    };

    /// Pose readback record for driving presentation transforms.
    struct PxwPoseEntry
    {
        PxU32 stableId;
        PxU32 kind;
        PxwPose pose;   //!< Quaternion-first, matching the managed SimTransform it marshals into.
    };

    /// The identity PhysX assigned to a registered body, paired with the stable ID the
    /// application knows it by.
    ///
    /// PhysX hands out both indices from insertion order, and they in turn decide the
    /// order the solver visits bodies and islands. Two peers that inserted the same
    /// bodies in different orders will therefore get different indices, sum contact
    /// impulses in a different order, and round differently -- measurably so: a
    /// sixteen-body scene built in reverse order diverges within a few hundred steps.
    ///
    /// Comparing these across peers turns that silent, slow desync into an immediate
    /// and precise error naming the body that was registered out of order.
    struct PxwInternalIdEntry
    {
        PxU32 stableId;
        PxU32 kind;
        PxU32 internalActorIndex;   ///< PxRigidActor::getInternalActorIndex
        PxU32 padding;
        PxU64 islandNodeIndex;      ///< PxRigidBody::getInternalIslandNodeIndex
    };

    /// The shape of a query volume for overlaps and sweeps. Mirrors the managed
    /// SimQueryShape: 0 sphere, 1 box, 2 capsule.
    struct PxwQueryShape
    {
        enum Enum : PxU32
        {
            eSPHERE = 0,
            eBOX = 1,
            eCAPSULE = 2
        };
    };

    /// One hit from a raycast or sweep, resolved to the stable ID of the body it struck.
    /// Laid out to match the managed SimRaycastHit field-for-field, since arrays of these
    /// are written straight across the interop boundary.
    ///
    /// Raycasts and sweeps return their hits sorted by distance ascending, with stable ID
    /// breaking ties. PhysX does not guarantee an order for touching hits, so one is
    /// imposed here where the distances are already known; two peers iterating the same
    /// hits in a different order would take different gameplay decisions and desync.
    struct PxwRaycastHit
    {
        PxU32 stableId;
        PxU32 kind;
        PxVec3 point;      //!< World-space contact point.
        PxVec3 normal;     //!< World-space surface normal at the hit.
        PxReal distance;   //!< Distance along the ray or sweep to the hit.
        PxU32 faceIndex;   //!< Struck triangle for a mesh, 0xFFFFFFFF otherwise.
    };

    /// One body found by an overlap query, resolved to its stable ID. Returned sorted by
    /// stable ID ascending, since an overlap has no natural order to sort by.
    struct PxwOverlapHit
    {
        PxU32 stableId;
        PxU32 kind;
    };

    /// One contact reported after a step, resolved to the two stable IDs in contact.
    /// Laid out to match the managed SimContactEvent field-for-field (36 bytes), since
    /// arrays of these cross the interop boundary directly.
    ///
    /// PhysX reports contacts in the order of its internal pair bookkeeping, which is
    /// exactly the state a snapshot cannot carry, so the raw order differs between peers
    /// and between an original pass and its replay. PxwWorldDrainContacts normalises each
    /// pair to ascending stable-ID order (idA < idB) with the normal oriented from A toward
    /// B, then sorts the whole buffer by (idA, idB), so gameplay sees the same events in
    /// the same order everywhere.
    struct PxwContactEvent
    {
        PxU32 idA;        //!< The smaller of the two stable IDs.
        PxU32 idB;        //!< The larger of the two stable IDs.
        PxVec3 point;     //!< A representative world-space contact point.
        PxVec3 normal;    //!< World-space normal, pointing from A toward B.
        PxReal impulse;   //!< Total normal impulse applied to resolve the contact.
    };

    /// Whether a trigger overlap began or ended this step. Mirrors the managed
    /// SimTriggerStatus.
    struct PxwTriggerStatus
    {
        enum Enum : PxU32
        {
            eLOST = 0,    //!< The other body stopped overlapping the trigger this step.
            eFOUND = 1    //!< The other body began overlapping the trigger this step.
        };
    };

    /// One trigger-volume overlap change reported after a step. Matches the managed
    /// SimTriggerEvent field-for-field (12 bytes). Sorted by (triggerId, otherId) for the
    /// same determinism reason as contacts.
    struct PxwTriggerEvent
    {
        PxU32 triggerId;   //!< Stable ID of the trigger shape's body.
        PxU32 otherId;     //!< Stable ID of the body that entered or left the trigger.
        PxU32 status;      //!< A PxwTriggerStatus.
    };

    /// How aggressively to erase the simulation state PhysX carries between steps but
    /// that no snapshot can capture: persistent contact manifolds, the solver
    /// warm-start impulses held in them, broadphase pair bookkeeping and islands.
    ///
    /// The intuition that a rewind should start by wiping this state turns out to be
    /// wrong, and the native test suite measures why. PhysX warm-starts the solver
    /// from the previous step's contact impulses, and no public API can read or write
    /// them, so a resimulated tick can never be bit-identical to the original. Given
    /// that, the goal changes from "reproduce exactly" to "stay as close as possible",
    /// and wiping the caches moves the replay further away rather than closer,
    /// because it discards warm-start data the original tick actually had.
    ///
    /// Measured over a 30 tick replay of a contact-heavy stack:
    ///
    ///     eNONE             1.8e-06 m   9.4e-06 m/s
    ///     eRESET_FILTERING  1.4e-02 m   7.0e-02 m/s
    ///     eREINSERT         9.8e-02 m   2.1e-01 m/s
    ///
    /// eNONE is therefore the correct default by four orders of magnitude. The other
    /// modes are kept because they are the right tool for a hard resynchronisation,
    /// where the goal is to discard history rather than preserve it.
    struct PxwContactResetMode
    {
        enum Enum : PxU32
        {
            /// Leave PhysX's carried state alone. The default, and the most accurate
            /// option for rollback.
            eNONE = 0,

            /// Discard contact pairs via PxScene::resetFiltering.
            eRESET_FILTERING = 1,

            /// Remove every actor from the scene and re-add it in stable-ID order,
            /// leaving PhysX with no carried state at all. Sleep state is reset by the
            /// reinsertion, so a restore must follow.
            eREINSERT = 2
        };
    };

    /// Mass properties in a form that can be compared, hashed and shipped between
    /// peers, so that every peer simulates a body with bit-identical mass, inertia and
    /// mass frame rather than each recomputing them and hoping to agree.
    ///
    /// Recomputing is not safe to do independently. PhysX only supports a diagonal
    /// inertia tensor, so PxRigidBodyExt::updateMassAndInertia diagonalises the tensor
    /// and stores the eigenvector rotation as the centre-of-mass orientation. For a
    /// body whose principal moments are close together, which is any body that is
    /// roughly as wide as it is tall and deep, those eigenvectors are ill conditioned:
    /// the native suite measures a spiked ball whose principal moments differ by 0.25%
    /// turning a 1e-6 m change in shape layout into an 8e-5 rad change in the mass
    /// frame, an amplification of about eighty. Two peers that build the same body from
    /// inputs differing in the last bit can therefore end up with visibly different
    /// mass frames, and from there they simulate different bodies.
    ///
    /// So mass properties are treated as authored data: computed once, hashed, and
    /// applied verbatim.
    struct PxwMassProperties
    {
        PxReal mass;                      //!< Total mass.
        PxVec3 inertia;                   //!< Diagonal inertia, expressed in the mass frame.
        PxwPose cMassLocalPose;           //!< Mass frame relative to the actor origin. Quaternion-first, matching the managed SimTransform.

        /// (largest - smallest) / largest principal moment. Near zero means the body is
        /// inertially close to a sphere and its principal axes are ill conditioned. Read
        /// it to decide whether to trust an independently computed mass frame.
        PxReal anisotropy;

        /// Number of shapes that contributed, so a peer with a different shape set is
        /// caught by the hash rather than by a desync twenty seconds later.
        PxU32 shapeCount;

        /// Set when the tensor was within the isotropy tolerance and the mass frame was
        /// collapsed to the identity. In that case a near-origin centre of mass is also
        /// snapped to the actor origin. See PxwComputeMassProperties.
        PxU32 massFrameCollapsed;
    };

    /// Severity levels passed to the log callback. Mirrors PxErrorCode loosely.
    struct PxwLogSeverity
    {
        enum Enum : PxI32
        {
            eDEBUG = 0,
            eINFO = 1,
            eWARNING = 2,
            eERROR = 3
        };
    };
}

extern "C"
{
    /// Callback invoked for every PhysX diagnostic. Replaces polling GetPhysxErrors.
    typedef void(*PxwLogCallbackFn)(PxI32 severity, const char* message);

    /// Installs a log callback. Pass null to revert to buffered polling only.
    PHYSX_WRAPPER_API void PxwSetLogCallback(PxwLogCallbackFn callback);

    // ---------------------------------------------------------------- scene ----

    /// Creates a scene from a fully explicit descriptor.
    PHYSX_WRAPPER_API PxScene* PxwCreateSceneEx(const pxw::PxwSceneDesc* desc);

    /// Begins simulation of a single scene. Does not touch other scenes.
    PHYSX_WRAPPER_API void PxwSceneSimulate(PxScene* scene, PxReal dt);

    /// Completes simulation of a single scene.
    PHYSX_WRAPPER_API void PxwSceneFetchResults(PxScene* scene);

    /// Simulate plus fetch for a single scene.
    PHYSX_WRAPPER_API void PxwSceneStep(PxScene* scene, PxReal dt);

    /// Discards contact pairs for every actor in the scene via PxScene::resetFiltering.
    /// See PxwContactResetMode for why this is a resynchronisation tool rather than
    /// something to do on every rollback.
    PHYSX_WRAPPER_API void PxwSceneResetContactState(PxScene* scene);

    // ------------------------------------------------------- rigid body extras ----

    PHYSX_WRAPPER_API void PxwSetRigidDynamicSolverIterations(PxRigidDynamic* actor, PxU32 positionIters, PxU32 velocityIters);
    PHYSX_WRAPPER_API void PxwSetRigidDynamicSleepThreshold(PxRigidDynamic* actor, PxReal threshold);
    PHYSX_WRAPPER_API PxReal PxwGetRigidDynamicSleepThreshold(PxRigidDynamic* actor);
    PHYSX_WRAPPER_API void PxwSetRigidDynamicWakeCounter(PxRigidDynamic* actor, PxReal wakeCounter);
    PHYSX_WRAPPER_API PxReal PxwGetRigidDynamicWakeCounter(PxRigidDynamic* actor);
    PHYSX_WRAPPER_API bool PxwIsRigidDynamicSleeping(PxRigidDynamic* actor);
    PHYSX_WRAPPER_API void PxwSetRigidBodyMassSpaceInertiaTensor(PxRigidBody* actor, const PxVec3* inertia);
    PHYSX_WRAPPER_API void PxwGetRigidBodyMassSpaceInertiaTensor(PxRigidBody* actor, PxVec3* outInertia);
    PHYSX_WRAPPER_API void PxwSetRigidBodyCMassLocalPose(PxRigidBody* actor, const pxw::PxwTransformData* pose);
    PHYSX_WRAPPER_API void PxwSetActorSimulationEnabled(PxActor* actor, bool enabled);

    /// Applies the standard set of deterministic defaults to a dynamic actor:
    /// explicit solver iterations, disabled speculative CCD variability and a fixed
    /// max depenetration velocity.
    PHYSX_WRAPPER_API void PxwApplyDeterministicRigidDefaults(PxRigidDynamic* actor, PxU32 positionIters, PxU32 velocityIters);

    // -------------------------------------------------------- body gameplay I/O ----
    //
    // The forces, teleport and reads gameplay applies to a single body inside a step
    // handler. Each takes the PxActor* the registry stored and forwards to the matching
    // PxRigidBody / PxRigidActor call. Everything here must run inside OnBeforeStep on the
    // managed side; a force applied outside the step handler happens on the original pass
    // and not on the replay, which desyncs a peer against itself. mode is a PxForceMode.

    PHYSX_WRAPPER_API void PxwBodyAddForce(PxActor* actor, const PxVec3* force, PxU32 mode);
    PHYSX_WRAPPER_API void PxwBodyAddTorque(PxActor* actor, const PxVec3* torque, PxU32 mode);
    PHYSX_WRAPPER_API void PxwBodyGetPose(PxActor* actor, pxw::PxwPose* outPose);

    /// Places a body at a pose and sets its velocities in one call, for bringing a pooled
    /// object into play. Re-pins the wake counter the way a restore does, so a spawned
    /// body is awake and simulated rather than inheriting the pooled slot's sleep state.
    /// A placement, not a physical move: use it only when activating a pooled entity.
    PHYSX_WRAPPER_API void PxwBodyTeleport(PxActor* actor, const pxw::PxwPose* pose,
                                           const PxVec3* velocity, const PxVec3* angularVelocity);

    PHYSX_WRAPPER_API void PxwBodyGetLinearVelocity(PxActor* actor, PxVec3* outVelocity);
    PHYSX_WRAPPER_API void PxwBodySetLinearVelocity(PxActor* actor, const PxVec3* velocity);
    PHYSX_WRAPPER_API void PxwBodyGetAngularVelocity(PxActor* actor, PxVec3* outVelocity);
    PHYSX_WRAPPER_API void PxwBodySetAngularVelocity(PxActor* actor, const PxVec3* velocity);
    PHYSX_WRAPPER_API PxReal PxwBodyGetMass(PxActor* actor);

    // ------------------------------------------------------------- scene queries ----
    //
    // World-level queries that resolve every hit to a stable ID and return them in a
    // deterministic order (raycast and sweep by distance then ID, overlap by ID). Each
    // returns the number of hits written, which may be fewer than were found when capacity
    // truncates; the retained hits are the front of the sorted list. filterMask is ANDed
    // against a per-kind bit (1u << kind); zero matches everything. A hit on an actor this
    // world did not register is dropped rather than reported with a fabricated ID. Run
    // these only from a step handler, against the committed scene outside the simulate
    // window, which is where the managed layer calls them.

    PHYSX_WRAPPER_API PxU32 PxwWorldRaycast(pxw::PxwWorld* world,
        const PxVec3* origin, const PxVec3* direction, PxReal maxDistance,
        PxU32 filterMask, pxw::PxwRaycastHit* hits, PxU32 capacity);

    PHYSX_WRAPPER_API PxU32 PxwWorldOverlap(pxw::PxwWorld* world,
        PxU32 shape, const PxVec3* center, const PxVec3* halfExtents, PxReal radius,
        const PxQuat* rotation, PxU32 filterMask, pxw::PxwOverlapHit* hits, PxU32 capacity);

    PHYSX_WRAPPER_API PxU32 PxwWorldSweep(pxw::PxwWorld* world,
        PxU32 shape, const PxVec3* origin, const PxVec3* halfExtents, PxReal radius,
        const PxQuat* rotation, const PxVec3* direction, PxReal maxDistance,
        PxU32 filterMask, pxw::PxwRaycastHit* hits, PxU32 capacity);

    // --------------------------------------------------------- contact draining ----
    //
    // Contact and trigger events produced by the last step, drained once after it. A
    // UNDPWR world installs a PxSimulationEventCallback and a notification-only filter
    // shader (eENABLE_CONTACT_EVENTS) that ORs the touch, contact-point and trigger flags
    // onto the default behaviour without changing which pairs collide or get solved, so the
    // reported contacts are the ones the simulation already generated. The drains resolve
    // each actor to its stable ID through the registry, drop hits on unregistered actors,
    // normalise and sort the buffer, and return how many events were written -- fewer than
    // were produced when capacity truncates, keeping the front of the sorted list. Call
    // once per step from a step handler; the buffers are cleared at the next simulate.
    // Contacts fire on replayed ticks too, so a replay produces the same sorted set.
    PHYSX_WRAPPER_API PxU32 PxwWorldDrainContacts(pxw::PxwWorld* world, pxw::PxwContactEvent* dst, PxU32 capacity);
    PHYSX_WRAPPER_API PxU32 PxwWorldDrainTriggers(pxw::PxwWorld* world, pxw::PxwTriggerEvent* dst, PxU32 capacity);

    // ------------------------------------------------------------------ mass ----

    /// Default relative spread below which the principal axes are considered
    /// meaningless and the mass frame is collapsed to the identity. 1% of the largest
    /// principal moment: deliberately narrow, so it collapses only bodies whose moments
    /// are almost exactly equal and leaves genuinely elongated bodies untouched. A
    /// near-spherical compound that sits just above this (a spiked ball is near 1.3%)
    /// should pass a wider tolerance explicitly rather than have this default widened.
    #define PXW_DEFAULT_ISOTROPY_TOLERANCE 0.01f

    /// Computes mass properties for an actor without applying them.
    ///
    /// Differs from PxRigidBodyExt::updateMassAndInertia in four ways that matter for
    /// networked determinism:
    ///
    ///  - shapes are accumulated strictly in attachment index order, so the result does
    ///    not depend on how the summation happens to be ordered internally;
    ///  - when the principal moments agree to within isotropyTolerance the mass frame is
    ///    collapsed to the identity instead of storing an arbitrary eigenvector
    ///    rotation, which removes the ill-conditioning described on PxwMassProperties
    ///    and also removes the rotated mass frame that makes actor poses lossy;
    ///  - in that same collapsed case, a centre of mass within 0.1% of the body's radius
    ///    of gyration is snapped to the actor origin, since a near-sphere's summed COM
    ///    is otherwise a last-bit-different quantity that still desyncs peers even after
    ///    the frame is collapsed;
    ///  - otherwise the mass frame quaternion is put in a canonical sign, since a
    ///    diagonalisation is free to return either q or -q.
    ///
    /// Pass 0 for isotropyTolerance to keep the exact principal axes, or a negative
    /// value to use PXW_DEFAULT_ISOTROPY_TOLERANCE.
    ///
    /// \param density Uniform density applied to every shape.
    /// \param includeNonSimShapes Whether shapes without eSIMULATION_SHAPE contribute.
    PHYSX_WRAPPER_API PxI32 PxwComputeMassProperties(PxRigidBody* actor, PxReal density, PxReal isotropyTolerance,
                                                     bool includeNonSimShapes, pxw::PxwMassProperties* out);

    /// Applies mass properties verbatim, with no recomputation. This is what every peer
    /// should call, using one authored or replicated PxwMassProperties value.
    PHYSX_WRAPPER_API PxI32 PxwApplyMassProperties(PxRigidBody* actor, const pxw::PxwMassProperties* props);

    /// Convenience for the single-machine case: compute then apply.
    PHYSX_WRAPPER_API PxI32 PxwSetupDeterministicMass(PxRigidBody* actor, PxReal density, PxReal isotropyTolerance,
                                                      bool includeNonSimShapes, pxw::PxwMassProperties* out);

    /// Hashes the physically meaningful fields, for comparing setup across peers before
    /// the simulation starts rather than diagnosing a desync afterwards.
    PHYSX_WRAPPER_API PxU64 PxwHashMassProperties(const pxw::PxwMassProperties* props);

    /// Reads back what an actor currently has, so a peer can verify it matches the
    /// properties it was told to apply.
    PHYSX_WRAPPER_API PxI32 PxwGetMassProperties(PxRigidBody* actor, pxw::PxwMassProperties* out);

    // ---------------------------------------------------------------- world ----

    /// Creates a world: one scene plus its stable-ID registry.
    PHYSX_WRAPPER_API pxw::PxwWorld* PxwWorldCreate(const pxw::PxwSceneDesc* desc);

    /// Releases the world, its registry and its scene.
    PHYSX_WRAPPER_API void PxwWorldDestroy(pxw::PxwWorld* world);

    PHYSX_WRAPPER_API PxScene* PxwWorldGetScene(pxw::PxwWorld* world);

    /// Registers a handle under a stable ID. The actor is not added to the scene
    /// until PxwWorldCommitPending runs, which adds everything in stable-ID order.
    PHYSX_WRAPPER_API PxI32 PxwWorldRegister(pxw::PxwWorld* world, PxU32 stableId, void* handle, PxU32 kind);

    /// Queues removal of a stable ID. Also applied by PxwWorldCommitPending.
    PHYSX_WRAPPER_API PxI32 PxwWorldUnregister(pxw::PxwWorld* world, PxU32 stableId);

    /// Flushes queued additions and removals to the scene in stable-ID order.
    PHYSX_WRAPPER_API PxI32 PxwWorldCommitPending(pxw::PxwWorld* world);

    PHYSX_WRAPPER_API PxU32 PxwWorldGetEntryCount(pxw::PxwWorld* world);
    PHYSX_WRAPPER_API void* PxwWorldFindHandle(pxw::PxwWorld* world, PxU32 stableId);

    /// Enables or disables simulation for a registered entry without unregistering
    /// it. This is how pooled objects are parked: the actor set never changes, so
    /// the scene composition stays identical on every peer.
    PHYSX_WRAPPER_API PxI32 PxwWorldSetEntryEnabled(pxw::PxwWorld* world, PxU32 stableId, bool enabled);

    /// Configures framework-driven sleeping, which replaces PhysX's own.
    ///
    /// PhysX's sleep timing does not survive rollback. Its wake counter resets to a
    /// value that includes the body's counted contact interactions, a count kept from
    /// touch-found and touch-lost edges against the previous step's touch state. A
    /// restore rewrites body state but not that touch state, so the reset that fires
    /// on the first replayed step is not a function of the snapshot. Measured: one
    /// tick replayed from 24 rewind depths reproduces pose and velocity bitwise every
    /// time and the wake counter only 22 times, the two misses landing one dt apart.
    ///
    /// So the wake counter is instead pinned high while a body is awake, which keeps
    /// PhysX's sleep bookkeeping from ever running, and the framework decides when a
    /// body sleeps: once its linear and angular speeds have stayed below the given
    /// thresholds for sleepTicks consecutive steps, it is put to sleep. That decision
    /// is a pure function of restored velocities and a per-body rest counter, both of
    /// which are in the snapshot, so it replays.
    ///
    /// sleepTicks of 0 disables framework sleeping: bodies are kept awake and pinned
    /// indefinitely, which is the safe default for a networked world where the CPU
    /// saved by sleeping is not worth any risk to determinism.
    ///
    /// Thresholds are compared against speed, in metres per second and radians per
    /// second. Intended to be set once at world creation; every peer must agree on
    /// all three values.
    PHYSX_WRAPPER_API PxI32 PxwWorldSetSleepParams(pxw::PxwWorld* world,
        PxReal linearThreshold, PxReal angularThreshold, PxU32 sleepTicks);

    PHYSX_WRAPPER_API void PxwWorldSimulate(pxw::PxwWorld* world, PxReal dt);
    PHYSX_WRAPPER_API void PxwWorldFetchResults(pxw::PxwWorld* world);
    PHYSX_WRAPPER_API void PxwWorldStep(pxw::PxwWorld* world, PxReal dt);
    /// Equivalent to PxwWorldResetContactStateEx with eRESET_FILTERING.
    PHYSX_WRAPPER_API void PxwWorldResetContactState(pxw::PxwWorld* world);

    /// Erases PhysX's carried simulation state using the given PxwContactResetMode.
    PHYSX_WRAPPER_API void PxwWorldResetContactStateEx(pxw::PxwWorld* world, PxU32 mode);

    // ---------------------------------------------------------------- state ----

    /// Upper bound in bytes for the current registry contents.
    PHYSX_WRAPPER_API PxU32 PxwWorldStateSize(pxw::PxwWorld* world);

    /// Captures the whole world into a self-describing blob and optionally returns
    /// its hash. Returns bytes written, or 0 on failure.
    PHYSX_WRAPPER_API PxU32 PxwWorldCaptureState(pxw::PxwWorld* world, void* dst, PxU32 capacity, PxU64* outHash);

    /// Restores a blob produced by PxwWorldCaptureState.
    PHYSX_WRAPPER_API PxI32 PxwWorldRestoreState(pxw::PxwWorld* world, const void* src, PxU32 size);

    /// Hash of the current live state, without materialising a blob.
    PHYSX_WRAPPER_API PxU64 PxwWorldHashState(pxw::PxwWorld* world);

    /// Per-entry hashes, for locating the first diverging body during a desync.
    PHYSX_WRAPPER_API PxU32 PxwWorldHashPerEntry(pxw::PxwWorld* world, pxw::PxwEntryHash* dst, PxU32 capacity);

    /// Bulk pose readback for presentation, in stable-ID order.
    PHYSX_WRAPPER_API PxU32 PxwWorldReadPoses(pxw::PxwWorld* world, pxw::PxwPoseEntry* dst, PxU32 capacity);

    /// Reads the PhysX-assigned identity of every registered body, in stable-ID order.
    /// Returns the number of records written.
    ///
    /// Indices are only assigned once an actor is in a scene, so this is meaningful
    /// after the first commit and simulation step.
    PHYSX_WRAPPER_API PxU32 PxwWorldReadInternalIds(pxw::PxwWorld* world, pxw::PxwInternalIdEntry* dst, PxU32 capacity);

    /// Hash of the stable-ID to PhysX-index mapping.
    ///
    /// Peers exchange this once after the world is built. Equal hashes mean every peer
    /// gave the same body the same place in PhysX's internal ordering, which is the
    /// precondition for their simulations agreeing at all. Unequal hashes mean the
    /// registration order differs and no amount of state synchronisation will help.
    PHYSX_WRAPPER_API PxU64 PxwWorldHashInternalIds(pxw::PxwWorld* world);

    /// Hash of how every registered body was BUILT, as opposed to what state it is in.
    ///
    /// Covers shape count and attachment order, each shape's geometry, local pose,
    /// contact and rest offsets, flags, filter data and material coefficients, and each
    /// body's mass properties, damping, velocity and depenetration clamps, solver
    /// iteration counts and thresholds. None of this appears in a snapshot, in
    /// PxwWorldHashState or in PxwWorldHashPerEntry, because none of it changes as the
    /// simulation runs -- and every one of them is read by every solve.
    ///
    /// Peers exchange this once after the world is built and again after a rebuild. Equal
    /// hashes mean the bodies really are the same bodies; unequal hashes mean two peers
    /// constructed the same entity differently and will diverge as soon as one of them is
    /// loaded hard enough for the difference to show, which for a lightly touched body can
    /// be hundreds of ticks later and look like anything but a construction bug.
    ///
    /// This matters most for compounds built from offset shapes. Twenty-five shapes are
    /// twenty-five geometries, local poses and material bindings that must match exactly,
    /// and a near-isotropic compound's mass is canonicalised precisely so that it does
    /// NOT reflect small shape differences -- so the mass hash cannot be relied on to
    /// catch them. A single ULP in one shape's local pose is enough to desync the body
    /// once it is squeezed between two others, while leaving it in perfect agreement for
    /// as long as it is only rolling on the floor.
    ///
    /// Addresses are never hashed: meshes are identified by vertex and element counts, so
    /// the value is comparable between processes and machines.
    PHYSX_WRAPPER_API PxU64 PxwWorldHashConstruction(pxw::PxwWorld* world);

    /// Per-entry construction hashes, for naming which body was built differently.
    /// Same contract as PxwWorldHashConstruction, one record per registered entry.
    PHYSX_WRAPPER_API PxU32 PxwWorldHashConstructionPerEntry(pxw::PxwWorld* world, pxw::PxwEntryHash* dst, PxU32 capacity);

    /// Link poses for one registered articulation, in PhysX link-index order.
    PHYSX_WRAPPER_API PxU32 PxwWorldReadArticulationLinkPoses(pxw::PxwWorld* world, PxU32 stableId, pxw::PxwTransformData* dst, PxU32 capacity);

    /// Hash of a blob previously produced by PxwWorldCaptureState.
    PHYSX_WRAPPER_API PxU64 PxwHashBuffer(const void* src, PxU32 size);
}
