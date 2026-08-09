// Tests for per-link articulation contact reporting.
//
// The Unity side of this feature needs a play session to exercise, so the accumulation
// semantics are pinned here instead: flags OR across every step in a window, Clear opens a new
// window, reporting is per link rather than per articulation, a scene without contact events
// reports nothing, and releasing an articulation drops its flags so a later allocation at the
// same address cannot inherit them.
//
// Built from the plugin sources, so the test and the code under test share one PhysX instance.
// The scene comes from the wrapper (that is what installs the tracker) but the bodies and the
// stepping are raw PhysX, to keep the test independent of the wrapper's step gating.

#include "PxwAPIs.h"
#include "DataInterop.h"
#include "PhysXWrapper.h"

#include <cstdio>
#include <vector>

using namespace physx;
using namespace pxw;

namespace {

int gFailures = 0;

void check(bool condition, const char* what)
{
    std::printf("%s  %s\n", condition ? "[ ok ]" : "[FAIL]", what);
    if (!condition)
    {
        ++gFailures;
    }
}

// A wide static box as ground, plus a two-link articulation resting height above it. Z-up,
// matching how this plugin is used. Self-collision is off so the only contacts possible are
// link-to-ground, which is what makes the per-link assertions meaningful.
struct Fixture
{
    PxScene* scene = NULL;
    PxArticulationReducedCoordinate* articulation = NULL;
    PxArticulationLink* lower = NULL;
    PxArticulationLink* upper = NULL;
    PxMaterial* material = NULL;

    void build(bool contactEvents)
    {
        PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();

        PxVec3 gravity(0.0f, 0.0f, -9.81f);
        scene = CreateSceneWithFlags(&gravity, PxPruningStructureType::eDYNAMIC_AABB_TREE,
                                     PxSolverType::eTGS, false,
                                     contactEvents ? (PxU32)PxwSceneFlag::eENABLE_CONTACT_EVENTS : 0u);

        material = physics->createMaterial(0.5f, 0.5f, 0.0f);

        PxShape* groundShape = physics->createShape(PxBoxGeometry(20.0f, 20.0f, 0.5f), *material, true);
        PxRigidStatic* ground = physics->createRigidStatic(PxTransform(PxVec3(0.0f, 0.0f, -0.5f)));
        ground->attachShape(*groundShape);
        groundShape->release();
        scene->addActor(*ground);

        articulation = physics->createArticulationReducedCoordinate();
        articulation->setArticulationFlag(PxArticulationFlag::eDISABLE_SELF_COLLISION, true);
        // PhysX stops generating touch events for a sleeping island, so resting contact would
        // vanish once this settles. A policy-driven articulation is woken every step by its
        // joint targets and never gets there; a threshold of zero reproduces that here. The
        // sleeping behaviour itself is asserted separately below.
        articulation->setSleepThreshold(0.0f);

        // Dropped from 1 m so the lower link lands and the upper one, fixed 1 m above it,
        // stays clear of the ground for the whole test.
        lower = articulation->createLink(NULL, PxTransform(PxVec3(0.0f, 0.0f, 1.0f)));
        attachBox(lower);

        upper = articulation->createLink(lower, PxTransform(PxVec3(0.0f, 0.0f, 2.0f)));
        attachBox(upper);

        PxArticulationJointReducedCoordinate* joint = upper->getInboundJoint();
        joint->setJointType(PxArticulationJointType::eFIX);
        joint->setParentPose(PxTransform(PxVec3(0.0f, 0.0f, 0.5f)));
        joint->setChildPose(PxTransform(PxVec3(0.0f, 0.0f, -0.5f)));

        scene->addArticulation(*articulation);
    }

    void attachBox(PxArticulationLink* link)
    {
        PxPhysics* physics = GetGlobalPhysXWrapper().GetPhysics();
        PxShape* shape = physics->createShape(PxBoxGeometry(0.2f, 0.2f, 0.2f), *material, true);
        link->attachShape(*shape);
        shape->release();
        PxRigidBodyExt::updateMassAndInertia(*link, 100.0f);
    }

    void step(int count)
    {
        for (int i = 0; i < count; ++i)
        {
            scene->simulate(1.0f / 120.0f);
            scene->fetchResults(true);
        }
    }

    // Per-link flags for the window that is currently open.
    std::vector<PxU8> readFlags(PxU32* written = NULL)
    {
        std::vector<PxU8> flags(8, 0);
        PxU32 n = GetArticulationContactFlags(articulation, flags.data(), (PxU32)flags.size());
        if (written != NULL)
        {
            *written = n;
        }
        return flags;
    }

    bool touched(PxArticulationLink* link)
    {
        std::vector<PxU8> flags = readFlags();
        return flags[link->getLinkIndex()] != 0;
    }

    int touchCount()
    {
        PxU32 written = 0;
        std::vector<PxU8> flags = readFlags(&written);
        int n = 0;
        for (PxU32 i = 0; i < written; ++i)
        {
            n += flags[i] != 0 ? 1 : 0;
        }
        return n;
    }

    void destroy()
    {
        if (articulation != NULL)
        {
            scene->removeArticulation(*articulation);
            ReleaseArticulation(articulation);
        }
        if (scene != NULL)
        {
            ReleaseScene(scene);
        }
        if (material != NULL)
        {
            material->release();
        }
    }
};

}  // namespace

int main()
{
    if (!InitializePhysX())
    {
        std::printf("[FAIL]  InitializePhysX\n");
        return 1;
    }

    // ---- a scene with contact events reports the link that lands -------------
    {
        Fixture f;
        f.build(/*contactEvents=*/true);

        ClearArticulationContactFlags();
        check(f.touchCount() == 0, "nothing is reported before the first step");

        // Long enough to fall 0.8 m and settle.
        f.step(120);
        check(f.touched(f.lower), "the link that lands reports contact");
        check(!f.touched(f.upper), "the link that never touches anything does not");

        // The window is what makes this usable from a decimated control loop: reading after a
        // Clear with no step in between must find nothing, or the flags are just sticky.
        ClearArticulationContactFlags();
        check(f.touchCount() == 0, "Clear discards the previous window");

        check(!f.articulation->isSleeping(), "the articulation is still awake after settling");
        f.step(1);
        check(f.touched(f.lower), "resting contact reappears within one step");

        // Accumulation across a window can only ever add links, never drop them.
        ClearArticulationContactFlags();
        f.step(1);
        int oneStep = f.touchCount();
        ClearArticulationContactFlags();
        f.step(8);
        check(f.touchCount() >= oneStep, "a longer window reports at least as much contact");

        PxU32 written = 0;
        f.readFlags(&written);
        check(written == GetArticulationLinkCount(f.articulation),
              "the readback covers every link, not just the ones that touched");

        f.destroy();

        // Post-release: the tracker must no longer know this pointer.
        std::vector<PxU8> flags(8, 0xFF);
        PxU32 n = GetArticulationContactFlags(f.articulation, flags.data(), (PxU32)flags.size());
        check(n == 0, "a released articulation reports no links");
        check(flags[0] == 0, "the readback zeroes a buffer it cannot fill");
    }

    // ---- a scene without contact events reports nothing ---------------------
    {
        Fixture f;
        f.build(/*contactEvents=*/false);
        ClearArticulationContactFlags();
        f.step(120);
        check(f.touchCount() == 0, "no contact is reported without eENABLE_CONTACT_EVENTS");
        f.destroy();
    }

    // ---- a sleeping articulation stops reporting ----------------------------
    //
    // PhysX does not run narrowphase for a sleeping island, so resting contact simply stops
    // being reported: the observation says "not touching" while the body sits on the ground.
    // Nothing here can fix that, so it is pinned as known behaviour. A policy-driven
    // articulation is woken every step by its joint targets and never reaches this state.
    {
        Fixture f;
        f.build(/*contactEvents=*/true);
        f.step(120);

        f.articulation->putToSleep();
        ClearArticulationContactFlags();
        f.step(4);
        check(f.articulation->isSleeping(), "the articulation stayed asleep");
        check(f.touchCount() == 0, "a sleeping articulation reports no contact while resting");

        f.articulation->wakeUp();
        ClearArticulationContactFlags();
        f.step(1);
        check(f.touched(f.lower), "waking it up restores the resting contact report");

        f.destroy();
    }

    ReleasePhysX();

    std::printf("\n%s (%d failure%s)\n", gFailures == 0 ? "PASSED" : "FAILED", gFailures,
                gFailures == 1 ? "" : "s");
    return gFailures == 0 ? 0 : 1;
}
