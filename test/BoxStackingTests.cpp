#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>

#include "Core.h"
#include "World.h"
#include "utils.h"

ce::Vec3 getMaxImpulses(ce::World* world) {
    auto cs = world->ContactSet;
    float maxImpulseN = 0.0f;
    float maxImpulseT1 = 0.0f;
    float maxImpulseT2 = 0.0f;
    for (int i = 0; i < cs->Size(); ++i) {
        auto m = cs->Manifolds[i];
        for (int j = 0; j < m.NumPoints; ++j) {
            auto cp = m.Points[j];
            maxImpulseN = glm::max(maxImpulseN, abs(cp.NImpulse));
            maxImpulseT1 = glm::max(maxImpulseT1, abs(cp.TImpulse[0]));
            maxImpulseT2 = glm::max(maxImpulseT2, abs(cp.TImpulse[1]));
        }
    }

    return ce::Vec3(maxImpulseN, maxImpulseT1, maxImpulseT2);
}

ce::World* initWorld(int numBodies) {
    ce::World* world = ce::NewWorld();

    ce::ConvexHullDef floorDef = ce::ConvexHullDef{};
    ce::BodyId floor =
        CreateBox(world, ce::Transform{}, &floorDef,
                  ce::Vec3(10.0f, 1.0f, 10.0f), ce::Vec3(0.4, 0.4, 0.4), true);

    ce::ConvexHullDef boxDef = ce::ConvexHullDef{.Mass = 1.0f};

    for (int i = 0; i < numBodies; ++i) {
        CreateBox(world,
                  ce::Transform{.Pos = {0.0f, 2.0f + (i * 2.1), 0.0f}}, &boxDef,
                  ce::Vec3(2.0, 1.0, 2.0), ce::Vec3(0.5, 0.1, 0.1), false);
    }

    return world;
}

void assertEquals(ce::Vec3 a, ce::Vec3 b) {
    REQUIRE(glm::length(a - b) < 1e-6);
}

TEST_CASE("Boxstacking, 10 Boxes", "[stacking]") {

    ce::World* world = initWorld(10);

    SECTION("World 10 box stack, 10 iterations, 0 relaxing") {
        world->Settings.NumOfSolverIterations = 10;
        world->Settings.NumOfRelaxationIterations = 0;

        for (int i = 0; i < 2000; ++i) {
            ce::Step(world, 1 / 60.0f);
        }
        ce::Vec3 max = getMaxImpulses(world);
        assertEquals(max, ce::Vec3(0.468094, 0.173227, 0.155699));
    }

    SECTION("World 10 box stack, 5 iterations, 5 relaxing") {
        world->Settings.NumOfSolverIterations = 5;
        world->Settings.NumOfRelaxationIterations = 5;

        for (int i = 0; i < 2000; ++i) {
            ce::Step(world, 1 / 60.0f);
        }
        ce::Vec3 max = getMaxImpulses(world);
        assertEquals(max, ce::Vec3(0.504107, 0.180658, 0.145362));
    }
}
