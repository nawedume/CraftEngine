#include "Drawing.h"
#include "GLFW/glfw3.h"
#include "World.h"
#include "Math.hpp"

#include "camera.hpp"
#include "utils.hpp"
#include "worldrender.hpp"
#include <OpenGL/gl.h>


/**
*  Max impulses after 2000 frames (10 normal, 0 relaxed): (0.468094, 0.173227, 0.155699).
*  Max impulses after 2000 frames (5 normal, 5 relaxed): (0.504107, 0.180658, 0.145362)
*/
int main() {
    draw::GSystem *gsys = draw::InitGSystem(1000, 1000, -20.0f, 10.0f, 20.0f);
    gsys->mCamera.Yaw = -45.0f;
    gsys->mCamera.Pitch = -15.0f;
    gsys->mLightDir = ce::Vec3(1.0f, 1.0f, 1.0f);
    std::vector<draw::GObject> gobjects;
    ce::World *world = ce::NewWorld();
    world->Settings.NumOfSolverIterations = 10;
    world->Settings.NumOfRelaxationIterations = 0;

    // Debug
    draw::GObject impulseArrow = draw::CreateArrow(.1, .05, 1., .9);

    ce::ConvexHullDef floorDef = ce::ConvexHullDef{};
    ce::BodyId floor =
        CreateBox(gobjects, world, ce::Transform{}, &floorDef,
                  ce::Vec3(10.0f, 1.0f, 10.0f), ce::Vec3(0.4, 0.4, 0.4), true);

    ce::ConvexHullDef boxDef = ce::ConvexHullDef{.Mass = .1f};


    const int NUM_BOXES = 10;
    for (int i = 0; i < NUM_BOXES; ++i) {
        CreateBox(gobjects, world,
                  ce::Transform{.Pos = {0.0f, 2.0f + (i * 2.1), 0.0f}}, &boxDef,
                  ce::Vec3(2.0, 1.0, 2.0), ce::Vec3(0.5, 0.1, 0.1), false);
    }

    draw::GObject debugBall = draw::CreateBall(0.1f, 6, 6);
    debugBall.BaseColor = ce::Vec3(1.0, 1.0, 1.0);
    unsigned int frame = 0;

    float stepSize = 1.0f / 60.0f;
    while (!glfwWindowShouldClose(gsys->mWindow)) {
        static bool pause = true;
        printf("Iteration %d\n", frame);

        if (!pause) {
            ce::Step(world, stepSize);
            frame += 1;

        }

        if (frame == 2000) {
            auto cs = world->ContactSet;
            float maxImpulseN = 0.0f;
            float maxImpulseT1 = 0.0f;
            float maxImpulseT2 = 0.0f;
            for (int i = 0; i < cs->Size(); ++i) {
                auto m = cs->Manifolds[i];
                for (int j = 0; j < m.NumPoints; ++j) {
                    auto cp = m.Points[j];
                    maxImpulseN = ce::max(maxImpulseN, abs(cp.NImpulse));
                    maxImpulseT1 = ce::max(maxImpulseT1, abs(cp.TImpulse[0]));
                    maxImpulseT2 = ce::max(maxImpulseT2, abs(cp.TImpulse[1]));
                }
            }

            printf("Max: %f, %f, %f\n", maxImpulseN, maxImpulseT1, maxImpulseT2);
        }

        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        draw::DrawPrep(gsys);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        DrawWorld(gobjects.data(), world, gsys);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        {
            auto cs = world->ContactSet;
            for (int idx = 0; idx < cs->Size(); ++idx) {
                ce::ManifoldId mid = cs->ManifoldIds[idx];
                ce::Manifold manifold = cs->Manifolds[idx];
                for (int pointIdx = 0; pointIdx < manifold.NumPoints; ++pointIdx) {
                    ce::ContactPoint p = manifold.Points[pointIdx];
                    auto p1 = p.RelContactPoint1 + world->Transforms[mid.Body1].Pos;
                    draw::DrawObject(&impulseArrow, &gsys->mShader, p1.x, p1.y, p1.z, 0, 0, 1, 0, 1, p.NImpulse * 10, 1);
                }
            }
        }
        printf("Size of store %lu\n", world->StoredImpulses.Store.size());

        glfwSwapBuffers(gsys->mWindow);
        glfwPollEvents();

        draw::ControlCamera(gsys->mWindow, &gsys->mCamera, 1.0f / 60.0f);
        if (glfwGetKey(gsys->mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            glfwSetWindowShouldClose(gsys->mWindow, 1);
        }

        if (glfwGetKey(gsys->mWindow, GLFW_KEY_E) == GLFW_PRESS) {
            pause = false;
        }
    }

    return 0;
}
