#include "Core.h"
#include "Debug.h"
#include "Drawing.h"
#include "GLFW/glfw3.h"
#include "World.h"
#include "Math.hpp"

#include "camera.hpp"
#include "glm/gtc/constants.hpp"
#include "utils.hpp"
#include "worldrender.hpp"
#include <OpenGL/gl.h>
#include <cmath>
#include <vector>

/**
*  Max impulses after 2000 frames (10 normal, 0 relaxed): (0.468094, 0.173227, 0.155699).
*  Max impulses after 2000 frames (5 normal, 5 relaxed): (0.504107, 0.180658, 0.145362)
*/
int main() {
    draw::GSystem *gsys = draw::InitGSystem(1000, 1000, -20.0f, 10.0f, 20.0f);
    gsys->mCamera.Yaw = -45.0f;
    ; //= 45.0f;
    gsys->mCamera.Pitch = -15.0f;
    ; //= 45.0f;
    gsys->mLightDir = ce::Vec3(1.0f, 1.0f, 1.0f);
    std::vector<draw::GObject> gobjects;
    ce::World *world = ce::NewWorld();
    world->Settings.NumOfSolverIterations = 100;
    world->Settings.NumOfRelaxationIterations = 0;

    ce::ConvexHullDef floorDef = ce::ConvexHullDef{};
    ce::BodyId floor =
        CreateBox(gobjects, world, ce::Transform{}, &floorDef,
                  ce::Vec3(10.0f, 1.0f, 10.0f), ce::Vec3(0.4, 0.4, 0.4), true);
    printf("The floor is %d\n", floor);

    ce::Quat ori = ce::aaToQuat(ce::Vec3(1.0, 0.0, 0.0), glm::quarter_pi<float>());
    ce::Vec3 p = ce::Vec3(0.0f, 10.f, -18.0f);
    ce::BodyId ramp =
        CreateBox(gobjects, world, ce::Transform{ .Pos = p, .Orientation = ori }, &floorDef,
                    ce::Vec3(10.0f, 1.0f, 10.0f), ce::Vec3(0.4, 0.4, 0.4), true);


    ce::SphereDef ballDef = ce::SphereDef { .Mass = 1000.f, .Radius = 1.0f };
    // ce::BodyId ball = CreateBall(
    //     gobjects, world, ce::Transform { .Pos = { 0.0f, 30.0f, -20.0f } }, &ballDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    // );

    ce::CapsuleDef capsuleDef = ce::CapsuleDef{.Mass = 100.0f, .Radius = 1.0f, .HalfLength = 1.0f};
    ce::BodyId capsule = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { 0.0f, 20.0f, -20.0f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    world->Materials[capsule].Friction = 0.2;

    ce::BodyId pin0 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { 00.0f, 3.0f, 0.0f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    ce::BodyId pin1 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { 1.0f, 3.0f, 2.1f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    ce::BodyId pin2 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { -1.0f, 3.0f, 2.1f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    ce::BodyId pin3 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { -2.0f, 3.0f, 4.2f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    ce::BodyId pin4 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { 0.0f, 3.0f, 4.2f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    ce::BodyId pin5 = CreateCapsule(
        gobjects, world, ce::Transform { .Pos = { 2.0f, 3.0f, 4.2f } }, &capsuleDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    );

    // printf("The Pin is %d\n", pin);
    // ce::BodyId pin = CreateBall(
    //     gobjects, world, ce::Transform { .Pos = { 00.0f, 0.0f, 0.0f } }, &ballDef, 10, ce::Vec3(0.7f, 0.0f, 0.0f)
    // );

    draw::GObject debugBall = draw::CreateBall(0.1f, 6, 6);
    debugBall.BaseColor = ce::Vec3(1.0, 1.0, 1.0);

    draw::GObject debugArrow = draw::CreateArrow(0.1f, 0.05f, 1.f, 0.9f);
    debugArrow.BaseColor = ce::Vec3(1.0, 1.0, 1.0);

    std::vector<ce::Vec3> ps;
    ce::SetDrawDebugBallFn([&ps](ce::Vec3 p) {
        ps.push_back(p);
        return;
    });

    unsigned int frame = 0;

    float stepSize = 1.0f / 60.0f;
    while (!glfwWindowShouldClose(gsys->mWindow)) {
        static bool pause = true;

        if (!pause) {
            printf("Frame %d\n", frame);
            ce::Step(world, stepSize);
            frame += 1;
        }

        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (world->ContactSet->Size() > 0) {
            for (int j = 0; j < world->ContactSet->Size(); ++j) {
                auto mid = world->ContactSet->ManifoldIds[j];
                auto manifold = world->ContactSet->Manifolds[j];
                for (int i = 0; i < manifold.NumPoints; ++i) {
                    auto cp = manifold.Points[i];
                    ce::Vec3 pos1 = world->Transforms[mid.Body1].Pos + cp.RelContactPoint1;
                    ce::Vec3 pos2 = world->Transforms[mid.Body2].Pos + cp.RelContactPoint2;
                    debugBall.BaseColor.x = 0.5 * i;
                    draw::DrawBall(&debugBall, &gsys->mShader, pos1.x, pos1.y, pos1.z, 1.0, 0.0, 0.0, 0.0);
                }

                if (mid.Body1 == 2 && mid.Body2 == 3) {
                    // pause = true;
                    // ce::PrintVec3("Normal: ", manifold.Normal);
                    // draw::DrawBall(&debugBall, &gsys->mShader, 0.000000, 1.990436, -1.251796, 1.0, 0.0, 0.0, 0.0);
                    // auto pp = ce::Vec3(0.000000, 1.990436, -1.877131) + manifold.Normal;
                    // draw::DrawBall(&debugBall, &gsys->mShader, pp.x, pp.y, pp.z, 1.0, 0.0, 0.0, 0.0);
                    // draw::DrawBall(&debugBall, &gsys->mShader, 0.000000, 2.032426, 0.655135, 1.0, 0.0, 0.0, 0.0);
                    for (auto p : ps) {
                        draw::DrawBall(&debugBall, &gsys->mShader, p.x, p.y, p.z, 1.0, 0.0, 0.0, 0.0);
                    }
                }
            }
        }

        draw::DrawPrep(gsys);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        DrawWorld(gobjects.data(), world, gsys);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glfwSwapBuffers(gsys->mWindow);
        glfwPollEvents();

        draw::ControlCamera(gsys->mWindow, &gsys->mCamera, 1.0f / 60.0f);
        if (glfwGetKey(gsys->mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            glfwSetWindowShouldClose(gsys->mWindow, 1);
        }

        if (gsys->IsKeyClicked(GLFW_KEY_E)) {
            pause = false;
            ps.clear();
        }

        if (gsys->IsKeyClicked(GLFW_KEY_C)) {
            pause = false;
        }
    }

    return 0;
}
