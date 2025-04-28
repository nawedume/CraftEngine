#include "Drawing.h"
#include "GLFW/glfw3.h"
#include "World.h"

#include "utils.hpp"
#include "worldrender.hpp"
#include <OpenGL/gl.h>


int main() {
    draw::GSystem *gsys = draw::InitGSystem(1000, 1000, -40.0f, 20.0f, 40.0f);
    gsys->mCamera.Yaw = -45.0f;
    ; //= 45.0f;
    gsys->mCamera.Pitch = -15.0f;
    ; //= 45.0f;
    gsys->mLightDir = ce::Vec3(1.0f, 1.0f, 1.0f);
    std::vector<draw::GObject> gobjects;
    ce::World *world = ce::NewWorld();
    world->Settings.NumOfSolverIterations = 10;
    world->Settings.NumOfRelaxationIterations = 0;

    world->GravityAcc.y = -10.0;

    ce::ConvexHullDef boxDef = ce::ConvexHullDef{.Mass = .1f};

    ce::BodyId topBox = CreateBox(gobjects, world,
              ce::Transform{.Pos = {0.0f, (11 * 6.0f), 0.0f}}, &boxDef,
              ce::Vec3(2.0, 2.0, 2.0), ce::Vec3(0.5, 0.1, 0.1), true);

    ce::BodyId prevBox = topBox;
    const int NUM_BOXES = 10;
    for (int i = 0; i < NUM_BOXES; ++i) {
        ce::Vec3 pos = {0.0f, ((10 - i) * 6.0f), 0.0f};
         auto currentBox = CreateBox(gobjects, world,
                   ce::Transform{.Pos = pos }, &boxDef,
                   ce::Vec3(1.0, 1.0, 1.0), ce::Vec3(0.5, 0.1, 0.1), false);

        ce::AddRevoluteJoint(world, prevBox, currentBox, pos + ce::Vec3(0.0f, 3.0f, 0.0f));
       prevBox = currentBox;
    }

    float stepSize = 1.0f / 60.0f;
    while (!glfwWindowShouldClose(gsys->mWindow)) {
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ce::Step(world, stepSize);

        draw::DrawPrep(gsys);

        DrawWorld(gobjects.data(), world, gsys);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glfwSwapBuffers(gsys->mWindow);
        glfwPollEvents();

        draw::ControlCamera(gsys->mWindow, &gsys->mCamera, 1.0f / 60.0f);
        if (glfwGetKey(gsys->mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            glfwSetWindowShouldClose(gsys->mWindow, 1);
        }

        if (glfwGetKey(gsys->mWindow, GLFW_KEY_E) == GLFW_PRESS) {
            ce::ApplyLinearForce(world, prevBox, ce::Vec3(10.0f, 0.0, 0.0));
        }
    }

    return 0;
}
