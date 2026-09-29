#include "Core.h"
#include "Debug.h"
#include "Drawing.h"
#include "GLFW/glfw3.h"
#include "Math.hpp"
#include "World.h"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/quaternion.hpp"
#include "utils.hpp"
#include "worldrender.hpp"
#include <cstdio>
#include <vector>

void ControlBody(draw::GSystem *gsys, ce::BodyId bid1, ce::World *world, float deltaTime) {
    ce::Vec3 movement{0};
    GLFWwindow *window = gsys->mWindow;
    if (glfwGetKey(window, GLFW_KEY_UP))
        movement.z += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_DOWN))
        movement.z -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_RIGHT))
        movement.x += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_LEFT))
        movement.x -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_PERIOD))
        movement.y += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_COMMA))
        movement.y -= 1.0f;

    static bool isRotate = false;
    static double isRotateRef = 0.0f;
    double time = glfwGetTime();
    if (glfwGetKey(window, GLFW_KEY_G) && time > (isRotateRef + 1.0f)) {
        isRotate = !isRotate;
        isRotateRef = glfwGetTime();
    }

    if (isRotate) {
        ce::Quat q = world->Transforms[bid1].Orientation;
        ce::Quat m;
        m.w = 0.0f;
        m.x = movement.x;
        m.y = movement.y;
        m.z = movement.z;
        q += 0.5f * deltaTime * m * q;
        q = normalize(q);
        world->Transforms[bid1].Orientation = q;
    } else {
        world->Transforms[bid1].Pos += deltaTime * movement;
    }
}
int main() {
    draw::GSystem *gsys = draw::InitGSystem(1000, 1000, 0.0f, 0.0f, 10.0f);
    gsys->mCamera.Position.y = 0.0f;
    gsys->mLightDir = ce::Vec3(1.0, 1.0, 1.0);

    ce::World *world = ce::NewWorld();
    world->GravityAcc = ce::Vec3(0.0f);

    std::vector<draw::GObject> gobjects;

    // Equal balls colliding
    ce::ConvexHullDef def1 = ce::ConvexHullDef{.Mass = 1.0f};
    ce::BodyId boxId =
        CreateBox(gobjects, world, ce::Transform{}, &def1, ce::Vec3(3.0, 1.0, 3.0), ce::Vec3(1.0, 0.0, 0.0), true);
    //world->Transforms[boxId].Orientation = ce::aaToQuat(ce::Vec3(1.0, 0.0, 0.0), 3.14 / 8);

     ce::SphereDef sphereDef = ce::SphereDef { .Mass = 1.0f, .Radius = 1.0f};
    ce::CapsuleDef capsuleDef = ce::CapsuleDef{.Mass = 1.0f, .Radius = 1.0f, .HalfLength = 1.0f};
    //ce::BodyId sphereId = CreateBall(gobjects, world, ce::Transform { .Pos = { 1, 2.5, 0 } }, &sphereDef, 10, ce::Vec3(0.3, 0.5, 0.3));
    ce::BodyId cId =
        CreateCapsule(gobjects, world, ce::Transform{.Pos = {0.0, 1., 0.0}}, &capsuleDef, 30, ce::Vec3(0.3, 0.5, 0.3));
    world->Transforms[cId].Orientation = ce::aaToQuat(ce::Vec3(0.0, 0.0, 1.0), glm::pi<float>() / 2.0);

    while (!glfwWindowShouldClose(gsys->mWindow)) {
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const float deltaTime = 1.0f / 60.0f;

        draw::DrawPrep(gsys);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        //  DrawWorldRelative(gobjects.data(), world, gsys, &o);
        DrawWorld(gobjects.data(), world, gsys);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        draw::ControlCamera(gsys->mWindow, &gsys->mCamera, 1.0f / 60.0f);
        if (glfwGetKey(gsys->mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            printf("Exited game\n");
            glfwSetWindowShouldClose(gsys->mWindow, 1);
        }

        static int counter = 0;
        static double time = 0;
        long currentTime = glfwGetTime();

        if (glfwGetKey(gsys->mWindow, GLFW_KEY_B)) {
            printf("here\n");
            ce::ApplyLinearForce(world, cId, ce::Vec3(0.0f, -3.0f, 0.0));
            // ce::ApplyLinearForce(world, c2Id, ce::Vec3(0.0f, -2.0, 0.0));
        }

        if (true || (glfwGetKey(gsys->mWindow, GLFW_KEY_C))) {
            counter += 1;
            printf("Step %d\n", counter);
            ce::Step(world, deltaTime);
                      ControlBody(gsys, cId, world, deltaTime);

            time = currentTime;
        }

        glfwSwapBuffers(gsys->mWindow);
        glfwPollEvents();

    }
    return 0;
}
