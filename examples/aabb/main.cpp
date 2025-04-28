#include "Core.h"
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
    if (gsys->IsKeyClicked(GLFW_KEY_R)) {
        isRotate = !isRotate;
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
    gsys->mLightDir = ce::Vec3(1.0, 1.0, 1.0);

    ce::World *world = ce::NewWorld();
    world->GravityAcc = ce::Vec3(0.0);

    std::vector<draw::GObject> gobjects;

    // Equal balls colliding
    ce::ConvexHullDef def1 = ce::ConvexHullDef{.Mass = 1.0f};
    ce::BodyId boxId = CreateBox(gobjects, world, ce::Transform{}, &def1,
                                 ce::Vec3(3.0, 1.0, 3.0), ce::Vec3(1.0, 0.0, 0.0), true);
    world->Transforms[boxId].Orientation = ce::aaToQuat(ce::Vec3(0, 0, 1), glm::pi<float>() / 2.0f);
    world->Transforms[boxId].Pos.x -= 0.1f;

    ce::SphereDef sphereDef = ce::SphereDef{.Mass = 1.0f, .Radius = 1.0f};
    // ce::BodyId sphereId =
    //   CreateBall(gobjects, world, ce::Transform{.Pos = {2, 0, 0}}, &sphereDef, 10, ce::Vec3(0.3, 0.5, 0.3));

    ce::CapsuleDef capsuleDef = ce::CapsuleDef{.Mass = 1.0f, .Radius = 1.0f, .HalfLength = 1.0f};
    ce::BodyId capsuleId = CreateCapsule(gobjects, world, ce::Transform{.Pos = {-2.0, 0.0, 0.0}}, &capsuleDef, 30,
                                         ce::Vec3(0.3, 0.5, 0.3));

    draw::GObject aabbBox = draw::CreateBox(1.0, 1.0, 1.0);
    aabbBox.BaseColor = ce::Vec3(1.0, 1.0, 1.0);

    while (!glfwWindowShouldClose(gsys->mWindow)) {
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        const float deltaTime = 1.0f / 60.0f;

        draw::DrawPrep(gsys);

        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        DrawWorld(gobjects.data(), world, gsys);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (int i = 0; i < world->NumBodies(); ++i) {
            ce::AABB box = world->AABBs[i];
            bool isOverlapping = false;
            for (int j = 0; j < world->BroadPhaseBodies.size(); ++j) {
                ce::BodyPair pair = world->BroadPhaseBodies[j];
                if (pair.Body1 == i || pair.Body2 == i) {
                    isOverlapping = true;
                    break;
                }
            }

            if (isOverlapping) {
                aabbBox.BaseColor = ce::Vec3(1.0, 1.0, 0.0);
            } else {
                aabbBox.BaseColor = ce::Vec3(1.0, 1.0, 1.0);
            }

            draw::DrawAABB(&aabbBox, &gsys->mShader, box.Center.x, box.Center.y, box.Center.z, box.HalfEdge.x, box.HalfEdge.y, box.HalfEdge.z);
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        draw::ControlCamera(gsys->mWindow, &gsys->mCamera, 1.0f / 60.0f);
        if (glfwGetKey(gsys->mWindow, GLFW_KEY_Q) == GLFW_PRESS) {
            printf("Exited game\n");
            glfwSetWindowShouldClose(gsys->mWindow, 1);
        }

        ce::Step(world, deltaTime);

        static int controlId = 0;
        if (gsys->IsKeyClicked(GLFW_KEY_T)) {
            controlId = (controlId + 1) % 3;
        }
        ControlBody(gsys, controlId, world, deltaTime);

        glfwSwapBuffers(gsys->mWindow);
        glfwPollEvents();
    }
    return 0;
}
