#pragma once

#include "camera.hpp"
#include "shader.hpp"
#include <GLFW/glfw3.h>

namespace draw {
typedef glm::vec3 Vec3;
typedef glm::mat4 Mat4;

struct GObject {
    GLuint Vao;
    GLuint NumElements;
    Vec3 BaseColor;
};

struct GSystem {
    Shader mShader; Camera mCamera;
    GLFWwindow *mWindow;
    Mat4 mProjectionMat;

    Vec3 mLightDir = {0.0f, 0.0f, 0.0f};

    GSystem(Shader shader, Camera camera, GLFWwindow *window, Mat4 projMat)
        : mShader{shader}, mCamera{camera}, mWindow{window}, mProjectionMat{projMat} {}

    double mCurrentTime;
    double mKeyPressedTime[256]{};

    bool IsKeyClicked(int keyID) {
        if (glfwGetKey(mWindow, keyID) == GLFW_PRESS && mCurrentTime > (mKeyPressedTime[keyID] + 1.0f)) {
            mKeyPressedTime[keyID] = mCurrentTime;
            return true;
        }

        return false;
    }
};

extern void DrawPrep(GSystem *gsys);

extern void ControlCamera(GLFWwindow *window, Camera *camera, float deltaTime);

extern GObject CreateBox(float x, float y, float z);

extern GObject CreateBall(float radius, unsigned int numLat, unsigned int numLong);

extern GObject CreateCapsule(float radius, float halfLength, unsigned int numLat, unsigned int numLong);

extern GObject CreateArrow(float headWidth, float lineWidth, float length, float tailLength);

extern void DrawAABB(GObject *gobj, Shader *shader, float px, float py, float pz, float sx, float sy, float sz);

extern void DrawBox(GObject *gobj, Shader *shader, float px, float py, float pz, float qw, float qx, float qy,
                    float qz);

extern void DrawBall(GObject *gobj, Shader *shader, float px, float py, float pz, float qw, float qx, float qy,
                     float qz);

extern void DrawObject(GObject* gobj, Shader* shader, float px, float py, float pz, float qw, float qx, float qy, float qz);

extern GSystem *InitGSystem(int windowWidth, int windowHeight, float px, float py, float pz);
} // namespace draw
