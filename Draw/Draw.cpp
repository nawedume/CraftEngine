#include "Drawing.h"
#include "GLFW/glfw3.h"
#include "camera.hpp"
#include "glad/glad.h"
#include "mesh.h"
#include <glm/gtc/type_ptr.hpp>

namespace draw {
void initBuffer(size_t size, const float *data) {
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, (void *)(3 * sizeof(float)));
}

GObject CreateBox(float x, float y, float z) {
    GObject gobj;
    float boxMesh[36 * 8];
    memcpy(boxMesh, BoxMesh, sizeof(float) * 36 * 8);
    for (int i = 0; i < 36 * 8; i += 8) {
        boxMesh[i] = BoxMesh[i] * x;
        boxMesh[i + 1] = BoxMesh[i + 1] * y;
        boxMesh[i + 2] = BoxMesh[i + 2] * z;
    }
    glGenVertexArrays(1, &gobj.Vao);
    glBindVertexArray(gobj.Vao);
    initBuffer(sizeof(boxMesh), boxMesh);
    gobj.NumElements = 36;
    return gobj;
}

GObject CreateObjectFromBuffers(GraphicBuffers buffers) {
    GObject gobj;
    glGenVertexArrays(1, &gobj.Vao);
    glBindVertexArray(gobj.Vao);
    initBuffer(sizeof(float) * 8 * buffers.NumVertices, buffers.VertexBuffer);
    GLuint idxVbo;
    glGenBuffers(1, &idxVbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idxVbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * buffers.NumIndices, buffers.IndexBuffer,
                 GL_STATIC_DRAW);

    gobj.NumElements = buffers.NumIndices;

    free(buffers.VertexBuffer);
    free(buffers.IndexBuffer);
    return gobj;
}

GObject CreateBall(float radius, unsigned int numLat, unsigned int numLong) {
    GraphicBuffers buffers = createSphereMesh(radius, 30, 30);
    return CreateObjectFromBuffers(buffers);
}

GObject CreateCapsule(float radius, float halfLength, unsigned int numLat, unsigned int numLong) {
    GraphicBuffers buffers = createCapsuleMesh(radius, halfLength, numLat, numLong);
    return CreateObjectFromBuffers(buffers);
}

GObject CreateArrow(float headWidth, float lineWidth, float length, float tailLength) {
    GraphicBuffers buffers = createArrowMesh(headWidth, lineWidth, length, tailLength);
    return CreateObjectFromBuffers(buffers);
}

void ControlCamera(GLFWwindow *window, Camera *camera, float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::BACKWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::RIGHT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::UP, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        camera->ProcessKeyboard(Camera_Movement::DOWN, deltaTime);
    }

    static double oldX = 0.0f;
    static double oldY = 0.0f;
    static bool isInitialized = false;

    double newX;
    double newY;
    glfwGetCursorPos(window, &newX, &newY);

    if (isInitialized) {
        float xOffset = newX - oldX;
        float yOffset = newY - oldY;
        camera->ProcessMouseMovement(xOffset, yOffset);
    } else {
        isInitialized = true;
    }
    oldX = newX;
    oldY = newY;
}

void DrawAABB(GObject *gobj, Shader *shader, float px, float py, float pz, float sx, float sy, float sz) {
    glm::mat4 t = glm::identity<glm::mat4>();
    t = glm::translate(t, glm::vec3(px, py, pz));
    t = glm::scale(t, glm::vec3(sx, sy, sz));

    shader->use();
    shader->setFloatMat4("uWorldTransform", (float *)glm::value_ptr(t));
    shader->setVec3("uBaseColor", (float *)glm::value_ptr(gobj->BaseColor));
    glBindVertexArray(gobj->Vao);
    glDrawArrays(GL_TRIANGLES, 0, gobj->NumElements);
}

void DrawBox(GObject *gobj, Shader *shader, float px, float py, float pz, float qw, float qx, float qy, float qz) {
    glm::mat4 t = glm::identity<glm::mat4>();
    t = glm::translate(t, glm::vec3(px, py, pz));
    t = t * glm::mat4(glm::quat(qw, qx, qy, qz));

    shader->use();
    shader->setFloatMat4("uWorldTransform", (float *)glm::value_ptr(t));
    shader->setVec3("uBaseColor", (float *)glm::value_ptr(gobj->BaseColor));
    glBindVertexArray(gobj->Vao);
    glDrawArrays(GL_TRIANGLES, 0, gobj->NumElements);
}

void DrawBall(GObject *gobj, Shader *shader, float px, float py, float pz, float qw, float qx, float qy, float qz) {
    glBindVertexArray(gobj->Vao);
    glm::mat4 t = glm::identity<glm::mat4>();
    t = glm::translate(t, glm::vec3(px, py, pz));
    t = t * glm::mat4(glm::quat(qw, qx, qy, qz));
    shader->use();
    shader->setFloatMat4("uWorldTransform", (float *)glm::value_ptr(t));
    shader->setVec3("uBaseColor", (float *)glm::value_ptr(gobj->BaseColor));
    glDrawElements(GL_TRIANGLES, gobj->NumElements, GL_UNSIGNED_INT, nullptr);
}

void DrawObject(GObject* gobj, Shader* shader, float px, float py, float pz, float qw, float qx, float qy, float qz) {
    glBindVertexArray(gobj->Vao);
    glm::mat4 t = glm::identity<glm::mat4>();
    t = glm::translate(t, glm::vec3(px, py, pz));
    t = t * glm::mat4(glm::quat(qw, qx, qy, qz));
    shader->use();
    shader->setFloatMat4("uWorldTransform", (float *)glm::value_ptr(t));
    shader->setVec3("uBaseColor", (float *)glm::value_ptr(gobj->BaseColor));
    glDrawElements(GL_TRIANGLES, gobj->NumElements, GL_UNSIGNED_INT, nullptr);
}

void DrawObject(GObject* gobj, Shader* shader, float px, float py, float pz, float qw, float qx, float qy, float qz, float sx, float sy, float sz) {
    glBindVertexArray(gobj->Vao);
    glm::mat4 t = glm::identity<glm::mat4>();
    t = glm::translate(t, glm::vec3(px, py, pz));
    t = t * glm::mat4(glm::quat(qw, qx, qy, qz));
    t = glm::scale(t, glm::vec3(sx, sy ,sz));
    shader->use();
    shader->setFloatMat4("uWorldTransform", (float *)glm::value_ptr(t));
    shader->setVec3("uBaseColor", (float *)glm::value_ptr(gobj->BaseColor));
    glDrawElements(GL_TRIANGLES, gobj->NumElements, GL_UNSIGNED_INT, nullptr);
}

void DrawPrep(GSystem *gsys) {
    gsys->mCurrentTime = glfwGetTime();
    gsys->mShader.use();
    Mat4 cameraMat = gsys->mCamera.GetViewMatrix();
    gsys->mShader.setVec3("uGlobalLightDir", glm::value_ptr(gsys->mLightDir));
    gsys->mShader.setFloatMat4("uViewTransform", glm::value_ptr(cameraMat));
    gsys->mShader.setFloatMat4("uProjectionTransform", glm::value_ptr(gsys->mProjectionMat));
}

void APIENTRY debugOpenGL(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message,
                          const void *userParam) {
    printf("%d : %d : %d : %s", source, type, id, message);
}

GSystem *InitGSystem(int windowWidth, int windowHeight, float px, float py, float pz) {
    glfwInit();
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);

    GLFWwindow *window = glfwCreateWindow(windowWidth, windowHeight, "Integration Example", nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr, "Could not create GLFW window\n");
        exit(1);
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        fprintf(stderr, "Could not initialize GLAD\n");
        exit(1);
    }

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    Shader shader("../Draw/shaders/vshader.glsl", "../Draw/shaders/fshader.glsl");
    float aspect = static_cast<float>(windowWidth) / windowHeight;
    glm::mat4 projectionMat = glm::perspective((float)glm::radians(45.0), aspect, 0.1f, 1000.f);
    GSystem *sys = new GSystem(shader, Camera(glm::vec3(px, py, pz)), window, projectionMat);

    return sys;
}
} // namespace draw
