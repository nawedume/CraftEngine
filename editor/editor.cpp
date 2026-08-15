#include "Core.h"
#include "Math.hpp"
#include "camera.hpp"
#include "editor.h"
#include "World.h"
#include "AABB.h"
#include "glm/detail/type_vec.hpp"
#include "shader.hpp"
#include <cassert>
#include <glad.h>
#include "mesh.h"
#include <glm/gtc/type_ptr.hpp>

// Assuming each face can have a maxiumum of 8 vertices,
// not actually enforced but I think it's a fine assumption for most cases
#define MAX_NUM_VERTICES MAX_NUM_FEATURES_CONVEX_HULL * 8
// 3 indicies per face, times the max number of faces times a multiplier
#define MAX_NUM_INDICES  3 * MAX_NUM_FEATURES_CONVEX_HULL * 2

namespace ceeditor {

static void EditorModeKeyCallback(GLFWwindow* window, int key, int sc, int action, int mods) {
    Editor* editor = (Editor*) glfwGetWindowUserPointer(window);
    assert(editor != NULL);

    // general callbacks
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        exit(0);
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        editor->IsSimulating = !editor->IsSimulating;
    }

    switch (editor->Mode) {
        case EditorMode::Select:
        break;
        case EditorMode::Move:
        break;
    }
}

static void EditorScrollWheelCallback(GLFWwindow* window, double xoffset, double yoffset) {
    Editor* editor = (Editor*) glfwGetWindowUserPointer(window);
    assert(editor != NULL);

    editor->UpdateCameraMovementSpeed(yoffset);
}


Editor::Editor(GLFWwindow* window): Window(window) {
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    SolidShader = new Shader("../editor/shaders/transform_shader.glsl", "../editor/shaders/diffuse_shader.glsl");
    ViewCamera = new Camera(
        glm::vec3(10., 10., 10.),
        glm::vec3(0.0f, 1.0f, 0.0f),
        -135.0,
        -35.0
    );
    float aspect = static_cast<float>(width) / height;
    ProjMat = glm::perspective((float)glm::radians(45.0), aspect, 0.1f, 1000.f);

    World = NewWorld();

    this->Mode = EditorMode::Select;
    glfwSetWindowUserPointer(Window, this);
    glfwSetKeyCallback(Window, EditorModeKeyCallback);
    glfwSetScrollCallback(Window, EditorScrollWheelCallback);

    GLuint quadVao;
    glGenVertexArrays(1, &quadVao);
    glBindVertexArray(quadVao);

    GLuint quadVbo;
    glGenBuffers(1, &quadVbo);
    glBindBuffer(GL_ARRAY_BUFFER, quadVbo);

    float quadVertics[30] = {
        -1.0,  0.0, -1.0,   0.0, 0.0,
         1.0,  0.0,  1.0,   1.0, 1.0,
         1.0,  0.0, -1.0,   1.0, 0.0,

        -1.0,  0.0, -1.0,   0.0, 0.0,
         1.0,  0.0,  1.0,   1.0, 1.0,
        -1.0,  0.0,  1.0,   0.0, 1.0,
    };
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertics), quadVertics, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void*) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, (void*) (3 * sizeof(float)));

    glBindVertexArray(0);

    GridDrawContext = {
        .MeshDataVao = quadVao,
        .Size = 18,
        .BaseColor = Vec3(0.3, 0.3, 0.3),
    };
    GridShader = new Shader("../editor/shaders/grid_vs_shader.glsl", "../editor/shaders/grid_fs_shader.glsl");
}

void Editor::AddConvexHullMesh(BodyId bodyId, ConvexHull* hull, Vec3 color) {
    // @todo replace with arena allocator
    u32 indices[MAX_NUM_INDICES];
    // Multiply by 2 to account for normals per vertex
    Vec3 vertexData[MAX_NUM_VERTICES * 2];
    Vec3* vertexPos = vertexData;
    Vec3* vertexNormal = vertexData + MAX_NUM_VERTICES;

    Vec3i coordinateData[MAX_NUM_VERTICES * 2];
    Vec3i* barycentricCoords = coordinateData;
    Vec3i* edgeMasks = coordinateData + MAX_NUM_VERTICES;

    u32 numIndices = 0;
    u32 numVertexEntries = 0;

    for (int i = 0; i < hull->NumFaces; ++i) {
        Face& face = hull->Faces[i];
        Vec3 normal = face.Normal;

        HalfEdge* edge = &hull->HalfEdges[hull->FirstEdgeIndex[i]];
        u8 rootVertex = edge->Vertex1;

        u32 rootBufferIdx = numVertexEntries;
        vertexPos[rootBufferIdx] = hull->Vertices[rootVertex];
        vertexNormal[rootBufferIdx] = normal;
        ++numVertexEntries;

        // Alternate the bary centric coordinates for each vertex but keep the root at 1, 0, 0
        // For the masks, set the mask for each vertex to be 1 in the x for the edge opposite of the root
        // and at the end set the root and midVertexBuffer to have 1 in the z/y as well, this will make the fragment shader
        // have the correct mask values to apply
        barycentricCoords[rootBufferIdx] = Vec3i(1, 0, 0);

        int yBary = 1;
        int zBary = 0;
        u32 midVertexBufferIdx;
        while (1) {
            HalfEdge* secondEdge = &hull->HalfEdges[edge->NextEdge];

            midVertexBufferIdx = numVertexEntries++;
            vertexPos[midVertexBufferIdx] = hull->Vertices[secondEdge->Vertex1];
            vertexNormal[midVertexBufferIdx] = normal;

            u8 tailVertex = secondEdge->Vertex2;

            barycentricCoords[midVertexBufferIdx] = Vec3i(0, yBary, zBary);
            yBary = 1 - yBary;
            zBary = 1 - zBary;
            edgeMasks[midVertexBufferIdx] = Vec3i(1, 0, 0);

            if (tailVertex == rootVertex) {
                break;
            }

            indices[numIndices++] = rootBufferIdx;
            indices[numIndices++] = midVertexBufferIdx;
            indices[numIndices++] = midVertexBufferIdx + 1;

            edge = secondEdge;
        };

        // edgeMasks[rootBufferIdx + 0] = Vec3i(1, 0, 1);
        edgeMasks[rootBufferIdx + 2] = Vec3i(1, 0, 1);
        edgeMasks[midVertexBufferIdx] = Vec3i(1, yBary, zBary);
    }

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), (void*) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), (void*) (sizeof(float) * MAX_NUM_VERTICES));

    GLuint coordVbo;
    glGenBuffers(1, &coordVbo);
    glBindBuffer(GL_ARRAY_BUFFER, coordVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(coordinateData), coordinateData, GL_STATIC_DRAW);

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_UNSIGNED_INT, GL_FALSE, sizeof(Vec3i), (void*) 0);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_UNSIGNED_INT, GL_FALSE, sizeof(Vec3i), (void*) (sizeof(Vec3i) * MAX_NUM_VERTICES));

    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);


    DrawContext context = {
        .Body = bodyId,
        .MeshDataVao=vao,
        .Size=numIndices,
        .BaseColor=color,
    };

    this->IndexedElements.push_back(context);
}

BodyId Editor::AddConvex(ConvexHullDef def, Transform t, Vec3 color) {
    BodyId bodyId = AddConvexHull(this->World, t, def);
    AddConvexHullMesh(bodyId, def.Hull, color);
    return bodyId;
}

BodyId Editor::AddBox(BoxDef def, Transform t, Vec3 color) {
    BodyId bodyId = ce::AddBox(this->World, t, def);
    GraphicBuffers buffer = createBoxMesh(def.HalfEdge.x, def.HalfEdge.y, def.HalfEdge.z);
    BuffersToDrawContext(bodyId, buffer, color);
    return bodyId;
}

BodyId Editor::AddCapsule(CapsuleDef def, Transform t, Vec3 color) {
    BodyId bodyId = ce::AddCapsule(this->World, t, def);
    GraphicBuffers buffer = createCapsuleMesh(def.Radius, def.HalfLength, def.Radius * 10, def.Radius * 10);
    BuffersToDrawContext(bodyId, buffer, Vec3(0.0, 0.3, 0.5));
    return bodyId;
}

BodyId Editor::AddBall(SphereDef def, Transform t, Vec3 color) {
    BodyId bodyId = ce::AddSphere(this->World, t, def);
    GraphicBuffers buffer = createSphereMesh(def.Radius, def.Radius * 10, def.Radius * 10);
    BuffersToDrawContext(bodyId, buffer, Vec3(0.1, 0.3, 0.7));
    return bodyId;
}

void Editor::BuffersToDrawContext(BodyId bodyId, GraphicBuffers buffer, ce::Vec3 baseColor)  {
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, buffer.NumVertices * sizeof(float) * 8, buffer.VertexBuffer, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, (void*) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, (void*) (sizeof(float) * 3));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, (void*) (sizeof(float) * 6));

    if (buffer.NumIndices != 0) {
        GLuint ebo;
        glGenBuffers(1, &ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, buffer.NumIndices, buffer.IndexBuffer, GL_STATIC_DRAW);

        DrawContext context = {
            .Body = bodyId,
            .MeshDataVao=vao,
            .Size=buffer.NumIndices,
            .BaseColor=baseColor,
        };

        IndexedElements.push_back(context);

    } else {
        DrawContext context = {
            .Body = bodyId,
            .MeshDataVao=vao,
            .Size=buffer.NumVertices,
            .BaseColor=baseColor,
        };

        FlatElements.push_back(context);
    }

    free(buffer.VertexBuffer);
    free(buffer.IndexBuffer);
}

void Editor::DrawWorld() {
    // DrawGrid();

    Mat4 cameraMat = this->ViewCamera->GetViewMatrix();
    SolidShader->use();
    SolidShader->setVec3("uGlobalLightDir", (float*) glm::value_ptr(glm::vec3(1.0, 1.0, 1.0)));
    SolidShader->setFloatMat4("uCameraTransform", glm::value_ptr(cameraMat));
    SolidShader->setFloatMat4("uProjTransform", glm::value_ptr(ProjMat));
    SolidShader->setVec3s("uLightDir", 1, glm::value_ptr(RenderSettings.LightDir));
    SolidShader->setFloat("uAmbient", RenderSettings.AmbientIntensity);

    DrawFlatElements();
    DrawIndexedElements();

    // @todo Add a instanced shader here that can fetch from the buffer
    // DrawInstancedFlatElements();
}

void Editor::DrawFlatElements() {
    for (auto& context : FlatElements) {
        Mat4 m = this->World->Transforms[context.Body].Matrix();

        SolidShader->setFloatMat4("uWorldTransform", (float*) glm::value_ptr(m));
        SolidShader->setVec3("uBaseColor", (float*) glm::value_ptr(context.BaseColor));

        glBindVertexArray(context.MeshDataVao);
        glDrawArrays(GL_TRIANGLES, 0, context.Size);
    }
}

void Editor::DrawIndexedElements() {
    for (auto& context : IndexedElements) {
        Mat4 m = this->World->Transforms[context.Body].Matrix();

        SolidShader->setFloatMat4("uWorldTransform", (float*) glm::value_ptr(m));
        SolidShader->setVec3("uBaseColor", (float*) glm::value_ptr(context.BaseColor));

        glBindVertexArray(context.MeshDataVao);
        glDrawElements(GL_TRIANGLES, context.Size, GL_UNSIGNED_INT, nullptr);
    }
}

void Editor::DrawInstancedFlatElements() {
    for (auto& context : InstancedFlatElements) {
        Mat4 m = this->World->Transforms[context.StartBody].Matrix();

        SolidShader->setVec3("uBaseColor", (float*) glm::value_ptr(context.BaseColor));

        // Set the the transform array here to use, need a new shader as well

        glBindVertexArray(context.MeshDataVao);
        glDrawArraysInstanced(GL_TRIANGLES, 0, context.Size, context.EndBody - context.StartBody + 1);
    }
}

void Editor::HandleCameraMoveMotion() {
    if (glfwGetKey(Window, GLFW_KEY_W) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::FORWARD, DeltaTime);
    }
    if (glfwGetKey(Window, GLFW_KEY_S) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::BACKWARD, DeltaTime);
    }
    if (glfwGetKey(Window, GLFW_KEY_D) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::RIGHT, DeltaTime);
    }
    if (glfwGetKey(Window, GLFW_KEY_A) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::LEFT, DeltaTime);
    }
    if (glfwGetKey(Window, GLFW_KEY_E) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::UP, DeltaTime);
    }
    if (glfwGetKey(Window, GLFW_KEY_Q) == GLFW_PRESS) {
        ViewCamera->ProcessKeyboard(Camera_Movement::DOWN, DeltaTime);
    }

    double cursorPosX, cursorPosY;
    glfwGetCursorPos(Window, &cursorPosX, &cursorPosY);
    Vec2 offset = Vec2(cursorPosX, cursorPosY) - OldCursorPos;
    ViewCamera->ProcessMouseMovement(offset.x, offset.y);
    OldCursorPos = Vec2(cursorPosX, cursorPosY);
}

void Editor::ToggleToMoveMode() {
    glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    Mode = EditorMode::Move;
    double cursorPosX, cursorPosY;
    glfwGetCursorPos(Window, &cursorPosX, &cursorPosY);
    OldCursorPos = Vec2(cursorPosX, cursorPosY);
}

void Editor::ToggleToSelectMode() {
    glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);
    Mode = EditorMode::Select;
}

void Editor::HandleInput() {

    switch (this->Mode) {
        case EditorMode::Select:
            if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
                ToggleToMoveMode();
            }
            break;
        case EditorMode::Move:
            HandleCameraMoveMotion();
            if (glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE) {
                ToggleToSelectMode();
            }
            break;
   }
}

void Editor::DrawGrid() {
    // @todo looks fine for now, but need to work on anti-aliasing, too distracting
    GridShader->use();
    GridShader->setFloat("uScale", 4096.0);
    GridShader->setFloatMat4("uCameraTransform", (float*) glm::value_ptr(ViewCamera->GetViewMatrix()));
    GridShader->setFloatMat4("uProjTransform", glm::value_ptr(ProjMat));
    GridShader->setVec3("uBaseColor", 1.0, 1.0, 1.0);

    glBindVertexArray(GridDrawContext.MeshDataVao);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

void Editor::UpdateCameraMovementSpeed(float speedIncrement) {
    float newSpeed = ViewCamera->MovementSpeed + speedIncrement;
    newSpeed = clamp(newSpeed, 1e-6, 1000.0);
    ViewCamera->MovementSpeed = newSpeed;
}
}
