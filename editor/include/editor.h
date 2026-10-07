#pragma once

#include "Core.h"
#include "Math.hpp"
#include "World.h"
#include "camera.hpp"
#include "shader.hpp"
#include "GLFW/glfw3.h"
#include <vector>
#include "mesh.h"

namespace ceeditor {
    using namespace ce;

    // Used to hold the drawing data for a single call
    // For example, drawing all the rigid bodies in the scene
    // which should use the same shader, and ideally be held in the same buffer.
    // Or drawing all AABBs, etc
    struct DrawContext {
        ce::BodyId Body;
        GLuint MeshDataVao;
        u32 Size;
        Vec3 BaseColor;
    };

    struct InstancedDrawContext {
        ce::BodyId StartBody;
        ce::BodyId EndBody;
        GLuint MeshDataVao;
        GLuint MeshTransformVbo;
        std::vector<Transform> InstancedTransformsScratch;
        u32 Size;
        u32 InstancedCount;
        Vec3 BaseColor;
    };

    struct DebugAABBDrawContext {
        GLuint LineDataVao;
        GLuint MeshTransformVbo;
        GLuint MeshSizeVbo;
        u32 InstanceCount;
        Vec3 Color;

        std::vector<Vec3> InstancedTransformsScratch;
        std::vector<Vec3> InstancedSizeScratch;
    };

    struct Ray {
        Vec3 Origin;
        Vec3 Dir;
    };

    enum class EditorMode {
        Select,
        Move
    };

    struct RenderSettings {
        Vec3 LightDir = normalize(Vec3(1.0, 1.0, 1.0));
        float AmbientIntensity = 0.3;
    };
    struct Editor {
        World* World;
        RenderSettings RenderSettings {};
        float DeltaTime = 1.0 / 60.0;
        bool IsSimulating = false;

        Editor(GLFWwindow* window);

        BodyId AddBox(BoxDef def, Transform t, Vec3 color = Vec3(1.0, 0.0, 0.0));
        BodyId AddConvex(ConvexHullDef def, Transform t, Vec3 color);
        BodyId AddCapsule(CapsuleDef def, Transform t, Vec3 color);
        BodyId AddBall(SphereDef def, Transform t, Vec3 color);

        void DrawWorld();
        void DrawBoundingBoxes();
        BodyId RayCast(Ray ray);
        void HandleInput();

        void ToggleToMoveMode();
        void ToggleToSelectMode();

        void UpdateCameraMovementSpeed(float speedIncrement);
        void Reset();
        void InitDebug();

        // @todo
        // void DrawDebug();
        EditorMode Mode;

        private:
        Shader* SolidShader;
        Camera* ViewCamera;
        Mat4 ProjMat;
        GLFWwindow* Window;

        std::vector<DrawContext> FlatElements;
        std::vector<DrawContext> IndexedElements;
        std::vector<InstancedDrawContext> InstancedFlatElements;

        Vec2 OldCursorPos = { 0.0, 0.0 };
        DrawContext GridDrawContext;
        Shader* GridShader;

        void DrawIndexedElements();
        void DrawFlatElements();
        void DrawInstancedFlatElements();
        void BuffersToDrawContext(BodyId bodyId, GraphicBuffers buffer, ce::Vec3 baseColor);

        void AddConvexHullMesh(BodyId bodyId, ConvexHull* hull, Vec3 color);
        void HandleCameraMoveMotion();
        void DrawGrid();

        // Debug stuff
        bool DebugIsDirty = false;
        Shader* DebugLineShader;
        DebugAABBDrawContext DebugAABBContext; // holds all lines for all objects.
    };
}
