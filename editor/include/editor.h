#include "Core.h"
#include "World.h"
#include "camera.hpp"
#include "glad.h"
#include "shader.hpp"
#include "GLFW/glfw3.h"
#include <unordered_map>

namespace ceeditor {
    using namespace ce;

    // Used to hold the drawing data for a single call
    // For example, drawing all the rigid bodies in the scene
    // which should use the same shader, and ideally be held in the same buffer.
    // Or drawing all AABBs, etc
    struct DrawContext {
        GLuint MeshDataVao;
        u32 Size;
        Vec3 BaseColor;
    };

    struct Ray {
        Vec3 Origin;
        Vec3 Dir;
    };

    enum class EditorMode {
        Select,
        Move
    };

    struct Editor {
        World* World;
        float DeltaTime = 1.0 / 60.0;
        bool IsSimulating = false;

        Editor(GLFWwindow* window);
        BodyId AddBox(BoxDef def, Transform t);
        BodyId AddConvex(ConvexHullDef def, Transform t);
        void DrawWorld();
        void DrawBoundingBoxes();
        void SetColor(BodyId bid, Vec3 color) {
            BodyToDrawCtx.at(bid).BaseColor = color;
        }
        BodyId RayCast(Ray ray);
        void HandleInput();

        void ToggleToMoveMode();
        void ToggleToSelectMode();

        // @todo
        // void DrawDebug();
        EditorMode Mode;

        private:
        std::unordered_map<BodyId, DrawContext> BodyToDrawCtx;
        Shader* SolidShader;
        Camera* ViewCamera;
        Mat4 ProjMat;
        GLFWwindow* Window;

        Vec2 OldCursorPos = { 0.0, 0.0 };
        DrawContext GridDrawContext;
        Shader* GridShader;

        void DrawBox(DrawContext& context);
        void DrawHull(DrawContext& context);
        void DrawObject(BodyId bid, DrawContext& context);
        void AddConvexHullMesh(BodyId bodyId, ConvexHull* hull);
        void HandleCameraMoveMotion();
        void DrawGrid();
    };
}
