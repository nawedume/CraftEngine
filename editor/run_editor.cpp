#include <ratio>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include "camera.hpp"
#include "glad.h"
#include "GLFW/glfw3.h"

#include "World.h"
#include "editor.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "ex_aabb.h"

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 1000

using namespace ceeditor;

void APIENTRY glDebugOutput(
    GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar* message,
    const void* userParam)
{
    std::cerr
        << "OpenGL Debug [" << id << "]: "
        << message << "\n";
}

GLFWwindow* initGlfw() {
    glfwInit();
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    // GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, "Editor", monitor, nullptr);
    GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, "Editor", nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr, "Could not create GLFW window\n");
        exit(1);
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        fprintf(stderr, "Could not initialize GLAD\n");
        exit(1);
    }

    return window;
}

enum Examples {
    AABB,
    JOINTS,
    BOWLING,
    BOX_STACKING,
    GJK,
};

int main() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    auto window = initGlfw();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();


    Editor editor(window);

    // BoxDef boxDef = {
    //     .Mass = 1.0,
    //     .HalfEdge = { 1., 1., 1. },
    // };

    // for (int i = 0; i < 10; ++i) {
    //     BodyId box = editor.AddBox(boxDef,
    //         Transform{.Pos = {0.0f, 2.0f + (i * 2.1), 0.0f}}
    //     );
    // }

    // BodyId floor = editor.AddBox({ .HalfEdge = { 10.0, 1.0, 10.0 } }, {}, { 0.8, 0.8, 0.8 });
    // SetStatic(editor.World, floor);

    // editor.World->GravityAcc.y = 0.0;

    editor.InitDebug();
    int activeExample = -1;
    bool debug_EnableAABB = false;

    printf("World interations: %d, %d\n", editor.World->Settings.NumOfSolverIterations, editor.World->Settings.NumOfRelaxationIterations);
    while (!glfwWindowShouldClose(window)) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();

        ImGui::NewFrame();
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        float paneWidth = 300.0;
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x  - paneWidth, viewport->WorkPos.y));
        ImGui::SetNextWindowSize(ImVec2(paneWidth, viewport->WorkSize.y));

        ImGui::Begin("Editor");
        ImGui::Text("Examples");

        if (ImGui::RadioButton("AABB", &activeExample, Examples::AABB)) {
            editor.Reset();
            ExAABBSetup(&editor);
        } else if (ImGui::RadioButton("Joints", &activeExample, Examples::JOINTS)) {
            editor.Reset();
        } else if (ImGui::RadioButton("Bowling", &activeExample, Examples::BOWLING)) {
            editor.Reset();
        } else if (ImGui::RadioButton("Box Stacking", &activeExample, Examples::BOX_STACKING)) {
            editor.Reset();
        } else if (ImGui::RadioButton("GJK", &activeExample, Examples::GJK)) {
            editor.Reset();
        }

        ImGui::Text("Debug");
        ImGui::Checkbox("Enable AABB", &debug_EnableAABB);

        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (editor.IsSimulating) {
            auto start = std::chrono::steady_clock::now();
            Step(editor.World, editor.DeltaTime);
            std::chrono::duration<double, std::milli> duration = std::chrono::steady_clock::now() - start;
            ImGui::Text("Physics (ms): %f", duration.count());
        }

        ImGui::End();

        editor.HandleInput();
        editor.DrawWorld();
        if (debug_EnableAABB) {
            editor.DrawBoundingBoxes();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    return 0;
}
