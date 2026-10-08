#include <ratio>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include "camera.hpp"
#include "ex_bowling.h"
#include "ex_joints.h"
#include "glad.h"
#include "GLFW/glfw3.h"

#include "World.h"
#include "editor.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "ex_aabb.h"
#include "ex_boxstacking.h"

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

    editor.InitDebug();
    int activeExample = Examples::BOWLING;
    bool debug_EnableAABB = false;
    float lightIntensity = 1.0;

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
        if (ImGui::Button("Reset") || editor.IsFirstStep) {
            editor.Reset();
            editor.IsFirstStep = false;

            switch (activeExample) {
                case Examples::AABB: ExAABBSetup(&editor); break;
                case Examples::JOINTS: ExJointsSetup(&editor); break;
                case Examples::BOWLING: ExBowlingSetup(&editor); break;
                case Examples::BOX_STACKING: ExBoxStackingSetup(&editor); break;
            }
        }

        ImGui::Text("Examples");

        if (ImGui::RadioButton("AABB", &activeExample, Examples::AABB)) {
            editor.Reset();
            ExAABBSetup(&editor);
        } else if (ImGui::RadioButton("Joints", &activeExample, Examples::JOINTS)) {
            editor.Reset();
            ExJointsSetup(&editor);
        } else if (ImGui::RadioButton("Bowling", &activeExample, Examples::BOWLING)) {
            editor.Reset();
            ExBowlingSetup(&editor);
        } else if (ImGui::RadioButton("Box Stacking", &activeExample, Examples::BOX_STACKING)) {
            editor.Reset();
            ExBoxStackingSetup(&editor);
        }

        if (ImGui::CollapsingHeader("Render Settings")) {
            ImGui::SliderFloat("Light Intensity", &editor.RenderSettings.LightIntensity, 0.0, 10.0);
            ImGui::SliderFloat("Ambient Intensity", &editor.RenderSettings.AmbientIntensity, 0.0, 10.0);
        }

        if (activeExample == Examples::JOINTS) {
            ExJointsUISetup(&editor);
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
