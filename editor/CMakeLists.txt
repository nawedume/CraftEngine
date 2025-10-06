cmake_minimum_required(VERSION 3.11)

project(Editor
    VERSION 0.1
    DESCRIPTION "Editor"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB ImGuiSources
    ${CMAKE_SOURCE_DIR}/external/imgui/*.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/*.h
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3.cpp
    ${CMAKE_SOURCE_DIR}/external/imgui/misc/cpp/imgui_stdlib.*
    ${CMAKE_SOURCE_DIR}/external/glad.cpp
)

add_executable(Editor
    ${CMAKE_SOURCE_DIR}/editor/run_editor.cpp
    ${CMAKE_SOURCE_DIR}/editor/editor.cpp
    ${ImGuiSources}
)

target_include_directories(Editor PRIVATE
    ${CMAKE_SOURCE_DIR}/CraftEngine/include/
    ${CMAKE_SOURCE_DIR}/CraftEngine/src/
    ${CMAKE_SOURCE_DIR}/external/imgui/
    ${CMAKE_SOURCE_DIR}/external/imgui/backends/
    ${CMAKE_SOURCE_DIR}/editor/include/
)

target_link_libraries(Editor PRIVATE
    CraftEngine
    /opt/homebrew/opt/glfw/lib/libglfw.3.dylib
)
