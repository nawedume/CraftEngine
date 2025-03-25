cmake_minimum_required(VERSION 3.8)

project(BoxStacking
    VERSION 0.1
    DESCRIPTION "Box stacking example"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STARDARD_REQUIRED ON)

FILE(GLOB MySources
    ${CMAKE_SOURCE_DIR}/examples/boxstacking/*.cpp
    ${CMAKE_SOURCE_DIR}/examples/common/*.cpp)
add_executable(BoxStacking ${MySources})

target_compile_features(BoxStacking PUBLIC cxx_std_20)

target_include_directories(BoxStacking PRIVATE
    ${CMAKE_SOURCE_DIR}/Draw/include/
    include
    /usr/local/include
    lib
    /opt/homebrew/include
)

target_link_libraries(BoxStacking PUBLIC
    Draw
    CraftEngine
    /opt/homebrew/opt/glfw/lib/libglfw.3.dylib
)

add_compile_definitions(DEBUG)
