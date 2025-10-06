cmake_minimum_required(VERSION 3.11)

project(AABB
    VERSION 0.1
    DESCRIPTION "A lib"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STARDARD_REQUIRED ON)

FILE(GLOB MySources
    ${CMAKE_SOURCE_DIR}/examples/aabb/*.cpp
    ${CMAKE_SOURCE_DIR}/examples/common/*.cpp)
add_executable(AABB ${MySources})

target_compile_features(AABB PUBLIC cxx_std_20)

target_include_directories(AABB PRIVATE
    ${CMAKE_SOURCE_DIR}/Draw/include/
    include
    /usr/local/include
    lib
    /opt/homebrew/include
    ${CMAKE_SOURCE_DIR}/CraftEngine/include/
    ${CMAKE_SOURCE_DIR}/examples/common/
)

find_package(glfw3 REQUIRED)

target_link_libraries(AABB PUBLIC
    Draw
    CraftEngine
    glfw
)

add_compile_definitions(DEBUG)
