cmake_minimum_required(VERSION 3.11)

project(Bowling
    VERSION 0.1
    DESCRIPTION "Bowling example"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STARDARD_REQUIRED ON)

FILE(GLOB MySources
    ${CMAKE_SOURCE_DIR}/examples/Bowling/*.cpp
    ${CMAKE_SOURCE_DIR}/examples/common/*.cpp)
add_executable(Bowling ${MySources})

target_compile_features(Bowling PUBLIC cxx_std_20)

target_include_directories(Bowling PRIVATE
    ${CMAKE_SOURCE_DIR}/Draw/include/
    include
    /usr/local/include
    lib
    /opt/homebrew/include
)

target_link_libraries(Bowling PUBLIC
    Draw
    CraftEngine
    /opt/homebrew/opt/glfw/lib/libglfw.3.dylib
)

add_compile_definitions(DEBUG)
