cmake_minimum_required(VERSION 3.8)

project(Draw VERSION 0.1)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FILE(GLOB Sources ${CMAKE_SOURCE_DIR}/Draw/*.cpp)
message(STATUS "Sources: ${Sources}")
add_library(Draw ${Sources})
target_compile_features(Draw PUBLIC cxx_std_20)

set_target_properties(Draw PROPERTIES VERSION ${PROJECT_VERSION} PUBLIC_HEADER "include/Drawing.h")

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-v")

target_include_directories(Draw PUBLIC
    ${CMAKE_SOURCE_DIR}/Draw/
    ${CMAKE_SOURCE_DIR}/Draw/include/
    include
    /usr/local/include
    lib
    /opt/homebrew/include
)

add_compile_definitions(DEBUG)
