cmake_minimum_required(VERSION 3.11)
project(Main)

find_package(Catch2 3 REQUIRED)
include(CTest)
include(Catch)

add_executable(CraftEngineTests
    BoxStackingTests.cpp
)

target_link_libraries(CraftEngineTests PRIVATE CraftEngine Catch2::Catch2WithMain)


catch_discover_tests(CraftEngineTests)
