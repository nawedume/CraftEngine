cmake_minimum_required(VERSION 3.8)
project(Main)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

enable_testing()

add_subdirectory(Draw)
add_subdirectory(CraftEngine)
add_subdirectory(test)
add_subdirectory(examples/boxstacking)
add_subdirectory(examples/gjk)
add_subdirectory(examples/aabb)
add_subdirectory(examples/joints)
