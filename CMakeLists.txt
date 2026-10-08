cmake_minimum_required(VERSION 3.11)
project(Main)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

enable_testing()

add_subdirectory(CraftEngine)
add_subdirectory(test)
add_subdirectory(editor)
