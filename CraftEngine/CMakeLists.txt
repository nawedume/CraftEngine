cmake_minimum_required(VERSION 3.8)

project(CraftEngine
    VERSION 0.1
    DESCRIPTION "A physics library used for a specific game"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  simde
  GIT_REPOSITORY https://github.com/simd-everywhere/simde.git
  GIT_TAG        master # or a specific commit/tag
)

FetchContent_MakeAvailable(simde)


# Set default build type to Debug if not specified
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
endif()

FILE(GLOB MySources src/*.cpp)
message(STATUS "My Sources ${MySources}")
add_library(CraftEngine ${MySources})
target_compile_features(CraftEngine PUBLIC cxx_std_20)

set_target_properties(CraftEngine PROPERTIES
    VERSION ${PROJECT_VERSION}
    PUBLIC_HEADER "include/Core.h;include/World.h")

target_include_directories(CraftEngine PUBLIC
    include
    /usr/local/include
    lib
    /opt/homebrew/include
    ${CMAKE_SOURCE_DIR}/examples/common/
    ${CMAKE_SOURCE_DIR}/Draw/include/
    ${simde_SOURCE_DIR}

)

target_compile_definitions(CraftEngine PRIVATE SIMDE_ENABLE_NATIVE_ALIASES)


option(SET_DEBUG "Set the debug flag for the build" ON)
if (SET_DEBUG)
    add_compile_definitions(DEBUG)
endif()

include(GNUInstallDirs)
install(TARGETS CraftEngine
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
