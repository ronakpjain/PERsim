cmake_minimum_required(VERSION 3.25)

# Generated headers and fetched dependencies belong in a disposable build tree.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported; run `cmake --preset default` first.")
endif()

project(PERsim VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(PERSIM_BUILD_VIEWER "Build the raylib viewer" ON)
option(PERSIM_WITH_FIL "Build the closed-loop firmware backend" ON)
set(PERSIM_FIL_DIR "$ENV{HOME}/coding/projects/fil" CACHE PATH "FIL source checkout (main)")
set(PERSIM_MONOREPO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../monorepo" CACHE PATH "PER monorepo")
set(PERSIM_SUPERDBC "" CACHE FILEPATH "SuperDBC JSON; empty selects the sole versioned file")

function(persim_enable_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

include(cmake/Dependencies.cmake)
include(cmake/SiblingModels.cmake)
include(cmake/Codegen.cmake)

# Plant core: parameters, model registries, physics, devices, and CAN codecs.
add_library(persim_core
    src/parameters.cpp src/tire.cpp src/terrain.cpp
    src/vehicle.cpp src/model_components.cpp src/powertrain.cpp
    src/protocol.cpp src/can_link.cpp src/sensors.cpp
    "${protocol_header}" "${suspension_header}")
target_include_directories(persim_core PUBLIC include "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(persim_core PUBLIC Eigen3::Eigen Threads::Threads PRIVATE persim_models)
persim_enable_warnings(persim_core)

if(PERSIM_WITH_FIL)
    if(NOT EXISTS "${PERSIM_FIL_DIR}/include/fil/sim/world.hpp")
        message(FATAL_ERROR "Set PERSIM_FIL_DIR to the FIL source checkout")
    endif()
    set(FIL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(FIL_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(FIL_ENABLE_IPO OFF CACHE BOOL "" FORCE)
    # FIL's top-level CMake would otherwise replace the source-root compile database.
    set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
    add_subdirectory("${PERSIM_FIL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/fil" EXCLUDE_FROM_ALL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    target_sources(persim_core PRIVATE src/fil_bridge.cpp)
    target_link_libraries(persim_core PRIVATE fil_core)
endif()
target_compile_definitions(persim_core PUBLIC PERSIM_WITH_FIL=$<BOOL:${PERSIM_WITH_FIL}>)

# Frontends: orchestration, headless runner, and optional raylib viewer.
add_library(persim_session src/session.cpp src/options.cpp src/telemetry.cpp)
target_link_libraries(persim_session PUBLIC persim_core)
persim_enable_warnings(persim_session)

add_executable(persim-headless src/headless.cpp)
target_link_libraries(persim-headless PRIVATE persim_session)

if(PERSIM_BUILD_VIEWER)
    add_executable(persim src/viewer.cpp)
    target_link_libraries(persim PRIVATE persim_session raylib)
    get_target_property(raylib_headers raylib INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(persim SYSTEM PRIVATE ${raylib_headers})
    persim_enable_warnings(persim)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(persim-vcan guest/vcan_bridge.c)
    persim_enable_warnings(persim-vcan)
endif()

if(DOXYGEN_FOUND)
    configure_file(Doxyfile.in Doxyfile @ONLY)
    add_custom_target(docs
        COMMAND Doxygen::doxygen "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Building the PERsim API and model reference" VERBATIM)
endif()
