cmake_minimum_required(VERSION 3.25)
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
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)
include(cmake/Modeling.cmake)
if(NOT PERSIM_SUPERDBC)
    file(GLOB dbc_files "${PERSIM_MONOREPO_DIR}/superdbc/superdbc_*.json")
    list(LENGTH dbc_files dbc_count)
    if(NOT dbc_count EQUAL 1)
        message(FATAL_ERROR "Set PERSIM_SUPERDBC to one versioned SuperDBC JSON")
    endif()
    list(GET dbc_files 0 PERSIM_SUPERDBC)
endif()
set(protocol_header "${CMAKE_CURRENT_BINARY_DIR}/generated/persim/protocol_db.hpp")
add_custom_command(OUTPUT "${protocol_header}"
    COMMAND Python3::Interpreter "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_protocol.py"
            "${PERSIM_SUPERDBC}" "${protocol_header}"
    DEPENDS tools/generate_protocol.py "${PERSIM_SUPERDBC}" VERBATIM)
set(suspension_header "${CMAKE_CURRENT_BINARY_DIR}/generated/persim/suspension_db.hpp")
add_custom_command(OUTPUT "${suspension_header}"
    COMMAND Python3::Interpreter "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_suspension.py"
            "${PERSIM_MODELING_DIR}" "${suspension_header}"
    DEPENDS tools/generate_suspension.py
        "${PERSIM_MODELING_DIR}/models/suspension/suspension_kinematics.py"
        "${PERSIM_MODELING_DIR}/models/suspension/data/kinematics_table.npz" VERBATIM)
add_library(persim_core
    src/vehicle.cpp src/model_components.cpp src/powertrain.cpp src/protocol.cpp src/can_link.cpp src/sensors.cpp
    "${protocol_header}" "${suspension_header}")
target_include_directories(persim_core PUBLIC include "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(persim_core PUBLIC Eigen3::Eigen Threads::Threads PRIVATE persim_models)
target_compile_options(persim_core PRIVATE -Wall -Wextra -Wpedantic)

if(PERSIM_WITH_FIL)
    if(NOT EXISTS "${PERSIM_FIL_DIR}/include/fil/sim/world.hpp")
        message(FATAL_ERROR "Set PERSIM_FIL_DIR to the FIL source checkout")
    endif()
    set(FIL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(FIL_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(FIL_ENABLE_IPO OFF CACHE BOOL "" FORCE)
    # FIL's top-level CMake otherwise replaces the source-root compile database.
    set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
    add_subdirectory("${PERSIM_FIL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/fil" EXCLUDE_FROM_ALL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    target_sources(persim_core PRIVATE src/fil_bridge.cpp)
    target_link_libraries(persim_core PRIVATE fil_core)
endif()
target_compile_definitions(persim_core PUBLIC PERSIM_WITH_FIL=$<BOOL:${PERSIM_WITH_FIL}>)
configure_file(include/persim/build_config.hpp.in generated/persim/build_config.hpp @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${PERSIM_MONOREPO_DIR}/generators/configs/system/bus_configs.json"
    "${PERSIM_MONOREPO_DIR}/generators/configs/nodes/DASHBOARD.json"
    "${PERSIM_MONOREPO_DIR}/generators/configs/nodes/MAIN_MODULE.json"
    "${PERSIM_MONOREPO_DIR}/generators/configs/nodes/TORQUE_VECTOR.json"
    "${PERSIM_MONOREPO_DIR}/generators/configs/nodes/DRIVELINE.json"
    tools/generate_network.py)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_network.py"
    "${PERSIM_MONOREPO_DIR}" "${PERSIM_FIL_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/generated"
    COMMAND_ERROR_IS_FATAL ANY)
add_library(persim_session src/session.cpp src/options.cpp src/telemetry.cpp)
target_link_libraries(persim_session PUBLIC persim_core)
target_compile_options(persim_session PRIVATE -Wall -Wextra -Wpedantic)
add_executable(persim-headless src/headless.cpp)
target_link_libraries(persim-headless PRIVATE persim_session)

if(PERSIM_BUILD_VIEWER)
    find_package(raylib 5.5 QUIET)
    if(NOT raylib_FOUND)
        include(FetchContent)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(raylib
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG c1ab645ca298a2801097931d1079b10ff7eb9df8
            GIT_SHALLOW TRUE)
        FetchContent_MakeAvailable(raylib)
    endif()
    add_executable(persim src/viewer.cpp)
    target_link_libraries(persim PRIVATE persim_session raylib)
    get_target_property(raylib_headers raylib INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(persim SYSTEM PRIVATE ${raylib_headers})
    target_compile_options(persim PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(persim-vcan guest/vcan_bridge.c)
    target_compile_options(persim-vcan PRIVATE -Wall -Wextra -Wpedantic)
endif()

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    configure_file(Doxyfile.in Doxyfile @ONLY)
    add_custom_target(docs
        COMMAND Doxygen::doxygen "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        COMMENT "Building the PERsim API and model reference" VERBATIM)
endif()
