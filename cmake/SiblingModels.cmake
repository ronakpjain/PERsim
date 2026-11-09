# Compile the sibling's components directly, without its Python bindings or demos.
# Link target: persim_models (rigid-body motion and the AMK geared motor).
set(PERSIM_MODELING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../modeling-and-simulation"
    CACHE PATH "PER vehicle component model source checkout")
if(NOT EXISTS "${PERSIM_MODELING_DIR}/models/vehicle/vehicle.hpp")
    message(FATAL_ERROR
        "Set PERSIM_MODELING_DIR to the modeling-and-simulation checkout "
        "(default: ${CMAKE_CURRENT_SOURCE_DIR}/../modeling-and-simulation)")
endif()

# Match the dependency revisions used by modeling-and-simulation/CMakeLists.txt.
include(FetchContent)
FetchContent_Declare(gsl_lite
    URL https://github.com/gsl-lite/gsl-lite/archive/56dab5ce071c4ca17d3e0dbbda9a94bd5a1cbca1.tar.gz
    URL_HASH SHA256=ec430989e35d99dd2082b5d006f6f36035209fd55b79dae6305f4e64bc25910d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(gsl_lite)
set(MP_UNITS_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(mp_units
    URL https://github.com/mpusz/mp-units/archive/4bd850a4dbe35aabd91d814f859c70e2fc6df871.tar.gz
    URL_HASH SHA256=c8604a1be64adbf5b149cb8fb095ee72fa4ac2ff19d4d73c9f8c21bca9898094
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR src)
FetchContent_MakeAvailable(mp_units)

add_library(persim_models STATIC
    "${PERSIM_MODELING_DIR}/models/dynamics/rigid_body.cpp"
    "${PERSIM_MODELING_DIR}/models/powertrain/geared_motor.cpp")
target_compile_features(persim_models PUBLIC cxx_std_20)
target_include_directories(persim_models SYSTEM PUBLIC "${PERSIM_MODELING_DIR}/models")
target_link_libraries(persim_models PUBLIC Eigen3::Eigen mp-units::mp-units mp-units::integrations)
