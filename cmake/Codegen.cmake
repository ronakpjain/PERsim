# Generated inputs: SuperDBC protocol codec, suspension splines, FIL wiring,
# and build configuration. Everything is written under the build tree.
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

configure_file(include/persim/build_config.hpp.in generated/persim/build_config.hpp @ONLY)

# Board wiring is derived from the monorepo generator configs, not stale examples.
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
