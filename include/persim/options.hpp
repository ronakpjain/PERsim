/** @file options.hpp
 * @brief Explicit backend selection, CLI validation, and headless driver scripts.
 * @ingroup frontend
 */
#pragma once
#include "persim/vehicle.hpp"
#include <cstdint>
#include <filesystem>
#include <string>
namespace persim {
/** @brief Runtime options shared by the interactive and headless frontends. */
struct Options {
    Parameters parameters;     ///< Source component presets and explicit runtime selections.
    bool standalone = false;    ///< Explicit pedal-map controller instead of FIL.
    bool external_tv = false;   ///< Replace STM32 TV with a single CM5 TCP peer.
    bool auto_start = true;     ///< Apply the simulated brake/start/buzzer sequence.
    bool verify = false;        ///< Require scenario success rather than finite state alone.
    bool realtime = false;      ///< Pace headless execution; always enabled for external TV.
    bool help = false;          ///< Print usage without loading the plant.
    double duration = 0;        ///< Simulated seconds; zero means viewer unlimited/headless default.
    std::uint16_t tv_port = 29500; ///< Localhost CAN gateway port.
    std::filesystem::path network; ///< FIL network JSON; defaults to generated current wiring.
    std::filesystem::path csv;    ///< Optional telemetry output.
    std::filesystem::path screenshot; ///< Optional final viewer-frame image.
    int frames = 0;             ///< Optional viewer frame limit; zero disables it.
    std::string scenario = "manual"; ///< Driver script name or interactive manual controls.
    TerrainKind terrain = TerrainKind::proving_ground; ///< Shared physics/display road.
};
/** @brief Parse and validate arguments, without loading images or opening sockets.
 * @throws std::invalid_argument For unknown, nonfinite, out-of-range, or incompatible options.
 */
Options parse_options(int argc, char** argv);
/** @brief Return the common command-line help text. */
std::string usage();
/** @brief Evaluate a deterministic normalized driver script at simulated time in seconds. */
DriverInput scenario_input(const std::string& scenario, double time);
}
