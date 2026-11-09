/** @file options.cpp
 * @brief Validated CLI options and deterministic driver scenarios.
 * @ingroup frontend
 */
#include "persim/options.hpp"
#include "persim/build_config.hpp"
#include "persim/terrain.hpp"
#include "persim/tire.hpp"
#include <cmath>
#include <stdexcept>
#include <string_view>
namespace persim {
Options parse_options(int argc, char** argv) {
    Options result;
    result.network = default_network;
    const auto number = [](const std::string& s) {
        std::size_t end = 0;
        const double n = std::stod(s, &end);
        if (end != s.size() || !std::isfinite(n)) throw std::invalid_argument("Expected a finite number: " + s);
        return n;
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        const auto value = [&]() -> std::string {
            if (++i >= argc) throw std::invalid_argument("Missing value for " + std::string(arg));
            return argv[i];
        };
        if (arg == "--standalone") result.standalone = true;
        else if (arg == "--external-tv") result.external_tv = true;
        else if (arg == "--manual-start") result.auto_start = false;
        else if (arg == "--verify") result.verify = true;
        else if (arg == "--realtime") result.realtime = true;
        else if (arg == "--flat") result.terrain = "flat";
        else if (arg == "--sensor-noise") result.parameters.sensor_noise = true;
        else if (arg == "--tire") {
            result.parameters.tire = value();
            if (!find_tire_model(result.parameters.tire))
                throw std::invalid_argument("Unknown tire model; expected one of " + tire_model_names());
        }
        else if (arg == "--terrain") {
            result.terrain = value();
            if (!find_terrain(result.terrain))
                throw std::invalid_argument("Unknown terrain; expected one of " + terrain_names());
        }
        else if (arg == "--list-models") result.list_models = true;
        else if (arg == "--help" || arg == "-h") result.help = true;
        else if (arg == "--network") result.network = value();
        else if (arg == "--csv") result.csv = value();
        else if (arg == "--screenshot") result.screenshot = value();
        else if (arg == "--scenario") {
            result.scenario = value();
            if (result.scenario != "manual" && result.scenario != "idle" && result.scenario != "acceleration"
                && result.scenario != "circle" && result.scenario != "brake" && result.scenario != "bumps")
                throw std::invalid_argument("Unknown driving scenario");
        } else if (arg == "--duration") {
            result.duration = number(value());
            if (result.duration <= 0 || result.duration > 3600) throw std::invalid_argument("--duration must be in (0, 3600]");
        } else if (arg == "--tv-port") {
            const double n = number(value());
            if (n < 1 || n > 65535 || std::floor(n) != n) throw std::invalid_argument("--tv-port must be an integer in [1,65535]");
            result.tv_port = static_cast<std::uint16_t>(n);
            result.external_tv = true;
        } else if (arg == "--frames") {
            const double n = number(value());
            if (n < 1 || n > 1000000 || std::floor(n) != n) throw std::invalid_argument("--frames must be a positive integer");
            result.frames = static_cast<int>(n);
        } else throw std::invalid_argument("Unknown option: " + std::string(arg));
    }
    if (result.standalone && result.external_tv) throw std::invalid_argument("External TV requires the FIL backend");
    if (result.external_tv) result.realtime = true;
#if !PERSIM_WITH_FIL
    if (!result.standalone && !result.help) throw std::invalid_argument("FIL was disabled at build time; use --standalone");
#endif
    return result;
}
std::string usage() {
    return
        "PERsim: 3D PER27 vehicle plant + AMK devices + FIL firmware\n"
        "  --standalone           Playground without firmware (never a FIL fallback)\n"
        "  --network PATH         FIL network, no scripted stimuli or a_box\n"
        "  --external-tv          Replace STM32 TV with a CM5/SocketCAN TCP peer\n"
        "  --tv-port PORT         Localhost CAN bridge port (default 29500)\n"
        "  --manual-start         Disable the automatic brake/start sequence\n"
        "  --scenario NAME        manual, idle, acceleration, circle, brake, bumps\n"
        "  --duration SECONDS     Stop at simulated time (headless default: 8)\n"
        "  --terrain NAME         Road profile: " + terrain_names() + " (default proving-ground)\n"
        "  --flat                 Shorthand for --terrain flat\n"
        "  --tire NAME            Tire model: " + tire_model_names() + " (default reduced-mf)\n"
        "  --sensor-noise         Enable seeded source-model IMU/GPS sample noise\n"
        "  --csv PATH             Record vehicle and firmware telemetry\n"
        "  --verify               Fail if the selected closed-loop scenario did not work\n"
        "  --realtime             Pace headless execution to wall time\n"
        "  --frames N             Viewer frame limit for bounded capture\n"
        "  --screenshot PATH      Save the viewer's final frame\n"
        "  --list-models          List tire and terrain models\n"
        "  --help\n";
}
std::string model_catalog() {
    std::string out = "Tire models (--tire):\n";
    for (const auto& tire : tire_models())
        out += "  " + std::string(tire.name) + "  " + std::string(tire.summary) + "\n";
    out += "Terrain profiles (--terrain):\n";
    for (const auto& terrain : terrains())
        out += "  " + std::string(terrain.name) + "  " + std::string(terrain.summary) + "\n";
    return out;
}
DriverInput scenario_input(const std::string& scenario, double time) {
    DriverInput input;
    if (scenario == "acceleration" || scenario == "bumps") input.throttle = 0.45;
    if (scenario == "circle") { input.throttle = 0.3; input.steering = time > 5 ? 0.3 : 0; }
    if (scenario == "brake") {
        input.throttle = time < 6 ? 0.5 : 0;
        input.brake = time >= 6 ? 0.85 : 0;
    }
    return input;
}
}
