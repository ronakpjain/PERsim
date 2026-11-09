/** @file headless.cpp
 * @brief Display-free firmware/plant runner with scenario verification.
 * @ingroup frontend
 */
#include "persim/options.hpp"
#include "persim/session.hpp"
#include "persim/telemetry.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
namespace {
using namespace persim;
/** @brief Execute a bounded scenario, stream telemetry, and optionally verify outcomes. */
int run(Options options) {
    if (options.help) { std::cout << usage(); return 0; }
    if (options.duration == 0) options.duration = 8;
    if (options.scenario == "manual") options.scenario = "acceleration";
    std::cout << (options.standalone ? "STANDALONE" : "FIL CLOSED LOOP") << " | "
        << (options.standalone ? "NO FIRMWARE" : options.external_tv ? "CM5 TV (wall-clock)" : "STM32 TV")
        << " | " << options.scenario << std::endl;
    Session session(options);
    TelemetryWriter csv(options.csv);
    const auto start = std::chrono::steady_clock::now();
    int previous = -1;
    double next_report = 0;
    Snapshot state = session.snapshot();
    while (state.time < options.duration) {
        session.step(scenario_input(options.scenario, state.time));
        state = session.snapshot();
        csv.write(state);
        if (state.firmware.car_state != previous || state.time >= next_report) {
            std::cout << std::fixed << std::setprecision(3) << state.time << "s "
                << car_state_name(state.firmware.car_state) << " | " << state.car.velocity.norm() * 3.6
                << " km/h | " << state.motor_commands << " AMK commands | TV " << state.firmware.tv_requests
                << " | " << state.realtime_factor << "x" << std::endl;
            previous = state.firmware.car_state;
            next_report = state.time + 1;
        }
        if (options.realtime)
            std::this_thread::sleep_until(start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(state.time)));
    }
    csv.finish();
    if (options.verify) session.verify();
    std::cout << "Completed: peak torque " << state.peak_torque << " Nm, peak speed "
              << state.peak_speed * 3.6 << " km/h, position "
              << state.car.position.transpose() << (options.verify ? " | VERIFIED\n" : "\n");
    return 0;
}
}
/** @brief Parse the headless CLI and turn exceptions into a nonzero process exit. */
int main(int argc, char** argv) {
    try { return run(persim::parse_options(argc, argv)); }
    catch (const std::exception& e) { std::cerr << "PERsim: " << e.what() << '\n'; return 1; }
}
