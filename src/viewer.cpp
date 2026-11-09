/**
 * @file viewer.cpp
 * @brief Raylib proving ground, procedural PER car, and asynchronous telemetry UI.
 * @details The render thread never owns FIL. SimulationWorker publishes value
 * snapshots, and its single simulation thread owns all plant, firmware, and CAN
 * state. Physics time is never discarded to maintain a target frame rate.
 */
#include "persim/options.hpp"
#include "persim/session.hpp"
#include "persim/telemetry.hpp"

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <thread>

namespace {
using namespace persim;
constexpr Color gold{221, 184, 103, 255}; ///< PER-inspired body/UI accent.
constexpr Color ink{16, 23, 30, 255}; ///< Dark panels and carbon proxy.
constexpr Color muted{139, 157, 169, 255}; ///< Secondary UI/metal color.
constexpr Color white{231, 237, 239, 255}; ///< Primary text and course paint.
constexpr Color asphalt{46, 54, 58, 255}; ///< Default road material.
constexpr Color green{122, 219, 158, 255}; ///< Enabled/healthy observation.
constexpr Color red{241, 119, 105, 255}; ///< Braking/fault observation.

/** @brief Convert ISO Z-up to raylib Y-up, preserving handedness. */
Vector3 render(const Vec3& p) { return {static_cast<float>(p.x()), static_cast<float>(p.z()), static_cast<float>(-p.y())}; }
/** @brief Apply the rigid-body transform to raylib-local (+X forward) geometry. */
void body_transform(const VehicleState& car) {
    const auto pos = render(car.position);
    rlTranslatef(pos.x, pos.y, pos.z);
    Eigen::Matrix3d c; c << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    const Eigen::Matrix3d rotation = c * car.orientation.toRotationMatrix() * c.transpose();
    float matrix[16]{};
    for (int column = 0; column < 3; ++column)
        for (int row = 0; row < 3; ++row) matrix[column * 4 + row] = static_cast<float>(rotation(row, column));
    matrix[15] = 1;
    rlMultMatrixf(matrix);
}
/** @brief One simulation owner with bounded snapshot/input critical sections. */
class SimulationWorker {
public:
    /** @brief Launch the single simulation owner with an explicit 8 MiB stack. */
    explicit SimulationWorker(Options options) : options_(std::move(options)) {
        // FIL's memory-map construction exceeds macOS's 512 KiB worker stack.
        pthread_attr_t attributes;
        if (pthread_attr_init(&attributes) != 0) throw std::runtime_error("Cannot initialize simulation thread attributes");
        const int stack_error = pthread_attr_setstacksize(&attributes, 8 * 1024 * 1024);
        const int start_error = stack_error ? stack_error : pthread_create(&worker_, &attributes, [](void* self) -> void* {
            static_cast<SimulationWorker*>(self)->run();
            return nullptr;
        }, this);
        pthread_attr_destroy(&attributes);
        if (start_error) throw std::runtime_error("Cannot start the 8 MiB simulation thread");
    }
    /** @brief Request stop and join before destroying shared snapshots. */
    ~SimulationWorker() { stop_ = true; pthread_join(worker_, nullptr); }
    /** @brief Copy the latest immutable observation under the publication mutex. */
    Snapshot snapshot() { std::lock_guard lock(mutex_); return latest_; }
    /** @brief Publish new driver input without touching the physical/FIL owner. */
    void input(DriverInput input) { std::lock_guard lock(mutex_); input_ = input; }
    /** @brief Toggle the shared simulated-time pause request. */
    void pause() { paused_ = !paused_.load(); }
    /** @brief Report whether simulated time is currently held. */
    bool paused() const { return paused_; }
    /** @brief Request a complete plant/firmware restart on the owner thread. */
    void reset() { reset_ = true; paused_ = false; }
private:
    void publish(const Snapshot& s) { std::lock_guard lock(mutex_); latest_ = s; }
    void run() {
        unsigned generation = 0;
        while (!stop_) {
            reset_ = false;
            try {
                Session simulation(options_);
                auto csv_path = options_.csv;
                if (generation > 0 && !csv_path.empty())
                    csv_path = csv_path.parent_path() / (csv_path.stem().string() + "-" + std::to_string(generation) + csv_path.extension().string());
                TelemetryWriter csv(csv_path);
                auto epoch = std::chrono::steady_clock::now();
                Snapshot state = simulation.snapshot();
                publish(state);
                while (!stop_ && !reset_) {
                    if (paused_) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        epoch = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(state.time));
                        continue;
                    }
                    DriverInput input;
                    { std::lock_guard lock(mutex_); input = input_; }
                    if (options_.scenario != "manual") input = scenario_input(options_.scenario, state.time);
                    simulation.step(input);
                    state = simulation.snapshot();
                    csv.write(state);
                    publish(state);
                    if (options_.duration > 0 && state.time >= options_.duration) {
                        if (options_.verify) simulation.verify();
                        paused_ = true;
                    }
                    std::this_thread::sleep_until(epoch + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(state.time)));
                }
                csv.finish();
            } catch (const std::exception& error) {
                auto state = snapshot(); state.error = error.what(); publish(state);
                while (!stop_ && !reset_) std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            ++generation;
        }
    }
    Options options_;
    std::mutex mutex_;
    DriverInput input_;
    Snapshot latest_;
    std::atomic<bool> paused_{false}, reset_{false}, stop_{false};
    pthread_t worker_{};
};

/** @brief Raylib-owned terrain mesh sampled from the exact physics height field. */
class Ground {
public:
    /** @brief Build a terrain mesh from physics height/normal samples. */
    explicit Ground(const TerrainModel& terrain) {
        mesh_ = GenMeshPlane(220, 120, 220, 120);
        for (int i = 0; i < mesh_.vertexCount; ++i) {
            const auto s = terrain.sample(mesh_.vertices[3 * i], -mesh_.vertices[3 * i + 2]);
            mesh_.vertices[3 * i + 1] = static_cast<float>(s.height);
            const auto n = render(s.normal);
            mesh_.normals[3 * i] = n.x; mesh_.normals[3 * i + 1] = n.y; mesh_.normals[3 * i + 2] = n.z;
        }
        UpdateMeshBuffer(mesh_, 0, mesh_.vertices, mesh_.vertexCount * 3 * sizeof(float), 0);
        UpdateMeshBuffer(mesh_, 2, mesh_.normals, mesh_.vertexCount * 3 * sizeof(float), 0);
        material_ = LoadMaterialDefault();
        material_.maps[MATERIAL_MAP_DIFFUSE].color = asphalt;
    }
    /** @brief Release the terrain mesh and raylib material. */
    ~Ground() { UnloadMesh(mesh_); UnloadMaterial(material_); }
    /** @brief Draw the terrain at the world origin. */
    void draw() const { DrawMesh(mesh_, material_, MatrixIdentity()); }
private:
    Mesh mesh_{};
    Material material_{};
};

/** @brief Draw an asphalt marking at the physical terrain elevation. */
void ground_line(Vec3 a, Vec3 b, const TerrainModel& terrain, Color color) {
    a.z() = terrain.sample(a.x(), a.y()).height + 0.008;
    b.z() = terrain.sample(b.x(), b.y()).height + 0.008;
    DrawLine3D(render(a), render(b), color);
}
/** @brief Non-collidable course cones, lane markings, skidpad, and grip patch. */
void draw_course(const Ground& ground, const Snapshot& state, const TerrainModel& terrain) {
    DrawPlane({0, -0.008f, 0}, {2000, 2000}, asphalt);
    ground.draw();
    const int grid_x = static_cast<int>(std::floor(state.car.position.x() / 5)) * 5;
    const int grid_y = static_cast<int>(std::floor(state.car.position.y() / 5)) * 5;
    for (int d = -60; d <= 60; d += 5) {
        ground_line(Vec3(grid_x + d, grid_y - 60, 0), Vec3(grid_x + d, grid_y + 60, 0), terrain, {58, 67, 69, 255});
        ground_line(Vec3(grid_x - 60, grid_y + d, 0), Vec3(grid_x + 60, grid_y + d, 0), terrain, {58, 67, 69, 255});
    }
    if (terrain.marked_course()) {
        DrawCube({35, 0.002f, -10}, 20, 0.004f, 8, {62, 78, 83, 255});
        for (double y : {-14.0, -6.0})
            for (double x = 14; x < 84; x += 0.5) ground_line(Vec3(x, y, 0), Vec3(x + 0.5, y, 0), terrain, gold);
    }
    for (double y : {-3.0, 3.0})
        for (double x = -10; x < 95; x += 2) ground_line(Vec3(x, y, 0), Vec3(x + 1, y, 0), terrain, {173, 177, 164, 255});
    for (int n = 0; n < 2; ++n) {
        const double radius = n == 0 ? 14 : 19;
        for (int i = 0; i < 180; ++i) {
            const double a = 2 * pi * i / 180, b = 2 * pi * (i + 1) / 180;
            ground_line(Vec3(35 + radius * std::cos(a), 30 + radius * std::sin(a), 0),
                        Vec3(35 + radius * std::cos(b), 30 + radius * std::sin(b), 0), terrain, {148, 155, 149, 255});
        }
    }
    for (int i = 0; i < 36; ++i) {
        const double x = -5 + std::floor(i / 2.0) * 6, y = (i % 2 == 0 ? -1 : 1) * 3.5;
        const auto p = render(Vec3(x, y, terrain.sample(x, y).height));
        DrawCube(Vector3Add(p, {0, 0.015f, 0}), 0.26f, 0.03f, 0.26f, {20, 25, 25, 255});
        DrawCylinder(Vector3Add(p, {0, 0.03f, 0}), 0.015f, 0.095f, 0.32f, 10, {239, 149, 66, 255});
        DrawCylinder(Vector3Add(p, {0, 0.16f, 0}), 0.044f, 0.062f, 0.065f, 10, white);
    }
    // Checkerboard start line, outside the raised terrain strip.
    for (int row = 0; row < 2; ++row) for (int column = 0; column < 12; ++column)
        DrawCube({-2 + row * 0.25f, 0.006f, -3 + column * 0.5f}, 0.25f, 0.006f, 0.5f,
                 (row + column) % 2 ? white : ink);
}
/** @brief Display-only spring coil joining an inboard mount to a moving hub. */
void spring(Vector3 a, Vector3 b) {
    DrawCylinderEx(a, b, 0.015f, 0.015f, 6, {178, 187, 189, 255});
    const Vector3 direction = Vector3Normalize(Vector3Subtract(b, a));
    Vector3 side = Vector3CrossProduct(direction, {0, 0, 1});
    if (Vector3Length(side) < 0.01f) side = Vector3CrossProduct(direction, {0, 1, 0});
    side = Vector3Normalize(side);
    const Vector3 other = Vector3CrossProduct(direction, side);
    Vector3 previous = a;
    for (int i = 1; i <= 48; ++i) {
        const float t = i / 48.0f, theta = t * 8 * 2 * PI;
        Vector3 point = Vector3Lerp(a, b, t);
        point = Vector3Add(point, Vector3Scale(side, 0.025f * std::cos(theta)));
        point = Vector3Add(point, Vector3Scale(other, 0.025f * std::sin(theta)));
        DrawCylinderEx(previous, point, 0.006f, 0.006f, 5, gold);
        previous = point;
    }
}
/** @brief Procedural editable Formula-style proxy, not PER CAD or collision geometry. */
void draw_car(const Snapshot& s, bool forces) {
    const Parameters p;
    const auto at = [&](const Vec3& local) { return render(s.car.position + s.car.orientation * local); };
    const auto pos = render(s.car.position);
    DrawCylinder({pos.x, 0.012f, pos.z}, 0.85f, 0.85f, 0.001f, 36, {26, 32, 34, 255});
    rlPushMatrix(); body_transform(s.car);
    DrawCube({-0.07f, -0.16f, 0}, 1.9f, 0.06f, 0.8f, {22, 28, 30, 255});
    DrawCube({-0.35f, -0.02f, 0}, 1.13f, 0.28f, 0.58f, {30, 36, 39, 255});
    DrawCube({0.53f, -0.04f, 0}, 0.90f, 0.18f, 0.36f, gold);
    DrawCube({1.02f, -0.09f, 0}, 0.35f, 0.13f, 0.23f, gold);
    DrawCube({-0.31f, -0.04f, -0.39f}, 0.9f, 0.19f, 0.18f, gold);
    DrawCube({-0.31f, -0.04f, 0.39f}, 0.9f, 0.19f, 0.18f, gold);
    DrawCube({0.57f, 0.057f, 0}, 0.65f, 0.008f, 0.06f, ink);
    for (float y : {-0.14f, -0.08f}) {
        DrawCube({1.12f, y, 0}, 0.24f, 0.022f, 1.48f, {30, 36, 38, 255});
        DrawCube({-1.02f, y + 0.62f, 0}, 0.28f, 0.025f, 1.25f, gold);
    }
    for (float z : {-0.74f, 0.74f}) DrawCube({1.12f, -0.07f, z}, 0.36f, 0.20f, 0.025f, gold);
    for (float z : {-0.62f, 0.62f}) DrawCube({-1.02f, 0.48f, z}, 0.36f, 0.22f, 0.025f, ink);
    for (float z : {-0.22f, 0.22f}) {
        DrawCylinderEx({-0.77f, 0.04f, z}, {-0.80f, 0.51f, z}, 0.018f, 0.018f, 8, muted);
        DrawCylinderEx({-0.80f, 0.51f, z}, {-0.52f, 0.10f, z}, 0.018f, 0.018f, 8, muted);
    }
    DrawCylinderEx({-0.80f, 0.51f, -0.22f}, {-0.80f, 0.51f, 0.22f}, 0.018f, 0.018f, 8, muted);
    DrawCube({-0.18f, 0.13f, 0}, 0.35f, 0.20f, 0.35f, ink);
    DrawSphere({-0.29f, 0.32f, 0}, 0.135f, gold);
    DrawCube({-0.17f, 0.34f, 0}, 0.015f, 0.05f, 0.17f, {26, 41, 48, 255});
    rlPopMatrix();
    for (int i = 0; i < 4; ++i) {
        const Vec3 hub = s.telemetry.wheels[i].hub;
        const Quat wheel_orientation = s.car.orientation * s.telemetry.wheels[i].orientation;
        const Vec3 axle = wheel_orientation * Vec3::UnitY();
        const auto a = render(hub - axle * 0.105), b = render(hub + axle * 0.105);
        DrawCylinderEx(a, b, p.radius, p.radius, 24, {17, 23, 25, 255});
        for (double side : {-1.0, 1.0}) {
            const Vec3 face = hub + axle * (0.108 * side);
            DrawCylinderEx(render(face), render(face + axle * (0.004 * side)), 0.115f, 0.115f, 16, {105, 119, 126, 255});
            for (int spoke = 0; spoke < 5; ++spoke) {
                const double angle = s.car.wheels[i].phase + spoke * 2 * pi / 5;
                const Vec3 radial = wheel_orientation * Vec3(std::cos(angle), 0, std::sin(angle));
                DrawCylinderEx(render(face), render(face + radial * 0.105), 0.009f, 0.009f, 6, gold);
            }
        }
        for (double level : {-0.13, 0.06}) for (double dx : {-0.18, 0.18}) {
            const Vec3 root(p.hub_reference(i).x() + dx, p.hub_reference(i).y() * 0.42, level);
            DrawCylinderEx(at(root), render(hub + s.car.orientation * Vec3(0, 0, level * 0.4)), 0.012f, 0.012f, 6, muted);
        }
        spring(at(Vec3(p.hub_reference(i).x() - 0.15, p.hub_reference(i).y() * 0.4, 0.16)),
               render(hub + s.car.orientation * Vec3(0, 0, 0.06)));
        if (forces) {
            const auto& t = s.telemetry.wheels[i];
            DrawCylinderEx(render(t.contact), render(t.contact + t.force * 0.0006), 0.012f, 0.004f, 8, green);
        }
    }
}
/** @brief Draw a labeled UI bar with an explicitly bounded normalized fill. */
void bar(int x, int y, int width, float fraction, Color color) {
    DrawRectangle(x, y, width, 5, {53, 65, 73, 255});
    DrawRectangle(x, y, static_cast<int>(width * std::clamp(fraction, 0.0f, 1.0f)), 5, color);
}
/** @brief Live plant/controller panel, including safe startup and errors. */
void hud(const Snapshot& s, bool paused, bool show_forces, bool external_tv) {
    const int height = GetScreenHeight(), x = GetScreenWidth() - 300;
    DrawRectangle(0, 0, GetScreenWidth(), 74, {16, 23, 30, 245});
    DrawRectangle(26, 22, 4, 29, gold);
    DrawText("PER / SIM", 44, 18, 29, white);
    DrawText("VEHICLE DYNAMICS LAB", 46, 51, 11, muted);
    DrawText(s.standalone ? "STANDALONE PLANT" : "FIRMWARE IN THE LOOP", 283, 22, 15, gold);
    DrawText(external_tv ? "CM5 TV  /  WALL-CLOCK CAN" : "PER27  /  AMK AWD  /  14 DOF", 283, 45, 12, muted);
    DrawRectangle(x, 90, 282, height - 108, {16, 23, 30, 239});
    DrawText("LIVE TELEMETRY", x + 18, 109, 15, gold);
    DrawText(TextFormat("%03.0f", s.car.velocity.norm() * 3.6), x + 17, 137, 63, white);
    DrawText("km/h", x + 159, 171, 17, muted);
    DrawText(car_state_name(s.firmware.car_state), x + 18, 211, 18, s.firmware.car_state == 5 ? green : gold);
    DrawText(TextFormat("t  %.2f s     %.2fx realtime", s.time, s.realtime_factor), x + 18, 240, 13, muted);
    DrawText("THROTTLE", x + 18, 273, 12, muted); bar(x + 101, 277, 158, s.driver.throttle, green);
    DrawText("BRAKE", x + 18, 296, 12, muted); bar(x + 101, 300, 158, s.driver.brake, red);
    DrawText(TextFormat("STEER     %+5.1f deg", s.car.steering * 180 / pi), x + 18, 324, 14, white);
    DrawLine(x + 18, 355, x + 263, 355, {55, 66, 75, 255});
    DrawText("WHEEL    LOAD      MOTOR     AMK", x + 18, 371, 12, muted);
    for (int i = 0; i < 4; ++i) {
        const int y = 401 + i * 48;
        DrawText(wheel_names[i], x + 18, y, 16, gold);
        DrawText(TextFormat("%4.0f N", s.telemetry.wheels[i].normal_load), x + 63, y, 14, white);
        DrawText(TextFormat("%+4.1f Nm", s.inverters[i].torque), x + 139, y, 14, white);
        DrawCircle(x + 253, y + 6, 4, s.inverters[i].enabled ? green : muted);
        bar(x + 63, y + 24, 147, s.telemetry.wheels[i].normal_load / 2000.0f, gold);
        DrawText(TextFormat("%2.0f%%", std::abs(s.telemetry.wheels[i].slip_ratio) * 100), x + 226, y + 20, 11,
                 std::abs(s.telemetry.wheels[i].slip_ratio) > 0.2 ? red : muted);
    }
    if (height > 730) {
        DrawLine(x + 18, 604, x + 263, 604, {55, 66, 75, 255});
        DrawText(TextFormat("PACK   %.1f V   %+.1f A", s.battery.voltage, s.battery.current), x + 18, 622, 14, white);
        DrawText(TextFormat("SOC    %.1f%%    AERO %.0f N", s.battery.state_of_charge * 100, s.telemetry.downforce), x + 18, 647, 13, muted);
        DrawText(TextFormat("CAN %llu     TV %s", static_cast<unsigned long long>(s.firmware.transmitted_frames),
                 s.firmware.tv_enabled ? "ON" : "OFF"), x + 18, 676, 13, s.firmware.tv_enabled ? green : muted);
        if (external_tv) DrawText(s.firmware.external_connected ? "CM5 CONNECTED" : "WAITING FOR CM5", x + 18, 703, 12, gold);
    }
    DrawRectangle(20, height - 62, std::max(600, x - 38), 42, {16, 23, 30, 220});
    DrawText("W / S  pedals     A / D  steer     ENTER  start     R  reset", 35, height - 53, 13, white);
    DrawText("C  camera    RMB  orbit    wheel  zoom    P  pause    T  TV    F  forces", 35, height - 33, 11, muted);
    DrawText(show_forces ? "TIRE FORCE VECTORS" : "PROVING GROUND /  LOW-GRIP + BUMP LANES", 26, 94, 12, gold);
    if (paused) DrawText("PAUSED", GetScreenWidth() / 2 - 75, 100, 28, gold);
    if (s.time < 0.001 && s.error.empty()) DrawText("LOADING PLANT / FIRMWARE...", 28, 126, 18, white);
    if (!s.error.empty()) {
        DrawRectangle(20, 120, std::max(500, x - 40), 95, {70, 28, 29, 245});
        DrawText("SIMULATION STOPPED / press R to retry", 34, 133, 18, red);
        DrawText(s.error.substr(0, 100).c_str(), 34, 165, 13, white);
    }
}
/** @brief Slew one normalized keyboard control toward its demand per real second. */
double approach(double value, double target, double rate, double dt) {
    return value + std::clamp(target - value, -rate * dt, rate * dt);
}
/** @brief Sample progressive keyboard controls and unassisted analog gamepad axes.
 * @details Keyboard steering uses a kinematic 0.85 g cornering-demand envelope
 * based on speed and modeled downforce; it does not add tire force or change
 * physical steering lock. Analog axes and scripted scenarios bypass this aid.
 */
DriverInput read_driver(const Snapshot& state, DriverInput& keyboard, bool tv) {
    const Parameters p;
    const double dt = std::clamp(static_cast<double>(GetFrameTime()), 0.0, 0.05);
    const bool throttle = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP);
    keyboard.throttle = approach(keyboard.throttle, throttle ? 1.0 : 0.0, throttle ? 1.6 : 5.0, dt);
    keyboard.brake = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_SPACE) ? 1 : 0;
    const int direction = (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1 : 0)
                        - (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1 : 0);
    const double speed_squared = state.car.velocity.head<2>().squaredNorm();
    const double cornering_acceleration = 0.85 * p.friction * (gravity + state.telemetry.downforce / p.mass);
    const double steering_limit = std::min(1.0,
        std::atan(p.wheelbase * cornering_acceleration / std::max(speed_squared, 1.0)) / p.max_steering);
    keyboard.steering = approach(keyboard.steering, direction * steering_limit, direction ? 2.5 : 3.5, dt);
    DriverInput input = keyboard;
    if (IsGamepadAvailable(0)) {
        const float axis = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        if (std::abs(axis) > 0.08) input.steering = -axis;
        if (GetGamepadAxisCount(0) > GAMEPAD_AXIS_RIGHT_TRIGGER) {
            input.throttle = std::max(input.throttle, (GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER) + 1) / 2.0);
            input.brake = std::max(input.brake, (GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER) + 1) / 2.0);
        }
    }
    input.start = IsKeyDown(KEY_ENTER);
    input.tv_enabled = tv;
    return input;
}
/** @brief Create the raylib frontend and join its simulation owner before releasing graphics resources. */
int run(Options options) {
    if (options.help) { std::cout << usage(); return 0; }
    if (options.list_models) { std::cout << model_catalog(); return 0; }
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1440, 900, "PER / SIM - Vehicle dynamics lab");
    if (!IsWindowReady()) throw std::runtime_error("Raylib window initialization failed");
    SetWindowMinSize(1040, 750);
    SetTargetFPS(60);
    int result = 0;
    {
        const auto terrain = make_terrain(options.terrain);
        Ground ground(*terrain);
        SimulationWorker worker(options);
        Camera3D camera{{-6, 2.5f, 0}, {0, 0.4f, 0}, {0, 1, 0}, 48, CAMERA_PERSPECTIVE};
        int camera_mode = 0, frame = 0;
        float orbit = 2.8f, pitch = 0.5f, distance = 6;
        bool forces = false, tv = true;
        std::deque<Vector3> trail;
        Snapshot state;
        DriverInput keyboard;
        while (!WindowShouldClose()) {
            if (IsKeyPressed(KEY_R)) { worker.reset(); trail.clear(); keyboard = {}; }
            if (IsKeyPressed(KEY_P)) worker.pause();
            if (IsKeyPressed(KEY_C)) camera_mode = (camera_mode + 1) % 3;
            if (IsKeyPressed(KEY_F)) forces = !forces;
            if (IsKeyPressed(KEY_T)) tv = !tv;
            state = worker.snapshot();
            worker.input(read_driver(state, keyboard, tv));
            const auto target = render(state.car.position + Vec3(0, 0, 0.12));
            const Vec3 heading = state.car.orientation * Vec3::UnitX();
            const double yaw = std::atan2(heading.y(), heading.x());
            distance = std::clamp(distance - GetMouseWheelMove() * 0.5f, 3.0f, 25.0f);
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                camera_mode = 1;
                const auto delta = GetMouseDelta(); orbit -= delta.x * 0.006f; pitch = std::clamp(pitch + delta.y * 0.004f, 0.08f, 1.4f);
            }
            if (camera_mode == 0) {
                // Follow heading, not velocity: world-space smoothing drifts sideways in turns.
                camera.position = Vector3Add(target, render(Vec3(-distance * std::cos(yaw),
                                                                -distance * std::sin(yaw), distance * 0.35)));
                camera.target = target;
            } else {
                const Vector3 desired = camera_mode == 1
                    ? Vector3Add(target, {distance * std::cos(pitch) * std::cos(orbit), distance * std::sin(pitch), distance * std::cos(pitch) * std::sin(orbit)})
                    : Vector3Add(target, {-0.01f, distance * 3, 0});
                const float blend = 1 - std::exp(-6 * std::min(GetFrameTime(), 0.1f));
                camera.position = Vector3Lerp(camera.position, desired, blend);
                camera.target = Vector3Lerp(camera.target, target, blend);
            }
            const auto pos = render(state.car.position - Vec3(0, 0, state.car.position.z() - 0.015));
            if (trail.empty() || Vector3Distance(trail.back(), pos) > 0.25f) trail.push_back(pos);
            if (trail.size() > 4000) trail.pop_front();
            BeginDrawing();
            ClearBackground({119, 139, 148, 255});
            DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), {76, 99, 118, 255}, {179, 192, 192, 255});
            BeginMode3D(camera);
            draw_course(ground, state, *terrain);
            for (std::size_t i = 1; i < trail.size(); ++i) DrawLine3D(trail[i - 1], trail[i], {170, 147, 92, 140});
            draw_car(state, forces);
            EndMode3D();
            hud(state, worker.paused(), forces, options.external_tv);
            DrawFPS(GetScreenWidth() - 94, 28);
            const bool finish = (options.frames > 0 && frame + 1 >= options.frames)
                             || (options.duration > 0 && state.time >= options.duration);
            EndDrawing();
            if (finish && !options.screenshot.empty()) {
                const Image image = LoadImageFromScreen();
                const bool saved = ExportImage(image, options.screenshot.c_str());
                UnloadImage(image);
                if (!saved) throw std::runtime_error("Cannot export screenshot: " + options.screenshot.string());
            }
            ++frame;
            if (finish) break;
        }
        if (!state.error.empty()) { std::cerr << "PERsim: " << state.error << '\n'; result = 1; }
    }
    CloseWindow();
    return result;
}
}
/** @brief Parse the viewer CLI and turn external failures into a nonzero process exit. */
int main(int argc, char** argv) {
    try { return run(persim::parse_options(argc, argv)); }
    catch (const std::exception& e) { std::cerr << "PERsim: " << e.what() << '\n'; return 1; }
}
