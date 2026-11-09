/**
 * @file terrain.cpp
 * @brief Terrain-model registry and the built-in flat/proving-ground profiles.
 * @ingroup physics
 */
#include "persim/terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace persim {
namespace {
/** @brief Smooth half-cosine window with flat support and zero endpoints. */
double window(double v, double begin, double end, double edge) {
    if (v <= begin || v >= end) return 0.0;
    const double t = std::min({(v - begin) / edge, (end - v) / edge, 1.0});
    return 0.5 - 0.5 * std::cos(3.14159265358979323846 * t);
}
/** @brief Convert a scalar height field into a surface sample by finite differences.
 * @tparam Height Callable with signature double(double x, double y).
 * @param height Height field to sample, m.
 * @param x World X coordinate, m.
 * @param y World Y coordinate, m.
 * @param friction Dimensionless grip multiplier at the sample.
 */
template<class Height>
Surface sample_height(Height&& height, double x, double y, double friction) {
    constexpr double e = 0.001;
    const Eigen::Vector3d normal{-(height(x + e, y) - height(x - e, y)) / (2 * e),
                                 -(height(x, y + e) - height(x, y - e)) / (2 * e), 1.0};
    return {height(x, y), normal.normalized(), friction};
}
/** @brief Zero-elevation, unit-grip road. */
class FlatTerrain final : public TerrainModel {
public:
    std::string_view name() const override { return "flat"; }
    std::string_view summary() const override { return "Zero-elevation, unit-grip road"; }
    Surface sample(double, double) const override { return {}; }
};
/** @brief Analytic proving-ground bump, ramp, ripple, and low-grip patch. */
class ProvingGroundTerrain final : public TerrainModel {
public:
    std::string_view name() const override { return "proving-ground"; }
    std::string_view summary() const override { return "Bump strip, ramp, ripples, 45% grip patch"; }
    bool marked_course() const override { return true; }
    Surface sample(double x, double y) const override {
        const auto height = [](double px, double py) {
            const double strip = window(py, -14.0, -6.0, 1.0);
            const double bump = 0.065 * window(px, 18.0, 22.0, 2.0);
            const double ramp = 0.4 * window(px, 35.0, 53.0, 7.0);
            const double ripple = 0.025 * window(px, 58.0, 80.0, 2.0) * (1.0 + std::sin(px * 3.0));
            return strip * (bump + ramp + ripple);
        };
        const double grip = x > 25 && x < 45 && y > 6 && y < 14 ? 0.45 : 1.0;
        return sample_height(height, x, y, grip);
    }
};

/** @brief Compiled registry; add one row here to expose a new terrain profile. */
constexpr std::array<TerrainInfo, 2> registry{{
    {"proving-ground", "Bump strip, ramp, ripples, 45% grip patch", [] { return std::unique_ptr<TerrainModel>(new ProvingGroundTerrain); }},
    {"flat", "Zero-elevation, unit-grip road", [] { return std::unique_ptr<TerrainModel>(new FlatTerrain); }},
}};
}

std::span<const TerrainInfo> terrains() { return registry; }

const TerrainInfo* find_terrain(std::string_view name) {
    for (const auto& info : registry) if (info.name == name) return &info;
    return nullptr;
}

std::unique_ptr<TerrainModel> make_terrain(std::string_view name) {
    if (const auto* info = find_terrain(name)) return info->make();
    throw std::invalid_argument("Unknown terrain '" + std::string(name) + "'; expected one of " + terrain_names());
}

std::string terrain_names() {
    std::string names;
    for (const auto& info : registry) names += (names.empty() ? "" : ", ") + std::string(info.name);
    return names;
}
}
