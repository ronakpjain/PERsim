/**
 * @file terrain.hpp
 * @brief Selectable road profiles and the name-based terrain registry.
 * @ingroup physics
 */
#pragma once
#include <Eigen/Core>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace persim {
/** @brief Local tangent plane and friction multiplier for one road sample. */
struct Surface {
    double height = 0.0;             ///< World Z at the sampled X,Y, m.
    Eigen::Vector3d normal{0, 0, 1}; ///< Upward unit normal in world coordinates.
    double friction = 1.0;           ///< Dimensionless multiplier on Parameters::friction.
};

/**
 * @brief One road height/grip profile, sampled by both physics and rendering.
 * @details Implementations are stateless and deterministic. Add a profile by
 * deriving from this class and adding one row to the registry in src/terrain.cpp.
 */
class TerrainModel {
public:
    virtual ~TerrainModel() = default;
    /** @brief CLI name used by `--terrain`. */
    virtual std::string_view name() const = 0;
    /** @brief One-line description for `--list-models`. */
    virtual std::string_view summary() const = 0;
    /** @brief Sample the road at a world X,Y position.
     * @param x World X coordinate, m.
     * @param y World Y coordinate, m.
     * @return Elevation, unit normal, and grip; the road has no lateral boundaries.
     */
    virtual Surface sample(double x, double y) const = 0;
    /** @brief Whether the viewer should paint the fixed proving-ground course. */
    virtual bool marked_course() const { return false; }
};

/** @brief Signature of a registered terrain-model factory. */
using TerrainFactory = std::unique_ptr<TerrainModel> (*)();

/** @brief One entry in the compiled terrain-model registry. */
struct TerrainInfo {
    std::string_view name;    ///< CLI name used by `--terrain`.
    std::string_view summary; ///< One-line description for `--list-models`.
    TerrainFactory make;      ///< Factory for this profile.
};

/** @brief All registered terrain profiles in display order. */
std::span<const TerrainInfo> terrains();
/** @brief Find a registry entry by CLI name, or nullptr when unregistered. */
const TerrainInfo* find_terrain(std::string_view name);
/** @brief Construct a registered terrain profile.
 * @throws std::invalid_argument If the name is not registered.
 */
std::unique_ptr<TerrainModel> make_terrain(std::string_view name);
/** @brief Comma-separated registered names, for help and diagnostic text. */
std::string terrain_names();
}
