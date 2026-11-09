/**
 * @file tire.hpp
 * @brief Selectable tangential tire models and the name-based model registry.
 * @ingroup physics
 */
#pragma once
#include "persim/parameters.hpp"
#include <Eigen/Core>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace persim {
/**
 * @brief One tangent-plane tire force producer, expressed in SI doubles.
 * @details Implementations wrap a component from
 * [modeling-and-simulation/models/tires](https://github.com/PurdueElectricRacing/modeling-and-simulation/tree/main/models/tires).
 * The plant owns one instance and calls force() once per wheel and substep.
 */
class TireModel {
public:
    virtual ~TireModel() = default;
    /**
     * @brief Evaluate combined longitudinal and lateral force.
     * @param slip_angle Tire slip angle, rad.
     * @param slip_ratio Dimensionless longitudinal slip.
     * @param load Unilateral normal load, N.
     * @param road_grip Dimensionless friction multiplier, including Parameters::friction.
     * @return Tire-plane Fx then Fy, N, after the model's friction-circle saturation.
     */
    virtual Eigen::Vector2d force(double slip_angle, double slip_ratio, double load, double road_grip) = 0;
};

/** @brief Signature of a registered tire-model factory. */
using TireFactory = std::unique_ptr<TireModel> (*)(const Parameters&);

/** @brief One entry in the compiled tire-model registry. */
struct TireModelInfo {
    std::string_view name;    ///< CLI name used by `--tire`.
    std::string_view summary; ///< One-line description for `--list-models`.
    TireFactory make;         ///< Factory for this model.
};

/** @brief All registered tire models in display order. */
std::span<const TireModelInfo> tire_models();
/** @brief Find a registry entry by CLI name, or nullptr when unregistered. */
const TireModelInfo* find_tire_model(std::string_view name);
/** @brief Construct a registered tire model.
 * @param name Registry name, as listed by tire_models().
 * @param parameters Plant parameters supplied to the factory.
 * @throws std::invalid_argument If the name is not registered.
 */
std::unique_ptr<TireModel> make_tire_model(std::string_view name, const Parameters& parameters);
/** @brief Comma-separated registered names, for help and diagnostic text. */
std::string tire_model_names();
/** @brief Default friction coefficient of the sibling tire preset, before any override. */
double source_tire_friction();
}
