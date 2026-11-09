/**
 * @file tire.cpp
 * @brief Tire-model registry and adapters to the sibling tire components.
 * @ingroup physics
 */
#include "persim/tire.hpp"

#include <tires/linear.hpp>
#include <tires/reduced_mf.hpp>
#include <tires/tanh.hpp>

#include <array>
#include <stdexcept>
#include <string>

namespace persim {
namespace {
/**
 * @brief Expose the source Tire's protected friction coefficient without editing it.
 * @tparam Model Sibling tire implementation to wrap.
 */
template<class Model>
class RoadTire final : public Model, public TireModel {
public:
    using Model::Model;
    /** @brief Read the source tire's preset coefficient before any road override. */
    double default_coefficient() const { return this->mu; }
    /** @brief Apply the source combined-slip policy at the current local road grip. */
    Eigen::Vector2d force(double alpha, double kappa, double load, double grip) override {
        this->mu = grip;
        return siValue(this->getForce(angle(alpha), kappa, ::force(load)));
    }
};

/** @brief Construct the source reduced-MF tire with preset stiffness and shape. */
std::unique_ptr<TireModel> make_reduced_mf(const Parameters& p) {
    const ReducedMFParameters shape{corneringStiffness(p.cornering_stiffness), force(p.longitudinal_stiffness),
                                    p.tire_shape, p.longitudinal_tire_shape};
    return std::make_unique<RoadTire<ReducedMFTire>>(shape);
}
/** @brief Construct the source smooth tanh tire. */
std::unique_ptr<TireModel> make_tanh(const Parameters& p) {
    return std::make_unique<RoadTire<TanhTire>>(corneringStiffness(p.cornering_stiffness), force(p.longitudinal_stiffness));
}
/** @brief Construct the source linear tire with friction-circle saturation. */
std::unique_ptr<TireModel> make_linear(const Parameters& p) {
    return std::make_unique<RoadTire<LinearTire>>(corneringStiffness(p.cornering_stiffness), force(p.longitudinal_stiffness));
}

/** @brief Compiled registry; add one row here to expose a new tire model. */
constexpr std::array<TireModelInfo, 3> registry{{
    {"reduced-mf", "Reduced Magic Formula (source default)", make_reduced_mf},
    {"tanh", "Smooth saturating tangent model", make_tanh},
    {"linear", "Linear stiffness with friction-circle saturation", make_linear},
}};
}

std::span<const TireModelInfo> tire_models() { return registry; }

const TireModelInfo* find_tire_model(std::string_view name) {
    for (const auto& info : registry) if (info.name == name) return &info;
    return nullptr;
}

std::unique_ptr<TireModel> make_tire_model(std::string_view name, const Parameters& parameters) {
    if (const auto* info = find_tire_model(name)) return info->make(parameters);
    throw std::invalid_argument("Unknown tire model '" + std::string(name) + "'; expected one of " + tire_model_names());
}

std::string tire_model_names() {
    std::string names;
    for (const auto& info : registry) names += (names.empty() ? "" : ", ") + std::string(info.name);
    return names;
}

double source_tire_friction() {
    return RoadTire<ReducedMFTire>(DEFAULT_TIRE).default_coefficient();
}
}
