#!/usr/bin/env python3
"""@file
@brief Export the sibling's actual suspension splines as native C++ coefficients.
@details Uses SuspensionKinematics.from_file, including its rotation-vector
conversion and wheel-order/unit validation. Does not refit or decimate the data.
Requires NumPy and SciPy only at build time; runtime does not embed Python.
"""
import hashlib
import importlib.util
import pathlib
import sys

import numpy as np


def main():
    """@brief Load source kinematics and export its tensor-product cubic B-splines."""
    source, output = map(pathlib.Path, sys.argv[1:3])
    module_path = source / "models/suspension/suspension_kinematics.py"
    table_path = source / "models/suspension/data/kinematics_table.npz"
    spec = importlib.util.spec_from_file_location("persim_source_suspension", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    kinematics = module.SuspensionKinematics.from_file(table_path)
    with np.load(table_path, allow_pickle=False) as data:
        rack = data["tie_offsets"].copy()
        reference = data["reference_chassis_center"].copy()
        centers = data["reference_wheel_centers"].copy()
        if np.max(data["max_constraint_residual"]) > 1e-8:
            raise ValueError("Suspension linkage residuals exceed 1e-8 m")
    # Source FR/FL/RR/RL -> AMK FR/FL/RL/RR. Each channel is one spline.
    curves = [curve for i in (0, 1, 3, 2) for curve in (
        *kinematics._positions[i], *kinematics._orientations[i], kinematics._lengths[i]
    )]
    travel_knots, rack_knots = curves[0].get_knots()
    for curve in curves:
        a, b = curve.get_knots()
        if not np.array_equal(a, travel_knots) or not np.array_equal(b, rack_knots):
            raise ValueError("Suspension spline channels must share knot vectors")
    steering = []
    for offset in rack:
        geometry = kinematics.evaluate(np.zeros(4), offset)
        axes = geometry.wheel_orientation[:2, :, 1]
        steering.append(np.mean(np.arctan2(-axes[:, 0], axes[:, 1])))
    if not np.all(np.diff(steering) > 0):
        raise ValueError("Mean front-wheel steering must increase monotonically with rack")
    digest = hashlib.sha256(table_path.read_bytes() + module_path.read_bytes()).hexdigest()
    lines = ["// Generated from modeling-and-simulation; do not edit.", "#pragma once",
             "#include <array>", "namespace persim::model_data {",
             f'inline constexpr char suspension_sha256[] = "{digest}";',
             f"inline constexpr int travel_coefficients = {len(travel_knots) - 4};",
             f"inline constexpr int rack_coefficients = {len(rack_knots) - 4};"]
    arrays = {
        "travel_knots": travel_knots,
        "rack_knots": rack_knots,
        "rack_samples": rack,
        "steering_samples": np.asarray(steering),
        "reference_chassis_center": reference,
        "reference_wheel_centers": centers.ravel(),
        "coefficients": np.concatenate([curve.get_coeffs() for curve in curves]),
    }
    for name, values in arrays.items():
        if not np.all(np.isfinite(values)):
            raise ValueError(f"Non-finite suspension data: {name}")
        lines.append(f"inline constexpr std::array<double, {len(values)}> {name}{{{{")
        for start in range(0, len(values), 6):
            lines.append("    " + ", ".join(format(float(x), ".17g") for x in values[start:start + 6]) + ",")
        lines.append("}};")
    lines.append("}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
