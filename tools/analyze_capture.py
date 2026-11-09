#!/usr/bin/env python3
"""@file
@brief Inspect historical PER seven-header-row CAN CSV captures without modifying them.
@details Offline data analysis, not a simulator test suite or automatic parameter fit.
Streams selected numeric columns, preserves asynchronous event timestamps, and
uses bounded-age alignment for comparisons. Requires NumPy, pandas, and SciPy.
Historical PER26 observations must not be silently applied to PER27 geometry.
"""
import argparse
import csv
import json
import pathlib

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def numeric_values(series):
    """@brief Decode numeric cells and symbolic enum labels such as ready2drive (5)."""
    if pd.api.types.is_numeric_dtype(series):
        return series.to_numpy(dtype=np.float64)
    numeric = pd.to_numeric(series, errors="coerce")
    labels = series.notna() & numeric.isna()
    if labels.any():
        encoded = series[labels].astype("string").str.extract(r"\((-?\d+(?:\.\d+)?)\)\s*$", expand=False)
        numeric.loc[labels] = pd.to_numeric(encoded, errors="coerce")
    invalid = series.notna() & numeric.isna()
    if invalid.any():
        raise ValueError(f"Unrecognized numeric/enum cells: {series[invalid].head(3).tolist()}")
    return numeric.to_numpy(dtype=np.float64)


def read_capture(path):
    """@brief Stream relevant sparse CAN signals with their original units and sample times."""
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = [next(reader) for _ in range(7)]
    if header[2][2] != "Message" or header[4][1] != "DAQ Timestamp":
        raise ValueError(f"Not a supported seven-row PER capture: {path}")
    selected_messages = {
        "main_hb", "pedals", "vcu_torque_request", "vcu_settings", "gps_speed_heading",
        "gps_velocity", "IMU_acceleration", "IMU_angular_rate", "IZZE_acceleration",
        "IZZE_angular_rate", "steering_angle", "front_shockpots", "rear_shockpots",
        "pack_stats", "wheel_speeds", "motor_speeds", "motor_temps", "igbt_temps",
    }
    columns = {i: (header[2][i] + "." + header[4][i], header[6][i])
               for i in range(3, len(header[2]))
               if header[2][i] in selected_messages or header[2][i].endswith(("_SET", "_CRIT", "_INFO", "_TEMPS"))}
    pieces = {name: [] for name, _ in columns.values()}
    rows, start, end, regressions = 0, None, None, 0
    for chunk in pd.read_csv(path, skiprows=7, header=None, usecols=[1, *columns],
                             low_memory=False, chunksize=100_000):
        if chunk.empty:
            continue
        times = numeric_values(chunk[1])
        if not np.isfinite(times).all():
            raise ValueError(f"Invalid DAQ timestamps: {path}")
        if (end is not None and times[0] < end) or np.any(np.diff(times) < 0):
            regressions += 1
        if start is None:
            start = float(times[0])
        end = float(times[-1])
        rows += len(chunk)
        for index, (name, _) in columns.items():
            values = numeric_values(chunk[index])
            valid = np.isfinite(values)
            if valid.any():
                pieces[name].append(np.column_stack((times[valid], values[valid])))
    if not rows:
        raise ValueError(f"No telemetry rows: {path}")
    if regressions:
        raise ValueError(f"DAQ time regressed in {regressions} chunks; split/rebase capture before alignment: {path}")
    events = {name: np.concatenate(chunks) for name, chunks in pieces.items() if chunks}
    units = {name: unit for name, unit in columns.values()}
    return events, units, {"rows": rows, "start_s": start, "end_s": end,
                           "duration_s": end - start if start is not None else 0}


def describe(events, unit):
    """@brief Report event-value quantiles and actual positive update-interval quantiles."""
    values = events[:, 1]
    intervals = np.diff(events[:, 0])
    intervals = intervals[intervals > 0]
    return {"unit": unit, "samples": len(values),
            "min_p01_p50_p99_max": np.quantile(values, [0, 0.01, 0.5, 0.99, 1]).tolist(),
            "update_p50_p99_ms": (np.quantile(intervals, [0.5, 0.99]) * 1000).tolist() if len(intervals) else None}


def align(events, name, times, tolerance):
    """@brief Backward-asof alignment; missing/stale values remain NaN, never zero-filled."""
    if name not in events:
        return np.full(len(times), np.nan)
    source = events[name]
    index = np.searchsorted(source[:, 0], times, side="right") - 1
    valid = index >= 0
    safe = np.maximum(index, 0)
    valid &= times - source[safe, 0] <= tolerance
    return np.where(valid, source[safe, 1], np.nan)


def imu_comparison(events, kind):
    """@brief Compare logged processed axes with the simulator's firmware mounting transform."""
    body_prefix, raw_prefix = "IMU_" + kind, "IZZE_" + kind
    axes = ["X_axis", "Y_axis", "Z_axis"]
    key = body_prefix + ".X_axis"
    if key not in events:
        return None
    times = events[key][:, 0]
    body = np.column_stack([align(events, body_prefix + "." + axis, times, 0.002) for axis in axes])
    raw = np.column_stack([align(events, raw_prefix + "." + axis, times, 0.06) for axis in axes])
    valid = np.isfinite(body).all(axis=1) & np.isfinite(raw).all(axis=1)
    body, raw = body[valid], raw[valid]
    if not len(body):
        return None
    mounting = (Rotation.from_euler("Z", -np.pi / 2) * Rotation.from_euler("X", 2.2689280276)).as_matrix()
    residual = body - raw @ mounting.T
    return {"samples": len(body), "rmse_per_axis": np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
            "median_abs_error_per_axis": np.median(np.abs(residual), axis=0).tolist(),
            "note": "Processed/raw reports are asynchronous; residual includes report age and quantization."}


def kinematic_signs(events):
    """@brief Inspect rest gravity and lateral acceleration/rate polarity, without fitting tire grip."""
    key = "gps_speed_heading.gps_speed"
    if key not in events:
        return None
    times, speed = events[key].T
    acceleration = np.column_stack([align(events, "IMU_acceleration." + axis, times, 0.015)
                                    for axis in ("X_axis", "Y_axis", "Z_axis")])
    gyro = align(events, "IMU_angular_rate.Z_axis", times, 0.015)
    stationary = (speed < 0.1) & np.isfinite(acceleration).all(axis=1)
    moving = (speed > 5) & (np.abs(gyro) < 100) & np.isfinite(gyro + acceleration[:, 1])
    ay, expected = acceleration[moving, 1], (speed * np.deg2rad(gyro) / 9.81)[moving]
    correlation = (float(np.corrcoef(ay, expected)[0, 1]) if len(ay) > 10
                   and np.std(ay) > 1e-9 and np.std(expected) > 1e-9 else None)
    return {"stationary_samples": int(stationary.sum()),
            "stationary_median_xyz_g": np.median(acceleration[stationary], axis=0).tolist() if stationary.any() else None,
            "moving_samples": int(moving.sum()), "ay_vs_speed_yaw_correlation": correlation,
            "note": "GPS <0.1 m/s for rest; >5 m/s and |yaw|<100 deg/s for lateral comparison. Bias, sideslip, road tilt and location are not fitted."}


def speed_comparison(events):
    """@brief Estimate rolling motor/GPS speed ratio using the assumed 0.2 m radius, not a gear fit."""
    key = "gps_speed_heading.gps_speed"
    if key not in events:
        return None
    times, speed = events[key].T
    yaw = align(events, "IMU_angular_rate.Z_axis", times, 0.06)
    valid = (speed > 5) & (speed < 40) & np.isfinite(yaw) & (np.abs(yaw) < 5)
    output = {}
    for inverter in "ABCD":
        rpm = align(events, "INV" + inverter + "_CRIT.AMK_ActualSpeed", times, 0.05)
        ratio = rpm[valid] * (2 * np.pi * 0.2) / (60 * speed[valid])
        ratio = ratio[np.isfinite(ratio)]
        if len(ratio):
            output[inverter] = {"samples": len(ratio), "p10_p50_p90": np.quantile(ratio, [0.1, 0.5, 0.9]).tolist()}
    return {"effective_ratio": output,
            "note": "Near-straight motion, GPS >5 m/s, gyro <5 deg/s; tire radius, slip, latency and turn geometry confound ratio."}


def regen_comparison(events):
    """@brief Inspect pack-current sign during fresh, concurrently reported four-motor regen."""
    key = "pack_stats.pack_current"
    if key not in events:
        return None
    times, current = events[key].T
    torque = np.column_stack([align(events, "INV" + inverter + "_CRIT.AMK_ActualTorque", times, 0.05) for inverter in "ABCD"])
    rpm = np.column_stack([align(events, "INV" + inverter + "_CRIT.AMK_ActualSpeed", times, 0.05) for inverter in "ABCD"])
    complete = np.isfinite(torque).all(axis=1) & np.isfinite(rpm).all(axis=1)
    # Historical metadata defines torque as nominal-torque percent; 9.8 Nm is
    # only the current simulator's assumption, not independently identified here.
    shaft_power = np.sum(torque * 9.8 / 100 * rpm * (2 * np.pi / 60), axis=1)
    braking = complete & (shaft_power < -1000)
    return {"regen_samples": int(braking.sum()),
            "negative_current_fraction": float(np.mean(current[braking] < 0)) if braking.any() else None,
            "current_p10_p50_p90_a": np.quantile(current[braking], [0.1, 0.5, 0.9]).tolist() if braking.any() else None,
            "note": "Fresh shaft power <-1 kW; logger current offset/scaling and auxiliary loads remain uncalibrated."}


def analyze(path):
    """@brief Produce a read-only per-capture evidence report, not calibrated simulator defaults."""
    events, units, timing = read_capture(path)
    return {"path": str(path), "bytes": path.stat().st_size, **timing,
            "signals": {name: describe(value, units[name]) for name, value in events.items()},
            "imu_acceleration_mount": imu_comparison(events, "acceleration"),
            "imu_gyro_mount": imu_comparison(events, "angular_rate"),
            "kinematic_signs": kinematic_signs(events),
            "speed_comparison": speed_comparison(events), "regen_comparison": regen_comparison(events)}


def main():
    """@brief Analyze explicitly selected captures and optionally write JSON outside the dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", type=pathlib.Path, nargs="+")
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()
    if args.output and args.output.resolve() in {path.resolve() for path in args.captures}:
        parser.error("Output must not overwrite an input capture")
    result = {"purpose": "Historical telemetry sanity review; not PER27 calibration", "captures": []}
    for path in args.captures:
        result["captures"].append(analyze(path))
    encoded = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded)
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()
