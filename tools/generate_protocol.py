#!/usr/bin/env python3
"""Generate the simulator's classic-CAN schema from PER's authoritative SuperDBC."""
import json
import pathlib
import re
import sys

def main():
    """@brief Generate build-local definitions from the selected authoritative inputs."""
    source, output = map(pathlib.Path, sys.argv[1:])
    db = json.loads(source.read_text())
    selected = {
        "pack_bms", "pack_analog", "LWS_Standard", "IZZE_angular_rate",
        "IZZE_acceleration", "main_hb", "vcu_settings", "vcu_driver_request",
        "vcu_torque_request", "pedals", "motor_speeds", "motor_temps",
        "igbt_temps", "IMU_acceleration", "IMU_angular_rate",
    }
    for inv in "ABCD":
        selected.update(f"INV{inv}_{suffix}" for suffix in ("SET", "CRIT", "INFO", "TEMPS"))

    lines = [
        "// Generated from SuperDBC. Do not edit.",
        "#pragma once",
        '#include "persim/protocol.hpp"',
        "namespace persim::db {",
        f'inline constexpr auto content_hash = "{db["content_hash"]}";',
    ]
    found = set()
    for bus_name, bus in db["buses"].items():
        for message in bus["messages"]:
            name = message["message_name"]
            if name not in selected:
                continue
            if name in found:
                raise ValueError(f"Ambiguous message: {name}")
            found.add(name)
            if message["length_bytes"] > 8 or message["is_extended_id"]:
                raise ValueError(f"Expected standard classic CAN: {name}")
            ident = re.sub(r"\W", "_", name)
            lines.append(f"inline constexpr Signal {ident}_signals[] = {{")
            for signal in message["signals"]:
                start = signal["start_bit"]
                width = signal["bit_length"]
                big = signal["byte_order"] == "big_endian"
                bit = start
                for _ in range(width):
                    if not 0 <= bit < message["length_bytes"] * 8:
                        raise ValueError(f"Signal outside frame: {name}/{signal['signal_name']}")
                    bit = bit + 1 if not big else (bit + 15 if bit % 8 == 0 else bit - 1)
                lines.append(
                    '    {"%s", %d, %d, %s, %s, %.17g, %.17g},'
                    % (signal["signal_name"], start, width,
                       str(big).lower(), str(signal["raw_type"] == "signed").lower(),
                       signal["scale"], signal["offset"])
                )
            lines += [
                "};",
                f'inline constexpr Message {ident}{{"{name}", "{bus_name}", '
                f'{message["id"]}, {message["length_bytes"]}, '
                f'{message["nominal_period_ms"] or 0}, {ident}_signals}};',
            ]
    missing = selected - found
    if missing:
        raise ValueError(f"Missing messages: {sorted(missing)}")
    lines.append("}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
