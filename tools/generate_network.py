#!/usr/bin/env python3
"""Build FIL wiring from current PER generator configs, never stale FIL examples."""
import json
import pathlib
import sys

def main():
    """@brief Generate build-local definitions from the selected authoritative inputs."""
    monorepo, fil, output = (pathlib.Path(p).resolve() for p in sys.argv[1:])
    output.mkdir(parents=True, exist_ok=True)
    bus_configs = json.loads((monorepo / "generators/configs/system/bus_configs.json").read_text())
    buses = {b["name"]: b for b in bus_configs["busses"]}
    network_buses = {}
    board_paths = []
    for board in ["dashboard", "main_module", "torque_vector", "front_driveline", "rear_driveline"]:
        node = "DRIVELINE" if "driveline" in board else board.upper()
        config = json.loads((monorepo / f"generators/configs/nodes/{node}.json").read_text())
        connections = {}
        for bus_name, connection in config["busses"].items():
            b = buses[bus_name]
            if b["is_extended_id"]:
                raise ValueError(f"Classic standard CAN only: {bus_name}")
            bus_id = str(b["bus_id"])
            connections[connection["peripheral"]] = {"bus": bus_id}
            network_buses[bus_id] = {"type": "can", "bitrate": b["baud_rate"]}
        destination = output / f"{board}.json"
        destination.write_text(json.dumps({
            "schema_version": 1, "name": board,
            "mcu": str(fil / "configs/mcus/stm32g474retx.json"),
            "elf": str(monorepo / f"firmware/output/{board}/{board}.elf"),
            "can": connections,
        }, indent=2) + "\n")
        board_paths.append(str(destination))
    (output / "per-network.json").write_text(json.dumps({
        "schema_version": 1, "name": "PERsim", "buses": network_buses,
        "boards": board_paths, "stimuli": [],
    }, indent=2) + "\n")


if __name__ == "__main__":
    main()
