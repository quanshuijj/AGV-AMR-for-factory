#!/usr/bin/env python3
import argparse
from copy import deepcopy
from pathlib import Path

from waypoint_io import dump_waypoint_file, load_waypoint_file


def shift_waypoint_data(data, dx, dy):
    shifted = deepcopy(data)
    for waypoint in shifted.get("waypoints", []):
        position = waypoint["position"]
        position["x"] = float(position["x"]) + float(dx)
        position["y"] = float(position["y"]) + float(dy)
    return shifted


def main():
    parser = argparse.ArgumentParser(description="Shift waypoint YAML files by a map-origin delta.")
    parser.add_argument("--dx", type=float, required=True)
    parser.add_argument("--dy", type=float, required=True)
    parser.add_argument("files", nargs="+")
    args = parser.parse_args()

    for file_name in args.files:
        path = Path(file_name)
        data = load_waypoint_file(path)
        shifted = shift_waypoint_data(data, args.dx, args.dy)
        dump_waypoint_file(path, shifted)
        print(f"shifted {path} by dx={args.dx:.6f}, dy={args.dy:.6f}")


if __name__ == "__main__":
    main()
