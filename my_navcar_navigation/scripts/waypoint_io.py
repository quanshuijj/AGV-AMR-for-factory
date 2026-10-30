#!/usr/bin/env python3
import math
from pathlib import Path

import yaml


def normalize_vector(x, y):
    norm = math.hypot(x, y)
    if norm < 1e-9:
        return 1.0, 0.0
    return x / norm, y / norm


def yaw_from_vector(x, y):
    x, y = normalize_vector(x, y)
    return math.atan2(y, x)


def snap_heading_to_cardinal(x, y):
    x, y = normalize_vector(x, y)
    if abs(x) >= abs(y):
        return (1.0, 0.0) if x >= 0.0 else (-1.0, 0.0)
    return (0.0, 1.0) if y >= 0.0 else (0.0, -1.0)


def vector_from_yaw(yaw):
    return math.cos(yaw), math.sin(yaw)


def waypoint_heading(wp):
    heading = wp.get("heading", {})
    return normalize_vector(float(heading.get("x", 1.0)), float(heading.get("y", 0.0)))


def smooth_sequence_headings(waypoints, lookahead=1, manual_weight=0.0):
    if len(waypoints) < 2:
        return

    lookahead = max(1, int(lookahead))
    manual_weight = max(0.0, min(1.0, float(manual_weight)))
    smoothed = []

    for index, wp in enumerate(waypoints):
        prev_index = max(0, index - lookahead)
        next_index = min(len(waypoints) - 1, index + lookahead)
        if prev_index == next_index:
            prev_index = max(0, index - 1)
            next_index = min(len(waypoints) - 1, index + 1)

        prev_pos = waypoints[prev_index]["position"]
        next_pos = waypoints[next_index]["position"]
        tx = float(next_pos["x"]) - float(prev_pos["x"])
        ty = float(next_pos["y"]) - float(prev_pos["y"])
        tx, ty = normalize_vector(tx, ty)

        if manual_weight > 0.0:
            mx, my = waypoint_heading(wp)
            tx = tx * (1.0 - manual_weight) + mx * manual_weight
            ty = ty * (1.0 - manual_weight) + my * manual_weight
            tx, ty = normalize_vector(tx, ty)

        smoothed.append((tx, ty))

    for wp, (hx, hy) in zip(waypoints, smoothed):
        wp["heading"] = {"x": hx, "y": hy}
        wp["yaw"] = yaw_from_vector(hx, hy)
        wp["heading_source"] = "smoothed_tangent"


def load_waypoint_file(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    data.setdefault("schema_version", 1)
    data.setdefault("frame_id", "map")
    data.setdefault("waypoints", [])
    validate_waypoints(data, path)
    return data


def validate_waypoints(data, path="<memory>"):
    if data.get("schema_version") != 1:
        raise ValueError("{}: unsupported schema_version {}".format(path, data.get("schema_version")))
    if not isinstance(data.get("waypoints"), list):
        raise ValueError("{}: waypoints must be a list".format(path))

    seen = set()
    for index, wp in enumerate(data["waypoints"]):
        prefix = "{} waypoint[{}]".format(path, index)
        wp_id = wp.get("id")
        if not wp_id:
            raise ValueError("{}: missing id".format(prefix))
        if wp_id in seen:
            raise ValueError("{}: duplicate waypoint id '{}'".format(prefix, wp_id))
        seen.add(wp_id)

        position = wp.get("position")
        if not isinstance(position, dict):
            raise ValueError("{}: missing position".format(prefix))
        for key in ("x", "y"):
            if key not in position:
                raise ValueError("{}: missing position.{}".format(prefix, key))

        heading = wp.get("heading")
        if heading is None:
            yaw = float(wp.get("yaw", 0.0))
            hx, hy = vector_from_yaw(yaw)
            wp["heading"] = {"x": hx, "y": hy}
            wp["yaw"] = yaw
        else:
            if not isinstance(heading, dict) or "x" not in heading or "y" not in heading:
                raise ValueError("{}: heading must contain x and y".format(prefix))
            hx, hy = normalize_vector(float(heading["x"]), float(heading["y"]))
            wp["heading"] = {"x": hx, "y": hy}
            wp["yaw"] = float(wp.get("yaw", yaw_from_vector(hx, hy)))


def dump_waypoint_file(path, data):
    path = Path(path)
    validate_waypoints(data, path)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
