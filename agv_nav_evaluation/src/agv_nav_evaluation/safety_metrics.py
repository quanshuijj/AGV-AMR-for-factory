import math

from agv_nav_evaluation.geometry_utils import polygon_distance_to_point


def compute_min_clearance_from_costmap(robot_poses, costmap_series, footprint):
    series = []
    if not robot_poses or not costmap_series:
        return series
    for pose in robot_poses:
        nearest_costmap = None
        best_gap = None
        for costmap in costmap_series:
            gap = abs(costmap["t"] - pose["t"])
            if best_gap is None or gap < best_gap:
                nearest_costmap = costmap
                best_gap = gap
        if nearest_costmap is None:
            continue
        obstacles = nearest_costmap.get("occupied_points", [])
        if not obstacles:
            series.append({"t": pose["t"], "clearance": math.nan})
            continue
        translated_polygon = []
        yaw = pose.get("yaw", 0.0)
        for px, py in footprint:
            translated_polygon.append(
                (
                    pose["x"] + px * math.cos(yaw) - py * math.sin(yaw),
                    pose["y"] + px * math.sin(yaw) + py * math.cos(yaw),
                )
            )
        best_clearance = None
        for obstacle in obstacles:
            dist = polygon_distance_to_point(translated_polygon, obstacle)
            if best_clearance is None or dist < best_clearance:
                best_clearance = dist
        series.append({"t": pose["t"], "clearance": best_clearance if best_clearance is not None else math.nan})
    return series


def compute_near_miss_count(clearance_series, threshold):
    return len([item for item in clearance_series if item.get("clearance") is not None and not math.isnan(item["clearance"]) and item["clearance"] < threshold])


def merge_manual_collision_events(manual_events):
    return [event for event in manual_events if event.get("event_type") == "collision"]
