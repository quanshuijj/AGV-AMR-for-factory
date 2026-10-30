import math

from agv_nav_evaluation.geometry_utils import (
    euclidean_distance,
    point_to_segment_signed_distance,
    segment_heading,
    transform_reference_point,
    wrap_angle,
)
from agv_nav_evaluation.metrics_common import safe_percentile


def compute_path_length(points):
    if not points or len(points) < 2:
        return 0.0
    length = 0.0
    for idx in range(1, len(points)):
        length += euclidean_distance((points[idx - 1]["x"], points[idx - 1]["y"]), (points[idx]["x"], points[idx]["y"]))
    return length


def compute_path_length_ratio(actual_path, reference_path):
    ref_length = compute_path_length(reference_path)
    actual_length = compute_path_length(actual_path)
    ratio = math.nan
    if ref_length > 1e-9:
        ratio = actual_length / ref_length
    return {
        "actual_path_length": actual_length,
        "reference_path_length": ref_length,
        "path_length_ratio": ratio,
    }


def _closest_path_segment(point, reference_path):
    if not reference_path or len(reference_path) < 2:
        return None
    best = None
    for idx in range(len(reference_path) - 1):
        start = (reference_path[idx]["x"], reference_path[idx]["y"])
        end = (reference_path[idx + 1]["x"], reference_path[idx + 1]["y"])
        signed_dist, proj = point_to_segment_signed_distance(point, start, end)
        abs_dist = abs(signed_dist)
        if best is None or abs_dist < best["abs_dist"]:
            best = {
                "segment_index": idx,
                "signed_dist": signed_dist,
                "abs_dist": abs_dist,
                "projection": proj,
                "heading": segment_heading(start, end),
            }
    return best


def compute_cross_track_error(actual_path, reference_path, reference_offset=None):
    series = []
    if not actual_path or not reference_path:
        return series
    offset = reference_offset or (0.0, 0.0)
    shifted_path = [transform_reference_point(pose, offset) for pose in actual_path]
    for pose in shifted_path:
        best = _closest_path_segment((pose["x"], pose["y"]), reference_path)
        if best is None:
            continue
        series.append({"t": pose["t"], "cte": best["signed_dist"], "abs_cte": abs(best["signed_dist"]), "segment_index": best["segment_index"]})
    return series


def compute_heading_error(actual_path, reference_path, reference_offset=None):
    series = []
    if not actual_path or not reference_path:
        return series
    offset = reference_offset or (0.0, 0.0)
    shifted_path = [transform_reference_point(pose, offset) for pose in actual_path]
    for pose in shifted_path:
        best = _closest_path_segment((pose["x"], pose["y"]), reference_path)
        if best is None:
            continue
        error = wrap_angle(pose["yaw"] - best["heading"])
        series.append({"t": pose["t"], "heading_error": error, "abs_heading_error": abs(error), "segment_index": best["segment_index"]})
    return series


def compute_final_pose_error(final_pose, goal_pose):
    if not final_pose or not goal_pose:
        return {"final_position_error": math.nan, "final_heading_error": math.nan}
    return {
        "final_position_error": euclidean_distance((final_pose["x"], final_pose["y"]), (goal_pose["x"], goal_pose["y"])),
        "final_heading_error": wrap_angle(final_pose["yaw"] - goal_pose["yaw"]),
    }


def summarize_cte(cte_series):
    values = [item["abs_cte"] for item in cte_series]
    if not values:
        return {"cte_mean": math.nan, "cte_rmse": math.nan, "cte_p95": math.nan, "cte_max": math.nan}
    mean = sum(values) / float(len(values))
    rmse = math.sqrt(sum(value * value for value in values) / float(len(values)))
    return {
        "cte_mean": mean,
        "cte_rmse": rmse,
        "cte_p95": safe_percentile(values, 95.0),
        "cte_max": max(values),
    }


def summarize_heading_error(series):
    values = [abs(item["heading_error"]) for item in series]
    if not values:
        return {"heading_error_rmse": math.nan, "heading_error_p95": math.nan}
    rmse = math.sqrt(sum(value * value for value in values) / float(len(values)))
    return {
        "heading_error_rmse": rmse,
        "heading_error_p95": safe_percentile(values, 95.0),
    }
