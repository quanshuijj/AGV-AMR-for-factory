import math

from agv_nav_evaluation.metrics_common import safe_rmse


def compute_acceleration(velocity_series):
    series = []
    for idx in range(1, len(velocity_series)):
        prev_item = velocity_series[idx - 1]
        curr_item = velocity_series[idx]
        dt = curr_item["t"] - prev_item["t"]
        if dt <= 1e-9:
            continue
        series.append(
            {
                "t": curr_item["t"],
                "linear_acceleration": (curr_item["v"] - prev_item["v"]) / dt,
                "angular_acceleration": (curr_item["omega"] - prev_item["omega"]) / dt,
            }
        )
    return series


def compute_jerk(acceleration_series):
    series = []
    for idx in range(1, len(acceleration_series)):
        prev_item = acceleration_series[idx - 1]
        curr_item = acceleration_series[idx]
        dt = curr_item["t"] - prev_item["t"]
        if dt <= 1e-9:
            continue
        series.append(
            {
                "t": curr_item["t"],
                "linear_jerk": (curr_item["linear_acceleration"] - prev_item["linear_acceleration"]) / dt,
                "angular_jerk": (curr_item["angular_acceleration"] - prev_item["angular_acceleration"]) / dt,
            }
        )
    return series


def compute_curvature(v, omega, min_velocity_for_curvature=0.05):
    if abs(v) < min_velocity_for_curvature:
        return math.nan
    return omega / v


def compute_turning_radius(curvature):
    if curvature is None or (isinstance(curvature, float) and math.isnan(curvature)) or abs(curvature) < 1e-9:
        return math.inf
    return 1.0 / abs(curvature)


def compute_turning_radius_violation(curvature_series, min_turning_radius):
    valid = [item for item in curvature_series if item["turning_radius"] != math.inf and not math.isnan(item["turning_radius"])]
    if not valid:
        return {"turning_radius_violation_rate": math.nan, "min_turning_radius_actual": math.nan}
    violations = [item for item in valid if item["turning_radius"] < min_turning_radius]
    return {
        "turning_radius_violation_rate": len(violations) / float(len(valid)),
        "min_turning_radius_actual": min(item["turning_radius"] for item in valid),
    }


def compute_cmd_tracking_error(cmd_series, actual_series):
    if not cmd_series or not actual_series:
        return []
    actual_times = [item["t"] for item in actual_series]
    series = []
    for cmd in cmd_series:
        best_idx = None
        best_gap = None
        for idx, actual in enumerate(actual_series):
            gap = abs(actual["t"] - cmd["t"])
            if best_gap is None or gap < best_gap:
                best_idx = idx
                best_gap = gap
        if best_idx is None:
            continue
        actual = actual_series[best_idx]
        series.append(
            {
                "t": cmd["t"],
                "cmd_v": cmd.get("v", 0.0),
                "cmd_omega": cmd.get("omega", 0.0),
                "actual_v": actual.get("v", 0.0),
                "actual_omega": actual.get("omega", 0.0),
                "e_v": cmd.get("v", 0.0) - actual.get("v", 0.0),
                "e_delta": cmd.get("delta", math.nan) - actual.get("delta", math.nan) if "delta" in cmd and "delta" in actual else math.nan,
            }
        )
    return series


def summarize_smoothness(acceleration_series, jerk_series, curvature_series, tracking_series, max_steering_angle=None):
    linear_jerk_values = [item["linear_jerk"] for item in jerk_series]
    angular_jerk_values = [item["angular_jerk"] for item in jerk_series]
    curvature_values = [abs(item["curvature"]) for item in curvature_series if not math.isnan(item["curvature"])]
    tracking_v = [item["e_v"] for item in tracking_series]
    tracking_delta = [item["e_delta"] for item in tracking_series if not math.isnan(item["e_delta"])]
    steering_saturation_count = 0
    if max_steering_angle is not None:
        for item in tracking_series:
            delta = item.get("delta_cmd")
            if delta is not None and not math.isnan(delta) and abs(delta) >= max_steering_angle:
                steering_saturation_count += 1
    return {
        "linear_jerk_rms": safe_rmse(linear_jerk_values),
        "angular_jerk_rms": safe_rmse(angular_jerk_values),
        "max_curvature": max(curvature_values) if curvature_values else math.nan,
        "steering_saturation_count": steering_saturation_count,
        "velocity_tracking_rmse": safe_rmse(tracking_v),
        "steering_tracking_rmse": safe_rmse(tracking_delta),
        "control_effort": sum(abs(item.get("cmd_v", 0.0)) + abs(item.get("cmd_omega", 0.0)) for item in tracking_series) if tracking_series else math.nan,
    }
