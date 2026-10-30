import math

from agv_nav_evaluation.geometry_utils import pose_distance
from agv_nav_evaluation.metrics_common import safe_rmse


def compute_rtk_fix_rate(rtk_status_series, fixed_values=None, float_values=None, invalid_values=None):
    fixed_values = set(str(v) for v in (fixed_values or []))
    float_values = set(str(v) for v in (float_values or []))
    invalid_values = set(str(v) for v in (invalid_values or []))
    if not rtk_status_series:
        return {"rtk_fix_rate": math.nan, "rtk_float_rate": math.nan, "rtk_invalid_rate": math.nan}
    total = float(len(rtk_status_series))
    fixed_count = 0
    float_count = 0
    invalid_count = 0
    for item in rtk_status_series:
        value = str(item.get("value"))
        if value in fixed_values:
            fixed_count += 1
        elif value in float_values:
            float_count += 1
        elif value in invalid_values:
            invalid_count += 1
    return {
        "rtk_fix_rate": fixed_count / total,
        "rtk_float_rate": float_count / total,
        "rtk_invalid_rate": invalid_count / total,
    }


def compute_tf_dropout(tf_series, timeout=0.5):
    if not tf_series or len(tf_series) < 2:
        return {"tf_dropout_count": math.nan}
    count = 0
    for idx in range(1, len(tf_series)):
        if tf_series[idx]["t"] - tf_series[idx - 1]["t"] > timeout:
            count += 1
    return {"tf_dropout_count": count}


def compute_localization_dropout(pose_series, timeout=0.5):
    if not pose_series or len(pose_series) < 2:
        return {"localization_dropout_count": math.nan}
    count = 0
    for idx in range(1, len(pose_series)):
        if pose_series[idx]["t"] - pose_series[idx - 1]["t"] > timeout:
            count += 1
    return {"localization_dropout_count": count}


def compute_ate(est_trajectory, gt_trajectory):
    if not est_trajectory or not gt_trajectory:
        return {"ate_rmse": math.nan, "ate_status": "GT unavailable, skip ATE/RPE"}
    errors = []
    for est in est_trajectory:
        best = None
        best_gap = None
        for gt in gt_trajectory:
            gap = abs(gt["t"] - est["t"])
            if best_gap is None or gap < best_gap:
                best = gt
                best_gap = gap
        if best is None:
            continue
        errors.append(pose_distance(est, best))
    return {"ate_rmse": safe_rmse(errors), "ate_status": "computed" if errors else "GT unavailable, skip ATE/RPE"}


def compute_rpe(est_trajectory, gt_trajectory):
    if not est_trajectory or not gt_trajectory:
        return {"rpe_rmse": math.nan}
    rel_errors = []
    for idx in range(1, min(len(est_trajectory), len(gt_trajectory))):
        est_delta = pose_distance(est_trajectory[idx], est_trajectory[idx - 1])
        gt_delta = pose_distance(gt_trajectory[idx], gt_trajectory[idx - 1])
        rel_errors.append(est_delta - gt_delta)
    return {"rpe_rmse": safe_rmse(rel_errors)}


def compute_degradation_segments(rtk_status_series, fixed_values=None):
    fixed_values = set(str(v) for v in (fixed_values or []))
    segments = []
    current = None
    for item in rtk_status_series:
        is_degraded = str(item.get("value")) not in fixed_values
        if is_degraded and current is None:
            current = {"start": item["t"], "end": item["t"]}
        elif is_degraded and current is not None:
            current["end"] = item["t"]
        elif not is_degraded and current is not None:
            segments.append(current)
            current = None
    if current is not None:
        segments.append(current)
    return segments
