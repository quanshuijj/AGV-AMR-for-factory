#!/usr/bin/env python3
import argparse
import math
import os

from agv_nav_evaluation.config_loader import load_all_configs
from agv_nav_evaluation.csv_json_io import load_csv_rows
from agv_nav_evaluation.localization_metrics import (
    compute_ate,
    compute_degradation_segments,
    compute_localization_dropout,
    compute_rpe,
    compute_rtk_fix_rate,
    compute_tf_dropout,
)
from agv_nav_evaluation.manual_events import load_manual_events
from agv_nav_evaluation.metrics_common import ensure_dir, write_json, write_single_row_csv
from agv_nav_evaluation.path_metrics import (
    compute_cross_track_error,
    compute_final_pose_error,
    compute_heading_error,
    compute_path_length_ratio,
    summarize_cte,
    summarize_heading_error,
)
from agv_nav_evaluation.planning_metrics import (
    compute_cmd_vel_dropout,
    compute_control_loop_frequency,
    compute_costmap_update_rate,
    compute_global_planning_time,
    compute_local_planning_time,
    compute_planning_to_control_latency,
    compute_replanning_count,
)
from agv_nav_evaluation.report_writer import build_markdown_report
from agv_nav_evaluation.safety_metrics import compute_min_clearance_from_costmap, compute_near_miss_count
from agv_nav_evaluation.smoothness_metrics import (
    compute_acceleration,
    compute_cmd_tracking_error,
    compute_curvature,
    compute_jerk,
    compute_turning_radius,
    compute_turning_radius_violation,
    summarize_smoothness,
)
from agv_nav_evaluation.task_metrics import compute_task_metrics


def compute_metrics_from_dataset(dataset, configs, test_id="unknown", manual_events=None):
    manual_events = manual_events or []
    topic_cfg = configs["topics"]
    vehicle_cfg = (configs["vehicle"] or {}).get("vehicle", {})
    metric_cfg = (configs["metrics"] or {}).get("metrics", {})
    warnings = list(dataset.get("warnings", []))

    odom_series = dataset.get("filtered_odom_series", []) or dataset.get("local_odom_series", []) or dataset.get("odom_series", [])
    goal_series = dataset.get("goal_series", [])
    result_series = dataset.get("result_series", [])
    global_plan_series = dataset.get("global_plan_series", [])
    local_plan_series = dataset.get("local_plan_series", [])
    cmd_vel_series = dataset.get("cmd_vel_series", [])
    tf_series = dataset.get("tf_series", [])
    costmap_series = dataset.get("costmap_series", [])
    rtk_status_series = dataset.get("rtk_status_series", [])
    rosout_series = dataset.get("rosout_series", [])
    gt_series = dataset.get("gt_series", [])

    reference_path = global_plan_series[-1]["poses"] if global_plan_series else []
    actual_path = [{"t": item["t"], "x": item["x"], "y": item["y"], "yaw": item["yaw"]} for item in odom_series]
    tracking_offset = tuple(vehicle_cfg.get("reference_points", {}).get(vehicle_cfg.get("reference_point_mode", "base_link"), [0.0, 0.0]))

    task_summary = compute_task_metrics(goal_series, dataset.get("status_series", []), result_series, manual_events, metric_cfg)
    path_length_summary = compute_path_length_ratio(actual_path, reference_path)
    cte_series = compute_cross_track_error(actual_path, reference_path, tracking_offset)
    heading_series = compute_heading_error(actual_path, reference_path, tracking_offset)
    final_pose_summary = compute_final_pose_error(actual_path[-1] if actual_path else None, goal_series[-1] if goal_series else None)

    velocity_series = [{"t": item["t"], "v": item.get("v", 0.0), "omega": item.get("omega", 0.0)} for item in odom_series]
    acceleration_series = compute_acceleration(velocity_series)
    jerk_series = compute_jerk(acceleration_series)
    curvature_series = []
    for item in velocity_series:
        curvature = compute_curvature(item.get("v", 0.0), item.get("omega", 0.0), metric_cfg.get("min_velocity_for_curvature", 0.05))
        curvature_series.append({"t": item["t"], "curvature": curvature, "turning_radius": compute_turning_radius(curvature)})
    tracking_series = compute_cmd_tracking_error(
        [{"t": item["t"], "v": item.get("v", 0.0), "omega": item.get("omega", 0.0)} for item in cmd_vel_series],
        velocity_series,
    )
    for index, item in enumerate(tracking_series):
        if index < len(cmd_vel_series):
            item["delta_cmd"] = cmd_vel_series[index].get("delta", math.nan)

    clearance_series = compute_min_clearance_from_costmap(actual_path, costmap_series, vehicle_cfg.get("footprint", []))
    min_clearance = min([item["clearance"] for item in clearance_series if item.get("clearance") is not None and not math.isnan(item["clearance"])], default=math.nan)
    near_miss_count = compute_near_miss_count(clearance_series, metric_cfg.get("near_miss_threshold", 0.3))
    safety_stop_count = len([event for event in manual_events if event.get("event_type") == "emergency_stop"])

    rtk_rates = compute_rtk_fix_rate(
        rtk_status_series,
        fixed_values=metric_cfg.get("rtk_fixed_values", []),
        float_values=metric_cfg.get("rtk_float_values", []),
        invalid_values=metric_cfg.get("rtk_invalid_values", []),
    )
    tf_dropout = compute_tf_dropout(tf_series, timeout=metric_cfg.get("tf_timeout", 0.5))
    localization_dropout = compute_localization_dropout(actual_path, timeout=metric_cfg.get("localization_timeout", 0.5))
    ate_summary = compute_ate(actual_path, gt_series)
    rpe_summary = compute_rpe(actual_path, gt_series)
    degradation_segments = compute_degradation_segments(rtk_status_series, fixed_values=metric_cfg.get("rtk_fixed_values", []))

    global_plan_timing = compute_global_planning_time(goal_series, global_plan_series, rosout_series, timeout=metric_cfg.get("global_planning_goal_timeout", 10.0))
    local_plan_timing = compute_local_planning_time(local_plan_series, cmd_vel_series, rosout_series, timeout=metric_cfg.get("local_plan_to_cmd_timeout", 2.0))
    replanning = compute_replanning_count(global_plan_series)
    control_frequency = compute_control_loop_frequency(cmd_vel_series)
    cmd_dropout = compute_cmd_vel_dropout(
        cmd_vel_series,
        expected_frequency=metric_cfg.get("cmd_vel_expected_frequency", 10.0),
        dropout_factor=metric_cfg.get("cmd_vel_dropout_factor", 2.0),
    )
    costmap_rate = compute_costmap_update_rate(costmap_series)
    planning_to_control = compute_planning_to_control_latency(local_plan_series, cmd_vel_series)

    smoothness = summarize_smoothness(jerk_series, jerk_series, curvature_series, tracking_series, max_steering_angle=vehicle_cfg.get("max_steering_angle"))
    turning_violation = compute_turning_radius_violation(curvature_series, vehicle_cfg.get("min_turning_radius", 3.0))

    completion_time = task_summary.get("completion_time")
    average_speed = math.nan
    if completion_time and not math.isnan(completion_time) and completion_time > 1e-9:
        average_speed = path_length_summary["actual_path_length"] / completion_time

    relocalization_time = math.nan
    if degradation_segments:
        relocalization_time = degradation_segments[0]["end"] - degradation_segments[0]["start"]

    summary = {
        "test_id": test_id,
        **task_summary,
        **path_length_summary,
        **summarize_cte(cte_series),
        **summarize_heading_error(heading_series),
        **final_pose_summary,
        "min_clearance": min_clearance,
        "near_miss_count": near_miss_count,
        "safety_stop_count": safety_stop_count,
        **smoothness,
        **turning_violation,
        **rtk_rates,
        **localization_dropout,
        **tf_dropout,
        "relocalization_time": relocalization_time,
        **ate_summary,
        **rpe_summary,
        **global_plan_timing,
        **local_plan_timing,
        **replanning,
        **control_frequency,
        **cmd_dropout,
        **planning_to_control,
        **costmap_rate,
        "average_speed": average_speed,
        "cpu_usage_mean": math.nan,
        "memory_usage_mean": math.nan,
    }

    timeseries = []
    for item in cte_series:
        row = {"t": item["t"], "cte": item["cte"]}
        timeseries.append(row)
    for index, item in enumerate(heading_series):
        if index < len(timeseries):
            timeseries[index]["heading_error"] = item["heading_error"]
        else:
            timeseries.append({"t": item["t"], "heading_error": item["heading_error"]})
    for index, item in enumerate(clearance_series):
        if index < len(timeseries):
            timeseries[index]["clearance"] = item["clearance"]
        else:
            timeseries.append({"t": item["t"], "clearance": item["clearance"]})
    report_md = build_markdown_report(summary, warnings, anomalies=manual_events)
    return {"summary": summary, "timeseries": timeseries, "warnings": warnings, "report_md": report_md}


def compute_metrics_from_csv(actual_csv, reference_csv, output_dir, topics_path, vehicle_path, metrics_path, test_id="csv_input"):
    configs = load_all_configs(topics_path, vehicle_path, metrics_path)
    actual_rows = load_csv_rows(actual_csv)
    reference_rows = load_csv_rows(reference_csv)
    dataset = {
        "warnings": [],
        "goal_series": [_csv_pose_rows([reference_rows[-1]])[0]] if reference_rows else [],
        "result_series": [],
        "status_series": [],
        "global_plan_series": [{"t": float(reference_rows[0]["t"]), "poses": _csv_pose_rows(reference_rows)}] if reference_rows else [],
        "local_plan_series": [],
        "cmd_vel_series": [],
        "odom_series": _csv_pose_rows(actual_rows, include_velocity=True),
        "local_odom_series": [],
        "filtered_odom_series": _csv_pose_rows(actual_rows, include_velocity=True),
        "tf_series": [],
        "costmap_series": [],
        "rtk_status_series": [],
        "rosout_series": [],
        "gt_series": [],
    }
    result = compute_metrics_from_dataset(dataset, configs, test_id=test_id, manual_events=[])
    write_outputs(output_dir, result)
    return result


def write_outputs(output_dir, result):
    ensure_dir(output_dir)
    write_single_row_csv(os.path.join(output_dir, "metrics_summary.csv"), result["summary"])
    fieldnames = sorted({key for row in result["timeseries"] for key in row.keys()}) if result["timeseries"] else ["t"]
    from agv_nav_evaluation.metrics_common import write_rows_csv

    write_rows_csv(os.path.join(output_dir, "metrics_timeseries.csv"), result["timeseries"], fieldnames)
    write_json(os.path.join(output_dir, "metrics_summary.json"), {"summary": result["summary"], "warnings": result["warnings"]})
    with open(os.path.join(output_dir, "report.md"), "w", encoding="utf-8") as handle:
        handle.write(result["report_md"])


def _csv_pose_rows(rows, include_velocity=False):
    converted = []
    for row in rows:
        item = {
            "t": float(row.get("t", row.get("timestamp", 0.0))),
            "x": float(row.get("x", 0.0)),
            "y": float(row.get("y", 0.0)),
            "yaw": float(row.get("yaw", 0.0)),
        }
        if include_velocity:
            item["v"] = float(row.get("v", 0.0))
            item["omega"] = float(row.get("omega", 0.0))
        converted.append(item)
    return converted


def main():
    parser = argparse.ArgumentParser(description="Compute AGV navigation metrics from CSV exports.")
    parser.add_argument("--actual", help="Actual trajectory CSV")
    parser.add_argument("--reference", help="Reference path CSV")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--config", required=True, help="topics.yaml path")
    parser.add_argument("--vehicle", required=True, help="vehicle.yaml path")
    parser.add_argument("--metrics", required=True, help="metrics.yaml path")
    parser.add_argument("--test-id", default="csv_input")
    args = parser.parse_args()
    compute_metrics_from_csv(args.actual, args.reference, args.output, args.config, args.vehicle, args.metrics, test_id=args.test_id)


if __name__ == "__main__":
    main()
