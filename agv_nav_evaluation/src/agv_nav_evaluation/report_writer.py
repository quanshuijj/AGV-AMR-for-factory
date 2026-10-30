def build_markdown_report(summary, warnings, anomalies=None):
    anomalies = anomalies or []
    lines = []
    lines.append("# AGV Navigation Evaluation Report")
    lines.append("")
    lines.append("## Experiment Basic Information")
    lines.append("")
    lines.append(f"- test_id: {summary.get('test_id', 'unknown')}")
    lines.append(f"- success: {summary.get('success')}")
    lines.append(f"- completion_time_s: {summary.get('completion_time')}")
    lines.append("")
    sections = [
        ("Task Success", ["success_rate", "failure_count", "failure_type", "human_intervention_count", "emergency_stop_count", "collision_count"]),
        ("Task Efficiency", ["actual_path_length", "reference_path_length", "path_length_ratio", "average_speed", "replanning_count", "replanning_frequency"]),
        ("Path Tracking", ["cte_mean", "cte_rmse", "cte_p95", "cte_max", "heading_error_rmse", "heading_error_p95", "final_position_error", "final_heading_error"]),
        ("Safety", ["min_clearance", "near_miss_count", "collision_count", "safety_stop_count"]),
        ("Smoothness / Executability", ["linear_jerk_rms", "angular_jerk_rms", "max_curvature", "min_turning_radius_actual", "turning_radius_violation_rate", "steering_saturation_count", "velocity_tracking_rmse", "steering_tracking_rmse", "control_effort"]),
        ("Localization and Robustness", ["rtk_fix_rate", "rtk_float_rate", "rtk_invalid_rate", "localization_dropout_count", "tf_dropout_count", "relocalization_time", "ate_rmse", "rpe_rmse", "ate_status"]),
        ("Planning and Control Real-Time Performance", ["global_planning_time_mean", "global_planning_time_p95", "local_planning_time_mean", "local_planning_time_p95", "planning_success_rate", "planning_failure_count", "control_loop_frequency_mean", "cmd_vel_dropout_count", "planning_to_control_latency_mean", "costmap_update_rate", "cpu_usage_mean", "memory_usage_mean"]),
    ]
    for title, keys in sections:
        lines.append(f"## {title}")
        lines.append("")
        for key in keys:
            lines.append(f"- {key}: {summary.get(key, 'unavailable')}")
        lines.append("")
    lines.append("## Exception / Warning List")
    lines.append("")
    if warnings:
        for warning in warnings:
            lines.append(f"- {warning}")
    else:
        lines.append("- none")
    lines.append("")
    lines.append("## Anomaly Events")
    lines.append("")
    if anomalies:
        for event in anomalies:
            lines.append(f"- t={event.get('t', 'n/a')}: {event.get('type', 'unknown')} {event.get('description', '')}")
    else:
        lines.append("- none")
    lines.append("")
    lines.append("## Metric Notes")
    lines.append("")
    lines.append("- time: s")
    lines.append("- distance: m")
    lines.append("- angle: rad")
    lines.append("- speed: m/s")
    lines.append("- acceleration: m/s^2")
    lines.append("- jerk: m/s^3")
    lines.append("- curvature: 1/m")
    lines.append("- frequency: Hz")
    return "\n".join(lines) + "\n"
