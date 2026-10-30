import math

from agv_nav_evaluation.metrics_common import safe_mean, safe_percentile
from agv_nav_evaluation.time_utils import mean_rate_from_times


def compute_global_planning_time(goal_series, global_plan_series, rosout_series=None, timeout=10.0):
    del rosout_series
    values = []
    for goal in goal_series:
        candidates = [plan for plan in global_plan_series if plan["t"] >= goal["t"] and plan["t"] - goal["t"] <= timeout]
        if candidates:
            values.append(candidates[0]["t"] - goal["t"])
    return {
        "global_planning_time_mean": safe_mean(values),
        "global_planning_time_p95": safe_percentile(values, 95.0),
        "planning_success_rate": (len(values) / float(len(goal_series))) if goal_series else math.nan,
        "planning_failure_count": (len(goal_series) - len(values)) if goal_series else math.nan,
    }


def compute_local_planning_time(local_plan_series, cmd_vel_series, rosout_series=None, timeout=2.0):
    del rosout_series
    values = []
    for plan in local_plan_series:
        candidates = [cmd for cmd in cmd_vel_series if cmd["t"] >= plan["t"] and cmd["t"] - plan["t"] <= timeout]
        if candidates:
            values.append(candidates[0]["t"] - plan["t"])
    return {
        "local_planning_time_mean": safe_mean(values),
        "local_planning_time_p95": safe_percentile(values, 95.0),
    }


def compute_replanning_count(plan_series):
    if not plan_series:
        return {"replanning_count": math.nan, "replanning_frequency": math.nan}
    times = [item["t"] for item in plan_series]
    count = max(0, len(times) - 1)
    duration = max(times) - min(times) if len(times) > 1 else math.nan
    frequency = count / duration if duration and not math.isnan(duration) and duration > 0 else math.nan
    return {"replanning_count": count, "replanning_frequency": frequency}


def compute_control_loop_frequency(cmd_vel_series):
    times = [item["t"] for item in cmd_vel_series]
    return {"control_loop_frequency_mean": mean_rate_from_times(times)}


def compute_cmd_vel_dropout(cmd_vel_series, expected_frequency, dropout_factor):
    if not cmd_vel_series or len(cmd_vel_series) < 2:
        return {"cmd_vel_dropout_count": math.nan}
    timeout = (1.0 / expected_frequency) * dropout_factor
    count = 0
    for idx in range(1, len(cmd_vel_series)):
        if cmd_vel_series[idx]["t"] - cmd_vel_series[idx - 1]["t"] > timeout:
            count += 1
    return {"cmd_vel_dropout_count": count}


def compute_costmap_update_rate(costmap_series):
    times = [item["t"] for item in costmap_series]
    return {"costmap_update_rate": mean_rate_from_times(times)}


def compute_planning_to_control_latency(local_plan_series, cmd_vel_series):
    values = []
    for plan in local_plan_series:
        candidates = [cmd for cmd in cmd_vel_series if cmd["t"] >= plan["t"]]
        if candidates:
            values.append(candidates[0]["t"] - plan["t"])
    return {"planning_to_control_latency_mean": safe_mean(values)}
