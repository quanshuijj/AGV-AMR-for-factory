import math


STATUS_SUCCEEDED = 3
STATUS_ABORTED = 4
STATUS_REJECTED = 5
STATUS_PREEMPTED = 2


def compute_task_metrics(goal_series, status_series, result_series, manual_events, metric_cfg):
    del status_series
    del metric_cfg
    success = False
    failure_type = "unavailable"
    failure_count = 0
    if result_series:
        final_status = result_series[-1].get("status")
        success = final_status == STATUS_SUCCEEDED
        if final_status in (STATUS_ABORTED, STATUS_REJECTED, STATUS_PREEMPTED):
            failure_count = 1
            failure_type = str(final_status)
    collision_count = len([event for event in manual_events if event.get("event_type") == "collision"])
    emergency_stop_count = len([event for event in manual_events if event.get("event_type") == "emergency_stop"])
    human_intervention_count = len([event for event in manual_events if event.get("event_type") == "human_intervention"])
    completion_time = math.nan
    if goal_series and result_series:
        completion_time = result_series[-1]["t"] - goal_series[0]["t"]
    return {
        "success": int(success),
        "success_rate": 1.0 if success else 0.0,
        "failure_count": failure_count,
        "failure_type": failure_type,
        "human_intervention_count": human_intervention_count,
        "emergency_stop_count": emergency_stop_count,
        "collision_count": collision_count,
        "completion_time": completion_time,
    }
