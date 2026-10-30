#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Record global path planning metrics to CSV.

This node is intentionally independent from the navigation controller. It only
subscribes to goal/path/costmap topics and computes metrics for experiments.
"""

import csv
import math
import os
from statistics import mean, pstdev

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


def safe_mean(values, default=0.0):
    return mean(values) if values else default


def safe_std(values, default=0.0):
    return pstdev(values) if len(values) > 1 else default


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_line_distance(point, start, end):
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    wx = point[0] - start[0]
    wy = point[1] - start[1]
    denom = math.hypot(vx, vy)
    if denom < 1e-6:
        return dist2d(point, start)
    return abs(vx * wy - vy * wx) / denom


class GlobalPathMetricsRecorder:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/move_base_node/SBPLLatticePlanner/plan")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.costmap_topic = rospy.get_param("~costmap_topic", "/move_base_node/global_costmap/costmap")
        self.output_csv = os.path.expanduser(
            rospy.get_param("~output_csv", "~/navCarProject_ws/logs/global_path_metrics.csv")
        )

        self.nominal_velocity = float(rospy.get_param("~nominal_velocity", 0.9))
        self.min_turn_radius = float(rospy.get_param("~min_turn_radius", 3.2))
        self.straight_curvature_threshold = float(rospy.get_param("~straight_curvature_threshold", 0.05))
        self.cost_threshold = int(rospy.get_param("~cost_threshold", 80))
        self.record_first_path_after_goal_only = bool(rospy.get_param("~record_first_path_after_goal_only", False))

        self.latest_costmap = None
        self.goal_seq = 0
        self.last_goal_time = None
        self.recorded_for_current_goal = False

        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.csv_file = open(self.output_csv, "a", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_header())
        if self.csv_file.tell() == 0:
            self.writer.writeheader()
            self.csv_file.flush()

        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1)

        rospy.loginfo("Global path metrics recorder started.")
        rospy.loginfo("  path_topic: %s", self.path_topic)
        rospy.loginfo("  goal_topic: %s", self.goal_topic)
        rospy.loginfo("  costmap_topic: %s", self.costmap_topic)
        rospy.loginfo("  output_csv: %s", self.output_csv)

    @staticmethod
    def csv_header():
        return [
            "time",
            "goal_seq",
            "path_topic",
            "frame_id",
            "point_count",
            "planning_time_s",
            "path_length_m",
            "path_cost_sum",
            "path_cost_mean",
            "path_cost_max",
            "unknown_cost_ratio",
            "lethal_cost_ratio",
            "straight_lateral_error_mean_m",
            "straight_lateral_error_max_m",
            "turn_curvature_mean_1pm",
            "turn_curvature_max_1pm",
            "turn_curvature_std_1pm",
            "curvature_over_limit_ratio",
            "yaw_change_sum_rad",
            "yaw_change_mean_rad",
            "yaw_change_max_rad",
            "yaw_change_std_rad",
            "curvature_change_mean_1pm",
            "curvature_change_std_1pm",
            "omega_change_mean_radps",
            "omega_change_std_radps",
        ]

    def goal_callback(self, _msg):
        self.goal_seq += 1
        self.last_goal_time = rospy.Time.now()
        self.recorded_for_current_goal = False

    def costmap_callback(self, msg):
        self.latest_costmap = msg

    def path_callback(self, msg):
        if not msg.poses:
            return
        if self.record_first_path_after_goal_only and self.recorded_for_current_goal:
            return

        row = self.compute_metrics(msg)
        self.writer.writerow(row)
        self.csv_file.flush()
        self.recorded_for_current_goal = True

        rospy.loginfo(
            "Path metrics: points=%d length=%.2fm yaw_sum=%.3f curvature_max=%.3f cost_mean=%.1f",
            row["point_count"],
            row["path_length_m"],
            row["yaw_change_sum_rad"],
            row["turn_curvature_max_1pm"],
            row["path_cost_mean"],
        )

    def compute_metrics(self, msg):
        now = rospy.Time.now()
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        point_count = len(points)

        segment_lengths = []
        yaws = []
        for i in range(point_count - 1):
            ds = dist2d(points[i], points[i + 1])
            segment_lengths.append(ds)
            if ds > 1e-6:
                yaws.append(math.atan2(points[i + 1][1] - points[i][1], points[i + 1][0] - points[i][0]))

        path_length = sum(segment_lengths)

        yaw_changes = []
        curvatures = []
        for i in range(len(yaws) - 1):
            dpsi = abs(normalize_angle(yaws[i + 1] - yaws[i]))
            ds = 0.5 * (segment_lengths[i] + segment_lengths[i + 1]) if i + 1 < len(segment_lengths) else 0.0
            yaw_changes.append(dpsi)
            if ds > 1e-6:
                curvatures.append(dpsi / ds)

        curvature_limit = 1.0 / max(self.min_turn_radius, 1e-3)
        turning_curvatures = [abs(k) for k in curvatures if abs(k) >= self.straight_curvature_threshold]
        over_limit_count = len([k for k in curvatures if abs(k) > curvature_limit])

        straight_errors = []
        for i in range(1, point_count - 1):
            k = curvatures[i - 1] if i - 1 < len(curvatures) else 0.0
            if abs(k) <= self.straight_curvature_threshold:
                straight_errors.append(point_line_distance(points[i], points[i - 1], points[i + 1]))

        curvature_changes = [abs(curvatures[i + 1] - curvatures[i]) for i in range(len(curvatures) - 1)]
        omega_changes = [self.nominal_velocity * dc for dc in curvature_changes]

        cost_stats = self.compute_cost_metrics(msg)

        planning_time = -1.0
        if self.last_goal_time is not None:
            planning_time = max(0.0, (now - self.last_goal_time).to_sec())

        return {
            "time": now.to_sec(),
            "goal_seq": self.goal_seq,
            "path_topic": self.path_topic,
            "frame_id": msg.header.frame_id,
            "point_count": point_count,
            "planning_time_s": planning_time,
            "path_length_m": path_length,
            "path_cost_sum": cost_stats["sum"],
            "path_cost_mean": cost_stats["mean"],
            "path_cost_max": cost_stats["max"],
            "unknown_cost_ratio": cost_stats["unknown_ratio"],
            "lethal_cost_ratio": cost_stats["lethal_ratio"],
            "straight_lateral_error_mean_m": safe_mean(straight_errors),
            "straight_lateral_error_max_m": max(straight_errors) if straight_errors else 0.0,
            "turn_curvature_mean_1pm": safe_mean(turning_curvatures),
            "turn_curvature_max_1pm": max(turning_curvatures) if turning_curvatures else 0.0,
            "turn_curvature_std_1pm": safe_std(turning_curvatures),
            "curvature_over_limit_ratio": (float(over_limit_count) / len(curvatures)) if curvatures else 0.0,
            "yaw_change_sum_rad": sum(yaw_changes),
            "yaw_change_mean_rad": safe_mean(yaw_changes),
            "yaw_change_max_rad": max(yaw_changes) if yaw_changes else 0.0,
            "yaw_change_std_rad": safe_std(yaw_changes),
            "curvature_change_mean_1pm": safe_mean(curvature_changes),
            "curvature_change_std_1pm": safe_std(curvature_changes),
            "omega_change_mean_radps": safe_mean(omega_changes),
            "omega_change_std_radps": safe_std(omega_changes),
        }

    def compute_cost_metrics(self, msg):
        if self.latest_costmap is None or not self.latest_costmap.data:
            return {"sum": -1.0, "mean": -1.0, "max": -1.0, "unknown_ratio": -1.0, "lethal_ratio": -1.0}

        costmap = self.latest_costmap
        if msg.header.frame_id and costmap.header.frame_id and msg.header.frame_id != costmap.header.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "Path frame '%s' differs from costmap frame '%s'; cost metrics may be invalid.",
                msg.header.frame_id,
                costmap.header.frame_id,
            )

        values = []
        unknown_count = 0
        lethal_count = 0
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        resolution = costmap.info.resolution
        width = costmap.info.width
        height = costmap.info.height

        for pose in msg.poses:
            mx = int((pose.pose.position.x - origin_x) / resolution)
            my = int((pose.pose.position.y - origin_y) / resolution)
            if mx < 0 or mx >= width or my < 0 or my >= height:
                unknown_count += 1
                continue

            cost = costmap.data[my * width + mx]
            if cost < 0:
                unknown_count += 1
                continue

            values.append(cost)
            if cost >= self.cost_threshold:
                lethal_count += 1

        sample_count = len(msg.poses)
        return {
            "sum": float(sum(values)) if values else -1.0,
            "mean": safe_mean(values, -1.0),
            "max": float(max(values)) if values else -1.0,
            "unknown_ratio": float(unknown_count) / sample_count if sample_count else -1.0,
            "lethal_ratio": float(lethal_count) / sample_count if sample_count else -1.0,
        }

    def close(self):
        self.csv_file.close()


if __name__ == "__main__":
    rospy.init_node("global_path_metrics_recorder")
    recorder = GlobalPathMetricsRecorder()
    rospy.on_shutdown(recorder.close)
    rospy.spin()
