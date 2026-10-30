import math

try:
    import rosbag
except ImportError:  # pragma: no cover
    rosbag = None

from agv_nav_evaluation.time_utils import to_sec


class RosbagLoader(object):
    def __init__(self, bag_path, topics_config):
        self.bag_path = bag_path
        self.topics_config = topics_config
        self.warnings = []

    def load(self):
        if rosbag is None:
            raise RuntimeError("rosbag module is not available in this environment")
        data = {
            "goal_series": [],
            "status_series": [],
            "result_series": [],
            "feedback_series": [],
            "global_plan_series": [],
            "local_plan_series": [],
            "cmd_vel_series": [],
            "odom_series": [],
            "local_odom_series": [],
            "filtered_odom_series": [],
            "tf_series": [],
            "costmap_series": [],
            "rtk_status_series": [],
            "rosout_series": [],
            "gt_series": [],
        }
        topic_map = self.topics_config.get("topics", {})
        optional_map = self.topics_config.get("optional_topics", {})
        watched = set(filter(None, list(topic_map.values()) + list(optional_map.values())))
        with rosbag.Bag(self.bag_path, "r") as bag:
            info = bag.get_type_and_topic_info()[1]
            available_topics = set(info.keys())
            for topic in watched:
                if topic not in available_topics:
                    self.warnings.append("Topic missing in bag: %s" % topic)
            for topic, msg, timestamp in bag.read_messages():
                t = to_sec(getattr(msg, "header", None).stamp) if hasattr(msg, "header") and getattr(msg.header, "stamp", None) else to_sec(timestamp)
                if topic == topic_map.get("goal"):
                    data["goal_series"].append(_pose_stamped_to_pose(msg, t))
                elif topic == topic_map.get("move_base_status"):
                    data["status_series"].append({"t": t, "status_list": [status.status for status in getattr(msg, "status_list", [])]})
                elif topic == topic_map.get("move_base_result"):
                    data["result_series"].append({"t": t, "status": getattr(getattr(msg, "status", None), "status", math.nan)})
                elif topic == topic_map.get("move_base_feedback"):
                    base_pose = getattr(getattr(getattr(msg, "feedback", None), "base_position", None), "pose", None)
                    if base_pose is not None:
                        data["feedback_series"].append(_pose_to_dict(base_pose, t))
                elif topic == topic_map.get("global_plan"):
                    data["global_plan_series"].append({"t": t, "poses": [_pose_stamped_to_pose(pose, to_sec(getattr(pose.header, "stamp", timestamp))) for pose in getattr(msg, "poses", [])]})
                elif topic == topic_map.get("local_plan"):
                    data["local_plan_series"].append({"t": t, "poses": [_pose_stamped_to_pose(pose, to_sec(getattr(pose.header, "stamp", timestamp))) for pose in getattr(msg, "poses", [])]})
                elif topic == topic_map.get("cmd_vel"):
                    data["cmd_vel_series"].append({"t": t, "v": getattr(msg.linear, "x", 0.0), "omega": getattr(msg.angular, "z", 0.0)})
                elif topic == topic_map.get("odom") and hasattr(msg, "pose") and hasattr(msg, "twist"):
                    pose = _odom_to_pose_velocity(msg, t)
                    data["local_odom_series"].append(pose)
                elif topic == topic_map.get("filtered_odom") and hasattr(msg, "pose") and hasattr(msg, "twist"):
                    pose = _odom_to_pose_velocity(msg, t)
                    data["filtered_odom_series"].append(pose)
                elif topic in [topic_map.get("tf"), topic_map.get("tf_static")]:
                    for transform in getattr(msg, "transforms", []):
                        data["tf_series"].append({"t": t, "parent": transform.header.frame_id, "child": transform.child_frame_id})
                elif topic in [topic_map.get("local_costmap"), topic_map.get("global_costmap")]:
                    data["costmap_series"].append(_costmap_to_points(msg, t))
                elif topic == topic_map.get("rtk_status"):
                    data["rtk_status_series"].append({"t": t, "value": _extract_generic_value(msg)})
                elif topic == topic_map.get("rosout"):
                    data["rosout_series"].append({"t": t, "text": getattr(msg, "msg", "")})
                elif topic == optional_map.get("ground_truth_pose"):
                    data["gt_series"].append(_pose_stamped_to_pose(msg, t))
        data["odom_series"] = data["filtered_odom_series"] if data["filtered_odom_series"] else data["local_odom_series"]
        return data, self.warnings


def _extract_generic_value(msg):
    for field_name in ["data", "status", "mode", "state", "level"]:
        if hasattr(msg, field_name):
            return getattr(msg, field_name)
    return str(msg)


def _pose_stamped_to_pose(msg, timestamp):
    pose = getattr(msg, "pose", msg)
    return _pose_to_dict(pose, timestamp)


def _pose_to_dict(pose, timestamp):
    orientation = pose.orientation
    yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
    return {"t": timestamp, "x": pose.position.x, "y": pose.position.y, "yaw": yaw}


def _odom_to_pose_velocity(msg, timestamp):
    pose = _pose_to_dict(msg.pose.pose, timestamp)
    pose["v"] = msg.twist.twist.linear.x
    pose["omega"] = msg.twist.twist.angular.z
    return pose


def _costmap_to_points(msg, timestamp):
    occupied_points = []
    resolution = msg.info.resolution
    width = msg.info.width
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    for index, cost in enumerate(msg.data):
        if cost < 90:
            continue
        x_idx = index % width
        y_idx = index // width
        occupied_points.append((origin_x + x_idx * resolution, origin_y + y_idx * resolution))
    return {"t": timestamp, "occupied_points": occupied_points}
