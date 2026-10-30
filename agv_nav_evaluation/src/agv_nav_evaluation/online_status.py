import math
import time

import rospy
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from agv_nav_evaluation.config_loader import load_all_configs
from agv_nav_evaluation.topic_utils import get_topic
from agv_nav_evaluation.time_utils import mean_rate_from_times


class OnlineStatusMonitor(object):
    def __init__(self, topics_path, vehicle_path, metrics_path):
        self.configs = load_all_configs(topics_path, vehicle_path, metrics_path)
        self.topic_cfg = self.configs["topics"]
        self.metrics_cfg = (self.configs["metrics"] or {}).get("metrics", {})
        self.goal_received = False
        self.move_base_state = "unknown"
        self.cmd_times = []
        self.global_plan_time = None
        self.local_plan_time = None
        self.rtk_state = "unavailable"
        self.tf_last_time = None
        self.can_last_time = None
        self.last_print = time.time()

    def start(self):
        self._subscribe(PoseStamped, "goal", self._goal_cb)
        self._subscribe(GoalStatusArray, "move_base_status", self._status_cb)
        self._subscribe(Twist, "cmd_vel", self._cmd_cb)
        self._subscribe(Path, "global_plan", self._global_plan_cb)
        self._subscribe(Path, "local_plan", self._local_plan_cb)
        self._subscribe(TFMessage, "tf", self._tf_cb)
        self._subscribe(String, "rtk_status", self._rtk_cb)
        self._subscribe(Odometry, "odom", self._noop)
        self._subscribe(Imu, "imu", self._noop)
        self._subscribe(OccupancyGrid, "local_costmap", self._noop)
        can_topic = get_topic(self.topic_cfg, "can_feedback")
        if can_topic:
            rospy.Subscriber(can_topic, String, self._can_cb, queue_size=10)
        diag_topic = get_topic(self.topic_cfg, "diagnostics")
        if diag_topic:
            rospy.Subscriber(diag_topic, DiagnosticArray, self._diag_cb, queue_size=10)
        timer = rospy.Timer(rospy.Duration(1.0), self._print_status)
        rospy.spin()
        timer.shutdown()

    def _subscribe(self, msg_type, key, callback):
        topic = get_topic(self.topic_cfg, key)
        if topic:
            rospy.Subscriber(topic, msg_type, callback, queue_size=10)
        else:
            rospy.logwarn("Topic config missing for key: %s", key)

    def _goal_cb(self, msg):
        del msg
        self.goal_received = True

    def _status_cb(self, msg):
        if msg.status_list:
            self.move_base_state = str(msg.status_list[-1].status)

    def _cmd_cb(self, msg):
        del msg
        now = rospy.get_time()
        self.cmd_times.append(now)
        self.cmd_times = self.cmd_times[-100:]

    def _global_plan_cb(self, msg):
        del msg
        self.global_plan_time = rospy.get_time()

    def _local_plan_cb(self, msg):
        del msg
        self.local_plan_time = rospy.get_time()

    def _tf_cb(self, msg):
        del msg
        self.tf_last_time = rospy.get_time()

    def _rtk_cb(self, msg):
        self.rtk_state = getattr(msg, "data", str(msg))

    def _can_cb(self, msg):
        del msg
        self.can_last_time = rospy.get_time()

    def _diag_cb(self, msg):
        if msg.status:
            self.rtk_state = msg.status[0].name

    def _noop(self, msg):
        del msg

    def _print_status(self, event):
        del event
        now = rospy.get_time()
        cmd_rate = mean_rate_from_times(self.cmd_times)
        tf_ok = self.tf_last_time is not None and (now - self.tf_last_time) <= self.metrics_cfg.get("tf_timeout", 0.5)
        can_ok = self.can_last_time is not None and (now - self.can_last_time) <= 2.0
        global_ok = self.global_plan_time is not None and (now - self.global_plan_time) <= 2.0
        local_ok = self.local_plan_time is not None and (now - self.local_plan_time) <= 2.0
        print("")
        print("=== AGV Online Evaluation Monitor ===")
        print("goal_received: %s" % self.goal_received)
        print("move_base_status: %s" % self.move_base_state)
        print("cmd_vel_frequency_hz: %.3f" % cmd_rate if not math.isnan(cmd_rate) else "cmd_vel_frequency_hz: unavailable")
        print("global_plan_updated_recently: %s" % global_ok)
        print("local_plan_updated_recently: %s" % local_ok)
        print("rtk_state: %s" % self.rtk_state)
        print("tf_ok: %s" % tf_ok)
        print("can_feedback_ok: %s" % can_ok)
