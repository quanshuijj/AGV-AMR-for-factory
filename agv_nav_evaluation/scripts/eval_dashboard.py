#!/usr/bin/env python3
import csv
import datetime
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import traceback

import rospy
import yaml
from actionlib_msgs.msg import GoalID, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from agv_nav_evaluation.config_loader import load_yaml_file
from agv_nav_evaluation.topic_utils import collect_record_topics, get_alias_list

try:
    from PyQt5.QtCore import QThread, QTimer, pyqtSignal
    from PyQt5.QtWidgets import (
        QApplication,
        QComboBox,
        QFileDialog,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QTableWidget,
        QTableWidgetItem,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:  # pragma: no cover
    print("PyQt5 is required to run eval_dashboard.py: %s" % exc, file=sys.stderr)
    sys.exit(1)


TOPIC_KEYS_TO_MONITOR = [
    "tf",
    "filtered_odom",
    "cmd_vel",
    "global_plan",
    "local_plan",
    "move_base_status",
    "gnss_fix",
    "rtk_status",
    "imu",
    "can_feedback",
    "emergency_stop",
]

EVENT_TYPES = [
    "碰撞",
    "险些碰撞",
    "紧急停止",
    "人工干预",
    "定位丢失",
    "规划失败",
    "控制器失败",
    "障碍物阻挡",
    "其他",
]


def _yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _safe_ros_time():
    try:
        return rospy.get_time()
    except Exception:
        return 0.0


def _read_git_commit(cwd):
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        return ""
    return ""


class CommandWorker(QThread):
    finished_with_result = pyqtSignal(bool, str)

    def __init__(self, command, workdir=None):
        super().__init__()
        self.command = command
        self.workdir = workdir

    def run(self):
        try:
            result = subprocess.run(
                self.command,
                cwd=self.workdir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            output = (result.stdout or "") + ("\n" if result.stdout and result.stderr else "") + (result.stderr or "")
            self.finished_with_result.emit(result.returncode == 0, output.strip())
        except Exception as exc:
            self.finished_with_result.emit(False, str(exc))


class EvalDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV 导航评测仪表板")
        self.resize(1500, 960)

        self.package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.topics_config_path = os.path.join(self.package_dir, "config", "topics.yaml")
        self.vehicle_config_path = os.path.join(self.package_dir, "config", "vehicle.yaml")
        self.metrics_config_path = os.path.join(self.package_dir, "config", "metrics.yaml")
        self.output_root_default = os.path.join(self.package_dir, "output")

        self.topics_cfg = load_yaml_file(self.topics_config_path)
        self.current_bag_process = None
        self.current_bag_prefix = ""
        self.current_bag_file = ""
        self.current_bag_dir = ""
        self.current_metadata_file = ""
        self.current_events_file = ""
        self.recording_started_wall = None
        self.recording_started_ros = None
        self.recorded_topics = []
        self.events = []
        self.process_workers = []

        self.topic_state = {}
        self.current_pose = {"x": math.nan, "y": math.nan, "yaw": math.nan}
        self.current_velocity = {"v": math.nan, "omega": math.nan}
        self.move_base_state = "Unknown"
        self.rtk_state = "Unknown"
        self.cmd_vel_dropout = True
        self.tf_ok = False
        self.can_ok = False
        self.emergency_stop_active = False
        self.human_intervention_active = False

        self._build_ui()
        self._init_topic_state()
        self._subscribe_topics()
        self._start_timers()
        self._log("仪表板以只读模式启动。不会发布控制话题。")

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        top_layout = QHBoxLayout()
        main_layout.addLayout(top_layout)

        info_group = QGroupBox("实验信息")
        info_layout = QFormLayout(info_group)
        self.test_id_edit = QLineEdit()
        self.route_name_edit = QLineEdit()
        self.map_id_edit = QLineEdit()
        self.vehicle_id_edit = QLineEdit()
        self.operator_edit = QLineEdit()
        self.weather_edit = QLineEdit()
        self.notes_edit = QTextEdit()
        self.notes_edit.setMaximumHeight(90)
        self.output_dir_edit = QLineEdit(self.output_root_default)
        browse_button = QPushButton("浏览")
        browse_button.clicked.connect(self._choose_output_dir)
        output_row = QHBoxLayout()
        output_row.addWidget(self.output_dir_edit)
        output_row.addWidget(browse_button)

        info_layout.addRow("测试ID", self.test_id_edit)
        info_layout.addRow("路线名称", self.route_name_edit)
        info_layout.addRow("地图ID", self.map_id_edit)
        info_layout.addRow("车辆ID", self.vehicle_id_edit)
        info_layout.addRow("操作员", self.operator_edit)
        info_layout.addRow("天气/场景", self.weather_edit)
        info_layout.addRow("备注", self.notes_edit)
        info_layout.addRow("Bag输出目录", output_row)
        top_layout.addWidget(info_group, 2)

        record_group = QGroupBox("录制控制")
        record_layout = QVBoxLayout(record_group)
        buttons_layout = QGridLayout()
        self.start_button = QPushButton("开始录制")
        self.stop_button = QPushButton("停止录制")
        self.mark_event_button = QPushButton("标记事件")
        self.save_events_button = QPushButton("保存事件")
        self.analyze_button = QPushButton("分析当前Bag")
        self.report_button = QPushButton("生成报告")
        self.open_folder_button = QPushButton("打开输出文件夹")
        self.stop_button.setEnabled(False)
        self.mark_event_button.setEnabled(False)
        self.save_events_button.setEnabled(False)
        self.analyze_button.setEnabled(False)
        self.report_button.setEnabled(False)
        self.open_folder_button.setEnabled(True)

        self.start_button.clicked.connect(self.start_recording)
        self.stop_button.clicked.connect(self.stop_recording)
        self.mark_event_button.clicked.connect(lambda: self._mark_event("other"))
        self.save_events_button.clicked.connect(self.save_events_csv)
        self.analyze_button.clicked.connect(self.analyze_current_bag)
        self.report_button.clicked.connect(self.generate_report)
        self.open_folder_button.clicked.connect(self.open_output_folder)

        buttons_layout.addWidget(self.start_button, 0, 0)
        buttons_layout.addWidget(self.stop_button, 0, 1)
        buttons_layout.addWidget(self.mark_event_button, 0, 2)
        buttons_layout.addWidget(self.save_events_button, 0, 3)
        buttons_layout.addWidget(self.analyze_button, 1, 0)
        buttons_layout.addWidget(self.report_button, 1, 1)
        buttons_layout.addWidget(self.open_folder_button, 1, 2)
        record_layout.addLayout(buttons_layout)

        self.record_status_label = QLabel("状态: 空闲")
        self.bag_name_label = QLabel("Bag文件: N/A")
        self.duration_label = QLabel("时长: 0.0 秒")
        self.bag_size_label = QLabel("Bag大小: 0 B")
        record_layout.addWidget(self.record_status_label)
        record_layout.addWidget(self.bag_name_label)
        record_layout.addWidget(self.duration_label)
        record_layout.addWidget(self.bag_size_label)
        top_layout.addWidget(record_group, 2)

        middle_layout = QHBoxLayout()
        main_layout.addLayout(middle_layout, 1)

        topic_group = QGroupBox("话题健康状态")
        topic_layout = QVBoxLayout(topic_group)
        self.topic_table = QTableWidget(0, 5)
        self.topic_table.setHorizontalHeaderLabels(["话题名称", "状态", "频率 (Hz)", "最后更新 (s)", "最后时间戳"])
        topic_layout.addWidget(self.topic_table)
        middle_layout.addWidget(topic_group, 3)

        status_group = QGroupBox("导航状态")
        status_layout = QFormLayout(status_group)
        self.move_base_value = QLabel("未知")
        self.velocity_value = QLabel("v=nan, omega=nan")
        self.yaw_value = QLabel("nan")
        self.rtk_value = QLabel("未知")
        self.cmd_dropout_value = QLabel("True")
        self.tf_value = QLabel("False")
        self.can_value = QLabel("False")
        self.estop_value = QLabel("False")
        self.manual_value = QLabel("False")
        status_layout.addRow("move_base 当前状态", self.move_base_value)
        status_layout.addRow("当前速度", self.velocity_value)
        status_layout.addRow("当前偏航角", self.yaw_value)
        status_layout.addRow("当前 RTK 状态", self.rtk_value)
        status_layout.addRow("cmd_vel 掉线", self.cmd_dropout_value)
        status_layout.addRow("TF 健康", self.tf_value)
        status_layout.addRow("CAN 在线", self.can_value)
        status_layout.addRow("紧急停止", self.estop_value)
        status_layout.addRow("人工干预", self.manual_value)
        middle_layout.addWidget(status_group, 2)

        event_group = QGroupBox("手动事件标注")
        event_layout = QVBoxLayout(event_group)
        event_buttons_layout = QGridLayout()
        for idx, event_type in enumerate(EVENT_TYPES):
            button = QPushButton(event_type)
            button.clicked.connect(lambda checked=False, event_name=event_type: self._mark_event(event_name))
            event_buttons_layout.addWidget(button, idx // 3, idx % 3)
        event_layout.addLayout(event_buttons_layout)

        self.event_description_edit = QLineEdit()
        self.event_severity_combo = QComboBox()
        self.event_severity_combo.addItems(["低", "中", "高", "严重"])
        event_layout.addWidget(QLabel("事件描述"))
        event_layout.addWidget(self.event_description_edit)
        event_layout.addWidget(QLabel("严重程度"))
        event_layout.addWidget(self.event_severity_combo)
        middle_layout.addWidget(event_group, 2)

        log_group = QGroupBox("日志输出")
        log_layout = QVBoxLayout(log_group)
        self.log_output = QPlainTextEdit()
        self.log_output.setReadOnly(True)
        log_layout.addWidget(self.log_output)
        main_layout.addWidget(log_group, 1)

    def _init_topic_state(self):
        monitored = []
        for key in TOPIC_KEYS_TO_MONITOR:
            configured = self._resolve_topic_for_key(key)
            monitored.append((key, configured))
            self.topic_state[key] = {
                "topic_name": configured or "(not configured)",
                "state": "missing",
                "msg_times": [],
                "last_ros_stamp": math.nan,
                "last_wall_stamp": None,
            }
        self.topic_table.setRowCount(len(monitored))
        for row, (key, topic_name) in enumerate(monitored):
            self.topic_table.setItem(row, 0, QTableWidgetItem(topic_name or "(not configured)"))
            self.topic_table.setItem(row, 1, QTableWidgetItem("missing"))
            self.topic_table.setItem(row, 2, QTableWidgetItem("nan"))
            self.topic_table.setItem(row, 3, QTableWidgetItem("nan"))
            self.topic_table.setItem(row, 4, QTableWidgetItem("nan"))

    def _resolve_topic_for_key(self, key):
        topic_name = (self.topics_cfg.get("topics") or {}).get(key)
        if topic_name:
            return topic_name
        alias_key = "%s_candidates" % key
        aliases = get_alias_list(self.topics_cfg, alias_key)
        return aliases[0] if aliases else ""

    def _subscribe_topics(self):
        self._subscribe_if_configured("goal", PoseStamped, self._goal_cb)
        self._subscribe_if_configured("move_base_status", GoalStatusArray, self._move_base_status_cb)
        self._subscribe_if_configured("cmd_vel", Twist, self._cmd_vel_cb)
        self._subscribe_if_configured("global_plan", Path, self._global_plan_cb)
        self._subscribe_if_configured("local_plan", Path, self._local_plan_cb)
        self._subscribe_if_configured("filtered_odom", Odometry, self._filtered_odom_cb)
        self._subscribe_if_configured("odom", Odometry, self._odom_cb)
        self._subscribe_if_configured("tf", TFMessage, self._tf_cb)
        self._subscribe_if_configured("gnss_fix", NavSatFix, self._generic_topic_cb("gnss_fix"))
        self._subscribe_if_configured("imu", Imu, self._generic_topic_cb("imu"))
        self._subscribe_if_configured("rtk_status", String, self._rtk_status_cb)
        self._subscribe_if_configured("can_feedback", String, self._can_feedback_cb)
        self._subscribe_if_configured("emergency_stop", String, self._emergency_stop_cb)
        self._subscribe_optional("diagnostics", DiagnosticArray, self._diagnostics_cb)
        self._subscribe_optional("localization_status", String, self._localization_status_cb, optional=True)

    def _subscribe_if_configured(self, key, msg_type, callback):
        topic_name = (self.topics_cfg.get("topics") or {}).get(key, "")
        if topic_name:
            rospy.Subscriber(topic_name, msg_type, callback, queue_size=10)
        else:
            self._log("Topic key '%s' is not configured. GUI will show Missing." % key)

    def _subscribe_optional(self, key, msg_type, callback, optional=False):
        topic_name = (self.topics_cfg.get("topics") or {}).get(key, "")
        if not topic_name:
            topic_name = (self.topics_cfg.get("optional_topics") or {}).get(key, "")
        if topic_name:
            rospy.Subscriber(topic_name, msg_type, callback, queue_size=10)
        elif not optional:
            self._log("Optional topic key '%s' is not configured." % key)

    def _generic_topic_cb(self, key):
        def _callback(msg):
            self._update_topic_health(key, msg)

        return _callback

    def _goal_cb(self, msg):
        self._update_topic_health("goal", msg)

    def _move_base_status_cb(self, msg):
        self._update_topic_health("move_base_status", msg)
        if msg.status_list:
            self.move_base_state = str(msg.status_list[-1].status)

    def _cmd_vel_cb(self, msg):
        self._update_topic_health("cmd_vel", msg)
        self.current_velocity["v"] = float(getattr(msg.linear, "x", math.nan))
        self.current_velocity["omega"] = float(getattr(msg.angular, "z", math.nan))

    def _global_plan_cb(self, msg):
        self._update_topic_health("global_plan", msg)

    def _local_plan_cb(self, msg):
        self._update_topic_health("local_plan", msg)

    def _filtered_odom_cb(self, msg):
        self._update_topic_health("filtered_odom", msg)
        self._update_pose_from_odom(msg)

    def _odom_cb(self, msg):
        self._update_topic_health("filtered_odom", msg)
        self._update_pose_from_odom(msg)

    def _update_pose_from_odom(self, msg):
        pose = msg.pose.pose
        self.current_pose["x"] = pose.position.x
        self.current_pose["y"] = pose.position.y
        self.current_pose["yaw"] = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.current_velocity["v"] = float(msg.twist.twist.linear.x)
        self.current_velocity["omega"] = float(msg.twist.twist.angular.z)

    def _tf_cb(self, msg):
        self._update_topic_health("tf", msg)

    def _rtk_status_cb(self, msg):
        self._update_topic_health("rtk_status", msg)
        self.rtk_state = getattr(msg, "data", str(msg))

    def _can_feedback_cb(self, msg):
        self._update_topic_health("can_feedback", msg)
        del msg

    def _emergency_stop_cb(self, msg):
        self._update_topic_health("emergency_stop", msg)
        data = getattr(msg, "data", str(msg))
        self.emergency_stop_active = str(data).strip().lower() not in ("", "0", "false", "off", "none")

    def _diagnostics_cb(self, msg):
        if msg.status:
            self.rtk_state = msg.status[0].name

    def _localization_status_cb(self, msg):
        value = getattr(msg, "data", str(msg)).strip().lower()
        self.human_intervention_active = value in ("manual", "intervention", "override", "taken_over")

    def _update_topic_health(self, key, msg):
        state = self.topic_state.get(key)
        if state is None:
            return
        now_wall = datetime.datetime.now().timestamp()
        ros_stamp = math.nan
        try:
            if hasattr(msg, "header") and getattr(msg.header, "stamp", None):
                ros_stamp = float(msg.header.stamp.to_sec())
            else:
                ros_stamp = _safe_ros_time()
        except Exception:
            ros_stamp = _safe_ros_time()
        state["msg_times"].append(now_wall)
        state["msg_times"] = state["msg_times"][-100:]
        state["last_ros_stamp"] = ros_stamp
        state["last_wall_stamp"] = now_wall
        state["state"] = "online"

    def _start_timers(self):
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(1000)

        self.master_timer = QTimer(self)
        self.master_timer.timeout.connect(self._refresh_master_topic_presence)
        self.master_timer.start(3000)

    def _refresh_master_topic_presence(self):
        try:
            published = {name for name, _type in rospy.get_published_topics()}
        except Exception as exc:
            self._log("Failed to query ROS master topics: %s" % exc)
            return
        for key in TOPIC_KEYS_TO_MONITOR:
            state = self.topic_state[key]
            configured = state["topic_name"]
            if not configured or configured == "(not configured)":
                state["state"] = "missing"
                continue
            if configured not in published and state["last_wall_stamp"] is None:
                state["state"] = "missing"

    def _refresh_ui(self):
        now_wall = datetime.datetime.now().timestamp()
        cmd_timeout = (load_yaml_file(self.metrics_config_path).get("metrics") or {}).get("cmd_vel_dropout_factor", 2.0) / max(
            (load_yaml_file(self.metrics_config_path).get("metrics") or {}).get("cmd_vel_expected_frequency", 10.0), 1e-9
        )
        self.tf_ok = self._topic_recent("tf", 0.5)
        self.can_ok = self._topic_recent("can_feedback", 1.0)
        self.cmd_vel_dropout = not self._topic_recent("cmd_vel", cmd_timeout)

        self.move_base_value.setText(self.move_base_state)
        self.velocity_value.setText("v=%.3f m/s, omega=%.3f rad/s" % (self.current_velocity["v"], self.current_velocity["omega"]))
        self.yaw_value.setText("%.3f rad" % self.current_pose["yaw"] if not math.isnan(self.current_pose["yaw"]) else "nan")
        self.rtk_value.setText(str(self.rtk_state))
        self.cmd_dropout_value.setText(str(self.cmd_vel_dropout))
        self.tf_value.setText(str(self.tf_ok))
        self.can_value.setText(str(self.can_ok))
        self.estop_value.setText(str(self.emergency_stop_active))
        self.manual_value.setText(str(self.human_intervention_active))

        for row, key in enumerate(TOPIC_KEYS_TO_MONITOR):
            state = self.topic_state[key]
            last_age = math.nan
            if state["last_wall_stamp"] is not None:
                last_age = now_wall - state["last_wall_stamp"]
            topic_state = state["state"]
            if state["topic_name"] != "(not configured)":
                if state["last_wall_stamp"] is None:
                    topic_state = "missing"
                elif last_age > 2.0:
                    topic_state = "timeout"
                else:
                    topic_state = "online"
            freq = math.nan
            if len(state["msg_times"]) >= 2:
                deltas = [state["msg_times"][i] - state["msg_times"][i - 1] for i in range(1, len(state["msg_times"])) if state["msg_times"][i] > state["msg_times"][i - 1]]
                if deltas:
                    freq = 1.0 / (sum(deltas) / len(deltas))
            self.topic_table.setItem(row, 1, QTableWidgetItem(topic_state))
            self.topic_table.setItem(row, 2, QTableWidgetItem("nan" if math.isnan(freq) else "%.2f" % freq))
            self.topic_table.setItem(row, 3, QTableWidgetItem("nan" if math.isnan(last_age) else "%.2f" % last_age))
            self.topic_table.setItem(row, 4, QTableWidgetItem("nan" if math.isnan(state["last_ros_stamp"]) else "%.3f" % state["last_ros_stamp"]))

        if self.current_bag_process and self.current_bag_process.poll() is None:
            duration = (datetime.datetime.now() - self.recording_started_wall).total_seconds() if self.recording_started_wall else 0.0
            self.record_status_label.setText("Status: Recording")
            self.duration_label.setText("Duration: %.1f s" % duration)
            self.bag_size_label.setText("Bag Size: %s" % self._format_bag_size())
        else:
            if self.current_bag_file:
                self.record_status_label.setText("Status: Stopped")
            else:
                self.record_status_label.setText("Status: Idle")

    def _topic_recent(self, key, timeout_sec):
        state = self.topic_state.get(key)
        if not state or state["last_wall_stamp"] is None:
            return False
        return (datetime.datetime.now().timestamp() - state["last_wall_stamp"]) <= timeout_sec

    def _choose_output_dir(self):
        chosen = QFileDialog.getExistingDirectory(self, "选择Bag输出目录", self.output_dir_edit.text() or self.output_root_default)
        if chosen:
            self.output_dir_edit.setText(chosen)

    def start_recording(self):
        try:
            if self.current_bag_process and self.current_bag_process.poll() is None:
                self._log("录制已在运行。")
                return
            route_name = self.route_name_edit.text().strip() or "default_route"
            test_id = self.test_id_edit.text().strip() or "000"
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            file_base = "%s_%s_%s" % (timestamp, route_name, test_id)
            output_root = self.output_dir_edit.text().strip() or self.output_root_default
            self.current_bag_dir = os.path.join(output_root, file_base)
            os.makedirs(self.current_bag_dir, exist_ok=True)
            self.current_bag_prefix = os.path.join(self.current_bag_dir, file_base)
            self.current_bag_file = self.current_bag_prefix + ".bag"
            self.current_metadata_file = os.path.join(self.current_bag_dir, "metadata.yaml")
            self.current_events_file = os.path.join(self.current_bag_dir, "manual_events.csv")
            self.events = []
            self._initialize_event_csv()

            self.recorded_topics = collect_record_topics(self.topics_cfg)
            command = ["rosbag", "record", "-O", self.current_bag_prefix] + self.recorded_topics
            self.current_bag_process = subprocess.Popen(command, cwd=self.current_bag_dir)
            self.recording_started_wall = datetime.datetime.now()
            self.recording_started_ros = _safe_ros_time()

            self.bag_name_label.setText("Bag: %s" % os.path.basename(self.current_bag_file))
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.mark_event_button.setEnabled(True)
            self.save_events_button.setEnabled(True)
            self.analyze_button.setEnabled(False)
            self.report_button.setEnabled(False)
            self._log("Started recording bag: %s" % self.current_bag_file)
            self._write_metadata(end_recording=False)
        except Exception as exc:
            self._log_exception("Failed to start recording", exc)

    def stop_recording(self):
        try:
            if not self.current_bag_process or self.current_bag_process.poll() is not None:
                self._log("没有活跃的录制需要停止。")
                return
            self.current_bag_process.send_signal(signal.SIGINT)
            self.current_bag_process.wait(timeout=20)
            self._log("Stopped recording safely with SIGINT.")
        except subprocess.TimeoutExpired:
            self._log("rosbag record did not stop in time. Waiting for external cleanup.")
        except Exception as exc:
            self._log_exception("Failed to stop recording", exc)
        finally:
            self._write_metadata(end_recording=True)
            self.save_events_csv()
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.mark_event_button.setEnabled(False)
            self.save_events_button.setEnabled(True)
            self.analyze_button.setEnabled(bool(self.current_bag_file and os.path.exists(self.current_bag_file)))
            self.report_button.setEnabled(False)

    def _mark_event(self, event_type):
        try:
            if not self.current_events_file:
                self._log("No active experiment context. Start recording first or save metadata path manually.")
                return
            event = {
                "test_id": self.test_id_edit.text().strip() or "000",
                "ros_timestamp": _safe_ros_time(),
                "wall_timestamp": datetime.datetime.now().isoformat(),
                "event_type": event_type,
                "description": self.event_description_edit.text().strip(),
                "severity": self.event_severity_combo.currentText(),
                "x": self.current_pose["x"],
                "y": self.current_pose["y"],
                "yaw": self.current_pose["yaw"],
            }
            self.events.append(event)
            self._append_event_csv(event)
            if event_type == "human_intervention":
                self.human_intervention_active = True
            if event_type == "emergency_stop":
                self.emergency_stop_active = True
            self._log("Marked event: %s" % event_type)
        except Exception as exc:
            self._log_exception("Failed to mark event", exc)

    def save_events_csv(self):
        try:
            if not self.current_events_file:
                self._log("No events file path available.")
                return
            with open(self.current_events_file, "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=["test_id", "ros_timestamp", "wall_timestamp", "event_type", "description", "severity", "x", "y", "yaw"],
                )
                writer.writeheader()
                for event in self.events:
                    writer.writerow(event)
            self._log("Saved manual events to %s" % self.current_events_file)
        except Exception as exc:
            self._log_exception("Failed to save manual events", exc)

    def analyze_current_bag(self):
        try:
            if not self.current_bag_file or not os.path.exists(self.current_bag_file):
                self._log("Current bag file is not available.")
                return
            output_dir = os.path.join(self.current_bag_dir, "analysis")
            command = [
                sys.executable,
                os.path.join(self.package_dir, "scripts", "analyze_bag.py"),
                "--bag",
                self.current_bag_file,
                "--config",
                self.topics_config_path,
                "--vehicle",
                self.vehicle_config_path,
                "--metrics",
                self.metrics_config_path,
                "--output",
                output_dir,
                "--manual-events",
                self.current_events_file,
                "--test-id",
                self.test_id_edit.text().strip() or "000",
            ]
            self._run_worker(command, "Analyze Current Bag", self._on_analyze_finished)
        except Exception as exc:
            self._log_exception("Failed to start analysis", exc)

    def _on_analyze_finished(self, success, output):
        if success:
            self._log("Bag analysis completed.")
            self.report_button.setEnabled(True)
        else:
            self._log("Bag analysis failed.")
        if output:
            self._log(output)

    def generate_report(self):
        try:
            analysis_dir = os.path.join(self.current_bag_dir, "analysis")
            summary_json = os.path.join(analysis_dir, "metrics_summary.json")
            report_md = os.path.join(analysis_dir, "report.md")
            if not os.path.exists(summary_json):
                self._log("metrics_summary.json not found. Run analysis first.")
                return
            command = [
                sys.executable,
                os.path.join(self.package_dir, "scripts", "generate_report.py"),
                "--summary-json",
                summary_json,
                "--output",
                report_md,
            ]
            self._run_worker(command, "Generate Report", self._on_report_finished)
        except Exception as exc:
            self._log_exception("Failed to generate report", exc)

    def _on_report_finished(self, success, output):
        if success:
            self._log("Report generation completed.")
        else:
            self._log("Report generation failed.")
        if output:
            self._log(output)

    def open_output_folder(self):
        try:
            target = self.current_bag_dir or self.output_dir_edit.text().strip() or self.output_root_default
            if not os.path.exists(target):
                os.makedirs(target, exist_ok=True)
            if sys.platform.startswith("linux"):
                subprocess.Popen(["xdg-open", target])
            elif sys.platform.startswith("win"):
                subprocess.Popen(["explorer", target])
            elif sys.platform == "darwin":
                subprocess.Popen(["open", target])
            else:
                self._log("Open folder is not supported on this platform: %s" % sys.platform)
                return
            self._log("Opened output folder: %s" % target)
        except Exception as exc:
            self._log_exception("Failed to open output folder", exc)

    def _run_worker(self, command, label, callback):
        self._log("%s started: %s" % (label, " ".join(command)))
        worker = CommandWorker(command, workdir=self.package_dir)
        worker.finished_with_result.connect(callback)
        worker.finished_with_result.connect(lambda success, output, worker_ref=worker: self._cleanup_worker(worker_ref))
        self.process_workers.append(worker)
        worker.start()

    def _cleanup_worker(self, worker):
        if worker in self.process_workers:
            self.process_workers.remove(worker)
        worker.deleteLater()

    def _initialize_event_csv(self):
        with open(self.current_events_file, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["test_id", "ros_timestamp", "wall_timestamp", "event_type", "description", "severity", "x", "y", "yaw"],
            )
            writer.writeheader()

    def _append_event_csv(self, event):
        with open(self.current_events_file, "a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["test_id", "ros_timestamp", "wall_timestamp", "event_type", "description", "severity", "x", "y", "yaw"],
            )
            writer.writerow(event)

    def _write_metadata(self, end_recording=False):
        try:
            metadata = {
                "test_id": self.test_id_edit.text().strip(),
                "route_name": self.route_name_edit.text().strip(),
                "map_id": self.map_id_edit.text().strip(),
                "vehicle_id": self.vehicle_id_edit.text().strip(),
                "operator": self.operator_edit.text().strip(),
                "weather_or_scene": self.weather_edit.text().strip(),
                "notes": self.notes_edit.toPlainText().strip(),
                "start_wall_time": self.recording_started_wall.isoformat() if self.recording_started_wall else "",
                "end_wall_time": datetime.datetime.now().isoformat() if end_recording else "",
                "start_ros_time": self.recording_started_ros if self.recording_started_ros is not None else "",
                "end_ros_time": _safe_ros_time() if end_recording else "",
                "bag_file": self.current_bag_file,
                "recorded_topics": self.recorded_topics,
                "config_files_used": {
                    "topics": self.topics_config_path,
                    "vehicle": self.vehicle_config_path,
                    "metrics": self.metrics_config_path,
                },
                "git_commit_hash": _read_git_commit(self.package_dir),
                "hostname": socket.gethostname(),
                "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI", ""),
            }
            with open(self.current_metadata_file, "w", encoding="utf-8") as handle:
                yaml.safe_dump(metadata, handle, allow_unicode=True, sort_keys=False)
            self._log("Metadata written to %s" % self.current_metadata_file)
        except Exception as exc:
            self._log_exception("Failed to write metadata", exc)

    def _format_bag_size(self):
        if not self.current_bag_file:
            return "0 B"
        size_bytes = 0
        bag_active = self.current_bag_file + ".active"
        if os.path.exists(self.current_bag_file):
            size_bytes = os.path.getsize(self.current_bag_file)
        elif os.path.exists(bag_active):
            size_bytes = os.path.getsize(bag_active)
        units = ["B", "KB", "MB", "GB"]
        value = float(size_bytes)
        idx = 0
        while value >= 1024.0 and idx < len(units) - 1:
            value /= 1024.0
            idx += 1
        return "%.2f %s" % (value, units[idx])

    def _log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_output.appendPlainText("[%s] %s" % (timestamp, message))

    def _log_exception(self, prefix, exc):
        self._log("%s: %s" % (prefix, exc))
        self._log(traceback.format_exc())

    def closeEvent(self, event):
        if self.current_bag_process and self.current_bag_process.poll() is None:
            answer = QMessageBox.question(
                self,
                "Stop Recording?",
                "Recording is still running. Stop rosbag safely before exit?",
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel,
                QMessageBox.Yes,
            )
            if answer == QMessageBox.Cancel:
                event.ignore()
                return
            if answer == QMessageBox.Yes:
                self.stop_recording()
        event.accept()


def main():
    rospy.init_node("agv_nav_evaluation_dashboard", anonymous=False, disable_signals=True)
    app = QApplication(sys.argv)
    window = EvalDashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
