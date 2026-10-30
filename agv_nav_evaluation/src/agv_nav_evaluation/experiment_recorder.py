import datetime
import os
import signal
import subprocess

import rospy

from agv_nav_evaluation.config_loader import load_yaml_file
from agv_nav_evaluation.topic_utils import collect_record_topics


class ExperimentRecorder(object):
    def __init__(self, route_name, test_id, topics_config_path, output_dir, bag_prefix):
        self.route_name = route_name
        self.test_id = test_id
        self.topics_config_path = topics_config_path
        self.output_dir = output_dir
        self.bag_prefix = bag_prefix
        self.process = None

    def run(self):
        config = load_yaml_file(self.topics_config_path)
        topics = collect_record_topics(config)
        if not topics:
            rospy.logwarn("No topics configured for recording")
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        test_name = "%s_%s_%s_%s" % (self.bag_prefix, self.route_name, self.test_id, timestamp)
        bag_dir = os.path.join(self.output_dir, test_name)
        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)
        bag_path = os.path.join(bag_dir, test_name)
        command = ["rosbag", "record", "-O", bag_path] + topics
        rospy.loginfo("Starting rosbag record: %s", " ".join(command))
        self.process = subprocess.Popen(command)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
        self.process.wait()

    def _handle_signal(self, signum, frame):
        del signum
        del frame
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
