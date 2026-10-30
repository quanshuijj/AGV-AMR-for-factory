#!/usr/bin/env python3
import rospy

from agv_nav_evaluation.experiment_recorder import ExperimentRecorder


def main():
    rospy.init_node("agv_nav_evaluation_record_experiment", anonymous=False)
    recorder = ExperimentRecorder(
        route_name=rospy.get_param("~route_name", "default_route"),
        test_id=rospy.get_param("~test_id", "000"),
        topics_config_path=rospy.get_param("~topics_config"),
        output_dir=rospy.get_param("~output_dir"),
        bag_prefix=rospy.get_param("~bag_prefix", "eval"),
    )
    recorder.run()


if __name__ == "__main__":
    main()
