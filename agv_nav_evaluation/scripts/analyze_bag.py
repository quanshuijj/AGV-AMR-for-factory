#!/usr/bin/env python3
import argparse
import os

from agv_nav_evaluation.config_loader import load_all_configs
from agv_nav_evaluation.manual_events import load_manual_events
from agv_nav_evaluation.rosbag_loader import RosbagLoader
from compute_metrics import compute_metrics_from_dataset, write_outputs


def main():
    parser = argparse.ArgumentParser(description="Analyze AGV navigation rosbag and export metrics.")
    parser.add_argument("--bag", required=True, help="Path to rosbag file")
    parser.add_argument("--config", required=True, help="topics.yaml path")
    parser.add_argument("--vehicle", required=True, help="vehicle.yaml path")
    parser.add_argument("--metrics", required=True, help="metrics.yaml path")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--manual-events", default=None, help="Optional manual events CSV")
    parser.add_argument("--test-id", default=None, help="Test identifier")
    args = parser.parse_args()

    configs = load_all_configs(args.config, args.vehicle, args.metrics)
    loader = RosbagLoader(args.bag, configs["topics"])
    dataset, warnings = loader.load()
    dataset["warnings"] = warnings
    manual_csv = args.manual_events
    if manual_csv is None:
        manual_csv = os.path.join(args.output, "manual_events.csv")
    manual_events = load_manual_events(manual_csv)
    test_id = args.test_id or os.path.splitext(os.path.basename(args.bag))[0]
    result = compute_metrics_from_dataset(dataset, configs, test_id=test_id, manual_events=manual_events)
    write_outputs(args.output, result)
    print("Analysis complete: %s" % args.output)


if __name__ == "__main__":
    main()
