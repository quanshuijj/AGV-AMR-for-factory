#!/usr/bin/env python3
import argparse

from agv_nav_evaluation.manual_events import export_manual_event_template


def main():
    parser = argparse.ArgumentParser(description="Export manual event CSV template.")
    parser.add_argument("--output", required=True, help="Output CSV path")
    args = parser.parse_args()
    export_manual_event_template(args.output)
    print("Manual event template exported to %s" % args.output)


if __name__ == "__main__":
    main()
