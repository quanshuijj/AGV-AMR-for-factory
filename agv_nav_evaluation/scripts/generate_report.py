#!/usr/bin/env python3
import argparse
import json
import os

from agv_nav_evaluation.report_writer import build_markdown_report


def main():
    parser = argparse.ArgumentParser(description="Generate Markdown report from metrics_summary.json.")
    parser.add_argument("--summary-json", required=True, help="metrics_summary.json path")
    parser.add_argument("--output", required=True, help="output markdown path")
    args = parser.parse_args()

    with open(args.summary_json, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    report = build_markdown_report(payload.get("summary", {}), payload.get("warnings", []))
    with open(args.output, "w", encoding="utf-8") as handle:
        handle.write(report)
    print("Report written to %s" % args.output)


if __name__ == "__main__":
    main()
