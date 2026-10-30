#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Plot global path metrics from CSV files.

Typical usage:
  rosrun my_nav_assist global_path_metrics_plotter.py \
    --raw ~/navCarProject_ws/logs/raw_global_path_metrics.csv \
    --processed ~/navCarProject_ws/logs/processed_global_path_metrics.csv \
    --out ~/navCarProject_ws/logs/path_metric_figures
"""

import argparse
import csv
import math
import os

import matplotlib.pyplot as plt


SUMMARY_METRICS = [
    ("planning_time_s", "Planning time (s)", "lower"),
    ("path_length_m", "Path length (m)", "lower"),
    ("path_cost_mean", "Mean path cost", "lower"),
    ("path_cost_max", "Max path cost", "lower"),
    ("straight_lateral_error_mean_m", "Straight lateral error mean (m)", "lower"),
    ("turn_curvature_max_1pm", "Max turn curvature (1/m)", "lower"),
    ("curvature_over_limit_ratio", "Curvature over-limit ratio", "lower"),
    ("yaw_change_sum_rad", "Yaw change sum (rad)", "lower"),
    ("omega_change_mean_radps", "Mean angular-velocity change (rad/s)", "lower"),
    ("curvature_change_mean_1pm", "Mean curvature change (1/m)", "lower"),
]


SERIES_METRICS = [
    ("planning_time_s", "Planning time per trial", "s"),
    ("path_length_m", "Path length per trial", "m"),
    ("turn_curvature_max_1pm", "Maximum turning curvature per trial", "1/m"),
    ("yaw_change_sum_rad", "Yaw change sum per trial", "rad"),
    ("omega_change_mean_radps", "Mean angular-velocity change per trial", "rad/s"),
]


def to_float(value):
    try:
        if value is None or value == "":
            return math.nan
        return float(value)
    except ValueError:
        return math.nan


def read_csv(path):
    path = os.path.expanduser(path)
    rows = []
    with open(path, "r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            rows.append(row)
    return rows


def metric_values(rows, metric):
    values = [to_float(row.get(metric)) for row in rows]
    return [v for v in values if not math.isnan(v) and v >= 0.0]


def avg(values):
    return sum(values) / len(values) if values else math.nan


def improvement_percent(raw_value, processed_value):
    if math.isnan(raw_value) or math.isnan(processed_value) or abs(raw_value) < 1e-9:
        return math.nan
    return (raw_value - processed_value) / raw_value * 100.0


def ensure_dir(path):
    path = os.path.expanduser(path)
    os.makedirs(path, exist_ok=True)
    return path


def plot_summary(raw_rows, processed_rows, out_dir):
    labels = []
    raw_means = []
    processed_means = []

    for metric, label, _direction in SUMMARY_METRICS:
        raw_value = avg(metric_values(raw_rows, metric))
        processed_value = avg(metric_values(processed_rows, metric))
        labels.append(label)
        raw_means.append(raw_value)
        processed_means.append(processed_value)

    x = list(range(len(labels)))
    width = 0.38

    plt.figure(figsize=(15, 7))
    plt.bar([i - width / 2 for i in x], raw_means, width=width, label="Raw SBPL path", color="#687a8f")
    plt.bar([i + width / 2 for i in x], processed_means, width=width, label="Processed path", color="#00a7c7")
    plt.xticks(x, labels, rotation=35, ha="right")
    plt.ylabel("Metric value")
    plt.title("Global path metric comparison")
    plt.grid(axis="y", alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "global_path_metrics_summary.png"), dpi=220)
    plt.close()


def plot_improvement(raw_rows, processed_rows, out_dir):
    labels = []
    improvements = []

    for metric, label, _direction in SUMMARY_METRICS:
        raw_value = avg(metric_values(raw_rows, metric))
        processed_value = avg(metric_values(processed_rows, metric))
        labels.append(label)
        improvements.append(improvement_percent(raw_value, processed_value))

    x = list(range(len(labels)))
    colors = ["#16825d" if v >= 0 else "#bc3f3f" for v in improvements]

    plt.figure(figsize=(15, 6))
    plt.bar(x, improvements, color=colors)
    plt.axhline(0.0, color="#333333", linewidth=1)
    plt.xticks(x, labels, rotation=35, ha="right")
    plt.ylabel("Improvement (%)")
    plt.title("Processed path improvement over raw SBPL path")
    plt.grid(axis="y", alpha=0.25)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "global_path_metrics_improvement.png"), dpi=220)
    plt.close()


def plot_series(raw_rows, processed_rows, out_dir):
    for metric, title, unit in SERIES_METRICS:
        raw_values = metric_values(raw_rows, metric)
        processed_values = metric_values(processed_rows, metric)
        n = max(len(raw_values), len(processed_values))
        if n == 0:
            continue

        plt.figure(figsize=(11, 5))
        if raw_values:
            plt.plot(range(1, len(raw_values) + 1), raw_values, marker="o", label="Raw SBPL path", color="#687a8f")
        if processed_values:
            plt.plot(
                range(1, len(processed_values) + 1),
                processed_values,
                marker="o",
                label="Processed path",
                color="#00a7c7",
            )
        plt.xlabel("Trial index")
        plt.ylabel(unit)
        plt.title(title)
        plt.grid(alpha=0.25)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, f"{metric}_series.png"), dpi=220)
        plt.close()


def write_summary_csv(raw_rows, processed_rows, out_dir):
    out_path = os.path.join(out_dir, "global_path_metrics_summary.csv")
    with open(out_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["metric", "raw_mean", "processed_mean", "improvement_percent", "better_direction"])
        for metric, _label, direction in SUMMARY_METRICS:
            raw_value = avg(metric_values(raw_rows, metric))
            processed_value = avg(metric_values(processed_rows, metric))
            writer.writerow([metric, raw_value, processed_value, improvement_percent(raw_value, processed_value), direction])


def parse_args():
    parser = argparse.ArgumentParser(description="Plot global path metric comparison figures.")
    parser.add_argument("--raw", required=True, help="CSV generated from raw SBPL global path.")
    parser.add_argument("--processed", required=True, help="CSV generated from processed global path.")
    parser.add_argument("--out", default="~/navCarProject_ws/logs/path_metric_figures", help="Output figure directory.")
    return parser.parse_args()


def main():
    args = parse_args()
    raw_rows = read_csv(args.raw)
    processed_rows = read_csv(args.processed)
    out_dir = ensure_dir(args.out)

    plot_summary(raw_rows, processed_rows, out_dir)
    plot_improvement(raw_rows, processed_rows, out_dir)
    plot_series(raw_rows, processed_rows, out_dir)
    write_summary_csv(raw_rows, processed_rows, out_dir)

    print(f"Figures saved to: {out_dir}")


if __name__ == "__main__":
    main()
