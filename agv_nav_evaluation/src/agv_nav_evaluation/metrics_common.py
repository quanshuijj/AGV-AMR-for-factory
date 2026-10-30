import csv
import json
import math
import os


def nan_if_none(value):
    return math.nan if value is None else value


def safe_mean(values):
    valid = [value for value in values if value is not None and not _is_nan(value)]
    if not valid:
        return math.nan
    return sum(valid) / float(len(valid))


def safe_rmse(values):
    valid = [value for value in values if value is not None and not _is_nan(value)]
    if not valid:
        return math.nan
    return math.sqrt(sum(value * value for value in valid) / float(len(valid)))


def safe_max(values):
    valid = [value for value in values if value is not None and not _is_nan(value)]
    return max(valid) if valid else math.nan


def safe_min(values):
    valid = [value for value in values if value is not None and not _is_nan(value)]
    return min(valid) if valid else math.nan


def safe_percentile(values, percentile):
    valid = sorted([value for value in values if value is not None and not _is_nan(value)])
    if not valid:
        return math.nan
    if len(valid) == 1:
        return valid[0]
    rank = (percentile / 100.0) * (len(valid) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return valid[lower]
    frac = rank - lower
    return valid[lower] * (1.0 - frac) + valid[upper] * frac


def summarize_series(values):
    return {
        "mean": safe_mean(values),
        "rmse": safe_rmse(values),
        "p95": safe_percentile(values, 95.0),
        "max": safe_max([abs(v) for v in values if v is not None and not _is_nan(v)]),
    }


def warn(warnings, message):
    warnings.append(message)


def ensure_dir(path):
    if path and not os.path.exists(path):
        os.makedirs(path)


def write_single_row_csv(path, row):
    fieldnames = list(row.keys())
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)


def write_rows_csv(path, rows, fieldnames):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path, data):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)


def _is_nan(value):
    return isinstance(value, float) and math.isnan(value)
