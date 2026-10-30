import os

import yaml


def load_yaml_file(path):
    """Load a YAML file and return an empty dict when the file is missing."""
    if not path:
        return {}
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data


def load_all_configs(topics_path, vehicle_path=None, metrics_path=None):
    return {
        "topics": load_yaml_file(topics_path),
        "vehicle": load_yaml_file(vehicle_path),
        "metrics": load_yaml_file(metrics_path),
    }
