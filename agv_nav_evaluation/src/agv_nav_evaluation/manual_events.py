import csv
import os


DEFAULT_FIELDS = ["test_id", "timestamp", "event_type", "description", "severity", "x", "y", "yaw"]


def load_manual_events(path):
    if not path or not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def export_manual_event_template(path):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=DEFAULT_FIELDS)
        writer.writeheader()
        writer.writerow(
            {
                "test_id": "001",
                "timestamp": "0.0",
                "event_type": "collision",
                "description": "example event",
                "severity": "high",
                "x": "",
                "y": "",
                "yaw": "",
            }
        )
