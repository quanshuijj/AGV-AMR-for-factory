import csv


def load_csv_rows(path):
    with open(path, "r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
