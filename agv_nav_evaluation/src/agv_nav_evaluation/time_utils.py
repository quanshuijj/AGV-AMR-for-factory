import math


def to_sec(stamp):
    if stamp is None:
        return math.nan
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    if hasattr(stamp, "secs"):
        return float(stamp.secs) + float(getattr(stamp, "nsecs", 0)) * 1e-9
    try:
        return float(stamp)
    except (TypeError, ValueError):
        return math.nan


def diff_times(times):
    result = []
    for index in range(1, len(times)):
        prev_time = times[index - 1]
        curr_time = times[index]
        dt = curr_time - prev_time
        if dt <= 0:
            continue
        result.append(dt)
    return result


def mean_rate_from_times(times):
    deltas = diff_times(times)
    if not deltas:
        return math.nan
    return 1.0 / (sum(deltas) / float(len(deltas)))


def nearest_index(times, target, max_gap=None):
    if not times:
        return None
    best_index = None
    best_gap = None
    for idx, timestamp in enumerate(times):
        gap = abs(timestamp - target)
        if best_gap is None or gap < best_gap:
            best_index = idx
            best_gap = gap
    if max_gap is not None and best_gap is not None and best_gap > max_gap:
        return None
    return best_index
