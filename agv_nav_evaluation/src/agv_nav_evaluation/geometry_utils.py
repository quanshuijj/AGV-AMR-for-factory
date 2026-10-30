import math


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euclidean_distance(point_a, point_b):
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def pose_distance(pose_a, pose_b):
    return euclidean_distance((pose_a["x"], pose_a["y"]), (pose_b["x"], pose_b["y"]))


def interpolate_segment_projection(point, start, end):
    sx, sy = start
    ex, ey = end
    px, py = point
    dx = ex - sx
    dy = ey - sy
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return (sx, sy), 0.0
    t = ((px - sx) * dx + (py - sy) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj = (sx + t * dx, sy + t * dy)
    return proj, t


def point_to_segment_signed_distance(point, start, end):
    proj, _ = interpolate_segment_projection(point, start, end)
    dist = euclidean_distance(point, proj)
    cross = (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (point[0] - start[0])
    return -dist if cross < 0.0 else dist, proj


def segment_heading(start, end):
    return math.atan2(end[1] - start[1], end[0] - start[0])


def transform_reference_point(pose, offset):
    yaw = pose.get("yaw", 0.0)
    ox, oy = offset
    return {
        "t": pose["t"],
        "x": pose["x"] + ox * math.cos(yaw) - oy * math.sin(yaw),
        "y": pose["y"] + ox * math.sin(yaw) + oy * math.cos(yaw),
        "yaw": yaw,
    }


def polygon_distance_to_point(polygon, point):
    if not polygon:
        return math.nan
    best = None
    count = len(polygon)
    for idx in range(count):
        start = polygon[idx]
        end = polygon[(idx + 1) % count]
        signed_dist, _ = point_to_segment_signed_distance(point, start, end)
        dist = abs(signed_dist)
        if best is None or dist < best:
            best = dist
    return best if best is not None else math.nan
