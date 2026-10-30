#!/usr/bin/env python3
import math
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from crop_map import parse_origin, read_pgm, read_yaml, write_pgm, write_yaml


class CorridorMapBuilder:
    def __init__(self):
        self.input_yaml = Path(rospy.get_param("~input_yaml"))
        self.output_yaml = Path(rospy.get_param("~output_yaml"))
        self.clicked_topic = rospy.get_param("~clicked_topic", "/clicked_point")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.half_width = float(rospy.get_param("~half_width", 12.0))
        self.margin = float(rospy.get_param("~margin", 2.0))
        self.outside_value = int(rospy.get_param("~outside_value", 0))
        self.chunk_rows = int(rospy.get_param("~chunk_rows", 128))
        self.points = []

        self.marker_pub = rospy.Publisher(
            "~markers", MarkerArray, queue_size=1, latch=True
        )
        self.point_sub = rospy.Subscriber(
            self.clicked_topic, PointStamped, self.on_clicked_point, queue_size=20
        )
        self.save_srv = rospy.Service("~save", Trigger, self.on_save)
        self.clear_srv = rospy.Service("~clear", Empty, self.on_clear)
        self.undo_srv = rospy.Service("~undo", Empty, self.on_undo)

        rospy.loginfo(
            "Corridor map builder ready. Publish points on %s, then call %s/save",
            self.clicked_topic,
            rospy.get_name(),
        )

    def on_clicked_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn(
                "Clicked point frame is %s, expected %s. Using coordinates as-is.",
                msg.header.frame_id,
                self.frame_id,
            )
        self.points.append((msg.point.x, msg.point.y))
        rospy.loginfo(
            "Added corridor point %d: %.3f, %.3f",
            len(self.points),
            msg.point.x,
            msg.point.y,
        )
        self.publish_markers()

    def on_clear(self, _req):
        self.points = []
        self.publish_markers()
        rospy.loginfo("Cleared corridor points")
        return EmptyResponse()

    def on_undo(self, _req):
        if self.points:
            removed = self.points.pop()
            rospy.loginfo("Removed corridor point: %.3f, %.3f", removed[0], removed[1])
        self.publish_markers()
        return EmptyResponse()

    def on_save(self, _req):
        if len(self.points) < 2:
            return TriggerResponse(False, "Need at least two clicked points")
        try:
            result = build_corridor_map(
                self.input_yaml,
                self.output_yaml,
                np.asarray(self.points, dtype=np.float64),
                self.half_width,
                self.margin,
                self.outside_value,
                self.chunk_rows,
            )
        except Exception as exc:
            rospy.logerr("Failed to build corridor map: %s", exc)
            return TriggerResponse(False, str(exc))

        message = (
            "Saved corridor map: {} ({}x{}, origin [{:.3f}, {:.3f}, {:.3f}])"
        ).format(
            result["output_yaml"],
            result["width"],
            result["height"],
            result["origin"][0],
            result["origin"][1],
            result["origin"][2],
        )
        rospy.loginfo(message)
        return TriggerResponse(True, message)

    def publish_markers(self):
        markers = MarkerArray()
        now = rospy.Time.now()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if self.points:
            points_marker = Marker()
            points_marker.header.frame_id = self.frame_id
            points_marker.header.stamp = now
            points_marker.ns = "corridor_points"
            points_marker.id = 1
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.scale.x = 0.8
            points_marker.scale.y = 0.8
            points_marker.scale.z = 0.8
            points_marker.color.r = 1.0
            points_marker.color.g = 0.55
            points_marker.color.a = 1.0
            points_marker.points = [make_point(x, y) for x, y in self.points]
            markers.markers.append(points_marker)

        if len(self.points) >= 2:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = now
            line.ns = "corridor_center"
            line.id = 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.45
            line.color.g = 1.0
            line.color.b = 1.0
            line.color.a = 1.0
            line.points = [make_point(x, y, 0.05) for x, y in self.points]
            markers.markers.append(line)

            boundary = Marker()
            boundary.header.frame_id = self.frame_id
            boundary.header.stamp = now
            boundary.ns = "corridor_width"
            boundary.id = 3
            boundary.type = Marker.LINE_LIST
            boundary.action = Marker.ADD
            boundary.scale.x = 0.2
            boundary.color.r = 0.1
            boundary.color.g = 0.8
            boundary.color.b = 1.0
            boundary.color.a = 0.9
            boundary.points = corridor_boundary_lines(self.points, self.half_width)
            markers.markers.append(boundary)

        self.marker_pub.publish(markers)


def make_point(x, y, z=0.15):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def corridor_boundary_lines(points, half_width):
    pts = np.asarray(points, dtype=np.float64)
    left = []
    right = []
    for i, point in enumerate(pts):
        if i == 0:
            direction = pts[1] - pts[0]
        elif i == len(pts) - 1:
            direction = pts[-1] - pts[-2]
        else:
            prev_dir = normalized(pts[i] - pts[i - 1])
            next_dir = normalized(pts[i + 1] - pts[i])
            direction = prev_dir + next_dir
            if np.linalg.norm(direction) < 1e-6:
                direction = pts[i + 1] - pts[i - 1]
        normal = normalized(np.array([-direction[1], direction[0]]))
        left.append(point + normal * half_width)
        right.append(point - normal * half_width)

    marker_points = []
    for edge in (left, right):
        for a, b in zip(edge[:-1], edge[1:]):
            marker_points.append(make_point(a[0], a[1], 0.08))
            marker_points.append(make_point(b[0], b[1], 0.08))
    return marker_points


def normalized(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        return np.array([1.0, 0.0], dtype=np.float64)
    return vector / norm


def build_corridor_map(
    input_yaml, output_yaml, points, half_width, margin, outside_value, chunk_rows
):
    meta = read_yaml(input_yaml)
    resolution = float(meta["resolution"])
    origin = parse_origin(meta["origin"])
    image_path = Path(input_yaml).parent / meta["image"]
    width, height, pixels = read_pgm(image_path)
    image = np.frombuffer(pixels, dtype=np.uint8).reshape((height, width))

    xmin = float(points[:, 0].min() - half_width - margin)
    xmax = float(points[:, 0].max() + half_width + margin)
    ymin = float(points[:, 1].min() - half_width - margin)
    ymax = float(points[:, 1].max() + half_width + margin)

    col0 = max(0, int(math.floor((xmin - origin[0]) / resolution)))
    col1 = min(width, int(math.ceil((xmax - origin[0]) / resolution)))
    map_row0 = max(0, int(math.floor((ymin - origin[1]) / resolution)))
    map_row1 = min(height, int(math.ceil((ymax - origin[1]) / resolution)))
    if col0 >= col1 or map_row0 >= map_row1:
        raise ValueError("Corridor bounds do not overlap the input map")

    img_row0 = height - map_row1
    img_row1 = height - map_row0
    cropped = image[img_row0:img_row1, col0:col1].copy()
    out_height, out_width = cropped.shape

    segments_a = points[:-1]
    segments_b = points[1:]
    segment_vectors = segments_b - segments_a
    segment_len2 = np.sum(segment_vectors * segment_vectors, axis=1)
    valid = segment_len2 > 1e-9
    if not np.any(valid):
        raise ValueError("All corridor segments are zero length")
    segments_a = segments_a[valid]
    segment_vectors = segment_vectors[valid]
    segment_len2 = segment_len2[valid]

    cols = np.arange(out_width, dtype=np.float64)
    xs = origin[0] + (col0 + cols + 0.5) * resolution
    half_width2 = half_width * half_width
    for row0 in range(0, out_height, chunk_rows):
        row1 = min(out_height, row0 + chunk_rows)
        rows = np.arange(row0, row1, dtype=np.float64)
        source_img_rows = img_row0 + rows
        map_rows = height - 1 - source_img_rows
        ys = origin[1] + (map_rows[:, None] + 0.5) * resolution
        x_grid = np.broadcast_to(xs[None, :], (row1 - row0, out_width))
        y_grid = np.broadcast_to(ys, (row1 - row0, out_width))

        min_dist2 = np.full((row1 - row0, out_width), np.inf, dtype=np.float64)
        for seg_a, seg_vec, seg_len2 in zip(segments_a, segment_vectors, segment_len2):
            px = x_grid - seg_a[0]
            py = y_grid - seg_a[1]
            t = (px * seg_vec[0] + py * seg_vec[1]) / seg_len2
            t = np.clip(t, 0.0, 1.0)
            dx = px - t * seg_vec[0]
            dy = py - t * seg_vec[1]
            dist2 = dx * dx + dy * dy
            min_dist2 = np.minimum(min_dist2, dist2)

        outside = min_dist2 > half_width2
        cropped[row0:row1][outside] = outside_value

    output_pgm = Path(output_yaml).with_suffix(".pgm")
    new_origin = [
        origin[0] + col0 * resolution,
        origin[1] + map_row0 * resolution,
        origin[2],
    ]
    write_pgm(output_pgm, out_width, out_height, cropped.tobytes())
    write_yaml(output_yaml, output_pgm.name, resolution, new_origin, meta)
    return {
        "output_yaml": str(output_yaml),
        "output_pgm": str(output_pgm),
        "width": out_width,
        "height": out_height,
        "origin": new_origin,
    }


def main():
    rospy.init_node("rviz_corridor_map_builder")
    CorridorMapBuilder()
    rospy.spin()


if __name__ == "__main__":
    main()
