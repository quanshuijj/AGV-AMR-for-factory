#!/usr/bin/env python3
import argparse
import math
import os
import struct
from pathlib import Path

import numpy as np


def read_yaml_map(path):
    values = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()

    origin_text = values["origin"].strip()[1:-1]
    origin = [float(x.strip()) for x in origin_text.split(",")]
    return {
        "image": values["image"],
        "resolution": float(values["resolution"]),
        "origin": origin,
        "negate": int(values.get("negate", "0")),
        "occupied_thresh": float(values.get("occupied_thresh", "0.65")),
        "free_thresh": float(values.get("free_thresh", "0.196")),
    }


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} is not a binary PGM (P5)")

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"{path} ended before PGM header was complete")
            line = line.split(b"#", 1)[0].strip()
            if line:
                tokens.extend(line.split())

        width, height, maxval = map(int, tokens[:3])
        if maxval > 255:
            raise ValueError("Only 8-bit PGM files are supported")
        data = np.frombuffer(f.read(width * height), dtype=np.uint8)
    return data.reshape((height, width)).copy(), width, height, maxval


def write_pgm(path, image):
    with open(path, "wb") as f:
        h, w = image.shape
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(image.astype(np.uint8, copy=False).tobytes())


def rotation(angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def rotate_yaml_and_pgm(input_yaml, output_yaml, output_pgm, angle_deg, fill_value):
    map_info = read_yaml_map(input_yaml)
    input_pgm = Path(input_yaml).parent / map_info["image"]
    image, width, height, _ = read_pgm(input_pgm)
    res = map_info["resolution"]
    ox, oy, oyaw = map_info["origin"]

    angle = math.radians(angle_deg)
    rot = rotation(angle)
    inv_rot = rotation(-angle)

    corners = np.array(
        [
            [ox, oy],
            [ox + width * res, oy],
            [ox, oy + height * res],
            [ox + width * res, oy + height * res],
        ],
        dtype=np.float64,
    )
    rotated_corners = corners @ rot.T
    min_xy = rotated_corners.min(axis=0)
    max_xy = rotated_corners.max(axis=0)

    out_width = int(math.ceil((max_xy[0] - min_xy[0]) / res))
    out_height = int(math.ceil((max_xy[1] - min_xy[1]) / res))
    output = np.full((out_height, out_width), fill_value, dtype=np.uint8)

    chunk_rows = 256
    cols = np.arange(out_width, dtype=np.float64)
    x_out = min_xy[0] + (cols + 0.5) * res
    for row0 in range(0, out_height, chunk_rows):
        row1 = min(row0 + chunk_rows, out_height)
        rows = np.arange(row0, row1, dtype=np.float64)
        # PGM rows are top-to-bottom; ROS map rows are bottom-to-top.
        y_out = min_xy[1] + (out_height - rows[:, None] - 0.5) * res
        x_grid = np.broadcast_to(x_out[None, :], (row1 - row0, out_width))
        y_grid = np.broadcast_to(y_out, (row1 - row0, out_width))

        points = np.stack((x_grid, y_grid), axis=-1)
        source = points @ inv_rot.T
        src_col = np.floor((source[..., 0] - ox) / res).astype(np.int64)
        src_map_row = np.floor((source[..., 1] - oy) / res).astype(np.int64)
        src_img_row = height - 1 - src_map_row
        mask = (
            (src_col >= 0)
            & (src_col < width)
            & (src_img_row >= 0)
            & (src_img_row < height)
        )
        output[row0:row1][mask] = image[src_img_row[mask], src_col[mask]]

    write_pgm(output_pgm, output)

    with open(output_yaml, "w", encoding="utf-8") as f:
        f.write(f"image: {Path(output_pgm).name}\n")
        f.write(f"resolution: {res:.8g}\n")
        # The raster data itself is rotated into the aligned map frame, so keep
        # OccupancyGrid yaw at zero for costmap/SBPL compatibility.
        f.write(f"origin: [{min_xy[0]:.6f}, {min_xy[1]:.6f}, 0.0]\n")
        f.write(f"negate: {map_info['negate']}\n")
        f.write(f"occupied_thresh: {map_info['occupied_thresh']:.8g}\n")
        f.write(f"free_thresh: {map_info['free_thresh']:.8g}\n")


def parse_pcd_header(path):
    header = []
    offset = 0
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD file ended before DATA line")
            header.append(line)
            offset += len(line)
            if line.startswith(b"DATA "):
                data_type = line.split()[1].decode("ascii")
                break

    fields = []
    sizes = []
    types = []
    counts = []
    points = None
    for raw in header:
        parts = raw.decode("ascii", errors="ignore").strip().split()
        if not parts:
            continue
        key = parts[0]
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = [int(x) for x in parts[1:]]
        elif key == "TYPE":
            types = parts[1:]
        elif key == "COUNT":
            counts = [int(x) for x in parts[1:]]
        elif key == "POINTS":
            points = int(parts[1])

    if data_type != "binary":
        raise ValueError("Only binary PCD files are supported")
    if not fields or not sizes or not types:
        raise ValueError("PCD header is missing FIELDS/SIZE/TYPE")
    if not counts:
        counts = [1] * len(fields)
    if points is None:
        raise ValueError("PCD header is missing POINTS")

    dtype_fields = []
    for field, size, typ, count in zip(fields, sizes, types, counts):
        if typ == "F" and size == 4:
            base = np.float32
        elif typ == "F" and size == 8:
            base = np.float64
        elif typ == "U" and size == 4:
            base = np.uint32
        elif typ == "I" and size == 4:
            base = np.int32
        else:
            raise ValueError(f"Unsupported PCD field type: {field} {typ}{size}")
        dtype_fields.append((field, base) if count == 1 else (field, base, (count,)))

    return header, offset, points, np.dtype(dtype_fields)


def rotate_pcd(input_pcd, output_pcd, angle_deg, chunk_points):
    header, offset, points, dtype = parse_pcd_header(input_pcd)
    if "x" not in dtype.names or "y" not in dtype.names:
        raise ValueError("PCD must contain x and y fields")

    angle = math.radians(angle_deg)
    c = np.float32(math.cos(angle))
    s = np.float32(math.sin(angle))

    expected_size = offset + points * dtype.itemsize
    actual_size = os.path.getsize(input_pcd)
    if actual_size < expected_size:
        raise ValueError(f"PCD data is shorter than expected: {actual_size} < {expected_size}")

    src = np.memmap(input_pcd, mode="r", dtype=dtype, offset=offset, shape=(points,))
    with open(output_pcd, "wb") as f:
        for line in header:
            f.write(line)
        for start in range(0, points, chunk_points):
            end = min(start + chunk_points, points)
            chunk = np.array(src[start:end], copy=True)
            x = chunk["x"].copy()
            y = chunk["y"].copy()
            chunk["x"] = c * x - s * y
            chunk["y"] = s * x + c * y
            if "normal_x" in dtype.names and "normal_y" in dtype.names:
                nx = chunk["normal_x"].copy()
                ny = chunk["normal_y"].copy()
                chunk["normal_x"] = c * nx - s * ny
                chunk["normal_y"] = s * nx + c * ny
            f.write(chunk.tobytes())


def main():
    parser = argparse.ArgumentParser(description="Rotate a ROS map yaml/pgm and matching binary PCD together.")
    parser.add_argument("--input-dir", default="src/my_navcar_navigation/maps/ADmap")
    parser.add_argument("--input-stem", default="AD")
    parser.add_argument("--output-stem", default="AD_aligned")
    parser.add_argument("--angle-deg", type=float, default=-8.0, help="Positive is counter-clockwise in map XY.")
    parser.add_argument("--skip-pcd", action="store_true")
    parser.add_argument("--skip-pgm", action="store_true")
    parser.add_argument("--fill-value", type=int, default=205)
    parser.add_argument("--chunk-points", type=int, default=1_000_000)
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    input_yaml = input_dir / f"{args.input_stem}.yaml"
    input_pcd = input_dir / f"{args.input_stem}.pcd"
    output_yaml = input_dir / f"{args.output_stem}.yaml"
    output_pgm = input_dir / f"{args.output_stem}.pgm"
    output_pcd = input_dir / f"{args.output_stem}.pcd"

    if not args.skip_pgm:
        print(f"Rotating PGM/YAML by {args.angle_deg} deg -> {output_yaml}")
        rotate_yaml_and_pgm(input_yaml, output_yaml, output_pgm, args.angle_deg, args.fill_value)

    if not args.skip_pcd:
        print(f"Rotating PCD by {args.angle_deg} deg -> {output_pcd}")
        rotate_pcd(input_pcd, output_pcd, args.angle_deg, args.chunk_points)

    print("Done.")


if __name__ == "__main__":
    main()
