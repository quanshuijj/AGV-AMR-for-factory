#!/usr/bin/env python3
import argparse
import math
from pathlib import Path


def read_yaml(path):
    data = {}
    for raw_line in Path(path).read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def parse_origin(value):
    return [float(item.strip()) for item in value.strip("[]").split(",")]


def read_pgm(path):
    with Path(path).open("rb") as stream:
        magic = stream.readline().strip()
        if magic != b"P5":
            raise ValueError(f"{path} is not a raw PGM (P5) file")

        tokens = []
        while len(tokens) < 3:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path} ended before the PGM header was complete")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if max_value != 255:
            raise ValueError(f"{path} has unsupported max value {max_value}")

        data = stream.read(width * height)
        if len(data) != width * height:
            raise ValueError(f"{path} data size does not match {width}x{height}")
        return width, height, data


def write_pgm(path, width, height, data):
    with Path(path).open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(data)


def write_yaml(path, image_name, resolution, origin, meta):
    lines = [
        f"image: {image_name}",
        f"resolution: {resolution:.12g}",
        "origin: [{:.6f}, {:.6f}, {:.6f}]".format(origin[0], origin[1], origin[2]),
        f"negate: {meta.get('negate', '0')}",
        f"occupied_thresh: {meta.get('occupied_thresh', '0.65')}",
        f"free_thresh: {meta.get('free_thresh', '0.196')}",
        "",
    ]
    Path(path).write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(
        description="Crop a ROS map_server PGM/YAML map by world-coordinate bounds."
    )
    parser.add_argument("--input-yaml", required=True)
    parser.add_argument("--output-yaml", required=True)
    parser.add_argument("--xmin", type=float, required=True)
    parser.add_argument("--ymin", type=float, required=True)
    parser.add_argument("--xmax", type=float, required=True)
    parser.add_argument("--ymax", type=float, required=True)
    args = parser.parse_args()

    if args.xmin >= args.xmax or args.ymin >= args.ymax:
        raise ValueError("Crop bounds must satisfy xmin < xmax and ymin < ymax")

    input_yaml = Path(args.input_yaml)
    output_yaml = Path(args.output_yaml)
    meta = read_yaml(input_yaml)
    resolution = float(meta["resolution"])
    origin = parse_origin(meta["origin"])
    image_path = input_yaml.parent / meta["image"]

    width, height, pixels = read_pgm(image_path)

    col0 = max(0, int(math.floor((args.xmin - origin[0]) / resolution)))
    col1 = min(width, int(math.ceil((args.xmax - origin[0]) / resolution)))
    map_row0 = max(0, int(math.floor((args.ymin - origin[1]) / resolution)))
    map_row1 = min(height, int(math.ceil((args.ymax - origin[1]) / resolution)))

    if col0 >= col1 or map_row0 >= map_row1:
        raise ValueError("Crop bounds do not overlap the input map")

    # map row 0 is the bottom row in world coordinates; PGM row 0 is the top.
    img_row0 = height - map_row1
    img_row1 = height - map_row0
    new_width = col1 - col0
    new_height = img_row1 - img_row0

    cropped = bytearray(new_width * new_height)
    for out_row, in_row in enumerate(range(img_row0, img_row1)):
        in_start = in_row * width + col0
        in_end = in_start + new_width
        out_start = out_row * new_width
        cropped[out_start : out_start + new_width] = pixels[in_start:in_end]

    output_pgm = output_yaml.with_suffix(".pgm")
    new_origin = [
        origin[0] + col0 * resolution,
        origin[1] + map_row0 * resolution,
        origin[2],
    ]
    write_pgm(output_pgm, new_width, new_height, cropped)
    write_yaml(output_yaml, output_pgm.name, resolution, new_origin, meta)

    print(f"cropped {image_path} -> {output_pgm}")
    print(f"size: {width}x{height} -> {new_width}x{new_height}")
    print(
        "origin: [{:.6f}, {:.6f}, {:.6f}]".format(
            new_origin[0], new_origin[1], new_origin[2]
        )
    )


if __name__ == "__main__":
    main()
