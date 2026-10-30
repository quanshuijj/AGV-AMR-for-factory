#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <bag_dir> <output_dir>"
  exit 1
fi

BAG_DIR="$1"
OUTPUT_DIR="$2"

for bag in "${BAG_DIR}"/*.bag; do
  [ -e "${bag}" ] || continue
  name="$(basename "${bag}" .bag)"
  rosrun agv_nav_evaluation analyze_bag.py \
    --bag "${bag}" \
    --config "$(rospack find agv_nav_evaluation)/config/topics.yaml" \
    --vehicle "$(rospack find agv_nav_evaluation)/config/vehicle.yaml" \
    --metrics "$(rospack find agv_nav_evaluation)/config/metrics.yaml" \
    --output "${OUTPUT_DIR}/${name}" \
    --test-id "${name}"
done
