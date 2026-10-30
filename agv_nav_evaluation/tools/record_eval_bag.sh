#!/usr/bin/env bash
set -euo pipefail

ROUTE_NAME="${1:-default_route}"
TEST_ID="${2:-000}"

roslaunch agv_nav_evaluation record_eval_data.launch route_name:="${ROUTE_NAME}" test_id:="${TEST_ID}"
