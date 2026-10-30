# agv_nav_evaluation

`agv_nav_evaluation` is an independent ROS1 package for AGV navigation experiment recording, online monitoring, rosbag post-processing, and metrics/report generation.

This package does not modify any existing navigation package, launch file, parameter file, planner, localization module, CAN node, TF logic, or move_base behavior. It only subscribes to topics, records data, and analyzes offline results.

## Scope

- Record experiment data with `rosbag`
- Monitor task state online without publishing control commands
- Parse rosbag files and optional manual event CSV files
- Compute task success, efficiency, tracking, safety, smoothness, localization, and planning metrics
- Export CSV, JSON, and Markdown reports

## Important Non-Intrusive Design

- No existing file is modified
- No dynamic reconfigure is used on existing navigation nodes
- No control topic is published
- Missing topics are handled as warnings, not fatal errors
- Topic names are configurable in YAML

## Workspace Compatibility

- Designed for ROS1
- Scripts use Python 3 syntax
- Recommended for ROS Noetic
- If your current ROS1 environment still uses Python 2, you will need to adapt the shebang and dependency setup before running the scripts

## Package Layout

- `config/topics.yaml`: frames, topic names, node names, optional field mappings
- `config/vehicle.yaml`: vehicle geometry, footprint, minimum turning radius
- `config/metrics.yaml`: thresholds and metric behavior
- `launch/record_eval_data.launch`: standalone rosbag recording launcher
- `launch/online_monitor.launch`: standalone online monitor launcher
- `scripts/`: ROS entry scripts
- `src/agv_nav_evaluation/`: reusable Python modules
- `tools/`: helper shell scripts
- `output/`: suggested output directory

## Build

```bash
catkin_make
source devel/setup.bash
```

## GUI Dashboard

The package also includes an independent PyQt5 dashboard for experiment recording, topic health monitoring, event annotation, and post-processing orchestration.

Launch it with:

```bash
rosrun agv_nav_evaluation eval_dashboard.py
```

If PyQt5 is missing, install it with:

```bash
sudo apt install python3-pyqt5
```

The GUI is read-only with respect to the navigation system:

- it does not publish `cmd_vel`
- it does not change move_base, SBPL, TEB, localization, TF, or CAN parameters
- it does not directly control vehicle motion
- it only starts/stops `rosbag record`, records manual events, and calls offline analysis scripts

## Configure Topics

Edit the YAML files under `config/`.

- Update `topics.yaml` with your real topic names
- Update `vehicle.yaml` with your real footprint and turning limits
- Update `metrics.yaml` with your thresholds

The default configuration is conservative and accepts missing topics.

## Online Monitor

```bash
roslaunch agv_nav_evaluation online_monitor.launch
```

This monitor only subscribes to topics and prints:

- goal reception state
- move_base status/result
- cmd_vel frequency
- global/local plan update state
- RTK state
- TF health
- CAN feedback health

It does not publish any control command.

## Record Real-Vehicle Data

```bash
roslaunch agv_nav_evaluation record_eval_data.launch route_name:=test_route_01 test_id:=001
```

This launch starts an independent recorder script which:

- reads topics from `config/topics.yaml`
- creates a timestamped output directory
- launches `rosbag record` as a subprocess
- keeps recording until interrupted

## GUI Workflow

1. Start the GUI:

```bash
rosrun agv_nav_evaluation eval_dashboard.py
```

2. Fill in:

- `test_id`
- `route_name`
- `map_id`
- `vehicle_id`
- `operator`
- `weather_or_scene`
- `notes`
- `bag_output_dir`

3. Press `Start Recording`

The GUI will:

- read record topics from `config/topics.yaml`
- create a timestamped bag file
- create `metadata.yaml`
- create `manual_events.csv`
- start `rosbag record` in a child process

4. Use the event buttons to annotate:

- `collision`
- `near_miss`
- `emergency_stop`
- `human_intervention`
- `localization_lost`
- `planning_failure`
- `controller_failure`
- `obstacle_blocked`
- `other`

5. Press `Stop Recording`

The GUI will:

- send `SIGINT` to `rosbag record`
- update `metadata.yaml`
- save `manual_events.csv`

6. Press `Analyze Current Bag`

The GUI calls `analyze_bag.py` in a child process and writes results into the current bag folder.

7. Press `Generate Report`

The GUI calls `generate_report.py` and creates `report.md`.

## Analyze a Rosbag

```bash
rosrun agv_nav_evaluation analyze_bag.py --bag path/to/test.bag --config config/topics.yaml --vehicle config/vehicle.yaml --metrics config/metrics.yaml --output output/test_001
```

Outputs:

- `metrics_summary.csv`
- `metrics_timeseries.csv`
- `metrics_summary.json`
- `report.md`

Optional:

- `manual_events.csv` for human intervention, collision, emergency stop, and other annotations
- ground truth CSV for ATE/RPE if available

## Batch Analysis

```bash
bash tools/batch_analyze_bags.sh data/bags output/batch_results
```

## Manual Event Template

```bash
rosrun agv_nav_evaluation export_manual_event_template.py --output output/manual_events.csv
```

Template fields:

- `test_id`
- `timestamp`
- `event_type`
- `description`
- `severity`
- `x`
- `y`
- `yaw`

## Output Files

- `metrics_summary.csv`: one-row summary table for the experiment
- `metrics_timeseries.csv`: time series data used in metrics computation
- `metrics_summary.json`: structured machine-readable summary
- `report.md`: readable experiment report
- `metadata.yaml`: experiment metadata recorded by the GUI
- `manual_events.csv`: event annotations recorded manually during testing

## Metrics Overview

- Task success: success rate, failure count/type, intervention count, collision count
- Efficiency: completion time, path length, replanning statistics, speed
- Tracking: CTE, heading error, final pose error
- Safety: clearance, near-miss count, safety stop count
- Smoothness: acceleration, jerk, curvature, turning radius, tracking error
- Localization: RTK state ratios, dropouts, ATE/RPE when GT exists
- Real-time performance: planning time, control loop frequency, latency, CPU, memory

## Ground Truth Handling

ATE and RPE are computed only when a ground truth trajectory is provided.

If ground truth is unavailable, the report will explicitly state:

- `GT unavailable, skip ATE/RPE`

## FAQ

### Some topics do not exist in my project

That is supported. The scripts log warnings and keep running. Related metrics are marked as `NaN` or `unavailable`.

### Will this affect SBPL, TEB, move_base, localization, CAN, TF, or costmap?

No. This package is intentionally read-only with respect to the existing navigation system.

### How do I change topic names?

Edit `config/topics.yaml`. The code is structured so that topic names are not hardcoded in the metric modules.

### How does the GUI handle missing topics?

The GUI shows `Missing` or `Timeout` in the topic health table and keeps running.

### How do I confirm the GUI does not control the vehicle?

- inspect the source of `scripts/eval_dashboard.py`
- confirm it only subscribes to status topics
- confirm it never publishes `cmd_vel`
- confirm it only launches `rosbag record` and offline analysis subprocesses

### How do I verify it is non-intrusive?

- confirm that only `rosbag record` and read-only monitor nodes are launched
- confirm no `/cmd_vel`, planner config, or TF publishing comes from this package
- confirm existing navigation launch files are unchanged
