# Nav2 MPPI/DWB runtime benchmark

This package measures two different quantities:

- `TimingController` wraps the selected Nav2 controller and publishes the wall
  time spent inside each `computeVelocityCommands()` call.
- `controller_runtime_monitor.py` records controller-server CPU, RSS/PSS,
  command output periods, direct compute-time percentiles, and deadline misses.

The timer stops before publishing the timing sample, so timing-topic overhead is
not included in `compute_time_ms`.

## Build

```bash
cd /home/wheeltec/xz/rtabmap_nav2_stack
source /home/wheeltec/wheeltec_ros2/install/setup.bash
colcon build --symlink-install --packages-select nav2_controller_benchmark
source install/setup.bash
```

## Generate test configurations

Run from the source package so the DWB defaults path is available:

```bash
PKG=/home/wheeltec/xz/rtabmap_nav2_stack/src/nav2_controller_benchmark
BASE=/home/wheeltec/xz/rtabmap_nav2_stack/src/robot_bringup/config/nav2_common.yaml

python3 $PKG/scripts/prepare_benchmark_params.py \
  --controller mppi --base $BASE --output /tmp/nav2_benchmark_mppi.yaml
python3 $PKG/scripts/prepare_benchmark_params.py \
  --controller dwb --base $BASE --output /tmp/nav2_benchmark_dwb.yaml
```

The generated files leave the original configuration untouched. Review and tune
`config/dwb_defaults.yaml` for the robot before a motion test.

## Run one trial

Start navigation with exactly one generated parameter file:

```bash
ros2 launch robot_bringup bringup.launch.py \
  mode:=navigation nav2_params_file:=/tmp/nav2_benchmark_mppi.yaml
```

In another sourced shell:

```bash
ros2 run nav2_controller_benchmark controller_runtime_monitor.py \
  --label mppi_run1 --controller-frequency 15 \
  --output /tmp/mppi_run1.csv
```

Repeat the same route with DWB and a new output file. Only rows with `active=1`
belong to active FollowPath computation. Each run also creates a raw per-call
file such as `/tmp/mppi_run1_compute.csv`.

Use at least five alternating trials (`MPPI, DWB, MPPI, DWB, ...`) after one
warm-up run per controller. Keep the map, route, controller frequency, costmaps,
sensor input, speed limits, and background processes unchanged.

Summarize completed trials:

```bash
ros2 run nav2_controller_benchmark summarize_benchmark.py \
  --controller-frequency 15 \
  /tmp/mppi_run1.csv /tmp/mppi_run2.csv \
  /tmp/dwb_run1.csv /tmp/dwb_run2.csv
```

Use the same label (for example `mppi`) for trials that should be pooled into
one summary row, and distinct output filenames for every trial.
