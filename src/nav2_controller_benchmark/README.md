# Nav2 MPPI runtime benchmark

This package measures two different quantities:

- `TimingController` wraps the selected Nav2 controller and publishes the wall
  time spent inside each `computeVelocityCommands()` call.
- `controller_runtime_monitor.py` records controller-server CPU, RSS/PSS,
  command output periods, direct compute-time percentiles, and deadline misses.

The timer stops before publishing the timing sample, so timing-topic overhead is
not included in `compute_time_ms`.

## Build

```bash
cd .
source $HOME/wheeltec_ros2/install/setup.bash
colcon build --symlink-install --packages-select nav2_controller_benchmark
source install/setup.bash
```

## Generate test configurations

Generate a timing-wrapper configuration from the existing MPPI parameters:

```bash
PKG=./src/nav2_controller_benchmark
BASE=./src/robot_bringup/config/nav2_mppi.yaml

python3 $PKG/scripts/prepare_benchmark_params.py \
  --controller mppi --base $BASE --output /tmp/nav2_benchmark_mppi.yaml
```

The generated file leaves the original configuration untouched.

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

Only rows with `active=1` belong to active FollowPath computation. Each run
also creates a raw per-call file such as `/tmp/mppi_run1_compute.csv`.

Use at least five trials after one warm-up run. Keep the map, route, controller
frequency, costmaps, sensor input, speed limits, and background processes
unchanged.

Summarize completed trials:

```bash
ros2 run nav2_controller_benchmark summarize_benchmark.py \
  --controller-frequency 15 \
  /tmp/mppi_run1.csv /tmp/mppi_run2.csv
```

Use the same label (for example `mppi`) for trials that should be pooled into
one summary row, and distinct output filenames for every trial.
