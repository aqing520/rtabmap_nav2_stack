# BT-468 RTK Driver

ROS 2 BT-468 RTK GNSS package with a standalone serial CLI and a ROS 2 node.

This project currently provides:
- a standalone serial CLI for bring-up
- a ROS 2 node publishing GNSS fixes and status
- NMEA parsing for `GGA`, `RMC`, and `VTG`

The module datasheet indicates:
- UART output over TTL
- default baud rate: `38400`
- protocols: `NMEA`, `UBX`, `RTCM 3.3`
- output sentences include `GGA`, `RMC`, `VTG`, `GSA`, `GSV`, `GLL`

## Layout

- `src/bt468_rtk_driver/nmea.py`: checksum validation and NMEA sentence parsing
- `src/bt468_rtk_driver/serial_reader.py`: serial port read loop
- `src/bt468_rtk_driver/cli.py`: command line entry point
- `src/bt468_rtk_driver/ros2_node.py`: ROS 2 node publishing GNSS data
- `launch/bt468_rtk.launch.py`: ROS 2 launch entry
- `config/bt468_rtk.yaml`: default ROS 2 parameters

## ROS 2 Build

On the target machine:

```bash
source /opt/ros/humble/setup.bash
cd /data/workspace/pans
colcon build --base-paths /data/workspace/pans/bt468_rtk_driver
source install/setup.bash
```

## Run ROS 2 Node

```bash
ros2 run bt468_rtk_driver bt468_rtk_node
```

Or with launch:

```bash
ros2 launch bt468_rtk_driver bt468_rtk.launch.py
```

Topics:
- `/fix`: `sensor_msgs/NavSatFix`
- `/fix_status`: `std_msgs/String`
- `/nmea_sentence`: `std_msgs/String`

## Run CLI

Read live data from the default BT-468 serial node on the Jetson:

```bash
source /opt/ros/humble/setup.bash
python3 -m bt468_rtk_driver.cli --port auto --baud 38400 --max-lines 20
```

If the device is exposed as a different port, adjust `--port` accordingly.

## Output

The CLI prints parsed JSON and also emits a separate summary line for `GGA`:

Example:

```json
{"type":"GGA","talker":"GN","fields":{"time":"090020.00","lat":22.680677,"lon":114.045125,"fix_quality":1,"num_sv":12,"hdop":0.48,"altitude_m":75.358,"geoid_separation_m":-2.521}}
{"定位状态":"普通定位","纬度":"22.680677462","经度":"114.045125012"}
```

## Remote integration next steps

1. Confirm the actual port name on `192.168.110.151` or `192.168.110.50`.
2. Decide whether the module should run as a polling process, a ROS 2 node, or a service publishing parsed fixes.
3. Add RTCM input handling on the RX pin if the same process will also forward correction data.
4. Add device-specific configuration commands if the boot mode, talker set, or update rate must be changed from defaults.
5. Add integration tests against a recorded NMEA log before touching the live device.

## Local Checks

Run the standard-library tests:

```bash
python3 -m unittest discover -s tests
```
