# Vine Launch Files

This package contains ROS 2 launch files for the VineSystem workspace.

## Available Launch Files

### 1. `vine_system_simple.py`

A simple launch file that starts all core system nodes with default parameters.

**Usage:**

    ```bash
    ros2 launch vine_launch vine_system_simple.py
    ```

### 2. `vine_system_launch.py`
A more advanced launch file with configurable options.

**Usage:**

    ```bash
    # Launch all systems (default)
    ros2 launch vine_launch vine_system_launch.py

    # Launch only pressure sensor and control
    ros2 launch vine_launch vine_system_launch.py launch_led_control:=false
    ```

## Building and Installing

After creating or modifying launch files, rebuild the workspace:

    ```bash
    cd /home/workspace/students/VineSystem/vine_ws
    colcon build --packages-select vine_launch
    source install/setup.bash
    ```

## Endoscope camera auto-configuration

The `vine_system_launch.py` launcher supports automatic endoscope device detection and
parameter file modification using the helper script at `scripts/tooling/find_endoscope.sh`.

Launch arguments:

- `endo_params_file` (default: `vine_ws/src/vine_launch/resources/endo_cam_params.yaml`) — path to the endoscope `usb_cam` parameter YAML.
- `auto_config_endoscope` (true/false) — when true, the launch will run the find-and-modify script before starting the camera node.

Examples:

    ```bash
    # Auto-detect and configure endoscope before launching
    ros2 launch vine_launch vine_system_launch.py auto_config_endoscope:=true

    # Use a custom params file and disable auto-config
    ros2 launch vine_launch vine_system_launch.py endo_params_file:=/path/to/params.yaml auto_config_endoscope:=false
    ```
