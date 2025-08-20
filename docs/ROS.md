# Working Notes for ROS

This document describes how to set up, build, run, and debug the ROS 2 components in this repository.

## 1. Environment and prerequisites

When working on a Raspberry Pi, ROS is installed in a docker container, the information for which can be found in the Dockerfile located at `docker/Dockerfile`. All work relating to ROS should be done within this container.

Information on building, starting, and connecting to the docker container can be found [here](./Docker.md).

## 2. Workspace layout

The colcon workspace is located at `vine_ws/` and follows a standard ROS 2 layout:

- `vine_ws/src/` — packages (led controllers, pressure controller, sensors, interfaces)
- `vine_ws/build/`, `vine_ws/install/`, `vine_ws/log/` — build artifacts

Key packages:

- `led_control_chassis` — chassis LED driver (C++), subscribes to `led_control/chassis`
- `led_control_vine` — vine LED driver (C++), subscribes to `led_control/vine`
- `pressure_control` — pressure control node (C++)
- `gravity_pressure_sensor` — Python package for pressure sensor readings
- `vine_interfaces` — custom messages: `ChassisLed`, `VineLed`

Additionally, there are other workspaces for other ROS nodes in `/pi_cam/` (for the Pi Camera nodes - sourced [here](https://github.com/MECH5080-Project-164/camera_ros)), in `/microros_ws/` (for the Micro-ROS agent - see the [Dockerfile](../docker/Dockerfile) for more information), and in `/pico_ws/` (in which the micro-ROS nodes are built).

These workspaces are sourced automatically due to being included in the `.bashrc` of the root user of the docker container and so should be immediately usable, though it may still be important to know these exist.

The key packages of these workspaces are:

- `camera_ros` - provides the ROS 2 interface for the ArduCam camera modules.
- `micro_ros_agent` - provides the Micro-ROS agent for communication with microcontrollers.

## 3. Build

From project root:

1. Open a shell in the Vine container (or ensure ROS 2 is sourced).
2. Build the workspace:

    ./vine_ws/build_vine_ws.sh

3. Source the workspace before running nodes:

    source vine_ws/install/setup.bash

If you use `colcon` directly:

    colcon build --workspace-dir vine_ws
    source vine_ws/install/setup.bash

## 4. Launching nodes

Nodes can be started with `ros2 run`. Example:

    ros2 run led_control_chassis led_control_chassis_node

A common order for launching nodes is:

1. Start `micro_ros_agent` - enables sensor readings from the Pi Pico
2. Start `gravity_pressure_sensor` - reads pressure data from the sensor and publishes it
3. Start `led_control_chassis` - controls the chassis LED
4. Start `led_control_vine` - controls the vine LED
5. Start any other control nodes as needed such as `pressure_control`.

There are also scripts to start these nodes in the `scripts/` directory.

## 5. Topics and message usage

### LED Control topics

- `/led_control/vine` — `vine_interfaces/msg/VineLed` (fields: `brightness`, `duration`, `force`)
- `/led_control/chassis` — `vine_interfaces/msg/ChassisLed` (fields: `brightness`)

Examples:

    ```bash
    ros2 topic pub -1 /led_control/vine vine_interfaces/msg/VineLed "{brightness: 80, duration: 1000, force: false}"
    ros2 topic pub -1 /led_control/chassis vine_interfaces/msg/ChassisLed "{brightness: 50}"
    ```

The Vine LED Control is protected such that certain brightnesses can only be run for certain durations, this is due to the risk of heat generation damaging the LED.

There are also scripts to work with the LED control topics:

- `scripts/pub_led_chassis.sh` - publishes to `/led_control/chassis`
- `scripts/pub_led_vine.sh` - publishes to `/led_control/vine`

### Pressure Control

Pressure Control is mostly controlled through parameters. As such a slightly different workflow is used as there are no topics to publish to. The key parameters are:

- `target_pressure` - the target pressure to maintain
- `kp` - the proportional gain for the pressure controller
- `ki` - the integral gain for the pressure controller
- `kd` - the derivative gain for the pressure controller
- `debug_force_pwm` - forces the PWM output to a specific value for debugging

Pump enable/disable

-- `pump_enabled` - (bool) global default that enables or disables pump control. Default: `false`.

There is also a runtime single source-of-truth topic for enabling/disabling the pump:

- `/pump/enable` — `std_msgs/msg/Bool` — publishing `true` or `false` immediately enables or disables pump control across the node.

Behavior:

- If `/pump/enable` publishes `false`, the pressure controller will immediately stop publishing PWM, reset integral/derivative state, and throttle status messages while disabled.
- The node still exposes the `pump_enabled` parameter. Setting the parameter updates the node state in the same way; however, publishing to `/pump/enable` acts as the live authoritative command source for runtime control.

Examples:

    ```bash
    # Disable pump at runtime (single publish)
    ros2 topic pub -1 /pump/enable std_msgs/msg/Bool "{data: false}"

    # Re-enable pump
    ros2 topic pub -1 /pump/enable std_msgs/msg/Bool "{data: true}"

    # Or set the parameter directly (persistent until changed)
    ros2 param set /pressure_control_node pump_enabled false
    ros2 param set /pressure_control_node pump_enabled true
    ```

There are more parameters which can be found with the command:

    ```bash
    ros2 param list /pressure_control_node
    ```

## 6. Helper scripts

There are also helper scripts available in the [`scripts/`](../scripts/) directory to facilitate common tasks such as starting nodes and publishing messages.
