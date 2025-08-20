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


## 5. Topics and message usage

### LED control topics

- `/led_control/vine` — `vine_interfaces/msg/VineLed` (fields: `brightness`, `duration`, `force`)
- `/led_control/chassis` — `vine_interfaces/msg/ChassisLed` (fields: `brightness`)

Examples:

    ```bash
    ros2 topic pub -1 /led_control/vine vine_interfaces/msg/VineLed "{brightness: 80, duration: 1000, force: false}"
    ros2 topic pub -1 /led_control/chassis vine_interfaces/msg/ChassisLed "{brightness: 50}"
    ```

## 6. Helper scripts
