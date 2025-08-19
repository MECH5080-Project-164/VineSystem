# Vine System ROS Workspace

## Overview

This workspace contains ROS nodes for controlling the LEDs on the Vine System delployment chassis and the LED on the Vine end-effector. It also contains custom interfaces for working with these nodes.

## Installation

To build this workspace, with ROS Jazzy installed/in the vine container, run the following command:

```bash
colcon build --symlink-install
```

## Usage

### Environment Setup

When using ROS, the appropriate environment variables need to be sourced. This can be done by running:

```bash
source /opt/ros/jazzy/setup.bash
```

And, from the root of this workspace:

```bash
source install/setup.bash
```

Note that whenever anything related to the ROS applications in the vine_ws is used in a terminal, then the ROS environment must be sourced. For example, when starting a ROS node or publishing to a topic, the environment must be set up first.

### Running Nodes

In one terminal, run:

```bash
ros2 run led_control_[chassis/vine] led_control_[chassis/vine]_node
```

This will start the LED control node for the chassis or the end-effector, respectively.

In another terminal, you can control the LEDs by publishing to the appropriate topic. For example, to turn on the chassis LED, you can run:

```bash
ros2 topic pub -1 /[chassis/vine]_led_control vine_interfaces/msg/[Chassis/Vine]Led <payload as JSON>
```

To determine the payload format, you can refer to the message definitions in the `vine_interfaces` package.

#### Example Payload

For a Vine LED payload, which has a brightness and duration to protect the LED, you can use:

```bash
ros2 topic pub -1 /led_control/vine vine_interfaces/msg/VineLed "{duration: 100, brightness: 50}"
```

For a Chassis LED payload, which only has a brightness, you can use:

```bash
ros2 topic pub -1 /led_control/chassis vine_interfaces/msg/ChassisLed "{brightness: 50}"
```
