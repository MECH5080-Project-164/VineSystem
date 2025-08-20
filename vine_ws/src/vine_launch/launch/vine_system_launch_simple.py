#!/usr/bin/env python3

"""
Launch file for the complete Vine System
Launches all nodes required for the vine robotic system including:
- Micro-ROS agent (first, for Pico communication)
- Pressure sensor reading
- LED control for chassis and vine
- Pressure control system
- Camera systems
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    """Generate launch description for the complete vine system."""

    # Environment setup - ensure all workspaces are sourced
    env_vars = os.environ.copy()

    # Add workspace setup to environment if running from build
    workspace_path = '/home/workspace/students/VineSystem/vine_ws'
    if os.path.exists(f'{workspace_path}/install/setup.bash'):
        # Note: ROS launch will automatically source the install space
        pass

    # Declare launch arguments for optional nodes
    launch_micro_ros_arg = DeclareLaunchArgument(
        'launch_micro_ros',
        default_value='true',
        description='Whether to launch the micro-ROS agent for Pico communication'
    )
    launch_pressure_sensor_arg = DeclareLaunchArgument(
        'launch_pressure_sensor',
        default_value='true',
        description='Whether to launch the pressure sensor node'
    )

    # Launch configuration variables
    launch_micro_ros = LaunchConfiguration('launch_micro_ros')
    launch_pressure_sensor = LaunchConfiguration('launch_pressure_sensor')

    # Micro-ROS agent - FIRST to establish Pico communication
    # Micro-ROS agent launcher script now lives in scripts/tooling
    micro_ros_agent = ExecuteProcess(
        cmd=['bash', '-c',
             'set -e; '
             'source /ros_env_setup.sh || true; '
             'FIND_PICO_SCRIPT="/home/workspace/students/VineSystem/scripts/tooling/find_pico.sh"; '
             'if [ ! -x "$FIND_PICO_SCRIPT" ]; then '
             '  echo "[micro_ros_agent] Pico finder script not found at $FIND_PICO_SCRIPT; skipping micro-ROS agent."; '
             '  exit 0; '
             'fi; '
             'PICO_DEVICE=$($FIND_PICO_SCRIPT --first || true); '
             'if [ -n "$PICO_DEVICE" ]; then '
             '  echo "[micro_ros_agent] Starting micro_ros_agent on $PICO_DEVICE"; '
             '  exec ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_DEVICE; '
             'else '
             '  echo "[micro_ros_agent] No Pico device found; agent not started."; '
             'fi' ],
        output='screen',
        condition=IfCondition(launch_micro_ros)
    )

    # Pressure sensor node
    pressure_sensor_node = Node(
        package='gravity_pressure_sensor',
        executable='pressure_reading_node',
        name='pressure_sensor_node',
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x16,
            'publish_rate_hz': 50.0,
            'mean_sample_size': 1
        }],
        output='screen',
        condition=IfCondition(launch_pressure_sensor),
        # Ensure workspace is sourced
        additional_env={'ROS_DOMAIN_ID': '0'}
    )

    # System startup message
    startup_message = LogInfo(
        msg=[
            'Vine System launch configuration:\n',
            '  - Micro-ROS Agent requested: ', launch_micro_ros, '\n',
            '  - Pressure Sensor requested: ', launch_pressure_sensor, '\n',
            'Will attempt to locate Pico via scripts/tooling/find_pico.sh before starting agent.'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        launch_micro_ros_arg,
        launch_pressure_sensor_arg,

        # Startup message
        startup_message,

        # Nodes (micro-ROS agent FIRST)
        micro_ros_agent,
        pressure_sensor_node
    ])
