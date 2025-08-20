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
from launch import LaunchConfiguration, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition


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

    # Launch configuration variables
    launch_micro_ros = LaunchConfiguration('launch_micro_ros')

    # Micro-ROS agent - FIRST to establish Pico communication
    micro_ros_agent = ExecuteProcess(
        cmd=['bash', '-c',
             'source /ros_env_setup.sh && '
             'PICO_DEVICE=$(bash /home/workspace/students/VineSystem/scripts/find_pico.sh --first) && '
             'if [ -n "$PICO_DEVICE" ]; then '
             'echo "Starting micro_ros_agent on $PICO_DEVICE" && '
             'ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_DEVICE; '
             'else '
             'echo "No Pico device found, micro_ros_agent not started"; '
             'fi'],
        output='screen',
        condition=IfCondition(launch_micro_ros)
    )

    # System startup message
    startup_message = LogInfo(
        msg=[
            'Starting Vine System with the following configuration:\n',
            '  - Micro-ROS Agent: ', launch_micro_ros, '\n',
            'Micro-ROS agent will start first to establish Pico communication.\n',
            'All nodes will publish to /vine_system/ namespace for organized topics.'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        launch_micro_ros_arg,

        # Startup message
        startup_message,

        # Nodes (micro-ROS agent FIRST)
        micro_ros_agent,
    ])
