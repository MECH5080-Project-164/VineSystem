#!/usr/bin/env python3

"""
Simple launch file for core Vine System components
Launches the essential nodes with minimal configuration
Includes micro-ROS agent for Pico communication
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a simple launch description for core vine system nodes."""
    
    return LaunchDescription([
        # Micro-ROS agent - FIRST to establish Pico communication
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'source /ros_env_setup.sh && '
                 'PICO_DEVICE=$(bash /home/workspace/students/VineSystem/scripts/find_pico.sh --first) && '
                 'if [ -n "$PICO_DEVICE" ]; then '
                 'echo "Starting micro_ros_agent on $PICO_DEVICE" && '
                 'ros2 run micro_ros_agent micro_ros_agent serial --dev $PICO_DEVICE; '
                 'else '
                 'echo "No Pico device found, micro_ros_agent not started"; '
                 'fi'],
            output='screen'
        ),
        
        # Pressure sensor node - core sensing capability
        Node(
            package='gravity_pressure_sensor',
            executable='pressure_reading_node',
            name='pressure_sensor',
            parameters=[{
                'publish_rate_hz': 50.0,  # High frequency for responsive control
                'mean_sample_size': 1,    # Minimal averaging for low latency
            }],
            output='screen'
        ),
        
        # Camera nodes
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pi_camera',
            parameters=[{
                'camera': 0,  # Default camera index
            }],
            output='screen'
        ),
        
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='endoscope_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg',
                'framerate': 30.0,
            }],
            output='screen'
        ),
        
        # Chassis LED control
        Node(
            package='led_control_chassis',
            executable='led_control_chassis_node',
            name='chassis_leds',
            output='screen'
        ),
        
        # Vine LED control
        Node(
            package='led_control_vine',
            executable='led_control_vine_node',
            name='vine_leds',
            output='screen'
        ),
        
        # Pressure control system
        Node(
            package='pressure_control',
            executable='pressure_control_node',
            name='pressure_controller',
            parameters=[{
                'target_pressure': 100.0,    # Target pressure in kPa
                'control_frequency': 20.0,   # Control loop frequency in Hz
            }],
            output='screen'
        ),
    ])
