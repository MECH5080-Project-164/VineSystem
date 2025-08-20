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
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


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
    
    launch_cameras_arg = DeclareLaunchArgument(
        'launch_cameras',
        default_value='true',
        description='Whether to launch the camera nodes (Pi camera and USB endoscope)'
    )
    
    launch_led_control_arg = DeclareLaunchArgument(
        'launch_led_control',
        default_value='true',
        description='Whether to launch the LED control nodes'
    )
    
    launch_pressure_control_arg = DeclareLaunchArgument(
        'launch_pressure_control',
        default_value='true',
        description='Whether to launch the pressure control node'
    )
    
    # Launch configuration variables
    launch_micro_ros = LaunchConfiguration('launch_micro_ros')
    launch_pressure_sensor = LaunchConfiguration('launch_pressure_sensor')
    launch_cameras = LaunchConfiguration('launch_cameras')
    launch_led_control = LaunchConfiguration('launch_led_control')
    launch_pressure_control = LaunchConfiguration('launch_pressure_control')
    
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
    
    # Camera nodes group
    camera_group = GroupAction([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pi_camera',
            parameters=[{
                'camera': 0,  # Default Pi camera index
            }],
            output='screen',
            remappings=[
                ('image_raw', '/vine_system/pi_camera/image_raw'),
                ('camera_info', '/vine_system/pi_camera/camera_info'),
            ]
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
                'camera_name': 'endoscope',
                'camera_info_url': '',
            }],
            output='screen',
            remappings=[
                ('image_raw', '/vine_system/endoscope/image_raw'),
                ('camera_info', '/vine_system/endoscope/camera_info'),
            ]
        ),
    ], condition=IfCondition(launch_cameras))
    
    # LED control nodes group
    led_control_group = GroupAction([
        Node(
            package='led_control_chassis',
            executable='led_control_chassis_node',
            name='led_control_chassis_node',
            output='screen',
            remappings=[
                ('chassis_led', '/vine_system/chassis_led'),
            ]
        ),
        Node(
            package='led_control_vine',
            executable='led_control_vine_node',
            name='led_control_vine_node',
            output='screen',
            remappings=[
                ('vine_led', '/vine_system/vine_led'),
            ]
        ),
    ], condition=IfCondition(launch_led_control))
    
    # Pressure control node
    pressure_control_node = Node(
        package='pressure_control',
        executable='pressure_control_node',
        name='pressure_control_node',
        parameters=[{
            'target_pressure': 100.0,  # kPa
            'pressure_tolerance': 5.0,  # kPa
            'control_frequency': 10.0,  # Hz
        }],
        output='screen',
        remappings=[
            ('pressure_input', '/pressure'),
            ('pressure_control_output', '/vine_system/pressure_control'),
        ],
        condition=IfCondition(launch_pressure_control)
    )
    
    # System startup message
    startup_message = LogInfo(
        msg=[
            'Starting Vine System with the following configuration:\n',
            '  - Micro-ROS Agent: ', launch_micro_ros, '\n',
            '  - Pressure Sensor: ', launch_pressure_sensor, '\n',
            '  - Cameras: ', launch_cameras, '\n',
            '  - LED Control: ', launch_led_control, '\n',
            '  - Pressure Control: ', launch_pressure_control, '\n',
            'Micro-ROS agent will start first to establish Pico communication.\n',
            'All nodes will publish to /vine_system/ namespace for organized topics.'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        launch_micro_ros_arg,
        launch_pressure_sensor_arg,
        launch_cameras_arg,
        launch_led_control_arg,
        launch_pressure_control_arg,
        
        # Startup message
        startup_message,
        
        # Nodes (micro-ROS agent FIRST)
        micro_ros_agent,
        pressure_sensor_node,
        camera_group,
        led_control_group,
        pressure_control_node,
    ])
