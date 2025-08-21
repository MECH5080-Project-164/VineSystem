from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch pressure, chassis LED, vine LED and gravity pressure sensor nodes."""

    return LaunchDescription([
        Node(
            package='pressure_control',
            executable='pressure_control_node',
            name='pressure_control_node',
            output='screen',
        ),

        Node(
            package='led_control_chassis',
            executable='led_control_chassis_node',
            name='led_control_chassis',
            output='screen',
        ),

        Node(
            package='led_control_vine',
            executable='led_control_vine_node',
            name='led_control_vine',
            output='screen',
        ),

        Node(
            package='gravity_pressure_sensor',
            executable='pressure_reading_node',
            name='pressure_reading_node',
            output='screen',
        ),
    ])
