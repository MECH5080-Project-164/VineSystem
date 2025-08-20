# Vine System Launch Files

This directory contains Python launch files for the Vine System ROS2 workspace.

## Available Launch Files

### 1. `vine_system_simple.py`
A simple launch file that starts all core system nodes with default parameters.

**Usage:**
```bash
ros2 launch vine_interfaces vine_system_simple.py
```

**Launched Nodes:**
- `micro_ros_agent` - Communication bridge to Raspberry Pi Pico (launched first)
- `pressure_sensor` - Reads pressure sensor data at 50Hz
- `pi_camera` - Raspberry Pi camera node
- `endoscope_camera` - USB endoscope camera node
- `chassis_leds` - Controls chassis LED indicators
- `vine_leds` - Controls vine LED indicators  
- `pressure_controller` - Manages pressure control system

### 2. `vine_system_launch.py`
A more advanced launch file with configurable options.

**Usage:**
```bash
# Launch all systems (default)
ros2 launch vine_interfaces vine_system_launch.py

# Launch only pressure sensor and control
ros2 launch vine_interfaces vine_system_launch.py launch_led_control:=false

# Launch only cameras and pressure sensor
ros2 launch vine_interfaces vine_system_launch.py launch_led_control:=false launch_pressure_control:=false

# Launch without cameras
ros2 launch vine_interfaces vine_system_launch.py launch_cameras:=false
```

**Available Parameters:**
- `launch_micro_ros:=true/false` - Enable/disable micro-ROS agent for Pico communication
- `launch_pressure_sensor:=true/false` - Enable/disable pressure sensor
- `launch_cameras:=true/false` - Enable/disable camera nodes (Pi cam + USB endoscope)
- `launch_led_control:=true/false` - Enable/disable LED controllers
- `launch_pressure_control:=true/false` - Enable/disable pressure control

## Building and Installing

The launch files automatically handle workspace sourcing. After creating or modifying launch files, rebuild the workspace:

```bash
cd /path/to/vine_ws
colcon build --packages-select vine_interfaces
source install/setup.bash
```

**Note:** The launch files will automatically:
- Source the ROS2 environment (`/opt/ros/$ROS_DISTRO/setup.bash`)
- Source the micro-ROS workspace (`/microros_ws/install/setup.bash`) 
- Source the camera workspace (`/picam/install/setup.bash`) if needed
- Detect and connect to the Raspberry Pi Pico automatically

## Topics

The launch files organize topics under the `/vine_system/` namespace:
- `/pressure` - Pressure sensor readings
- `/vine_system/pi_camera/image_raw` - Raspberry Pi camera feed
- `/vine_system/pi_camera/camera_info` - Pi camera information
- `/vine_system/endoscope/image_raw` - USB endoscope camera feed
- `/vine_system/endoscope/camera_info` - Endoscope camera information
- `/vine_system/chassis_led` - Chassis LED commands
- `/vine_system/vine_led` - Vine LED commands
- `/vine_system/pressure_control` - Pressure control output

## Monitoring

Use the temperature monitoring script to watch sensor data:
```bash
./scripts/monitor_temperatures.sh
```

## Troubleshooting

1. **Nodes fail to start**: Ensure all packages are built successfully
2. **Hardware errors**: Check I2C connections and GPIO permissions
3. **Missing topics**: Verify micro-ROS agent is running for Pico communication
