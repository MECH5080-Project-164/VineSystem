# VineSystem Launch Guide

Quick reference for launching and operating the complete VineSystem with pressure control.

## Quick Start

### 0. Prerequisites

- (Optional) Enter `tmux` or similar terminal multiplexer for better terminal management.
- Enter `~/students/VineSystem` directory in your terminal.

### 1. Launch Docker Environment

```bash
# Start docker container with docker-compose
docker compose up -d
```

### 2. Enter Container

```bash
# Enter the running container
docker exec -it vine-docker bash
```

Or using the script:

```bash
./scripts/start_docker.sh
```

### 3. Build Workspace

```bash
# Navigate to workspace and build
cd /home/workspace/students/VineSystem/vine_ws
./build_vine_ws.sh
source install/setup.bash
```

### 4. Start Core Components

#### Terminal 1 - Micro ROS Agent (Pico Communication)

```bash
cd /home/workspace/students/VineSystem/scripts
./start_micro_ros_agent.sh
```

#### Terminal 2 - Pressure Sensor

```bash
cd /home/workspace/students/VineSystem/scripts
./start_pressure_sensor.sh
```

#### Terminal 3 - Pressure Control

```bash
cd /home/workspace/students/VineSystem/scripts
./start_pressure_control.sh
```

## System Components

### Docker Environment

- **Container**: `vine-docker` (ROS 2 Jazzy + dependencies)
- **Volumes**: Maps local workspace to `/home/workspace/students/VineSystem`
- **Ports**: Exposes necessary ROS 2 communication ports

### ROS 2 Nodes

| Node | Package | Function | Topics |
|------|---------|----------|---------|
| `pressure_reading_node` | `gravity_pressure_sensor` | Reads pressure sensor | Publishes: `pressure` |
| `pressure_control_node` | `pressure_control` | PID pressure control | Subscribes: `pressure`<br>Publishes: `pump_pwm_control` |
| `micro_ros_agent` | System | Pico communication bridge | Bridges serial ↔ ROS 2 |

### Key Topics

- **`pressure`** (`Float32`) - Current pressure in kPa
- **`pump_pwm_control`** (`Int32`) - PWM command for pump (0-100)

## Configuration & Tuning

### Pressure Control Parameters

```bash
# View current parameters
ros2 param list /pressure_control_node

# Set target pressure (kPa)
ros2 param set /pressure_control_node target_pressure 120.0

# Tune PID gains
ros2 param set /pressure_control_node kp 1.0
ros2 param set /pressure_control_node ki 0.2
ros2 param set /pressure_control_node kd 0.05

# Debug mode - force PWM output
ros2 param set /pressure_control_node debug_force_pwm 100  # Force 100% PWM
ros2 param set /pressure_control_node debug_force_pwm -1   # Disable debug mode
```

## Troubleshooting

### Pico Connection Issues

```bash
# List available Pico devices
./scripts/tooling/find_pico.sh --list

# Start agent with auto-detection
./scripts/start_micro_ros_agent.sh

# Start agent with specific device
./scripts/start_micro_ros_agent.sh --device /dev/ttyACM0

# Start agent with verbose output
./scripts/start_micro_ros_agent.sh --verbose

# Check if Pico appears in system
lsusb | grep -i [Pp]ico
ls /dev/tty*
```

### Pressure Sensor Issues

```bash
# Check sensor connection
ros2 topic echo /pressure

# Start sensor with debug logging
./scripts/start_pressure_sensor.sh --debug

# Start sensor with verbose output
./scripts/start_pressure_sensor.sh --verbose
```

### Control System Issues

```bash
# Monitor control output
ros2 topic echo /pump_pwm_control

# View control system status
ros2 topic echo /rosout | grep pressure_control

# Check parameter values
ros2 param get /pressure_control_node target_pressure
```

## Monitoring

### Real-time Topic Monitoring

```bash
# Monitor all key topics
ros2 topic echo /pressure &
ros2 topic echo /pump_pwm_control &
```

### System Status

```bash
# Check running nodes
ros2 node list

# View node information
ros2 node info /pressure_control_node
ros2 node info /pressure_reading_node
```

## Shutdown Sequence

1. **Stop control node**: `Ctrl+C` in pressure control terminal
2. **Stop sensor node**: `Ctrl+C` in pressure sensor terminal
3. **Stop micro_ros_agent**: `Ctrl+C` in agent terminal
4. **Exit container**: `exit`
5. **Stop Docker**: `docker-compose down`

## Common Workflows

### Development Testing

```bash
# 1. Start system with debug PWM
ros2 param set /pressure_control_node debug_force_pwm 50

# 2. Monitor pressure response
ros2 topic echo /pressure

# 3. Disable debug and tune PID
ros2 param set /pressure_control_node debug_force_pwm -1
ros2 param set /pressure_control_node kp 0.8
```

### Production Operation

```bash
# 1. Set target pressure
ros2 param set /pressure_control_node target_pressure 150.0

# 2. Monitor system performance
ros2 topic echo /pressure --field data
ros2 topic echo /pump_pwm_control --field data

# 3. Adjust gains as needed
ros2 param set /pressure_control_node ki 0.15
```

## Key File Locations

- **Docker Config**: `docker-compose.yml`, `docker_config/`
- **Scripts**: `scripts/` (start scripts, utilities)
- **Workspace**: `vine_ws/` (ROS 2 packages)
- **Launch Files**: `launch/` (system launch configurations)
- **Documentation**: `docs/` (component-specific docs)

---

Tip: Keep multiple terminals open for real-time monitoring while operating the system. I do this with `tmux` which should be installed on the base system. If my (Sam's) tmux configuration is being used, the prefix is `Ctrl+space` and the configuration should be in `~/.config/tmux/tmux.conf`. Otherwise, the default prefix is `Ctrl+b` and you can use `Ctrl+b ?` to see the key bindings.
