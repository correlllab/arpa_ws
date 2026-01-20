# Linear Actuator Hardware Interface

This ROS2 package demonstrates a complete implementation of a hardware interface for a linear actuator using the `hardware_interface::ActuatorInterface` from ros2_control.

## Overview

This package provides:
- **Hardware Interface**: Complete implementation of `LinearActuatorHardware` with position, velocity, and effort control
- **URDF/Xacro**: Robot description with ros2_control configuration
- **Controllers**: Pre-configured position, velocity, and effort controllers
- **Launch Files**: Easy startup and testing
- **Simulation Mode**: Test without physical hardware

## Package Structure

```
linear_actuator_hardware/
├── config/
│   ├── controllers.yaml          # Controller configurations
│   └── linear_actuator.rviz      # RViz visualization config
├── include/
│   └── linear_actuator_hardware/
│       └── linear_actuator_hardware.hpp
├── launch/
│   └── linear_actuator.launch.py # Main launch file
├── src/
│   └── linear_actuator_hardware.cpp
├── urdf/
│   ├── linear_actuator_robot.urdf.xacro
│   └── linear_actuator.ros2_control.xacro
├── CMakeLists.txt
├── package.xml
└── linear_actuator_hardware.xml  # Plugin description
```

## Key Features

### Hardware Interface Implementation

The `LinearActuatorHardware` class implements:
- **State Interfaces**: position, velocity, effort
- **Command Interfaces**: position, velocity, effort
- **Lifecycle Management**: proper activation/deactivation
- **Safety Limits**: enforces min/max position, velocity, and effort
- **Simulation Mode**: for testing without hardware

### Supported Controllers

1. **Position Controller**: Send target positions
2. **Velocity Controller**: Send target velocities
3. **Effort Controller**: Send target forces/efforts
4. **Joint State Broadcaster**: Publishes current state

## Building

```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Clone or copy this package
# (package should be in ~/ros2_ws/src/linear_actuator_hardware)

# Build
cd ~/ros2_ws
colcon build --packages-select linear_actuator_hardware

# Source
source install/setup.bash
```

## Usage

### Launch in Simulation Mode

```bash
ros2 launch linear_actuator_hardware linear_actuator.launch.py use_sim:=true gui:=true
```

### Launch with Real Hardware

```bash
ros2 launch linear_actuator_hardware linear_actuator.launch.py \
    use_sim:=false \
    device_port:=/dev/ttyUSB0
```

### Activate Controller

```bash
# Activate the position controller
ros2 control set_controller_state position_controller active
```

### Send Commands

#### Position Control
```bash
# Move to 0.25 meters
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.25]" --once

# Move to home position
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.0]" --once
```

#### Velocity Control
```bash
# First, switch to velocity controller
ros2 control set_controller_state position_controller inactive
ros2 control set_controller_state velocity_controller active

# Move at 0.05 m/s
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.05]" --once

# Stop
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [0.0]" --once
```

#### Effort Control
```bash
# Switch to effort controller
ros2 control set_controller_state velocity_controller inactive
ros2 control set_controller_state effort_controller active

# Apply 50 N force
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
    "data: [50.0]" --once
```

### Monitor Joint States

```bash
# View current joint state
ros2 topic echo /joint_states

# View controller manager status
ros2 control list_controllers

# View hardware interface status
ros2 control list_hardware_interfaces
```

## Hardware Integration

To integrate with real hardware, modify the following methods in `src/linear_actuator_hardware.cpp`:

1. **`connect_to_hardware()`**: Establish communication (serial, CAN, etc.)
2. **`send_command_to_hardware()`**: Send position/velocity/effort commands
3. **`read_state_from_hardware()`**: Read current position/velocity/effort
4. **`disconnect_from_hardware()`**: Clean up connections

Example for serial communication:
```cpp
bool LinearActuatorHardware::connect_to_hardware()
{
    // Open serial port
    serial_port_ = open(device_port_.c_str(), O_RDWR | O_NOCTTY);
    
    if (serial_port_ < 0) {
        return false;
    }
    
    // Configure serial settings
    struct termios tty;
    tcgetattr(serial_port_, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tcsetattr(serial_port_, TCSANOW, &tty);
    
    is_connected_ = true;
    return true;
}
```

## Configuration Parameters

Edit `urdf/linear_actuator.ros2_control.xacro` to adjust:
- `min_position`: Minimum extension (meters)
- `max_position`: Maximum extension (meters)
- `max_velocity`: Maximum speed (m/s)
- `max_effort`: Maximum force (Newtons)
- `device_port`: Hardware communication port
- `baud_rate`: Communication speed

## Troubleshooting

### Controller Won't Activate
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# View controller manager logs
ros2 run controller_manager controller_manager --ros-args --log-level debug
```

### Hardware Communication Errors
- Verify device port permissions: `sudo chmod 666 /dev/ttyUSB0`
- Check cable connections
- Verify baud rate matches hardware settings
- Review logs: `ros2 launch linear_actuator_hardware linear_actuator.launch.py --log-level debug`

## References

- [ros2_control Documentation](https://control.ros.org/)
- [Writing a Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)
- [Available Controllers](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html)

## License

Apache-2.0
