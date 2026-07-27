# vesc_hardware

ROS 2 hardware interface component for VESC (Vedder Electronic Speed Controller) motor controllers, compatible with the `ros2_control` framework.

## Overview

This package provides a hardware interface plugin that allows VESC motor controllers to be used seamlessly with `ros2_control`. It handles communication with the VESC, converts between VESC units (degrees, ERPM) and standard ROS units (radians, radians/second), and supports both position and velocity control modes.

## Hardware Parameters

The following parameters must be configured in your robot's URDF/XACRO file within the `<hardware>` tag:

### Required Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `device` | string | Serial port device path for VESC communication (e.g., `/dev/ttyACM0`, `/dev/ttyUSB0`) |

### Optional Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `gear_ratio` | double | `1.0` | Gear ratio between motor and output shaft. For a reduction gearbox with ratio N:1, set this to N. Used to convert between motor position/velocity and mechanical output. |
| `pole_pairs` | int | `1` | Number of motor pole pairs. Used to convert between ERPM (Electrical RPM) and mechanical RPM. For a motor with P poles, pole_pairs = P/2. |

## Supported Interfaces

The hardware component supports a single joint with the following interfaces:

### State Interfaces

- `position` - Joint position in radians (mechanical output)
- `velocity` - Joint velocity in radians/second (mechanical output)

### Command Interfaces

- `position` - Position command in radians (mechanical output)
- `velocity` - Velocity command in radians/second (mechanical output)

**Note:** At least one state interface and one command interface must be specified in the URDF.

## URDF Configuration Example

### Basic Configuration (Direct Drive Motor)

```xml
<ros2_control name="vesc_system" type="system">
  <hardware>
    <plugin>vesc_hardware/VescHardware</plugin>
    <param name="device">/dev/ttyACM0</param>
  </hardware>

  <joint name="wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Advanced Configuration (Geared Motor)

```xml
<ros2_control name="vesc_system" type="system">
  <hardware>
    <plugin>vesc_hardware/VescHardware</plugin>
    <param name="device">/dev/ttyACM0</param>
    <param name="gear_ratio">14.0</param>  <!-- 14:1 reduction gearbox -->
    <param name="pole_pairs">7</param>     <!-- 14-pole motor -->
  </hardware>

  <joint name="wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Position Control Example

```xml
<ros2_control name="vesc_system" type="system">
  <hardware>
    <plugin>vesc_hardware/VescHardware</plugin>
    <param name="device">/dev/ttyACM0</param>
    <param name="gear_ratio">50.0</param>  <!-- High reduction for position control -->
    <param name="pole_pairs">7</param>
  </hardware>

  <joint name="actuator_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## License

Copyright 2024 Ekumen, Inc.

This software is licensed under the BSD 3-Clause License. See the LICENSE file for details.
