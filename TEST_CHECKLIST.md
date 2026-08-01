# VESC Hardware Interface Test Checklist

## IMU State Interface Units Verification

### Quaternion Orientation (unitless, normalized)
- [ ] Verify `orientation.x` is normalized quaternion component
- [ ] Verify `orientation.y` is normalized quaternion component
- [ ] Verify `orientation.z` is normalized quaternion component
- [ ] Verify `orientation.w` is normalized quaternion component
- [ ] Verify quaternion magnitude: sqrt(x² + y² + z² + w²) ≈ 1.0

### Euler Angles (radians)
- [ ] Verify `roll` is in radians (compare raw VESC degrees * π/180)
- [ ] Verify `pitch` is in radians (compare raw VESC degrees * π/180)
- [ ] Verify `yaw` is in radians (compare raw VESC degrees * π/180)
- [ ] Verify range: all values between -π and +π

### Angular Velocity (rad/s)
- [ ] Verify `angular_velocity.x` is in rad/s (compare raw VESC deg/s * π/180)
- [ ] Verify `angular_velocity.y` is in rad/s (compare raw VESC deg/s * π/180)
- [ ] Verify `angular_velocity.z` is in rad/s (compare raw VESC deg/s * π/180)

### Linear Acceleration (m/s²)
- [ ] Verify `linear_acceleration.x` is in m/s² (compare raw VESC 'g' * 9.80665)
- [ ] Verify `linear_acceleration.y` is in m/s² (compare raw VESC 'g' * 9.80665)
- [ ] Verify `linear_acceleration.z` is in m/s² (compare raw VESC 'g' * 9.80665)
- [ ] Verify stationary gravity reading: sqrt(x² + y² + z²) ≈ 9.80665 m/s²

### Magnetic Field (raw units)
- [ ] Verify `magnetic_field.x` matches raw VESC magnetometer value
- [ ] Verify `magnetic_field.y` matches raw VESC magnetometer value
- [ ] Verify `magnetic_field.z` matches raw VESC magnetometer value

## Motor Position Interface

### Position Unit Conversion (radians, mechanical)
- [ ] Verify position is in mechanical radians (output shaft)
- [ ] Test conversion formula: `mechanical_rad = (vesc_deg * π/180) / gear_ratio`
- [ ] With gear_ratio=1.0: verify 360° VESC = 2π rad mechanical
- [ ] With gear_ratio=10.0: verify 3600° VESC = 2π rad mechanical
- [ ] Verify position increases with forward motor rotation

### Position and Pole Pairs
- [ ] Verify pole_pairs does NOT affect position reading
- [ ] Position should be independent of pole_pairs parameter

### Position Command Conversion (radians to degrees)
- [ ] Verify command conversion: `vesc_deg = mechanical_rad * gear_ratio * 180/π`
- [ ] Command 2π rad with gear_ratio=1.0 → verify VESC receives 360°
- [ ] Command π rad with gear_ratio=5.0 → verify VESC receives 900°

## Motor Velocity Interface

### Velocity Unit Conversion (rad/s, mechanical)
- [ ] Verify velocity is in mechanical rad/s (output shaft)
- [ ] Test conversion formula: `mechanical_rad_s = (vesc_erpm / pole_pairs) * (2π/60) / gear_ratio`
- [ ] With pole_pairs=1, gear_ratio=1.0: verify 60 ERPM = 2π rad/s
- [ ] With pole_pairs=7, gear_ratio=1.0: verify 420 ERPM = 2π rad/s
- [ ] With pole_pairs=1, gear_ratio=10.0: verify 600 ERPM = 2π rad/s

### Velocity and Gear Ratio
- [ ] Verify higher gear_ratio reduces mechanical velocity (given constant ERPM)
- [ ] With gear_ratio=1.0 vs 10.0: same ERPM → 10x difference in rad/s
- [ ] Verify velocity sign matches motor rotation direction

### Velocity and Pole Pairs
- [ ] Verify higher pole_pairs increases mechanical velocity (given constant ERPM)
- [ ] With pole_pairs=1 vs 7: same ERPM → 7x difference in rad/s
- [ ] ERPM = motor_rpm * pole_pairs (verify this relationship)

### Velocity Command Conversion (rad/s to ERPM)
- [ ] Verify command conversion: `erpm = mechanical_rad_s * (60/2π) * gear_ratio * pole_pairs`
- [ ] Command 2π rad/s with gear_ratio=1.0, pole_pairs=1 → verify VESC receives 60 ERPM
- [ ] Command 1 rad/s with gear_ratio=5.0, pole_pairs=7 → verify VESC receives ~333 ERPM

## Motor Current State Interfaces (D/Q Components)

### Average D-axis Current (Amperes)
- [ ] Verify `average_id` is in Amperes
- [ ] Verify value matches VESC packet `avg_id()`
- [ ] Check sign: positive/negative indicates direction in D-axis
- [ ] Verify updates in real-time during motor operation

### Average Q-axis Current (Amperes)
- [ ] Verify `average_iq` is in Amperes
- [ ] Verify value matches VESC packet `avg_iq()`
- [ ] Check sign: positive/negative indicates direction in Q-axis
- [ ] Verify updates in real-time during motor operation
- [ ] Iq typically correlates with torque production

## Motor Voltage State Interfaces (D/Q Components)

### Average D-axis Voltage (Volts)
- [ ] Verify `average_vd` is in Volts
- [ ] Verify value matches VESC packet `avg_vd()`
- [ ] Check magnitude relative to battery voltage
- [ ] Verify updates in real-time during motor operation

### Average Q-axis Voltage (Volts)
- [ ] Verify `average_vq` is in Volts
- [ ] Verify value matches VESC packet `avg_vq()`
- [ ] Check magnitude relative to battery voltage
- [ ] Verify updates in real-time during motor operation

## Duty Cycle State Interface

### Duty Cycle (unitless, range [0, 1])
- [ ] Verify `duty_cycle` is unitless (0.0 to 1.0)
- [ ] Verify value matches VESC packet `duty_cycle_now()`
- [ ] At rest: duty cycle should be near 0.0
- [ ] At full throttle: duty cycle should approach 1.0
- [ ] Verify updates in real-time during motor operation

## Servo State Interface

### Servo Position (unitless)
- [ ] Verify `servo` mirrors the commanded servo value
- [ ] VESC cannot read back servo position, so state = last command
- [ ] Verify value persists between write cycles

## Command Interfaces

### Position Command (radians, mechanical)
- [ ] Command position in mechanical radians
- [ ] Verify conversion to VESC degrees: `vesc_deg = mechanical_rad * gear_ratio * 180/π`
- [ ] Test with various gear_ratio values
- [ ] Verify motor moves to commanded position

### Velocity Command (rad/s, mechanical)
- [ ] Command velocity in mechanical rad/s
- [ ] Verify conversion to VESC ERPM: `erpm = mechanical_rad_s * (60/2π) * gear_ratio * pole_pairs`
- [ ] Test with various gear_ratio and pole_pairs values
- [ ] Verify motor rotates at commanded speed

### Servo Command (unitless)
- [ ] Command servo position (typically -1.0 to +1.0 or 0.0 to 1.0)
- [ ] Verify value is sent directly to VESC `setServo()`
- [ ] Verify state interface mirrors commanded value

### Current/Effort Command (Amperes)
- [ ] Command current in Amperes using `effort` interface
- [ ] Verify value is sent to VESC `setCurrent()`
- [ ] Test positive values (forward torque)
- [ ] Test negative values (reverse torque)
- [ ] Verify motor torque correlates with commanded current

### Duty Cycle Command (unitless, range [0, 1])
- [ ] Command duty cycle between 0.0 and 1.0
- [ ] Verify values are clamped to [0, 1] range
- [ ] Test command < 0.0 → verify clamped to 0.0
- [ ] Test command > 1.0 → verify clamped to 1.0
- [ ] Test command = 0.5 → verify exactly 0.5 sent to VESC
- [ ] Verify motor speed correlates with duty cycle

### Brake Command (Amperes)
- [ ] Command brake current in Amperes
- [ ] Verify value is sent to VESC `setBrake()`
- [ ] Test various brake current values
- [ ] Verify motor decelerates/stops when brake is applied
- [ ] Brake current should be positive (magnitude of braking force)

## Published Topics

- [ ] Verify `~/vesc_state` topic publishes `vesc_msgs::msg::VescState` messages
- [ ] Verify `~/vesc_imu` topic publishes `vesc_msgs::msg::VescImuStamped` messages
