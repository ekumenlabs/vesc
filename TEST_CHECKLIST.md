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

## Published topics

- [ ] Verify `~/vesc_state` topic publishes `vesc_msgs::msg::VescState` messages
- [ ] Verify `~/vesc_imu` topic publishes `vesc_msgs::msg::VescImuStamped` messages
