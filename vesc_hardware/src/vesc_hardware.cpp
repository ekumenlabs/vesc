// Copyright 2024 Ekumen, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Ekumen, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "vesc_hardware/vesc_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace
{

// Custom control interfaces
constexpr char CUSTOM_HW_IF_DUTY_CYCLE[] = "duty_cycle";
constexpr char CUSTOM_HW_IF_SERVO[] = "servo";
constexpr char CUSTOM_HW_IF_BRAKE[] = "brake";

// Motor current and voltage interface names
constexpr char CUSTOM_HW_IF_AVG_ID[] = "average_id";
constexpr char CUSTOM_HW_IF_AVG_IQ[] = "average_iq";
constexpr char CUSTOM_HW_IF_AVG_VD[] = "average_vd";
constexpr char CUSTOM_HW_IF_AVG_VQ[] = "average_vq";

// IMU sensor interface names - Quaternion orientation
constexpr char CUSTOM_HW_IF_ORIENTATION_X[] = "orientation.x";
constexpr char CUSTOM_HW_IF_ORIENTATION_Y[] = "orientation.y";
constexpr char CUSTOM_HW_IF_ORIENTATION_Z[] = "orientation.z";
constexpr char CUSTOM_HW_IF_ORIENTATION_W[] = "orientation.w";

// IMU sensor interface names - Euler angles
constexpr char CUSTOM_HW_IF_ROLL[] = "euler_angles.roll";
constexpr char CUSTOM_HW_IF_PITCH[] = "euler_angles.pitch";
constexpr char CUSTOM_HW_IF_YAW[] = "euler_angles.yaw";

// IMU sensor interface names - Angular velocity
constexpr char CUSTOM_HW_IF_ANGULAR_VELOCITY_X[] = "angular_velocity.x";
constexpr char CUSTOM_HW_IF_ANGULAR_VELOCITY_Y[] = "angular_velocity.y";
constexpr char CUSTOM_HW_IF_ANGULAR_VELOCITY_Z[] = "angular_velocity.z";

// IMU sensor interface names - Linear acceleration
constexpr char CUSTOM_HW_IF_LINEAR_ACCELERATION_X[] = "linear_acceleration.x";
constexpr char CUSTOM_HW_IF_LINEAR_ACCELERATION_Y[] = "linear_acceleration.y";
constexpr char CUSTOM_HW_IF_LINEAR_ACCELERATION_Z[] = "linear_acceleration.z";

// IMU sensor interface names - Magnetic field
constexpr char CUSTOM_HW_IF_MAGNETIC_FIELD_X[] = "magnetic_field.x";
constexpr char CUSTOM_HW_IF_MAGNETIC_FIELD_Y[] = "magnetic_field.y";
constexpr char CUSTOM_HW_IF_MAGNETIC_FIELD_Z[] = "magnetic_field.z";
}  // namespace

namespace vesc_hardware
{
hardware_interface::CallbackReturn VescHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters
  auto it_device = info_.hardware_parameters.find("device");
  if (it_device == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(get_logger(),
                 "Parameter 'device' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  device_ = it_device->second;

  // Read optional gear ratio parameter (default: 1.0)
  auto it_gear_ratio = info_.hardware_parameters.find("gear_ratio");
  if (it_gear_ratio != info_.hardware_parameters.end()) {
    gear_ratio_ = std::stod(it_gear_ratio->second);
  } else {
    gear_ratio_ = 1.0;
  }

  // Read optional pole pairs parameter (default: 1)
  auto it_pole_pairs = info_.hardware_parameters.find("pole_pairs");
  if (it_pole_pairs != info_.hardware_parameters.end()) {
    pole_pairs_ = std::stoi(it_pole_pairs->second);
  } else {
    pole_pairs_ = 1;
  }

  // Read optional publish_raw_state parameter (default: false)
  auto it_publish_raw_state = info_.hardware_parameters.find("publish_raw_state");
  if (it_publish_raw_state != info_.hardware_parameters.end()) {
    publish_raw_state_ = (it_publish_raw_state->second == "true" ||
      it_publish_raw_state->second == "1");
  } else {
    publish_raw_state_ = false;
  }

  RCLCPP_INFO(get_logger(), "Configured device: %s", device_.c_str());
  RCLCPP_INFO(get_logger(), "Gear ratio: %.2f, Pole pairs: %d", gear_ratio_,
              pole_pairs_);
  RCLCPP_INFO(get_logger(), "Publish raw state: %s", publish_raw_state_ ? "true" : "false");

  // Validate interfaces
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(), "Expected exactly 1 joint, found %zu",
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];

  // Initialize list of supported interfaces
  populate_state_definitions();
  populate_command_definitions();

  // Check which state interfaces are requested
  for (const auto & state_interface : joint.state_interfaces) {
    auto it = state_interfaces_.find(state_interface.name);
    if (it == state_interfaces_.end()) {
      RCLCPP_FATAL(get_logger(),
                   "Unsupported state interface '%s' requested for joint '%s'",
                   state_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (it->second.requested) {
      RCLCPP_FATAL(
          get_logger(),
          "Duplicate state interface '%s' requested for joint '%s'",
          state_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    it->second.requested = true;
    RCLCPP_INFO(get_logger(), "State interface '%s' requested",
                state_interface.name.c_str());
  }

  // Check which command interfaces are requested
  for (const auto & command_interface : joint.command_interfaces) {
    auto it = command_interfaces_.find(command_interface.name);
    if (it == command_interfaces_.end()) {
      RCLCPP_FATAL(
          get_logger(),
          "Unsupported command interface '%s' requested for joint '%s'",
          command_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (it->second.requested) {
      RCLCPP_FATAL(
          get_logger(),
          "Duplicate command interface '%s' requested for joint '%s'",
          command_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    it->second.requested = true;
    RCLCPP_INFO(get_logger(), "Command interface '%s' requested",
                command_interface.name.c_str());
  }

  // Check that at least one interface is requested
  bool has_state_interface = false;
  for (const auto & [name, data] : state_interfaces_) {
    if (data.requested) {
      has_state_interface = true;
      break;
    }
  }

  bool has_command_interface = false;
  for (const auto & [name, data] : command_interfaces_) {
    if (data.requested) {
      has_command_interface = true;
      break;
    }
  }

  if (!has_state_interface) {
    RCLCPP_WARN(get_logger(), "No state interfaces requested for joint '%s'",
                joint.name.c_str());
  }

  if (!has_command_interface) {
    RCLCPP_WARN(get_logger(), "No command interfaces requested for joint '%s'",
                joint.name.c_str());
  }

  // Initialize state storage
  hw_state_position_ = 0.0;
  hw_state_velocity_ = 0.0;
  hw_avg_id_ = 0.0;
  hw_avg_iq_ = 0.0;
  hw_avg_vd_ = 0.0;
  hw_avg_vq_ = 0.0;
  hw_duty_cycle_ = 0.0;

  // Initialize command storage
  hw_command_servo_ = 0.0;

  // Create publisher for hardware values (only if enabled)
  if (publish_raw_state_) {
    hardware_values_publisher_ =
      get_node()->create_publisher<vesc_msgs::msg::VescState>(
        "~/vesc_state", 10);

    // Create realtime publisher wrapper
    realtime_hardware_values_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<
          vesc_msgs::msg::VescState>>(
          hardware_values_publisher_);

    // Create publisher for IMU data
    imu_publisher_ =
      get_node()->create_publisher<vesc_msgs::msg::VescImuStamped>(
        "~/vesc_imu", 10);

    // Create realtime publisher wrapper
    realtime_imu_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<
          vesc_msgs::msg::VescImuStamped>>(
          imu_publisher_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
VescHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring VESC hardware interface...");

  // Log which interfaces are active
  RCLCPP_INFO(get_logger(), "Active state interfaces:");
  for (const auto & [name, data] : state_interfaces_) {
    if (data.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Active command interfaces:");
  for (const auto & [name, data] : command_interfaces_) {
    if (data.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
  }

  // Reset values when configuring hardware (only for requested interfaces)
  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto &[name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
VescHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating VESC hardware interface...");

  // Create VESC interface and setup callbacks
  try {
    vesc_interface_ = std::make_unique<vesc_driver::VescInterface>();

    // Setup packet and error handlers
    vesc_interface_->setPacketHandler(std::bind(
        &VescHardware::vescPacketCallback, this, std::placeholders::_1));
    vesc_interface_->setErrorHandler(std::bind(&VescHardware::vescErrorCallback,
                                               this, std::placeholders::_1));

    // Connect to VESC
    vesc_interface_->connect(device_);
    RCLCPP_INFO(get_logger(), "Connected to VESC on %s", device_.c_str());

    // Request firmware version
    vesc_interface_->requestFWVersion();
  } catch (const vesc_driver::SerialException & e) {
    RCLCPP_FATAL(get_logger(), "Failed to connect to VESC: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Synchronize command with current state
  for (const auto &[name, descr] : joint_state_interfaces_) {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating VESC hardware interface...");

  // Disconnect and destroy VESC interface
  if (vesc_interface_) {
    vesc_interface_->disconnect();
    vesc_interface_.reset();
    RCLCPP_INFO(get_logger(), "Disconnected from VESC");
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
VescHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Request state and IMU data from VESC (non-blocking)
  if (vesc_interface_) {
    vesc_interface_->requestState();
    vesc_interface_->requestImuData();
  }

  // Update state interfaces with current values from atomic variables
  // (These are updated asynchronously by vescPacketCallback)
  for (const auto &[name, descr] : joint_state_interfaces_) {
    std::string interface_type = name.substr(name.find_last_of("/") + 1);
    auto it = state_interfaces_.find(interface_type);
    if (it != state_interfaces_.end() && it->second.get_value) {
      set_state(name, it->second.get_value());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
VescHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!vesc_interface_) {
    return hardware_interface::return_type::ERROR;
  }

  // Iterate through claimed interfaces
  // Only claimed/active interfaces appear in joint_command_interfaces_
  for (const auto &[name, descr] : joint_command_interfaces_) {
    // Extract interface type from full name "motor_joint/velocity"
    std::string interface_type = name.substr(name.find_last_of("/") + 1);
    auto it = command_interfaces_.find(interface_type);
    if (it != command_interfaces_.end() && it->second.set_command) {
      it->second.set_command(get_command(name));
    }
  }

  return hardware_interface::return_type::OK;
}

double VescHardware::convertDegToMechanicalRad(double vesc_position_deg) const
{
  const double position_rad = vesc_position_deg * M_PI / 180.0;
  return position_rad / gear_ratio_;
}

double
VescHardware::convertMechanicalRadToDeg(double mechanical_position_rad) const
{
  const double motor_position_rad = mechanical_position_rad * gear_ratio_;
  return motor_position_rad * 180.0 / M_PI;
}

double VescHardware::convertERPMtoMechanicalRadSec(double vesc_erpm) const
{
  const double motor_rpm = vesc_erpm / static_cast<double>(pole_pairs_);
  const double motor_rad_s = motor_rpm * (2.0 * M_PI / 60.0);
  return motor_rad_s / gear_ratio_;
}

double VescHardware::convertMechanicalRadSecToERPM(
  double mechanical_velocity_rad_s) const
{
  const double output_rpm = mechanical_velocity_rad_s * 60.0 / (2.0 * M_PI);
  const double motor_rpm = output_rpm * gear_ratio_;
  const double erpm = motor_rpm * static_cast<double>(pole_pairs_);
  return erpm;
}

void VescHardware::populate_state_definitions()
{
  state_interfaces_[hardware_interface::HW_IF_POSITION] = {
    false,
    [this]() {return hw_state_position_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[hardware_interface::HW_IF_VELOCITY] = {
    false,
    [this]() {return hw_state_velocity_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_SERVO] = {
    false,
    [this]() {return hw_command_servo_;}
  };

  // Motor current state interfaces
  state_interfaces_[CUSTOM_HW_IF_AVG_ID] = {
    false,
    [this]() {return hw_avg_id_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_AVG_IQ] = {
    false,
    [this]() {return hw_avg_iq_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_AVG_VD] = {
    false,
    [this]() {return hw_avg_vd_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_AVG_VQ] = {
    false,
    [this]() {return hw_avg_vq_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_DUTY_CYCLE] = {
    false,
    [this]() {return hw_duty_cycle_.load(std::memory_order_relaxed);}
  };

  // IMU state interfaces - Quaternion orientation
  state_interfaces_[CUSTOM_HW_IF_ORIENTATION_X] = {
    false,
    [this]() {return hw_imu_orientation_x_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_ORIENTATION_Y] = {
    false,
    [this]() {return hw_imu_orientation_y_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_ORIENTATION_Z] = {
    false,
    [this]() {return hw_imu_orientation_z_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_ORIENTATION_W] = {
    false,
    [this]() {return hw_imu_orientation_w_.load(std::memory_order_relaxed);}
  };

  // IMU state interfaces - Euler angles
  state_interfaces_[CUSTOM_HW_IF_ROLL] = {
    false,
    [this]() {return hw_imu_roll_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_PITCH] = {
    false,
    [this]() {return hw_imu_pitch_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_YAW] = {
    false,
    [this]() {return hw_imu_yaw_.load(std::memory_order_relaxed);}
  };

  // IMU state interfaces - Angular velocity
  state_interfaces_[CUSTOM_HW_IF_ANGULAR_VELOCITY_X] = {
    false,
    [this]() {return hw_imu_angular_velocity_x_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_ANGULAR_VELOCITY_Y] = {
    false,
    [this]() {return hw_imu_angular_velocity_y_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_ANGULAR_VELOCITY_Z] = {
    false,
    [this]() {return hw_imu_angular_velocity_z_.load(std::memory_order_relaxed);}
  };

  // IMU state interfaces - Linear acceleration
  state_interfaces_[CUSTOM_HW_IF_LINEAR_ACCELERATION_X] = {
    false,
    [this]() {return hw_imu_linear_acceleration_x_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_LINEAR_ACCELERATION_Y] = {
    false,
    [this]() {return hw_imu_linear_acceleration_y_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_LINEAR_ACCELERATION_Z] = {
    false,
    [this]() {return hw_imu_linear_acceleration_z_.load(std::memory_order_relaxed);}
  };

  // IMU state interfaces - Magnetic field
  state_interfaces_[CUSTOM_HW_IF_MAGNETIC_FIELD_X] = {
    false,
    [this]() {return hw_imu_magnetic_field_x_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_MAGNETIC_FIELD_Y] = {
    false,
    [this]() {return hw_imu_magnetic_field_y_.load(std::memory_order_relaxed);}
  };
  state_interfaces_[CUSTOM_HW_IF_MAGNETIC_FIELD_Z] = {
    false,
    [this]() {return hw_imu_magnetic_field_z_.load(std::memory_order_relaxed);}
  };
}

void VescHardware::populate_command_definitions()
{
  command_interfaces_[hardware_interface::HW_IF_POSITION] = {
    false,
    [this](double value) {
      double vesc_position = convertMechanicalRadToDeg(value);
      vesc_interface_->setPosition(vesc_position);
    }
  };
  command_interfaces_[hardware_interface::HW_IF_VELOCITY] = {
    false,
    [this](double value) {
      double vesc_erpm = convertMechanicalRadSecToERPM(value);
      vesc_interface_->setSpeed(vesc_erpm);
    }
  };
  command_interfaces_[CUSTOM_HW_IF_SERVO] = {
    false,
    [this](double value) {
      hw_command_servo_ = value;
      vesc_interface_->setServo(hw_command_servo_);
    }
  };
  command_interfaces_[CUSTOM_HW_IF_DUTY_CYCLE] = {
    false,
    [this](double value) {
      // Clamp duty cycle to [0, 1] range
      double clamped_duty_cycle = std::clamp(value, 0.0, 1.0);
      vesc_interface_->setDutyCycle(clamped_duty_cycle);
    }
  };
  command_interfaces_[hardware_interface::HW_IF_CURRENT] = {
    false,
    [this](double value) {
      vesc_interface_->setCurrent(value);
    }
  };
  command_interfaces_[CUSTOM_HW_IF_BRAKE] = {
    false,
    [this](double value) {
      vesc_interface_->setBrake(value);
    }
  };
}

void VescHardware::processValuesPacket(
  const vesc_driver::VescPacketValues *values_packet)
{
  if (!values_packet) {
    return;
  }

  double mechanical_position =
    convertDegToMechanicalRad(values_packet->pid_pos_now());
  hw_state_position_.store(mechanical_position, std::memory_order_relaxed);

  double mechanical_velocity =
    convertERPMtoMechanicalRadSec(values_packet->rpm());
  hw_state_velocity_.store(mechanical_velocity, std::memory_order_relaxed);

  // Store motor current values
  hw_avg_id_.store(values_packet->avg_id(), std::memory_order_relaxed);
  hw_avg_iq_.store(values_packet->avg_iq(), std::memory_order_relaxed);

  // Store motor voltage values
  hw_avg_vd_.store(values_packet->avg_vd(), std::memory_order_relaxed);
  hw_avg_vq_.store(values_packet->avg_vq(), std::memory_order_relaxed);

  // Store duty cycle
  hw_duty_cycle_.store(values_packet->duty_cycle_now(), std::memory_order_relaxed);

  // Publish telemetry data
  publishVescState(*values_packet);
}

void VescHardware::processImuPacket(
  const vesc_driver::VescPacketImu *imu_packet)
{
  if (!imu_packet) {
    return;
  }

  // Conversion lambda: degrees to radians
  auto deg_to_rad = [](double deg) {return deg * M_PI / 180.0;};

  // Standard gravity constant for converting acceleration from 'g' to m/s²
  constexpr double STANDARD_GRAVITY = 9.80665;

  // Store quaternion orientation (already in correct units)
  hw_imu_orientation_x_.store(imu_packet->q_x(), std::memory_order_relaxed);
  hw_imu_orientation_y_.store(imu_packet->q_y(), std::memory_order_relaxed);
  hw_imu_orientation_z_.store(imu_packet->q_z(), std::memory_order_relaxed);
  hw_imu_orientation_w_.store(imu_packet->q_w(), std::memory_order_relaxed);

  // Store Euler angles (convert from degrees to radians)
  hw_imu_roll_.store(deg_to_rad(imu_packet->roll()), std::memory_order_relaxed);
  hw_imu_pitch_.store(deg_to_rad(imu_packet->pitch()), std::memory_order_relaxed);
  hw_imu_yaw_.store(deg_to_rad(imu_packet->yaw()), std::memory_order_relaxed);

  // Store angular velocity (convert from degrees/second to radians/second)
  hw_imu_angular_velocity_x_.store(deg_to_rad(imu_packet->gyr_x()), std::memory_order_relaxed);
  hw_imu_angular_velocity_y_.store(deg_to_rad(imu_packet->gyr_y()), std::memory_order_relaxed);
  hw_imu_angular_velocity_z_.store(deg_to_rad(imu_packet->gyr_z()), std::memory_order_relaxed);

  // Store linear acceleration (convert from 'g' to m/s²)
  hw_imu_linear_acceleration_x_.store(imu_packet->acc_x() * STANDARD_GRAVITY,
      std::memory_order_relaxed);
  hw_imu_linear_acceleration_y_.store(imu_packet->acc_y() * STANDARD_GRAVITY,
      std::memory_order_relaxed);
  hw_imu_linear_acceleration_z_.store(imu_packet->acc_z() * STANDARD_GRAVITY,
      std::memory_order_relaxed);

  // Store magnetic field (in raw units as received)
  hw_imu_magnetic_field_x_.store(imu_packet->mag_x(), std::memory_order_relaxed);
  hw_imu_magnetic_field_y_.store(imu_packet->mag_y(), std::memory_order_relaxed);
  hw_imu_magnetic_field_z_.store(imu_packet->mag_z(), std::memory_order_relaxed);

  // Publish raw IMU data
  publishVescImu(*imu_packet);
}

void VescHardware::publishVescImu(
  const vesc_driver::VescPacketImu & imu_packet)
{
  if (!realtime_imu_publisher_) {
    return;
  }

  if (realtime_imu_publisher_->trylock()) {
    auto & msg = realtime_imu_publisher_->msg_;

    // Set timestamp
    msg.header.stamp = get_node()->now();
    msg.header.frame_id = "";

    // Yaw, Pitch, Roll (in degrees as received)
    msg.imu.ypr.x = imu_packet.yaw();
    msg.imu.ypr.y = imu_packet.pitch();
    msg.imu.ypr.z = imu_packet.roll();

    // Linear acceleration (in m/s²)
    msg.imu.linear_acceleration.x = imu_packet.acc_x();
    msg.imu.linear_acceleration.y = imu_packet.acc_y();
    msg.imu.linear_acceleration.z = imu_packet.acc_z();

    // Angular velocity (in degrees/second as received)
    msg.imu.angular_velocity.x = imu_packet.gyr_x();
    msg.imu.angular_velocity.y = imu_packet.gyr_y();
    msg.imu.angular_velocity.z = imu_packet.gyr_z();

    // Compass (in raw units as received)
    msg.imu.compass.x = imu_packet.mag_x();
    msg.imu.compass.y = imu_packet.mag_y();
    msg.imu.compass.z = imu_packet.mag_z();

    // Orientation quaternion
    msg.imu.orientation.w = imu_packet.q_w();
    msg.imu.orientation.x = imu_packet.q_x();
    msg.imu.orientation.y = imu_packet.q_y();
    msg.imu.orientation.z = imu_packet.q_z();

    realtime_imu_publisher_->unlockAndPublish();
  }
}

void VescHardware::publishVescState(
  const vesc_driver::VescPacketValues & values_packet)
{
  if (!realtime_hardware_values_publisher_) {
    return;
  }

  if (realtime_hardware_values_publisher_->trylock()) {
    auto & msg = realtime_hardware_values_publisher_->msg_;

    // Temperature measurements
    msg.temp_fet = values_packet.temp_fet();
    msg.temp_motor = values_packet.temp_motor();
    msg.ntc_temp_mos1 = values_packet.temp_mos1();
    msg.ntc_temp_mos2 = values_packet.temp_mos2();
    msg.ntc_temp_mos3 = values_packet.temp_mos3();

    // Current measurements
    msg.current_motor = values_packet.avg_motor_current();
    msg.current_input = values_packet.avg_input_current();
    msg.avg_id = values_packet.avg_id();
    msg.avg_iq = values_packet.avg_iq();

    // Voltage measurements
    msg.voltage_input = values_packet.v_in();
    msg.avg_vd = values_packet.avg_vd();
    msg.avg_vq = values_packet.avg_vq();

    // Duty cycle and speed
    msg.duty_cycle = values_packet.duty_cycle_now();
    msg.speed = values_packet.rpm();

    // Energy and charge tracking
    msg.charge_drawn = values_packet.amp_hours();
    msg.charge_regen = values_packet.amp_hours_charged();
    msg.energy_drawn = values_packet.watt_hours();
    msg.energy_regen = values_packet.watt_hours_charged();

    // Position and distance tracking
    msg.displacement = values_packet.tachometer();
    msg.distance_traveled = values_packet.tachometer_abs();
    msg.pid_pos_now = values_packet.pid_pos_now();

    // Status
    msg.fault_code = values_packet.fault_code();
    msg.controller_id = values_packet.controller_id();

    realtime_hardware_values_publisher_->unlockAndPublish();
  }
}

void VescHardware::vescPacketCallback(
  const vesc_driver::VescPacketConstPtr & packet)
{
  if (!packet) {
    RCLCPP_WARN(rclcpp::get_logger("VescHardware"), "vescPacketCallback called, but "
                                                       "no packet received");
    return;
  }

  if (packet->name() == "Values") {
    const auto *values_packet =
      dynamic_cast<const vesc_driver::VescPacketValues *>(packet.get());
    processValuesPacket(values_packet);
  } else if (packet->name() == "FWVersion") {
    const auto *fw_packet =
      dynamic_cast<const vesc_driver::VescPacketFWVersion *>(packet.get());
    if (fw_packet) {
      RCLCPP_INFO(
        get_logger(),
        "VESC Firmware Version: %d.%d, Hardware: %s, Paired: %s",
        fw_packet->fwMajor(),
        fw_packet->fwMinor(),
        fw_packet->hwname().c_str(),
        fw_packet->paired() ? "yes" : "no"
      );
    }
  } else if (packet->name() == "ImuData") {
    const auto *imu_packet =
      dynamic_cast<const vesc_driver::VescPacketImu *>(packet.get());
    processImuPacket(imu_packet);
  }
}

void VescHardware::vescErrorCallback(const std::string & error)
{
  RCLCPP_ERROR(get_logger(), "VESC error: %s", error.c_str());
}

}  // namespace vesc_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vesc_hardware::VescHardware,
                       hardware_interface::SystemInterface)
