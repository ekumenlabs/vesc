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

  RCLCPP_INFO(get_logger(), "Configured device: %s", device_.c_str());
  RCLCPP_INFO(get_logger(), "Gear ratio: %.2f, Pole pairs: %d", gear_ratio_,
              pole_pairs_);

  // Validate interfaces
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(), "Expected exactly 1 joint, found %zu",
                 info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];

  // Initialize list of supported interfaces
  state_interfaces_ = {{hardware_interface::HW_IF_POSITION, "position", false},
    {hardware_interface::HW_IF_VELOCITY, "velocity", false},
    {"servo", "servo", false}};

  command_interfaces_ = {
    {hardware_interface::HW_IF_POSITION, "position", false},
    {hardware_interface::HW_IF_VELOCITY, "velocity", false},
    {"servo", "servo", false}};

  // Check which state interfaces are requested
  for (const auto & state_interface : joint.state_interfaces) {
    bool found = false;
    for (auto & supported_interface : state_interfaces_) {
      if (state_interface.name == supported_interface.name) {
        if (supported_interface.requested) {
          RCLCPP_FATAL(
              get_logger(),
              "Duplicate state interface '%s' requested for joint '%s'",
              state_interface.name.c_str(), joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        supported_interface.requested = true;
        found = true;
        RCLCPP_INFO(get_logger(), "State interface '%s' requested",
                    state_interface.name.c_str());
        break;
      }
    }
    if (!found) {
      RCLCPP_FATAL(get_logger(),
                   "Unsupported state interface '%s' requested for joint '%s'",
                   state_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Check which command interfaces are requested
  for (const auto & command_interface : joint.command_interfaces) {
    bool found = false;
    for (auto & supported_interface : command_interfaces_) {
      if (command_interface.name == supported_interface.name) {
        if (supported_interface.requested) {
          RCLCPP_FATAL(
              get_logger(),
              "Duplicate command interface '%s' requested for joint '%s'",
              command_interface.name.c_str(), joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
        supported_interface.requested = true;
        found = true;
        RCLCPP_INFO(get_logger(), "Command interface '%s' requested",
                    command_interface.name.c_str());
        break;
      }
    }
    if (!found) {
      RCLCPP_FATAL(
          get_logger(),
          "Unsupported command interface '%s' requested for joint '%s'",
          command_interface.name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Check that at least one interface is requested
  bool has_state_interface = false;
  for (const auto & iface : state_interfaces_) {
    if (iface.requested) {
      has_state_interface = true;
      break;
    }
  }

  bool has_command_interface = false;
  for (const auto & iface : command_interfaces_) {
    if (iface.requested) {
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

  // Initialize state and command storage
  hw_state_position_ = 0.0;
  hw_state_velocity_ = 0.0;
  hw_command_position_ = 0.0;
  hw_command_velocity_ = 0.0;
  hw_command_servo_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
VescHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring VESC hardware interface...");

  // Log which interfaces are active
  RCLCPP_INFO(get_logger(), "Active state interfaces:");
  for (const auto & iface : state_interfaces_) {
    if (iface.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", iface.type.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Active command interfaces:");
  for (const auto & iface : command_interfaces_) {
    if (iface.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", iface.type.c_str());
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
  // Request state from VESC (non-blocking)
  if (vesc_interface_) {
    vesc_interface_->requestState();
  }

  // Update state interfaces with current values from atomic variables
  // (These are updated asynchronously by vescPacketCallback)
  for (const auto &[name, descr] : joint_state_interfaces_) {
    std::string interface_type = name.substr(name.find_last_of("/") + 1);

    if (interface_type == hardware_interface::HW_IF_POSITION) {
      set_state(name, hw_state_position_.load(std::memory_order_relaxed));
    } else if (interface_type == hardware_interface::HW_IF_VELOCITY) {
      set_state(name, hw_state_velocity_.load(std::memory_order_relaxed));
    } else if (interface_type == "servo") {
      // Servo state mirrors command because the VESC cannot read servo position
      set_state(name, hw_command_servo_);
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

    if (interface_type == hardware_interface::HW_IF_POSITION) {
      hw_command_position_ = get_command(name);
      double vesc_position = convertMechanicalRadToDeg(hw_command_position_);
      vesc_interface_->setPosition(vesc_position);
    } else if (interface_type == hardware_interface::HW_IF_VELOCITY) {
      hw_command_velocity_ = get_command(name);
      double vesc_erpm = convertMechanicalRadSecToERPM(hw_command_velocity_);
      vesc_interface_->setSpeed(vesc_erpm);
    } else if (interface_type == "servo") {
      hw_command_servo_ = get_command(name);
      vesc_interface_->setServo(hw_command_servo_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Unknown command interface type: %s",
                           interface_type.c_str());
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
