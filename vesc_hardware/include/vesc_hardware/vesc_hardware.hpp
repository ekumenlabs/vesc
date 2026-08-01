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

#ifndef VESC_HARDWARE__VESC_HARDWARE_HPP_
#define VESC_HARDWARE__VESC_HARDWARE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "vesc_driver/vesc_interface.hpp"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"

namespace vesc_hardware
{
class VescHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VescHardware)

  /**
   * \brief Initialization of the hardware interface from data parsed from the
   * robot's URDF. \param params Hardware component interface parameters
   * containing executor and HardwareInfo. \return CallbackReturn::SUCCESS if
   * initialization was successful, CallbackReturn::ERROR otherwise.
   *
   * This method is called once during the initialization phase. It should:
   * - Initialize all member variables
   * - Process parameters from the HardwareInfo structure
   * - Validate that all required parameters are present and valid
   */
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  override;

  /**
   * \brief Configuration of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if configuration was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is configured. It should:
   * - Setup communication to the hardware
   * - Prepare everything so that the hardware can be activated
   * - Allocate resources needed for communication
   */
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Activation of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if activation was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is activated. It should:
   * - Enable hardware "power"
   * - Start any background processes needed for operation
   * - Prepare the hardware to accept commands
   */
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Deactivation of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if deactivation was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is deactivated. It
   * should:
   * - Disable hardware "power"
   * - Stop any background processes
   * - Put hardware in a safe state
   */
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Read the current state from the hardware.
   * \param time Current time.
   * \param period Time elapsed since the last read.
   * \return return_type::OK if the read was successful, return_type::ERROR
   * otherwise.
   *
   * This method is called periodically to get the states from the hardware and
   * store them to internal variables that were defined in
   * export_state_interfaces. This method must be real-time safe.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * \brief Write commands to the hardware.
   * \param time Current time.
   * \param period Time elapsed since the last write.
   * \return return_type::OK if the write was successful, return_type::ERROR
   * otherwise.
   *
   * This method is called periodically to command the hardware based on the
   * values stored in internal variables that were defined in
   * export_command_interfaces. This method must be real-time safe.
   */
  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // VESC callback handlers
  void vescPacketCallback(const vesc_driver::VescPacketConstPtr & packet);
  void vescErrorCallback(const std::string & error);

  // VESC packet processing
  void processValuesPacket(const vesc_driver::VescPacketValues *values_packet);
  void processImuPacket(const vesc_driver::VescPacketImu *imu_packet);
  void publishVescState(const vesc_driver::VescPacketValues & values_packet);
  void publishVescImu(const vesc_driver::VescPacketImu & imu_packet);

  // Conversion functions for VESC values to mechanical values (for reading
  // state)
  double convertDegToMechanicalRad(double vesc_position_deg) const;
  double convertERPMtoMechanicalRadSec(double vesc_erpm) const;

  // Conversion functions for mechanical values to VESC values (for writing
  // commands)
  double convertMechanicalRadToDeg(double mechanical_position_rad) const;
  double convertMechanicalRadSecToERPM(double mechanical_velocity_rad_s) const;

  // Interface definition initialization
  void populate_state_definitions();
  void populate_command_definitions();

  // Interface data structures
  struct StateInterfaceData
  {
    bool requested;  // Whether this interface was requested in URDF
    std::function<double()> get_value;  // Functor to retrieve current state value
  };

  struct CommandInterfaceData
  {
    bool requested;  // Whether this interface was requested in URDF
    std::function<void(double)> set_command;  // Functor to send command to hardware
  };

  // Hardware parameters
  std::string device_;
  double gear_ratio_;  // Gear ratio between motor and output
  int pole_pairs_;  // Motor pole pairs
  bool publish_raw_state_;  // Whether to publish raw VESC telemetry

  // State storage (atomic for thread-safe access from callback)
  std::atomic<double> hw_state_position_;
  std::atomic<double> hw_state_velocity_;
  std::atomic<double> hw_avg_id_;
  std::atomic<double> hw_avg_iq_;
  std::atomic<double> hw_avg_vd_;
  std::atomic<double> hw_avg_vq_;
  std::atomic<double> hw_duty_cycle_;

  // IMU state storage (atomic for thread-safe access from callback)
  std::atomic<double> hw_imu_orientation_x_;
  std::atomic<double> hw_imu_orientation_y_;
  std::atomic<double> hw_imu_orientation_z_;
  std::atomic<double> hw_imu_orientation_w_;
  std::atomic<double> hw_imu_roll_;
  std::atomic<double> hw_imu_pitch_;
  std::atomic<double> hw_imu_yaw_;
  std::atomic<double> hw_imu_angular_velocity_x_;
  std::atomic<double> hw_imu_angular_velocity_y_;
  std::atomic<double> hw_imu_angular_velocity_z_;
  std::atomic<double> hw_imu_linear_acceleration_x_;
  std::atomic<double> hw_imu_linear_acceleration_y_;
  std::atomic<double> hw_imu_linear_acceleration_z_;
  std::atomic<double> hw_imu_magnetic_field_x_;
  std::atomic<double> hw_imu_magnetic_field_y_;
  std::atomic<double> hw_imu_magnetic_field_z_;

  // Command storage (servo only - state mirrors command since VESC can't read it)
  double hw_command_servo_;

  // Interface availability tracking
  std::unordered_map<std::string, StateInterfaceData> state_interfaces_;
  std::unordered_map<std::string, CommandInterfaceData> command_interfaces_;

  std::unordered_set<std::string> state_interface_groups_;

  // VESC interface
  std::unique_ptr<vesc_driver::VescInterface> vesc_interface_;

  // Publisher for hardware values
  std::shared_ptr<rclcpp::Publisher<vesc_msgs::msg::VescState>>
  hardware_values_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vesc_msgs::msg::VescState>>
  realtime_hardware_values_publisher_;

  // Publisher for IMU data
  std::shared_ptr<rclcpp::Publisher<vesc_msgs::msg::VescImuStamped>>
  imu_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vesc_msgs::msg::VescImuStamped>>
  realtime_imu_publisher_;
};

}  // namespace vesc_hardware

#endif  // VESC_HARDWARE__VESC_HARDWARE_HPP_
