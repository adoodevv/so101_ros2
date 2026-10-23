// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SO101_HARDWARE__SO101_SYSTEM_HPP_
#define SO101_HARDWARE__SO101_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/serial_port.hpp"

namespace so101_hardware
{

// ros2_control system for the SO-101 follower arm: six Feetech STS3215
// servos on one serial bus, calibrated with LeRobot.
//
// Hardware parameters (<hardware><param>):
//   port              serial device, e.g. /dev/ttyACM0 (required)
//   calibration_file  LeRobot calibration JSON (required)
//   baud_rate         default 1000000
//   reply_timeout_ms  per-servo reply timeout, default 10
//   max_consecutive_read_errors  default 5; read() returns ERROR after that
//   configure_servos  apply LeRobot's servo settings on configure, default true
//   disable_torque_on_deactivate  default true (the arm then goes limp)
//   read_only         never enable torque; feedback only, the arm stays limp.
//                     For checking directions and calibration by hand.
//   p_coefficient, i_coefficient, d_coefficient  default 16, 0, 32 (LeRobot)
//
// Joint parameters (<joint><param>):
//   motor             key in the calibration file, default: the joint name
//   mapping           "centered" (default) or "linear"; linear maps the
//                     calibrated range onto the command interface min/max
//   max_torque_limit, protection_current, overload_torque  optional servo
//                     protection settings (LeRobot sets them on the gripper)
//
// Interfaces: command "position"; state "position" (required) and optionally
// "velocity", "load" (fraction of max torque, -1..1), "temperature" (deg C),
// "voltage" (V).
class SO101SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SO101SystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    std::string motor;
    uint8_t id = 0;
    MotorCalibration calibration;
    std::unique_ptr<JointMapping> mapping;
    bool has_velocity = false;
    bool has_load = false;
    bool has_temperature = false;
    bool has_voltage = false;
    int max_torque_limit = -1;
    int protection_current = -1;
    int overload_torque = -1;
    int last_ticks = 0;
  };

  bool open_bus();
  void close_bus();
  bool verify_servos();
  bool apply_servo_settings();
  bool write_if_different(
    const Joint & joint, feetech::Register reg, uint32_t value,
    bool * torque_disabled);
  bool set_torque(bool enable);
  bool read_feedback();

  std::vector<Joint> joints_;
  std::vector<uint8_t> ids_;

  std::string port_;
  std::string calibration_file_;
  int baud_rate_ = 1000000;
  int reply_timeout_ms_ = 10;
  int max_consecutive_read_errors_ = 5;
  bool configure_servos_ = true;
  bool disable_torque_on_deactivate_ = true;
  bool read_only_ = false;
  int p_coefficient_ = 16;
  int i_coefficient_ = 0;
  int d_coefficient_ = 32;

  SerialPort serial_;
  std::unique_ptr<FeetechBus> bus_;
  int consecutive_read_errors_ = 0;
  bool torque_enabled_ = false;
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__SO101_SYSTEM_HPP_
