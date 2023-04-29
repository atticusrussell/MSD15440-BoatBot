// Copyright (c) 2023, Atticus Russell
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESC_HARDWARE__ESC_HARDWARE_HPP_
#define ESC_HARDWARE__ESC_HARDWARE_HPP_

#include <string>
#include <vector>

#include "esc_hardware/visibility_control.h"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// #include "esc_hardware/angular_servo.hpp"
#include "esc_hardware/esc.hpp"

namespace esc_hardware
{
class EscHardware : public hardware_interface::ActuatorInterface
{

struct ESCConfig
{
  std::string name = "";
  int pi = 0;
  int pwmPin = 0;
  int fullRevThrottle = 0;
  int fullFwdThrottle = 0;
  int minPulseWidth_us = 0;
  int maxPulseWidth_us = 0;
  int powerPin = 0;
  int neutralThrottle = 0;
  float minFwdThrottle = 0;
  float minRevThrottle = 0;
};

struct PropJoint
{
  std::string name = "";
  std::unique_ptr<ESC> esc;
  double vel = 0;
  double cmd = 0;
};

public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  ESCConfig esc_cfg_;
  PropJoint prop_joint_;
};

}  // namespace esc_hardware

#endif  // ESC_HARDWARE__ESC_HARDWARE_HPP_
