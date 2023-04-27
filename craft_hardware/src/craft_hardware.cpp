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

#include <limits>
#include <pigpiod_if2.h>
#include <vector>

#include "craft_hardware/craft_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace craft_hardware
{
hardware_interface::CallbackReturn CraftHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // TEST(anyone): read parameters and initialize the hardware
  rudder_joint_.name = info_.hardware_parameters["srv_name"];
  srv_cfg_.pin = std::stoi(info_.hardware_parameters["srv_pin"]);
  srv_cfg_.min_angle = std::stof(info_.hardware_parameters["srv_min_angle"]);
  srv_cfg_.max_angle = std::stof(info_.hardware_parameters["srv_max_angle"]);
  srv_cfg_.min_pulse_width_us = std::stoi(info_.hardware_parameters["srv_min_pulse_width_us"]);
  srv_cfg_.max_pulse_width_us = std::stoi(info_.hardware_parameters["srv_max_pulse_width_us"]);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CraftHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  bool piGpiodRunning  = system("pgrep pigpiod");

  if (piGpiodRunning == 0) {
      // pigpiod daemon is running
      return CallbackReturn::SUCCESS;
  } else {
      // pigpiod daemon is not running
      return CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn CraftHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TEST(anyone): free resources, stop threads, etc.
  pigpio_stop(rudder_joint_.servo->pi);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CraftHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
    // TEST(anyone): insert correct interfaces
    rudder_joint_.name, hardware_interface::HW_IF_POSITION, &rudder_joint_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CraftHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TEST(anyone): insert correct interfaces
      rudder_joint_.name, hardware_interface::HW_IF_POSITION, &rudder_joint_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn CraftHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TEST(anyone): prepare the robot to receive commands
  rudder_joint_.name = srv_cfg_.name;
  rudder_joint_.servo = std::make_unique<AngularServo>(srv_cfg_.pin, srv_cfg_.min_angle, srv_cfg_.max_angle, srv_cfg_.min_pulse_width_us, srv_cfg_.max_pulse_width_us);
  // srv_joint.servo = ang_srv_;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CraftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TEST(anyone): prepare the robot to stop receiving commands
  // FUTURE maybe make a destructor for AngularServo?
  rudder_joint_.servo->setPulseWidth(0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CraftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TEST(anyone): read robot states
  float currentAngleDegrees = rudder_joint_.servo->getAngle();
  // convert from degrees to radians
  rudder_joint_.pos = currentAngleDegrees * M_PI / 180;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CraftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TEST(anyone): write robot's commands'
  // FUTURE change function names to setAngleDeg and getAngleDeg
  // convert from radians to degrees
  float desiredAngleDegrees = rudder_joint_.cmd * 180 / M_PI;
  rudder_joint_.servo->setAngle(desiredAngleDegrees);

  return hardware_interface::return_type::OK;
}

}  // namespace craft_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  craft_hardware::CraftHardware, hardware_interface::SystemInterface)
