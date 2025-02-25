// Copyright 2021 ros2_control Development Team
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

#include "include/microros_hardware_interface/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "include/microros_hardware_interface/constants.hpp"

#include "include/microros_hardware_interface/util/utility.hpp"

namespace microros_hardware_interface{

hardware_interface::CallbackReturn MicroSystemHardware::on_init(const hardware_interface::HardwareInfo & info){

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string prefix = info_.hardware_parameters["prefix"];

  std::string left_joint = info_.hardware_parameters["left_joint"];
  std::string right_joint = info_.hardware_parameters["right_joint"];

  std::string left_topic = info_.hardware_parameters["left_topic"];
  std::string right_topic = info_.hardware_parameters["right_topic"];

  double conversion = std::stod(info_.hardware_parameters["conversion"]);

  bool left_inverted = utility::string_to_bool(info_.hardware_parameters["left_inverted"]);
  bool right_inverted = utility::string_to_bool(info_.hardware_parameters["right_inverted"]);

  left_motor = std::make_shared<MotorInterface>(left_joint, left_topic, prefix, conversion, left_inverted);
  right_motor = std::make_shared<MotorInterface>(right_joint, right_topic, prefix, conversion, right_inverted);

  motors.emplace_back(left_motor);
  motors.emplace_back(right_motor);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MicroSystemHardware::export_state_interfaces(){
  
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for(int i = 0; i < motors.size(); i++){
    
    for(hardware_interface::StateInterface state_interface : motors.at(i)->get_state_interfaces()){
      state_interfaces.emplace_back(state_interface);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MicroSystemHardware::export_command_interfaces(){

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for(int i = 0; i < motors.size(); i++){
    command_interfaces.emplace_back(motors.at(i)->get_command_interface());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MicroSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "Activating ...please wait...");
  
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MicroSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "Deactivating ...please wait...");
  
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MicroSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){

  if(!rclcpp::ok())
    return hardware_interface::return_type::ERROR;
 
  for(int i = 0; i < motors.size(); i++){
    rclcpp::spin_some(motors.at(i));
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type microros_hardware_interface ::MicroSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
 
  if(!rclcpp::ok())
    return hardware_interface::return_type::ERROR;

  for(int i = 0; i < motors.size(); i++){
    motors.at(i)->send_command();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace microros_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  microros_hardware_interface::MicroSystemHardware, hardware_interface::SystemInterface)
