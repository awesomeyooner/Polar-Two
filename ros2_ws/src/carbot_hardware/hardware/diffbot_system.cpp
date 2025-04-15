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

#include "carbot_hardware/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "include/carbot_hardware/arduino_interface_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "include/carbot_hardware/constants.hpp"

namespace carbot_hardware{

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info){

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
    return hardware_interface::CallbackReturn::ERROR;
  }

  //set values to config
  config.port = info_.hardware_parameters["port"];

  config.drive_id = info_.hardware_parameters["drive_id"];
  config.steer_id = info_.hardware_parameters["steer_id"];

  config.voltage_sensor_id = info_.hardware_parameters["voltage_sensor_id"];

  config.heartbeat_id = info_.hardware_parameters["heartbeat_id"];

  config.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  config.loop_rate = std::stoi(info_.hardware_parameters["loop_rate"]);
  config.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  superstructure.initialize(config);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces(){
  return superstructure.getStateInterfaces();
}

std::vector<hardware_interface::CommandInterface> CarlikeBotSystemHardware::export_command_interfaces(){
  return superstructure.getCommandInterfaces();
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");
  comms.connect(config.port, config.baud_rate, config.timeout_ms);

  comms.send_message(superstructure.configDevices()); 
  
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/){
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");
  comms.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period){

  std::vector<ArduinoUtility::ArduinoMessage> messages = comms.get_message_dump();
 
  for(ArduinoUtility::ArduinoMessage message : messages){
    superstructure.applyToAll(message);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type carbot_hardware ::CarlikeBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  std::vector<ArduinoUtility::ArduinoMessage> messages = superstructure.getMessagesToSend();

  comms.send_message(messages); 

  return hardware_interface::return_type::OK;
}

}  // namespace carbot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  carbot_hardware::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
