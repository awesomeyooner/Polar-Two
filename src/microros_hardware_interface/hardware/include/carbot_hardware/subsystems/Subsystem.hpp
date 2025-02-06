#ifndef SUBSYSTEM_SUBSYSTEM_HPP
#define SUBSYSTEM_SUBSYSTEM_HPP

#include "microros_hardware_interface/arduino_interface_types.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <vector>
#include "microros_hardware_interface/diffbot_system.hpp"
#include "rclcpp/rclcpp.hpp"

namespace subsystem{

    class Subsystem{

        private:

        public:

            Subsystem(){}

            virtual std::vector<ArduinoUtility::ArduinoMessage> getMessagesToSend(){return {};}
        
            virtual std::vector<hardware_interface::StateInterface> getStateInterfaces(){return {};};
            virtual std::vector<hardware_interface::CommandInterface> getCommandInterfaces(){return {};}

            virtual std::vector<ArduinoUtility::ArduinoMessage> configDevices(){return {};}

            virtual void initialize(ArduinoUtility::Config config){
                RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "====================== init ======================");
            }

            virtual void applyToAll(ArduinoUtility::ArduinoMessage message){
                RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "====================== apply ======================");
            }

    };
}

#endif