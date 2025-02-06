#ifndef DEVICE_DEVICE_HPP
#define DEVICE_DEVICE_HPP

#include "microros_hardware_interface/arduino_comms.hpp"
#include "microros_hardware_interface/arduino_interface_types.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace hardware_component{

    struct InterfaceValue{
        std::string interface_type;
        double value;

        InterfaceValue(std::string if_type, double val) : interface_type(if_type), value(val){}
        InterfaceValue(std::string if_type) : interface_type(if_type), value(0.0){}
        InterfaceValue() : interface_type(""), value(0.0){}
    };

    class Device{

        private:

        public:

            Device(){}
            
            virtual hardware_interface::StateInterface getStateInterface(){return hardware_interface::StateInterface("null");};
            virtual hardware_interface::CommandInterface getCommandInterface(){return hardware_interface::CommandInterface("null");}

            virtual ArduinoUtility::ArduinoMessage send_message(std::string message_type, std::string type_value, double value){
                return ArduinoUtility::ArduinoMessage{
                    .device = device,
                    .message_type = message_type,
                    .type_value = type_value,
                    .value = value
                };
            }

            virtual ArduinoUtility::ArduinoMessage send_message(){return {};}

            virtual void apply(ArduinoUtility::ArduinoMessage message){}

            std::string device;
            


    };
}

#endif