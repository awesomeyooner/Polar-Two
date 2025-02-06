#ifndef MOTOR_MOTOR_HPP
#define MOTOR_MOTOR_HPP

#include "Device.hpp"
#include "nlohmann/json.hpp"
#include "microros_hardware_interface/arduino_interface_types.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

namespace hardware_component{

    class Motor : public Device{

        public:
            hardware_component::InterfaceValue command = {MessageType::UNCONFIGURED};

            hardware_component::InterfaceValue position = {hardware_interface::HW_IF_POSITION};
            hardware_component::InterfaceValue velocity = {hardware_interface::HW_IF_VELOCITY};
            hardware_component::InterfaceValue effort = {hardware_interface::HW_IF_EFFORT};

            Motor(const std::string& command_interface) : Device(), command{command_interface, 0}{}
            
            void apply(ArduinoUtility::ArduinoMessage message){
                if(message.device != device)
                    return;

                if(message.message_type == MessageType::STATUS){
                    if(message.type_value == TypeValue::POSITION)
                        position.value = message.value;
                    else if(message.type_value == TypeValue::VELOCITY)
                        velocity.value = message.value;
                    else if(message.type_value == TypeValue::EFFORT)
                        effort.value = message.value;
                }
            }

            ArduinoUtility::ArduinoMessage send_message(){
                return send_message(command.interface_type, command.value);
            }

            ArduinoUtility::ArduinoMessage send_message(std::string mode, double value){
                return ArduinoUtility::ArduinoMessage{
                    .device = device,
                    .message_type = MessageType::CONTROL,
                    .type_value = mode,
                    .value = value
                };
            }

            hardware_interface::StateInterface getStateInterface(hardware_component::InterfaceValue* wanted){
                return hardware_interface::StateInterface(
                    device,
                    wanted->interface_type,
                    &wanted->value
                );
            }

            hardware_interface::CommandInterface getCommandInterfaces(hardware_component::InterfaceValue* wanted){
                return hardware_interface::CommandInterface(
                    device,
                    wanted->interface_type,
                    &wanted->value
                );
            }

            ArduinoUtility::ArduinoMessage config_bound(std::string type, double value){
                return ArduinoUtility::ArduinoMessage{
                    .device = device,
                    .message_type = MessageType::CONFIG,
                    .type_value = type,
                    .value = value
                };
            }
    };
}
#endif