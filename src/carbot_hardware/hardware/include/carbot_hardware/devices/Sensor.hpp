#ifndef SENSOR_SENSOR_HPP
#define SENSOR_SENSOR_HPP

#include <nlohmann/json.hpp>

#include "Device.hpp"
#include "carbot_hardware/arduino_interface_types.hpp"

using json = nlohmann::json;

namespace hardware_component{

    class Sensor : public Device{

        private:

        public:

            hardware_component::InterfaceValue state = {MessageType::UNCONFIGURED};

            Sensor(const std::string& state_interface) : Device(), state{state_interface, 0}{}

            void apply(ArduinoUtility::ArduinoMessage message){
                if(message.device != device)
                    return;

                if(message.message_type == MessageType::STATUS){
                    if(message.type_value == state.interface_type)
                        state.value = message.value;                    
                }
            }

            ArduinoUtility::ArduinoMessage send_message(std::string type_value, double value){
                return Device::send_message(MessageType::STATUS, state.interface_type, state.value);
            }

            ArduinoUtility::ArduinoMessage send_message(){
                return send_message(state.interface_type, state.value);
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
            
    };
}

#endif