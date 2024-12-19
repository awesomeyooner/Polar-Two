#ifndef HEARTBEAT_MONITOR_HEARTBEAT_MONITOR_HPP
#define HEARTBEAT_MONITOR_HEARTBEAT_MONITOR_HPP

#include "Device.hpp"
#include "hardware_interface/handle.hpp"

namespace hardware_component{
    class HeartbeatMonitor : public Device{

        public:

            hardware_component::InterfaceValue heartbeat = {TypeValue::RAW};
            hardware_component::InterfaceValue latency = {TypeValue::LATENCY};
            hardware_component::InterfaceValue hertz = {TypeValue::HERTZ};

            HeartbeatMonitor() : Device(){}

            void apply(ArduinoUtility::ArduinoMessage message){
                if(message.device != device)
                    return;
                
                if(message.message_type == MessageType::STATUS){
                    if(message.type_value == heartbeat.interface_type){
                        latency.value = message.value - heartbeat.value;
                        hertz.value = 1 / (latency.value / 1000);
                        heartbeat.value = message.value;
                    }
                }
            }

            ArduinoUtility::ArduinoMessage send_message(std::string type_value, double value){
                return Device::send_message(MessageType::STATUS, heartbeat.interface_type, heartbeat.value);
            }

            ArduinoUtility::ArduinoMessage send_message(){
                return send_message(heartbeat.interface_type, heartbeat.value);
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