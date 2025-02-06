#ifndef SYSTEM_MANAGER_SYSTEM_MANAGER_HPP
#define SYSTEM_MANAGER_SYSTEM_MANAGER_HPP

#include "microros_hardware_interface/arduino_interface_types.hpp"
#include <vector>
#include "Subsystem.hpp"
#include "microros_hardware_interface/devices/Motor.hpp"
#include "microros_hardware_interface/devices/Sensor.hpp"
#include "microros_hardware_interface/constants.hpp"
#include "microros_hardware_interface/diffbot_system.hpp"
#include "microros_hardware_interface/constants.hpp"
#include "hardware_interface/handle.hpp"
#include "microros_hardware_interface/devices/HeartbeatMonitor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace subsystem{

    class SystemManager : public Subsystem{

        private:
            hardware_component::HeartbeatMonitor heartbeat_monitor;
            hardware_component::Sensor voltage_sensor;

            std::vector<hardware_component::Device*> devices;

        public:         
  
            SystemManager() : Subsystem(), voltage_sensor(TypeValue::VOLTAGE){

                devices.emplace_back(&heartbeat_monitor);
                devices.emplace_back(&voltage_sensor);
            }

            void initialize(ArduinoUtility::Config config) override{
                heartbeat_monitor.device = config.heartbeat_id;
                voltage_sensor.device = config.voltage_sensor_id;
            }

            std::vector<ArduinoUtility::ArduinoMessage> getMessagesToSend() override{
                std::vector<ArduinoUtility::ArduinoMessage> messages;

                for(hardware_component::Device* device : devices){
                    messages.emplace_back(device->send_message());
                }

                return messages;
            }

            std::vector<hardware_interface::StateInterface> getStateInterfaces() override{
                std::vector<hardware_interface::StateInterface> state_interfaces;

                state_interfaces.emplace_back(heartbeat_monitor.getStateInterface(&heartbeat_monitor.heartbeat));
                state_interfaces.emplace_back(heartbeat_monitor.getStateInterface(&heartbeat_monitor.latency));
                state_interfaces.emplace_back(heartbeat_monitor.getStateInterface(&heartbeat_monitor.hertz));

                state_interfaces.emplace_back(voltage_sensor.getStateInterface(&voltage_sensor.state));

                return state_interfaces;
            }

            void applyToAll(ArduinoUtility::ArduinoMessage message) override{
                for(hardware_component::Device* device : devices){
                    device->apply(message);
                }
            }
    };
}

#endif