#ifndef DRIVE_DRIVE_HPP
#define DRIVE_DRIVE_HPP

#include "carbot_hardware/arduino_interface_types.hpp"
#include <vector>
#include "Subsystem.hpp"
#include "carbot_hardware/devices/Motor.hpp"
#include "carbot_hardware/devices/Sensor.hpp"
#include "carbot_hardware/constants.hpp"
#include "carbot_hardware/diffbot_system.hpp"
#include "carbot_hardware/constants.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace subsystem{

    class Drive : public Subsystem{

        private:
            
            std::vector<hardware_component::Device*> devices;

        public:         
            hardware_component::Motor drive_motor;
            hardware_component::Motor steer_motor;

            Drive() : Subsystem(),
             drive_motor(hardware_interface::HW_IF_VELOCITY), 
             steer_motor(hardware_interface::HW_IF_POSITION){

                devices.emplace_back(&drive_motor);
                devices.emplace_back(&steer_motor);
            }

            void initialize(ArduinoUtility::Config config) override{
                //RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), config.drive_id.c_str());
                drive_motor.device = config.drive_id;
                steer_motor.device = config.steer_id;
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

                state_interfaces.emplace_back(drive_motor.getStateInterface(&drive_motor.velocity));
                state_interfaces.emplace_back(drive_motor.getStateInterface(&drive_motor.position));
                state_interfaces.emplace_back(drive_motor.getStateInterface(&drive_motor.effort));

                state_interfaces.emplace_back(steer_motor.getStateInterface(&steer_motor.position));
                state_interfaces.emplace_back(steer_motor.getStateInterface(&steer_motor.effort));

                return state_interfaces;
            }

            std::vector<hardware_interface::CommandInterface> getCommandInterfaces() override{
                std::vector<hardware_interface::CommandInterface> command_interfaces;

                command_interfaces.emplace_back(drive_motor.getCommandInterfaces(&drive_motor.command));
                command_interfaces.emplace_back(steer_motor.getCommandInterfaces(&steer_motor.command));

                return command_interfaces;
            }

            void applyToAll(ArduinoUtility::ArduinoMessage message) override{
                for(hardware_component::Device* device : devices){
                    device->apply(message);
                }
            }

            std::vector<ArduinoUtility::ArduinoMessage> configDevices() override{
                std::vector<ArduinoUtility::ArduinoMessage> messages;

                messages.push_back(drive_motor.config_bound(TypeValue::LOWER_BOUND, MotorConstants::MAX_REVERSE));
                messages.push_back(drive_motor.config_bound(TypeValue::UPPER_BOUND, MotorConstants::MAX_FORWARD));
                messages.push_back(drive_motor.config_bound(TypeValue::NEUTRAL, MotorConstants::NEUTRAL));

                messages.push_back(steer_motor.config_bound(TypeValue::LOWER_BOUND, ServoConstants::MAX_RIGHT));
                messages.push_back(steer_motor.config_bound(TypeValue::UPPER_BOUND, ServoConstants::MAX_LEFT));
                messages.push_back(steer_motor.config_bound(TypeValue::NEUTRAL, ServoConstants::NEUTRAL));

                return messages;
            }

    };
}

#endif