#ifndef SUPERSTRUCTURE_SUPERSTRUCTURE_HPP
#define SUPERSTRUCTURE_SUPERSTRUCTURE_HPP

#include "microros_hardware_interface/arduino_interface_types.hpp"
#include <vector>
#include "Subsystem.hpp"
#include "microros_hardware_interface/devices/Motor.hpp"
#include "microros_hardware_interface/devices/Sensor.hpp"
#include "microros_hardware_interface/constants.hpp"
#include "microros_hardware_interface/diffbot_system.hpp"
#include "microros_hardware_interface/constants.hpp"
#include "hardware_interface/handle.hpp"
#include "Drive.hpp"
#include "SystemManager.hpp"
#include <iterator>
#include "rclcpp/rclcpp.hpp"

namespace subsystem{

    class Superstructure : public Subsystem{

        private:
            std::vector<subsystem::Subsystem*> subsystems;

            subsystem::SystemManager system_manager;
            subsystem::Drive drive;
            
        public:         
  
            Superstructure() : Subsystem(){
                subsystems.emplace_back(&system_manager);
                subsystems.emplace_back(&drive);
            }

            void initialize(ArduinoUtility::Config config) override{
                
                for(subsystem::Subsystem* subsystem : subsystems){
                    subsystem->initialize(config);
                }
            }

            std::vector<ArduinoUtility::ArduinoMessage> getMessagesToSend() override{
                std::vector<ArduinoUtility::ArduinoMessage> commands;

                for(subsystem::Subsystem* subsystem : subsystems){
                    std::vector<ArduinoUtility::ArduinoMessage> currentCommandPacket = subsystem->getMessagesToSend();
                    
                    commands.insert(commands.end(), currentCommandPacket.begin(), currentCommandPacket.end());
                }

                return commands;
            }

            std::vector<hardware_interface::StateInterface> getStateInterfaces() override{
                std::vector<hardware_interface::StateInterface> state_interfaces;

                for(subsystem::Subsystem* subsystem : subsystems){

                    for(hardware_interface::StateInterface interface : subsystem->getStateInterfaces()){
                        state_interfaces.emplace_back(interface);
                    }

                }

                return state_interfaces;
            }

            std::vector<hardware_interface::CommandInterface> getCommandInterfaces() override{
                std::vector<hardware_interface::CommandInterface> command_interfaces;

                for(subsystem::Subsystem* subsystem : subsystems){
                    std::vector<hardware_interface::CommandInterface> currentCommandInterface = subsystem->getCommandInterfaces();
                    
                    for(int i = 0; i < currentCommandInterface.size(); i++){
                        //cant emplace back, cant copy command interface
                    }
                }

                return  drive.getCommandInterfaces();
            }

            void applyToAll(ArduinoUtility::ArduinoMessage message) override{

                for(subsystem::Subsystem* subsystem : subsystems){
                    subsystem->applyToAll(message);
                }
            }

            std::vector<ArduinoUtility::ArduinoMessage> configDevices() override{
                std::vector<ArduinoUtility::ArduinoMessage> messages;

                for(subsystem::Subsystem* subsystem : subsystems){
                    std::vector<ArduinoUtility::ArduinoMessage> currentConfigPacket = subsystem->configDevices();

                    messages.insert(messages.end(), currentConfigPacket.begin(), currentConfigPacket.end());
                }

                return messages;
            }

    };
}

#endif