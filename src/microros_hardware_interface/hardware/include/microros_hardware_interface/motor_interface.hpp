#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "topic_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"

namespace microros_hardware_interface{

    class MotorInterface : public TopicInterface{

        public:

            std::vector<HardwareTopic> states = {
                HardwareTopic("/velocity", hardware_interface::HW_IF_VELOCITY),
                HardwareTopic("/position", hardware_interface::HW_IF_POSITION),
                HardwareTopic("/effort", hardware_interface::HW_IF_EFFORT)
            };

            HardwareTopic command = HardwareTopic("/command", hardware_interface::HW_IF_EFFORT);

            std::string name;

            MotorInterface(std::string name, double conversion) : name(name), 
                TopicInterface(name, command, set_conversion_to_states(conversion, states)){}

            std::vector<hardware_interface::StateInterface> get_state_interfaces(){
                return TopicInterface::get_state_interfaces(name);
            }

            hardware_interface::CommandInterface get_command_interface(){
                return TopicInterface::get_command_interface(name);
            }

        private:

            std::vector<HardwareTopic> set_conversion_to_states(double conversion, std::vector<HardwareTopic> states){
                for(int i = 0; i < states.size(); i++){
                    states.at(i).conversion = conversion; 
                }

                return states;
            }

    };

};


#endif