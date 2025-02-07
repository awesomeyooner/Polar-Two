#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "topic_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"

namespace microros_hardware_interface{

    class MotorInterface : public TopicInterface{

        public:

            std::vector<HardwareTopic> state_interfaces = {
                HardwareTopic("/velocity", hardware_interface::HW_IF_VELOCITY),
                HardwareTopic("/position", hardware_interface::HW_IF_POSITION),
                HardwareTopic("/effort", hardware_interface::HW_IF_EFFORT)
            };

            HardwareTopic command_interface = HardwareTopic("/command", hardware_interface::HW_IF_EFFORT);

            std::string joint_name;
            std::string topic_namespace;

            MotorInterface(std::string joint, std::string topic, double conversion) : joint_name(joint), topic_namespace(topic), 
                TopicInterface(topic, command_interface, state_interfaces/*set_conversion_to_states(conversion, state_interfaces)*/){
                    // RCLCPP_INFO(this->get_logger(), "Topic Size: '%f'", states.size());
                }

            std::vector<hardware_interface::StateInterface> get_state_interfaces(){
                return TopicInterface::get_state_interfaces(joint_name);
            }

            hardware_interface::CommandInterface get_command_interface(){
                return TopicInterface::get_command_interface(joint_name);
            }

        private:

            std::vector<HardwareTopic> set_conversion_to_states(double conversion, std::vector<HardwareTopic> convert){
                for(int i = 0; i < convert.size(); i++){
                    convert.at(i).conversion = conversion; 
                }

                return convert;
            }

    };

};


#endif