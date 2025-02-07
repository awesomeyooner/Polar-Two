#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "topic_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"

namespace microros_hardware_interface{

    class MotorInterface : public TopicInterface{

        public:

            std::vector<HardwareTopic> state_interfaces;
            HardwareTopic command_interface = HardwareTopic("/command", hardware_interface::HW_IF_EFFORT);

            std::string joint_name;

            MotorInterface(const std::string& joint, const std::string& topic, const double& conversion) : 
                state_interfaces{
                    HardwareTopic("/velocity", hardware_interface::HW_IF_VELOCITY),
                    HardwareTopic("/position", hardware_interface::HW_IF_POSITION),
                    HardwareTopic("/effort", hardware_interface::HW_IF_EFFORT)
                },
                joint_name(joint),
                TopicInterface(topic, &command_interface, &state_interfaces){
                    RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "saijdodsa: '%i'", static_cast<int>(state_interfaces.size()));
                }

            MotorInterface(const std::string& joint, const std::string& topic, HardwareTopic* command, std::vector<HardwareTopic>* states) : 
                state_interfaces{
                    HardwareTopic("/velocity", hardware_interface::HW_IF_VELOCITY),
                    HardwareTopic("/position", hardware_interface::HW_IF_POSITION),
                    HardwareTopic("/effort", hardware_interface::HW_IF_EFFORT)
                },
                joint_name(topic),
                TopicInterface(topic, command, states){
                    RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), "saijdodsa: '%i'", static_cast<int>(state_interfaces.size()));
                }

            std::vector<hardware_interface::StateInterface> get_state_interfaces(){
                return TopicInterface::get_state_interfaces(joint_name);
            }

            hardware_interface::CommandInterface get_command_interface(){
                return TopicInterface::get_command_interface(joint_name);
            }

            static std::vector<HardwareTopic>& set_conversion_for_states(const double& conversion, std::vector<HardwareTopic>& convert){

                for(int i = 0; i < convert.size(); i++){
                    convert.at(i).conversion = conversion; 
                }

                return convert;
            }

            static std::vector<HardwareTopic>& set_inverted(const bool& invert, std::vector<HardwareTopic>& convert){

                for(int i = 0; i < convert.size(); i++){
                    convert.at(i).set_inverted(invert); 
                }

                return convert;
            }

        private:

    };

};


#endif