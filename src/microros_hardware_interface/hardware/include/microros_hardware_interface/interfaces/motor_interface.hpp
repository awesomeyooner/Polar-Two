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

            HardwareTopic command_interface = HardwareTopic("/command", hardware_interface::HW_IF_VELOCITY);

            std::string joint_name;

            MotorInterface(const std::string& joint, const std::string& topic, const double& conversion) : 
                MotorInterface(joint, topic, "", conversion){}

            MotorInterface(const std::string& joint, const std::string& topic, const std::string& prefix, const double& conversion) : 
                joint_name(joint),
                TopicInterface(topic){
                    set_conversion_for_states(conversion, &state_interfaces);
                    this->initialize(topic, prefix, &command_interface, &state_interfaces);
            }

            std::vector<hardware_interface::StateInterface> get_state_interfaces(){
                return TopicInterface::get_state_interfaces(joint_name);
            }

            hardware_interface::CommandInterface get_command_interface(){
                return TopicInterface::get_command_interface(joint_name);
            }

            void set_conversion_for_states(const double& conversion, std::vector<HardwareTopic>* convert){
                for(int i = 0; i < convert->size(); i++){
                    convert->at(i).conversion = conversion; 
                }
            }

            void set_inverted(const bool& invert, std::vector<HardwareTopic>* convert){
                for(int i = 0; i < convert->size(); i++){
                    convert->at(i).set_inverted(invert); 
                }
            }

        private:

    };

};


#endif