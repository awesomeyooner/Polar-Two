#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include "topic_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "master_node.hpp"

namespace microros_hardware_interface{

    class MotorInterface : public TopicInterface{

        public:

            /**
             * @brief Construct a new Motor Interface object
             * 
             * @param node MasterNode to use for subscribers and publishers
             * @param joint The name of the joint
             * @param topic The overall name of the topic
             * @param conversion The conversion to apply to the states
             */
            MotorInterface(std::shared_ptr<MasterNode> node, const std::string& joint, const std::string& topic, const double& conversion) : 
                MotorInterface(node, joint, topic, "", conversion){}


            /**
             * @brief Construct a new Motor Interface object
             * 
             * @param node MasterNode to use for subscribers and publishers
             * @param joint The name of the joint
             * @param topic The overall name of the topic
             * @param prefix The prefix to apply to the topics
             * @param conversion The conversion to apply to the states
             */
            MotorInterface(std::shared_ptr<MasterNode> node, const std::string& joint, const std::string& topic, const std::string& prefix, const double& conversion) : 
                MotorInterface(node, joint, topic, prefix, conversion, false){}

            /**
             * @brief Construct a new Motor Interface object
             * 
             * @param node MasterNode to use for subscribers and publishers
             * @param joint The name of the joint
             * @param topic The overall name of the topic
             * @param prefix The prefix to apply to the topics
             * @param conversion The conversion to apply to the states
             * @param invert Invert the values of the state interfaces
             */
            MotorInterface(std::shared_ptr<MasterNode> node, const std::string& joint, const std::string& topic, const std::string& prefix, const double& conversion, const bool& invert) : 
                joint_name(joint),
                TopicInterface(){
                    set_conversion(conversion, &state_interfaces);
                    set_inverted(invert, &state_interfaces);
                    command_interface.set_inverted(invert);

                    this->initialize(node, topic, prefix, &command_interface, &state_interfaces);
            }

            /**
             * @brief Construct a new Motor Interface object
             * 
             * @param node MasterNode to use for subscribers and publishers
             * @param joint The name of the joint
             * @param topic The overall name of the topic
             * @param prefix The prefix to apply to the topics
             * @param conversion The conversion to apply to the states
             * @param invert_encoder Invert the values of the state interfaces
             * @param invert_motor Invert the values of the command interface
             */
            MotorInterface(std::shared_ptr<MasterNode> node, const std::string& joint, const std::string& topic, const std::string& prefix, const double& conversion, const bool& invert_encoder, const bool& invert_motor) : 
                joint_name(joint),
                TopicInterface(){
                    set_conversion(conversion, &state_interfaces);
                    set_inverted(invert_encoder, &state_interfaces);
                    command_interface.set_inverted(invert_motor);

                    this->initialize(node, topic, prefix, &command_interface, &state_interfaces);
            }

            /**
             * @brief Gets the state interfaces for this joint
             * 
             * @return std::vector<hardware_interface::StateInterface> 
             */
            std::vector<hardware_interface::StateInterface> get_state_interfaces(){
                return TopicInterface::get_state_interfaces(joint_name);
            }

            /**
             * @brief Gets the command interface for this joint
             * 
             * @return hardware_interface::CommandInterface 
             */
            hardware_interface::CommandInterface get_command_interface(){
                return TopicInterface::get_command_interface(joint_name);
            }

            /**
             * @brief Sets the conversion factor for the given HardwareTopics
             * 
             * @param conversion The conversion factor to set. If you want to negate it, use set_inverted
             * @param hardware_topics The HardwareTopics you want to set the conversions for
             */
            void set_conversion(const double& conversion, std::vector<HardwareTopic>* hardware_topics){
                for(int i = 0; i < hardware_topics->size(); i++){
                    if(hardware_topics->at(i).interface == hardware_interface::HW_IF_EFFORT)
                        continue;

                    hardware_topics->at(i).conversion = conversion; 
                }
            }

            /**
             * @brief Inverts the given HardwareTopics
             * 
             * @param invert 
             * @param hardware_topics
             */
            void set_inverted(const bool& invert, std::vector<HardwareTopic>* hardware_topics){
                for(int i = 0; i < hardware_topics->size(); i++){
                    hardware_topics->at(i).set_inverted(invert); 
                }
            }

        private:
            
            //state interfaces for this joint
            std::vector<HardwareTopic> state_interfaces = {
                HardwareTopic("/velocity", hardware_interface::HW_IF_VELOCITY),
                HardwareTopic("/position", hardware_interface::HW_IF_POSITION),
                HardwareTopic("/effort", hardware_interface::HW_IF_EFFORT)
            };

            //command interface for this joint
            HardwareTopic command_interface = HardwareTopic("/command", hardware_interface::HW_IF_VELOCITY);

            //name of the joint
            std::string joint_name;
    };

};


#endif