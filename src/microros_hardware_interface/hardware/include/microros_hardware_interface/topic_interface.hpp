#ifndef TOPIC_INTERFACE_HPP
#define TOPIC_INTERFACE_HPP

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "hardware_interface/handle.hpp"

namespace microros_hardware_interface{

    struct HardwareTopic{
        std::string topic_name;
        std::string interface;

        double data = 0;
        double conversion = 1;

        HardwareTopic(std::string topic_name, std::string interface) : topic_name(topic_name), interface(interface){}
        HardwareTopic(std::string topic_name, std::string interface, double conversion) : topic_name(topic_name), interface(interface), conversion(conversion){}

        void set_inverted(bool invert){
            conversion *= invert ? -1 : 1; //if inverted then negate
        }
    };

    class TopicInterface : public rclcpp::Node {

        private:
            std::vector<HardwareTopic>* states;
            HardwareTopic* command;

        public:

            std::vector<rclcpp::Time> timestamps;

            TopicInterface(const std::string& name) : Node(name + "_node"){}

            virtual void initialize(const std::string& name, const std::string& prefix, HardwareTopic* command_topic, std::vector<HardwareTopic>* state_topics){
    
                timestamps.resize(state_topics->size());
                
                states = state_topics;
                command = command_topic;

                //populates subscriptions
                for(int i = 0; i < state_topics->size(); i++){
                    
                    // subscribers
                    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription = this->create_subscription<std_msgs::msg::Float64>(
                        prefix + "/" + name + state_topics->at(i).topic_name, rclcpp::SystemDefaultsQoS(), 
                        [this, i](const std_msgs::msg::Float64 &message) {
                            states->at(i).data = states->at(i).conversion * message.data;
                            timestamps.at(i) = this->now();
                        }
                    );

                    subscriptions.push_back(subscription);
                }

                //publisher
                publisher = this->create_publisher<std_msgs::msg::Float64>(
                        prefix + "/" + name + command_topic->topic_name, 10);
            }

            void send_command(){
                std_msgs::msg::Float64 message = std_msgs::msg::Float64();

                message.data = command->data;

                publisher->publish(message);
            }

            virtual std::vector<hardware_interface::StateInterface> get_state_interfaces(std::string owner){
                std::vector<hardware_interface::StateInterface> state_interfaces;

                for(int i = 0; i < states->size(); i++){
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            owner,
                            states->at(i).interface,
                            &states->at(i).data
                        )
                    );
                }

                return state_interfaces;
            }

            virtual hardware_interface::CommandInterface get_command_interface(std::string owner){
                return hardware_interface::CommandInterface(
                    owner,
                    command->interface,
                    &command->data
                );
            }
        
        private:
            std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
    };
};

#endif