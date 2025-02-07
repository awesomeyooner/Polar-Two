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
    };

    class TopicInterface : public rclcpp::Node{

        public:

            std::vector<HardwareTopic> states;
            HardwareTopic command;

            // std::vector<rclcpp::Time> timestamps;

            TopicInterface(std::string name, std::string prefix, HardwareTopic command_topic, std::vector<HardwareTopic> state_topics) : 
                Node(name + "_node"),
                command(command_topic), states(state_topics){

                // states.resize(state_topics.size());

                //timestamps.resize(state_topics.size());

                RCLCPP_INFO(this->get_logger(), "Topic Size: '%f'", state_topics.size());
                // subscriptions.resize(state_topics.size());
                
                //populates subscriptions
                for(int i = 0; i < state_topics.size(); i++){
                    
                    //subscribers
                    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription = this->create_subscription<std_msgs::msg::Float64>(
                        prefix + state_topics.at(i).topic_name, rclcpp::SystemDefaultsQoS(), 
                        [this, i](const std_msgs::msg::Float64 &message) {
                            states.at(i).data = states.at(i).conversion * message.data;
                            // timestamps.at(i) = this->now();
                        }
                    );

                    subscriptions.emplace_back(subscription);
                }

                //publisher
                publisher = this->create_publisher<std_msgs::msg::Float64>(
                        prefix + command_topic.topic_name, 10);
            }

            //defaults name to just the name with _node appended
            TopicInterface(std::string name, HardwareTopic command_topic, std::vector<HardwareTopic> state_topics) : 
                TopicInterface(name, "", command_topic, state_topics){}

            void send_command(){
                std_msgs::msg::Float64 message = std_msgs::msg::Float64();

                message.data = command.data;

                publisher->publish(message);
            }

            std::vector<hardware_interface::StateInterface> get_state_interfaces(std::string owner){
                std::vector<hardware_interface::StateInterface> state_interfaces;

                for(int i = 0; i < states.size(); i++){
                    state_interfaces.emplace_back(
                        hardware_interface::StateInterface(
                            owner,
                            states.at(i).interface,
                            &states.at(i).data
                        )
                    );
                }

                return state_interfaces;
            }

            hardware_interface::CommandInterface get_command_interface(std::string owner){
                return hardware_interface::CommandInterface(
                    owner,
                    command.interface,
                    &command.data
                );
            }
        
        private:

            std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
    };
};

#endif