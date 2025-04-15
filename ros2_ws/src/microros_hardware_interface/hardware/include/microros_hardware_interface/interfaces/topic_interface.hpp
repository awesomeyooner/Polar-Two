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
#include "master_node.hpp"

namespace microros_hardware_interface{
    struct HardwareTopic{
        std::string topic_name;
        std::string interface;

        double data = 0;
        double conversion = 1;
        double inverted = 1;

        /**
         * @brief Construct a new Hardware Topic object
         * 
         * @param topic_name Name of the topic
         * @param interface The type of interface this is associate with
         */
        HardwareTopic(std::string topic_name, std::string interface) : topic_name(topic_name), interface(interface){}

        /**
         * @brief Construct a new Hardware Topic object
         * 
         * @param topic_name Name of the topic
         * @param interface The type of interace this is associate with
         * @param conversion the conversion to use
         */
        HardwareTopic(std::string topic_name, std::string interface, double conversion) : topic_name(topic_name), interface(interface), conversion(conversion){}

        /**
         * @brief Construct a new Hardware Topic object
         * 
         * @param topic_name Name of the topic
         * @param interface The type of interace this is associate with
         * @param conversion the conversion to use
         * @param inverted Whether or not to invert
         */
        HardwareTopic(std::string topic_name, std::string interface, double conversion, bool inverted) : 
            topic_name(topic_name), interface(interface), conversion(conversion){
                set_inverted(inverted);
            }

        /**
         * @brief Inverts the value (negates)
         * 
         * @param invert 
         */
        void set_inverted(bool invert){
            inverted = invert ? -1 : 1;
        }

        /**
         * @brief Gets the value at this topic with the conversion factor and inversion applied
         * 
         * @return double 
         */
        double get(){
            return data * conversion * inverted;
        }
    };

    class TopicInterface {

        private:
            std::vector<HardwareTopic>* states;
            HardwareTopic* command;

        public:

            std::vector<rclcpp::Time> timestamps;

            TopicInterface(){}

            /**
             * @brief Initializes the subscribers and publishers
             * 
             * @param node The master node that the subscribers and publishers will be using
             * @param name The name of the topic 
             * @param prefix A prefix to add before the name
             * @param command_topic The HardwareTopic to publish to when sending commands
             * @param state_topics The HardwareTopics to subscribe to for states
             */
            virtual void initialize(std::shared_ptr<MasterNode> node, const std::string& name, const std::string& prefix, HardwareTopic* command_topic, std::vector<HardwareTopic>* state_topics){
                
                //resize the array to avoid exception
                timestamps.resize(state_topics->size());
                
                //sets the global variables to the parameters
                states = state_topics;
                command = command_topic;

                //populates subscriptions
                for(int i = 0; i < state_topics->size(); i++){
                    
                    // create a subscriber for each state topic
                    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription = node->create_subscription<std_msgs::msg::Float64>(
                        prefix + "/" + name + state_topics->at(i).topic_name, rclcpp::SystemDefaultsQoS(), 
                        [this, node, i](const std_msgs::msg::Float64 &message) {
                            states->at(i).data = states->at(i).conversion * states->at(i).inverted * message.data;
                            timestamps.at(i) = node->now();
                        }
                    );

                    //add this subscriber to the vector of all subscribers
                    subscriptions.push_back(subscription);
                }

                //create command publisher
                publisher = node->create_publisher<std_msgs::msg::Float64>(
                        prefix + "/" + name + command_topic->topic_name, 
                        10
                    );
            }

            /**
             * @brief Sends the command member to the topic
             */
            void send_command(){
                std_msgs::msg::Float64 message = std_msgs::msg::Float64();

                message.data = command->get();

                publisher->publish(message);
            }

            /**
             * @brief Gets all the state interfaces configured
             * 
             * @param owner The joint that these states are associated with
             * @return std::vector<hardware_interface::StateInterface> 
             */
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

            /**
             * @brief Gets the command interface associated with this topic interface
             * 
             * @param owner The joint that this is associated with
             * @return hardware_interface::CommandInterface 
             */
            virtual hardware_interface::CommandInterface get_command_interface(std::string owner){
                return hardware_interface::CommandInterface(
                    owner,
                    command->interface,
                    &command->data
                );
            }
        
        private:
            
            //subscribers for the states
            std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscriptions;

            //publisher for the command
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
    };

}; //namespace microros_hardware_interface

#endif