#ifndef MASTER_NODE_HPP
#define MASTER_NODE_HPP

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

    class MasterNode : public rclcpp::Node{

        public:

            MasterNode(std::string name) : Node(name){

            }

        private:

            

    };

};

#endif