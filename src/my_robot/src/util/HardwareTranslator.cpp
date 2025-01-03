#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>

using std::placeholders::_1;

class HardwareTranslator : public rclcpp::Node
{
  public:
 
    HardwareTranslator() : Node("hardware_translator")
    {
        //declare parameters
        this->declare_parameter<std::vector<std::string>>("joint_names", {""});
        this->declare_parameter<std::string>("prefix", "esp32");
        this->declare_parameter<double>("publish_rate", 50.0);
    
        //make variables for parameters
        std::string prefix = this->get_parameter("prefix").as_string();
        std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        positions.resize(joint_names.size());
        velocities.resize(joint_names.size());

        //create subscribers per joint
        for(int i = 0; i < joint_names.size(); i++){
            std::string joint_name = joint_names[i];

            //position
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription = 
                this->create_subscription<std_msgs::msg::Float64>(prefix + "/" + joint_name + "/position", 10, 
                    [this, i](const std_msgs::msg::Float64 &message){
                        positions.at(i) = message;
                    }  
                );

            //velocity
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription = 
                this->create_subscription<std_msgs::msg::Float64>(prefix + "/" + joint_name + "/velocity", 10, 
                    [this, i](const std_msgs::msg::Float64 &message){
                        velocities.at(i) = message;
                    }  
                );

            position_state_subscribers.push_back(position_subscription);
            velocity_state_subscribers.push_back(velocity_subscription);
        }

        //create joint_state_publisher
        joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        //create timer to publish states
        timer = this->create_wall_timer(
            std::chrono::milliseconds(int((1.0 / publish_rate) * 1000)),
            [this, joint_names](){

                sensor_msgs::msg::JointState message;
                std_msgs::msg::Header header;

                header.frame_id = "";
                header.stamp.nanosec  = this->now().nanoseconds();
                header.stamp.sec = this->now().seconds();

                message.header = header;

                message.position.resize(joint_names.size());
                message.velocity.resize(joint_names.size());
                message.name.resize(joint_names.size());
                
                for(int i = 0; i < joint_names.size(); i++){
                    message.position.at(i) = positions.at(i).data;
                    message.velocity.at(i) = velocities.at(i).data;
                    message.name.at(i) = joint_names.at(i);
                
                }

                joint_state_publisher->publish(message);
            }
        );
    }

  private:
    std::vector<std_msgs::msg::Float64> positions;
    std::vector<std_msgs::msg::Float64> velocities;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> position_state_subscribers;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> velocity_state_subscribers;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareTranslator>());
  rclcpp::shutdown();
  return 0;
}