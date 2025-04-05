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

class GazeboTranslator : public rclcpp::Node
{
  public:
 
    GazeboTranslator() : Node("gazebo_translator")
    {
        this->declare_parameter<std::vector<std::string>>("joint_names", {""});
        this->declare_parameter<std::string>("prefix", "esp32");
        this->declare_parameter<double>("publish_rate", 50.0);
        this->declare_parameter<std::string>("publish_topic", "velocity_controller/commands");

        //make variables for parameters
        std::string prefix = this->get_parameter("prefix").as_string();
        std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string publish_topic = this->get_parameter("publish_topic").as_string();

        //create subscribers per joint
        command_messages.data.resize(joint_names.size());

        for(int i = 0; i < joint_names.size(); i++){
            std::string joint_name = joint_names[i];

            //subscriber
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription = 
                this->create_subscription<std_msgs::msg::Float64>(prefix + "/" + joint_name + "/command", 10, 
                    [this, i](const std_msgs::msg::Float64 &message){
                        command_messages.data.at(i) = message.data;
                    }  
                );

            command_subscriptions.push_back(subscription);

            //state publishers
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub = 
                this->create_publisher<std_msgs::msg::Float64>(prefix + "/" + joint_name + "/position", 10);

            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub = 
                this->create_publisher<std_msgs::msg::Float64>(prefix + "/" + joint_name + "/velocity", 10);

            position_state_publishers.push_back(position_pub);
            velocity_state_publishers.push_back(velocity_pub);
        }
        //create publisher
        command_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic, 10);

        //create timer to publish array
        timer = this->create_wall_timer(
            std::chrono::milliseconds(int((1.0 / publish_rate) * 1000)),
            [this](){
                command_publisher->publish(command_messages);
            }
        );

        //====== state publishers and subscribers ========
        joint_state_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        [this, joint_names](const sensor_msgs::msg::JointState &message){
            //for each joint in joint_states
            for(int i = 0; i < message.name.size(); i++){
                std_msgs::msg::Float64 position;
                std_msgs::msg::Float64 velocity;

                position.data = message.position.at(i);
                velocity.data = message.velocity.at(i);

                position_state_publishers.at(i)->publish(position);
                velocity_state_publishers.at(i)->publish(velocity);
            }
        }
        );
    }

  private:
    std_msgs::msg::Float64MultiArray command_messages;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> command_subscriptions;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> position_state_publishers;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> velocity_state_publishers;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboTranslator>());
  rclcpp::shutdown();
  return 0;
}