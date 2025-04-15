#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

class EffortTranslator : public rclcpp::Node
{
  public:
 
    EffortTranslator() : Node("effort_translator")
    {
        this->declare_parameter<std::vector<std::string>>("left_joint_names", {""});
        this->declare_parameter<std::vector<std::string>>("right_joint_names", {""});
        this->declare_parameter<std::string>("prefix", "esp32");
        this->declare_parameter<double>("publish_rate", 50.0);

        //make variables for parameters
        std::string prefix = this->get_parameter("prefix").as_string();
        std::vector<std::string> left_joint_names = this->get_parameter("left_joint_names").as_string_array();
        std::vector<std::string> right_joint_names = this->get_parameter("right_joint_names").as_string_array();

        //create pubs for left
        for(std::string left_joint_name : left_joint_names){
          rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::Float64>(prefix + "/" + left_joint_name + "/command", 10);

          left_publishers.push_back(publisher);
        }

        //create pubs for right
        for(std::string right_joint_name : right_joint_names){
          rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher = 
            this->create_publisher<std_msgs::msg::Float64>(prefix + "/" + right_joint_name + "/command", 10);

          right_publishers.push_back(publisher);
        }

        //create subscriber
        subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, std::bind(&EffortTranslator::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::TwistStamped & msg) const{
      std_msgs::msg::Float64 left_command = std_msgs::msg::Float64();
      std_msgs::msg::Float64 right_command = std_msgs::msg::Float64();

      left_command.data = 30 * (msg.twist.linear.x - (msg.twist.angular.z * 0.7));
      right_command.data = 30 * (msg.twist.linear.x + (msg.twist.angular.z * 0.7));

      for(int i = 0; i < left_publishers.size(); i++){
        left_publishers.at(i)->publish(left_command);
      }

      for(int i = 0; i < right_publishers.size(); i++){
        right_publishers.at(i)->publish(right_command);
      }
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> left_publishers;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> right_publishers;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffortTranslator>());
  rclcpp::shutdown();
  return 0;
}