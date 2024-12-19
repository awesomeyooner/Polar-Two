#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class EffortTranslator : public rclcpp::Node
{
  public:
 
    EffortTranslator() : Node("translator")
    {
        velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("velocity_controller/commands", 10);
        position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller/commands", 10);
        
        subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, std::bind(&EffortTranslator::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::TwistStamped & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.axes[0]);

      std_msgs::msg::Float64MultiArray velocity_command = std_msgs::msg::Float64MultiArray();
      std_msgs::msg::Float64MultiArray position_command = std_msgs::msg::Float64MultiArray();

      velocity_command.data.push_back(msg.twist.linear.x);
      position_command.data.push_back(msg.twist.angular.z);

      velocity_publisher -> publish(velocity_command);
      position_publisher -> publish(position_command);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffortTranslator>());
  rclcpp::shutdown();
  return 0;
}