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
 
    EffortTranslator() : Node("translator")
    {
        
        left_publisher = this->create_publisher<std_msgs::msg::Float64>("esp32/left_motor/command", 10);
        right_publisher = this->create_publisher<std_msgs::msg::Float64>("esp32/right_motor/command", 10);

        subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, std::bind(&EffortTranslator::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::TwistStamped & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.axes[0]);

      std_msgs::msg::Float64 left_command = std_msgs::msg::Float64();
      std_msgs::msg::Float64 right_command = std_msgs::msg::Float64();

      left_command.data = (msg.twist.linear.x - (msg.twist.angular.z * 0.4));
      right_command.data = (msg.twist.linear.x + (msg.twist.angular.z * 0.4));
  
      left_publisher->publish(left_command);
      right_publisher->publish(right_command);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EffortTranslator>());
  rclcpp::shutdown();
  return 0;
}