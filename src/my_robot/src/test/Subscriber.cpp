#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class Subscriber : public rclcpp::Node{

  public:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

    Subscriber() : Node("subscriber"){
      subscription = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&Subscriber::topic_callback, this, _1));

      RCLCPP_INFO(this->get_logger(), "======Subscriber: Initialized!======");
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr message) const{
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", message -> data.c_str());
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Subscriber>());

  rclcpp::shutdown();
  return 0;
}