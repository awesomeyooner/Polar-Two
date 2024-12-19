#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node{
  public:

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    int count = 0;

    Publisher() : Node("publisher"){
        publisher = this -> create_publisher<std_msgs::msg::String>("topic", 10);

        timer = this -> create_wall_timer(
            20ms, 
            std::bind(&Publisher::timer_callback, this)
            );

        RCLCPP_INFO(this->get_logger(), "======Publisher: Initialized!======");
    }

  private:
    void timer_callback(){
        auto message = std_msgs::msg::String();

        count++;

        message.data = "Hello, World! " + std::to_string(count);

        publisher -> publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}