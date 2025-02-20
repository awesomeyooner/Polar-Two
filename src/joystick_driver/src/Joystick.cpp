#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "../include/gamepad_interface.hpp"
#include "../include/constants.hpp"

using std::placeholders::_1;

class Joystick : public rclcpp::Node
{
  public:
  //0 left x 
  //1 left y
  //2 left trigger
  
  //3 right x
  //4 right y
  //5 right trigger
    Joystick() : Node("joystick_teleop")
    {
        this->declare_parameter<std::string>("joystick_type", "ps4");
        this->declare_parameter<double>("max_tangential_velocity", 1);
        this->declare_parameter<double>("max_angular_velocity", 3.14);

        std::string joystick_type = this->get_parameter("joystick_type").as_string();

        if(joystick_type == "ps4")
          gamepad.initialize(hid_devices::PS4::MAP);
        else if(joystick_type == "xbox")
          gamepad.initialize(hid_devices::Xbox::MAP);
        
        // rclcpp::Parameter
        max_tan = this->get_parameter("max_tangential_velocity").as_double();
        max_ang = this->get_parameter("max_angular_velocity").as_double();
  
        publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Joystick::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy &msg) {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.axes[0]);

      gamepad.update(msg);

      if(gamepad.get_button(hid_devices::PS4::X)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed X");
      
      if(gamepad.get_button(hid_devices::PS4::X)->on_release)
        RCLCPP_INFO(this->get_logger(), "released X");

      if(gamepad.get_button(hid_devices::PS4::CIRCLE)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed CIRCLE");

      if(gamepad.get_button(hid_devices::PS4::SQUARE)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed SQUARE");

      if(gamepad.get_button(hid_devices::PS4::TRIANGLE)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed TRIANGLE");

      if(gamepad.get_button(hid_devices::PS4::LEFT_STICK)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed LEFT STICk");

      if(gamepad.get_button(hid_devices::PS4::RIGHT_STICK)->on_press)
        RCLCPP_INFO(this->get_logger(), "pressed RIGHT STICK");

      auto twist_stamped = geometry_msgs::msg::TwistStamped();

      twist_stamped.header.frame_id = "command_velocity";
      twist_stamped.header.stamp = this->now();

      twist_stamped.twist.linear.x = gamepad.get_axis(hid_devices::GamepadAxis::LEFT_Y)->get() * max_tan;
      twist_stamped.twist.angular.z = gamepad.get_axis(hid_devices::GamepadAxis::RIGHT_X)->get() * max_ang;

      publisher->publish(twist_stamped);
      
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;

    hid_devices::Gamepad gamepad;

    double max_tan;
    double max_ang;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joystick>());
  rclcpp::shutdown();
  return 0;
}