#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <driverstation_msgs/msg/driverstation_status.hpp>

#include "../include/driverstation/dsgui.h"
#include <QApplication>

using std::placeholders::_1;

class DriverstationNode : public rclcpp::Node {

  public:
    DriverstationNode(int argc, char * argv[]) : Node("driverstation_gui"), application(argc, argv){
          this->declare_parameter<int>("max_statuses", 10);

          // rclcpp::Parameter
          max_statuses = this->get_parameter("max_statuses").as_int();
      
          subscription = this->create_subscription<driverstation_msgs::msg::DriverstationStatus>(
          "console", 10, std::bind(&DriverstationNode::topic_callback, this, _1));

          initialize();
    }

    int initialize(){
      gui.setWindowTitle(QString::fromStdString("Driverstation"));
      gui.show();
      return application.exec();
    }

  private:
    void topic_callback(const driverstation_msgs::msg::DriverstationStatus &message) {
      RCLCPP_INFO(this->get_logger(), message.message.c_str());

      recent_statuses.insert(recent_statuses.begin(), message);
      
      //if the current size is greater than the max, then start removing
      if(recent_statuses.size() > max_statuses){
        //for every item thats over the limit, remove it
        for(int i = 0; i < recent_statuses.size() - max_statuses; i++){
          recent_statuses.pop_back();
        }
      }
      
      
    }

    rclcpp::Subscription<driverstation_msgs::msg::DriverstationStatus>::SharedPtr subscription;

    std::vector<driverstation_msgs::msg::DriverstationStatus> recent_statuses;
    int max_statuses;

    QApplication application;
    DSGui gui;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverstationNode>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
