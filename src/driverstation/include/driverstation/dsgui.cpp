#include "dsgui.h"
#include "ui_dsgui.h"

DSGui::DSGui(QWidget *parent) : QWidget(parent), ui(new Ui::DSGui){
    ui->setupUi(this);
}

DSGui::~DSGui(){
    delete ui;
}

DSGui::initialize(rclcpp::Node *node, int argc, char * argv[]){
    subscription = node->create_subscription<driverstation_msgs::msg::DriverstationStatus>(
        "console", 10, std::bind(&DSGui::topic_callback, this, _1));
}

DSGui::topic_callback(const driverstation_msgs::msg::DriverstationStatus &message) {
    // RCLCPP_INFO(this->get_logger(), message.message.c_str());

    recent_statuses.insert(recent_statuses.begin(), message);
    
    //if the current size is greater than the max, then start removing
    if(recent_statuses.size() > max_statuses){
      //for every item thats over the limit, remove it
      for(int i = 0; i < recent_statuses.size() - max_statuses; i++){
        recent_statuses.pop_back();
      }
    }
    
}


