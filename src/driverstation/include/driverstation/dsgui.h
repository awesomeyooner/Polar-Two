#ifndef DSGUI_H
#define DSGUI_H

#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <driverstation_msgs/msg/driverstation_status.hpp>

using std::placeholders::_1;
namespace Ui {
    class DSGui;
}
class DSGui : public QWidget{
    Q_OBJECT

public:
    explicit DSGui(QWidget *parent = nullptr);
    ~DSGui();

    virtual int initialize(rclcpp::Node *node, int argc, char * argv[]);

private:

    void topic_callback(const driverstation_msgs::msg::DriverstationStatus &message);

    Ui::DSGui *ui;

    rclcpp::Subscription<driverstation_msgs::msg::DriverstationStatus>::SharedPtr subscription;

    std::vector<driverstation_msgs::msg::DriverstationStatus> recent_statuses;
    int max_statuses;
};

#endif // DSGUI_H
