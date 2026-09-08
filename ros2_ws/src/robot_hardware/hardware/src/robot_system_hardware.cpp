#include "robot_hardware/robot_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace robot_hardware;
using namespace hardware_interface;
using namespace std;


CallbackReturn RobotSystemHardware::on_init(const HardwareComponentInterfaceParams& params)
{

    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // Using parameters in the ros2_control.xacro file
    string my_param = info_.hardware_parameters["my_param"];

    if(init_status != StatusCode::OK)
        return CallbackReturn::ERROR;

    return CallbackReturn::SUCCESS;

} // end of "on_init(const HardwareComponentInterfaceParams&)"


vector<StateInterface> RobotSystemHardware::export_state_interfaces()
{
    vector<StateInterface> state_interfaces;

    // Code here...

    return state_interfaces;

} // end of "export_state_interfaces()"


vector<CommandInterface> RobotSystemHardware::export_command_interfaces()
{
    vector<CommandInterface> command_interfaces;

    // Code here...

    return command_interfaces;

} // end of "export_command_interfaces()"


CallbackReturn RobotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Activating ...please wait...");
    
    // Code here...
    
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Successfully activated!");

    return CallbackReturn::SUCCESS;

} // end of "on_actiate(const rclcpp_lifecycle::State&)"


CallbackReturn RobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Deactivating ...please wait...");

    // Code here...

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Successfully deactivated!");

    return CallbackReturn::SUCCESS;

} // end of "on_deactiate(const rclcpp_lifecycle::State&)"


return_type RobotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Code here...

    return return_type::OK;

} // end of "read(const rclcpp::Time&, const rclcpp::Duration&)"


return_type RobotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Code here...

    return return_type::OK;

} // end of "write(const rclcpp::Time&, const rclcpp::Duration&)"


// Export this as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotSystemHardware, hardware_interface::SystemInterface)
