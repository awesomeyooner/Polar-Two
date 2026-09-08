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


using namespace status_utils;
using namespace robot_hardware;
using namespace hardware_interface;
using namespace std;


CallbackReturn RobotSystemHardware::on_init(const HardwareComponentInterfaceParams& params)
{

    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    m_serial.init_field("product", "STM32 Virtual ComPort");

    m_serial.set_timeout_ms(500);

    // Using parameters in the ros2_control.xacro file
    string my_param = info_.hardware_parameters["my_param"];

    return CallbackReturn::SUCCESS;

} // end of "on_init(const HardwareComponentInterfaceParams&)"


vector<StateInterface> RobotSystemHardware::export_state_interfaces()
{
    vector<StateInterface> state_interfaces;

    state_interfaces.emplace_back(StateInterface("left_wheel_joint", HW_IF_POSITION, &m_positions[0]));
    state_interfaces.emplace_back(StateInterface("right_wheel_joint", HW_IF_POSITION, &m_positions[1]));

    state_interfaces.emplace_back(StateInterface("left_wheel_joint", HW_IF_VELOCITY, &m_velocities[0]));
    state_interfaces.emplace_back(StateInterface("right_wheel_joint", HW_IF_VELOCITY, &m_velocities[1]));

    return state_interfaces;

} // end of "export_state_interfaces()"


vector<CommandInterface> RobotSystemHardware::export_command_interfaces()
{
    vector<CommandInterface> command_interfaces;

    command_interfaces.emplace_back(CommandInterface("left_wheel_joint", HW_IF_VELOCITY, &m_targets[0]));
    command_interfaces.emplace_back(CommandInterface("right_wheel_joint", HW_IF_VELOCITY, &m_targets[0]));

    return command_interfaces;

} // end of "export_command_interfaces()"


CallbackReturn RobotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Activating ...please wait...");
    
    if(m_serial.write_data<int>(98, 0, true) != StatusCode::OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Failed to enable device!");

        return CallbackReturn::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Successfully activated!");

    return CallbackReturn::SUCCESS;

} // end of "on_actiate(const rclcpp_lifecycle::State&)"


CallbackReturn RobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Deactivating ...please wait...");

    if(m_serial.write_data<int>(98, 1, true) != StatusCode::OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Failed to disable device!");

        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotSystemHardware"), "Successfully deactivated!");

    return CallbackReturn::SUCCESS;

} // end of "on_deactiate(const rclcpp_lifecycle::State&)"


return_type RobotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto left_angle_read = m_serial.request_data<double>(101, 500);
    m_positions[0] = left_angle_read.value;

    auto left_velocity_read = m_serial.request_data<double>(102, 500);
    m_velocities[0] = left_angle_read.value;
    
    auto right_angle_read = m_serial.request_data<double>(104, 500);
    m_positions[1] = right_angle_read.value;

    auto right_velocity_read = m_serial.request_data<double>(105, 500);
    m_velocities[1] = right_velocity_read.value;

    return return_type::OK;

} // end of "read(const rclcpp::Time&, const rclcpp::Duration&)"


return_type RobotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Write Left Command
    if(m_serial.write_data<double>(100, m_targets[0]) != StatusCode::OK)
        return return_type::ERROR;

    // Write Right Command
    if(m_serial.write_data<double>(103, m_targets[1]) != StatusCode::OK)
        return return_type::ERROR;

    return return_type::OK;

} // end of "write(const rclcpp::Time&, const rclcpp::Duration&)"


// Export this as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotSystemHardware, hardware_interface::SystemInterface)
