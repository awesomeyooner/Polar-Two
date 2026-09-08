#ifndef ROBOT_SYSTEM_HARDWARE_HPP
#define ROBOT_SYSTEM_HARDWARE_HPP


#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robot_hardware/visibility_control.h"

#include "CommiFaceLib/protocols/serial.hpp"


namespace robot_hardware
{    

    
class RobotSystemHardware : public hardware_interface::SystemInterface
{

    public:

        RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystemHardware)

        ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

        ROBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROBOT_HARDWARE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        ROBOT_HARDWARE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        ROBOT_HARDWARE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        ROBOT_HARDWARE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:

        SerialInterface m_serial;

        double m_targets[2] = {0, 0};
        double m_positions[2] = {0, 0};
        double m_velocities[2] = {0, 0};


}; // class RobotSystemHardware


}  // namespace carbot_hardware


#endif // ROBOT_SYSTEM_HARDWARE_HPP
