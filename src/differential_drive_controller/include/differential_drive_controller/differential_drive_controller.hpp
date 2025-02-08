#ifndef DIFFERENTIAL_DRIVE_CONTROLLER_HPP
#define DIFFERENTIAL_DRIVE_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "visibility_control.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "utility/DifferentialDriveKinematics.hpp"
// #include "utility/DifferentialDriveOdometry.hpp"
#include <differential_drive_controller/differential_drive_controller_parameters.hpp>

namespace differential_drive_controller{

    class DifferentialDriveController : public controller_interface::ControllerInterface{
        
        public:
            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            DifferentialDriveController();

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_init() override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

            DIFFERENTIAL_DRIVE_CONTROLLER_PUBLIC
            controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

            protected:

                struct WheelHandle{
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
                    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
                    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
                };

                const char* feedback_type() const;

                controller_interface::CallbackReturn configure_side(
                    const std::vector<std::string>& wheel_names,
                    std::vector<WheelHandle>& registered_handles
                );

                std::shared_ptr<ParamListener> param_listener;
                Params params;     

                std::vector<WheelHandle> registered_left_wheel_handles;
                std::vector<WheelHandle> registered_right_wheel_handles;

                //wheel info
                int wheels_per_side;
                double wheel_separation;
                double wheel_radius;

                // std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher = nullptr;
                // std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odom_publisher = nullptr;

                // std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odom_transform_publisher = nullptr;
                // std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_transform_publisher = nullptr;

                rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscriber = nullptr;

                // realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::TwistStamped>> recievedVelocityMessagePtr{nullptr};
                std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command;
                // std::queue<geometry_msgs::msg::TwistStamped> previousCommands;
    };   
}

#endif