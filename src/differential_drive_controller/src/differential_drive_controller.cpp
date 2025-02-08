#include "../include/differential_drive_controller/differential_drive_controller.hpp"

namespace{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}

namespace differential_drive_controller{

    DifferentialDriveController::DifferentialDriveController() : controller_interface::ControllerInterface(){}

    controller_interface::CallbackReturn DifferentialDriveController::on_init(){
        try{
            // Create the parameter listener and get the parameters
            param_listener = std::make_shared<ParamListener>(get_node());
            params = param_listener->get_params();
        }
        catch (const std::exception & e){
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DifferentialDriveController::command_interface_configuration() const{

        std::vector<std::string> config_names;

        for(const std::string& joint_name : params.left_wheel_names){
            config_names.push_back(joint_name + "/" + params.command_type);
        }

        for(const std::string& joint_name : params.right_wheel_names){
            config_names.push_back(joint_name + "/" + params.command_type);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, config_names};
    }

    controller_interface::InterfaceConfiguration DifferentialDriveController::state_interface_configuration() const{

        std::vector<std::string> config_names;

        for(const std::string& feedback_type : params.feedback_types){
            for(const std::string& joint_name : params.left_wheel_names){
                config_names.push_back(joint_name + "/" + feedback_type);
            }

            for(const std::string& joint_name : params.right_wheel_names){
                config_names.push_back(joint_name + "/" + feedback_type);
            }

        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, config_names};
    }

    controller_interface::return_type DifferentialDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {

        if(!last_command)
            return controller_interface::return_type::ERROR;

        double linear = last_command->twist.linear.x;
        double angular = last_command->twist.angular.z;

        double left_command = (linear - (angular * (wheel_separation / 2))) / wheel_radius;
        double right_command = (linear + (angular * (wheel_separation / 2))) / wheel_radius;

        for(int i = 0; i < wheels_per_side; i++){
            registered_left_wheel_handles[i].command.get().set_value(left_command);
            registered_right_wheel_handles[i].command.get().set_value(right_command);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_configure(const rclcpp_lifecycle::State & previous_state){
        //param stuff

        wheels_per_side = (int)params.wheels_per_side;
        wheel_separation = params.wheel_separation;
        wheel_radius = params.wheel_radius;

        command_subscriber = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            params.command_topic, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> message){
                last_command = message;
            }
        );

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_activate(const rclcpp_lifecycle::State & previous_state){
        //configure each handle

        const controller_interface::CallbackReturn left_status = configure_side(params.left_wheel_names, registered_left_wheel_handles);
        const controller_interface::CallbackReturn right_status = configure_side(params.right_wheel_names, registered_right_wheel_handles);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        //clear handle
        registered_left_wheel_handles.clear();
        registered_right_wheel_handles.clear();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
        return controller_interface::CallbackReturn::SUCCESS;
    }   

    controller_interface::CallbackReturn DifferentialDriveController::on_error(const rclcpp_lifecycle::State & previous_state){
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_shutdown(const rclcpp_lifecycle::State & previous_state){
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::configure_side( 
        const std::vector<std::string>& wheel_names,
        std::vector<WheelHandle>& registered_handles){

        registered_handles.reserve(wheel_names.size());

        for(const std::string& wheel_name : wheel_names){
            
            const auto velocity_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                }
            );

            const auto position_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                }
            );

            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [this, &wheel_name](const auto & interface){
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == params.command_type;
                }
            );

            registered_handles.emplace_back(
                WheelHandle{std::ref(*velocity_handle), std::ref(*position_handle), std::ref(*command_handle)}
            );
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  differential_drive_controller::DifferentialDriveController, controller_interface::ControllerInterface)