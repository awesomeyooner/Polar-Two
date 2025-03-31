#include "../include/differential_drive_controller/differential_drive_controller.hpp"
// #include "differential_drive_controller/include/differential_drive_controller/differential_drive_controller.hpp"

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

        //get_node()->get_clock()->now();

        // if(!last_command)
        //     return controller_interface::return_type::ERROR;

        last_command = std::make_shared<geometry_msgs::msg::TwistStamped>();

        last_command.get()->twist.angular.z = 3.14;
        // send command
        drivetrain->drive_from_chassis(Twist::from_message(*last_command.get()));

        // update odometry
        drivetrain->update();

        // publish messages
        Pose2d pose = drivetrain->get_odometry()->get_pose();
        Twist twist = drivetrain->to_chassis_speed();

        std_msgs::msg::Header header;
        header.frame_id = odom_frame_id;
        header.stamp = time;

        //transform message
        geometry_msgs::msg::TransformStamped transform_message;

            transform_message.header = header;
            transform_message.child_frame_id = base_frame_id;

            transform_message.transform.translation = pose.getTranslation().to_message();
            transform_message.transform.rotation = pose.get_quaternion();

        tf_broadcaster->sendTransform(transform_message);

        //odometry message
        nav_msgs::msg::Odometry odometry_message;

            odometry_message.header = header;
            odometry_message.child_frame_id = base_frame_id;

            odometry_message.pose.pose.orientation = pose.get_quaternion();
            odometry_message.pose.pose.position = pose.get_point();
            odometry_message.twist.twist = twist.to_message();
        
        odom_publisher->publish(odometry_message);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_configure(const rclcpp_lifecycle::State & previous_state){
        //param stuff

        //set constants to the ones specified in the params
        wheels_per_side = (int)params.wheels_per_side;
        wheel_separation = params.wheel_separation;
        wheel_radius = params.wheel_radius;

        odom_frame_id = params.odom_frame_id;
        base_frame_id = params.base_frame_id;

        //create the odometry object with the given wheel_separation
        drivetrain = std::make_shared<DifferentialDriveDrivetrain>(wheel_separation, wheel_radius);

        //create publishers and subscribers
        command_subscriber = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            params.command_topic, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> message){
                last_command = message;
            }
        );

        odom_publisher = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
        
        // odom_transform_publisher = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        //     DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DifferentialDriveController::on_activate(const rclcpp_lifecycle::State & previous_state){
        //configure each handle

        const controller_interface::CallbackReturn left_status = configure_side(params.left_wheel_names, registered_left_wheel_handles);
        const controller_interface::CallbackReturn right_status = configure_side(params.right_wheel_names, registered_right_wheel_handles);

        //assign wheels to drivetrain
        drivetrain->initialize(registered_left_wheel_handles, registered_right_wheel_handles);

        //if both are good, then its a success
        if(left_status == controller_interface::CallbackReturn::SUCCESS && right_status == controller_interface::CallbackReturn::SUCCESS)
            return controller_interface::CallbackReturn::SUCCESS;
        //if either fail, throw an error
        else
            return controller_interface::CallbackReturn::ERROR;
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
        std::vector<utility::WheelHandle>& registered_handles){

        registered_handles.reserve(wheel_names.size());

        // for reach wheel in wheel_names
        for(const std::string& wheel_name : wheel_names){
            
            // create the velocity handle
            const auto velocity_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                }
            );

            // create the position handle
            const auto position_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                }
            );

            // create the command handle
            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [this, &wheel_name](const auto & interface){
                    return interface.get_prefix_name() == wheel_name &&
                        interface.get_interface_name() == params.command_type;
                }
            );

            // add this WheelHandle to the list of all wheel handles
            registered_handles.emplace_back(
                utility::WheelHandle{std::ref(*velocity_handle), std::ref(*position_handle), std::ref(*command_handle)}
            );
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  differential_drive_controller::DifferentialDriveController, controller_interface::ControllerInterface)