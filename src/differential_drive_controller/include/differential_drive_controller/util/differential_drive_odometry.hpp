#ifndef DIFFERENTIAL_DRIVE_ODOMETRY_HPP
#define DIFFERENTIAL_DRIVE_ODOMETRY_HPP

#include "utility.hpp"
#include "differential_drive_kinematics.hpp"
#include <cmath>
#include <vector>
#include "../differential_drive_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace utility;

class DifferentialDriveOdometry{

    private:
        DifferentialDriveKinematics kinematics;
        WheelPositions previous_wheel_positions;
        Pose2d pose;

    public:

        DifferentialDriveOdometry(double track_width) : kinematics(track_width){}
        DifferentialDriveOdometry(double track_width, Pose2d initial_pose) : kinematics(track_width), pose(initial_pose){}

        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics_) : kinematics(kinematics_){}
        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics_, Pose2d initialPose) : kinematics(kinematics_), pose(initialPose){}

        DifferentialDriveKinematics* get_kinematics(){
            return &kinematics;
        }

        Pose2d get_pose(){
            return pose;
        }

        void set_pose(Pose2d new_pose){
            pose = new_pose;
        }

        void set_pose(Pose2d new_pose, WheelPositions wheel_positions){
            pose = new_pose;
            previous_wheel_positions = wheel_positions;
        }

        void update(WheelPositions current_wheel_positions){
            Twist twist = kinematics.toTwist(previous_wheel_positions, current_wheel_positions);

            Pose2d new_pose = pose.exponential(twist);

            pose = new_pose;
        }

        void update(double current_left, double current_right){
            WheelPositions current_wheel_positions(current_left, current_right);
            update(current_wheel_positions);
        }
};

#endif