#ifndef DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP
#define DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP

#include "utility.hpp"
#include "differential_drive_odometry.hpp"

using namespace utility;

class DifferentialDriveDrivetrain{

    public:

        DifferentialDriveOdometry odometry;

        std::vector<WheelHandle> left_wheels;
        std::vector<WheelHandle> right_wheels;

        double wheel_radius;

        DifferentialDriveDrivetrain(double track_width_, double wheel_radius_) : 
            odometry(track_width_), wheel_radius(wheel_radius_){}
    
        void initialize(std::vector<WheelHandle>& left, std::vector<WheelHandle>& right ){
            left_wheels = left;
            right_wheels = right;
        }

        void update(){
            double left_positions = get_left_positions();
            double right_positions = get_right_positions();

            odometry.update(left_positions, right_positions);
        }

        // get members
        DifferentialDriveOdometry* get_odometry(){
            return &odometry;
        }

        DifferentialDriveKinematics* get_kinematics(){
            return odometry.get_kinematics();
        }

        // get positions
        double get_left_positions(){
            return WheelHandle::get_position(left_wheels) * wheel_radius;
        }

        double get_right_positions(){
            return WheelHandle::get_position(right_wheels) * wheel_radius;
        }

        WheelPositions get_wheel_positions(){
            return WheelPositions(get_left_positions(), get_right_positions());
        }

        // get velocities
        double get_left_velocities(){
            return WheelHandle::get_velocity(left_wheels) * wheel_radius;
        }

        double get_right_velocities(){
            return WheelHandle::get_velocity(right_wheels) * wheel_radius;
        }

        WheelSpeeds get_wheel_speeds(){
            return WheelSpeeds(get_left_velocities(), get_right_velocities());
        }

        Twist to_chassis_speed(){
            return get_kinematics()->toChassisSpeeds(get_wheel_speeds());
        }

        // controls
        void drive_from_chassis(Twist desired, bool is_open_loop = false){
            //meters per second -> radians per second
            WheelSpeeds wheel_speeds = is_open_loop ? 
                odometry.get_kinematics()->toWheelSpeedsOpenLoop(desired) : 
                odometry.get_kinematics()->toWheelSpeeds(desired).times(1 / (wheel_radius));

            //set the wheels to the speeds
            set_left_command(wheel_speeds.left_velocity);
            set_right_command(wheel_speeds.right_velocity);
        }

        /**
         * @brief Drives with values of the twist as percentages (-1, 1)
         * 
         * @param desired Desired chassis speed, each value represented as a percent from (-1, 1)
         */
        void drive_open_loop(Twist desired){
            double linear = desired.dx;
            double angular = desired.dTheta;

            double angular_coef;

            double left_speed = linear - (angular * angular_coef);
            double right_speed = linear + (angular * angular_coef);

            set_left_command(left_speed);
            set_right_command(right_speed);
        }

        void stop(){
            set_left_command(0);
            set_right_command(0);
        }

        /**
         * @brief Sets the speed in RPS
         * 
         * @param command Desired speed in RPS
         */
        void set_left_command(double command){
            WheelHandle::set_command(command, left_wheels);
        }

        /**
         * @brief Set the speed in RPS
         * 
         * @param command Desired speed in RPS
         */
        void set_right_command(double command){
            WheelHandle::set_command(command, right_wheels);
        }

    private:



};

#endif //DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP