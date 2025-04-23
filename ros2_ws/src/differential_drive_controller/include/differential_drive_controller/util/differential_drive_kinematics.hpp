#ifndef DIFFERENTIAL_DRIVE_KINEMATICS
#define DIFFERENTIAL_DRIVE_KINEMATICS

#include "utility.hpp"

using namespace utility;

class DifferentialDriveKinematics{

    private:
        double track_width;

    public:

        DifferentialDriveKinematics(double track_width_) : track_width(track_width_){}

        /**
         * @brief Returns the chassis twist of the robot
         * 
         * @param wheelSpeeds Wheel speeds of the robot
         * @return Twist 
         */
        Twist toChassisSpeeds(WheelSpeeds wheelSpeeds){
            return Twist(
                (wheelSpeeds.left_velocity + wheelSpeeds.right_velocity) / 2,
                0,
                (wheelSpeeds.right_velocity - wheelSpeeds.left_velocity) / track_width
            );
        }

        /**
         * @brief Returns the wheel speeds required to move at the desired chassisSpeeds
         * 
         * @param chassisSpeed Desired chassis twist
         * @return WheelSpeeds 
         */
        WheelSpeeds toWheelSpeeds(Twist chassisSpeed){
            return WheelSpeeds(
                chassisSpeed.dx - (chassisSpeed.dTheta * track_width / 2),
                chassisSpeed.dx + (chassisSpeed.dTheta * track_width / 2)
            );
        }

        WheelSpeeds toWheelSpeedsOpenLoop(Twist chassisSpeed, double turnCoefficient = 1){
            return WheelSpeeds(
                chassisSpeed.dx - (chassisSpeed.dTheta * turnCoefficient),
                chassisSpeed.dx + (chassisSpeed.dTheta * turnCoefficient)
            );
        }

        /**
         * @brief Returns a twist for a difference in initial and final
         * 
         * @param initial Initial wheel positions
         * @param final Final wheel positions
         * @return Twist 
         */
        Twist toTwist(WheelPositions initial, WheelPositions final){
            return toTwist(
                final.left_distance - initial.left_distance, 
                final.right_distance - initial.right_distance
            );   
        }

        /**
         * @brief Returns the twist given how much each wheel has turned
         * 
         * @param left_distance 
         * @param right_distance 
         * @return Twist 
         */
        Twist toTwist(double left_distance, double right_distance){
            return Twist(
                (left_distance + right_distance) / 2,
                0,
                (right_distance - left_distance) / track_width
            );
        }


};

#endif