#ifndef DIFFERENTIAL_DRIVE_ODOMETRY_HPP
#define DIFFERENTIAL_DRIVE_ODOMETRY_HPP

#include "Utility.hpp"
#include "DifferentialDriveKinematics.hpp"
#include <cmath>

using namespace Utility;

class DifferentialDriveOdometry{

    private:
        DifferentialDriveKinematics kinematics;
        WheelPositions previousWheelPositions;
        Pose pose;

    public:
        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics) : kinematics(kinematics){

        }

        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics, Pose initialPose) : kinematics(kinematics), pose(initialPose){

        }

        Pose getPose(){
            return pose;
        }

        void setPose(Pose newPose){
            pose = newPose;
        }

        void setPose(Pose newPose, WheelPositions WheelPositions){
            pose = newPose;
            previousWheelPositions = WheelPositions;
        }

        void update(WheelPositions wheelPositions){
            Twist twist = kinematics.toTwist(previousWheelPositions, wheelPositions);

            Pose newPose = pose.exponential(twist);

            pose = newPose;
        }
};

#endif