#ifndef DIFFERENTIAL_DRIVE_KINEMATICS
#define DIFFERENTIAL_DRIVE_KINEMATICS

#include "Utility.hpp"

using namespace Utility;

class DifferentialDriveKinematics{

    private:
        double trackWidth;

    public:

        DifferentialDriveKinematics(double trackWidth) : trackWidth(trackWidth){}

        Twist toChassisSpeeds(WheelSpeeds wheelSpeeds){
            return Twist(
                (wheelSpeeds.leftVelocity + wheelSpeeds.rightVelocity) / 2,
                0,
                (wheelSpeeds.rightVelocity - wheelSpeeds.leftVelocity) / trackWidth
            );
        }

        WheelSpeeds toWheelSpeeds(Twist chassisSpeed){
            return WheelSpeeds(
                chassisSpeed.dx - (chassisSpeed.dTheta * trackWidth / 2),
                chassisSpeed.dx + (chassisSpeed.dTheta * trackWidth / 2)
            );
        }

        Twist toTwist(WheelPositions initial, WheelPositions final){
            return toTwist(
                final.leftDistance - initial.leftDistance, 
                final.rightDistance - initial.rightDistance
            );   
        }

        Twist toTwist(double leftDistance, double rightDistance){
            return Twist(
                (leftDistance + rightDistance) / 2,
                0,
                (rightDistance - leftDistance) / trackWidth
            );
        }


};

#endif