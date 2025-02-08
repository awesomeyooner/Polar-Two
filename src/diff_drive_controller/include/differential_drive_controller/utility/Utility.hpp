#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <cmath>

namespace Utility{

    struct WheelPositions{
        double leftDistance;
        double rightDistance;

        WheelPositions(double leftDistance, double rightDistance) : leftDistance(leftDistance), rightDistance(rightDistance){}
        WheelPositions() : leftDistance(0), rightDistance(0){}
    };

    struct WheelSpeeds{
        double leftVelocity;
        double rightVelocity;

        WheelSpeeds(double leftVelocity, double rightVelocity) : leftVelocity(leftVelocity), rightVelocity(rightVelocity){}
        WheelSpeeds() : leftVelocity(0), rightVelocity(0){}
    };

    struct Twist{
        double dx;
        double dy;
        double dTheta;

        Twist(double dx, double dy, double dTheta) : dx(dx), dy(dy), dTheta(dTheta){}
        Twist() : dx(0), dy(0), dTheta(0){}
    };

    //pretty much just a vector
    struct Translation{
        double x;
        double y;

        Translation(double x, double y) : x(x), y(y){}
        Translation() : x(0), y(0){}

        Translation rotateBy(double rotation){
            return Translation(
                (x * cos(rotation)) - (y * sin(rotation)),
                (x * sin(rotation)) + (y * cos(rotation))
            );
        }

        Translation plus(Translation other){
            return Translation(
                x + other.x,
                y + other.y
            );
        }
    };

    struct Transform{
        double x;
        double y;
        double theta;

        Transform(double x, double y, double theta) : x(x), y(y), theta(theta){}
        Transform() : x(0), y(0), theta(0){}

        Translation getTranslation(){
            return Translation(x, y);
        }
    };

    struct Pose{
        double x;
        double y;
        double theta;

        Pose(double x, double y, double theta) : x(x), y(y), theta(theta){}
        Pose(Translation translation, double theta) : x(translation.x), y(translation.y), theta(theta){}
        Pose() : x(0), y(0), theta(0){}

        Translation getTranslation(){
            return Translation(x, y);
        }

        Pose transformBy(Transform transform){
            return Pose(
                getTranslation().plus(transform.getTranslation()).rotateBy(theta),
                theta + transform.theta
            );
        }

        Pose exponential(Twist twist){
            double dx = twist.dx;
            double dy = twist.dy;
            double dTheta = twist.dTheta;

            double sinTheta = sin(dTheta);
            double cosTheta = cos(dTheta);

            double sinExpression;
            double cosExpression;

            if(abs(dTheta) < 1E-9){ //if its too small, use taylor
                sinExpression = 1.0 - (pow(dTheta, 2) / 6.0);
                cosExpression = dTheta / 2;
            }
            else{ //if its big enough, use the actual equation
                sinExpression = sinTheta / dTheta;
                cosExpression = (1 - cosTheta) / dTheta;
            }

            //twist vector times robot relaltive rotation vector
            Transform transform(
                (dx * sinExpression) - (dy * cosExpression), 
                (dx * cosExpression) + (dy * sinExpression), 
                dTheta
            );

            return Pose(x, y, theta).transformBy(transform);
        }
    };
}

#endif