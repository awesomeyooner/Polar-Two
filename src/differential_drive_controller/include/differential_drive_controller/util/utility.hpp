#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace utility{

    struct WheelHandle{
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;

        void set_command(double desired){
            command.get().set_value(desired);
        }

        static void set_command(double desired, std::vector<WheelHandle> wheel_handles){
            for(int i = 0; i < wheel_handles.size(); i++){
                wheel_handles[i].set_command(desired);
            }
        }

        double get_position(){
            return position.get().get_value();
        }

        static double get_position(std::vector<WheelHandle> wheel_handles){
            double total = 0;

            int num_wheels = static_cast<int>(wheel_handles.size());

            for(WheelHandle wheel : wheel_handles){
                total += wheel.get_position();
            }

            return total / num_wheels;
        }

        double get_velocity(){
            return velocity.get().get_value();
        }

        static double get_velocity(std::vector<WheelHandle> wheel_handles){
            double total = 0;

            int num_wheels = static_cast<int>(wheel_handles.size());

            for(WheelHandle wheel : wheel_handles){
                total += wheel.get_velocity();
            }

            return total / num_wheels;
        }
    };

    struct WheelPositions{
        double left_distance;
        double right_distance;

        WheelPositions(double left_dist, double right_dist) : left_distance(left_dist), right_distance(right_dist){}
        WheelPositions() : left_distance(0), right_distance(0){}
    };  // struct WheelPositions

    struct WheelSpeeds{
        double left_velocity;
        double right_velocity;

        WheelSpeeds(double left_velo, double right_velo) : left_velocity(left_velo), right_velocity(right_velo){}
        WheelSpeeds() : left_velocity(0), right_velocity(0){}

        WheelSpeeds times(double scale){
            left_velocity *= scale;
            right_velocity *= scale;

            return *this;
        }
    };  // struct WheelSpeeds

    struct Twist{
        double dx;
        double dy;
        double dTheta;

        Twist(double dx_, double dy_, double dTheta_) : dx(dx_), dy(dy_), dTheta(dTheta_){}
        Twist() : dx(0), dy(0), dTheta(0){}

        geometry_msgs::msg::Twist to_message(){
            geometry_msgs::msg::Twist twist;
            twist.linear.x = dx;
            twist.linear.y = dy;
            twist.angular.z = dTheta;

            return twist;
        }

        static Twist from_message(geometry_msgs::msg::TwistStamped message){
            return Twist(message.twist.linear.x, 0, message.twist.angular.z);
        }
    };  // struct Twist

    //pretty much just a vector
    struct Translation{
        double x;
        double y;

        Translation(double x_, double y_) : x(x_), y(y_){}
        Translation() : x(0), y(0){}

        geometry_msgs::msg::Vector3 to_message(){
            geometry_msgs::msg::Vector3 vector;

            vector.x = x;
            vector.y = y;
            vector.z = 0;

            return vector;
        }

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
    };  // struct Translation

    struct Transform{
        double x;
        double y;
        double theta;

        Transform(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_){}
        Transform() : x(0), y(0), theta(0){}

        Translation getTranslation(){
            return Translation(x, y);
        }


    };  // struct Transform

    struct Pose2d{
        double x;
        double y;
        double theta;

        Pose2d(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_){}
        Pose2d(Translation translation, double theta_) : x(translation.x), y(translation.y), theta(theta_){}
        Pose2d() : x(0), y(0), theta(0){}

        geometry_msgs::msg::Point get_point(){
            geometry_msgs::msg::Point point;

            point.x = x;
            point.y = y;
            point.z = 0;

            return point;
        }

        geometry_msgs::msg::Quaternion get_quaternion(){
            tf2::Quaternion quaternion;

            quaternion.setRPY(0, 0, theta);

            geometry_msgs::msg::Quaternion native;

            native.w = quaternion.getW();
            native.x = quaternion.getX();
            native.y = quaternion.getY();
            native.z = quaternion.getZ();

            return native;
        }

        Translation getTranslation(){
            return Translation(x, y);
        }

        Pose2d transformBy(Transform transform){
            return Pose2d(
                getTranslation().plus(transform.getTranslation().rotateBy(theta)),
                theta + transform.theta
            );
        }

        Pose2d exponential(Twist twist){
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

            return Pose2d(x, y, theta).transformBy(transform);
        }
    };  // struct Pose2d
}   //namespace utility

#endif