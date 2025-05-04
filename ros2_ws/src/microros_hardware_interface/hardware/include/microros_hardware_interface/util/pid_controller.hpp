/**
 * @file pid_controller.hpp
 * @author Aaron Yoon (awesomeyooner@gmail.com)
 * @brief PID Controller Implementation
 * @version 0.1
 * @date 2025-05-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "timestamped_value.hpp"
#include <cstdlib>
#include <cmath>
#include "utility.hpp"

enum class FeedForwardType{
    STATIC,
    LINEAR,
    ANGULAR
}; // enum class FeedForwardType

class PIDController{

    public:

        double k_P;
        double k_I;
        double k_D;
        double k_F;
        FeedForwardType ff_type = FeedForwardType::LINEAR;

        bool is_continuous_input_enabled = false;
        double min_input;
        double max_input;

        double i_Zone = INFINITY;

        PIDController(double _p, double _i, double _d, double _f) : prev_error(0, 0){
            k_P = _p;
            k_I = _i;
            k_D = _d;
            k_F = _f;
        }

        void reset(){
            error = 0;
            error_derivative = 0;
            error_integral = 0;

            prev_error.set_value(0);
        }

        void set_setpoint(double new_setpoint){
            setpoint = is_continuous_input_enabled ? utility::input_modulus(new_setpoint, min_input, max_input) : new_setpoint;
        }

        double calculate(double measurement, double time){
            prev_error.update(error, time);

            error = setpoint - measurement;
            error_integral += error;
            error_derivative = prev_error.get_rate();

            switch(ff_type){
                case FeedForwardType::STATIC:
                    feedforward = k_F;
                    break;
                case FeedForwardType::LINEAR:
                    feedforward = k_F * setpoint;
                    break;
                case FeedForwardType::ANGULAR:
                    feedforward = k_F * cos(setpoint);
                    break;
            }

            if(abs(error_integral) > i_Zone)
                error_integral = copysign(i_Zone, error_integral);
            
            return (k_P * error) + (k_I * error_integral) + (k_D * error_derivative) + (k_F * feedforward);
        }

        double calculate(double measurement, double setpoint, double time){
            set_setpoint(setpoint);
            return calculate(measurement, time);
        }

        double get_error(){
            return error;
        }

        double get_error_integral(){
            return error_integral;
        }

        double get_error_derivative(){
            return error_derivative;
        }

        double get_feedforward(){
            return feedforward;
        }

    private:

        TimestampedValue prev_error;

        double error;
        double error_derivative;
        double error_integral;
        double feedforward;    
        
        double setpoint;

}; // class PIDController

#endif