#ifndef HARDWAREMANAGER_HPP
#define HARDWAREMANAGER_HPP

#include "devices/motor_controller.hpp"
#include "util/builtin_led.hpp"
#include "Constants.hpp"
#include <vector>

namespace hardware_component{

    class HardwareManager{
        
        public:

            HardwareManager() : 
                left_motor("left_motor", MotorConstants::INDEX_LEFT_MOTOR, MotorConstants::LEFT_MOTOR_ID),
                right_motor("right_motor", MotorConstants::INDEX_RIGHT_MOTOR, MotorConstants::RIGHT_MOTOR_ID){}

            void initialize(rcl_node_t* node, rclc_executor_t* executor){

                for(MotorController* motor : motors){
                    motor->initialize(node, executor);
                }

                status_light.initialize();
            }

            void update(){
                for(MotorController* motor : motors){
                    motor->update(enabled);
                }

                if(enabled)
                    status_light.turn_on();
                else
                    status_light.turn_off();
            }

            void toggleEnabled(bool toggle){
                enabled = toggle;
            }

            bool isEnabled(){
                return enabled; 
            }

            int getNumberOfHandles(){
                int sum = 1; // 1 for the timer

                for(MotorController* motor : motors){
                    sum += motor->getNumberOfHandles();
                }
                return sum;
            }

        private:
            MotorController left_motor;
            MotorController right_motor;

            BuiltinLED status_light;

            std::vector<MotorController*> motors = {
                &left_motor,
                &right_motor
            };

            bool enabled = true;
    };

}

#endif