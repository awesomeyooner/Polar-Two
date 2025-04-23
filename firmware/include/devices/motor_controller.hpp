#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>

#include "motor.hpp"
#include "quadrature_encoder.hpp"
#include "l298n.hpp"
#include "../util/Utility.hpp"
#include "../Constants.hpp"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

namespace hardware_component{

    class MotorController{

        public:

            std::string name;

            rcl_publisher_t positionPublisher, velocityPublisher;
            std_msgs__msg__Float64 positionMessage, velocityMessage, commandMessage;

            rcl_subscription_t commandSubscriber;


            MotorController(std::string name, int index, int enA, int in1, int in2, int chA, int chB) : 
                name(name),
                motor(false),
                encoder(chA, chB),
                driver(index, enA, in1, in2){
                    commandMessage.data = 0;
                }

            MotorController(std::string name, int index, SensoredMotorID id) : 
                MotorController(name, index, id.enA, id.in1, id.in2, id.chA, id.chB){}

            void initialize(rcl_node_t* node, rclc_executor_t* executor){
                motor.link_driver(&driver);
                motor.link_encoder(&encoder);
                motor.initialize();

                RCCHECK(rclc_publisher_init_best_effort(
                    &positionPublisher,
                    node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                    (name + "/position").c_str()
                ));

                RCCHECK(rclc_publisher_init_best_effort(
                    &velocityPublisher,
                    node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                    (name + "/velocity").c_str()
                ));

                RCCHECK(rclc_subscription_init_best_effort(
                    &commandSubscriber,
                    node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                    (name + "/command").c_str()
                ));

                RCCHECK(rclc_executor_add_subscription(
                    executor, 
                    &commandSubscriber, 
                    &commandMessage, 
                    [](const void * msgin) -> void{}, 
                    ON_NEW_DATA
                ));
            }

            void update(bool enabled){
                motor.update(enabled);

                positionMessage.data = encoder.get_position();
                velocityMessage.data = encoder.get_velocity();

                motor.set_command(commandMessage.data);

                RCSOFTCHECK(rcl_publish(&positionPublisher, &positionMessage, NULL));
                RCSOFTCHECK(rcl_publish(&velocityPublisher, &velocityMessage, NULL));
            }

            int getNumberOfHandles(){
                return 1; // subscriber
            }

            private:
                Motor motor;
                QuadratureEncoder encoder;
                L298N driver;

    }; // class MotorController
} // namespace hardware_component 

#endif