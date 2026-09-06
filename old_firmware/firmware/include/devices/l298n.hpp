#ifndef L298N_HPP
#define L298N_HPP

#include "base/driver.hpp"
#include "Constants.hpp"
#include <esp32-hal-ledc.h>
#include <esp32-hal-gpio.h>
#include <Arduino.h>
namespace hardware_component{
    
    class L298N : public Driver{

        public:

            /**
             * @brief Creates a new driver object for the L298N motor driver
             * 
             * @param _index Starting index for the PWM channel
             * @param _in1 Input 1 pin
             * @param _in2 Input 2 pin
             */
            L298N(int _index, int _enA, int _in1, int _in2) : Driver(){
                enA = _enA;
                in1 = _in1;
                in2 = _in2;

                c_in1 = _index;
                c_in2 = _index + 1;

            }

            void initialize() override{
                pinMode(enA, OUTPUT);

                //setup pins for pwm control
                ledcSetup(c_in1, PWM_HERTZ, PWM_RESOLUTION);
                ledcSetup(c_in2, PWM_HERTZ, PWM_RESOLUTION);
          
                //attach pins
                ledcAttachPin(in1, c_in1);
                ledcAttachPin(in2, c_in2);
            }

            void set_percent(double request) override{

                //out of bounds protection
                if(request > 1)
                    request = 1;
                else if(request < -1)
                    request = -1;

                if(request > 0){
                    digitalWrite(enA, HIGH);
                    ledcWrite(c_in1, map(abs(request) * 1000, 0, 1000, 0, MAX_DUTY_CYCLE));
                    ledcWrite(c_in2, 0);
                }     
                else if(request < 0){
                    digitalWrite(enA, HIGH);
                    ledcWrite(c_in1, 0);
                    ledcWrite(c_in2, map(abs(request) * 1000, 0, 1000, 0, MAX_DUTY_CYCLE));
                }
                else{
                    digitalWrite(enA, LOW);
                    ledcWrite(c_in1, 0);
                    ledcWrite(c_in2, 0);
                }
            }

            void stop() override{
                set_percent(0);
            }

            void brake(){
                digitalWrite(enA, HIGH);
                ledcWrite(in1, MAX_DUTY_CYCLE);
                ledcWrite(in2, MAX_DUTY_CYCLE);
            }
            

        private:

            int enA, in1, in2;
            int c_in1, c_in2;


    }; // class L298N

} // namespace hardware_component

#endif