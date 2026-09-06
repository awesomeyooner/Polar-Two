#ifndef QUADRATURE_ENCODER_HPP
#define QUADRATURE_ENCODER_HPP

#include <string>
#include "../Constants.hpp"
#include <esp32-hal-ledc.h>
#include <Arduino.h>
#include "../util/Utility.hpp"
#include <map>
#include "esp_timer.h"
#include <FunctionalInterrupt.h>
#include "base/encoder.hpp"

namespace hardware_component{

    class QuadratureEncoder : public Encoder{

        private:
            int channelA, channelB;

            Utility::TimestampedNumber previousPosition;
            Utility::TimestampedNumber positionRaw;

            Utility::TimestampedNumber position;
            Utility::TimestampedNumber velocity;

        public:
            double countsPerRevolution = 1.0;
            double sensorToMechanismRatio = 1.0;
            bool inverted = false;


            QuadratureEncoder(int channelA, int channelB) : 
                channelA(channelA),
                channelB(channelB){
    
                    //set default values for position and velocity so that it doesn't freak out initially
                    double time = esp_timer_get_time() / 1000000;
                    previousPosition.timestamp = time;
                    position.timestamp = time;
                    velocity.timestamp = time;
            }

            void initialize() override{
                pinMode(channelA, INPUT_PULLDOWN);
                pinMode(channelB, INPUT_PULLDOWN);

                attachInterrupt(digitalPinToInterrupt(channelA), [this](){handleA();}, CHANGE);
                attachInterrupt(digitalPinToInterrupt(channelB), [this](){handleB();}, CHANGE);
            }

            double get_position() override{
                return position.value; 
            }

            double get_velocity() override{
                return velocity.value;
            }

            void handleA(){
                if(digitalRead(channelA) == HIGH){
                    if(digitalRead(channelB) == LOW)
                        positionRaw.value += 1;
                    else if (digitalRead(channelB) == HIGH)
                        positionRaw.value -= 1;
                }

                else if(digitalRead(channelA) == LOW){
                    if(digitalRead(channelB) == HIGH)
                        positionRaw.value += 1;
                    else if (digitalRead(channelB) == LOW)
                        positionRaw.value -= 1;
                }
            }

            void handleB(){
                if(digitalRead(channelB) == HIGH){
                    if(digitalRead(channelA) == HIGH)
                        positionRaw.value += 1;
                    else if (digitalRead(channelA) == LOW)
                        positionRaw.value -= 1;
                }

                else if(digitalRead(channelB) == LOW){
                    if(digitalRead(channelA) == LOW)
                        positionRaw.value += 1;
                    else if (digitalRead(channelA) == HIGH)
                        positionRaw.value -= 1;
                }
            }

            /**
             * this NEEDS some sort of delay for it to work properly
             */
            void update() override{
                double time = (double)esp_timer_get_time() / 1000000.0;
                
                //counts / cpr = motor rotations
                //motor rotations * sensorToMechRatio = total
                double invertedCoef = inverted ? -1 : 1;

                position.value = invertedCoef * 2 * M_PI * (positionRaw.value / countsPerRevolution) / sensorToMechanismRatio;
                position.timestamp = time;

                positionRaw.timestamp = time;

                velocity.value = invertedCoef * 2 * M_PI * (positionRaw.getRate(previousPosition) / countsPerRevolution) / sensorToMechanismRatio;
                velocity.timestamp = time;

                previousPosition = positionRaw;
            }
  
    };
}

#endif