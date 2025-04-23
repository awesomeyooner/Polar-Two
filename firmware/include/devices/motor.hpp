#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>
#include "../Constants.hpp"
#include <esp32-hal-ledc.h>
#include <Arduino.h>
#include "l298n.hpp"
#include "quadrature_encoder.hpp"

namespace hardware_component{

    class Motor{

        public:

            Driver *driver = nullptr;
            Encoder *encoder = nullptr;

            bool inverted = false;
            double command = 0;

            Motor(bool fake){
                if(fake){
                    driver = new Driver();
                    encoder = new Encoder();
                }
            }

            void link_driver(Driver* _driver){
                driver = _driver;
            }

            void link_encoder(Encoder* _encoder){
                encoder = _encoder;
            }

            void initialize(){
                driver->initialize();
                encoder->initialize();
            }

            void update(bool enabled){
               encoder->update();

                if(enabled)
                    set_percent(command);
                else
                    stop();
            }

            void stop(){
                driver->stop();
            }

            void set_command(double request){
                command = request;
            }

        private:

            void set_percent(double request){
                request *= inverted ? -1 : 1; 

                driver->set_percent(request);
            }

    }; // class Motor
} // namespace hardware_component

#endif