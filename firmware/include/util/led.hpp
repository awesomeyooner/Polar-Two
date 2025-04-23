#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

class LED{

    private:
        int pin;

    public:

        LED(int _pin) : pin(_pin){}

        void initialize(){
            pinMode(pin, OUTPUT);
        }

        void toggle(bool turnOn){
            turnOn ? turn_on() : turn_off();
        }

        void turn_on(){
            digitalWrite(pin, HIGH);
        }

        void turn_off(){
            digitalWrite(pin, LOW);
        }
};


#endif