#ifndef BUILTIN_LED_HPP
#define BUILTIN_LED_HPP

#include <Arduino.h>
#include "led.hpp"

#define LED_PIN 2

class BuiltinLED : public LED{

    private:

    public:

        BuiltinLED() : LED(LED_PIN){}
};


#endif