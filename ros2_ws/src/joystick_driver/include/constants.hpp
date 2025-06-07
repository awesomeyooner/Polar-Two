#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "gamepad_interface.hpp"

namespace hid_devices{

    namespace PS4{
        enum{
            X = GamepadButton::ACTION_DOWN,
            CIRCLE = GamepadButton::ACTION_RIGHT,
            SQUARE = GamepadButton::ACTION_LEFT,
            TRIANGLE = GamepadButton::ACTION_UP,

            LEFT_BUMPER = GamepadButton::LEFT_BUMPER,
            RIGHT_BUMPER = GamepadButton::RIGHT_BUMPER,

            LEFT_STICK = GamepadButton::LEFT_STICK,
            RIGHT_STICK = GamepadButton::RIGHT_STICK
        };

        const GamepadMapping MAP{
            .stick_left_x = 0,
            .stick_left_y = 1,
            .stick_right_x = 3,
            .stick_right_y = 4,

            .trigger_left = 2,
            .trigger_right = 5,

            .bumper_left = 5,
            .bumper_right = 6,

            .button_down = 0,
            .button_right = 1,
            .button_left = 3,
            .button_up = 2,

            .button_left_stick = 11,
            .button_right_stick = 12
        };
    };

    namespace GameSir{
        enum{
            A = GamepadButton::ACTION_DOWN,
            B =  GamepadButton::ACTION_RIGHT,
            X = GamepadButton::ACTION_LEFT,
            Y = GamepadButton::ACTION_UP,

            LEFT_BUMPER = GamepadButton::LEFT_BUMPER,
            RIGHT_BUMPER = GamepadButton::RIGHT_BUMPER,

            LEFT_STICK = GamepadButton::LEFT_STICK,
            RIGHT_STICK = GamepadButton::RIGHT_STICK
        };
        
        const GamepadMapping MAP{
            .stick_left_x = 0,
            .stick_left_y = 1,
            .stick_right_x = 3,
            .stick_right_y = 4,

            .trigger_left = 2,
            .trigger_right = 5,

            .bumper_left = 5,
            .bumper_right = 6,

            .button_down = 0,
            .button_right = 1,
            .button_left = 3,
            .button_up = 2,

            .button_left_stick = 9,
            .button_right_stick = 10
        };
    };

    namespace Xbox{
        enum{
            A = 0,
            B = 1,
            X = 2,
            Y = 3,

            LEFT_BUMPER = 4,
            RIGHT_BUMPER = 5,

            LEFT_STICK = 6,
            RIGHT_STICK = 7
        };
        
    // I don't actually know the real map for xbox controllers this is just a placeholder
        const GamepadMapping MAP{
            .stick_left_x = 0,
            .stick_left_y = 1,
            .stick_right_x = 3,
            .stick_right_y = 4,

            .trigger_left = 2,
            .trigger_right = 5,

            .bumper_left = 5,
            .bumper_right = 6,

            .button_down = 0,
            .button_right = 1,
            .button_left = 3,
            .button_up = 2,

            .button_left_stick = 11,
            .button_right_stick = 12
        };
    };
};

#endif