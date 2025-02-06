#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "gamepad_interface.hpp"

namespace hid_devices{

    namespace PS4{
        enum {
            X = 0,
            CIRCLE = 1,
            SQUARE = 2,
            TRIANGLE = 3,

            LEFT_BUMPER = 4,
            RIGHT_BUMPER = 5,

            LEFT_STICK = 6,
            RIGHT_STICK = 7
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
            .button_up = 2
        };
    };

    namespace Xbox{
        enum {
            A = 0,
            B = 1,
            X = 2,
            Y = 3,

            LEFT_BUMPER = 4,
            RIGHT_BUMPER = 5,

            LEFT_STICK = 6,
            RIGHT_STICK = 7
        };
    };
};

#endif