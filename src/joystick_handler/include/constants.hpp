#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "gamepad_interface.hpp"

namespace hid_devices{

    namespace gamepads{
        const GamepadMapping MAPPING_PS4_CONTROLLER{
            .stick_left_x_index = 0,
            .stick_left_y_index = 1,
            .stick_right_x_index = 3,
            .stick_right_y_index = 4,

            .trigger_left_index = 2,
            .trigger_right_index = 5,

            .bumper_left_index = 5,
            .bumper_right_index = 6,

            .button_a_index = 0,
            .button_b_index = 1,
            .button_x_index = 3,
            .button_y_index = 2
        };
    };
};

#endif