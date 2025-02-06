#ifndef GAMEPAD_INTERFACE_HPP
#define GAMEPAD_INTERFACE_HPP

#include "sensor_msgs/msg/joy.hpp"

namespace hid_devices{

    struct GamepadMapping{
        int stick_left_x_index;
        int stick_left_y_index;
        int stick_right_x_index;
        int stick_right_y_index;

        int trigger_left_index;
        int trigger_right_index;

        int bumper_left_index;
        int bumper_right_index;

        int button_a_index;
        int button_b_index;
        int button_x_index;
        int button_y_index;
    };

    struct GamepadStatus{
        double stick_left_x = 0;
        double stick_left_y = 0;
        double stick_right_x = 0;
        double stick_right_y = 0;

        double trigger_left = 0;
        double trigger_right = 0;

        bool bumper_left;
        bool bumper_right;

        bool button_a = false;
        bool button_b = false;
        bool button_x = false;
        bool button_y = false;

        bool bumper_left_on_press = false;
        bool bumper_left_on_release = false;

        bool bumper_right_on_press = false;
        bool bumper_right_on_release = false;

        bool button_a_on_press = false;
        bool button_a_on_release = false;

        bool button_b_on_press = false;
        bool button_b_on_release = false;

        bool button_x_on_press;
        bool button_x_on_release = false;

        bool button_y_on_press = false;
        bool button_y_on_release = false;
    };

    class Gamepad{

        public:
            GamepadStatus current_status;
            GamepadStatus previous_status;

            Gamepad(GamepadMapping mapping){
                map = mapping;
            }

            Gamepad(){}

            void update(const sensor_msgs::msg::Joy &joy_packet){

                previous_status = current_status;

                current_status.stick_left_x = joy_packet.axes.at(map.stick_left_x_index);
                current_status.stick_left_y = joy_packet.axes.at(map.stick_left_y_index);
                current_status.stick_right_x = joy_packet.axes.at(map.stick_right_x_index);
                current_status.stick_right_y = joy_packet.axes.at(map.stick_right_y_index);

                current_status.trigger_left = joy_packet.axes.at(map.trigger_left_index);
                current_status.trigger_right = joy_packet.axes.at(map.trigger_right_index);

                current_status.bumper_left = joy_packet.buttons.at(map.bumper_left_index) == 1 ? true : false;
                current_status.bumper_right = joy_packet.buttons.at(map.bumper_right_index) == 1 ? true : false;

                current_status.button_a = joy_packet.buttons.at(map.button_a_index) == 1 ? true : false;
                current_status.button_b = joy_packet.buttons.at(map.button_b_index) == 1 ? true : false;
                current_status.button_x = joy_packet.buttons.at(map.button_x_index) == 1 ? true : false;
                current_status.button_y = joy_packet.buttons.at(map.button_y_index) == 1 ? true : false;

                //left bumper
                current_status.bumper_left_on_press = false;
                if(current_status.bumper_left && !previous_status.bumper_left){ //if previously not press and now is pressed
                    current_status.bumper_left_on_press = true;
                }
                
                current_status.bumper_left_on_release = false;
                if(!current_status.bumper_left && previous_status.bumper_left){ //if previously pressed and is now not pressed
                    current_status.bumper_left_on_release = true;
                }

                //right bumper
                current_status.bumper_right_on_press = false;
                if(current_status.bumper_right && !previous_status.bumper_right){ //if previously not press and now is pressed
                    current_status.bumper_right_on_press = true;
                }

                current_status.bumper_right_on_release = false;
                if(!current_status.bumper_right && previous_status.bumper_right){ //if previously pressed and is now not pressed
                    current_status.bumper_right_on_release = true;
                }

                //button A
                current_status.button_a_on_press = false;
                if(current_status.button_a && !previous_status.button_a){ //if previously not press and now is pressed
                    current_status.button_a_on_press = true;
                }

                current_status.button_a_on_release = false;
                if(!current_status.button_a && previous_status.button_a){ //if previously pressed and is now not pressed
                    current_status.button_a_on_release = true;
                }

                //button B
                current_status.button_b_on_press = false;
                if(current_status.button_b && !previous_status.button_b){ //if previously not press and now is pressed
                    current_status.button_b_on_press = true;
                }

                current_status.button_b_on_release = false;
                if(!current_status.button_b && previous_status.button_b){ //if previously pressed and is now not pressed
                    current_status.button_b_on_release = true;
                }

                //button X
                current_status.button_x_on_press = false;
                if(current_status.button_x && !previous_status.button_x){ //if previously not press and now is pressed
                    current_status.button_x_on_press = true;
                }

                current_status.button_x_on_release = false;
                if(!current_status.button_x && previous_status.button_x){ //if previously pressed and is now not pressed
                    current_status.button_x_on_release = true;
                }

                //button Y
                current_status.button_y_on_press = false;
                if(current_status.button_y && !previous_status.button_y){ //if previously not press and now is pressed
                    current_status.button_y_on_press = true;
                }

                current_status.button_y_on_release = false;
                if(!current_status.button_y && previous_status.button_y){ //if previously pressed and is now not pressed
                    current_status.button_y_on_release = true;
                }

            }

            

        private:
            GamepadMapping map;

    };
};

#endif