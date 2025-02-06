#ifndef GAMEPAD_INTERFACE_HPP
#define GAMEPAD_INTERFACE_HPP

#include "sensor_msgs/msg/joy.hpp"
#include <map>

namespace hid_devices{

    enum GamepadButton{
        ACTION_DOWN = 0,
        ACTION_RIGHT = 1,
        ACTION_LEFT = 2,
        ACTION_UP = 3,

        LEFT_BUMPER = 4,
        RIGHT_BUMPER = 5,

        LEFT_STICK = 6,
        RIGHT_STICK = 7
    };

    enum GamepadAxis{
        LEFT_X = 0,
        LEFT_Y = 1,
        RIGHT_X = 2,
        RIGHT_Y = 3,

        LEFT_TRIGGER = 4,
        RIGHT_TRIGGER = 5
    };

    struct Button{
        int id;
        int index;

        Button(){}
        Button(int index, int id) : index(index), id(id){}

        bool pressed = false;
        bool on_press = false;
        bool on_release = false;

        bool get(){
            return pressed;
        }

        void update(std::vector<int> buttons){
            bool status = buttons.at(index) == 1 ? true : false;

            update(status);
        }

        void update(bool status){
            previous = pressed;

            pressed = status;

            on_press = false;
            if(pressed && !previous) //if pressed and not previously pressed aka ON PRESS
                on_press = true;

            on_release = false;
            if(!pressed && previous) //if not pressed and previously was pressed aka ON RELEASE
                on_release = true;
        }

        private:
            bool previous = false;
            //int index;
    };

    struct Axis{
        int id;

        Axis(){}
        Axis(int index, int id) : index(index), id(id){}

        double value = 0;

        double get(){
            return value;
        }

        void update(std::vector<float> axes){
            update(axes.at(index));
        }

        void update(double status){
            value = status;
        }

        private:
            int index;
    };

    struct GamepadMapping{
        int stick_left_x;
        int stick_left_y;
        int stick_right_x;
        int stick_right_y;

        int trigger_left;
        int trigger_right;

        int bumper_left;
        int bumper_right;

        int button_down;
        int button_right;
        int button_left;
        int button_up;
    };

    struct GamepadStatus{

        GamepadStatus(){}

        GamepadStatus(GamepadMapping map) : 
            stick_left_x(map.stick_left_x, GamepadAxis::LEFT_X),
            stick_left_y(map.stick_left_y, GamepadAxis::LEFT_Y),
            stick_right_x(map.stick_right_x, GamepadAxis::RIGHT_X),
            stick_right_y(map.stick_right_y, GamepadAxis::RIGHT_Y),

            trigger_left(map.trigger_left, GamepadAxis::LEFT_TRIGGER),
            trigger_right(map.trigger_right, GamepadAxis::RIGHT_TRIGGER),

            bumper_left(map.bumper_left, GamepadButton::LEFT_BUMPER),
            bumper_right(map.bumper_left, GamepadButton::RIGHT_BUMPER),

            button_down(map.button_down, GamepadButton::ACTION_DOWN),
            button_right(map.button_right, GamepadButton::ACTION_RIGHT),
            button_left(map.button_left, GamepadButton::ACTION_LEFT),
            button_up(map.button_up, GamepadButton::ACTION_UP){}

        void update(sensor_msgs::msg::Joy joy_packet){
            update(joy_packet.axes, joy_packet.buttons);
        }

        void update(std::vector<float> axes, std::vector<int> buttons){
            stick_left_x.update(axes);
            stick_left_y.update(axes);
            stick_right_x.update(axes);
            stick_right_y.update(axes);

            trigger_left.update(axes);
            trigger_right.update(axes);

            bumper_left.update(buttons);
            bumper_right.update(buttons);

            button_down.update(buttons);
            button_right.update(buttons);
            button_left.update(buttons);
            button_up.update(buttons);
        }

        Axis stick_left_x;
        Axis stick_left_y;
        Axis stick_right_x;
        Axis stick_right_y;

        Axis trigger_left;
        Axis trigger_right;

        Button bumper_left;
        Button bumper_right;

        Button button_down;
        Button button_right;
        Button button_left;
        Button button_up;
    };

    class Gamepad{

        public:
            GamepadStatus status;

            Gamepad(GamepadMapping mapping) : status(mapping){
                axes[status.stick_left_x.id] = &status.stick_left_x;
                axes[status.stick_left_y.id] = &status.stick_left_y;

                axes[status.stick_right_x.id] = &status.stick_right_x;
                axes[status.stick_right_y.id] = &status.stick_right_y;

                axes[status.trigger_left.id] = &status.trigger_left;
                axes[status.trigger_right.id] = &status.trigger_right;

                buttons[status.bumper_left.id] = &status.bumper_left;
                buttons[status.bumper_right.id] = &status.bumper_right;

                buttons[status.button_down.id] = &status.button_down;
                buttons[status.button_right.id] = &status.button_right;
                buttons[status.button_left.id] = &status.button_left;
                buttons[status.button_up.id] = &status.button_up;
            }

            // void initialize(GamepadMapping mapping){
                
            // }

            void update(const sensor_msgs::msg::Joy &joy_packet){
                status.update(joy_packet);
            }

            Button* get_button(int id){
                return buttons[id];
            }

            Axis* get_axis(int id){
                return axes[id];
            }
            
        private:
            std::map<int, Button*> buttons;
            std::map<int, Axis*> axes;

            

    };
};

#endif