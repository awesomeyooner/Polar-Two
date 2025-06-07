# Joystick Driver ROS2 Package

## Layout

Each button and axis has an index in the array, the GamepadMapping class is used to store all of these indexes in a format that makes it easy to know what's what. The enum in `constants.hpp` is pretty much just for naming, it's the same as `GamepadButton`. I only made it so that it's easier to call buttons instead of like `ACTION_LEFT` or something