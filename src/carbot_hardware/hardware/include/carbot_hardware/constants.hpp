#ifndef CONSTANTS_CONSTANTS_HPP
#define CONSTANTS_CONSTANTS_HPP

#include <string>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace MotorConstants{
  const std::string DEVICE = "virtual_rear_wheel_joint";
  const std::string COMMAND_INTERFACE = hardware_interface::HW_IF_VELOCITY;

  const int MAX_FORWARD = 500;
  const int NEUTRAL = 360;
  const int MAX_REVERSE = 280;

  const int CHANNEL = 0;
}

namespace ServoConstants{
  const std::string DEVICE = "virtual_front_wheel_joint";
  const std::string COMMAND_INTERFACE = hardware_interface::HW_IF_POSITION;

  const int MAX_LEFT = 520;
  const int NEUTRAL = 380;
  const int MAX_RIGHT = 260;

  const int CHANNEL = 1;
}

namespace VoltageSensorConstants{
  const std::string DEVICE = "voltage_sensor";

  const int CHANNEL = 1;
}
 
#endif