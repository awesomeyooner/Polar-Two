#ifndef ARDUINO_INTERFACE__TYPES__ARDUINO_INTERFACE_TYPES_HPP_
#define ARDUINO_INTERFACE__TYPES__ARDUINO_INTERFACE_TYPES_HPP_

namespace ArduinoUtility{

  struct ArduinoMessage{
    std::string device = ""; //(id) left, right, voltage_sensor
    std::string message_type = ""; //status, control, config
    std::string type_value = ""; //velocity, position, percent, inverted
    double value = 0; //whatever value you want to send
  };

  struct Config{
      std::string port = "";

      std::string drive_id = "";
      std::string steer_id = "";

      std::string voltage_sensor_id = "";

      std::string heartbeat_id = "";

      int32_t baud_rate = 0;
      int32_t loop_rate = 0;
      int32_t timeout_ms = 0;
  };

}

namespace MessageType{
  constexpr char STATUS[] = "status";

  constexpr char CONTROL[] = "control";

  constexpr char CONFIG[] = "config";

  constexpr char UNCONFIGURED[] = "unconfigured";
}

namespace TypeValue{
  constexpr char VELOCITY[] = "velocity";

  constexpr char POSITION[] = "position";

  constexpr char EFFORT[] = "effort";

  constexpr char VOLTAGE[] = "voltage";

  constexpr char RAW[] = "raw";

  constexpr char LATENCY[] = "latency";

  constexpr char HERTZ[] = "hertz";

  constexpr char LOWER_BOUND[] = "lower_bound";

  constexpr char NEUTRAL[] = "neutral";

  constexpr char UPPER_BOUND[] = "upper_bound";
}

#endif  // ARDUINO_INTERFACE__TYPES__ARDUINO_INTERFACE_TYPES_HPP_
