#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "arduino_interface_types.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;

    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms{

  private:

    LibSerial::SerialPort serial_connection;
    int timeout_ms;

  public:

    ArduinoComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {  
      this->timeout_ms = timeout_ms;
      serial_connection.Open(serial_device);
      serial_connection.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect()
    {
      serial_connection.Close();
    }

    bool connected() const
    {
      return serial_connection.IsOpen();
    }

    std::string debug(std::vector<ArduinoUtility::ArduinoMessage> messages){

      json total_packet;

      for(ArduinoUtility::ArduinoMessage message : messages){

        json single_packet = {
          {"device", message.device},
          {"message_type", message.message_type},
          {"type_value", message.type_value},
          {"value", message.value}
        };

        total_packet.push_back(single_packet);
      }

      return total_packet.dump() + '\n';
      //serial_connection.Write(total_packet.dump());
    }

    int findDifference(std::string str1, std::string str2){
      for (int i = 0; i < str1.length(); ++i) {
        if (str1[i] != str2[i]) {
            return i;
        }
      }

      return -1;
    }

    bool compareStrings(const std::string& str1, const std::string& str2) {
    if (str1.length() != str2.length()) {
        return false; // If the lengths are different, the strings are not equal
    }

    for (size_t i = 0; i < str1.length(); ++i) {
        if (str1[i] != str2[i]) {
            return false; // If any characters are different, the strings are not equal
        }
    }

    return true; // If all characters are the same, the strings are equal
}
    
    std::string stringToLiteral(const std::string& str) {
      std::string result = "\""; // Add opening quote
      for (char c : str) {
          if (c == '\"' || c == '\\') { // Escape double quotes and backslashes
              result += '\\'; // Add escape character
          }
          result += c; // Add character
      }
      result += "\""; // Add closing quote
      return result;
    }
    
    json get_json_dump(){
      std::string raw_data;

      serial_connection.ReadLine(raw_data, '\n', timeout_ms);

      raw_data.pop_back();

      try{
        return json::parse(raw_data).at(0);
      }
      catch (json::parse_error& ex){
        return NULL;
      }
    }

    std::vector<ArduinoUtility::ArduinoMessage> get_message_dump(){
      std::vector<ArduinoUtility::ArduinoMessage> messages;

      json dump = get_json_dump();

      //RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), dump.dump().c_str());

      if(dump == NULL)
        return messages;

      for(auto object : dump){
        
        messages.push_back(
          ArduinoUtility::ArduinoMessage
          {
            .device = object["device"],
            .message_type = object["message_type"],
            .type_value = object["type_value"],
            .value = (double)object["value"]
          }
        );
      } //for loop

      return messages;
    }

    void send_message(ArduinoUtility::ArduinoMessage message){

        json packet = {
            {"device", message.device},
            {"message_type", message.message_type},
            {"type_value", message.type_value},
            {"value", message.value}
        };

       
      //serial_connection.FlushIOBuffers(); // Just in case
      serial_connection.Write(packet.dump() + '\n');
    }

    void send_message(std::vector<ArduinoUtility::ArduinoMessage> messages){

      json total_packet;

      for(ArduinoUtility::ArduinoMessage message : messages){

        json single_packet = {
          {"device", message.device},
          {"message_type", message.message_type},
          {"type_value", message.type_value},
          {"value", message.value}
        };

        total_packet.push_back(single_packet);
      }

      //RCLCPP_INFO(rclcpp::get_logger("MicroSystemHardware"), total_packet.dump().c_str());

      serial_connection.Write(total_packet.dump() + '\n');
    }
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP