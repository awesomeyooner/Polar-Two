#include <iostream>

#include "plib/util/system.hpp"
#include "plib/util/logger.hpp"

#include "plib/util/string_util.hpp"

#include "CommiFaceLib/protocols/serial.hpp"
#include "CommiFaceLib/interfaces/communication_interface.hpp"

#include "PlotLib/scrolling_buffer.hpp"
#include "PlotLib/implot_plotter.hpp"

#include "plib/util/util.hpp"
#include "plib/util/system.hpp"


using namespace status_utils;
using namespace std;


float left_target = 0;
float right_target = 0;

float kP = 0;
float kI = 0;
float kD = 0;
float kF = 0;
float kV = 0;


int main(int argc, char* argv[])
{
    ImPlotter::init();

    SerialInterface serial;

    serial.init_field("product", "STM32 Virtual ComPort");

    serial.set_timeout_ms(1000);

    Logger::info("Attempting to enable device...");

    if(serial.write_data<int>(98, 0, true) == StatusCode::OK)
        Logger::info("Successfully enabled!");
    else
    {
        Logger::error("Failed to enable device! Exitting...");
        System::shutdown();
    }

    while(System::is_alive())
    {
        auto left_angle_read = serial.request_data<double>(101, 500);
        auto left_velocity_read = serial.request_data<double>(102, 500);

        auto right_angle_read = serial.request_data<double>(104, 500);
        auto right_velocity_read = serial.request_data<double>(105, 500);

        if(serial.write_data<double>(100, left_target) != StatusCode::OK)
            Logger::error("Failed to write to left motor!");

        if(serial.write_data<double>(103, right_target) != StatusCode::OK)
            Logger::error("Failed to write to right motor!");

        ImPlotter::push_data(
            left_angle_read.value,
            "Left Angle (Radians)"
        );

        ImPlotter::push_data(
            left_velocity_read.value,
            "Left Velocity (Radians / sec)"
        );

        ImPlotter::push_data(
            left_target,
            "Left Setpoint (Radians / sec)"
        );

        ImPlotter::push_data(
            right_angle_read.value,
            "Right Angle (Radians)"
        );

        ImPlotter::push_data(
            right_velocity_read.value,
            "Right Velocity (Radians / sec)"
        );


        function<void()> add_inputs = [&serial]()
        {
            ImGui::SliderFloat("Left Velocity Target", &left_target, -40, 40, "%.3f Rads/sec");
            ImGui::SliderFloat("Right Velocity Target", &right_target, -40, 40, "%.3f Rads/sec");

            // if(ImGui::InputFloat("kP", &kP, 0.0001, 0.0001, "%.9f units"))
            //     serial.write_data<double>(106, kP);

            // if(ImGui::InputFloat("kI", &kI, 0.0001, 0.0001, "%.9f units"))
            //     serial.write_data<double>(107, kI);

            // if(ImGui::InputFloat("kD", &kD, 0.0001, 0.0001, "%.9f units"))
            //     serial.write_data<double>(108, kD);

            // if(ImGui::InputFloat("kF", &kF, 0.0001, 0.0001, "%.9f units"))
            //     serial.write_data<double>(109, kF);

            // if(ImGui::InputFloat("kV", &kV, 0.0001, 0.0001, "%.9f units"))
            //     serial.write_data<double>(110, kV);
        };

        if(ImPlotter::update(add_inputs) == StatusCode::FAILED)
            System::shutdown();

    }

    Logger::info("Disabling device...");

    if(serial.write_data<int>(98, 1, true) == StatusCode::OK)
        Logger::info("Successsfully disabled device.");
    else
        Logger::error("Failed to disable device!");

    serial.close();
    
    ImPlotter::shutdown();
    
    return 0;

} // end of "main(int, char*)"