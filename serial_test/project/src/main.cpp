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


float percent = 0;


int main(int argc, char* argv[])
{
    ImPlotter::init();

    SerialInterface serial;

    serial.init_field("product", "STM32 Virtual ComPort");

    while(System::is_alive())
    {
        auto angle_read = serial.request_data<double>(101, 500);
        auto velocity_read = serial.request_data<double>(102, 500);

        if(!angle_read.is_OK() || !velocity_read.is_OK())
        {
            Logger::error("Failed to read! Skipping iteration...");

            continue;
        }

        serial.write_data<double>(100, percent);

        ImPlotter::push_data(
            angle_read.value / 45.0,
            "Angle (Radians)"
        );

        ImPlotter::push_data(
            velocity_read.value,
            "Velocity (Radians / sec)"
        );

        function<void()> add_inputs = []()
        {
            ImGui::SliderFloat("Percent Output", &percent, -1, 1, "%.3f V");
        };

        if(ImPlotter::update(add_inputs) == StatusCode::FAILED)
            System::shutdown();

    }

    serial.close();
    
    ImPlotter::shutdown();
    
    return 0;

} // end of "main(int, char*)"