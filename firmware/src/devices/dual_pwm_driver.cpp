#include "devices/dual_pwm_driver.hpp"


using namespace status_utils;


DualPWMDriver::DualPWMDriver(TIM_HandleTypeDef* timer, int in1, int in2) 
    :   m_in1(timer, in1),
        m_in2(timer, in2)
{} // end of "DualPWMDriver(TIM_HandleTypeDef*, int, int)"


StatusCode DualPWMDriver::init()
{
    StatusCode in1_init = m_in1.init();
    StatusCode in2_init = m_in2.init();

    return combine_statuses({in1_init, in2_init});

} // end of "init()"


void DualPWMDriver::set_inverted(bool invert)
{
    m_is_inverted = invert;

} // end of "set_inverted(bool)"


bool DualPWMDriver::is_inverted()
{
    return m_is_inverted;

} // end of "is_inverted()"


void DualPWMDriver::set_input_voltage(double input_voltage)
{
    // Make sure it's positive
    if(input_voltage < 0)
        input_voltage *= -1;

    m_input_voltage = input_voltage;

} // end of "set_input_voltage(double)"


double DualPWMDriver::get_input_voltage()
{
    return m_input_voltage;

} // end of "get_input_voltage"


void DualPWMDriver::set_percent(double percent)
{
    if(percent > 1)
        percent = 1;
    else if(percent < -1)
        percent = -1;

    // Invert percent if needed
    if(m_is_inverted)
        percent *= -1;

    // Keep track of the commanded percent out
    m_percent = percent;

    if(percent > 0)
    {
        m_in1.set_duty(fabs(percent));
        m_in2.set_duty(0);
    }
    else if(percent < 0)
    {
        m_in1.set_duty(0);
        m_in2.set_duty(fabs(percent));
    }
    else // percent == 0
    {
        m_in1.set_duty(0);
        m_in2.set_duty(0);
    }

} // end of "set_percent(double)"


void DualPWMDriver::set_voltage(double voltage)
{
    if(m_input_voltage == -1)
        return;

    // This is just a linear scale, ie
    // 24V input voltage, 12V requested out
    // Percent is then 50%
    double percent_out = voltage / m_input_voltage;

    set_percent(percent_out);

} // end of "set_voltage(double)"


double DualPWMDriver::get_percent()
{
    return m_percent;

} // end of "get_percent()"


void DualPWMDriver::stop()
{
    set_percent(0);

} // end of "stop()"


TimerDevice& DualPWMDriver::get_IN1()
{
    return m_in1;

} // end of "get_IN1()"


TimerDevice& DualPWMDriver::get_IN2()
{
    return m_in2;

} // end of "get_IN2()"