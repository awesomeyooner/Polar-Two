#include "devices/quadrature_encoder.hpp"


QuadratureEncoder::QuadratureEncoder(
    GPIO_TypeDef* gpio_family_A, uint16_t pin_A, // GPIO Channel A
    GPIO_TypeDef* gpio_family_B, uint16_t pin_B, // GPIO Channel B 
    int counts_per_revolution, double gear_ratio) :
        m_channelA(gpio_family_A, pin_A),
        m_channelB(gpio_family_B, pin_B)

{
    m_counts_per_revolution = counts_per_revolution;
    m_gear_ratio = gear_ratio;

    // Attach the interrupt logic for incrementing the counts
    m_channelA.attach_callback(
        [this]()
        {
            int sign0 = m_channelA.is_high() ? 1 : -1;
            int sign1 = m_channelB.is_high() ? -1 : 1;

            m_counts += sign0 * sign1;
        }
    );

    m_channelB.attach_callback(
        [this]()
        {
            int sign0 = m_channelA.is_high() ? 1 : -1;
            int sign1 = m_channelB.is_high() ? 1 : -1;

            m_counts += sign0 * sign1;
        }
    );

} // end of "QuadratureEncoder(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, int = 1, double = 1)"


void QuadratureEncoder::on_EXTI_callback(uint16_t pin)
{
    m_channelA.on_EXTI_callback(pin);
    m_channelB.on_EXTI_callback(pin);

} // end of "on_EXTI_callback(uint16_t)"


void QuadratureEncoder::update()
{
    double now = System::get_seconds(true);
    double angle = get_angle();

    double dt = now - m_prev_timestamp;
    double dTheta = angle - m_prev_angle;

    if(dt != 0)
        m_velocity = dTheta / dt;

    m_prev_timestamp = now;
    m_prev_angle = angle;

} // end of "update()"


double QuadratureEncoder::get_angle()
{
    double inversion = m_is_inverted ? -1 : 1;
    
    // Radians = Rotations * 2PI
    return get_rotations() * 2 * M_PI * inversion;

} // end of "get_angle()"


double QuadratureEncoder::get_velocity()
{
    return m_velocity;

} // end of "get_velocity()"


double QuadratureEncoder::get_rotations()
{
    // Raw Rotations = counts / CPR
    // Shaft Rotations = Raw Rotations / Gear Ratio
    // If the Gear Ratio is 25 : 1 (motor : shaft)
    // Then the motor rotating 25 times means the shaft rotates once
    // Hence division
    return ((double)get_counts() / (double)m_counts_per_revolution) / m_gear_ratio;

} // end of "get_rotations()"


int QuadratureEncoder::get_counts()
{
    return m_counts;

} // end of "get_counts()"


void QuadratureEncoder::set_inverted(bool inverted)
{
    m_is_inverted = inverted;

} // end of "set_inverted(bool)"


bool QuadratureEncoder::is_inverted()
{
    return m_is_inverted;

} // end of "is_inverted()"


void QuadratureEncoder::set_gear_ratio(double gear_ratio)
{
    m_gear_ratio = gear_ratio;

} // end of "set_gear_ratio(double)"


void QuadratureEncoder::set_CPR(int counts_per_revolution)
{
    m_counts_per_revolution = counts_per_revolution;

} // end of "set_CPR"


