#ifndef QUADRATURE_ENCODER_HPP
#define QUADRATURE_ENCODER_HPP


#include "EmbeddedLib/system.hpp"

#include "EmbeddedLib/devices/gpio_device.hpp"

#include <cmath>


class QuadratureEncoder
{

    public:

        QuadratureEncoder(
            GPIO_TypeDef* gpio_family_A, uint16_t pin_A, // GPIO Channel A
            GPIO_TypeDef* gpio_family_B, uint16_t pin_B, // GPIO Channel B 
            int counts_per_revolution = 1, double gear_ratio = 1
        );

        void on_EXTI_callback(uint16_t pin);

        void update();

        double get_angle();

        double get_velocity();

        double get_rotations();

        int get_counts();

        void set_inverted(bool inverted);

        bool is_inverted();

        void set_gear_ratio(double gear_ratio);

        void set_CPR(int counts_per_revolution);


    private:

        // The last time `update()` was called, in seconds
        double m_prev_timestamp = 0;

        // The angle since the last time `update()` was called. Used to calculate velocity
        double m_prev_angle = 0;

        // Optional gear ratio if this encoder is attached to a gearbox
        double m_gear_ratio = 1;

        // The number of counts per mechanical revolution of the encoder wheel
        int m_counts_per_revolution = 1;

        // Set to true to negate the returned position
        bool m_is_inverted = false;

        // The number of counts this encoder is currently at
        int m_counts = 0;

        // The velocity in radians per second, with gear ratio included
        double m_velocity = 0;

        // The A Channel of the encoder
        GPIODevice m_channelA;

        // The B Channel of the encoder
        GPIODevice m_channelB;

}; // class QuadratureEncoder


#endif // QUADRATURE_ENCODER_HPP