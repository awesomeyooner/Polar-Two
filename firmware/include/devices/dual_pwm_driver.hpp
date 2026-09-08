#ifndef DUAL_PWM_DRIVER_HPP
#define DUAL_PWM_DRIVER_HPP

#include <cmath>

#include "tim.h"

#include "EmbeddedLib/status.hpp"
#include "EmbeddedLib/devices/timer_device.hpp"


class DualPWMDriver
{

    public:

        /**
         * @brief Create a new DualPWMDriver motor driver with the given pins
         * 
         * @param timer `TIM_HandleTypeDef*` Timer family
         * @param in1 `int` IN1 Channel
         * @param in2 `int` IN2 Channel
         */
        DualPWMDriver(TIM_HandleTypeDef* timer, int in1, int in2);

        /**
         * @brief Initialize the timers
         * 
         * @return `status_utils::StatusCode` 
         */
        status_utils::StatusCode init();

        /**
         * @brief Invert the input of the driver
         * 
         */
        void set_inverted(bool invert = true);

        /**
         * @brief Get whether or not the input is inverted
         * 
         * @return `bool`
         */
        bool is_inverted();

        /**
         * @brief Set the input voltage
         * 
         * @param input_voltage `double`
         */
        void set_input_voltage(double input_voltage);

        /**
         * @brief Gets the input voltage. Note this does not READ voltage but rather
         * returns the input voltage set in code
         * 
         * @return `double` -1 if not set 
         */
        double get_input_voltage();

        /**
         * @brief Sets the percent output of the motor, from `[-1, 1]`. 
         * 
         * Example: `0.5` is 50% output in the forward direction. `-0.5` will be reverse
         * 
         * This will clamp input to [-1, 1]
         * 
         * @param percent `double` The percent speed from `[-1, 1]`
         */
        void set_percent(double percent);

        /**
         * @brief Set the output voltage. This only works if `set_input_voltage()` is called.
         * 
         * @param voltage `double`
         */
        void set_voltage(double voltage);

        /**
         * @brief Gets the percent output command sent to the motor, from `[-1, 1]
         * 
         * @return `double`
         */
        double get_percent();

        /**
         * @brief Stops the motor (sets percent to 0)
         * 
         */
        void stop();

        /**
         * @brief Get the underlying `TimerDevice` for the IN1 channel
         * 
         * @return `TimerDevice&` 
         */
        TimerDevice& get_IN1();

        /**
         * @brief Get the underlying `TimerDevice` for the IN2 channel
         * 
         * @return `TimerDevice&` 
         */
        TimerDevice& get_IN2();

    private:

        // IN1 Channel
        TimerDevice m_in1;

        // IN2 Channel
        TimerDevice m_in2;

        // True if input should be negated
        bool m_is_inverted = false;

        // Bookkeeping for the percent output applied [-1, 1]
        double m_percent = 0;

        double m_input_voltage = -1;

}; // class DualPWMDriver 


#endif // DUAL_PWM_DRIVER_HPP