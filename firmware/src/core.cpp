#include "core.hpp"

#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

#include "EmbeddedLib/devices/gpio_device.hpp"

#include "ActionLib/ActionManager.hpp"

#include "WireLib/communication/protocols/serial_interface.hpp"
#include "WireLib/communication/wire_manager.hpp"
#include "WireLib/registers/register_manager.hpp"

#include "devices/quadrature_encoder.hpp"
#include "devices/dual_pwm_driver.hpp"


using namespace status_utils;
using namespace std;


GPIODevice led = GPIODevice(GPIOC, GPIO_PIN_1);

QuadratureEncoder left_encoder = QuadratureEncoder(
    GPIOC, GPIO_PIN_10,
    GPIOA, GPIO_PIN_15,
    12 * 4,
    45
);

DualPWMDriver left_driver = DualPWMDriver(
    &htim8,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2
);

QuadratureEncoder right_encoder = QuadratureEncoder(
    GPIOC, GPIO_PIN_12,
    GPIOC, GPIO_PIN_11,
    12 * 4,
    45
);

DualPWMDriver right_driver = DualPWMDriver(
    &htim8,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
);


void init()
{
    System::init();

    Serial.set_parse_type(ParseType::PACKET);

    WireManager::attach(Serial);

    left_driver.init();
    right_driver.init();

    // Enable and Disable
    RegisterManager::add_command(
        Command<int>(
            98,
            [](int data) -> StatusCode
            {
                System::feed();

                if(data == 0)
                    System::set_state(SystemState::OK);
                else
                    System::set_state(SystemState::HALT);

                return StatusCode::OK;
            },
            true
        )
    );

    // Left Control
    RegisterManager::add_command(
        Command<double>(
            100,
            [](double data) -> StatusCode
            {
                System::feed();

                left_driver.set_percent(data);

                return StatusCode::OK;
            }
        )
    );

    // Left Get Angle
    RegisterManager::add_request(
        Request<double>(
            101,
            []() -> double
            {
                return left_encoder.get_angle();
            }
        )
    );

    // Left Get Velocity
    RegisterManager::add_request(
        Request<double>(
            102,
            []() -> double
            {
                return left_encoder.get_velocity();
            }
        )
    );

    // Right Control
    RegisterManager::add_command(
        Command<double>(
            103,
            [](double data) -> StatusCode
            {
                System::feed();

                right_driver.set_percent(data);

                return StatusCode::OK;
            }
        )
    );

    // Right Get Angle
    RegisterManager::add_request(
        Request<double>(
            104,
            []() -> double
            {
                return right_encoder.get_angle();
            }
        )
    );

    // Right Get Velocity
    RegisterManager::add_request(
        Request<double>(
            105,
            []() -> double
            {
                return right_encoder.get_velocity();
            }
        )
    );

    // Update encoder velocity every 5ms
    ActionManager::add(
        Action(0.005).link_callback(
            [](double, double) -> StatusedValue<bool>
            {
                left_encoder.update();
                right_encoder.update();

                return StatusedValue<bool>(false, StatusCode::OK);
            }
        )
    );

} // end of "init()"


void update()
{
    ActionManager::update();

    System::update();

    if(!System::is_OK())
    {
        left_driver.stop();
        right_driver.stop();

        led.set_high();

        return;
    }

    led.set_low();

} // end of "update()"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Link EXTI Callbacks
    left_encoder.on_EXTI_callback(GPIO_Pin);
    right_encoder.on_EXTI_callback(GPIO_Pin);

} // end of "HAL_GPIO_EXTI_Callback(uint16_t)"