#include "core.hpp"

#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"

#include "EmbeddedLib/devices/gpio_device.hpp"

#include "WireLib/communication/protocols/serial_interface.hpp"
#include "WireLib/communication/wire_manager.hpp"
#include "WireLib/registers/register_manager.hpp"

#include "devices/quadrature_encoder.hpp"
#include "devices/dual_pwm_driver.hpp"


using namespace status_utils;
using namespace std;


GPIODevice led = GPIODevice(GPIOC, GPIO_PIN_1);

QuadratureEncoder encoder = QuadratureEncoder(
    GPIOC, GPIO_PIN_11,
    GPIOC, GPIO_PIN_12,
    12 * 4,
    1
);

DualPWMDriver driver = DualPWMDriver(
    &htim8,
    TIM_CHANNEL_1,
    TIM_CHANNEL_2
);

void init()
{
    System::init();

    Serial.set_parse_type(ParseType::PACKET);

    WireManager::attach(Serial);

    driver.init();

    RegisterManager::add_command(
        Command<double>(
            100,
            [](double data) -> StatusCode
            {
                System::feed();

                driver.set_percent(data);

                return StatusCode::OK;
            }
        )
    );

    RegisterManager::add_request(
        Request<double>(
            101,
            []() -> double
            {
                return encoder.get_angle();
            }
        )
    );

    RegisterManager::add_request(
        Request<double>(
            102,
            []() -> double
            {
                return encoder.get_velocity();
            }
        )
    );

} // end of "init()"


void update()
{
    encoder.update();

    System::update();

    if(!System::is_OK())
    {
        driver.stop();

        led.set_high();

        return;
    }

    led.set_low();

    // double counts = encoder.get_counts();
    // double cpr = 48.0 * 45.0;
    // double gear_ratio = 45;

    // double rotations = counts / cpr;
    // double angle = rotations;

    // Serial.println(encoder.get_angle() / 45.0);

    // driver.set_percent(0);

    // Serial.println(((double)encoder.get_counts() / (double)48) / 45.0);

} // end of "update()"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    encoder.on_EXTI_callback(GPIO_Pin);

} // end of "HAL_GPIO_EXTI_Callback(uint16_t)"