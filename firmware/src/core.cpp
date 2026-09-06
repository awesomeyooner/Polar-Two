#include "core.hpp"

#include "gpio.h"

#include "EmbeddedLib/devices/gpio_device.hpp"


GPIODevice led = GPIODevice(GPIOC, GPIO_PIN_1);


void init()
{

} // end of "init()"


void update()
{

    // led.set_high();
    // HAL_Delay(500);
    // led.set_low();
    // HAL_Delay(500);
    
} // end of "update()"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_11)
        led.set_high();
    else if(GPIO_Pin == GPIO_PIN_12)
        led.set_low();
}