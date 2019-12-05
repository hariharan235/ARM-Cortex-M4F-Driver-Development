/**
 * main.c
 */

#include "tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/mcu_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/gpio_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/wait.h"
#include <string.h>

void init_led_gpio(void)
{
    gpio_handle_t led;
    memset(&led,0,sizeof(led));
    led.p_gpio_x = GPIOF;
    led.gpio_pin_config.direction = GPIO_DIR_OUTPUT;
    led.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
    led.gpio_pin_config.pin_number = 1;
    gpio_init(&led);
    led.gpio_pin_config.pin_number = 2;
    gpio_init(&led);
    led.gpio_pin_config.pin_number = 3;
    gpio_init(&led);
}

void init_pb_gpio(void)
{
    gpio_handle_t pushbtn;
    memset(&pushbtn,0,sizeof(pushbtn));
    pushbtn.p_gpio_x = GPIOF;
    pushbtn.gpio_pin_config.direction = GPIO_DIR_INPUT;
    pushbtn.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
    pushbtn.gpio_pin_config.pull_up_down = GPIO_PUR_ENABLE;
    pushbtn.gpio_pin_config.pin_number = 4;
    gpio_init(&pushbtn);
}

void cycle_led_toggle(void)
{
    static uint8_t id = 1;

    while(1)
    {
        while(gpio_read_pin(GPIOF,4));

            if(id == 4)
            {
                id = 1;
            }
            gpio_write_pin(GPIOF,id,SET);
            waitMicrosecond(100000);
            gpio_write_pin(GPIOF,id,RESET);
            id++;
    }
}


int main(void)
{
    init_led_gpio();
    init_pb_gpio();
    cycle_led_toggle();
    for(;;);
    //return 0;
}
