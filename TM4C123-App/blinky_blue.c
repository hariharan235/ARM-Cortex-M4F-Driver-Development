

/*
 * @file  blinky_blue.c
 * @brief Crude test for header file structure defines
 */



#include "main_tm4c123gh6pm.h"
#include "gpio_tm4c123gh6pm.h"


void initHW()
{

    // clock set up
    SYSCTL_t *pSYSCTL = SYSCTL;

    // 40 Hz
    pSYSCTL->RCC = 0x00000540 | 0x00400000 | (0x04 << 23);


}


// Dr.Losh's delay
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6"       );
    __asm("WMS_LOOP1:   SUB  R1, #1"       );
    __asm("             CBZ  R1, WMS_DONE1");
    __asm("             NOP"               );
    __asm("             NOP"               );
    __asm("             B    WMS_LOOP1"    );
    __asm("WMS_DONE1:   SUB  R0, #1"       );
    __asm("             CBZ  R0, WMS_DONE0");
    __asm("             NOP"               );
    __asm("             B    WMS_LOOP0"    );
    __asm("WMS_DONE0:"                     );
}


/**
 * main.c
 */
int main(void)
{
    GPIO_HANDLE_t ioTest, led3;

    // The usual initHw
    initHW();

    //GPIO_clockEnable(GPIOF);

    ioTest.pGPIOx = GPIOF;
    ioTest.gpioPinConfig.PINNUMBER = 2;
    ioTest.gpioPinConfig.DIRECTION = GPIO_DIR_OUTPUT;
    ioTest.gpioPinConfig.PINMODE   = GPIO_DEN_ENABLE;

    GPIO_Init(&ioTest);

    led3.pGPIOx = GPIOF;
    led3.gpioPinConfig.PINNUMBER = 1;
    led3.gpioPinConfig.DIRECTION = GPIO_DIR_OUTPUT;
    led3.gpioPinConfig.PINMODE   = GPIO_DEN_ENABLE;

    GPIO_Init(&led3);

    ioTest.pGPIOx = GPIOF;
    ioTest.gpioPinConfig.PINNUMBER  = 4;
    ioTest.gpioPinConfig.DIRECTION  = GPIO_DIR_INPUT;
    ioTest.gpioPinConfig.PINMODE    = GPIO_DEN_ENABLE;
    ioTest.gpioPinConfig.PULLUPDOWN = GPIO_PUR_ENABLE;

    GPIO_Init(&ioTest);

    while(1)
    {

        if(GPIO_ReadFromPin(GPIOF, 4) == 0)
        {
            GPIO_WriteToPin(GPIOF, 2, 1);
            GPIO_WriteToPin(GPIOF, 1, 1);
            waitMicrosecond(100000);

            GPIO_WriteToPin(GPIOF, 2, 0);
            GPIO_WriteToPin(GPIOF, 1, 0);
            waitMicrosecond(100000);
        }

    }

    return 0;

}
