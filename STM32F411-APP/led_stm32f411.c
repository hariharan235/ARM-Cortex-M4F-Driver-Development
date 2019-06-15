/*
 * @file  led_stm32f411.c
 * @brief Crude Test for GPIO API
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpio_stm32f411.h"


#define BTN_PRESSED 1
#define EX_BTN_PRESSED 0

//#define test1 /*!< Test Onboard-led toggle - Pass*/
//#define test2 /*!< Test On board-pushbutton - Pass */
//#define test3 /*!< Test GPIO read using external pushbutton (Test read fns) - Pass*/
//#define test4 /*!< Test GPIO write using external led (Test write fns) - Pass */
#define test5 /*!< IRQ config and handler - Pass */

void delay()
{
	volatile uint32_t i;
	for(i = 0 ; i < 1000000 ; i++);
}


void ledInitPP()
{
	/*!< GPIO handler */
	GPIO_handle_t LED;

	memset(&LED,0,sizeof(LED));

	/*!< GPIO Port */
    LED.pGPIO = GPIOD;

    /*!< GPIO Pin Number */
    LED.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM12;

    /*!< GPIO Pin Mode */
    LED.GPIO_PinConfig.PINMODE = GPIO_MODE_OUT;

    /*!< GPIO Pin Speed */
    LED.GPIO_PinConfig.PINSPEED = GPIO_OSPEED_Fast;

    /*!< GPIO Pin Output Type */
    LED.GPIO_PinConfig.PINOPTYPE = GPIO_OPTYPE_PuPl;

    /*!< GPIO pull up and pull down control */
    LED.GPIO_PinConfig.PINPULLPDCONTROL = GPIO_PuPd_None;

    GPIO_PprlClkCtrl(GPIOD,ENABLE);

    GPIO_Init(&LED);


}


void extLedInit()
{

	/*!< GPIO handler */
	GPIO_handle_t eLED;

	memset(&eLED,0,sizeof(eLED));

	/*!< GPIO Port */
    eLED.pGPIO = GPIOD;

    /*!< GPIO Pin Number */
    eLED.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM7;

    /*!< GPIO Pin Mode */
    eLED.GPIO_PinConfig.PINMODE = GPIO_MODE_OUT;

    /*!< GPIO Pin Speed */
    eLED.GPIO_PinConfig.PINSPEED = GPIO_OSPEED_Fast;

    /*!< GPIO Pin Output Type */
    eLED.GPIO_PinConfig.PINOPTYPE = GPIO_OPTYPE_PuPl;

    /*!< GPIO pull up and pull down control */
    eLED.GPIO_PinConfig.PINPULLPDCONTROL = GPIO_PuPd_None;

    GPIO_PprlClkCtrl(GPIOD,ENABLE);

    GPIO_Init(&eLED);

}

void pushBtnInit()
{

	/*!< GPIO handler */
	GPIO_handle_t PB;

	memset(&PB,0,sizeof(PB));

	/*!< GPIO Port */
    PB.pGPIO = GPIOA;

    /*!< GPIO Pin Number */
    PB.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM0;

    /*!< GPIO Pin Mode */
    PB.GPIO_PinConfig.PINMODE = GPIO_MODE_IN;

    /*!< GPIO pull up and pull down control */
    PB.GPIO_PinConfig.PINPULLPDCONTROL = GPIO_PuPd_None; /*!<Already has a resistor to ground*/

    GPIO_PprlClkCtrl(GPIOA,ENABLE);

    GPIO_Init(&PB);

}

void extPushBtnInit()
{

	/*!<External PB on PE1*/

	GPIO_handle_t ePB;

	memset(&ePB,0,sizeof(ePB));

	/*!< GPIO Port */
    ePB.pGPIO = GPIOE;

    /*!< GPIO Pin Number */
    ePB.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM1;

    /*!< GPIO Pin Mode */
    ePB.GPIO_PinConfig.PINMODE = GPIO_MODE_IN_RE;

    /*!< GPIO pull up and pull down control */
    ePB.GPIO_PinConfig.PINPULLPDCONTROL = GPIO_PuPd_Pu; /*!<Already has a resistor to ground*/

    GPIO_PprlClkCtrl(GPIOE,ENABLE);

    GPIO_Init(&ePB);


}

void ledToggle()
{
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NUM12); /*!< Toggle PD12 */
}

void eledSetHigh()
{
	GPIO_WriteOutputPin(GPIOD,GPIO_PIN_NUM7,SET); /*!< Set  PD7 */
}

void eledSetLow()
{
	GPIO_WriteOutputPin(GPIOD,GPIO_PIN_NUM7,UNSET); /*!< Reset  PD7 */
}

void EXTI1_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PIN_NUM1);
	ledToggle();
}

int main ()
{

	ledInitPP();


#ifdef test1
	while(1)
	{
		ledToggle();
		delay();
	}
#endif

#ifdef test2
	pushBtnInit();
	while(1)
	{
		/*!
		 * @note Pressing the user button pulls the gpio pin high
		 */

		while(GPIO_ReadInputPin(GPIOA,GPIO_PIN_NUM0) == BTN_PRESSED) /*!< If pushbutton is pressed */
		{
			ledToggle();
			delay();
		}
	}
#endif

#ifdef test3
	extPushBtnInit();
	while(1)
	{
           while(GPIO_ReadInputPin(GPIOB,GPIO_PIN_NUM3) == EX_BTN_PRESSED)
           {
        	   ledToggle();
        	   delay();
           }

	}
#endif

#ifdef test4
	extLedInit();
	extPushBtnInit();
	while(1)
	{
           while(GPIO_ReadInputPin(GPIOB,GPIO_PIN_NUM3) == EX_BTN_PRESSED)
           {
        	   eledSetHigh();
        	   delay();
        	   eledSetLow();
        	   delay();
           }

	}
#endif

#ifdef test5
	extPushBtnInit();

	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI1,IRQ_PRIORITY15);

	GPIO_IRQConfig(IRQ_NUM_EXTI1,ENABLE);

#endif

	while(1);
}
