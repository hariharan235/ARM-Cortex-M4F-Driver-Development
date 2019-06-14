/**
 ******************************************************************************
 * @file    gpio_stm32f411.c, file name will change
 * @author  Hariharan Gopalakrishnan,
 * @brief   STM32F411VE Device Peripheral Access Layer Source File.
 *
 *  This file contains:
 *              - Helper functions for Driver exposed API functions
 *              - Driver exposed APIs, for GPIO
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 Aditya Mall, HariHaran Gopalakrishnan </center></h2>
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */



#include <assert.h>

/*!< GPIO driver header file*/

#include "gpio_stm32f411.h"


/******************************************************************************/
/*                                                                            */
/*                       Driver API Functions                                 */
/*                                                                            */
/******************************************************************************/


/*!< Peripheral clock setup*/

/*
 * @fn               - GPIO_PprlClkCtrl
 * @brief            - Function to enable/disable peripheral clock for the GPIO port
 * @param[in]        - Base Address of the GPIO peripheral
 * @param[in]        - ENABLE / DISABLE macros
 * @return           - None
 * @note             - None
 */

void GPIO_PprlClkCtrl(GPIO_TypeDef* pGPIO , uint8_t setState)
{

	assert(pGPIO);
	assert((setState==ENABLE || setState == DISABLE));
	if(setState == ENABLE)
	{

		if(pGPIO == GPIOA)
		{
			RCC->AHB1ENR &= ~(1 << 0);
			RCC->AHB1ENR |= (1 << 0);
		}
		else if(pGPIO == GPIOB)
		{
			RCC->AHB1ENR &= ~(1 << 1);
			RCC->AHB1ENR |= (1 << 1);
		}
		else if(pGPIO == GPIOC)
		{
			RCC->AHB1ENR &= ~(1 << 2);
			RCC->AHB1ENR |= (1 << 2);
		}
		else if(pGPIO == GPIOD)
		{
			RCC->AHB1ENR &= ~(1 << 3);
			RCC->AHB1ENR |= (1 << 3);
		}
		else if(pGPIO == GPIOE)
		{
			RCC->AHB1ENR &= ~(1 << 4);
			RCC->AHB1ENR |= (1 << 4);
		}
		else if(pGPIO == GPIOH)
		{
			RCC->AHB1ENR &= ~(1 << 7);
			RCC->AHB1ENR |= (1 << 7);
		}
		else
			assert(0); //Error
	}
	else
	{
		if(pGPIO == GPIOA)
		{
			RCC->AHB1ENR &= ~(1 << 0);
		}
		else if(pGPIO == GPIOB)
		{
			RCC->AHB1ENR &= ~(1 << 1);
		}
		else if(pGPIO == GPIOC)
		{
			RCC->AHB1ENR &= ~(1 << 2);
		}
		else if(pGPIO == GPIOD)
		{
			RCC->AHB1ENR &= ~(1 << 3);
		}
		else if(pGPIO == GPIOE)
		{
			RCC->AHB1ENR &= ~(1 << 4);
		}
		else if(pGPIO == GPIOH)
		{
			RCC->AHB1ENR &= ~(1 << 7);
		}
		else
		{
			assert(0); //Error
		}
	}

}


/*!< GPIO initialization Functions*/

/*
 * @fn               - GPIO_Init
 * @brief            - Function to initialize GPIO port
 * @param[in]        - Pointer to the GPIO configuration structure
 * @return           - None
 * @note             - Corresponding peripheral clock must be enabled using GPIO_PprlClkCtrl()
 */

void GPIO_Init(GPIO_handle_t *pGPIOHandle)
{

	assert(pGPIOHandle);

	/*!< Configure GPIO Pin Mode */

    if(pGPIOHandle->GPIO_PinConfig.PINMODE <= GPIO_MODE_ANALOG)
    {
    	/*!< Non-Interrupt Configuration */

    	pGPIOHandle->pGPIO->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.PINNUMBER); /*!< Clear the bit fields*/
    	pGPIOHandle->pGPIO->MODER |= ((pGPIOHandle->GPIO_PinConfig.PINMODE) << (2 * pGPIOHandle->GPIO_PinConfig.PINNUMBER)); /*!< Set GPIO Pin Mode */

    }
    else
    {
    	if(pGPIOHandle->GPIO_PinConfig.PINMODE == GPIO_MODE_IN_FE)
    	{

    		/*!< Configure FTSR */

    		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);

    		/*!< Clear RTSR bit */

    		EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);
    	}
    	else if(pGPIOHandle->GPIO_PinConfig.PINMODE == GPIO_MODE_IN_RE)
    	{
    		/*!< Configure RTSR */

    		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);

    		/*!< Clear FTSR bit */

    		EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);
    	}
    	else if(pGPIOHandle->GPIO_PinConfig.PINMODE == GPIO_MODE_IN_FRE)
    	{
    		/*!< Configure FTSR */

    		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);

    		/*!< Configure RTSR */

    		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);
    	}
    	else
    		assert(0);


    	/*!< System configuration clock */

    	RCC->APB2ENR |= (1 << 14);

    	/*!< EXTI control */

    	SYSCFG->EXTICR[(pGPIOHandle->GPIO_PinConfig.PINNUMBER >> 2)] |= (GPIO_PORT_CODE(pGPIOHandle->pGPIO) << (4 *(pGPIOHandle->GPIO_PinConfig.PINNUMBER & 3)));

    	/*!< Enable Interrupt delivery to NVIC */

    	EXTI->IMR |= (0x01 << pGPIOHandle->GPIO_PinConfig.PINNUMBER);

    }

    /*!< Configure GPIO Pin Speed */

    pGPIOHandle->pGPIO->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.PINNUMBER); /*!< Clear the bit fields*/
    pGPIOHandle->pGPIO->OSPEEDR |= ((pGPIOHandle->GPIO_PinConfig.PINSPEED) << (2 * pGPIOHandle->GPIO_PinConfig.PINNUMBER)); /*!< Set GPIO Pin Output Speed */


    /*!< Configure GPIO Pull-up and Pull-down resistors for GPIO pin */

    pGPIOHandle->pGPIO->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.PINNUMBER); /*!< Clear the bit fields*/
    pGPIOHandle->pGPIO->PUPDR |= ((pGPIOHandle->GPIO_PinConfig.PINPULLPDCONTROL) << (2 * pGPIOHandle->GPIO_PinConfig.PINNUMBER)); /*!< Set Pull-up or Pull-down for GPIO Pin */

    /*!< Configure GPIO Output Type  */

    pGPIOHandle->pGPIO->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.PINNUMBER); /*!< Clear the bit fields*/
    pGPIOHandle->pGPIO->OTYPER |= ((pGPIOHandle->GPIO_PinConfig.PINOPTYPE) << (pGPIOHandle->GPIO_PinConfig.PINNUMBER)); /*!< Set GPIO Pin Output Type */

    /*!< Configure Alternate Function */

    if(pGPIOHandle->GPIO_PinConfig.PINMODE == GPIO_MODE_ALTFN)
    {

    	if(pGPIOHandle->GPIO_PinConfig.PINNUMBER <= 7)
    	{
    		pGPIOHandle->pGPIO->AFR[0] &= ~(0x0F << (4 * pGPIOHandle->GPIO_PinConfig.PINNUMBER)); /*!< Clear the bit fields*/
    	    pGPIOHandle->pGPIO->AFR[0] |= ((pGPIOHandle->GPIO_PinConfig.PINALTFNMODE) << (4 *pGPIOHandle->GPIO_PinConfig.PINNUMBER));
    	}
    	else
    	{
    		pGPIOHandle->pGPIO->AFR[1] &= ~(0x0F << (4 *(pGPIOHandle->GPIO_PinConfig.PINNUMBER & 3))); /*!< Clear the bit fields*/
    		pGPIOHandle->pGPIO->AFR[1] |= ((pGPIOHandle->GPIO_PinConfig.PINALTFNMODE) << (4 *(pGPIOHandle->GPIO_PinConfig.PINNUMBER & 3)));

    	}

    }

}


/*
 * @fn               - GPIO_DeInit
 * @brief            - Function to disable GPIO port
 * @param[in]        - Base Address of the GPIO peripheral
 * @return           - None
 * @note             - None
 */

void GPIO_DeInit(GPIO_TypeDef* pGPIO)
{

	assert(pGPIO);

	if(pGPIO == GPIOA)
	{
			RCC->AHB1RSTR |= (1 << 0);
			RCC->AHB1RSTR &= ~(1 << 0);
	}
	else if(pGPIO == GPIOB)
	{
			RCC->AHB1RSTR |= (1 << 1);
			RCC->AHB1RSTR &= ~(1 << 1);
	}
	else if(pGPIO == GPIOC)
	{
			RCC->AHB1RSTR |= (1 << 2);
			RCC->AHB1RSTR &= ~(1 << 2);
	}
	else if(pGPIO == GPIOD)
	{
			RCC->AHB1RSTR |= (1 << 3);
			RCC->AHB1RSTR &= ~(1 << 3);
	}
	else if(pGPIO == GPIOE)
	{
			RCC->AHB1RSTR |= (1 << 4);
			RCC->AHB1RSTR &= ~(1 << 4);
	}
	else if(pGPIO == GPIOH)
	{
			RCC->AHB1RSTR |= (1 << 7);
			RCC->AHB1RSTR &= ~(1 << 7);
	}
	else
	{
			assert(0); //Error
	}

}


/*!< GPIO read and write Functions*/

/*
 * @fn               - GPIO_ReadInputPin
 * @brief            - Function to read from a GPIO pin
 * @param[in]        - Base Address of the GPIO peripheral
 * @param[in]        - 8-bit PinNumber on the GPIO port
 * @return           - Returns the current level (High or low) of the pin
 * @note             - Corresponding pin must be initialized as input using GPIO_Init()
 */

uint8_t GPIO_ReadInputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber)
{

	uint8_t val = 0;
	assert(pGPIO);
	assert(pinNumber <= GPIO_PIN_NUM15);

	val = (uint8_t)((pGPIO->IDR >> pinNumber ) & 0x00000001);

	return val;
}


/*
 * @fn               - GPIO_ReadInputPort
 * @brief            - Function to read from GPIO input register
 * @param[in]        - Base Address of the GPIO peripheral
 * @return           - Returns the value (16-bit) stored in the GPIO input register
 * @note             - Corresponding peripheral clock must be enabled using GPIO_PprlClkCtrl()
 */

uint16_t GPIO_ReadInputPort(GPIO_TypeDef* pGPIO)
{
	uint16_t val = 0;
	assert(pGPIO);

	val = (uint16_t)pGPIO->IDR;

	return val;

}


/*
 * @fn               - GPIO_WriteOutputPin
 * @brief            - Function to write to GPIO pin
 * @param[in]        - Base Address of the GPIO peripheral
 * @param[in]        - 8-bit PinNumber on the GPIO port
 * @param[in]        - GPIO_PIN_SET/GPIO_PIN_UNSET Macros
 * @return           - None
 * @note             - Corresponding pin must be initialized as output using GPIO_Init()
 */

void GPIO_WriteOutputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber , uint8_t setState)
{

	assert(pGPIO);
	assert(pinNumber <= GPIO_PIN_NUM15);
	assert((setState == 0 || setState == 1));

	if(setState == GPIO_PIN_SET)
	{
		pGPIO->BSRR |= (1 << pinNumber); //Set the bit
	}
	else
	{
		pGPIO->BSRR &= ~(1 << pinNumber); //Clear the Bit set register
		pGPIO->BSRR |= (1 << (pinNumber + 16)); //Reset the bit
	}

}


/*
 * @fn               - GPIO_WriteOutputPort
 * @brief            - Function to write to GPIO output register
 * @param[in]        - Base Address of the GPIO peripheral
 * @param[in]        - 16-bit Value to be written to the register
 * @return           - None
 * @note             - Corresponding peripheral clock must be enabled using GPIO_PprlClkCtrl()
 */

void GPIO_WriteOutputPort(GPIO_TypeDef* pGPIO , uint16_t Val )
{
	assert(pGPIO);

	pGPIO->ODR |= Val;

}


/*
 * @fn               - GPIO_ToggleOutputPin
 * @brief            - Function to toggle the output of GPIO pin
 * @param[in]        - Base Address of the GPIO peripheral
 * @param[in]        - 8-bit PinNumber on the GPIO port
 * @return           - None
 * @note             - Corresponding pin must be initialized as output using GPIO_Init()
 */

void GPIO_ToggleOutputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber)
{

	assert(pGPIO);
	assert(pinNumber <= GPIO_PIN_NUM15);

	pGPIO->ODR ^= (1 << pinNumber);


}


/*!< GPIO Interrupt Control Function*/

/*
 * @fn               - GPIO_IRQConfig
 * @brief            - Function to configure GPIO interrupts
 * @param[in]        - 8-bit Interrupt Request Number
 * @param[in]        - ENABLE/DISABLE Macros
 * @return           - None
 * @note             - EXTI module must be configured using GPIO_Init()
 */

void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t setState)
{
	    assert(IRQNumber < 86);
	    assert((setState == ENABLE || setState == DISABLE));
		if(setState == ENABLE)
		{
               if(IRQNumber <= 31)
               {
            	   NVIC->ISER[0] |= (1 << IRQNumber);
               }
               else if((IRQNumber > 31 && IRQNumber <= 63))
               {
            	   NVIC->ISER[1] |= (1 << (IRQNumber & 31));
               }
               else
            	   NVIC->ISER[2] |= (1 << (IRQNumber & 31));
		}
		else
		{
            if(IRQNumber <= 31)
            {
         	   NVIC->ICER[0] |= (1 << IRQNumber);
            }
            else if((IRQNumber > 31 && IRQNumber <= 63))
            {
         	   NVIC->ICER[1] |= (1 << (IRQNumber & 31));
            }
            else
         	   NVIC->ICER[2] |= (1 << (IRQNumber & 31));

		}

}


/*
 * @fn               - GPIO_IRQPriorityConfig
 * @brief            - Function to configure GPIO interrupt Priority
 * @param[in]        - 8-bit Interrupt Request Number
 * @param[in]        - 8-bit Interrupt Priority Number
 * @return           - None
 * @note             - Interrupt must be enabled using GPIO_IRQConfig()
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority)
{
	assert(IRQPriority <= 15);
	NVIC->IP[IRQNumber] |= (IRQPriority << __NVIC_PRIO_BITS);

}


/*
 * @fn               - GPIO_IRQHandler
 * @brief            - Function to process triggered interrupts
 * @param[in]        - 8-bit PinNumber on GPIO port
 * @return           - None
 * @note             - Interrupt must be enabled using GPIO_IRQConfig()
 */

void GPIO_IRQHandler(uint8_t pinNumber)
{
	assert(pinNumber <= GPIO_PIN_NUM15);

	/*!< If Interrupt occurred*/
	if(EXTI->PR & (1 << pinNumber))
	{
		/*!< Clear the pending interrupt */
		EXTI->PR |= (1 << pinNumber);
	}

}
