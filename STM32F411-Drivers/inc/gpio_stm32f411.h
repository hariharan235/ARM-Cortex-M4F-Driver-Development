/**
 ******************************************************************************
 * @file    gpio_stm32f411.h, file name may change
 * @author  Hariharan Gopalakrishnan,
 * @brief   STM32F411VE Device Peripheral Access Layer Header File.
 *
 *  This file contains:
 *              - Macros for GPIO pin Configurations
 *              - Macros for enabling & disabling Clock at GPIO Ports
 *              - Data Structures for GPIO Pin Configurations
 *              - API Function Prototypes
 *              - API Interrupt Prototypes
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



#ifndef INCLUDE_STM32F4XX_GPIO_STM32F411_H_
#define INCLUDE_STM32F4XX_GPIO_STM32F411_H_



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*!@brief
 *
 * Device Specific header file
 */

#include "stm32f411xe.h"



/******************************************************************************/
/*                                                                            */
/*                  Enumerations for GPIO Pin Configuration                   */
/*                                                                            */
/******************************************************************************/

/*!@brief
 *
 * @GPIO_CLK_CNTRL
 * GPIO clock enable or disable
 */

typedef enum
{
	GPIO_CLK_DISABLE,
	GPIO_CLK_ENABLE

}GPIO_clk_ctrl_t;


/*!@brief
 *
 * @GPIO_PIN_MODES
 * Possible GPIO modes
 */
typedef enum
{
	GPIO_MODE_IN,     /*!<Input Mode*/
	GPIO_MODE_OUT,    /*!<Output Mode*/
	GPIO_MODE_ALTFN,  /*!<Alternate Function Mode*/
	GPIO_MODE_ANALOG, /*!<Analog*/
	GPIO_MODE_IN_FE,  /*!<Interrupt Falling Edge*/
	GPIO_MODE_IN_RE,  /*!<Interrupt Rising Edge*/
	GPIO_MODE_IN_FRE  /*!<Interrupt Rising and Falling Edge Triggers*/

}GPIO_mode_t;


/*!@brief
 *
 * @GPIO_PIN_OTYPE
 * GPIO Pin Output Types
 */

typedef enum
{
	GPIO_OPTYPE_PuPl,/*!<Push Push Output Type*/
	GPIO_OPTYPE_OPD  /*!<Open Drain Output Type*/

}GPIO_pin_type_t;


/*!@brief
 *
 * @GPIO_PIN_OSPEED
 * GPIO Pin Output Speed
 */

typedef enum
{
	GPIO_OSPEED_Low,  /*!<Low Output Speed*/
	GPIO_OSPEED_Med,  /*!<Medium Output Speed*/
	GPIO_OSPEED_Fast, /*!<Fast Output Speed*/
	GPIO_OSPEED_High  /*!<High Output Speed*/

}GPIO_pin_speed_t;


/*!@brief
 *
 * @GPIO_PUPD_CTRL
 * GPIO Pin Pull-up and Pull-down control
 */

typedef enum
{
	GPIO_PuPd_None, /*No internal pull-up or pull-down register*/
	GPIO_PuPd_Pu,  /*Internal pull-up(40k with exceptions for some pins (refer data-sheet))*/
	GPIO_PuPd_Pd   /*Internal pull-down(40k with exceptions for some pins (refer data-sheet))*/

}GPIO_pupd_ctrl_t;

/*!@brief
 *
 * @GPIO_PINNUMBER
 * GPIO PinNumbers
 */

typedef enum
{
	GPIO_PIN_NUM0,        /*!< Pin Number 0 */
	GPIO_PIN_NUM1,        /*!< Pin Number 1 */
	GPIO_PIN_NUM2,         /*!< Pin Number 2 */
	GPIO_PIN_NUM3,         /*!< Pin Number 3 */
	GPIO_PIN_NUM4,         /*!< Pin Number 4 */
	GPIO_PIN_NUM5,         /*!< Pin Number 5 */
	GPIO_PIN_NUM6,         /*!< Pin Number 6 */
	GPIO_PIN_NUM7,         /*!< Pin Number 7 */
	GPIO_PIN_NUM8,         /*!< Pin Number 8 */
	GPIO_PIN_NUM9,         /*!< Pin Number 9 */
	GPIO_PIN_NUM10,       /*!< Pin Number 10 */
	GPIO_PIN_NUM11,       /*!< Pin Number 11 */
	GPIO_PIN_NUM12,       /*!< Pin Number 12 */
	GPIO_PIN_NUM13,       /*!< Pin Number 13 */
	GPIO_PIN_NUM14,       /*!< Pin Number 14 */
	GPIO_PIN_NUM15       /*!< Pin Number 15 */

}GPIO_pinNo_t;

/*!@brief
 *
 * @GPIO_PINSTATE
 * GPIO Pin States
 */

typedef enum
{
	GPIO_PIN_RESET,
	GPIO_PIN_SET

}GPIO_pinState_t;

/*!@brief
 *
 * @GPIO_ALT_FN
 * GPIO Alternate Functions
 */

typedef enum
{
	GPIO_PIN_AFR0,
	GPIO_PIN_AFR1,
	GPIO_PIN_AFR2,
	GPIO_PIN_AFR3,
	GPIO_PIN_AFR4,
	GPIO_PIN_AFR5,
	GPIO_PIN_AFR6,
	GPIO_PIN_AFR7,
	GPIO_PIN_AFR8,
	GPIO_PIN_AFR9,
	GPIO_PIN_AFR10,
	GPIO_PIN_AFR11,
	GPIO_PIN_AFR12,
	GPIO_PIN_AFR13,
	GPIO_PIN_AFR14,
	GPIO_PIN_AFR15

}GPIO_altfn_t;


/*!@brief
 *
 * GPIO base address to Port Code
 */

#define GPIO_PORT_CODE(x)  ((x == GPIOA) ? 0 :\
		(x == GPIOB) ? 1 :\
				(x == GPIOC) ? 2 :\
						(x == GPIOD) ? 3 :\
								(x == GPIOE) ? 4 :\
										(x == GPIOH) ? 7 :0)


/*!@brief
 *
 *
 * GPIO_IRQ_Numbers
 */

typedef enum
{
	GPIO_IRQ_NUM_EXTI0 = 6,
	GPIO_IRQ_NUM_EXTI1,
	GPIO_IRQ_NUM_EXTI2,
	GPIO_IRQ_NUM_EXTI3,
	GPIO_IRQ_NUM_EXTI4,
	GPIO_IRQ_NUM_EXTI9_5 = 23,
	GPIO_IRQ_NUM_EXTI15_10 = 40,
	GPIO_IRQ_NUM_EXTI17

}GPIO_IRQ_num_t;

/*!@brief
 *
 *
 * IRQ_Enable / IRQ_Disable
 */

typedef enum
{
	GPIO_IRQ_DISABLE,
	GPIO_IRQ_ENABLE

}GPIO_IRQ_ctrl_t;

/*!@brief
 *
 *
 * IRQ_Priority Values
 */

typedef enum
{
	GPIO_IRQ_PRIORITY0,
	GPIO_IRQ_PRIORITY1,
	GPIO_IRQ_PRIORITY2,
	GPIO_IRQ_PRIORITY3,
	GPIO_IRQ_PRIORITY4,
	GPIO_IRQ_PRIORITY5,
	GPIO_IRQ_PRIORITY6,
	GPIO_IRQ_PRIORITY7,
	GPIO_IRQ_PRIORITY8,
	GPIO_IRQ_PRIORITY9,
	GPIO_IRQ_PRIORITY10,
	GPIO_IRQ_PRIORITY11,
	GPIO_IRQ_PRIORITY12,
	GPIO_IRQ_PRIORITY13,
	GPIO_IRQ_PRIORITY14,
	GPIO_IRQ_PRIORITY15

}GPIO_IRQ_priority_t;


/******************************************************************************/
/*                                                                            */
/*                  Data Structures for GPIO Pin Configuration                */
/*                                                                            */
/******************************************************************************/


/*!@brief
 *
 *
 *Configuration Structure for GPIO pin
 */


typedef struct
{
	GPIO_pinNo_t PINNUMBER;                     /*!<GPIO pin number*/ /*!<Possible Values from @GPIO_PINNUMBER*/
	GPIO_mode_t PINMODE;                       /*!<GPIO pin mode*/    /*!<Possible Values from @GPIO_PIN_MODES*/
	GPIO_pin_speed_t PINSPEED;                      /*!<GPIO pin speed*/   /*!<Possible Values from @GPIO_PIN_OSPEED*/
	GPIO_pupd_ctrl_t PINPULLPDCONTROL;              /*!<GPIO pin pull-up and pull-down control*/ /*!<Possible Values from @GPIO_PUPD_CTRL*/
	GPIO_pin_type_t PINOPTYPE;                     /*!<GPIO output type*/ /*!<Possible Values from @GPIO_PIN_OTYPE*/
	GPIO_altfn_t PINALTFNMODE;                  /*!<GPIO Alternate Function mode*/

}GPIO_PinConfig_t;


/*! @brief
 *
 *
 *  GPIO Configuration Handle
 */

typedef struct
{
	GPIO_TypeDef* pGPIO;                                    /*!<Holds the Base Address of the GPIO Port*/
	GPIO_PinConfig_t GPIO_PinConfig;                                   /*!<Handle to the pin configuration settings*/

}GPIO_handle_t;



/******************************************************************************/
/*                                                                            */
/*                      Driver API Function Prototypes                        */
/*                                                                            */
/******************************************************************************/



/*! @brief
 *
 *
 * API's supported by this driver
 */


/*!< Peripheral clock setup*/

void GPIO_PprlClkCtrl(GPIO_TypeDef* pGPIO , GPIO_clk_ctrl_t setState);                                    /*!<Function to control peripheral clock*/


/*!< GPIO initialization Functions*/

void GPIO_Init(GPIO_handle_t *pGPIOHandle);                                                        /*!<Function to initialize GPIO port*/
void GPIO_DeInit(GPIO_TypeDef* pGPIO);                                                             /*!<Function to disable GPIO port*/

/*!< GPIO read and write Functions*/

uint8_t GPIO_ReadInputPin(GPIO_TypeDef* pGPIO , GPIO_pinNo_t pinNumber);                               /*!<Function to read from a GPIO pin*/
uint16_t GPIO_ReadInputPort(GPIO_TypeDef* pGPIO);                                                  /*!<Function to read from Input Port*/
void GPIO_WriteOutputPin(GPIO_TypeDef* pGPIO , GPIO_pinNo_t pinNumber , GPIO_pinState_t setState);              /*!<Function to write to a GPIO pin*/
void GPIO_WriteOutputPort(GPIO_TypeDef* pGPIO , uint16_t Val );                                    /*!<Function to write to GPIO port*/
void GPIO_ToggleOutputPin(GPIO_TypeDef* pGPIO ,GPIO_pinNo_t  pinNumber);                              /*!<Function to toggle GPIO output Pin*/


/*!< GPIO Interrupt Control Function*/

void GPIO_IRQConfig(GPIO_IRQ_num_t IRQNumber ,GPIO_IRQ_ctrl_t setState);                                        /*!<Function to configure GPIO Interrupts*/
void GPIO_IRQPriorityConfig(GPIO_IRQ_num_t IRQNumber , GPIO_IRQ_priority_t IRQPriority);                             /*!<Function to configure Interrupt priority */
void GPIO_IRQHandler(GPIO_pinNo_t pinNumber);                                                          /*!<Function to process triggered interrupts*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_STM32F4XX_GPIO_STM32F411_H_ */
