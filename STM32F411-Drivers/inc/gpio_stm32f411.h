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


 /*!@brief
  *
  * Device Specific header file
  */

#include "stm32f411xe.h"


 /*!@brief
  *
  * Generic Macros
  */

 #define ENABLE 1
 #define DISABLE 0
 #define SET ENABLE
 #define UNSET DISABLE
 #define GPIO_PIN_SET SET
 #define GPIO_PIN_RESET UNSET


/******************************************************************************/
/*                                                                            */
/*                  Macros for GPIO Pin Configuration                         */
/*                                                                            */
/******************************************************************************/



/*!@brief
 *
 * @GPIO_PIN_MODES
 * Possible GPIO modes
 */

#define GPIO_MODE_IN 0  /*!<Input Mode*/
#define GPIO_MODE_OUT 1 /*!<Output Mode*/
#define GPIO_MODE_ALTFN 2 /*!<Alternate Function Mode*/
#define GPIO_MODE_ANALOG 3 /*!<Analog*/
#define GPIO_MODE_IN_FE 4 /*!<Interrupt Falling Edge*/
#define GPIO_MODE_IN_RE 5 /*!<Interrupt Rising Edge*/
#define GPIO_MODE_IN_FRE 6 /*!<Interrupt Rising and Falling Edge Triggers*/


/*!@brief
 *
 * @GPIO_PIN_OTYPE
 * GPIO Pin Output Types
 */

#define GPIO_OPTYPE_PuPl 0/*!<Push Push Output Type*/
#define GPIO_OPTYPE_OPD  1/*!<Open Drain Output Type*/


/*!@brief
 *
 * @GPIO_PIN_OSPEED
 * GPIO Pin Output Speed
 */

#define GPIO_OSPEED_Low 0 /*!<Low Output Speed*/
#define GPIO_OSPEED_Med 1 /*!<Medium Output Speed*/
#define GPIO_OSPEED_Fast 2 /*!<Fast Output Speed*/
#define GPIO_OSPEED_High 3 /*!<High Output Speed*/


/*!@brief
 *
 * @GPIO_PUPD_CTRL
 * GPIO Pin Pull-up and Pull-down control
 */

#define GPIO_PuPd_None 0 /*No internal pull-up or pull-down register*/
#define GPIO_PuPd_Pu 1  /*Internal pull-up(40k with exceptions for some pins (refer data-sheet))*/
#define GPIO_PuPd_Pd 2  /*Internal pull-down(40k with exceptions for some pins (refer data-sheet))*/


/*!@brief
 *
 * @GPIO_PINNUMBER
 * GPIO PinNumbers
 */

#define GPIO_PIN_NUM0 0         /*!< Pin Number 0 */
#define GPIO_PIN_NUM1 1         /*!< Pin Number 1 */
#define GPIO_PIN_NUM2 2         /*!< Pin Number 2 */
#define GPIO_PIN_NUM3 3         /*!< Pin Number 3 */
#define GPIO_PIN_NUM4 4         /*!< Pin Number 4 */
#define GPIO_PIN_NUM5 5         /*!< Pin Number 5 */
#define GPIO_PIN_NUM6 6         /*!< Pin Number 6 */
#define GPIO_PIN_NUM7 7         /*!< Pin Number 7 */
#define GPIO_PIN_NUM8 8         /*!< Pin Number 8 */
#define GPIO_PIN_NUM9 9         /*!< Pin Number 9 */
#define GPIO_PIN_NUM10 10       /*!< Pin Number 10 */
#define GPIO_PIN_NUM11 11       /*!< Pin Number 11 */
#define GPIO_PIN_NUM12 12       /*!< Pin Number 12 */
#define GPIO_PIN_NUM13 13       /*!< Pin Number 13 */
#define GPIO_PIN_NUM14 14       /*!< Pin Number 14 */
#define GPIO_PIN_NUM15 15       /*!< Pin Number 15 */


/*!@brief
 *
 * @GPIO_ALT_FN
 * GPIO Alternate Functions
 */

//#define GPIO_PIN_AFR0 0
//#define GPIO_PIN_AFR1 1
//#define GPIO_PIN_AFR2 2
//#define GPIO_PIN_AFR3 3
//#define GPIO_PIN_AFR4 4
//#define GPIO_PIN_AFR5 5
//#define GPIO_PIN_AFR6 6
//#define GPIO_PIN_AFR7 7
//#define GPIO_PIN_AFR8 8
//#define GPIO_PIN_AFR9 9
//#define GPIO_PIN_AFR10 10
//#define GPIO_PIN_AFR11 11
//#define GPIO_PIN_AFR12 12
//#define GPIO_PIN_AFR13 13
//#define GPIO_PIN_AFR14 14
//#define GPIO_PIN_AFR15 15


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
 * IRQ_Numbers
 */

#define IRQ_NUM_EXTI0 6
#define IRQ_NUM_EXTI1 7
#define IRQ_NUM_EXTI2 8
#define IRQ_NUM_EXTI3 9
#define IRQ_NUM_EXTI4 10
#define IRQ_NUM_EXTI9_5 23
#define IRQ_NUM_EXTI15_10 40
#define IRQ_NUM_EXTI17 41

/*!@brief
 *
 *
 * IRQ_Priority Values
 */

#define IRQ_PRIORITY0  0
#define IRQ_PRIORITY1  1
#define IRQ_PRIORITY2  2
#define IRQ_PRIORITY3  3
#define IRQ_PRIORITY4  4
#define IRQ_PRIORITY5  5
#define IRQ_PRIORITY6  6
#define IRQ_PRIORITY7  7
#define IRQ_PRIORITY8  8
#define IRQ_PRIORITY9  9
#define IRQ_PRIORITY10 10
#define IRQ_PRIORITY11 11
#define IRQ_PRIORITY12 12
#define IRQ_PRIORITY13 13
#define IRQ_PRIORITY14 14
#define IRQ_PRIORITY15 15



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
	uint8_t PINNUMBER;                     /*!<GPIO pin number*/ /*!<Possible Values from @GPIO_PINNUMBER*/
	uint8_t PINMODE;                       /*!<GPIO pin mode*/    /*!<Possible Values from @GPIO_PIN_MODES*/
	uint8_t PINSPEED;                      /*!<GPIO pin speed*/   /*!<Possible Values from @GPIO_PIN_OSPEED*/
	uint8_t PINPULLPDCONTROL;              /*!<GPIO pin pull-up and pull-down control*/ /*!<Possible Values from @GPIO_PUPD_CTRL*/
	uint8_t PINOPTYPE;                     /*!<GPIO output type*/ /*!<Possible Values from @GPIO_PIN_OTYPE*/
	uint8_t PINALTFNMODE;                  /*!<GPIO Alternate Function mode*/

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

void GPIO_PprlClkCtrl(GPIO_TypeDef* pGPIO , uint8_t setState);                                     /*!<Function to control peripheral clock*/


/*!< GPIO initialization Functions*/

void GPIO_Init(GPIO_handle_t *pGPIOHandle);                                                        /*!<Function to initialize GPIO port*/
void GPIO_DeInit(GPIO_TypeDef* pGPIO);                                                             /*!<Function to disable GPIO port*/

/*!< GPIO read and write Functions*/

uint8_t GPIO_ReadInputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber);                                /*!<Function to read from a GPIO pin*/
uint16_t GPIO_ReadInputPort(GPIO_TypeDef* pGPIO);                                                  /*!<Function to read from Input Port*/
void GPIO_WriteOutputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber , uint8_t setState);              /*!<Function to write to a GPIO pin*/
void GPIO_WriteOutputPort(GPIO_TypeDef* pGPIO , uint16_t Val );                                    /*!<Function to write to GPIO port*/
void GPIO_ToggleOutputPin(GPIO_TypeDef* pGPIO , uint8_t pinNumber);                                /*!<Function to toggle GPIO output Pin*/


/*!< GPIO Interrupt Control Function*/

void GPIO_IRQConfig(uint8_t IRQNumber , uint8_t setState);                                         /*!<Function to configure GPIO Interrupts*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint8_t IRQPriority);                              /*!<Function to configure Interrupt priority */
void GPIO_IRQHandler(uint8_t pinNumber);                                                           /*!<Function to process triggered interrupts*/



#endif /* INCLUDE_STM32F4XX_GPIO_STM32F411_H_ */
