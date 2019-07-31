/**
 ******************************************************************************
 * @file    gpio.c, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
 *  TODO complete details
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 Aditya Mall </center></h2>
 *
 * TODO Add license, add your name as you make changes
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



/*
 * @brief Standard Header and driver specific header files
 */
#include "gpio_tm4c123gh6pm.h"
#include "gpio.h"
#include <stdarg.h>
#include <string.h>


/******************************************************************************/
/*                                                                            */
/*                             API Functions                                  */
/*                                                                            */
/******************************************************************************/



static inline void _hf_setPinDefaults(GPIO_HANDLE_T *gpioPin)
{
    gpioPin->gpioPinConfig.DRIVE      = GPIO_DR2R;
    gpioPin->gpioPinConfig.PINMODE    = GPIO_DEN_ENABLE;
    gpioPin->gpioPinConfig.PULLUPDOWN = GPIO_NOPUPD;
    gpioPin->gpioPinConfig.OPENDRAIN  = GPIO_ODDR_DISABLE;
}


int8_t pinMode(char *port_pin, uint8_t pinDirection,...)
{

    va_list gpioArgs;

    GPIO_HANDLE_T gpioPin;

    uint8_t pinNumber = 0;
    uint8_t argValue  = 0;
    uint8_t numOfArgs = 2;
    uint8_t loopVar   = 0;


    va_start(gpioArgs, pinDirection);


    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOA;
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOB;
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOC;
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOD;
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOE;
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        gpioPin.pGPIOx = GPIOF;
    }
    else
    {
        return -1;
    }


    pinNumber = port_pin[2] - 48;

    gpioPin.gpioPinConfig.PINNUMBER = pinNumber;
    gpioPin.gpioPinConfig.DIRECTION = pinDirection;


    _hf_setPinDefaults(&gpioPin);


    while(loopVar < numOfArgs)
    {
        argValue = va_arg(gpioArgs, int);

        switch(argValue)
        {

        case DIGITAL:
            gpioPin.gpioPinConfig.PINMODE = GPIO_DEN_ENABLE;
            break;

        case ANALOG:
            gpioPin.gpioPinConfig.PINMODE = GPIO_AMSEL_ENABLE;
            break;

        case PULLUP:
            gpioPin.gpioPinConfig.PULLUPDOWN = GPIO_PUR_ENABLE;
            break;

        case PULLDOWN:
            gpioPin.gpioPinConfig.PULLUPDOWN = GPIO_PDR_ENABLE;
            break;

        case OPEN_DRAIN:
            gpioPin.gpioPinConfig.OPENDRAIN = GPIO_ODDR_ENABLE;
            break;

        }

        loopVar++;
    }

    // TODO, this should be a function pointer
    GPIO_Init(&gpioPin);


    va_end(gpioArgs);

    return 0;
}





int8_t digitalWrite(char *port_pin, uint8_t pinState)
{

    uint8_t pinNumber = 0;

    pinNumber = port_pin[2] - 48;

    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOA, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOB, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOC, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOD, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOE, pinNumber, pinState);
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        GPIO_WriteToPin(GPIOF, pinNumber, pinState);
    }
    else
    {
        return -1;
    }

    return 0;
}



uint8_t digitalRead(char *port_pin)
{
    uint8_t returnValue = 0;
    uint8_t pinNumber  = 0;

    pinNumber = port_pin[2] - 48;


    if( (strncmp(port_pin,"pa",2 ) == 0) || (strncmp(port_pin,"PA",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOA, pinNumber);
    }
    else if( (strncmp(port_pin,"pb",2 ) == 0) || (strncmp(port_pin,"PB",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOB, pinNumber);
    }
    else if( (strncmp(port_pin,"pc",2 ) == 0) || (strncmp(port_pin,"PC",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOC, pinNumber);
    }
    else if( (strncmp(port_pin,"pd",2 ) == 0) || (strncmp(port_pin,"PD",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOD, pinNumber);
    }
    else if( (strncmp(port_pin,"pe",2 ) == 0) || (strncmp(port_pin,"PE",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOE, pinNumber);
    }
    else if( (strncmp(port_pin,"pf",2 ) == 0) || (strncmp(port_pin,"PF",2 ) == 0) )
    {
        returnValue = GPIO_ReadFromPin(GPIOF, pinNumber);
    }
    else
    {
        return 0;
    }

    return returnValue;

}


