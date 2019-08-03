/**
 ******************************************************************************
 * @file    gpio.c, file name will change
 * @author  Aditya Mall,
 * @brief   GPIO API Layer Header File.
 *
 *  This file contains:
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



#ifndef API_GPIO_H_
#define API_GPIO_H_


/*
 * @brief Standard Header file
 */
#include <stdint.h>


/******************************************************************************/
/*                                                                            */
/*                  Macros for GPIO API Initialization                        */
/*                                                                            */
/******************************************************************************/


/*
 * @brief GPIO SET and DISABLE Macros
 */
#define HIGH      EN
#define LOW       DS


/*
 * @brief GPIO Direction
 */
#define INPUT    0x00UL
#define OUTPUT   0x01UL


/*
 * @brief GPIO Pin Mode
 */
#define DIGITAL  0x02UL
#define ANALOG   0x03UL


/*
 * @brief GPIO Pin Type
 */
#define PULLUP   0x04UL
#define PULLDOWN 0x05UL

/*
 * @brief GPIO Open Drain Select
 */
#define OPEN_DRAIN 0x06UL


/*
 * @brief GPIO Drive Select
 */
#define DRIVE_2MA 0x07UL
#define DRIVE_4MA 0x08UL
#define DRIVE_8MA 0x09UL


/******************************************************************************/
/*                                                                            */
/*                       API Function Prototypes                              */
/*                                                                            */
/******************************************************************************/



/*
 * @brief   Intializes GPIO pin.
 * @param   *port_pin : Name of the GPIO Pin
 * @param
 * @param
 * @retval  None.
 */
int8_t pinMode(char *port_pin, uint8_t pinDirection,...);



/*
 * @brief   Write to GPIO Pin
 * @param   *port_pin : Name of the GPIO pin/
 * @param   pinState  : Value of State to set on the pin.
 * @retval  int8_t    : Success = 0, Failure = -1.
 */
int8_t digitalWrite(char *port_pin, uint8_t pinState);



/*
 * @brief   Read from GPIO pin (Blocking function)
 * @param   *port_pin : Name of the GPIO pin
 * @retval  int8_t    : Success = Return value from the pin, Failure = -1
 */
int8_t digitalRead(char *port_pin);

#endif /* API_INC_GPIO_H_ */
