/**
 ******************************************************************************
 * @file    gpio.c, file name will change
 * @author  Aditya Mall,
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
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



/******************************************************************************/
/*                                                                            */
/*                       API Function Prototypes                              */
/*                                                                            */
/******************************************************************************/


int8_t pinMode(char *pin_port, uint8_t pinDirection,...);




#endif /* API_INC_GPIO_H_ */
