/**
 ******************************************************************************
 * @file    spi_stm32f411.h, file name may change
 * @author  Hariharan Gopalakrishnan,
 * @brief   STM32F411VE Device Peripheral Access Layer Header File.
 *
 *  This file contains:
 *              - Enumerations for SPI peripheral Configurations
 *              - Enumerations for enabling & disabling SPI peripheral clocks
 *              - Data Structures for SPI peripheral Configurations
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


#ifndef INC_SPI_STM32F411_H_
#define INC_SPI_STM32F411_H_

#include <stdint.h>
#include <stdbool.h>


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
 /*                  Enumerations for SPI Peripheral Configuration             */
 /*                                                                            */
 /******************************************************************************/

 /*!@brief
  *
  * SPI peripheral clock enable / disable
  */

typedef enum
{
	SPI_CLK_ENABLE,
	SPI_CLK_DISABLE

}SPI_PerifClk_t;


/*!@brief
 *
 * SPI device modes
 */

typedef enum
{
	SPI_MODE_SLAVE,
	SPI_MODE_MASTER

}SPI_DeviceMode_t;

/*!@brief
 *
 * SPI Bus Configuration Types
 */

typedef enum
{
	SPI_BUS_CONF_FD,
	SPI_BUS_CONF_HD_RX,
	SPI_BUS_CONF_HD_TX,
	SPI_BUS_CONF_SM_RX

}SPI_BusConfig_t;

/*!@brief
 *
 * SPI Serial Clock Speed
 */

typedef enum
{
	SPI_PCLK_PDIV_2,
	SPI_PCLK_PDIV_4,
	SPI_PCLK_PDIV_8,
	SPI_PCLK_PDIV_16,
	SPI_PCLK_PDIV_32,
	SPI_PCLK_PDIV_64,
	SPI_PCLK_PDIV_128,
	SPI_PCLK_PDIV_256

}SPI_SclkSpeed_t;

/*!@brief
 *
 * SPI Date Frame Format
 */

typedef enum
{
	SPI_DFF_8BIT,
	SPI_DFF_16BIT

}SPI_DFF_t;

/*!@brief
 *
 * SPI Clock Polarity (CPOL)
 */

typedef enum
{
	SPI_CPOL_IDLE_LOW,
	SPI_CPOL_IDLE_HIGH

}SPI_CPOL_t;

/*!@brief
 *
 * SPI Clock Phase (CPHA)
 */

typedef enum
{
	SPI_CPHA_LEADINGEDGE,
	SPI_CPHA_TRAILINGEDGE

}SPI_CPHA_t;

/*!@brief
 *
 * SPI Software Slave Management
 */

typedef enum
{
	SPI_SSM_HW,
	SPI_SSM_SW

}SPI_SSM_t;



/* @brief
 * Configuration Structure for SPIx
 * */


typedef struct
{
	SPI_DeviceMode_t DEVICEMODE;
	SPI_BusConfig_t BUSCONFIG;
	SPI_SclkSpeed_t SCLKSPEED;
	SPI_DFF_t DFF;
	SPI_CPOL_t CPOL;
	SPI_CPHA_t CPHA;
	SPI_SSM_t SSM;

}SPI_PerifConfig_t;


/*! @brief
 *  GPIO Configuration Handle
 */

typedef struct
{
	SPI_TypeDef *pSPIx;                /*!<Holds the Base Address of the SPIx Peripheral>*/
	SPI_PerifConfig_t SPIConfig;       /*!<Handle to the SPI peripheral configuration settings>*/

}SPI_handle_t;


/*! @brief
 *
 *  API's supported by this driver
 */


/*!< Peripheral clock setup>*/

void SPI_PprlClkCtrl(SPI_TypeDef* pSPI , SPI_PerifClk_t setState);    /*!<Function to control peripheral clock>*/

/*!< SPI initialization Functions>*/

void SPI_Init(SPI_handle_t *pSPIHandle);                                                        /*!<Function to initialize SPI peripheral>*/
void SPI_DeInit(SPI_TypeDef* pSPI);                                                             /*!<Function to disable SPI peripheral>*/




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_SPI_STM32F411_H_ */
