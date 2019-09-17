/**
 ******************************************************************************
 * @file    spi_tm4c123gh6pm.h, file name will change
 * @author  Aditya Mall,Hariharan Gopalakrishnan
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *  This file contains:
 *  TODO add details
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 Aditya Mall,Hariharan Gopalakrishnan </center></h2>
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


#ifndef SPI_TM4C123GH6PM_H_
#define SPI_TM4C123GH6PM_H_



#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

 /*!@brief
  *
  * Device Specific header file
  */

#ifdef PART_TM4C123GH6PM
#include "main_tm4c123gh6pm.h"
#elif
#error "Target Board is TM4C123GH6PM!"
#endif

 /******************************************************************************/
 /*                                                                            */
 /*                  Defines for SPI Peripheral Registers                      */
 /*                                                                            */
 /******************************************************************************/

 /*!@brief
  *
  * SSI RCGC Register Modules
  */

#define SSI_RCGC_MODULE0 (1U << 0)
#define SSI_RCGC_MODULE1 (1U << 1)
#define SSI_RCGC_MODULE2 (1U << 2)
#define SSI_RCGC_MODULE3 (1U << 3)


 /******************************************************************************/
 /*                                                                            */
 /*                  Enumerations for SSI Peripheral Configuration             */
 /*                                                                            */
 /******************************************************************************/

 /*!@brief
  *
  * SSI peripheral clock enable / disable
  */

typedef enum
{
    SSI_CLK_DISABLE,
    SSI_CLK_ENABLE

}ssi_perifclk_t;


/*!@brief
 *
 * SSI peripheral enable / disable
 */

typedef enum
{
    SSI_PORT_DISABLE,
    SSI_PORT_ENABLE

}ssi_portctrl_t;


/*!@brief
 *
 * SSI device modes
 */

typedef enum
{
    SSI_MODE_MASTER,
    SSI_MODE_SLAVE

}ssi_devicemode_t;

/*!@brief
 *
 * SSI Loop-back mode control
 */

typedef enum
{
    SSI_LOOPBACK_DISABLE,
    SSI_LOOPBACK_ENABLE


}ssi_loopbackmode_t;

/*!@brief
 *
 * SSI Date Frame Format
 */

typedef enum
{
    SSI_DSS_RESERVED = 2,
    SSI_DSS_4BIT,
    SSI_DSS_5BIT,
    SSI_DSS_6BIT,
    SSI_DSS_7BIT,
    SSI_DSS_8BIT,
    SSI_DSS_9BIT,
    SSI_DSS_10BIT,
    SSI_DSS_11BIT,
    SSI_DSS_12BIT,
    SSI_DSS_13BIT,
    SSI_DSS_14BIT,
    SSI_DSS_15BIT,
    SSI_DSS_16BIT

}ssi_dss_t;


/*!@brief
 *
 * SSI Clock Polarity (SPO)
 */

typedef enum
{
    SSI_SPO_IDLE_LOW,
    SSI_SPO_IDLE_HIGH

}ssi_spo_t;

/*!@brief
 *
 * SSI Clock Phase (SPH)
 */

typedef enum
{
    SSI_SPH_LEADINGEDGE,
    SSI_SPH_TRAILINGEDGE

}ssi_sph_t;

/*!@brief
 *
 * SSI Frame Format (FRF)
 */

typedef enum
{
    SSI_FRF_FREESCALE,
    SSI_FRF_TISSI,
    SSI_FRF_MICROWAVE

}ssi_frameformat_t;


/* @brief
 * Configuration Structure for SPIx
 * */


typedef struct
{
    ssi_devicemode_t MS;
    ssi_loopbackmode_t LBM;
    uint8_t SCR;
    ssi_frameformat_t FRF;
    uint8_t CPSDVSR; /*!< NOTE : Even numbers (2 to 254 only) >*/
    ssi_dss_t DSS;
    ssi_spo_t SPO;
    ssi_sph_t SPH;

}ssi_perifconfig_t;


/*! @brief
 *  GPIO Configuration Handle
 */

typedef struct
{
    ssi_periph_t *SSIX;                /*!<Holds the Base Address of the SSIx Peripheral>*/
    ssi_perifconfig_t SSICONFIG;       /*!<Handle to the SSI peripheral configuration settings>*/

}ssi_handle_t;

/*! @brief
 *
 *  API's supported by this driver
 */


/*!< Peripheral clock setup>*/

void ssi_pprl_clock_control (ssi_periph_t * pssi , ssi_perifclk_t setstate);    /*!<Function to control peripheral clock>*/

void ssi_port_control (ssi_periph_t *pssi , ssi_portctrl_t setstate);

///*!< SPI initialization Functions>*/

void ssi_init(ssi_handle_t *pssihandle);                                                        /*!<Function to initialize SPI peripheral>*/
//void SSI_DeInit(SPI_handle_t* pSPIhandle);                                                          /*!<Function to disable SPI peripheral>*/
//
//
//
///*!< SPI send/receive Functions>*/
//
//void SSI_SendData(SPI_TypeDef *pSPIx ,uint8_t *pData , uint32_t Size);                         /*!<Blocking function that sends output data >*/


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
