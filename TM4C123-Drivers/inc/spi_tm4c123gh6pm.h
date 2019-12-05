/**
 ******************************************************************************
 * @file    spi_tm4c123gh6pm.h, file name will change
 * @author  Hariharan Gopalakrishnan
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
#include "mcu_tm4c123gh6pm.h"
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


 /*!@brief
  *
  * SSI CR1 Register Bit-field Defines
  */

#define SPI_CR1_LBM      (1U << 0)
#define SPI_CR1_SSE      (1U << 1)
#define SPI_CR1_MS       (1U << 2)
#define SPI_CR1_EOT      (1U << 4)

 /*!@brief
  *
  * SSI CR1 Register Bit-field Defines
  */

#define SPI_CR0_FRF_POS 4U
#define SPI_CR0_SPO_POS 6U
#define SPI_CR0_SPH_POS 7U
#define SPI_CR0_SCR_POS 8U

 /*!@brief
  *
  * SSI SR Register Bit-field Defines
  */

#define SPI_SR_TFE_FLAG_POS (1U << 0)
#define SPI_SR_TNF_FLAG_POS (1U << 1)
#define SPI_SR_RNE_FLAG_POS (1U << 2)
#define SPI_SR_RFF_FLAG_POS (1U << 3)
#define SPI_SR_BSY_FLAG_POS (1U << 4)

 /******************************************************************************/
 /*                                                                            */
 /*                  Enumerations for SSI Peripheral Configuration             */
 /*                                                                            */
 /******************************************************************************/


 /*!@brief
  *
  * SSI Status Flags
  */

 typedef enum
 {
     SSI_SR_TFE_FLAG,
     SSI_SR_TNF_FLAG,
     SSI_SR_RNE_FLAG,
     SSI_SR_RFF_FLAG,
     SSI_SR_BSY_FLAG

 }ssi_statusflags_t;

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
 * SSI Baud Clock Source
 */

typedef enum
{
    SSI_CSRC_SYSTEMCLOCK,
    SSI_CSRC_RESERVED = 4,
    SSI_CSRC_PIOSC

}ssi_csrc_t;


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
    ssi_devicemode_t ms;
    ssi_loopbackmode_t lbm;
    ssi_frameformat_t frf;
    uint8_t scr;
    uint8_t cpsdvsr; /*!< NOTE : Even numbers (2 to 254 only) >*/
    ssi_dss_t dss;
    ssi_spo_t spo;
    ssi_sph_t sph;
    ssi_csrc_t csrc;

}ssi_perifconfig_t;


/*! @brief
 *  GPIO Configuration Handle
 */

typedef struct
{
    ssi_periph_t *ssix;                /*!<Holds the Base Address of the SSIx Peripheral>*/
    ssi_perifconfig_t ssiconfig;       /*!<Handle to the SSI peripheral configuration settings>*/

}ssi_handle_t;

/*! @brief
 *
 *  API's supported by this driver
 */


/*!< Peripheral clock setup>*/

void ssi_pprl_clock_control (ssi_periph_t * pssi , ssi_perifclk_t setstate);    /*!<Function to control peripheral clock>*/

void ssi_port_control (ssi_periph_t *pssi , ssi_portctrl_t setstate);


uint8_t ssi_isflag(ssi_periph_t* pssi , uint8_t flag);

///*!< SPI initialization Functions>*/

void ssi_init(ssi_handle_t *pssihandle);                                                        /*!<Function to initialize SPI peripheral>*/
//void SSI_DeInit(SPI_handle_t* pSPIhandle);                                                          /*!<Function to disable SPI peripheral>*/
//
//
//
/*!< SPI send/receive Functions>*/

void ssi_send_data(ssi_periph_t *pssi ,uint8_t *pdata , uint32_t size);                         /*!<Blocking function that sends output data >*/
void ssi_receive_data(ssi_periph_t *pssi,uint8_t *pdata , uint32_t size);                                       /*!<Blocking function that receives input data >*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
