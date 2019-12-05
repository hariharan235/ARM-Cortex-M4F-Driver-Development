/**
 ******************************************************************************
 * @file    spi_tm4c123gh6pm.c, file name will change
 * @author  Hariharan Gopalakrishnan
 * @brief   TM4C123GH6PM Device Peripheral Access Layer Header File.
 *
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

#include <assert.h>
#include "TM4C123-Drivers/inc/spi_tm4c123gh6pm.h"


/*!< Peripheral clock setup*/

/*
 * @fn               - ssi_pprl_clock_control
 * @brief            - Function to enable/disable peripheral clock in RCGCSSI Register for the SSI peripheral
 * @param[in]        - Base Address of the SSI peripheral
 * @param[in]        - SSI_CLK_ENABLE / SSI_CLK_DISABLE
 * @return           - None
 * @note             - None
 */

void ssi_pprl_clock_control (ssi_periph_t * pssi , ssi_perifclk_t setstate)
{

    assert(pssi);

    if(setstate == SSI_CLK_ENABLE)
    {

        if(pssi == SSI0)
        {
            SYSCTL->RCGCSSI |= SSI_RCGC_MODULE0;
        }
        else if (pssi == SSI1)
        {
            SYSCTL->RCGCSSI |= SSI_RCGC_MODULE1;
        }
        else if (pssi == SSI2)
        {
            SYSCTL->RCGCSSI |= SSI_RCGC_MODULE2;
        }
        else if (pssi == SSI3)
        {
            SYSCTL->RCGCSSI |= SSI_RCGC_MODULE3;
        }
        else
        {
            assert(0);
        }
    }
    else
    {
        if(pssi == SSI0)
        {
            SYSCTL->RCGCSSI &= ~SSI_RCGC_MODULE0;
        }
        else if (pssi == SSI1)
        {
            SYSCTL->RCGCSSI &= ~SSI_RCGC_MODULE1;
        }
        else if (pssi == SSI2)
        {
            SYSCTL->RCGCSSI &= ~SSI_RCGC_MODULE2;
        }
        else if (pssi == SSI3)
        {
            SYSCTL->RCGCSSI &= ~SSI_RCGC_MODULE3;
        }
        else
        {
            assert(0);
        }
    }
}

/*
 *
 * @fn               - ssi_port_control
 * @brief            - Function Enable / Disable the SSI port
 * @param[in]        - Base Address of the SSI peripheral
 * @param[in]        - SPI_PORT_ENABLE / SPI_PORT_DISABLE
 * @return           - None
 * @note             - None
 */

void ssi_port_control (ssi_periph_t *pssi , ssi_portctrl_t setstate)
{

    assert(pssi);

    if(setstate == SSI_PORT_ENABLE)
    {
        pssi->CR1 |=  SPI_CR1_SSE;
    }
    else
    {
        while(ssi_isflag(pssi,SSI_SR_BSY_FLAG));
        pssi->CR1 &= ~SPI_CR1_SSE;
    }

}


/*
 * @fn               - ssi_isflag
 * @brief            - Function that return the status of flag in SSI_SR register
 * @param[in]        - Pointer to the SSI configuration structure
 * @param[in]        - SSI_SR register flags
 * @return           - True or False
 * @note             - None
 */

uint8_t ssi_isflag (ssi_periph_t* pssi , ssi_statusflags_t flag)
{

    uint8_t result;

    assert(pssi);

    switch(flag)
    {
    case SSI_SR_TFE_FLAG:
        result = SPI_SR_TFE_FLAG_POS;
        break;
    case SSI_SR_TNF_FLAG:
        result = SPI_SR_TNF_FLAG_POS;
        break;
    case SSI_SR_RNE_FLAG:
        result = SPI_SR_RNE_FLAG_POS;
        break;
    case SSI_SR_RFF_FLAG:
        result = SPI_SR_RFF_FLAG_POS;
        break;
    default:
        result = SPI_SR_BSY_FLAG_POS;
        break;
    }

    return (pssi->SR & result);
}







/*!< SSI initialization Function*/

/*
 * @fn               - ssi_init
 * @brief            - Function to initialize SSI peripheral
 * @param[in]        - Pointer to the SSI configuration structure
 * @return           - None
 * @note             - Corresponding peripheral clock must be enabled using ssi_pprl_clock_control()
 */


void ssi_init(ssi_handle_t *pssihandle)
{
    assert(pssihandle);

    assert(pssihandle->ssix);

    ssi_pprl_clock_control(pssihandle->ssix,SSI_CLK_ENABLE);

    ssi_port_control(pssihandle->ssix,SSI_PORT_DISABLE);

    if(pssihandle->ssiconfig.ms == SSI_MODE_MASTER)
    {
        pssihandle->ssix->CR1 &= ~SPI_CR1_MS;
    }
    else
    {
        pssihandle->ssix->CR1 |= SPI_CR1_MS;
    }

    if(pssihandle->ssiconfig.lbm == SSI_LOOPBACK_ENABLE)
    {
        pssihandle->ssix->CR1 |= SPI_CR1_LBM;
    }
    else
    {
        pssihandle->ssix->CR1 &= ~SPI_CR1_LBM;
    }

    pssihandle->ssix->CC |= pssihandle->ssiconfig.csrc;

    pssihandle->ssix->CPSR |= pssihandle->ssiconfig.cpsdvsr;

    pssihandle->ssix->CR0 |= (pssihandle->ssiconfig.scr << SPI_CR0_SCR_POS);

    pssihandle->ssix->CR0 |= (pssihandle->ssiconfig.frf << SPI_CR0_FRF_POS);

    switch (pssihandle->ssiconfig.frf)
    {
    case SSI_FRF_TISSI:
        /*!< Not Supported in current version >*/
        break;
    case SSI_FRF_MICROWAVE:
        /*!< Not Supported in current version >*/
        break;
    default :
        if(pssihandle->ssiconfig.spo == SSI_SPO_IDLE_HIGH)
        {
            pssihandle->ssix->CR0 |=  (1U << SPI_CR0_SPO_POS);
        }
        else
        {
            pssihandle->ssix->CR0 &=  ~(1U << SPI_CR0_SPO_POS);
        }

        if(pssihandle->ssiconfig.sph == SSI_SPH_TRAILINGEDGE)
        {
            pssihandle->ssix->CR0 |=  (1U << SPI_CR0_SPH_POS);
        }
        else
        {
            pssihandle->ssix->CR0 &=  ~(1U << SPI_CR0_SPH_POS);
        }
        break;
    }

    pssihandle->ssix->CR0 |= pssihandle->ssiconfig.dss;

    ssi_port_control(pssihandle->ssix,SSI_PORT_ENABLE);



}




/*
 *
 * @fn               - ssi_send_data
 * @brief            - Function to Send Data over SSI lines
 * @param[in]        - Base Address of the SSI peripheral
 * @param[in]        - Pointer to the Data to be sent
 * @param[in]        - Size of Transfer
 * @return           - None
 * @note             - Blocking Call
 */

void ssi_send_data(ssi_periph_t *pssi ,uint8_t *pdata , uint32_t size)
{
    assert(pssi);

    assert(pdata);

    while(size > 0)
    {
        while(ssi_isflag(pssi,SSI_SR_BSY_FLAG));

        if((pssi->CR0 & 0xF) > SSI_DSS_8BIT)
        {
            pssi->DR = *(uint16_t *)pdata;
            size -= 2;
            (uint16_t *)pdata++;
            (uint16_t *)pdata++;
        }
        else
        {
            pssi->DR = (*pdata);
            size--;
            pdata++;
        }
    }

}

/*
 *
 * @fn               - ssi_receive_data
 * @brief            - Function to receive Data over SSI lines
 * @param[in]        - Base Address of the SSI peripheral
 * @param[in]        - Pointer to the RxBuffer
 * @param[in]        - Size of Reception
 * @return           - None
 * @note             - Blocking Call
 */

void ssi_receive_data(ssi_periph_t *pssi,uint8_t *pdata,uint32_t size)
{

    assert(pssi);

    assert(pdata);

    while(size > 0)
    {
       while(!(ssi_isflag(pssi,SSI_SR_RNE_FLAG)));

        if((pssi->CR0 & 0xF) > SSI_DSS_8BIT)
        {
            *(uint16_t *)pdata = pssi->DR;
            size -= 2;
            (uint16_t *)pdata++;
            (uint16_t *)pdata++;
        }
        else
        {
            *pdata = pssi->DR;
            size--;
            pdata++;
        }
    }
}
