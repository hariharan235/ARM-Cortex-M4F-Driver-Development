/**
 ******************************************************************************
 * @file    spi_stm32f411.c, file name will change
 * @author  Hariharan Gopalakrishnan,
 * @brief   STM32F411VE Device Peripheral Access Layer Source File.
 *
 *  This file contains:
 *              - Helper functions for Driver exposed API functions
 *              - Driver exposed APIs, for SPI
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
#include "spi_stm32f411.h"


/*!< Peripheral clock setup*/

/*
 * @fn               - SPI_PprlClkCtrl
 * @brief            - Function to enable/disable peripheral clock for the SPI peripheral
 * @param[in]        - Base Address of the SPI peripheral
 * @param[in]        - SPI_CLK_ENABLE / SPI_CLK_DISABLE
 * @return           - None
 * @note             - None
 */

void SPI_PprlClkCtrl(SPI_TypeDef* pSPI , SPI_PerifClk_t setState)
{
	assert(pSPI);

	if(setState == SPI_CLK_ENABLE)
	{
		if(pSPI == SPI1)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI1EN_Pos);
			RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI1EN_Pos);
		}
		else if(pSPI == SPI2)
		{
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI2EN_Pos);
			RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI2EN_Pos);
		}
		else if(pSPI == SPI3)
		{
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI3EN_Pos);
			RCC->APB1ENR |= (1 << RCC_APB1ENR_SPI3EN_Pos);
		}
		else if(pSPI == SPI4)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI4EN_Pos);
			RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI4EN_Pos);
		}
		else if(pSPI == SPI5)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI5EN_Pos);
			RCC->APB2ENR |= (1 << RCC_APB2ENR_SPI5EN_Pos);
		}
		else
			assert(0);
	}
	else
	{
		if(pSPI == SPI1)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI1EN_Pos);
		}
		else if(pSPI == SPI2)
		{
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI2EN_Pos);
		}
		else if(pSPI == SPI3)
		{
			RCC->APB1ENR &= ~(1 << RCC_APB1ENR_SPI3EN_Pos);
		}
		else if(pSPI == SPI4)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI4EN_Pos);
		}
		else if(pSPI == SPI5)
		{
			RCC->APB2ENR &= ~(1 << RCC_APB2ENR_SPI5EN_Pos);
		}
		else
			assert(0);
	}
}

/*!< GPIO initialization Functions*/

/*
 * @fn               - SPI_Init
 * @brief            - Function to initialize SPI peripheral
 * @param[in]        - Pointer to the SPI configuration structure
 * @return           - None
 * @note             - Corresponding peripheral clock must be enabled using SPI_PprlClkCtrl()
 */

void SPI_Init(SPI_handle_t *pSPIHandle)
{
	assert(pSPIHandle);

	assert(pSPIHandle->pSPIx);

	/*!< Enable Peripheral clock >*/

	SPI_PprlClkCtrl(pSPIHandle->pSPIx ,SPI_CLK_ENABLE);

	/*!< Configure Device mode >*/

	if(pSPIHandle->SPIConfig.DEVICEMODE == SPI_MODE_MASTER)
	{
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_MSTR_Pos);
	}
	else
	{
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_MSTR_Pos);
	}

	/*!< Configure SPI bus >*/

	switch(pSPIHandle->SPIConfig.BUSCONFIG)
	{
	case SPI_BUS_CONF_FD:

		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE_Pos); /*!< 2-Wire Unidirectional Mode >*/
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_RXONLY_Pos);   /*!< Full-Duplex >*/
		break;

	case SPI_BUS_CONF_HD_RX:

		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE_Pos);  /*!< 1-Wire Bidirectional Mode >*/
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIOE_Pos);   /*< Receive Only >*/
		break;

	case SPI_BUS_CONF_HD_TX:

		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE_Pos);  /*!< 1-Wire Bidirectional Mode >*/
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIOE_Pos);    /*< Transmit Only >*/
		break;

	case SPI_BUS_CONF_SM_RX:

		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE_Pos); /*!< Unidirectional Mode >*/
		pSPIHandle->pSPIx->CR1 |=  (1 << SPI_CR1_RXONLY_Pos);   /*!< Receive Only >*/
		break;

	default:

		assert(0);
		break;
	}

	/*!< Configure SPI Baud Rate >*/

	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SCLKSPEED << SPI_CR1_BR_Pos);

	/*!< Configure SPI Data Frame Format >*/

	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.DFF << SPI_CR1_DFF_Pos);

	/*!< Configure SPI CPOL >*/

	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.CPOL << SPI_CR1_CPOL_Pos);

	/*!< Configure SPI CPHA >*/

	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.CPHA << SPI_CR1_CPHA_Pos);

	/*!< Configure SPI SSM >*/

	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SSM << SPI_CR1_SSM_Pos);

	if(pSPIHandle->SPIConfig.SSM == SPI_SSM_SW)
	{
		pSPIHandle->pSPIx->CR1 |=  (SPI_CR1_SSI);  /*!< Enable SSI in CR1 >*/

	}

}

/*
 * @fn               - SPI_isFlag
 * @brief            - Function that return the status of flag in SPI_SR register
 * @param[in]        - Pointer to the SPI configuration structure
 * @param[in]        - SPI_SR register flags
 * @return           - True or False
 * @note             - None
 */

uint8_t SPI_IsFlag(SPI_TypeDef* pSPIx , uint8_t flag)
{
	return (pSPIx->SR & flag);
}

/*
 * todo
 * @fn               - SPI_DeInit
 * @brief            - Function to disable SPI peripheral
 * @param[in]        - Pointer to the SPI configuration structure
 * @return           - None
 * @note             - None
 */

void SPI_DeInit(SPI_handle_t* pSPIhandle)
{
	assert(pSPIhandle);

	assert(pSPIhandle->pSPIx);

	//Page no 20.3.8

	switch(pSPIhandle->SPIConfig.BUSCONFIG)
	{

	case SPI_BUS_CONF_FD:

		while(!SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_RXNE)); /* Wait till last data is received */

		while(!SPI_IsFlag(pSPIhandle->pSPIx , SPI_SR_TXE));  /* Wait till Tx Buffer is empty */

		while(SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_BSY));     /* Wait till Buffers are free*/

		pSPIhandle->pSPIx->CR1 &= ~(1U << SPI_CR1_SPE_Pos); /* Disable SPI */

		SPI_PprlClkCtrl(pSPIhandle->pSPIx ,SPI_CLK_DISABLE); /* Disable Peripheral clock */

	    break;

	case SPI_BUS_CONF_HD_TX:

		while(!SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_TXE));  /* Wait till Tx Buffer is empty */

		while(SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_BSY));     /* Wait till Buffers are free*/

		pSPIhandle->pSPIx->CR1 &= ~(1U << SPI_CR1_SPE_Pos); /* Disable SPI */

		SPI_PprlClkCtrl(pSPIhandle->pSPIx ,SPI_CLK_DISABLE); /* Disable Peripheral clock */

	    break;

	default :

		while(!SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_RXNE)); /* Wait till second last data is received */

		__asm(" NOP");

		pSPIhandle->pSPIx->CR1 &= ~(1U << SPI_CR1_SPE_Pos); /* Disable SPI */

		while(!SPI_IsFlag(pSPIhandle->pSPIx, SPI_SR_RXNE)); /* Wait till second last data is received */

		SPI_PprlClkCtrl(pSPIhandle->pSPIx ,SPI_CLK_DISABLE); /* Disable Peripheral clock */

		break;

	}

}


/*
 *
 * @fn               - SPI_PeripheralConrol
 * @brief            - Function Enable / Disable the SPI Communication
 * @param[in]        - Base Address of the SPI peripheral
 * @param[in]        - SPI_Enable / SPI_Disable
 * @return           - None
 * @note             - None
 */

void SPI_PeripheralConrol(SPI_TypeDef *pSPIx , SPI_PerifCtrl_t setState)
{
	assert(pSPIx);

	if(setState == SPI_ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE_Pos);
	}
	else
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_Pos);
}

/*
 *
 * @fn               - SPI_SendData
 * @brief            - Function to Send Data over SPI lines
 * @param[in]        - Base Address of the SPI peripheral
 * @param[in]        - Pointer to the Data to be sent
 * @param[in]        - Size of Transfer
 * @return           - None
 * @note             - Blocking Call
 */

void SPI_SendData(SPI_TypeDef *pSPIx ,uint8_t *pData , uint32_t Size)
{

	assert(pSPIx);

	assert(pData);

	while(Size > 0)
	{
		while(!(SPI_IsFlag(pSPIx , SPI_SR_TXE)));

		if(pSPIx->CR1 & (SPI_CR1_DFF))
		{
			pSPIx->DR = *(uint16_t*)pData;
			Size-=2;
			(uint16_t*)pData++;
		}
		else
		{
			pSPIx->DR = *pData;
			Size--;
			pData++;
		}

	}
}


