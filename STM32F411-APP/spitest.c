/*
 * spitest.c
 *
 *  Created on: Sep 9, 2019
 *      Author: root
 */


#include "gpio_stm32f411.h"
#include "spi_stm32f411.h"
#include <string.h>

//PB15 - MOSI
//PB14 - MISO
//PB13 - SCK
//PB12 - NSS

//AF05

void SPI_GPIOInitHw(void)
{
	GPIO_handle_t gpioSPI;

	memset(&gpioSPI,0,sizeof(gpioSPI));

	gpioSPI.pGPIO =  GPIOB;
	gpioSPI.GPIO_PinConfig.PINALTFNMODE = GPIO_PIN_AFR5;
	gpioSPI.GPIO_PinConfig.PINMODE = GPIO_MODE_ALTFN;
	gpioSPI.GPIO_PinConfig.PINOPTYPE = GPIO_OPTYPE_PuPl;
	gpioSPI.GPIO_PinConfig.PINPULLPDCONTROL = GPIO_PuPd_None;
	gpioSPI.GPIO_PinConfig.PINSPEED = GPIO_OSPEED_Fast;

	// SPI2 SCLK
	gpioSPI.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM13;
	GPIO_Init(&gpioSPI);

	// SPI2 MOSI
	gpioSPI.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM15;
	GPIO_Init(&gpioSPI);

	// SPI2 MISO
	gpioSPI.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM14;
	GPIO_Init(&gpioSPI);

	// SPI2 NSS
	gpioSPI.GPIO_PinConfig.PINNUMBER = GPIO_PIN_NUM12;
	GPIO_Init(&gpioSPI);

}

void SPI2InitHw(void)
{
	SPI_handle_t SSI2;

	memset(&SSI2,0,sizeof(SSI2));

	SSI2.pSPIx = SPI2;
	SSI2.SPIConfig.BUSCONFIG = SPI_BUS_CONF_FD;
	SSI2.SPIConfig.DEVICEMODE = SPI_MODE_MASTER;
	SSI2.SPIConfig.SCLKSPEED = SPI_PCLK_PDIV_2;
	SSI2.SPIConfig.DFF = SPI_DFF_8BIT;
	SSI2.SPIConfig.CPHA = SPI_CPHA_LEADINGEDGE;
	SSI2.SPIConfig.CPOL = SPI_CPOL_IDLE_LOW;
	SSI2.SPIConfig.SSM = SPI_SSM_SW;

	SPI_Init(&SSI2);

}

int main()
{
	char user_data[] = "Hello World";
	SPI_GPIOInitHw();
	SPI2InitHw();
	SPI_PeripheralConrol(SPI2,SPI_ENABLE);
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
	for(;;);
	return 0;
}
