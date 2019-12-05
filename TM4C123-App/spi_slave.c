/*
 * spi_slave.c
 *
 *  Created on: Sep 19, 2019
 *      Author: root
 */



#include "TM4C123-Drivers/inc/mcu_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/gpio_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/spi_tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>

uint8_t loopData[15];

void init_gpio_ssi2_hwSlave(void)
{
    gpio_handle_t spitest1;
    memset(&spitest1,0,sizeof(spitest1));

    /*!< PB4 - SSI2CLK >*/
    spitest1.p_gpio_x = GPIOB;
    spitest1.gpio_pin_config.alternate_function = GPIO_AFSEL_ENABLE;
    spitest1.gpio_pin_config.pctl_val = PCTL_AF2;
    spitest1.gpio_pin_config.direction = GPIO_DIR_INPUT;
    spitest1.gpio_pin_config.drive = GPIO_DR2R;
    spitest1.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
    spitest1.gpio_pin_config.pull_up_down = GPIO_NOPUPD;
    spitest1.gpio_pin_config.pin_number = 4;
    gpio_init(&spitest1);

//    /*!< PB7 - SSI2TX >*/
//    spitest1.gpio_pin_config.direction = GPIO_DIR_OUTPUT;
//    spitest1.gpio_pin_config.pin_number = 7;
//    gpio_init(&spitest1);

    /*!< PB6 - SSI2RX >*/

    spitest1.gpio_pin_config.pin_number  = 6;
    gpio_init(&spitest1);


    /*!< PB5 - SSI2FSS >*/
    spitest1.gpio_pin_config.pull_up_down = GPIO_PDR_ENABLE;
    spitest1.gpio_pin_config.pin_number = 5;
    gpio_init(&spitest1);


}



/*!< SSI2 Master , 8-bit DSS , MODE 0 , 16/(16(1+0)) = 1 MHz , LBM >*/


void init_spi_hwslave(void)
{
    ssi_handle_t spitest1;
    memset(&spitest1,0,sizeof(spitest1));

    spitest1.ssix = SSI2;
    spitest1.ssiconfig.ms = SSI_MODE_SLAVE;
    spitest1.ssiconfig.csrc = SSI_CSRC_SYSTEMCLOCK;
    spitest1.ssiconfig.frf = SSI_FRF_FREESCALE;
    spitest1.ssiconfig.lbm = SSI_LOOPBACK_DISABLE;
    spitest1.ssiconfig.dss = SSI_DSS_8BIT;
    spitest1.ssiconfig.cpsdvsr = 16;
    spitest1.ssiconfig.scr = 0;
    spitest1.ssiconfig.sph = SSI_SPH_LEADINGEDGE;
    spitest1.ssiconfig.spo = SSI_SPO_IDLE_LOW;
    ssi_init(&spitest1);
}

int main()
{
    uint8_t dummyLoad = 0;
    init_gpio_ssi2_hwSlave();
    init_spi_hwslave();
    while(1)
    {
       //while(gpio_read_pin(GPIOB,5));
        ssi_receive_data(SSI2,(uint8_t*)loopData,10);
        //Clear buffer here
        ssi_send_data(SSI2,&dummyLoad,1);
       // memset(loopData,0,sizeof(loopData));
    }
}
