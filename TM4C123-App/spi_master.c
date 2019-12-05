/*
 * spi_test1.c
 *
 *  Created on: Sep 17, 2019
 *      Author: root
 */

#include "TM4C123-Drivers/inc/mcu_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/gpio_tm4c123gh6pm.h"
#include "TM4C123-Drivers/inc/spi_tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>

//static uint8_t dummpyLoad = 0;

/* Port B SSI2
 * PB4 - CLK , PB7 - TX , PB5 - FSS , PB6 -RX
 */

void init_gpio_ssi2_hw(void)
{
    gpio_handle_t spitest1;
    memset(&spitest1,0,sizeof(spitest1));

    /*!< PB4 - SSI2CLK >*/
    spitest1.p_gpio_x = GPIOB;
    spitest1.gpio_pin_config.alternate_function = GPIO_AFSEL_ENABLE;
    spitest1.gpio_pin_config.pctl_val = PCTL_AF2;
    spitest1.gpio_pin_config.direction = GPIO_DIR_OUTPUT;
    spitest1.gpio_pin_config.drive = GPIO_DR2R;
    spitest1.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
    spitest1.gpio_pin_config.pull_up_down = GPIO_NOPUPD;
    spitest1.gpio_pin_config.pin_number = 4;
    gpio_init(&spitest1);

    /*!< PB7 - SSI2TX >*/

    spitest1.gpio_pin_config.pin_number = 7;
    gpio_init(&spitest1);

    /*!< PB5 - SSI2FSS >*/

    spitest1.gpio_pin_config.pin_number = 5;
    gpio_init(&spitest1);

    //    /*!< PB6 - SSI2RX >*/
    //
    //    spitest1.gpio_pin_config.direction = GPIO_DIR_INPUT;
    //    spitest1.gpio_pin_config.pin_number  = 6;
    //    gpio_init(&spitest1);

}

/* Port A SSI0
 * PA2 - CLK , PA5 - TX , PA3 - FSS
 */


//void init_gpio_ssi0_hw(void)
//{
//    gpio_handle_t spitest1;
//    memset(&spitest1,0,sizeof(spitest1));
//
//    /*!< PA2 - SSI0CLK >*/
//    spitest1.p_gpio_x = GPIOA;
//    spitest1.gpio_pin_config.alternate_function = GPIO_AFSEL_ENABLE;
//    spitest1.gpio_pin_config.pctl_val = PCTL_AF2;
//    spitest1.gpio_pin_config.direction = GPIO_DIR_OUTPUT;
//    spitest1.gpio_pin_config.drive = GPIO_DR2R;
//    spitest1.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
//    spitest1.gpio_pin_config.pull_up_down = GPIO_NOPUPD;
//    spitest1.gpio_pin_config.pin_number = 2;
//    gpio_init(&spitest1);
//
//    /*!< PA5 - SSI0TX >*/
//
//    spitest1.gpio_pin_config.pin_number = 5;
//    gpio_init(&spitest1);
//
//    /*!< PA3 - SSI0FSS >*/
//
//    spitest1.gpio_pin_config.pin_number = 3;
//    gpio_init(&spitest1);        while(1)
//
//    //    /*!< PB6 - SSI2RX >*/
//    //
//    //    spitest1.gpioPinConfig.DIRECTION = GPIO_DIR_INPUT;
//    //    spitest1.gpioPinConfig.PINNUMBER = 6;
//    //    GPIO_Init(&spitest1);
//
//}

void init_pb_gpio(void)
{
    gpio_handle_t pushbtn;
    memset(&pushbtn,0,sizeof(pushbtn));
    pushbtn.p_gpio_x = GPIOF;
    pushbtn.gpio_pin_config.direction = GPIO_DIR_INPUT;
    pushbtn.gpio_pin_config.pin_mode = GPIO_DEN_ENABLE;
    pushbtn.gpio_pin_config.pull_up_down = GPIO_PUR_ENABLE;
    pushbtn.gpio_pin_config.pin_number = 4;
    gpio_init(&pushbtn);
}


/*!< SSI2 Master , 8-bit DSS , MODE 0 , 16/(16(1+0)) = 1 MHz >*/

void init_spi_hw(void)
{
    ssi_handle_t spitest1;
    memset(&spitest1,0,sizeof(spitest1));

    spitest1.ssix = SSI2;
    spitest1.ssiconfig.ms = SSI_MODE_MASTER;
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
    char testData[] = "HelloWorld";
    // char testData2[] = "ByeWorld";
    //uint16_t testData3[] = {192,168,299,4000};
    init_gpio_ssi2_hw();
    //init_gpio_ssi0_hw();
    init_pb_gpio();
    init_spi_hw();
    //init_spi_hw1();
    while(gpio_read_pin(GPIOF,4));
    ssi_send_data(SSI2,(uint8_t*)testData,sizeof(testData));
    // ssi_receive_data(SSI2,&dummpyLoad,1); /*!< Clear Tx BUFFER */

    //ssi_send_data(SSI0,(uint8_t *)testData2 , sizeof(testData2)-1);
    while(1);

}
