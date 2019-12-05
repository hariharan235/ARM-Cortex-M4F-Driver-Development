/*
 * mpu_tm4c123gh6pm.h
 *
 *  Created on: Nov 9, 2019
 *      Author: Hariharan Gopalakrishnan
 */

#ifndef MPU_TM4C123GH6PM_H_
#define MPU_TM4C123GH6PM_H_


#include <stdint.h>
#include "mcu_tm4c123gh6pm.h"
/*!< Macros >*/



/*!< Type defines >*/


typedef enum
{
    NO_ACCESS,
    NO_UNPRIV_PRIV_RW,
    RO_UNPRIV_PRIV_RW,
    FULL_ACCESS,
    ACCESS_RESERV,
    NO_UNPRIV_RO_PRIV,
    RO_UNPRIV_RO_PRIV,

}mpu_access_t;

typedef enum
{
    MPU_REGION0,
    MPU_REGION1,
    MPU_REGION2,
    MPU_REGION3,
    MPU_REGION4,
    MPU_REGION5,
    MPU_REGION6,
    MPU_REGION7

}mpu_region_t;

typedef enum
{
    MPU_SIZE_1_KiB = 9,
    MPU_SIZE_2_KiB = 10,
    MPU_SIZE_8_KiB = 12,
    MPU_SIZE_256_KiB = 17,
    FULL_MAP_SIZE_FIELD = 31

}mpu_region_size;

typedef enum
{
    MPU_SUB_REGION0_DIS,
    MPU_SUB_REGION1_DIS,
    MPU_SUB_REGION2_DIS,
    MPU_SUB_REGION3_DIS,
    MPU_SUB_REGION4_DIS,
    MPU_SUB_REGION5_DIS,
    MPU_SUB_REGION6_DIS,
    MPU_SUB_REGION7_DIS

}mpu_sub_region_t;

typedef enum
{
    MPU_DISABLE,
    MPU_ENABLE

}mpu_control_t;

typedef enum
{
    MPU_CONFIG_NONE,
    MPU_CONFIG_PRIV_DEFAULT,
    MPU_CONFIG_HARDFLT_NMI,


}mpu_default_flags_t;


typedef enum
{
    MPU_ENABLE_RESERVED,
    MPU_ENABLE_MEM_FAULT,
    MPU_ENABLE_BUS_FAULT,
    MPU_ENABLE_USAGE_FAULT

}mpu_fault_enable_t;

typedef struct mpu_config
{
     uint8_t region_nos;
     uint32_t mpu_array_template[8];

}mpu_handle_t;


// API

uint8_t _mpu_get_regions(void);
uint8_t _mpu_init(mpu_default_flags_t mpu_configuration);
void _mpu_enable_fault_handler(mpu_fault_enable_t flags);
void _mpu_region_set(uint32_t mpu_rbar,uint32_t mpu_rattr);
void _mpu_subregion_enable(uint8_t region_num,uint8_t subregion_num);
void _mpu_subregion_disable(uint8_t region_num,uint8_t subregion_num);





#endif /* MPU_TM4C123GH6PM_H_ */
