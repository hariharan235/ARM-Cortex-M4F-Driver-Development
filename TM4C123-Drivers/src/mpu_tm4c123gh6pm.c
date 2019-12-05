/*
 * mpu_tm4c123gh6pm.c
 *
 *  Created on: Nov 9, 2019
 *      Author: root
 */

#include "TM4C123-Drivers/inc/mpu_tm4c123gh6pm.h"
#include <assert.h>

#define MPU_CTRL_ENABLE_Pos 0
#define MPU_CTRL_HFNMIENA_Pos 1
#define MPU_CTRL_PRIVDEFEN_Pos 2
#define MPU_ATTR_ENABLE_Pos 0


#define SCB_HND_CTRL_FAULT_EN_Pos 16


/*
 * @brief   Get available MPU Regions on MCU.
 * @param   None.
 * @retval  8-bit unsigned number.
 */

uint8_t _mpu_get_regions(void)
{
    uint8_t regions;
    regions = (MPU->TYPE & 0xFF) >> 8 ;
    return regions;
}


/*
 * @brief   MPU configuration
 * @param   uint32_t user_configuration
 * @brief   MPU_CONFIG_PRIV_DEFAULT enables the default memory map when in
 *          privileged mode and when no other regions are defined.  If this option
 *          is not enabled, then there must be at least one valid region already
 *          defined when the MPU is enabled.
 * @brief   MPU_CONFIG_HARDFLT_NMI enables the MPU while in a hard fault or NMI
 *          exception handler.  If this option is not enabled, then the MPU is
 *          disabled while in one of these exception handlers and the default
 *          memory map is applied.
 * @brief   MPU_CONFIG_NONE chooses none of the above options.  In this case,
 *          no default memory map is provided in privileged mode, and the MPU is not
 *          enabled in the fault handlers.
 * @retval  Init pass/fail.
 */

uint8_t _mpu_init(mpu_default_flags_t mpu_configuration)
{
    __asm(" ISB");
    __asm(" DSB");

    uint8_t status = 0;

    if(mpu_configuration != MPU_CONFIG_NONE)
    {
        if(mpu_configuration == MPU_CONFIG_PRIV_DEFAULT)
        {
            MPU->CTRL |= (1 << MPU_CTRL_PRIVDEFEN_Pos);
            MPU->CTRL |= (1 << MPU_CTRL_ENABLE_Pos);
            status = 1;
        }
        else
        {
            if(MPU->CTRL & 1)
            {
                MPU->CTRL |= (1 << MPU_CTRL_HFNMIENA_Pos);
                status = 1;
            }
        }
    }

    return status;

}

/*
 * @brief
 * @param
 * @retval
 *
 */

void _mpu_enable_fault_handler(mpu_fault_enable_t flags)
{
    SCB->SYSHNDCTRL |= (flags << SCB_HND_CTRL_FAULT_EN_Pos);
}

/*
 * @brief
 * @param
 * @retval
 *
 */

void _mpu_region_set(uint32_t mpu_rbar,uint32_t mpu_rattr)
{
    __asm(" ISB");
    __asm(" DSB");

    MPU->BASE = mpu_rbar;
    MPU->ATTR = mpu_rattr;

}

/*
 * @brief
 * @param
 * @retval
 *
 */

void _mpu_region_update(uint8_t region_num,uint32_t mpu_rbar,uint32_t mpu_rattr)
{
    __asm(" ISB");
    __asm(" DSB");

    /*!< Disable the region >*/

    MPU->NUMBER |= region_num;
    MPU->ATTR &= ~(1 << MPU_ATTR_ENABLE_Pos);

    /*!< Update setting >*/

    _mpu_region_set(mpu_rbar,mpu_rattr);

}

/*
 * @brief
 * @param
 * @retval
 *
 */

void _mpu_subregion_enable(uint8_t region_num,uint8_t subregion_num)
{
    __asm(" ISB");
    __asm(" DSB");

    if(subregion_num < 8)
    {
        /*!< Disable the region >*/

        MPU->NUMBER = region_num;
        MPU->ATTR &= ~(1 << MPU_ATTR_ENABLE_Pos);

        /*!< Enable sub-region>*/

        MPU->ATTR |= (1 << (8 + subregion_num));

        /*!< Enable the region >*/

        MPU->ATTR |= (1 << MPU_ATTR_ENABLE_Pos);

    }
}

/*
 * @brief
 * @param
 * @retval
 *
 */

void _mpu_subregion_disable(uint8_t region_num,uint8_t subregion_num)
{
    __asm(" ISB");
    __asm(" DSB");

    if(subregion_num < 8)
    {
        /*!< Disable the region >*/

        MPU->NUMBER = region_num;
        MPU->ATTR &= ~(1 << MPU_ATTR_ENABLE_Pos);

        /*!< Disable sub-region>*/

        MPU->ATTR &= ~(1 << (8 + subregion_num));

        /*!< Enable the region >*/

        MPU->ATTR |= (1 << MPU_ATTR_ENABLE_Pos);

    }
}

/*
 * @brief
 * @param
 * @retval
 *
 */


void _mpu_region_set_extend(const uint32_t mpu_table[][2],uint8_t region_nos)
{
    __asm(" ISB");
    __asm(" DSB");

    volatile uint32_t *mpu_array = &MPU->BASE;

    uint8_t mpu_entries = 0;
    uint8_t region = 0;

    /* Set multiple of 4 regions */

    while((region_nos > 0) && (region & 3 == 0))
    {
        while(mpu_entries < 8)
        {
            mpu_array[mpu_entries] = mpu_table[region][0];
            mpu_array[mpu_entries + 1] = mpu_table[region][1];
            mpu_entries += 2;
            region++;
            region_nos--;
        }
        mpu_entries = 0;
    }

    /* Set the rest */

    while(region_nos)
    {
        mpu_array[0] = mpu_table[region][0];
        mpu_array[1] = mpu_table[region][1];
        region++;
        region_nos--;
    }

}

