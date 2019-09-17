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
        assert(0);
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

}

