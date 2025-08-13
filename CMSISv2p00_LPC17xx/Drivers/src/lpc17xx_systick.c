/**
 * @file        lpc17xx_systick.c
 * @brief       Contains all functions support for SYSTICK firmware library on LPC17xx
 * @version     2.0
 * @date        21. May. 2010
 * @author      NXP MCU SW Application Team
 *
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 */

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup SYSTICK
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_systick.h"
#include "lpc17xx_clkpwr.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _SYSTICK

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup SYSTICK_Public_Functions
 * @{
 */

void SYSTICK_InternalInit(uint32_t time) {
    SYSTICK_ExternalInit(SystemCoreClock, time);
    SysTick->CTRL |= ST_CTRL_CLKSOURCE;
}

void SYSTICK_ExternalInit(uint32_t extFreq, uint32_t time) {
    const float maxtime = (float)(ST_MAX_LOAD) * 1000 / (float)extFreq;

    SysTick->CTRL &= ~ ST_CTRL_CLKSOURCE;

    if ((float)time > maxtime)
        SysTick->LOAD = ST_MAX_LOAD;
    else
        SysTick->LOAD = (extFreq/1000)*time - 1;
}

void SYSTICK_Cmd(FunctionalState NewState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

    if(NewState == ENABLE)
        SysTick->CTRL |= ST_CTRL_ENABLE;
    else
        SysTick->CTRL &= ~ST_CTRL_ENABLE;
}

void SYSTICK_IntCmd(FunctionalState NewState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

    if(NewState == ENABLE)
        SysTick->CTRL |= ST_CTRL_TICKINT;
    else
        SysTick->CTRL &= ~ST_CTRL_TICKINT;
}

/**
 * @}
 */

#endif /* _SYSTICK */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */

