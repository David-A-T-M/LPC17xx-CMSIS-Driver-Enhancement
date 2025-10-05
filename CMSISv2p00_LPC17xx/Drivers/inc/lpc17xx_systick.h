/**
 * @file        lpc17xx_systick.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for SYSTICK firmware library on LPC17xx
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
 *
 * @par Refactor:
 * Date: 09/07/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup SYSTICK SYSTICK
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_SYSTICK_H_
#define LPC17XX_SYSTICK_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup SYSTICK_Private_Macros SYSTICK Private Macros
 * @{
 */

/** Enable SysTick Counter bit. */
#define ST_CTRL_ENABLE    ((uint32_t)(1 << 0))
/** Enable SysTick Interrupt bit. */
#define ST_CTRL_TICKINT   ((uint32_t)(1 << 1))
/** Clock Source selection bit. */
#define ST_CTRL_CLKSOURCE ((uint32_t)(1 << 2))
/** Count Flag bit. */
#define ST_CTRL_COUNTFLAG ((uint32_t)(1 << 16))
/** Maximum reload value for SysTick timer (24 bits). */
#define ST_MAX_LOAD       ((uint32_t)(0x00FFFFFF))
/** PCB bit position in PINSEL7 register. */
#define ST_PIN_PCB_POS    ((uint32_t)(20))

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup SYSTICK_Public_Functions SYSTICK Public Functions
 * @{
 */

/**
 * @brief       Initializes the System Tick timer using the internal CPU clock source.
 *
 * @param[in]   time    Time interval in milliseconds.
 *
 * @note        If the requested time exceeds the maximum possible interval for the SysTick timer,
 *              the LOAD register is set to its maximum value ST_MAX_LOAD, resulting in the
 *              longest possible timer interval.
 */
void SYSTICK_InternalInit(uint32_t time);

/**
 * @brief       Initializes the System Tick timer using an external clock source.
 *
 * This function configures the pin for external SysTick clock input, and initializes the
 * SysTick timer to use the external clock source. If the requested time exceeds the maximum
 * possible interval for the SysTick timer with the given external frequency, the LOAD register
 * is set to its maximum value ST_MAX_LOAD, resulting in the longest possible timer interval.
 *
 * @param[in]   extFreq External clock frequency in Hz.
 * @param[in]   time    Time interval in milliseconds.
 *
 * @note        If the requested time exceeds the maximum possible interval for the SysTick timer
 *              with the given external frequency, the LOAD register is set to its maximum value
 *              ST_MAX_LOAD, resulting in the longest possible timer interval.
 */
void SYSTICK_ExternalInit(uint32_t extFreq, uint32_t time);

/**
 * @brief       Enable or disable the System Tick counter.
 *
 * @param[in]   newState    System Tick counter status, should be:
 *                          - ENABLE
 *                          - DISABLE
 */
void SYSTICK_Cmd(FunctionalState newState);

/**
 * @brief       Enable or disable the System Tick interrupt.
 *
 * @param[in]   newState    System Tick interrupt status, should be:
 *                          - ENABLE
 *                          - DISABLE
 */
void SYSTICK_IntCmd(FunctionalState newState);

/**
 * @brief       Get the current value of the System Tick counter.
 *
 * @return      Current value of the System Tick counter.
*/
static __INLINE uint32_t SYSTICK_GetCurrentValue(void) {
    return (SysTick->VAL);
}

/**
 * @brief       Clear the System Tick counter flag.
*/
static __INLINE void SYSTICK_ClearCounterFlag(void) {
    (void)SysTick->CTRL;
}

/**
 * @brief       Get the current reload value of the System Tick timer.
 *
 * @return      Current reload value.
 */
static __INLINE uint32_t SYSTICK_GetReloadValue(void) {
    return (SysTick->LOAD & ST_MAX_LOAD);
}

/**
 * @brief       Set a new reload value for the System Tick timer.
 *
 * @param[in]   reloadTicks Reload value to set, in SysTick timer ticks.
 *
 * @note        If reloadValue exceeds 24 bits, only the least significant 24 bits are used.
*/
static __INLINE void SYSTICK_SetReloadValue(uint32_t reloadTicks) {
    SysTick->LOAD = (reloadTicks & ST_MAX_LOAD);
}

/**
 * @brief       Check if the System Tick timer is currently enabled.
 *
 * @return      SET if enabled, RESET otherwise.
 */
static __INLINE FlagStatus SYSTICK_IsActive(void) {
    return (SysTick->CTRL & ST_CTRL_ENABLE) ? SET : RESET;
}

/**
 * @brief       Check if the System Tick COUNTFLAG is set (timer has fired).
 *
 * @return      1 if COUNTFLAG is set, 0 otherwise.
*/
static __INLINE FlagStatus SYSTICK_HasFired(void) {
    return (SysTick->CTRL & ST_CTRL_COUNTFLAG) ? SET : RESET;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  // LPC17XX_SYSTICK_H_

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
