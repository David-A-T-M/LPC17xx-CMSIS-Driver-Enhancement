/**
 * @file        lpc17xx_pinsel.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for Pin connect block firmware library on LPC17xx
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
 * Last update: 20/02/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup PINSEL PINSEL
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_PINSEL_H_
#define LPC17XX_PINSEL_H_

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx.h"
#include "lpc17xx_common.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup PINSEL_Private_Macros PINSEL Private Macros
 * @{
 */

/* ------------------------ MACROS MASKS DEFINITIONS ------------------------ */
#define PINSEL_FUNC_MASK     ((0x3UL)) /**< Function selection mask. */
#define PINSEL_RES_MODE_MASK ((0x3UL)) /**< Resistor mode selection mask. */
#define PINSEL_PIN_MASK      ((0x1UL)) /** Pin selection mask. */
#define PINSEL_OD_MASK       ((0x1UL)) /**< Open-drain mode selection mask. */

/* ------------------------- MACROS BIT DEFINITIONS ------------------------- */
#define PINSEL_TRACE_POS ((0x3UL)) /**< Trace pin position. */

/* Pin selection define */
/* I2C Pin Configuration register bit description */
#define PINSEL_I2CPADCFG_SDADRV0 _BIT(0) /**< Drive mode control for the SDA0 pin, P0.27. */
#define PINSEL_I2CPADCFG_SDAI2C0 _BIT(1) /**< I2C mode control for the SDA0 pin, P0.27. */
#define PINSEL_I2CPADCFG_SCLDRV0 _BIT(2) /**< Drive mode control for the SCL0 pin, P0.28. */
#define PINSEL_I2CPADCFG_SCLI2C0 _BIT(3) /**< I2C mode control for the SCL0 pin, P0.28. */

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup PINSEL_Public_Types PINSEL Public Types
 * @{
 */

/**
 * @brief Pin function selection for PINSEL.
 */
typedef enum {
    PINSEL_FUNC_00 = 0,
    PINSEL_FUNC_01 = 1,
    PINSEL_FUNC_10 = 2,
    PINSEL_FUNC_11 = 3
} PINSEL_FUNC;
/** Check PINSEL function option parameter. */
#define PARAM_PINSEL_FUNC(FUNC) ((FUNC) >= PINSEL_FUNC_00 && (FUNC) <= PINSEL_FUNC_11)

/**
 * @brief Pin mode selection for PINSEL.
 */
typedef enum {
    PINSEL_PULLUP = 0,
    PINSEL_REPEATER,
    PINSEL_TRISTATE,
    PINSEL_PULLDOWN
} PINSEL_MODE;
/** Check PINSEL pin mode option parameter. */
#define PARAM_PINSEL_MODE(MODE) ((MODE) >= PINSEL_PULLUP && (MODE) <= PINSEL_PULLDOWN)

/**
 * @brief I2C drive mode selection for PINSEL.
 */
typedef enum {
    PINSEL_I2C_NORMAL = 0,
    PINSEL_I2C_FAST
} PINSEL_I2C_MODE;
/** Check PINSEL I2C mode option parameter. */
#define PARAM_PINSEL_I2C_MODE(MODE) ((MODE) == PINSEL_I2C_NORMAL || (MODE) == PINSEL_I2C_FAST)

/**
 * @brief Pin configuration structure.
 */
typedef struct {
    LPC_PORT port;             /**< PORT_x [0...4]. */
    LPC_PIN pin;               /**< PIN_x [0...31]. */
    PINSEL_FUNC func;          /**< PINSEL_FUNC_x [00...11]. */
    PINSEL_MODE mode;          /**< Should be:
                               - PINSEL_PULLUP : Internal pull-up resistor.
                               - PINSEL_REPEATER : Repeater mode.
                               - PINSEL_TRISTATE : Tri-state.
                               - PINSEL_PULLDOWN : Internal pull-down resistor. */
    FunctionalState openDrain; /**< Should be:
                               - ENABLE : Open-drain mode enabled.
                               - DISABLE : Open-drain mode disabled (normal mode). */
} PINSEL_CFG_T;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup PINSEL_Public_Functions PINSEL Public Functions
 * @{
 */

/**
 * @brief       Configures the pin according to the parameters in pinCfg.
 *
 * @param[in]   pinCfg  Pointer to a PINSEL_CFG_T structure that contains
 *                      the configuration information for the specified pin.
 */
void PINSEL_ConfigPin(const PINSEL_CFG_T* pinCfg);

/**
 * @brief       Configures multiple pins according to the parameters in
 *              pinCfg and the pins mask.
 *
 * @param[in]   pinCfg  Pointer to a PINSEL_CFG_T structure containing
 *                      the base configuration for the pins.
 * @param[in]   pinMask 32-bit value where each bit corresponds to a pin (0-31).
 *
 * @note        For each bit set in pins, the corresponding pin is configured
 *              using the parameters from pinCfg, except that the pinNum field in
 *              the original pinCfg is ignored and set automatically for each pin.
 */
void PINSEL_ConfigMultiplePins(const PINSEL_CFG_T* pinCfg, uint32_t pinMask);

/**
 * @brief       Configures the trace function.
 *
 * @param[in]   newState Must be:
 *                       - ENABLE : Enable Trace Function.
 *                       - DISABLE : Disable Trace Function.
 */
void PINSEL_ConfigTraceFunc(FunctionalState newState);

/**
 * @brief       Configures the I2C pins according to the specified parameters.
 *
 * @param[in]   driveMode Should be one of the following:
 *                        - PINSEL_I2C_NORMAL : Standard drive mode.
 *                        - PINSEL_I2C_FAST   : Fast Mode Plus drive mode.
 *
 * @param[in]   filterSlewRate Should be:
 *                             - ENABLE  : Enables filter and slew rate control.
 *                             - DISABLE : Disables filter and slew rate control.
 *
 * @note        If filterSlewRate is DISABLE, the driveMode parameter
 *              is ignored and both pins are configured as standard drive mode
 *              (PINSEL_I2C_NORMAL) with filter and slew rate control disabled.
 */
void PINSEL_SetI2CPins(PINSEL_I2C_MODE driveMode, FunctionalState filterSlewRate);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  // LPC17XX_PINSEL_H_

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
