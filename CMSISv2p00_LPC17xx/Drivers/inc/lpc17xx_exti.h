/**
 * @file        lpc17xx_exti.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for External interrupt firmware library on LPC17xx
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
 * Date: 25/07/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup EXTI EXTI
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_EXTI_H_
#define LPC17XX_EXTI_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup EXTI_Private_Macros EXTI Private Macros
 * @{
 */

/* ------------------------ MACROS MASKS DEFINITIONS ------------------------ */
/** All external interrupt lines mask. */
#define EINT_ALL_MASK ((0xF))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup EXTI_Public_Types EXTI Public Types
 * @{
 */

/**
 * @brief EXTI external interrupt line option.
 */
typedef enum {
    EXTI_EINT0, /*!<  P2.10.*/
    EXTI_EINT1, /*!<  P2.11.*/
    EXTI_EINT2, /*!<  P2.12.*/
    EXTI_EINT3  /*!<  P2.13.*/
} EXTI_LINE;
/** Check EXTI line option parameter. */
#define PARAM_EXTI_LINE(LINE) ((LINE) >= EXTI_EINT0 && (LINE) <= EXTI_EINT3)

/**
 * @brief EXTI mode option.
 */
typedef enum {
    EXTI_LEVEL_SENSITIVE,
    EXTI_EDGE_SENSITIVE
} EXTI_MODE;
/** Check EXTI mode option parameter. */
#define PARAM_EXTI_MODE(MODE) ((MODE) == EXTI_LEVEL_SENSITIVE || (MODE) == EXTI_EDGE_SENSITIVE)

/**
 * @brief EXTI polarity option.
 */
typedef enum {
    EXTI_LOW_ACTIVE   = 0,
    EXTI_FALLING_EDGE = 0,
    EXTI_HIGH_ACTIVE  = 1,
    EXTI_RISING_EDGE  = 1
} EXTI_POLARITY;
/** Check EXTI polarity option parameter. */
#define PARAM_EXTI_POLARITY(POL) ((POL) == EXTI_LOW_ACTIVE || (POL) == EXTI_HIGH_ACTIVE)

/**
 * @brief EXTI resistor option.
 */
typedef enum {
    EXTI_PULLUP = 0,
    EXTI_PULLDOWN,
    EXTI_NOPULL
} EXTI_RESISTOR;
/** Check EXTI resistor option parameter. */
#define PARAM_EXTI_RESISTOR(RES) ((RES) >= EXTI_PULLUP && (RES) <= EXTI_NOPULL)

/**
 * @brief EXTI Initialize structure.
 */
typedef struct {
    EXTI_LINE line;         /*!< EXTI_EINTx [0...3]. */
    EXTI_MODE mode;         /*!< EXTI_LEVEL_SENSITIVE or EXTI_EDGE_SENSITIVE. */
    EXTI_POLARITY polarity; /*!< - EXTI_LOW_ACTIVE
                                 - EXTI_FALLING_EDGE
                                 - EXTI_HIGH_ACTIVE
                                 - EXTI_RISING_EDGE */
} EXTI_CFG_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup EXTI_Public_Functions EXTI Public Functions
 * @{
 */

/**
 * @brief       Initializes the External Interrupt (EXTI) controller.
 *
 * This function disables all external IRQs (EINT0 to EINT3) in the NVIC and sets the EXTMODE
 * and EXTPOLAR registers to their default values (level-sensitive mode, low polarity).
 *
 * @note        It is safe to call this function during system initialization or before configuring
 *              individual external interrupt lines. To clear pending flags, use EXTI_ClearEXTIFlag
 *              or EINT_EnableIRQ as appropriate.
 */
void EXTI_Init(void);

/**
 * @brief       Configures the pin function and resistor mode for a specific EXTI line.
 *
 * This function sets the pin function for the selected EXTI line (EINT0-EINT3) on P2.10-P2.13,
 * and configures the internal resistor mode as pull-up, pull-down, or no-pull.
 *
 * @param[in]   line     EXTI line to configure (EXTI_EINT0 to EXTI_EINT3).
 * @param[in]   resMode  Resistor mode:
 *                       - EXTI_PULLUP:   Enable pull-up
 *                       - EXTI_PULLDOWN: Enable pull-down
 *                       - EXTI_NOPULL:   No pull
 *
 * @note        This function does not enable the interrupt; it only configures the pin.
 */
void EXTI_PinConfig(EXTI_LINE line, EXTI_RESISTOR resMode);

/**
 * @brief       Configures a specific External Interrupt (EXTI) line.
 *
 * This function disables the corresponding external IRQ in the NVIC before making any changes,
 * sets the mode and polarity for the selected EXTI line.
 *
 * @param[in]   extiCfg  Pointer to an EXTI_CFG_Type structure containing the configuration
 *                       information for the specified external interrupt line.
 */
void EXTI_Config(const EXTI_CFG_Type* extiCfg);

/**
 * @brief       Configures and enables a specific External Interrupt (EXTI) line.
 *
 * This function disables the corresponding external IRQ in the NVIC before making any changes,
 * sets the mode and polarity for the selected EXTI line, clears the interrupt flag for that line,
 * and finally enables the IRQ in the NVIC.
 *
 * This sequence ensures safe configuration and activation of the external interrupt, preventing
 * spurious interrupts and guaranteeing that the interrupt flag is cleared before enabling.
 *
 * @param[in]   extiCfg  Pointer to an EXTI_CFG_Type structure containing the configuration
 *                       information for the specified external interrupt line.
 */
void EXTI_ConfigEnable(const EXTI_CFG_Type* extiCfg);

/**
 * @brief       Clears the external interrupt flag for the specified EXTI line.
 *
 * @param[in]   line  EXTI_EINTx [0...3].
 */
void EXTI_ClearFlag(EXTI_LINE line);

/**
 * @brief       Gets the status of the external interrupt flag for the specified EXTI line.
 *
 * @param[in]   line  EXTI_EINTx [0...3].
 *
 * @return      SET if the interrupt flag is set, RESET otherwise.
 */
FlagStatus EXTI_GetFlag(EXTI_LINE line);

/**
 * @brief       Clears the interrupt flag and enables the IRQ for the specified EXTI line.
 *
 * @param[in]   line  EXTI_EINTx [0...3].
 */
void EXTI_EnableIRQ(EXTI_LINE line);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  // LPC17XX_EXTI_H_

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
