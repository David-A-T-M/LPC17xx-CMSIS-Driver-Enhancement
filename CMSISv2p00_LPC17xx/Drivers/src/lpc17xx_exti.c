/**
 * @file        lpc17xx_exti.c
 * @brief       Contains all functions support for External interrupt firmware library on LPC17xx
 * @version     3.0
 * @date        18. June. 2010
 * @author      NXP MCU SW Application Team
 **************************************************************************
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
 **********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup EXTI
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_exti.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _EXTI

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup EXTI_Public_Functions
 * @{
 */

/*********************************************************************/ /**
 * @brief       Initializes the External Interrupt (EXTI) controller.
 *
 * This function disables all external IRQs (EINT0 to EINT3) in the NVIC and sets the EXTMODE
 * and EXTPOLAR registers to their default values (level-sensitive mode, low polarity).
 *
 * @note        It is safe to call this function during system initialization or before configuring
 *              individual external interrupt lines. To clear pending flags, use EXTI_ClearEXTIFlag
 *              or EINT_EnableIRQ as appropriate.
 *********************************************************************/
void EXTI_Init(void) {
    NVIC_DisableIRQ(EINT0_IRQn);
    NVIC_DisableIRQ(EINT1_IRQn);
    NVIC_DisableIRQ(EINT2_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);

    LPC_SC->EXTMODE = 0x0;
    LPC_SC->EXTPOLAR = 0x0;
}

/*********************************************************************/ /**
 * @brief       Configures a specific External Interrupt (EXTI) line.
 *
 * This function disables the corresponding external IRQ in the NVIC before making any changes,
 * sets the mode and polarity for the selected EXTI line.
 *
 * @param[in]   EXTICfg  Pointer to an EXTI_InitTypeDef structure containing the configuration
 *                       information for the specified external interrupt line.
 *********************************************************************/
void EXTI_Config(EXTI_InitTypeDef* EXTICfg) {
    NVIC_DisableIRQ((IRQn_Type)(EINT0_IRQn + EXTICfg->EXTI_Line));

    EXTI_SetMode(EXTICfg->EXTI_Line, EXTICfg->EXTI_Mode);
    EXTI_SetPolarity(EXTICfg->EXTI_Line, EXTICfg->EXTI_Polarity);
}

/*********************************************************************/ /**
 * @brief       Configures and enables a specific External Interrupt (EXTI) line.
 *
 * This function disables the corresponding external IRQ in the NVIC before making any changes,
 * sets the mode and polarity for the selected EXTI line, clears the interrupt flag for that line,
 * and finally enables the IRQ in the NVIC.
 *
 * This sequence ensures safe configuration and activation of the external interrupt, preventing
 * spurious interrupts and guaranteeing that the interrupt flag is cleared before enabling.
 *
 * @param[in]   EXTICfg  Pointer to an EXTI_InitTypeDef structure containing the configuration
 *                       information for the specified external interrupt line.
 *********************************************************************/
void EXTI_ConfigEnable(EXTI_InitTypeDef* EXTICfg) {
    EXTI_Config(EXTICfg);
    EXTI_EnableIRQ(EXTICfg->EXTI_Line);
}

/*********************************************************************/ /**
 * @brief       Sets the mode (level or edge sensitivity) for a specific EXTI line.
 *
 * @param[in]   EXTILine  External interrupt line, must be:
 *                        - EXTI_EINTx, where x is in the range [0,3].
 * @param[in]   mode      Mode selection, must be:
 *                        - EXTI_MODE_LEVEL_SENSITIVE
 *                        - EXTI_MODE_EDGE_SENSITIVE
 * @note        If the mode value is invalid, the function does nothing.
*********************************************************************/
void EXTI_SetMode(EXTI_LINE_ENUM EXTILine, EXTI_MODE_ENUM mode) {
    if (mode == EXTI_MODE_EDGE_SENSITIVE) {
        LPC_SC->EXTMODE |= (1 << EXTILine);
    } else if (mode == EXTI_MODE_LEVEL_SENSITIVE) {
        LPC_SC->EXTMODE &= ~(1 << EXTILine);
    }
}

/*********************************************************************/ /**
 * @brief       Sets the polarity (active level or edge) for a specific EXTI line.
 *
 * @param[in]   EXTILine  External interrupt line, must be:
 *                        - EXTI_EINTx, where x is in the range [0,3].
 * @param[in]   polarity  Polarity selection, should be:
 *                        - EXTI_POLARITY_LOW_ACTIVE or EXTI_POLARITY_FALLING_EDGE (equivalent)
 *                        - EXTI_POLARITY_HIGH_ACTIVE or EXTI_POLARITY_RISING_EDGE (equivalent)
 * @note        If the polarity value is invalid, the function does nothing.
*********************************************************************/
void EXTI_SetPolarity(EXTI_LINE_ENUM EXTILine, EXTI_POLARITY_ENUM polarity) {
    if (polarity == EXTI_POLARITY_HIGH_ACTIVE) {
        LPC_SC->EXTPOLAR |= (1 << EXTILine);
    } else if (polarity == EXTI_POLARITY_LOW_ACTIVE) {
        LPC_SC->EXTPOLAR &= ~(1 << EXTILine);
    }
}

/**
 * @}
 */

#endif /* _EXTI */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
