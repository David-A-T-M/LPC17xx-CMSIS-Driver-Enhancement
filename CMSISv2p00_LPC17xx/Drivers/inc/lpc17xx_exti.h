/***********************************************************************/ /**
 * @file		lpc17xx_exti.h
 * @brief		Contains all macro definitions and function prototypes
 * 				support for External interrupt firmware library on LPC17xx
 * @version		2.0
 * @date		21. May. 2010
 * @author		NXP MCU SW Application Team
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
 **************************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup EXTI EXTI
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_EXTI_H_
#define LPC17XX_EXTI_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private Macros ------------------------------------------------------------- */
/** @defgroup EXTI_Private_Macros EXTI Private Macros
 * @{
 */

/*********************************************************************/ /**
 * Macro defines for EXTI masks.
 **********************************************************************/
#define EINT_ALL_MASK ((0xF))

/**
 * @}
 */

/* Private Macros ------------------------------------------------------------- */
/** @defgroup EXTI_Public_Types EXTI Public Types
 * @{
 */

/**
 * @brief EXTI external interrupt line option.
 */
typedef enum {
    EXTI_EINT0, /*!<  External interrupt 0, P2.10.*/
    EXTI_EINT1, /*!<  External interrupt 0, P2.11.*/
    EXTI_EINT2, /*!<  External interrupt 0, P2.12.*/
    EXTI_EINT3  /*!<  External interrupt 0, P2.13.*/
} EXTI_LINE_ENUM;

/**
 * @brief EXTI mode option.
 */
typedef enum {
    EXTI_MODE_LEVEL_SENSITIVE, /*!< Level sensitivity is selected.*/
    EXTI_MODE_EDGE_SENSITIVE   /*!< Edge sensitivity is selected.*/
} EXTI_MODE_ENUM;

/**
 * @brief EXTI polarity option.
 */
typedef enum {
    EXTI_POLARITY_LOW_ACTIVE = 0,   /*!< Low active sensitive.*/
    EXTI_POLARITY_FALLING_EDGE = 0, /*!< Falling edge sensitive.*/
    EXTI_POLARITY_HIGH_ACTIVE = 1,  /*!< High active sensitive.*/
    EXTI_POLARITY_RISING_EDGE = 1   /*!< Rising edge sensitive.*/
} EXTI_POLARITY_ENUM;

/**
 * @brief EXTI Initialize structure.
 */
typedef struct {
    EXTI_LINE_ENUM EXTI_Line;           /*!<Select external interrupt pin (EINTx) */

    EXTI_MODE_ENUM EXTI_Mode;           /*!< Choose between Level or Edge sensitivity */

    EXTI_POLARITY_ENUM EXTI_Polarity;   /*!< Selects the active level (low/high) if EXTI_Mode is level-sensitive,
                                             or the edge (falling/rising) if EXTI_Mode is edge-sensitive. */

} EXTI_CFG_Type;

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup EXTI_Public_Functions EXTI Public Functions
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
void EXTI_Init(void);

/*********************************************************************/ /**
 * @brief       Configures a specific External Interrupt (EXTI) line.
 *
 * This function disables the corresponding external IRQ in the NVIC before making any changes,
 * sets the mode and polarity for the selected EXTI line.
 *
 * @param[in]   EXTICfg  Pointer to an EXTI_CFG_Type structure containing the configuration
 *                       information for the specified external interrupt line.
 *********************************************************************/
void EXTI_Config(const EXTI_CFG_Type* EXTICfg);

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
 * @param[in]   EXTICfg  Pointer to an EXTI_CFG_Type structure containing the configuration
 *                       information for the specified external interrupt line.
 *********************************************************************/
void EXTI_ConfigEnable(const EXTI_CFG_Type* EXTICfg);

/*********************************************************************/ /**
 * @brief       Clears the external interrupt flag for the specified EXTI line.
 * @param[in]   EXTILine  External interrupt line, must be:
 *                        - EXTI_EINTx, where x is in the range [0,3].
 *********************************************************************/
static __INLINE void EXTI_ClearFlag(EXTI_LINE_ENUM EXTILine) {
    LPC_SC->EXTINT |= (1 << EXTILine);
}

/*********************************************************************/ /**
 * @brief       Gets the status of the external interrupt flag for the specified EXTI line.
 * @param[in]   EXTILine  External interrupt line, must be:
 *                        - EXTI_EINTx, where x is in the range [0,3].
 * @return      1 if the interrupt flag is set, 0 otherwise.
 *********************************************************************/
static __INLINE uint8_t EXTI_GetFlag(EXTI_LINE_ENUM EXTILine) {
    return (LPC_SC->EXTINT & (1 << EXTILine)) ? 1 : 0;
}

/*********************************************************************/ /**
 * @brief       Clears the interrupt flag and enables the IRQ for the specified EXTI line.
 * @param[in]   EXTILine  External interrupt line, must be:
 *                        - EXTI_EINTx, where x is in the range [0,3].
 *********************************************************************/
static __INLINE void EXTI_EnableIRQ(EXTI_LINE_ENUM EXTILine) {
    EXTI_ClearFlag(EXTILine);
    NVIC_ClearPendingIRQ((IRQn_Type)(EINT0_IRQn + EXTILine));
    NVIC_EnableIRQ((IRQn_Type)(EINT0_IRQn + EXTILine));
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_EXTI_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
