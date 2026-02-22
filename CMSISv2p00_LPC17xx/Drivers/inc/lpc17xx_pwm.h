/**
 * @file        lpc17xx_pwm.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for PWM firmware library on LPC17xx
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
 * Last update: 21/02/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup PWM PWM
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_PWM_H_
#define LPC17XX_PWM_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup PWM_Private_Macros PWM Private Macros
 * @{
 */

/* ------------------------ MACROS MASKS DEFINITIONS ------------------------ */

/** CTCR register mode mask. */
#define PWM_CTCR_MODE_MASK ((0x3))
/** CTCR register count input select mask. */
#define PWM_CTCR_INPUT_MASK ((0xC))

/* ------------------------- MACROS BIT DEFINITIONS ------------------------- */
/** CTCR register mode mask. */
#define PWM_CTCR_MODE_MASK          ((0x3))
/** PWM Capture input select */
#define PWM_CTCR_SELECT_INPUT(n)    ((uint32_t)((n) << 2))
/** IR register mask */
#define PWM_IR_BITMASK              ((uint32_t)(0x0000073F))
/** MCR interrupt on MATCHx bit. */
#define PWM_MCR_INT(MRx)            ((uint32_t)(1 << (3 * (MRx))))
/** MCR reset on MATCHx bit. */
#define PWM_MCR_RESET(MRx)          ((uint32_t)(1 << ((3 * (MRx)) + 1)))
/** MCR stop on MATCHx bit. */
#define PWM_MCR_STOP(MRx)           ((uint32_t)(1 << ((3 * (MRx)) + 2)))
/** MCR register channel mask bit. */
#define PWM_MCR_CHANNEL_MASKBIT(CH) ((uint32_t)(7 << (CH * 3)))
/** TCR counter enable bit. */
#define PWM_TCR_COUNTER_ENABLE      ((uint32_t)(1 << 0))
/** TCR counter reset bit. */
#define PWM_TCR_COUNTER_RESET       ((uint32_t)(1 << 1))
/** TCR PWM enable bit. */
#define PWM_TCR_PWM_ENABLE          ((uint32_t)(1 << 3))
/** PCR edge select bit. */
#define PWM_PCR_PWMSELn(CH)         ((uint32_t)(_BIT(CH)))
/** PCR PWM channel output enable bit. */
#define PWM_PCR_PWMENAn(CH)         ((uint32_t)((1) << ((CH) + 8)))
/** CCR register channel mask bit. */
#define PWM_CCR_CHANNEL_MASKBIT(CH) ((uint32_t)(7 << (CH * 3)))
/** CCR rising edge sensitive channel bit. */
#define PWM_CCR_CAP_RISING(CAPx)    ((uint32_t)(1 << (((CAPx & 0x2) << 1) + (CAPx & 0x1))))
/** CCR falling edge sensitive channel bit.*/
#define PWM_CCR_CAP_FALLING(CAPx)   ((uint32_t)(1 << (((CAPx & 0x2) << 1) + (CAPx & 0x1) + 1)))
/** CCR interrupt on event channel bit. */
#define PWM_CCR_INT_ON_CAP(CAPx)    ((uint32_t)(1 << (((CAPx & 0x2) << 1) + (CAPx & 0x1) + 2)))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup PWM_Public_Types PWM Public Types
 * @{
 */

/**
 * @brief Timer/counter operating mode.
 */
typedef enum {
    PWM_TIMER_MODE = 0,
    PWM_COUNTER_MODE
} PWM_MODE;
/** Check PWM mode option parameter. */
#define PARAM_PWM_TIM_MODE(n) ((n >= PWM_TIMER_MODE) && (n <= PWM_COUNTER_MODE))

/**
 * @brief Counter mode edge selection.
 */
typedef enum {
    PWM_CTR_RISING = 1,
    PWM_CTR_FALLING,
    PWM_CTR_ANY
} PWM_CTR_EDGE;
/** Check PWM counter mode edge selection parameter. */
#define PARAM_PWM_CTR_EDGE(EDGE) ((EDGE) >= PWM_CTR_RISING && (EDGE) <= PWM_CTR_ANY)

/**
 * @brief Timer/Counter prescale option.
 */
typedef enum {
    PWM_TICK = 0,
    PWM_US
} PWM_PRESCALE;
/** Check PWM prescale option parameter. */
#define PARAM_PWM_PRESCALE(OPT) ((OPT == PWM_TICK) || (OPT == PWM_US))

/**
 * @brief Capture channel enum and parameter macro
 */
typedef enum {
    PWM_CAPTURE_0 = 0,
    PWM_CAPTURE_1
} PWM_CAPTURE;
/** Check PWM capture channel parameter. */
#define PARAM_PWM_CAPTURE(CH) (((CH) >= PWM_CAPTURE_0) && ((CH) <= PWM_CAPTURE_1))

/**
 *@brief PWM channel enum and parameter macro
 */
typedef enum {
    PWM_CHANNEL_1 = 1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4,
    PWM_CHANNEL_5,
    PWM_CHANNEL_6
} PWM_CHANNEL;
/** Check PWM channel parameter. */
#define PARAM_PWM_CHANNEL(CH) ((CH) >= PWM_CHANNEL_1 && (CH) <= PWM_CHANNEL_6)

/**
 *@brief Match register enum and parameter macro
 */
typedef enum {
    PWM_MATCH_0 = 0,
    PWM_MATCH_1,
    PWM_MATCH_2,
    PWM_MATCH_3,
    PWM_MATCH_4,
    PWM_MATCH_5,
    PWM_MATCH_6
} PWM_MATCH_OPT;
/** Check PWM match register parameter. */
#define PARAM_PWM_MATCH_OPT(CH) ((CH) >= PWM_MATCH_0 && (CH) <= PWM_MATCH_6)

/**
 * @brief PWM operating mode options.
 */
typedef enum {
    PWM_SINGLE_EDGE = 0,
    PWM_DUAL_EDGE
} PWM_CHANNEL_EDGE;
/** Check PWM channel edge mode parameter. */
#define PARAM_PWM_CHANNEL_EDGE(n) ((n == PWM_SINGLE_EDGE) || (n == PWM_DUAL_EDGE))

/**
 * @brief PWM Interrupt status type.
 */
typedef enum {
    PWM_MR0_INT = 0,
    PWM_MR1_INT,
    PWM_MR2_INT,
    PWM_MR3_INT,
    PWM_CR0_INT,
    PWM_CR1_INT,
    PWM_MR4_INT = 8,
    PWM_MR5_INT,
    PWM_MR6_INT
} PWM_INT_TYPE;
/** Check PWM interrupt type parameter. */
#define PARAM_PWM_INT_TYPE(TYPE) \
    (((TYPE) >= PWM_MR0_INT && (TYPE) <= PWM_CR1_INT) || ((TYPE) >= PWM_MR4_INT && (TYPE) <= PWM_MR6_INT))

/**
 * @brief PWM pin selection options.
 */
typedef enum {
    PWM1_P1_18,
    PWM1_P2_0,
    PWM2_P1_20,
    PWM2_P2_1,
    PWM2_P3_25,
    PWM3_P1_21,
    PWM3_P2_2,
    PWM3_P3_26,
    PWM4_P1_23,
    PWM4_P2_3,
    PWM5_P1_24,
    PWM5_P2_4,
    PWM6_P1_26,
    PWM6_P2_5,
} PWM_PIN_OPTION;
/** Check PWM pin option parameter. */
#define PARAM_PWM_PIN_OPTION(OPT) ((OPT >= PWM1_P1_18) && (OPT <= PWM6_P2_5))

/**
 * @brief PWM configuration structure for TIMER mode.
 */
typedef struct {
    PWM_PRESCALE prescaleOpt; /**< Should be:
                                - PWM_TICK: Absolute value.
                                - PWM_US  : Value in microseconds. */
    uint32_t prescaleValue;   /**< Prescale max value. */
} PWM_TIMERCFG_T;

/**
 * @brief PWM configuration structure for COUNTER mode.
 */
typedef struct {
    PWM_CAPTURE input; /**< Should be:
                        - PWM_CAPTURE_0 : CAPn.0 input pin for PWM timer.
                        - PWM_CAPTURE_1 : CAPn.1 input pin for PWM timer. */
    PWM_CTR_EDGE edge; /**< Should be:
                        - PWM_CTR_RISING  : Count rising edges on the selected capture input.
                        - PWM_CTR_FALLING : Count falling edges on the selected capture input.
                        - PWM_CTR_ANY     : Count both rising and falling edges on the selected capture input. */
} PWM_COUNTERCFG_T;

/**
 * @brief Capture input configuration structure.
 */
typedef struct {
    PWM_CAPTURE channel;       /**< PWM_CAPTURE_x [0...1]. */
    FunctionalState risingEn;  /**< Should be:
                                - ENABLE  : Enable capture on rising edge.
                                - DISABLE : Disable capture on rising edge. */
    FunctionalState fallingEn; /**< Should be:
                                - ENABLE  : Enable capture on falling edge.
                                - DISABLE : Disable capture on falling edge. */
    FunctionalState intEn;     /**< Should be:
                                - ENABLE  : Enable interrupt on capture event.
                                - DISABLE : Disable interrupt on capture event. */
} PWM_CAPTURECFG_T;

/**
 * @brief Match channel configuration structure.
 */
typedef struct {
    PWM_MATCH_OPT channel;   /**< PWM_MATCH_x [0...6]. */
    FunctionalState intEn;   /**< Should be:
                                - ENABLE  : Enable interrupt on match.
                                - DISABLE : Disable interrupt on match. */
    FunctionalState stopEn;  /**< Should be:
                                - ENABLE  : Stop timer on match.
                                - DISABLE : Do not stop timer on match. */
    FunctionalState resetEn; /**< Should be:
                                - ENABLE  : Reset timer on match.
                                - DISABLE : Do not reset timer on match. */
    uint32_t matchValue;     /**< Match value to compare with timer counter. */
} PWM_MATCHCFG_T;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup PWM_Public_Functions PWM Public Functions
 * @{
 */

/**
 * @brief      Initializes the PWM peripheral in TIMER mode.
 *
 * This function configures the PWM peripheral to operate in TIMER mode based on the
 * provided configuration structure. It sets the prescale option and value, and prepares
 * the PWM for operation. The function also enables the clock for the PWM peripheral.
 *
 * @param[in]  timerCfg   Pointer to a PWM_TIMERCFG_T structure containing timer configuration.
 *
 * @note:
 * - Call this function before using any other PWM functions to set up the timer mode.
 * - The PWM must be enabled with PWM_CounterEnable() and PWM_Enable() after initialization to start operation.
 */
void PWM_InitTimer(const PWM_TIMERCFG_T* timerCfg);

/**
 * @brief      Initializes the PWM peripheral in COUNTER mode.
 *
 * This function configures the PWM peripheral to operate in COUNTER mode based on the
 * provided configuration structure. It sets the capture input and edge detection options,
 * and prepares the PWM for operation. The function also enables the clock for the PWM peripheral.
 *
 * @param[in]  counterCfg Pointer to a PWM_COUNTERCFG_T structure containing counter configuration.
 *
 * @note:
 * - Call this function before using any other PWM functions to set up the counter mode.
 * - The PWM must be enabled with PWM_CounterEnable() and PWM_Enable() after initialization to start operation.
 */
void PWM_InitCounter(const PWM_COUNTERCFG_T* counterCfg);

/**
 * @brief      De-initializes the PWM peripheral.
 *
 * This function disables the PWM by clearing its control register and powers down
 * the PWM peripheral to save power. After calling this function, the PWM must be
 * re-initialized before use.
 *
 * @note:
 * - The function disables the PWM and its clock.
 */
void PWM_DeInit(void);

/**
 * @brief      Configures the pin for the specified PWM output channel.
 *
 * This function selects and configures the appropriate pin for the given
 * PWM output channel using the PINSEL API. Only output channels are supported.
 * Capture pins (for PWM capture functionality) must be configured manually.
 *
 * @param[in]  option  PWM output pin option (see PWM_PIN_OPTION).
 *
 * @note
 * - Only output channels are configurable with this function.
 * - For capture channels, configure the pin manually with PINSEL_ConfigPin.
 * - The mapping is fixed and based on the device datasheet.
 */
void PWM_PinConfig(PWM_PIN_OPTION option);

/**
 * @brief      Configures the edge mode for a specified PWM channel.
 *
 * This function sets the selected PWM channel to single or dual edge mode by updating
 * the PWM Control Register (PCR). Only channels 2 to 6 support edge mode configuration.
 * Channel 1 is not configurable for edge mode.
 *
 * @param[in]  channel   PWM match channel to configure (PWM_CHANNEL_x [2...6]).
 * @param[in]  edgeMode  Edge mode option:
 *                       - PWM_SINGLE_EDGE : Single edge mode.
 *                       - PWM_DUAL_EDGE   : Dual edge mode.
 *
 * @note
 * - Channel 1 is not configurable for edge mode.
 * - Use this function after initializing the PWM peripheral.
 */
void PWM_ChannelConfig(PWM_CHANNEL channel, PWM_CHANNEL_EDGE edgeMode);

/**
 * @brief      Enables the output for a specific PWM channel.
 *
 * This function sets the output enable bit for the selected PWM channel
 * in the PWM Control Register (PCR).
 *
 * @param[in]  channel   PWM channel to control (PWM_CHANNEL_x [1...6]).
 *
 * @note
 * - Use this function to enable the PWM output on a per-channel basis.
 * - The PWM peripheral must be initialized before calling this function.
 */
void PWM_ChannelEnable(PWM_CHANNEL channel);

/**
 * @brief      Disables the output for a specific PWM channel.
 *
 * This function clears the output enable bit for the selected PWM channel
 * in the PWM Control Register (PCR).
 *
 * @param[in]  channel   PWM channel to control (PWM_CHANNEL_x [1...6]).
 *
 * @note
 * - Use this function to disable the PWM output on a per-channel basis.
 * - The PWM peripheral must be initialized before calling this function.
 */
void PWM_ChannelDisable(PWM_CHANNEL channel);

/**
 * @brief      Enables the PWM peripheral.
 *
 * This function sets the PWM enable bit in the TCR register of LPC_PWM1.
 *
 * @note:
 * - Use this function to start PWM output after configuration.
 */
void PWM_Enable();

/**
 * @brief      Disables the PWM peripheral.
 *
 * This function clears the PWM enable bit in the TCR register of LPC_PWM1.
 *
 * @note:
 * - Use this function to stop PWM output.
 */
void PWM_Disable();

/**
 * @brief      Enables the PWM counter.
 *
 * This function sets the counter enable bit in the TCR register of LPC_PWM1.
 *
 * @note:
 * - Use this function to start or stop the PWM counter.
 */
void PWM_CounterEnable();

/**
 * @brief      Disables the PWM counter.
 *
 * This function clears the counter enable bit in the TCR register of LPC_PWM1.
 *
 * @note:
 * - Use this function to stop the PWM counter while retaining its current count value.
 */
void PWM_CounterDisable();

/**
 * @brief      Resets the PWM counter.
 *
 * This function synchronously resets the Timer Counter (TC) and Prescale Counter (PC)
 * of the PWM peripheral (LPC_PWM1) by setting and then clearing the reset bit in the TCR register.
 *
 * @note:
 * - Use this function to reset the PWM counters to zero.
 */
void PWM_ResetCounter(void);

/**
 * @brief      Configures the match channel for the PWM peripheral.
 *
 * This function sets up the interrupt, reset, and stop actions for the specified
 * PWM match channel according to the provided configuration structure. It updates
 * the PWM Match Control Register (MCR) to enable or disable interrupt, reset, and
 * stop on match for the selected channel, and sets the match value in the corresponding
 * match register.
 *
 * @param[in]  pwmMatchCfg  Pointer to a PWM_MATCHCFG_Type structure.
 *
 * @note:
 * - This function only configures the match control actions and sets the match value.
 * - Call this function after initializing the PWM to set up match behavior.
 */
void PWM_ConfigMatch(const PWM_MATCHCFG_T* pwmMatchCfg);

/**
 * @brief      Updates the match value for a specified PWM channel.
 *
 * This function sets the match register (MR0-MR6) of LPC_PWM1 to the provided value
 * for the selected channel.
 *
 * @param[in]  match          PWM match register to update (PWM_MATCH_x [0...6]).
 * @param[in]  newMatchValue  New value to set in the match register.
 *
 * @note
 * - Only the specified channel is affected.
 * - Use this function to change the match value during runtime.
 * - The new match value will take effect after the next match event for that channel.
 * - To force an immediate update of the match value, use PWM_ResetCounter() after calling this function.
 */
void PWM_MatchUpdateSingle(PWM_MATCH_OPT match, uint32_t newMatchValue);

/**
 * @brief      Updates the match values for two specified PWM channels simultaneously.
 *
 * This function sets the match registers of LPC_PWM1 for two selected channels to the provided values
 * in a single operation. This is useful for synchronously updating match values for related channels.
 *
 * @param[in]  matchA       First PWM match channel to update (PWM_MATCH_x [0...6]).
 * @param[in]  newValueA      New value to set in the match register for channel A.
 * @param[in]  matchB       Second PWM match channel to update (PWM_MATCH_x [0...6]).
 * @param[in]  newValueB      New value to set in the match register for channel B.
 *
 * @note
 * - Both specified channels are updated simultaneously.
 * - Use this function to change match values for two channels at the same time during runtime.
 * - The new match values will take effect after the next match event for each respective channel.
 * - To force an immediate update of the match values, use PWM_ResetCounter() after calling this function.
 */
void PWM_MatchUpdateDouble(PWM_MATCH_OPT matchA, uint32_t newValueA, PWM_MATCH_OPT matchB, uint32_t newValueB);

/**
 * @brief      Clears the specified PWM interrupt pending flag.
 *
 * This function clears the interrupt pending flag for the given PWM match or capture
 * channel in the PWM's interrupt register (IR). It should be used to acknowledge and
 * clear PWM interrupts after they are handled.
 *
 * @param[in]  intFlag  Interrupt type to clear:
 *                      - PWM_MRx_INT [0...6].
 *                      - PWM_CRx_INT [0...1].
 *
 * @note:
 * - Only the specified interrupt flag is cleared.
 * - The function operates on LPC_PWM1.
 */
void PWM_ClearIntPending(PWM_INT_TYPE intFlag);

/**
 * @brief      Gets the interrupt status for the specified PWM channel.
 *
 * This function checks if the interrupt flag for the given match or capture channel
 * is set in the PWM's interrupt register (IR). It can be used for both match and
 * capture interrupts.
 *
 * @param[in]  intFlag  Interrupt type to check:
 *                      - PWM_MRx_INT [0...6].
 *                      - PWM_CRx_INT [0...1].
 *
 * @return     FlagStatus
 *             - SET   : Interrupt is pending
 *             - RESET : No interrupt pending
 *
 * @note:
 * - Only the specified interrupt flag is checked.
 */
FlagStatus PWM_GetIntStatus(PWM_INT_TYPE intFlag);

/**
 * @brief      Configures the capture channel for the PWM peripheral.
 *
 * This function sets up the capture behavior for the selected channel, including
 * edge detection (rising, falling), interrupt generation, and channel selection.
 *
 * @param[in]  capCfg  Pointer to a PWM_CAPTURECFG_Type structure.
 *
 * @note
 * - Only the specified channel is affected.
 * - Call this function after initializing the PWM to set up capture behavior.
 */
void PWM_ConfigCapture(const PWM_CAPTURECFG_T* capCfg);

/**
 * @brief      Reads the value of the capture register for the specified PWM channel.
 *
 * This function returns the value stored in the capture register (CR0 or CR1)
 * of the PWM peripheral, depending on the selected capture channel.
 *
 * @param[in]  capChannel  Capture channel to read:
 *                         - PWM_CAPTURE_0 : CAP0 input pin for PWM
 *                         - PWM_CAPTURE_1 : CAP1 input pin for PWM
 *
 * @return     Value of the selected capture register.
 *
 * @note
 * - Use this function to obtain the timestamp captured on the specified input.
 * - The PWM must be configured for capture mode before using this function.
 */
uint32_t PWM_GetCaptureValue(PWM_CAPTURE capChannel);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_PWM_H_ */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
