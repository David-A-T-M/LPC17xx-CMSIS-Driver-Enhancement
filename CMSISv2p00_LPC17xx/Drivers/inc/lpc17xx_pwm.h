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
 * Date: 04/10/2025, Author: David Trujillo Medina
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

/* ---------------------------- BIT DEFINITIONS ----------------------------- */
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
    PWM_MODE_TIMER = 0,
    PWM_COUNTER_RISING_MODE,
    PWM_COUNTER_FALLING_MODE,
    PWM_COUNTER_ANY_MODE
} PWM_TIM_MODE;
/** Check PWM timer/counter mode parameter. */
#define PARAM_PWM_TIM_MODE(n) ((n >= PWM_MODE_TIMER) && (n <= PWM_COUNTER_ANY_MODE))

/**
 * @brief Timer/Counter prescale option.
 */
typedef enum {
    PWM_TICKVAL = 0,
    PWM_USVAL
} PWM_PRESCALE;
/** Check PWM prescale option parameter. */
#define PARAM_PWM_PRESCALE(OPT) ((OPT == PWM_TICKVAL) || (OPT == PWM_USVAL))

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
 * @brief PWM Match update options.
 */
typedef enum {
    PWM_UPDATE_NOW = 0,
    PWM_UPDATE_RESET
} PWM_UPDATE_OPT;
/** Check PWM match update option parameter. */
#define PARAM_PWM_UPDATE_OPT(OPT) ((OPT == PWM_UPDATE_NOW) || (OPT == PWM_UPDATE_RESET))

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
    PWM_PRESCALE prescaleOption; /**< Should be:
                                    - PWM_TICKVAL : Absolute value.
                                    - PWM_USVAL   : Value in microseconds. */
    uint32_t prescaleValue;      /**< Prescale max value. */
} PWM_TIMERCFG_Type;

/**
 * @brief PWM configuration structure for COUNTER mode.
 */
typedef struct {
    PWM_CAPTURE countInputSelect; /**< Should be:
                                        - PWM_CAPTURE_0 : CAPn.0 input pin for PWM timer.
                                        - PWM_CAPTURE_1 : CAPn.1 input pin for PWM timer. */
} PWM_COUNTERCFG_Type;

/**
 * @brief Match channel configuration structure.
 */
typedef struct {
    PWM_MATCH_OPT matchChannel;   /**< PWM_MATCH_x [0...6]. */
    FunctionalState intOnMatch;   /**< Should be:
                                            - ENABLE  : Enable interrupt on match.
                                            - DISABLE : Disable interrupt on match. */
    FunctionalState stopOnMatch;  /**< Should be:
                                            - ENABLE  : Stop timer on match.
                                            - DISABLE : Do not stop timer on match. */
    FunctionalState resetOnMatch; /**< Should be:
                                            - ENABLE  : Reset timer on match.
                                            - DISABLE : Do not reset timer on match. */
    uint32_t matchValue;          /**< Match value to compare with timer counter. */
} PWM_MATCHCFG_Type;

/**
 * @brief Capture input configuration structure.
 */
typedef struct {
    PWM_CAPTURE captureChannel;   /**< PWM_CAPTURE_x [0...1]. */
    FunctionalState risingEdge;   /**< Should be:
                                        - ENABLE  : Enable capture on rising edge.
                                        - DISABLE : Disable capture on rising edge. */
    FunctionalState fallingEdge;  /**< Should be:
                                        - ENABLE  : Enable capture on falling edge.
                                        - DISABLE : Disable capture on falling edge. */
    FunctionalState intOnCapture; /**< Should be:
                                        - ENABLE  : Enable interrupt on capture event.
                                        - DISABLE : Disable interrupt on capture event. */
} PWM_CAPTURECFG_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup PWM_Public_Functions PWM Public Functions
 * @{
 */

/**
 * @brief      Initializes the PWM peripheral in timer or counter mode.
 *
 * This function enables power and clock for the PWM peripheral, configures
 * its mode (timer or counter), sets the prescaler or counter input as required,
 * and resets all relevant registers. The PWM is left in a disabled state after initialization.
 *
 * @param[in]  mode         PWM timer/counter mode selection:
 *                          - PWM_MODE_TIMER
 *                          - PWM_COUNTER_RISING_MODE
 *                          - PWM_COUNTER_FALLING_MODE
 *                          - PWM_COUNTER_ANY_MODE
 * @param[in]  pwmCfg       Pointer to configuration structure:
 *                          - PWM_TIMERCFG_Type for timer mode
 *                          - PWM_COUNTERCFG_Type for counter mode
 *
 * @note:
 * - The function enables the PWM's power and sets the peripheral clock divider.
 * - It resets and initializes the prescaler and counters.
 * - It clears all interrupt flags in the IR register.
 */
void PWM_Init(PWM_TIM_MODE mode, void* pwmCfg);

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
 * @brief      Initializes a PWM configuration structure with default values.
 *
 * This function sets default values for the provided configuration structure,
 * depending on the selected mode. For timer mode, it sets the prescale option
 * to microseconds and the prescale value to 1. For counter mode, it sets the
 * count input select to capture channel 0.
 *
 * @param[in]  mode     PWM timer/counter mode selection:
 *                      - PWM_MODE_TIMER
 *                      - PWM_COUNTER_RISING_MODE
 *                      - PWM_COUNTER_FALLING_MODE
 *                      - PWM_COUNTER_ANY_MODE
 * @param[out] pwmCfg   Pointer to configuration structure to initialize:
 *                      - PWM_TIMERCFG_Type for timer mode
 *                      - PWM_COUNTERCFG_Type for counter mode
 *
 * @note       Call this function before configuring the PWM to ensure
 *             the structure has valid default values.
 */
void PWM_ConfigStructInit(PWM_TIM_MODE mode, void* pwmCfg);

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
 * @brief      Enables or disables the output for a specific PWM channel.
 *
 * This function sets or clears the output enable bit for the selected PWM channel
 * in the PWM Control Register (PCR).
 *
 * @param[in]  channel   PWM channel to control (PWM_CHANNEL_x [1...6]).
 * @param[in]  newState  Functional state:
 *                       - ENABLE  : Enable output for this PWM channel.
 *                       - DISABLE : Disable output for this PWM channel.
 *
 * @note
 * - Use this function to enable or disable the PWM output on a per-channel basis.
 * - The PWM peripheral must be initialized before calling this function.
 */
void PWM_ChannelCmd(PWM_CHANNEL channel, FunctionalState newState);

/**
 * @brief      Enables or disables the PWM peripheral.
 *
 * This function sets or clears the PWM enable bit in the TCR register of LPC_PWM1.
 *
 * @param[in]  newState  Functional state:
 *                       - ENABLE  : Enable PWM operation.
 *                       - DISABLE : Disable PWM operation.
 *
 * @note:
 * - Use this function to start or stop PWM output after configuration.
 */
void PWM_Cmd(FunctionalState newState);

/**
 * @brief      Enables or disables the PWM counter.
 *
 * This function sets or clears the counter enable bit in the TCR register of LPC_PWM1.
 *
 * @param[in]  NewState  Functional state:
 *                       - ENABLE  : Enable the counter.
 *                       - DISABLE : Disable the counter.
 *
 * @note:
 * - Use this function to start or stop the PWM counter.
 */
void PWM_CounterCmd(FunctionalState NewState);

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
void PWM_ConfigMatch(const PWM_MATCHCFG_Type* pwmMatchCfg);

/**
 * @brief      Updates the match value for a specified PWM channel.
 *
 * This function sets the match register (MR0-MR6) of LPC_PWM1 to the provided value
 * for the selected channel and optionally resets the PWM counter immediately.
 *
 * @param[in]  channel        PWM match channel to update (PWM_MATCH_x [0...6]).
 * @param[in]  newMatchValue  New value to set in the match register.
 * @param[in]  updateType     Update type:
 *                            - PWM_UPDATE_NOW      : Update and reset the counter immediately.
 *                            - PWM_UPDATE_RESET    : Update on next PWM reset event.
 *
 * @note
 * - Only the specified channel is affected.
 * - Use this function to change the match value during runtime.
 */
void PWM_MatchUpdate(PWM_MATCH_OPT channel, uint32_t newMatchValue, PWM_UPDATE_OPT updateType);

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
void PWM_ConfigCapture(PWM_CAPTURECFG_Type* capCfg);

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
