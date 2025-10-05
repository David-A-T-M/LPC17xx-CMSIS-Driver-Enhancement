/**
 * @file        lpc17xx_timer.h
 * @brief       Contains all functions support for Timer firmware library on LPC17xx
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
 * Date: 01/08/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup TIM TIM
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_TIMER_H_
#define LPC17XX_TIMER_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup TIM_Private_Macros TIM Private Macros
 * @{
 */
/* ------------------------ MACROS MASKS DEFINITIONS ------------------------ */
/** TCR register mask. */
#define TIM_TCR_MASKBIT            ((uint32_t)(3))
/** MCR register mask. */
#define TIM_MCR_MASKBIT            ((uint32_t)(0x0FFF))
/** MCR register channel mask bit. */
#define TIM_MCR_CHANNEL_MASKBIT(n) ((uint32_t)(7 << (n * 3)))
/** CCR register mask. */
#define TIM_CCR_MASKBIT            ((uint32_t)(0x3F))
/** CCR register channel mask bit. */
#define TIM_CCR_CHANNEL_MASKBIT(n) ((uint32_t)(7 << (n * 3)))
/** EMR register mask. */
#define TIM_EMR_MASKBIT            ((0x0FFF))
/** EMR register channel external control mask bit. */
#define TIM_EMR_MASK(n)            (_SBF(((n << 1) + 4), 0x03))
/** CTCR register mask. */
#define TIM_CTCR_MASKBIT           ((0xF))
/** CTCR register mode mask. */
#define TIM_CTCR_MODE_MASK         ((0x3))
/** CTCR register count input select mask. */
#define TIM_CTCR_INPUT_MASK        ((0xC))

/* ------------------------- MACROS BIT DEFINITIONS ------------------------- */
/** Timer/counter enable bit. */
#define TIM_ENABLE            ((uint32_t)(1 << 0))
/** Timer/counter reset bit. */
#define TIM_RESET             ((uint32_t)(1 << 1))
/** Interrupt flag bit. */
#define TIM_IR_CLR(n)         (_BIT(n))
/** Interrupt flag bit for match register n. */
#define TIM_MATCH_INT(n)      (_BIT(n & 0x0F))
/** Interrupt flag bit for capture register n. */
#define TIM_CAP_INT(n)        (_BIT(((n & 0x0F) + 4)))
/** Interrupt on match for match register n. */
#define TIM_INT_ON_MATCH(n)   (_BIT((n * 3)))
/** Reset on match for match register n. */
#define TIM_RESET_ON_MATCH(n) (_BIT(((n * 3) + 1)))
/** Stop on match for match register n. */
#define TIM_STOP_ON_MATCH(n)  (_BIT(((n * 3) + 2)))
/** Capture channel n on CRx rising edge. */
#define TIM_CAP_RISING(n)     (_BIT((n * 3)))
/** Capture channel n on CRx falling edge. */
#define TIM_CAP_FALLING(n)    (_BIT(((n * 3) + 1)))
/** Interrupt on capture channel n. */
#define TIM_INT_ON_CAP(n)     (_BIT(((n * 3) + 2)))
/** Mask bit for rising and falling edge bit. */
#define TIM_EDGE_MASK(n)      (_SBF((n * 3), 0x03))
/** Mask bit for match channel n. */
#define TIM_EM(n)             (_BIT(n))
/** Output state change of MAT.n when external match happens: no change. */
#define TIM_EM_NOTHING        ((uint8_t)(0x0))
/** Output state change of MAT.n when external match happens: low. */
#define TIM_EM_LOW            ((uint8_t)(0x1))
/** Output state change of MAT.n when external match happens: high. */
#define TIM_EM_HIGH           ((uint8_t)(0x2))
/** Output state change of MAT.n when external match happens: toggle. */
#define TIM_EM_TOGGLE         ((uint8_t)(0x3))
/** Macro for setting for the MAT.n change state bits. */
#define TIM_EM_SET(n, s)      (_SBF(((n << 1) + 4), (s & 0x03)))
/** Counter mode bit. */
#define TIM_COUNTER_MODE      ((uint8_t)(1))

/* ---------------------- CHECK PARAMETER DEFINITIONS ----------------------- */
/** Check TIMER parameter. */
#define PARAM_TIMx(n)                                                                      \
    (((uintptr_t)(n) == (uintptr_t)LPC_TIM0) || ((uintptr_t)(n) == (uintptr_t)LPC_TIM1) || \
     ((uintptr_t)(n) == (uintptr_t)LPC_TIM2) || ((uintptr_t)(n) == (uintptr_t)LPC_TIM3))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup TIM_Public_Types TIM Public Types
 * @{
 */

/**
 * @brief interrupt type.
 */
typedef enum {
    TIM_MR0_INT = 0,
    TIM_MR1_INT,
    TIM_MR2_INT,
    TIM_MR3_INT,
    TIM_CR0_INT,
    TIM_CR1_INT
} TIM_INT;
/** Check TIMER interrupt type parameter. */
#define PARAM_TIM_INT(INT) ((INT) >= TIM_MR0_INT && (INT) <= TIM_CR1_INT)

/**
 * @brief Timer/counter operating mode.
 */
typedef enum {
    TIM_TIMER_MODE = 0,
    TIM_COUNTER_RISING_MODE,
    TIM_COUNTER_FALLING_MODE,
    TIM_COUNTER_ANY_MODE
} TIM_MODE;
/** Check TIMER mode option parameter. */
#define PARAM_TIM_MODE(MODE) ((MODE) >= TIM_TIMER_MODE && (MODE) <= TIM_COUNTER_ANY_MODE)

/**
 * @brief Timer/Counter prescale option.
 */
typedef enum {
    TIM_TICKVAL = 0,
    TIM_USVAL
} TIM_PRESCALE;
/** Check TIMER prescale option parameter. */
#define PARAM_TIM_PRESCALE(PS) ((PS == TIM_TICKVAL) || (PS == TIM_USVAL))

/**
 * @brief Timer/Counter external match option.
 */
typedef enum {
    TIM_NOTHING = 0,
    TIM_LOW,
    TIM_HIGH,
    TIM_TOGGLE
} TIM_EXTMATCH_OPT;
/** Check TIMER external match option parameter. */
#define PARAM_TIM_EXTMATCH_OPT(EM) ((EM) >= TIM_NOTHING && (EM) <= TIM_TOGGLE)

/**
 * @brief Timer/counter capture mode options.
 */
typedef enum {
    TIM_NONE = 0,
    TIM_RISING,
    TIM_FALLING,
    TIM_ANY
} TIM_CAP_MODE;
/** Check TIMER capture mode option parameter. */
#define PARAM_TIM_CAP_MODE(MODE) ((MODE) >= TIM_NONE && (MODE) <= TIM_ANY)

/**
 *@brief Match channel enum and parameter macro
 */
typedef enum {
    TIM_MATCH_0 = 0,
    TIM_MATCH_1,
    TIM_MATCH_2,
    TIM_MATCH_3
} TIM_MATCH_CHANNEL;
/** Check TIMER match channel option parameter. */
#define PARAM_TIM_MATCH_CHANNEL(CH) ((CH) >= TIM_MATCH_0 && (CH) <= TIM_MATCH_3)

/**
 * @brief Capture channel enum and parameter macro
 */
typedef enum {
    TIM_CAPTURE_0 = 0,
    TIM_CAPTURE_1
} TIM_CAPTURE_CHANNEL;
/** Check TIMER capture channel option parameter. */
#define PARAM_TIM_CAPTURE_CHANNEL(CH) (((CH) >= TIM_CAPTURE_0) && ((CH) <= TIM_CAPTURE_1))

/**
 * @brief Timer pin selection options.
 */
typedef enum {
    TIM_CAP0_0_P1_26 = 0,
    TIM_CAP0_1_P1_27,
    TIM_MAT0_0_P1_28,
    TIM_MAT0_0_P3_25,
    TIM_MAT0_1_P1_29,
    TIM_MAT0_1_P3_26,
    TIM_CAP1_0_P1_18,
    TIM_CAP1_1_P1_19,
    TIM_MAT1_0_P1_22,
    TIM_MAT1_1_P1_25,
    TIM_CAP2_0_P0_4,
    TIM_CAP2_1_P0_5,
    TIM_MAT2_0_P0_6,
    TIM_MAT2_0_P4_28,
    TIM_MAT2_1_P0_7,
    TIM_MAT2_1_P4_29,
    TIM_MAT2_2_P0_8,
    TIM_MAT2_3_P0_9,
    TIM_CAP3_0_P0_23,
    TIM_CAP3_1_P0_24,
    TIM_MAT3_0_P0_10,
    TIM_MAT3_1_P0_11
} TIM_PIN_OPTION;
/** Check TIMER pin option parameter. */
#define PARAM_TIM_PIN_OPTION(OPT) ((OPT >= TIM_CAP0_0_P1_26) && (OPT <= TIM_MAT3_1_P0_11))

/**
 * @brief Timer configuration structure for TIMER mode.
 */
typedef struct {
    TIM_PRESCALE prescaleOption; /**< Should be:
                                 - TIM_TICKVAL : Absolute value.
                                 - TIM_USVAL   : Value in microseconds. */
    uint32_t prescaleValue;      /**< Prescale max value. */
} TIM_TIMERCFG_Type;

/**
 * @brief Timer configuration structure for COUNTER mode.
 */
typedef struct {
    TIM_CAPTURE_CHANNEL countInputSelect; /**< Should be:
                                          - TIM_CAPTURE_0 : CAPn.0 input pin for TIMERn.
                                          - TIM_CAPTURE_1 : CAPn.1 input pin for TIMERn. */
} TIM_COUNTERCFG_Type;

/**
 * @brief Match channel configuration structure.
 */
typedef struct {
    TIM_MATCH_CHANNEL matchChannel;      /**< TIM_MATCH_CHANNEL_x [0...3]. */
    FunctionalState intOnMatch;          /**< Should be:
                                         - ENABLE  : Enable interrupt on match.
                                         - DISABLE : Disable interrupt on match. */
    FunctionalState stopOnMatch;         /**< Should be:
                                         - ENABLE  : Stop timer on match.
                                         - DISABLE : Do not stop timer on match. */
    FunctionalState resetOnMatch;        /**< Should be:
                                         - ENABLE  : Reset timer on match.
                                         - DISABLE : Do not reset timer on match. */
    TIM_EXTMATCH_OPT extMatchOutputType; /**< Should be:
                                         - TIM_NOTHING : Do nothing for external output pin if matched.
                                         - TIM_LOW     : Force external output pin to low if matched.
                                         - TIM_HIGH    : Force external output pin to high if matched.
                                         - TIM_TOGGLE  : Toggle external output pin if matched. */
    uint32_t matchValue;                 /**< Match value to compare with timer counter. */
} TIM_MATCHCFG_Type;

/**
 * @brief Capture input configuration structure.
 */
typedef struct {
    TIM_CAPTURE_CHANNEL captureChannel; /**< TIM_CAPTURE_x [0...1]. */
    FunctionalState risingEdge;         /**< Should be:
                                        - ENABLE  : Enable capture on rising edge.
                                        - DISABLE : Disable capture on rising edge. */
    FunctionalState fallingEdge;        /**< Should be:
                                        - ENABLE  : Enable capture on falling edge.
                                        - DISABLE : Disable capture on falling edge. */
    FunctionalState intOnCapture;       /**< Should be:
                                        - ENABLE  : Enable interrupt on capture event.
                                        - DISABLE : Disable interrupt on capture event. */
} TIM_CAPTURECFG_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup TIM_Public_Functions TIM Public Functions
 * @{
 */

/**
 * @brief      Initializes the specified Timer/Counter peripheral.
 *
 * This function enables the power and clock for the selected timer, configures
 * its mode (timer or counter), sets the prescaler or counter input as required,
 * resets the Timer Counter (TC) and Prescale Counter (PC), and clears all
 * pending interrupt flags. It prepares the timer for further configuration and use.
 *
 * @param[in]  TIMx         Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  mode         Timer/counter mode selection:
 *                          - TIM_TIMER_MODE
 *                          - TIM_COUNTER_RISING_MODE
 *                          - TIM_COUNTER_FALLING_MODE
 *                          - TIM_COUNTER_ANY_MODE
 * @param[in]  timCtrCfg    Pointer to configuration structure:
 *                          - TIM_TIMERCFG_Type for timer mode
 *                          - TIM_COUNTERCFG_Type for counter mode
 *
 * @note:
 * - The function enables the timer's power and sets the peripheral clock divider.
 * - It resets and initializes the prescaler and counters.
 * - It clears all interrupt flags in the IR register.
 * - The timer is left in a disabled state after initialization.
 */
void TIM_Init(LPC_TIM_TypeDef* TIMx, TIM_MODE mode, void* timCtrCfg);

/**
 * @brief      De-initializes the specified Timer/Counter peripheral.
 *
 * This function disables the timer, and removes power from
 * the selected timer peripheral. It should be called to safely
 * power down the timer and release its resources.
 *
 * @param[in]  TIMx  Pointer to the timer peripheral (LPC_TIMx [0...3]).
 *
 * @note:
 * - The function disables the timer.
 * - It disables the peripheral clock and powers down the timer.
 * - After calling this function, the timer must be re-initialized before use.
 */
void TIM_DeInit(LPC_TIM_TypeDef* TIMx);

/**
 * @brief      Initializes a timer or counter configuration structure with default values.
 *
 * This function sets default values for the provided configuration structure,
 * depending on the selected mode. For timer mode, it sets the prescale option
 * to microseconds and the prescale value to 0. For counter mode, it sets the
 * count input select to CAPn.0. Reserved fields are not initialized.
 *
 * @param[in]  mode         Timer/counter mode selection:
 *                          - TIM_TIMER_MODE
 *                          - TIM_COUNTER_RISING_MODE
 *                          - TIM_COUNTER_FALLING_MODE
 *                          - TIM_COUNTER_ANY_MODE
 * @param[out] timCtrCfg    Pointer to configuration structure to initialize:
 *                          - TIM_TIMERCFG_Type for timer mode
 *                          - TIM_COUNTERCFG_Type for counter mode
 *
 * @note       Call this function before configuring a timer or counter to ensure
 *             the structure has valid default values.
 */
void TIM_ConfigStructInit(TIM_MODE mode, void* timCtrCfg);

/**
 * @brief      Enables or disables the specified Timer/Counter peripheral.
 *
 * This function sets or clears the enable bit in the TCR register of the given timer,
 * effectively starting or stopping the timer/counter.
 *
 * @param[in]  TIMx      Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  newState  Functional state:
 *                       - ENABLE  : Start the timer/counter.
 *                       - DISABLE : Stop the timer/counter.
 *
 * @note:
 * - Use this function to control timer operation after configuration.
 * - The timer must be initialized before calling this function.
 */
void TIM_Cmd(LPC_TIM_TypeDef* TIMx, FunctionalState newState);

/**
 * @brief      Resets the Timer/Counter peripheral.
 *
 * This function synchronously resets the Timer Counter (TC) and Prescale Counter (PC)
 * of the specified timer by setting and then clearing the reset bit in the TCR register.
 *
 * @param[in]  TIMx  Pointer to the timer peripheral (LPC_TIMx [0...3]).
 *
 * @note:
 * - Use this function to reset the timer counters to zero.
 */
void TIM_ResetCounter(LPC_TIM_TypeDef* TIMx);

/**
 * @brief      Configures the match channel for the specified Timer/Counter peripheral.
 *
 * This function sets up the match value, interrupt, reset, stop, and external match output
 * for the selected match channel. It also clears the corresponding interrupt flag before
 * configuration to avoid spurious interrupts.
 *
 * @param[in]  TIMx         Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  matchCfg     Pointer to a TIM_MATCHCFG_Type structure.
 *
 * @note:
 * - The interrupt flag for the selected channel is cleared before configuration.
 * - The function updates MRx, MCR, and EMR registers according to the configuration.
 * - Call this function after initializing the timer to set up match behavior.
 */
void TIM_ConfigMatch(LPC_TIM_TypeDef* TIMx, const TIM_MATCHCFG_Type* matchCfg);

/**
 * @brief      Updates the match value for the specified Timer/Counter channel.
 *
 * This function sets the match register (MR0-MR3) of the given timer peripheral
 * to the provided value for the selected match channel. It does not modify any
 * match control or interrupt settings.
 *
 * @param[in]  TIMx         Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  channel      Match channel to update (TIM_MATCH_x [0..3]).
 * @param[in]  matchValue   New value to set in the match register.
 *
 * @note:
 * - Only the match value is updated; match behavior must be configured separately.
 * - Call this function to change the match value during runtime.
 */
void TIM_UpdateMatchValue(LPC_TIM_TypeDef* TIMx, TIM_MATCH_CHANNEL channel, uint32_t matchValue);

/**
 * @brief      Sets the external match output type for a specific match channel.
 *
 * This function configures the external match output behavior for the selected match channel
 * (MAT0...MAT3) of the specified Timer/Counter peripheral. It updates the EMR register to set
 * the output type for the given channel.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  channel  Match channel to configure (TIM_MATCH_x [0..3]).
 * @param[in]  type     External match output type:
 *                      - TIM_NOTHING
 *                      - TIM_LOW
 *                      - TIM_HIGH
 *                      - TIM_TOGGLE
 *
 * @note:
 * - Only the specified channel is affected.
 * - Call this function after initializing the timer and before starting it.
 */
void TIM_SetMatchExt(LPC_TIM_TypeDef* TIMx, TIM_MATCH_CHANNEL channel, TIM_EXTMATCH_OPT type);

/**
 * @brief      Configures the capture channel for the specified Timer/Counter peripheral.
 *
 * This function sets up the capture behavior for the selected channel, including
 * edge detection (rising, falling), interrupt generation, and channel selection.
 * It updates the CCR register according to the configuration structure.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  capCfg   Pointer to a TIM_CAPTURECFG_Type.
 *
 * @note:
 * - Only the specified channel is affected.
 * - Call this function after initializing the timer to set up capture behavior.
 */
void TIM_ConfigCapture(LPC_TIM_TypeDef* TIMx, const TIM_CAPTURECFG_Type* capCfg);

/**
 * @brief      Reads the value of the capture register for the specified channel.
 *
 * This function returns the value stored in the capture register (CR0 or CR1)
 * of the given timer peripheral, depending on the selected capture channel.
 *
 * @param[in]  TIMx     Pointer to the timer/counter peripheral (LPC_TIMx [0...3]).
 * @param[in]  channel  Capture channel to read:
 *                      - TIM_CAPTURE_0 : CAPn.0 input pin for TIMERn
 *                      - TIM_CAPTURE_1 : CAPn.1 input pin for TIMERn
 *
 * @return     Value of the selected capture register.
 *
 * @note:
 * - Use this function to obtain the timestamp captured on the specified input.
 * - The timer must be configured for capture mode before using this function.
 */
uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef* TIMx, TIM_CAPTURE_CHANNEL channel);

/**
 * @brief      Clears the specified Timer/Counter interrupt pending flag.
 *
 * This function clears the interrupt pending flag for the given match or capture
 * channel in the timer's interrupt register (IR). It can be used for both match
 * and capture interrupts.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  intFlag  Interrupt type to clear:
 *                      - TIM_MR0_INT: Match channel 0
 *                      - TIM_MR1_INT: Match channel 1
 *                      - TIM_MR2_INT: Match channel 2
 *                      - TIM_MR3_INT: Match channel 3
 *                      - TIM_CR0_INT: Capture channel 0
 *                      - TIM_CR1_INT: Capture channel 1
 */
void TIM_ClearIntPending(LPC_TIM_TypeDef* TIMx, TIM_INT intFlag);

/**
 * @brief      Gets the interrupt status for the specified Timer/Counter channel.
 *
 * This function checks if the interrupt flag for the given match or capture channel
 * is set in the timer's interrupt register (IR). It can be used for both match and
 * capture interrupts.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIMx [0...3]).
 * @param[in]  intFlag  Interrupt type to check:
 *                      - TIM_MR0_INT: Match channel 0
 *                      - TIM_MR1_INT: Match channel 1
 *                      - TIM_MR2_INT: Match channel 2
 *                      - TIM_MR3_INT: Match channel 3
 *                      - TIM_CR0_INT: Capture channel 0
 *                      - TIM_CR1_INT: Capture channel 1
 *
 * @return     FlagStatus
 *             - SET   : Interrupt is pending
 *             - RESET : No interrupt pending
 */
FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef* TIMx, TIM_INT intFlag);

/**
 * @brief      Configures the pin function for a specific timer/counter channel.
 *
 * This function sets the appropriate pin function for the selected timer/counter
 * capture or match channel, enabling its use for timer operations (capture or match).
 *
 * @param[in]  option  Timer pin selection option (see TIM_PIN_OPTION).
 *
 * @note:
 * - All resistor modes for timer pins are set to tristate (floating) by default.
 * - Call this function before using the timer channel for capture or external match operations.
 */
void TIM_PinConfig(TIM_PIN_OPTION option);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_TIMER_H_ */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
