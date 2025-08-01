/***********************************************************************//**
 * @file		lpc17xx_timer.h
 * @brief		Contains all functions support for Timer firmware library on LPC17xx
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
 **********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup TIM TIM
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef __LPC17XX_TIMER_H_
#define __LPC17XX_TIMER_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Private Macros ------------------------------------------------------------- */
/** @defgroup TIM_Private_Macros TIM Private Macros
 * @{
 */

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/**********************************************************************
** Interrupt information
**********************************************************************/
/** Macro to clean interrupt pending */
#define TIM_IR_CLR(n) _BIT(n)

/**********************************************************************
** Timer interrupt register definitions
**********************************************************************/
/** Macro for getting a timer match interrupt bit */
#define TIM_MATCH_INT(n)		(_BIT(n & 0x0F))
/** Macro for getting a capture event interrupt bit */
#define TIM_CAP_INT(n)     (_BIT(((n & 0x0F) + 4)))

/**********************************************************************
* Timer control register definitions
**********************************************************************/
/** Timer/counter enable bit */
#define TIM_ENABLE			((uint32_t)(1<<0))
/** Timer/counter reset bit */
#define TIM_RESET			((uint32_t)(1<<1))
/** Timer control bit mask */
#define TIM_TCR_MASKBIT		((uint32_t)(3))

/**********************************************************************
* Timer match control register definitions
**********************************************************************/
/** Bit location for interrupt on MRx match, n = 0 to 3 */
#define TIM_INT_ON_MATCH(n)      	(_BIT((n * 3)))
/** Bit location for reset on MRx match, n = 0 to 3 */
#define TIM_RESET_ON_MATCH(n)    	(_BIT(((n * 3) + 1)))
/** Bit location for stop on MRx match, n = 0 to 3 */
#define TIM_STOP_ON_MATCH(n)     	(_BIT(((n * 3) + 2)))
/** Timer Match control bit mask */
#define TIM_MCR_MASKBIT			   ((uint32_t)(0x0FFF))
/** Timer Match control bit mask for specific channel*/
#define	TIM_MCR_CHANNEL_MASKBIT(n)		((uint32_t)(7<<(n*3)))

/**********************************************************************
* Timer capture control register definitions
**********************************************************************/
/** Bit location for CAP.n on CRx rising edge, n = 0 to 3 */
#define TIM_CAP_RISING(n)   	(_BIT((n * 3)))
/** Bit location for CAP.n on CRx falling edge, n = 0 to 3 */
#define TIM_CAP_FALLING(n)   	(_BIT(((n * 3) + 1)))
/** Bit location for CAP.n on CRx interrupt enable, n = 0 to 3 */
#define TIM_INT_ON_CAP(n)    	(_BIT(((n * 3) + 2)))
/** Mask bit for rising and falling edge bit */
#define TIM_EDGE_MASK(n)		(_SBF((n * 3), 0x03))
/** Timer capture control bit mask */
#define TIM_CCR_MASKBIT			((uint32_t)(0x3F))
/** Timer Capture control bit mask for specific channel*/
#define	TIM_CCR_CHANNEL_MASKBIT(n)		((uint32_t)(7<<(n*3)))

/**********************************************************************
* Timer external match register definitions
**********************************************************************/
/** Bit location for output state change of MAT.n when external match
   happens, n = 0 to 3 */
#define TIM_EM(n)    			_BIT(n)
/** Output state change of MAT.n when external match happens: no change */
#define TIM_EM_NOTHING    	((uint8_t)(0x0))
/** Output state change of MAT.n when external match happens: low */
#define TIM_EM_LOW         	((uint8_t)(0x1))
/** Output state change of MAT.n when external match happens: high */
#define TIM_EM_HIGH        	((uint8_t)(0x2))
/** Output state change of MAT.n when external match happens: toggle */
#define TIM_EM_TOGGLE      	((uint8_t)(0x3))
/** Macro for setting for the MAT.n change state bits */
#define TIM_EM_SET(n,s) 	(_SBF(((n << 1) + 4), (s & 0x03)))
/** Mask for the MAT.n change state bits */
#define TIM_EM_MASK(n) 		(_SBF(((n << 1) + 4), 0x03))
/** Timer external match bit mask */
#define TIM_EMR_MASKBIT	0x0FFF

/**********************************************************************
* Timer Count Control Register definitions
**********************************************************************/
/** Mask to get the Counter/timer mode bits */
#define TIM_CTCR_MODE_MASK  0x3
/** Mask to get the count input select bits */
#define TIM_CTCR_INPUT_MASK 0xC
/** Timer Count control bit mask */
#define TIM_CTCR_MASKBIT	0xF
#define TIM_COUNTER_MODE ((uint8_t)(1))


/* ---------------- CHECK PARAMETER DEFINITIONS ---------------------------- */
/** Macro to determine if it is valid TIMER peripheral */
#define PARAM_TIMx(n)	((((uint32_t *)n)==((uint32_t *)LPC_TIM0)) || (((uint32_t *)n)==((uint32_t *)LPC_TIM1)) \
|| (((uint32_t *)n)==((uint32_t *)LPC_TIM2)) || (((uint32_t *)n)==((uint32_t *)LPC_TIM3)))

/* Macro check interrupt type */
#define PARAM_TIM_INT_TYPE(TYPE)	((TYPE ==TIM_MR0_INT)||(TYPE ==TIM_MR1_INT)\
||(TYPE ==TIM_MR2_INT)||(TYPE ==TIM_MR3_INT)\
||(TYPE ==TIM_CR0_INT)||(TYPE ==TIM_CR1_INT))

/* Macro check TIMER mode */
#define PARAM_TIM_MODE_OPT(MODE)	((MODE == TIM_TIMER_MODE)||(MODE == TIM_COUNTER_RISING_MODE)\
|| (MODE == TIM_COUNTER_RISING_MODE)||(MODE == TIM_COUNTER_RISING_MODE))

/* Macro check TIMER prescale value */
#define PARAM_TIM_PRESCALE_OPT(OPT)	((OPT == TIM_PRESCALE_TICKVAL)||(OPT == TIM_PRESCALE_USVAL))

/* Macro check TIMER counter intput mode */
#define PARAM_TIM_COUNTER_INPUT_OPT(OPT)	((OPT == TIM_COUNTER_INCAP0)||(OPT == TIM_COUNTER_INCAP1))

/* Macro check TIMER external match mode */
#define PARAM_TIM_EXTMATCH_OPT(OPT)	((OPT == TIM_EXTMATCH_NOTHING)||(OPT == TIM_EXTMATCH_LOW)\
||(OPT == TIM_EXTMATCH_HIGH)||(OPT == TIM_EXTMATCH_TOGGLE))

/* Macro check TIMER external match mode */
#define PARAM_TIM_CAP_MODE_OPT(OPT)	((OPT == TIM_CAPTURE_NONE)||(OPT == TIM_CAPTURE_RISING) \
||(OPT == TIM_CAPTURE_FALLING)||(OPT == TIM_CAPTURE_ANY))

/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup TIM_Public_Types TIM Public Types
 * @{
 */

/***********************************************************************
 * Timer device enumeration
**********************************************************************/
/**
 * @brief interrupt type.
 */
typedef enum
{
	TIM_MR0_INT = 0,                /*!< interrupt for Match channel 0*/
	TIM_MR1_INT = 1,                /*!< interrupt for Match channel 1*/
	TIM_MR2_INT = 2,                /*!< interrupt for Match channel 2*/
	TIM_MR3_INT = 3,                /*!< interrupt for Match channel 3*/
	TIM_CR0_INT = 4,                /*!< interrupt for Capture channel 0*/
	TIM_CR1_INT = 5,                /*!< interrupt for Capture channel 1*/
}TIM_INT_TYPE;

/**
 * @brief Timer/counter operating mode.
 */
typedef enum
{
	TIM_TIMER_MODE = 0,				/*!< Timer mode */
	TIM_COUNTER_RISING_MODE,		/*!< Counter rising mode */
	TIM_COUNTER_FALLING_MODE,		/*!< Counter falling mode */
	TIM_COUNTER_ANY_MODE			/*!< Counter on both edges */
} TIM_MODE_OPT;

/**
 * @brief Timer/Counter prescale option.
 */
typedef enum
{
	TIM_PRESCALE_TICKVAL = 0,		/*!< Prescale in absolute value */
	TIM_PRESCALE_USVAL				/*!< Prescale in microsecond value */
} TIM_PRESCALE_OPT;

/**
 * @brief Counter input option.
 */
typedef enum
{
	TIM_COUNTER_INCAP0 = 0,			/*!< CAPn.0 input pin for TIMERn */
	TIM_COUNTER_INCAP1,				/*!< CAPn.1 input pin for TIMERn */
} TIM_COUNTER_INPUT_OPT;

/**
 * @brief Timer/Counter external match option.
 */
typedef enum
{
	TIM_EXTMATCH_NOTHING = 0,		/*!< Do nothing for external output pin if match */
	TIM_EXTMATCH_LOW,				/*!< Force external output pin to low if match */
	TIM_EXTMATCH_HIGH,				/*!< Force external output pin to high if match */
	TIM_EXTMATCH_TOGGLE				/*!< Toggle external output pin if match */
}TIM_EXTMATCH_OPT;

/**
 * @brief Timer/counter capture mode options.
 */
typedef enum {
	TIM_CAPTURE_NONE = 0,	/*!< No Capture */
	TIM_CAPTURE_RISING,		/*!< Rising capture mode */
	TIM_CAPTURE_FALLING,	/*!< Falling capture mode */
	TIM_CAPTURE_ANY			/*!< On both edges */
} TIM_CAP_MODE_OPT;

/**
 * @brief Timer configuration structure for TIMER mode.
 */
typedef struct {
    uint8_t PrescaleOption; /**< Should be:
                                    - TIM_PRESCALE_TICKVAL : Absolute value.
                                    - TIM_PRESCALE_USVAL   : Value in microseconds. */
    uint8_t Reserved[3];    /**< Reserved, not used. */
    uint32_t PrescaleValue; /**< Prescale value. */
} TIM_TIMERCFG_Type;

/**
 * @brief Timer configuration structure for COUNTER mode.
 */
typedef struct {
    uint8_t CountInputSelect;  /**< Should be:
                                    - TIM_COUNTER_INCAP0 : CAPn.0 input pin for TIMERn.
                                    - TIM_COUNTER_INCAP1 : CAPn.1 input pin for TIMERn. */
    uint8_t Reserved[3];       /**< Reserved, not used. */
} TIM_COUNTERCFG_Type;

/**
 * @brief Match channel configuration structure.
 */
typedef struct {
    uint8_t MatchChannel;         /**< Should be in range 0...3. */
    uint8_t IntOnMatch;           /**< Should be:
                                       - ENABLE  : Enable interrupt on match.
                                       - DISABLE : Disable interrupt on match. */
    uint8_t StopOnMatch;          /**< Should be:
                                       - ENABLE  : Stop timer on match.
                                       - DISABLE : Do not stop timer on match. */
    uint8_t ResetOnMatch;         /**< Should be:
                                       - ENABLE  : Reset timer on match.
                                       - DISABLE : Do not reset timer on match. */
    uint8_t ExtMatchOutputType;   /**< Should be:
                                       - TIM_EXTMATCH_NOTHING : Do nothing for external output pin if match.
                                       - TIM_EXTMATCH_LOW     : Force external output pin to low if match.
                                       - TIM_EXTMATCH_HIGH    : Force external output pin to high if match.
                                       - TIM_EXTMATCH_TOGGLE  : Toggle external output pin if match. */
    uint8_t Reserved[3];          /**< Reserved, not used. */
    uint32_t MatchValue;          /**< Match value to compare with timer counter. */
} TIM_MATCHCFG_Type;

/**
 * @brief Capture input configuration structure.
 */
typedef struct {
    uint8_t CaptureChannel;   /**< Should be in range 0...1. */
    uint8_t RisingEdge;       /**< Should be:
                                   - ENABLE  : Enable capture on rising edge.
                                   - DISABLE : Disable capture on rising edge. */
    uint8_t FallingEdge;      /**< Should be:
                                   - ENABLE  : Enable capture on falling edge.
                                   - DISABLE : Disable capture on falling edge. */
    uint8_t IntOnCaption;     /**< Should be:
                                   - ENABLE  : Enable interrupt on capture event.
                                   - DISABLE : Disable interrupt on capture event. */
} TIM_CAPTURECFG_Type;

/**
 * @}
 */


/* Public Functions ----------------------------------------------------------- */
/** @defgroup TIM_Public_Functions TIM Public Functions
 * @{
 */
/* Init/DeInit TIM functions -----------*/
/**
 * @brief      Initializes the specified Timer/Counter peripheral.
 *
 * This function enables the power and clock for the selected timer, configures
 * its mode (timer or counter), sets the prescaler or counter input as required,
 * resets the Timer Counter (TC) and Prescale Counter (PC), and clears all
 * pending interrupt flags. It prepares the timer for further configuration and use.
 *
 * @param[in]  TIMx              Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  TimerCounterMode  Timer/counter mode selection:
 *                               - TIM_TIMER_MODE
 *                               - TIM_COUNTER_RISING_MODE
 *                               - TIM_COUNTER_FALLING_MODE
 *                               - TIM_COUNTER_ANY_MODE
 * @param[in]  TIM_ConfigStruct  Pointer to configuration structure:
 *                               - TIM_TIMERCFG_Type for timer mode
 *                               - TIM_COUNTERCFG_Type for counter mode
 *
 * @remark:
 * - The function enables the timer's power and sets the peripheral clock divider.
 * - It resets and initializes the prescaler and counters.
 * - It clears all interrupt flags in the IR register.
 * - The timer is left in a disabled state after initialization.
 */
void TIM_Init(LPC_TIM_TypeDef *TIMx, TIM_MODE_OPT TimerCounterMode, void *TIM_ConfigStruct);

/**
 * @brief      De-initializes the specified Timer/Counter peripheral.
 *
 * This function disables the timer, and removes power from
 * the selected timer peripheral. It should be called to safely
 * power down the timer and release its resources.
 *
 * @param[in]  TIMx  Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 *
 * @note:
 * - The function disables the timer.
 * - It disables the peripheral clock and powers down the timer.
 * - After calling this function, the timer must be re-initialized before use.
 */
void TIM_DeInit(LPC_TIM_TypeDef *TIMx);

/* TIM interrupt functions -------------*/
/**
 * @brief      Clears the specified Timer/Counter interrupt pending flag.
 *
 * This function clears the interrupt pending flag for the given match or capture
 * channel in the timer's interrupt register (IR). It can be used for both match
 * and capture interrupts.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  IntFlag  Interrupt type to clear:
 *                      - TIM_MR0_INT: Match channel 0
 *                      - TIM_MR1_INT: Match channel 1
 *                      - TIM_MR2_INT: Match channel 2
 *                      - TIM_MR3_INT: Match channel 3
 *                      - TIM_CR0_INT: Capture channel 0
 *                      - TIM_CR1_INT: Capture channel 1
 */
void TIM_ClearIntPending(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE IntFlag);

/**
 * @brief      Gets the interrupt status for the specified Timer/Counter channel.
 *
 * This function checks if the interrupt flag for the given match or capture channel
 * is set in the timer's interrupt register (IR). It can be used for both match and
 * capture interrupts.
 *
 * @param[in]  TIMx     Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  IntFlag  Interrupt type to check:
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
FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE IntFlag);

/* TIM configuration functions --------*/
/**
 * @brief      Initializes a timer or counter configuration structure with default values.
 *
 * This function sets default values for the provided configuration structure,
 * depending on the selected mode. For timer mode, it sets the prescale option
 * to microseconds and the prescale value to 0. For counter mode, it sets the
 * count input select to CAPn.0. Reserved fields are not initialized.
 *
 * @param[in]  TimerCounterMode  Timer/counter mode selection:
 *                               - TIM_TIMER_MODE
 *                               - TIM_COUNTER_RISING_MODE
 *                               - TIM_COUNTER_FALLING_MODE
 *                               - TIM_COUNTER_ANY_MODE
 * @param[out] TIM_ConfigStruct  Pointer to configuration structure to initialize:
 *                               - TIM_TIMERCFG_Type for timer mode
 *                               - TIM_COUNTERCFG_Type for counter mode
 *
 * @note       Call this function before configuring a timer or counter to ensure
 *             the structure has valid default values.
 */
void TIM_ConfigStructInit(TIM_MODE_OPT TimerCounterMode, void *TIM_ConfigStruct);

/**
 * @brief      Configures the match channel for the specified Timer/Counter peripheral.
 *
 * This function sets up the match value, interrupt, reset, stop, and external match output
 * for the selected match channel. It also clears the corresponding interrupt flag before
 * configuration to avoid spurious interrupts.
 *
 * @param[in]  TIMx                 Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  TIM_MatchConfigStruct Pointer to a `TIM_MATCHCFG_Type` structure.
 *
 * @note:
 * - The interrupt flag for the selected channel is cleared before configuration.
 * - The function updates MRx, MCR, and EMR registers according to the configuration.
 * - Call this function after initializing the timer to set up match behavior.
 */
void TIM_ConfigMatch(LPC_TIM_TypeDef *TIMx, TIM_MATCHCFG_Type *TIM_MatchConfigStruct);

/**
 * @brief      Updates the match value for the specified Timer/Counter channel.
 *
 * This function sets the match register (MR0-MR3) of the given timer peripheral
 * to the provided value for the selected match channel. It does not modify any
 * match control or interrupt settings.
 *
 * @param[in]  TIMx         Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  MatchChannel Match channel to update (0...3).
 * @param[in]  MatchValue   New value to set in the match register.
 *
 * @note:
 * - Only the match value is updated; match behavior must be configured separately.
 * - Call this function to change the match value during runtime.
 */
void TIM_UpdateMatchValue(LPC_TIM_TypeDef *TIMx,uint8_t MatchChannel, uint32_t MatchValue);


/**
 * @brief      Sets the external match output type for a specific match channel.
 *
 * This function configures the external match output behavior for the selected match channel
 * (MAT0...MAT3) of the specified Timer/Counter peripheral. It updates the EMR register to set
 * the output type for the given channel.
 *
 * @param[in]  TIMx         Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  MatchChannel Match channel to configure (0...3).
 * @param[in]  ExtMatchOutputType   External match output type:
 *                                  - TIM_EXTMATCH_NOTHING
 *                                  - TIM_EXTMATCH_LOW
 *                                  - TIM_EXTMATCH_HIGH
 *                                  - TIM_EXTMATCH_TOGGLE
 *
 * @note:
 * - Only the specified channel is affected.
 * - Call this function after initializing the timer and before starting it.
 */
void TIM_SetMatchExt(LPC_TIM_TypeDef *TIMx, uint8_t MatchChannel, TIM_EXTMATCH_OPT ExtMatchOutputType);

/**
 * @brief      Configures the capture channel for the specified Timer/Counter peripheral.
 *
 * This function sets up the capture behavior for the selected channel, including
 * edge detection (rising, falling), interrupt generation, and channel selection.
 * It updates the CCR register according to the configuration structure.
 *
 * @param[in]  TIMx                   Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  TIM_CaptureConfigStruct Pointer to a `TIM_CAPTURECFG_Type`.
 *
 * @note:
 * - Only the specified channel is affected.
 * - Call this function after initializing the timer to set up capture behavior.
 */
void TIM_ConfigCapture(LPC_TIM_TypeDef *TIMx, TIM_CAPTURECFG_Type *TIM_CaptureConfigStruct);

/**
 * @brief      Enables or disables the specified Timer/Counter peripheral.
 *
 * This function sets or clears the enable bit in the TCR register of the given timer,
 * effectively starting or stopping the timer/counter.
 *
 * @param[in]  TIMx      Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 * @param[in]  NewState  Functional state:
 *                       - ENABLE  : Start the timer/counter.
 *                       - DISABLE : Stop the timer/counter.
 *
 * @note:
 * - Use this function to control timer operation after configuration.
 * - The timer must be initialized before calling this function.
 */
void TIM_Cmd(LPC_TIM_TypeDef *TIMx, FunctionalState NewState);

/**
 * @brief      Reads the value of the capture register for the specified channel.
 *
 * This function returns the value stored in the capture register (CR0 or CR1)
 * of the given timer peripheral, depending on the selected capture channel.
 *
 * @param[in]  TIMx           Pointer to the timer/counter peripheral (LPC_TIM0/1/2/3).
 * @param[in]  CaptureChannel Capture channel to read:
 *                            - TIM_COUNTER_INCAP0 : CAPn.0 input pin for TIMERn
 *                            - TIM_COUNTER_INCAP1 : CAPn.1 input pin for TIMERn
 *
 * @return     Value of the selected capture register.
 *
 * @note:
 * - Use this function to obtain the timestamp captured on the specified input.
 * - The timer must be configured for capture mode before using this function.
 */
uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef *TIMx, TIM_COUNTER_INPUT_OPT CaptureChannel);

/**
 * @brief      Resets the Timer/Counter peripheral.
 *
 * This function synchronously resets the Timer Counter (TC) and Prescale Counter (PC)
 * of the specified timer by setting and then clearing the reset bit in the TCR register.
 *
 * @param[in]  TIMx  Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 *
 * @note:
 * - Use this function to reset the timer counters to zero.
 */
void TIM_ResetCounter(LPC_TIM_TypeDef *TIMx);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __LPC17XX_TIMER_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
