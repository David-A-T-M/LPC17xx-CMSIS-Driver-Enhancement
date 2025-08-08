/***********************************************************************//**
 * @file        lpc17xx_adc.h
 * @brief        Contains all macro definitions and function prototypes
 *                 support for ADC firmware library on LPC17xx
 * @version        3.0
 * @date        18. June. 2010
 * @author        NXP MCU SW Application Team
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
/** @defgroup ADC ADC
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_ADC_H_
#define LPC17XX_ADC_H_

/* Includes ------------------------------------------------------------------- */
#include <LPC17xx.h>
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Private macros ------------------------------------------------------------- */
/** @defgroup ADC_Private_Macros ADC Private Macros
 * @{
 */

/* -------------------------- BIT DEFINITIONS ----------------------------------- */
/*********************************************************************//**
 * Macro defines for ADC  control register
 **********************************************************************/
/**  Selects which of the AD0.0:7 pins is (are) to be sampled and converted */
#define ADC_CR_CH_SEL(n)            ((1UL << n))
/**  The APB clock (PCLK) is divided by (this value plus one)
* to produce the clock for the A/D */
#define ADC_CR_CLKDIV(n)            ((n<<8))
/**  Repeated conversions A/D enable bit */
#define ADC_CR_BURST                ((1UL<<16))
/**  ADC convert in power down mode */
#define ADC_CR_PDN                  ((1UL<<21))
/**  Start mask bits */
#define ADC_CR_START_MASK           ((7UL<<24))
/**  Select Start Mode */
#define ADC_CR_START_MODE_SEL(SEL)  ((SEL<<24))
/**  Start conversion now */
#define ADC_CR_START_NOW            ((1UL<<24))
/**  Start conversion when the edge selected by bit 27 occurs on P2.10/EINT0 */
#define ADC_CR_START_EINT0          ((2UL<<24))
/** Start conversion when the edge selected by bit 27 occurs on P1.27/CAP0.1 */
#define ADC_CR_START_CAP01          ((3UL<<24))
/**  Start conversion when the edge selected by bit 27 occurs on MAT0.1 */
#define ADC_CR_START_MAT01          ((4UL<<24))
/**  Start conversion when the edge selected by bit 27 occurs on MAT0.3 */
#define ADC_CR_START_MAT03          ((5UL<<24))
/**  Start conversion when the edge selected by bit 27 occurs on MAT1.0 */
#define ADC_CR_START_MAT10          ((6UL<<24))
/**  Start conversion when the edge selected by bit 27 occurs on MAT1.1 */
#define ADC_CR_START_MAT11          ((7UL<<24))
/**  Start conversion on a falling edge on the selected CAP/MAT signal */
#define ADC_CR_EDGE                 ((1UL<<27))

/*********************************************************************//**
 * Macro defines for ADC Global Data register
 **********************************************************************/
/** When DONE is 1, this field contains result value of ADC conversion */
#define ADC_GDR_RESULT(n)           (((n>>4)&0xFFF))
/** These bits contain the channel from which the LS bits were converted */
#define ADC_GDR_CH(n)               (((n>>24)&0x7))
/** This bit is 1 in burst mode if the results of one or
 * more conversions was (were) lost */
#define ADC_GDR_OVERRUN_FLAG        ((1UL<<30))
/** This bit is set to 1 when an A/D conversion completes */
#define ADC_GDR_DONE_FLAG           ((1UL<<31))

/** This bits is used to mask for Channel */
#define ADC_GDR_CH_MASK             ((7UL<<24))
/*********************************************************************//**
 * Macro defines for ADC Interrupt register
 **********************************************************************/
/** These bits allow control over which A/D channels generate
 * interrupts for conversion completion */
#define ADC_INTEN_CH(n)             ((1UL<<n))
/** When 1, enables the global DONE flag in ADDR to generate an interrupt */
#define ADC_INTEN_GLOBAL            ((1UL<<8))

/*********************************************************************//**
 * Macro defines for ADC Data register
 **********************************************************************/
/** When DONE is 1, this field contains result value of ADC conversion */
#define ADC_DR_RESULT(n)            (((n>>4)&0xFFF))
/** These bits mirror the OVERRRUN status flags that appear in the
 * result register for each A/D channel */
#define ADC_DR_OVERRUN_FLAG         ((1UL<<30))
/** This bit is set to 1 when an A/D conversion completes. It is cleared
 * when this register is read */
#define ADC_DR_DONE_FLAG            ((1UL<<31))

/*********************************************************************//**
 * Macro defines for ADC Status register
**********************************************************************/
/** These bits mirror the DONE status flags that appear in the result
 * register for each A/D channel */
#define ADC_STAT_CH_DONE_FLAG(n)    ((n&0xFF))
/** These bits mirror the OVERRRUN status flags that appear in the
 * result register for each A/D channel */
#define ADC_STAT_CH_OVERRUN_FLAG(n) (((n>>8)&0xFF))
/** This bit is the A/D interrupt flag */
#define ADC_STAT_INT_FLAG           ((1UL<<16))

/*********************************************************************//**
 * Macro defines for ADC Trim register
**********************************************************************/
/** Offset trim bits for ADC operation */
#define ADC_ADCOFFS(n)              (((n&0xF)<<4))
/** Written to boot code*/
#define ADC_TRIM(n)                 (((n&0xF)<<8))

/* ------------------- CHECK PARAM DEFINITIONS ------------------------- */
/** Check ADC parameter */
#define PARAM_ADCx(n)    (((uint32_t *)n)==((uint32_t *)LPC_ADC))

/** Check ADC state parameter */
#define PARAM_ADC_START_ON_EDGE_OPT(OPT)    ((OPT == ADC_START_ON_RISING)||(OPT == ADC_START_ON_FALLING))

/** Check ADC state parameter */
#define PARAM_ADC_DATA_STATUS(OPT)    ((OPT== ADC_DATA_BURST)||(OPT== ADC_DATA_DONE))

/** Check ADC rate parameter */
#define PARAM_ADC_RATE(rate)    ((rate > 0) && (rate <= 200000))

/** Check ADC channel selection parameter */
#define PARAM_ADC_CHANNEL_SELECTION(SEL)    ( (SEL == ADC_CHANNEL_0) || (ADC_CHANNEL_1) \
                                           || (SEL == ADC_CHANNEL_2) || (ADC_CHANNEL_3) \
                                           || (SEL == ADC_CHANNEL_4) || (ADC_CHANNEL_5) \
                                           || (SEL == ADC_CHANNEL_6) || (ADC_CHANNEL_7) )

/** Check ADC start option parameter */
#define PARAM_ADC_START_OPT(OPT)    ( (OPT == ADC_START_CONTINUOUS) ||  (OPT == ADC_START_NOW) \
                                   || (OPT == ADC_START_ON_EINT0)   ||  (OPT == ADC_START_ON_CAP01) \
                                   || (OPT == ADC_START_ON_MAT01)   ||  (OPT == ADC_START_ON_MAT03) \
                                   || (OPT == ADC_START_ON_MAT10)   ||  (OPT == ADC_START_ON_MAT11) )

/** Check ADC interrupt type parameter */
#define PARAM_ADC_CHN_INT_OPT(OPT)    ( (OPT == ADC_ADINTEN0) || (OPT == ADC_ADINTEN1) \
                                      || (OPT == ADC_ADINTEN2) || (OPT == ADC_ADINTEN3) \
                                      || (OPT == ADC_ADINTEN4) || (OPT == ADC_ADINTEN5) \
                                      || (OPT == ADC_ADINTEN6) || (OPT == ADC_ADINTEN7) \
                                      || (OPT == ADC_ADGINTEN) )
/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup ADC_Public_Types ADC Public Types
 * @{
 */

/*********************************************************************//**
 * @brief ADC enumeration
 **********************************************************************/
/**
 * @brief ADC channel selection.
 */
typedef enum {
    ADC_CHANNEL_0 = 0,    /*!< Channel 0 */
    ADC_CHANNEL_1,        /*!< Channel 1 */
    ADC_CHANNEL_2,        /*!< Channel 2 */
    ADC_CHANNEL_3,        /*!< Channel 3 */
    ADC_CHANNEL_4,        /*!< Channel 4 */
    ADC_CHANNEL_5,        /*!< Channel 5 */
    ADC_CHANNEL_6,        /*!< Channel 6 */
    ADC_CHANNEL_7         /*!< Channel 7 */
} ADC_CHANNEL_SELECTION;

/**
 * @brief ADC start option.
 */
typedef enum {
    ADC_START_CONTINUOUS = 0, /*!< Continuous mode */
    ADC_START_NOW,            /*!< Start conversion now */
    ADC_START_ON_EINT0,       /*!< Start conversion on P2.10/EINT0 edge */
    ADC_START_ON_CAP01,       /*!< Start conversion on P1.27/CAP0.1 edge */
    ADC_START_ON_MAT01,       /*!< Start conversion on MAT0.1 edge */
    ADC_START_ON_MAT03,       /*!< Start conversion on MAT0.3 edge */
    ADC_START_ON_MAT10,       /*!< Start conversion on MAT1.0 edge */
    ADC_START_ON_MAT11        /*!< Start conversion on MAT1.1 edge */
} ADC_START_OPT;


/**
 * @brief ADC edge selection for start conversion.
 */
typedef enum {
    ADC_START_ON_RISING = 0, /*!< Start conversion on rising edge */
    ADC_START_ON_FALLING     /*!< Start conversion on falling edge */
} ADC_START_ON_EDGE_OPT;

/**
 * @brief ADC interrupt type.
 */
typedef enum {
    ADC_ADINTEN0 = 0,    /*!< Interrupt for channel 0 */
    ADC_ADINTEN1,        /*!< Interrupt for channel 1 */
    ADC_ADINTEN2,        /*!< Interrupt for channel 2 */
    ADC_ADINTEN3,        /*!< Interrupt for channel 3 */
    ADC_ADINTEN4,        /*!< Interrupt for channel 4 */
    ADC_ADINTEN5,        /*!< Interrupt for channel 5 */
    ADC_ADINTEN6,        /*!< Interrupt for channel 6 */
    ADC_ADINTEN7,        /*!< Interrupt for channel 7 */
    ADC_ADGINTEN         /*!< Global DONE flag interrupt */
} ADC_CHN_INT_OPT;

/**
 * @brief ADC data status.
 */
typedef enum {
    ADC_DATA_BURST = 0,  /*!< Burst mode status */
    ADC_DATA_DONE        /*!< Done status */
} ADC_DATA_STATUS;

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup ADC_Public_Functions ADC Public Functions
 * @{
 */
/* Init/DeInit ADC peripheral ----------------*/
/**
 * @brief      Initializes the ADC peripheral with the specified conversion rate.
 *
 * This function enables power and clock for the ADC, configures the conversion rate,
 * and powers up the ADC. Conversions do not start until explicitly triggered.
 *
 * @param[in]  rate  ADC conversion rate (Hz), must be less than or equal to 200 [kHz].
 *
 * @note:
 * - The function sets the ADC clock divider and powers up the ADC.
 * - The ADC must be initialized before use.
 */
void ADC_Init(uint32_t rate);

/**
 * @brief      De-initializes the ADC peripheral.
 *
 * This function powers down the ADC and disables its clock.
 * Call this to safely release ADC resources and save power.
 *
 * @note:
 * - The ADC is powered down and disabled.
 * - The ADC must be re-initialized before reuse.
 */
void ADC_DeInit(void);

/* Enable/Disable ADC functions --------------*/
/**
 * @brief      Enables or disables ADC burst mode.
 *
 * This function sets or clears the burst mode bit in the ADC control register.
 *
 * @param[in]  NewState  Functional state:
 *                       - ENABLE  : Enable burst mode.
 *                       - DISABLE : Disable burst mode.
 * @note:
 * - START bits must be 000 before enabling burst mode.
 * - Enabling burst mode starts conversions immediately.
 * - Disabling burst mode stops conversions.
 */
void ADC_BurstCmd(FunctionalState NewState);

/**
 * @brief      Powers up or powers down the ADC.
 *
 * This function sets or clears the power-down bit in the ADC control register.
 *
 * @param[in]  NewState  Functional state:
 *                       - ENABLE  : Power up ADC.
 *                       - DISABLE : Power down ADC.
 * @note:
 * - Disables the ADC power.
 * - Does not disable the ADC clock.
 * - Enable after enabling the ADC clock.
 * - Disable before disabling the ADC clock.
 */
void ADC_PowerdownCmd(FunctionalState NewState);

/**
 * @brief      Starts ADC conversion in the specified mode.
 *
 * This function sets the start mode bits in the ADC control register.
 *
 * @param[in]  startMode  ADC start mode selection:
 *                        - ADC_START_CONTINUOUS (burst mode)
 *                        - ADC_START_NOW (manual start)
 *                        - ADC_START_ON_EINT0
 *                        - ADC_START_ON_CAP01
 *                        - ADC_START_ON_MAT01
 *                        - ADC_START_ON_MAT03
 *                        - ADC_START_ON_MAT10
 *                        - ADC_START_ON_MAT11
 */
void ADC_StartCmd(ADC_START_OPT startMode);

/**
 * @brief      Enables or disables the specified ADC channel.
 *
 * This function sets or clears the channel select bit in the ADC control register.
 *
 * @param[in]  Channel    ADC channel to configure:
 *                        - ADC_CHANNEL_x [0...7]
 * @param[in]  NewState   Functional state:
 *                        - ENABLE  : Enable channel.
 *                        - DISABLE : Disable channel.
 */
void ADC_ChannelCmd(ADC_CHANNEL_SELECTION Channel, FunctionalState NewState);

/* Configure ADC functions -------------------*/
/**
 * @brief      Configures the edge for ADC start on external signal.
 *
 * This function sets the edge select bit in the ADC control register.
 *
 * @param[in]  EdgeOption  ADC edge selection:
 *                         - ADC_START_ON_RISING
 *                         - ADC_START_ON_FALLING
 * @note:
 * - This option is only effective when the start bits are set to [010...111]
 */
void ADC_EdgeStartConfig(ADC_START_ON_EDGE_OPT EdgeOption);

/**
 * @brief      Enables or disables ADC interrupt for the specified type.
 *
 * This function sets or clears the interrupt enable bit for the given channel or global interrupt.
 *
 * @param[in]  Channel   ADC interrupt type:
 *                       - ADC_ADINTENx [0...7]
                         - ADC_ADGINTEN
 * @param[in]  NewState  Functional state:
 *                       - ENABLE  : Enable interrupt.
 *                       - DISABLE : Disable interrupt.
 * @note:
 * - If ADC_ADGINTEN is selected, only the global DONE flag is enabled to generate an interrupt.
 * - ADC_ADGINTEN must be disabled for burst mode operation.
 */
void ADC_IntConfig(ADC_CHN_INT_OPT Channel, FunctionalState NewState);

/* Get ADC information functions -------------------*/
/**
 * @brief      Gets the global ADC status flag.
 *
 * This function checks the global ADC status register for DONE or OVERRUN flags.
 *
 * @param[in]  StatusType  ADC data status:
 *                         - ADC_DATA_BURST
 *                         - ADC_DATA_DONE
 *
 * @return     FlagStatus
 *             - SET   : Status flag is set.
 *             - RESET : Status flag is not set.
 */
FlagStatus ADC_GlobalGetStatus(ADC_DATA_STATUS StatusType);

/**
 * @brief      Gets the status flag for the specified ADC channel.
 *
 * This function checks the status register for DONE or OVERRUN flags for the given channel.
 *
 * @param[in]  channel     ADC channel to check:
 *                         - ADC_CHANNEL_x [0...7]
 * @param[in]  StatusType  ADC data status:
 *                         - ADC_DATA_BURST
 *                         - ADC_DATA_DONE
 *
 * @return     FlagStatus
 *             - SET   : Status flag is set.
 *             - RESET : Status flag is not set.
 */
FlagStatus ADC_ChannelGetStatus(ADC_CHANNEL_SELECTION channel, ADC_DATA_STATUS StatusType);

/**
 * @brief      Gets the global ADC conversion result.
 *
 * This function returns the result value from the global ADC data register.
 *
 * @return     12-bit ADC conversion result.
 * @note:
 * - The returned value is right-aligned to bits [11:0].
 */
uint32_t ADC_GlobalGetData(void);

/**
 * @brief      Gets the conversion result for the specified ADC channel.
 *
 * This function returns the result value from the data register for the given channel.
 *
 * @param[in]  channel  ADC channel to read:
 *                      - ADC_CHANNEL_x [0...7]
 *
 * @return     12-bit ADC conversion result for the channel.
 * @note:
 * - The returned value is right-aligned to bits [11:0].
 */
uint16_t ADC_ChannelGetData(ADC_CHANNEL_SELECTION channel);


/**
 * @}
 */


#ifdef __cplusplus
}
#endif


#endif /* LPC17XX_ADC_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
