/**
 * @file        lpc17xx_dac.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for DAC firmware library on LPC17xx
 * @version     3.0
 * @date        18. June. 2010
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
 * Date: 11/08/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup DAC DAC
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_DAC_H_
#define LPC17XX_DAC_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup DAC_Private_Macros DAC Private Macros
 * @{
 */

/** After the selected settling time after this field is written with a
new VALUE, the voltage on the AOUT pin (with respect to VSSA)
is VALUE/1024 × VREF */
#define DAC_VALUE(n)      ((uint32_t)((n & 0x3FF) << 6))
/** If this bit = 0: The settling time of the DAC is 1 microsecond max,
 * and the maximum current is 700 microAmpere
 * If this bit = 1: The settling time of the DAC is 2.5 microsecond
 * and the maximum current is 350 microAmpere */
#define DAC_BIAS_EN       ((uint32_t)(1 << 16))
/** Value to reload interrupt DMA counter */
#define DAC_CCNT_VALUE(n) ((uint32_t)(n & 0xffff))

/** DCAR double buffering */
#define DAC_DBLBUF_ENA   ((uint32_t)(1 << 1))
/** DCAR Time out count enable */
#define DAC_CNT_ENA      ((uint32_t)(1 << 2))
/** DCAR DMA access */
#define DAC_DMA_ENA      ((uint32_t)(1 << 3))
/** DCAR DACCTRL mask bit */
#define DAC_DACCTRL_MASK ((uint32_t)(0x0F))

/** Macro to determine if it is valid DAC peripheral */
#define PARAM_DACx(n) (((uint32_t*)n) == ((uint32_t*)LPC_DAC))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup DAC_Public_Types DAC Public Types
 * @{
 */

/**
 * @brief DAC current options for bias configuration.
 */
typedef enum {
    DAC_700uA = 0, /**< Settling time: 1 us max, max current: 700 uA */
    DAC_350uA      /**< Settling time: 2.5 us max, max current: 350 uA */
} DAC_MAX_CURRENT;
/** Check DAC max current option parameter. */
#define PARAM_DAC_MAX_CURRENT(MAX) ((MAX == DAC_700uA) || (MAX == DAC_350uA))

/**
 * @brief DAC converter control configuration.
 *
 * Used to configure double buffering, timeout counter, and DMA access for the DAC.
 */
typedef struct {
    FunctionalState doubleBufferEnable; /**< ENABLE or DISABLE. */
    FunctionalState counterEnable;      /**< ENABLE or DISABLE. */
    FunctionalState dmaEnable;          /**< ENABLE or DISABLE. */
} DAC_CONVERTER_CFG_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup DAC_Public_Functions DAC Public Functions
 * @{
 */
/**
 * @brief      Initializes the DAC peripheral and the DAC pin.
 *
 * This function configures the DAC pin, sets the peripheral clock divider,
 * and initializes the DAC with maximum current (700 uA) and output value 0.
 *
 * @note:
 * - The DAC pin is configured for analog output.
 * - The DAC is set to maximum current by default.
 * - Call this function before using other DAC functions.
 */
void DAC_Init(void);

/**
 * @brief      Updates the output value of the DAC.
 *
 * This function sets the 10-bit value to be converted to analog output
 * on the DAC pin. Only the value bits are updated; other bits remain unchanged.
 *
 * @param[in]  newValue  10-bit value to be converted (0-1023).
 *
 * @note:
 * - The output voltage is calculated as VALUE * (Vrefp - Vrefn) / 1024 + Vrefn.
 * - Call this function to change the DAC output.
 */
void DAC_UpdateValue(uint32_t newValue);

/**
 * @brief      Sets the bias (maximum current) for the DAC.
 *
 * This function configures the DAC bias to select the settling time and
 * maximum output current.
 *
 * @param[in]  maxCurr  DAC current option:
 *                      - DAC_700uA : 1 us settling, 700 uA max current.
 *                      - DAC_350uA : 2.5 us settling, 350 uA max current.
 *
 * @note:
 * - Use this function to optimize power or speed.
 * - Only the bias bit is affected.
 */
void DAC_SetBias(DAC_MAX_CURRENT maxCurr);

/**
 * @brief      Configures the DAC converter control features.
 *
 * This function enables or disables double buffering, timeout counter,
 * and DMA access for the DAC peripheral.
 *
 * @param[in]  dacCfg   Pointer to a DAC_CONVERTER_CFG_Type structure.
 *
 * @note:
 * - Only the specified features are affected.
 * - Call this function after DAC initialization.
 */
void DAC_ConfigDAConverterControl(const DAC_CONVERTER_CFG_Type* dacCfg);

/**
 * @brief      Sets the reload value for the DAC interrupt/DMA counter.
 *
 * This function sets the timeout value for the DAC DMA or interrupt counter.
 *
 * @param[in]  timeOut  Timeout value to reload (16-bit).
 *
 * @note:
 * - Use this function to configure DMA or interrupt timing.
 * - If timeOut is more than 16 bits, only the lower 16 bits are used.
 * - Only the counter value is updated.
 */
void DAC_SetDMATimeOut(uint32_t timeOut);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_DAC_H_ */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
