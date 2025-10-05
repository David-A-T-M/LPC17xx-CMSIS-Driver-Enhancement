/**
 * @file        lpc17xx_dac.c
 * @brief       Contains all functions support for DAC firmware library on LPC17xx
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
 * Date: 11/08/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @addtogroup DAC
 * @{
 */

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx_dac.h"
#include "lpc17xx_clkpwr.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _DAC

/* ---------------------------- Public Functions ---------------------------- */
/** @addtogroup DAC_Public_Functions
 * @{
 */

void DAC_Init(void) {
    LPC_PINCON->PINSEL1 &= ~(0x3 << 20);
    LPC_PINCON->PINSEL1 |= (0x1 << 21);

    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_DAC, CLKPWR_PCLKSEL_CCLK_DIV_4);

    DAC_SetBias(DAC_700uA);
}

void DAC_UpdateValue(uint32_t newValue) {
    LPC_DAC->DACR &= ~DAC_VALUE(0x3FF);
    LPC_DAC->DACR |= DAC_VALUE(newValue);
}

void DAC_SetBias(DAC_MAX_CURRENT maxCurr) {
    CHECK_PARAM(PARAM_DAC_MAX_CURRENT(maxCurr));

    LPC_DAC->DACR &= ~DAC_BIAS_EN;

    if (maxCurr == DAC_350uA) {
        LPC_DAC->DACR |= DAC_BIAS_EN;
    }
}

void DAC_ConfigDAConverterControl(const DAC_CONVERTER_CFG_Type* dacCfg) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(dacCfg->doubleBufferEnable));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(dacCfg->counterEnable));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(dacCfg->dmaEnable));

    LPC_DAC->DACCTRL &= ~DAC_DACCTRL_MASK;

    if (dacCfg->doubleBufferEnable) {
        LPC_DAC->DACCTRL |= DAC_DBLBUF_ENA;
    }
    if (dacCfg->counterEnable) {
        LPC_DAC->DACCTRL |= DAC_CNT_ENA;
    }
    if (dacCfg->dmaEnable) {
        LPC_DAC->DACCTRL |= DAC_DMA_ENA;
    }
}

void DAC_SetDMATimeOut(uint32_t timeOut) {
    LPC_DAC->DACCNTVAL = DAC_CCNT_VALUE(timeOut);
}

/**
 * @}
 */

#endif /* _DAC */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
