/***********************************************************************//**
 * @file        lpc17xx_adc.c
 * @brief       Contains all functions support for ADC firmware library on LPC17xx
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
/** @addtogroup ADC
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_adc.h"
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


#ifdef _ADC

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup ADC_Public_Functions
 * @{
 */

void ADC_Init(uint32_t rate) {
    CHECK_PARAM(PARAM_ADC_RATE(rate));

    CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCAD, ENABLE);

    uint32_t adc_ctrl = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_ADC);
    adc_ctrl = (adc_ctrl / (rate * 65)) - 1;
    adc_ctrl = ADC_CR_CLKDIV(adc_ctrl & 0xFF);
    adc_ctrl |= ADC_CR_PDN;

    LPC_ADC->ADCR = adc_ctrl;
}

void ADC_DeInit(void) {
    LPC_ADC->ADCR &= ~ADC_CR_PDN;

    CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCAD, DISABLE);
}

void ADC_BurstCmd(FunctionalState newState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState) {
        LPC_ADC->ADCR |= ADC_CR_BURST;
        return;
    }
    LPC_ADC->ADCR &= ~ADC_CR_BURST;
}

void ADC_PowerdownCmd(FunctionalState newState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState) {
        LPC_ADC->ADCR |= ADC_CR_PDN;
        return;
    }
    LPC_ADC->ADCR &= ~ADC_CR_PDN;
}

void ADC_StartCmd(ADC_START_OPT startMode) {
    CHECK_PARAM(PARAM_ADC_START_OPT(startMode));

    LPC_ADC->ADCR &= ~ADC_CR_START_MASK;
    LPC_ADC->ADCR |= ADC_CR_START_MODE_SEL((uint32_t)startMode);
}

void ADC_ChannelCmd(ADC_CHANNEL_SELECTION channel, FunctionalState newState) {
    CHECK_PARAM(PARAM_ADC_CHANNEL_SELECTION(channel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState) {
        LPC_ADC->ADCR |= ADC_CR_CH_SEL(channel);
        return;
    }
    LPC_ADC->ADCR &= ~ADC_CR_CH_SEL(channel);
}

void ADC_EdgeStartConfig(ADC_START_ON_EDGE_OPT edgeOption) {
    CHECK_PARAM(PARAM_ADC_START_ON_EDGE_OPT(edgeOption));

    if (edgeOption) {
        LPC_ADC->ADCR |= ADC_CR_EDGE;
        return;
    }
    LPC_ADC->ADCR &= ~ADC_CR_EDGE;
}

void ADC_IntConfig(ADC_CHN_INT_OPT channel, FunctionalState newState) {
    CHECK_PARAM(PARAM_ADC_CHN_INT_OPT(channel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState) {
        LPC_ADC->ADINTEN |= ADC_INTEN_CH(channel);
        return;
    }
    LPC_ADC->ADINTEN &= ~ADC_INTEN_CH(channel);
}

FlagStatus ADC_GlobalGetStatus(ADC_DATA_STATUS statusType) {
    CHECK_PARAM(PARAM_ADC_DATA_STATUS(statusType));

    uint32_t temp = LPC_ADC->ADGDR;

    if (statusType)
        temp &= ADC_DR_DONE_FLAG;
    else
        temp &= ADC_DR_OVERRUN_FLAG;

    if (temp) return SET;

    return RESET;
}

FlagStatus ADC_ChannelGetStatus(ADC_CHANNEL_SELECTION channel, ADC_DATA_STATUS statusType) {
    CHECK_PARAM(PARAM_ADC_CHANNEL_SELECTION(channel));
    CHECK_PARAM(PARAM_ADC_DATA_STATUS(statusType));

    uint32_t temp = *(uint32_t*)((&LPC_ADC->ADDR0) + channel);

    if (statusType)
        temp &= ADC_DR_DONE_FLAG;
    else
        temp &= ADC_DR_OVERRUN_FLAG;

    if (temp) return SET;

    return RESET;
}

uint32_t ADC_GlobalGetData() {
    return ADC_GDR_RESULT(LPC_ADC->ADGDR);
}

uint16_t ADC_ChannelGetData(ADC_CHANNEL_SELECTION channel) {
    CHECK_PARAM(PARAM_ADC_CHANNEL_SELECTION(channel));

    const uint32_t adc_value = *(uint32_t*)((&LPC_ADC->ADDR0) + channel);
    return ADC_DR_RESULT(adc_value);
}
/**
 * @}
 */

#endif /* _ADC */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */

