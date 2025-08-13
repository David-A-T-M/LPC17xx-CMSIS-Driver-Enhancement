/**
 * @file        lpc17xx_timer.c
 * @brief       Contains all functions support for Timer firmware library on LPC17xx
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
 */

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup TIM
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_pinsel.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _TIM

/* Private Functions ---------------------------------------------------------- */
static uint32_t getPClock (uint32_t timerNum);
static uint32_t converUSecToVal (uint32_t timerNum, uint32_t uSec);
static uint32_t converPtrToTimeNum (LPC_TIM_TypeDef *TIMx);

/**
 * @brief Returns the peripheral clock frequency for the specified timer.
 *
 * @param[in] timerNum Timer index (0 to 3).
 *
 * @return Peripheral clock frequency in Hz, or 0 if the timer number is invalid.
 */
static uint32_t getPClock(uint32_t timerNum) {
    static const uint32_t clk_selectors[] = {
        CLKPWR_PCLKSEL_TIMER0,
        CLKPWR_PCLKSEL_TIMER1,
        CLKPWR_PCLKSEL_TIMER2,
        CLKPWR_PCLKSEL_TIMER3
    };
    if (timerNum > 3) return 0;
    return CLKPWR_GetPCLK(clk_selectors[timerNum]);
}

/**
 * @brief Converts a time in microseconds to timer ticks for the specified timer.
 *
 * @param[in] timerNum Timer index (0 to 3).
 * @param[in] uSec Time duration in microseconds.
 * @return Number of timer ticks required for the given time, or 0 if input is invalid.
 */
uint32_t converUSecToVal(uint32_t timerNum, uint32_t uSec) {
    uint64_t pclk = getPClock(timerNum);
    if (uSec == 0) return 0;
    return (uint32_t)(pclk * uSec / 1000000);
}

/**
 * @brief      Converts a timer register pointer to its timer number.
 *
 * This function maps the given LPC_TIM_TypeDef pointer to its corresponding
 * timer number representation.
 * If the pointer does not match any known timer, it returns 0xFFFFFFFF.
 *
 * @param[in]  TIMx  Pointer to the timer peripheral (LPC_TIM0/1/2/3).
 *
 * @return     Timer number (0 to 3), or 0xFFFFFFFF if the pointer is invalid.
 *
 * @note
 * - Use this function to identify the timer index from its register pointer.
 * - The return value 0xFFFFFFFF indicates an invalid or unknown timer pointer.
 */
static uint32_t converPtrToTimeNum(LPC_TIM_TypeDef *TIMx) {
    switch ((uintptr_t)TIMx) {
        case (uintptr_t)LPC_TIM0: return 0;
        case (uintptr_t)LPC_TIM1: return 1;
        case (uintptr_t)LPC_TIM2: return 2;
        case (uintptr_t)LPC_TIM3: return 3;
        default: break;
    }
    return 0xFFFFFFFF;
}

/* End of Private Functions ---------------------------------------------------- */


/* Public Functions ----------------------------------------------------------- */
/** @addtogroup TIM_Public_Functions
 * @{
 */

void TIM_Init(LPC_TIM_TypeDef *TIMx, TIM_MODE_OPT timerCounterMode, void *TIM_ConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_MODE_OPT(timerCounterMode));

    switch ((uintptr_t)TIMx) {
        case (uintptr_t)LPC_TIM0:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uintptr_t)LPC_TIM1:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER1, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uintptr_t)LPC_TIM2:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER2, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uintptr_t)LPC_TIM3:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM3, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER3, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        default:
            break;
    }

    TIMx->CTCR &= ~TIM_CTCR_MODE_MASK;
    TIMx->CTCR |= timerCounterMode;

    TIMx->PR =0;
    TIMx->TCR |= (1<<1);
    TIMx->TCR &= ~(1<<1);
    if (timerCounterMode == TIM_TIMER_MODE) {
        TIM_TIMERCFG_Type* pTimeCfg = (TIM_TIMERCFG_Type*)TIM_ConfigStruct;
        if (pTimeCfg->prescaleOption == TIM_TICKVAL)
            TIMx->PR = pTimeCfg->prescaleValue - 1;
        else
            TIMx->PR = converUSecToVal(converPtrToTimeNum(TIMx),pTimeCfg->prescaleValue) - 1;
    }
    else {
        TIM_COUNTERCFG_Type* pCounterCfg = (TIM_COUNTERCFG_Type*)TIM_ConfigStruct;
        TIMx->CTCR &= ~TIM_CTCR_INPUT_MASK;
        if (pCounterCfg->countInputSelect == TIM_CAPTURE_CHANNEL_1)
            TIMx->CTCR |= _BIT(2);
    }

    TIMx->IR = 0x3F;
}

void TIM_DeInit(LPC_TIM_TypeDef *TIMx) {
    CHECK_PARAM(PARAM_TIMx(TIMx));

    TIMx->TCR = 0x00;

    switch ((uintptr_t)TIMx) {
        case (uintptr_t)LPC_TIM0:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, DISABLE);
            break;
        case (uintptr_t)LPC_TIM1:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1, DISABLE);
            break;
        case (uintptr_t)LPC_TIM2:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, DISABLE);
            break;
        case (uintptr_t)LPC_TIM3:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM3, DISABLE);
            break;
        default:
            break;
    }
}

void TIM_ClearIntPending(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE intFlag) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_INT_TYPE(intFlag));

    TIMx->IR |= TIM_IR_CLR(intFlag);
}

FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE intFlag) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_INT_TYPE(intFlag));

    return ((TIMx->IR & TIM_IR_CLR(intFlag)) ? SET : RESET);
}

void TIM_ConfigStructInit(TIM_MODE_OPT timerCounterMode, void *TIM_ConfigStruct) {
    CHECK_PARAM(PARAM_TIM_MODE_OPT(timerCounterMode));

    if (timerCounterMode == TIM_TIMER_MODE) {
        TIM_TIMERCFG_Type * pTimeCfg = (TIM_TIMERCFG_Type *)TIM_ConfigStruct;
        pTimeCfg->prescaleOption = TIM_USVAL;
        pTimeCfg->prescaleValue = 1;
    }
    else {
        TIM_COUNTERCFG_Type * pCounterCfg = (TIM_COUNTERCFG_Type *)TIM_ConfigStruct;
        pCounterCfg->countInputSelect = TIM_CAPTURE_CHANNEL_0;
    }
}

void TIM_ConfigMatch(LPC_TIM_TypeDef *TIMx, TIM_MATCHCFG_Type *TIM_MatchConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_MATCH_CHANNEL(TIM_MatchConfigStruct->matchChannel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_MatchConfigStruct->intOnMatch));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_MatchConfigStruct->stopOnMatch));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_MatchConfigStruct->resetOnMatch));
    CHECK_PARAM(PARAM_TIM_EXTMATCH_OPT(TIM_MatchConfigStruct->extMatchOutputType));

    TIMx->IR |= TIM_IR_CLR(TIM_MatchConfigStruct->matchChannel);

    switch (TIM_MatchConfigStruct->matchChannel) {
        case 0:
            TIMx->MR0 = TIM_MatchConfigStruct->matchValue;
            break;
        case 1:
            TIMx->MR1 = TIM_MatchConfigStruct->matchValue;
            break;
        case 2:
            TIMx->MR2 = TIM_MatchConfigStruct->matchValue;
            break;
        case 3:
            TIMx->MR3 = TIM_MatchConfigStruct->matchValue;
            break;
        default:
            return;
    }

    TIMx->MCR &= ~TIM_MCR_CHANNEL_MASKBIT(TIM_MatchConfigStruct->matchChannel);

    if (TIM_MatchConfigStruct->intOnMatch)
        TIMx->MCR |= TIM_INT_ON_MATCH(TIM_MatchConfigStruct->matchChannel);

    if (TIM_MatchConfigStruct->resetOnMatch)
        TIMx->MCR |= TIM_RESET_ON_MATCH(TIM_MatchConfigStruct->matchChannel);

    if (TIM_MatchConfigStruct->stopOnMatch)
        TIMx->MCR |= TIM_STOP_ON_MATCH(TIM_MatchConfigStruct->matchChannel);

    TIMx->EMR &= ~TIM_EMR_MASK(TIM_MatchConfigStruct->matchChannel);
    TIMx->EMR |= TIM_EM_SET(TIM_MatchConfigStruct->matchChannel,TIM_MatchConfigStruct->extMatchOutputType);
}

void TIM_UpdateMatchValue(LPC_TIM_TypeDef *TIMx, TIM_MATCH_CHANNEL_OPT matchChannel, uint32_t matchValue) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_MATCH_CHANNEL(matchChannel));

    switch(matchChannel) {
        case 0: TIMx->MR0 = matchValue; break;
        case 1: TIMx->MR1 = matchValue; break;
        case 2: TIMx->MR2 = matchValue; break;
        case 3: TIMx->MR3 = matchValue; break;
        default: break;
    }
}

void TIM_SetMatchExt(LPC_TIM_TypeDef *TIMx, TIM_MATCH_CHANNEL_OPT matchChannel, TIM_EXTMATCH_OPT extMatchOutputType) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_MATCH_CHANNEL(matchChannel));
    CHECK_PARAM(PARAM_TIM_EXTMATCH_OPT(extMatchOutputType));

    TIMx->EMR &= ~TIM_EMR_MASK(matchChannel);
    TIMx->EMR |= TIM_EM_SET(matchChannel, extMatchOutputType);
}

void TIM_ConfigCapture(LPC_TIM_TypeDef *TIMx, TIM_CAPTURECFG_Type *TIM_CaptureConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_CAPTURE_CHANNEL(TIM_CaptureConfigStruct->captureChannel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_CaptureConfigStruct->risingEdge));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_CaptureConfigStruct->fallingEdge));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(TIM_CaptureConfigStruct->intOnCapture));

    TIMx->CCR &= ~TIM_CCR_CHANNEL_MASKBIT(TIM_CaptureConfigStruct->captureChannel);

    if (TIM_CaptureConfigStruct->risingEdge)
        TIMx->CCR |= TIM_CAP_RISING(TIM_CaptureConfigStruct->captureChannel);

    if (TIM_CaptureConfigStruct->fallingEdge)
        TIMx->CCR |= TIM_CAP_FALLING(TIM_CaptureConfigStruct->captureChannel);

    if (TIM_CaptureConfigStruct->intOnCapture)
        TIMx->CCR |= TIM_INT_ON_CAP(TIM_CaptureConfigStruct->captureChannel);
}

void TIM_Cmd(LPC_TIM_TypeDef *TIMx, FunctionalState newState) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE)
        TIMx->TCR |= TIM_ENABLE;
    else
        TIMx->TCR &= ~TIM_ENABLE;
}

uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef *TIMx, TIM_CAPTURE_CHANNEL_OPT captureChannel) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_CAPTURE_CHANNEL(captureChannel));

    return (captureChannel == 0)? TIMx->CR0 : TIMx->CR1;
}

void TIM_ResetCounter(LPC_TIM_TypeDef *TIMx) {
    CHECK_PARAM(PARAM_TIMx(TIMx));

    TIMx->TCR |= TIM_RESET;
    TIMx->TCR &= ~TIM_RESET;
}

/**
 * @}
 */

#endif /* _TIM */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
