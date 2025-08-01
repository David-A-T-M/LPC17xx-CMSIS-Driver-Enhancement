/***********************************************************************//**
 * @file        lpc17xx_timer.c
 * @brief       Contains all functions support for Timer firmware library on LPC17xx
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
    switch ((uint32_t)TIMx) {
        case (uint32_t)LPC_TIM0: return 0;
        case (uint32_t)LPC_TIM1: return 1;
        case (uint32_t)LPC_TIM2: return 2;
        case (uint32_t)LPC_TIM3: return 3;
        default:
    }
    return 0xFFFFFFFF;
}

/* End of Private Functions ---------------------------------------------------- */


/* Public Functions ----------------------------------------------------------- */
/** @addtogroup TIM_Public_Functions
 * @{
 */

void TIM_Init(LPC_TIM_TypeDef *TIMx, TIM_MODE_OPT TimerCounterMode, void *TIM_ConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_MODE_OPT(TimerCounterMode));

    switch ((uint32_t)TIMx) {
        case (uint32_t)LPC_TIM0:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uint32_t)LPC_TIM1:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER1, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uint32_t)LPC_TIM2:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER2, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        case (uint32_t)LPC_TIM3:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM3, ENABLE);
            CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER3, CLKPWR_PCLKSEL_CCLK_DIV_4);
            break;
        default:
    }

    TIMx->CTCR &= ~TIM_CTCR_MODE_MASK;
    TIMx->CTCR |= TimerCounterMode;

    TIMx->PR =0;
    TIMx->TCR |= (1<<1);
    TIMx->TCR &= ~(1<<1);
    if (TimerCounterMode == TIM_TIMER_MODE) {
        TIM_TIMERCFG_Type* pTimeCfg = (TIM_TIMERCFG_Type*)TIM_ConfigStruct;
        if (pTimeCfg->PrescaleOption  == TIM_PRESCALE_TICKVAL) {
            TIMx->PR = pTimeCfg->PrescaleValue - 1;
        }
        else {
            TIMx->PR = converUSecToVal(converPtrToTimeNum(TIMx),pTimeCfg->PrescaleValue) - 1;
        }
    }
    else {
        TIM_COUNTERCFG_Type* pCounterCfg = (TIM_COUNTERCFG_Type*)TIM_ConfigStruct;
        TIMx->CTCR  &= ~TIM_CTCR_INPUT_MASK;
        if (pCounterCfg->CountInputSelect == TIM_COUNTER_INCAP1)
            TIMx->CTCR |= _BIT(2);
    }

    TIMx->IR = 0x3F;
}

void TIM_DeInit(LPC_TIM_TypeDef *TIMx) {
    CHECK_PARAM(PARAM_TIMx(TIMx));

    TIMx->TCR = 0x00;

    switch ((uint32_t)TIMx) {
        case (uint32_t)LPC_TIM0:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, DISABLE);
            break;
        case (uint32_t)LPC_TIM1:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM1, DISABLE);
            break;
        case (uint32_t)LPC_TIM2:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM2, DISABLE);
            break;
        case (uint32_t)LPC_TIM3:
            CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM3, DISABLE);
            break;
        default:
    }
}

void TIM_ClearIntPending(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE IntFlag) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_INT_TYPE(IntFlag));

    TIMx->IR |= TIM_IR_CLR(IntFlag);
}

FlagStatus TIM_GetIntStatus(LPC_TIM_TypeDef *TIMx, TIM_INT_TYPE IntFlag) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_INT_TYPE(IntFlag));

    return ((TIMx->IR & TIM_IR_CLR(IntFlag)) ? SET : RESET);
}

void TIM_ConfigStructInit(TIM_MODE_OPT TimerCounterMode, void *TIM_ConfigStruct) {
    if (TimerCounterMode == TIM_TIMER_MODE) {
        TIM_TIMERCFG_Type * pTimeCfg = (TIM_TIMERCFG_Type *)TIM_ConfigStruct;
        pTimeCfg->PrescaleOption = TIM_PRESCALE_USVAL;
        pTimeCfg->PrescaleValue = 0;
    }
    else {
        TIM_COUNTERCFG_Type * pCounterCfg = (TIM_COUNTERCFG_Type *)TIM_ConfigStruct;
        pCounterCfg->CountInputSelect = TIM_COUNTER_INCAP0;
    }
}

void TIM_ConfigMatch(LPC_TIM_TypeDef *TIMx, TIM_MATCHCFG_Type *TIM_MatchConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_EXTMATCH_OPT(TIM_MatchConfigStruct->ExtMatchOutputType));

    TIMx->IR |= TIM_IR_CLR(TIM_MatchConfigStruct->MatchChannel);

    switch (TIM_MatchConfigStruct->MatchChannel) {
        case 0:
            TIMx->MR0 = TIM_MatchConfigStruct->MatchValue;
            break;
        case 1:
            TIMx->MR1 = TIM_MatchConfigStruct->MatchValue;
            break;
        case 2:
            TIMx->MR2 = TIM_MatchConfigStruct->MatchValue;
            break;
        case 3:
            TIMx->MR3 = TIM_MatchConfigStruct->MatchValue;
            break;
        default:
            return;
    }

    TIMx->MCR &= ~TIM_MCR_CHANNEL_MASKBIT(TIM_MatchConfigStruct->MatchChannel);

    if (TIM_MatchConfigStruct->IntOnMatch)
        TIMx->MCR |= TIM_INT_ON_MATCH(TIM_MatchConfigStruct->MatchChannel);

    if (TIM_MatchConfigStruct->ResetOnMatch)
        TIMx->MCR |= TIM_RESET_ON_MATCH(TIM_MatchConfigStruct->MatchChannel);

    if (TIM_MatchConfigStruct->StopOnMatch)
        TIMx->MCR |= TIM_STOP_ON_MATCH(TIM_MatchConfigStruct->MatchChannel);

    TIMx->EMR &= ~TIM_EM_MASK(TIM_MatchConfigStruct->MatchChannel);
    TIMx->EMR |= TIM_EM_SET(TIM_MatchConfigStruct->MatchChannel,TIM_MatchConfigStruct->ExtMatchOutputType);
}

void TIM_UpdateMatchValue(LPC_TIM_TypeDef *TIMx,uint8_t MatchChannel, uint32_t MatchValue) {
    CHECK_PARAM(PARAM_TIMx(TIMx));

    switch(MatchChannel) {
        case 0: TIMx->MR0 = MatchValue; break;
        case 1: TIMx->MR1 = MatchValue; break;
        case 2: TIMx->MR2 = MatchValue; break;
        case 3: TIMx->MR3 = MatchValue; break;
        default:
    }
}

void TIM_SetMatchExt(LPC_TIM_TypeDef *TIMx, uint8_t MatchChannel, TIM_EXTMATCH_OPT ExtMatchOutputType) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_EXTMATCH_OPT(ExtMatchOutputType));

    if (MatchChannel > 3) return;

    TIMx->EMR &= ~TIM_EM_MASK(MatchChannel);
    TIMx->EMR |= TIM_EM_SET(MatchChannel, ExtMatchOutputType);
}

void TIM_ConfigCapture(LPC_TIM_TypeDef *TIMx, TIM_CAPTURECFG_Type *TIM_CaptureConfigStruct) {
    CHECK_PARAM(PARAM_TIMx(TIMx));

    TIMx->CCR &= ~TIM_CCR_CHANNEL_MASKBIT(TIM_CaptureConfigStruct->CaptureChannel);

    if (TIM_CaptureConfigStruct->RisingEdge)
        TIMx->CCR |= TIM_CAP_RISING(TIM_CaptureConfigStruct->CaptureChannel);

    if (TIM_CaptureConfigStruct->FallingEdge)
        TIMx->CCR |= TIM_CAP_FALLING(TIM_CaptureConfigStruct->CaptureChannel);

    if (TIM_CaptureConfigStruct->IntOnCaption)
        TIMx->CCR |= TIM_INT_ON_CAP(TIM_CaptureConfigStruct->CaptureChannel);
}

void TIM_Cmd(LPC_TIM_TypeDef *TIMx, FunctionalState NewState) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

    if (NewState == ENABLE) {
        TIMx->TCR |= TIM_ENABLE;
    }
    else {
        TIMx->TCR &= ~TIM_ENABLE;
    }
}

uint32_t TIM_GetCaptureValue(LPC_TIM_TypeDef *TIMx, TIM_COUNTER_INPUT_OPT CaptureChannel) {
    CHECK_PARAM(PARAM_TIMx(TIMx));
    CHECK_PARAM(PARAM_TIM_COUNTER_INPUT_OPT(CaptureChannel));

    return (CaptureChannel == 0)? TIMx->CR0 : TIMx->CR1;
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
