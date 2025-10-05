/**
 * @file        lpc17xx_pwm.c
 * @brief       Contains all functions support for PWM firmware library on LPC17xx
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
/** @addtogroup PWM
 * @{
 */

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx_pwm.h"
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

#ifdef _PWM

static uint32_t converUSecToVal(uint32_t uSec);

/**
 * @brief Converts a time in microseconds to timer ticks for the specified timer.
 *
 * @param[in] uSec Time duration in microseconds.
 * @return Number of timer ticks required for the given time, or 0 if input is invalid.
 */
static uint32_t converUSecToVal(uint32_t uSec) {
    const uint64_t pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_PWM1);
    if (uSec == 0)
        return 0;
    return (uint32_t)(pclk * uSec / 1000000);
}

/* ---------------------------- Public Functions ---------------------------- */
/** @addtogroup PWM_Public_Functions
 * @{
 */

void PWM_Init(PWM_TIM_MODE mode, void* pwmCfg) {
    CHECK_PARAM(PARAM_PWM_TIM_MODE(mode));

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1, ENABLE);
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_PWM1, CLKPWR_PCLKSEL_CCLK_DIV_4);

    LPC_PWM1->IR  = PWM_IR_BITMASK;
    LPC_PWM1->TCR = 0x00;
    LPC_PWM1->MCR = 0x00;
    LPC_PWM1->CCR = 0x00;
    LPC_PWM1->PCR = 0x00;
    LPC_PWM1->LER = 0x00;

    LPC_PWM1->CTCR = 0x00;
    LPC_PWM1->CTCR |= mode;

    if (mode == PWM_MODE_TIMER) {
        const PWM_TIMERCFG_Type* pTimeCfg = (PWM_TIMERCFG_Type*)pwmCfg;
        CHECK_PARAM(PARAM_PWM_PRESCALE(pTimeCfg->prescaleOption));

        if (pTimeCfg->prescaleOption == PWM_TICKVAL) {
            LPC_PWM1->PR = pTimeCfg->prescaleValue - 1;
        } else {
            LPC_PWM1->PR = converUSecToVal(pTimeCfg->prescaleValue) - 1;
        }
    } else {
        const PWM_COUNTERCFG_Type* pCounterCfg = (PWM_COUNTERCFG_Type*)pwmCfg;
        CHECK_PARAM(PARAM_PWM_CAPTURE(pCounterCfg->countInputSelect));

        LPC_PWM1->CTCR |= (PWM_CTCR_SELECT_INPUT((uint32_t)pCounterCfg->countInputSelect));
    }
}

void PWM_DeInit(void) {
    LPC_PWM1->TCR = 0x00;
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCPWM1, DISABLE);
}

void PWM_ConfigStructInit(PWM_TIM_MODE mode, void* pwmCfg) {
    CHECK_PARAM(PARAM_PWM_TIM_MODE(mode));

    if (mode == PWM_MODE_TIMER) {
        PWM_TIMERCFG_Type* pTimeCfg = (PWM_TIMERCFG_Type*)pwmCfg;
        pTimeCfg->prescaleOption    = PWM_USVAL;
        pTimeCfg->prescaleValue     = 1;
    } else {
        PWM_COUNTERCFG_Type* pCounterCfg = (PWM_COUNTERCFG_Type*)pwmCfg;
        pCounterCfg->countInputSelect    = PWM_CAPTURE_0;
    }
}

void PWM_PinConfig(PWM_PIN_OPTION option) {
    CHECK_PARAM(PARAM_PWM_PIN_OPTION(option));

    static const PINSEL_CFG_Type PinCfg[14] = {{PINSEL_PORT_1, PINSEL_PIN_18, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_0, PINSEL_FUNC_1, PINSEL_TRISTATE},
                                               {PINSEL_PORT_1, PINSEL_PIN_20, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_1, PINSEL_FUNC_1, PINSEL_TRISTATE},
                                               {PINSEL_PORT_3, PINSEL_PIN_25, PINSEL_FUNC_3, PINSEL_TRISTATE},
                                               {PINSEL_PORT_1, PINSEL_PIN_21, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_2, PINSEL_FUNC_1, PINSEL_TRISTATE},
                                               {PINSEL_PORT_3, PINSEL_PIN_26, PINSEL_FUNC_3, PINSEL_TRISTATE},
                                               {PINSEL_PORT_1, PINSEL_PIN_23, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_3, PINSEL_FUNC_1, PINSEL_TRISTATE},
                                               {PINSEL_PORT_1, PINSEL_PIN_24, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_4, PINSEL_FUNC_1, PINSEL_TRISTATE},
                                               {PINSEL_PORT_1, PINSEL_PIN_26, PINSEL_FUNC_2, PINSEL_TRISTATE},
                                               {PINSEL_PORT_2, PINSEL_PIN_5, PINSEL_FUNC_1, PINSEL_TRISTATE}};

    PINSEL_ConfigPin(&PinCfg[option]);
}

void PWM_ChannelConfig(PWM_CHANNEL channel, PWM_CHANNEL_EDGE edgeMode) {
    CHECK_PARAM(PARAM_PWM_CHANNEL(channel));
    CHECK_PARAM(PARAM_PWM_CHANNEL_EDGE(edgeMode));

    if (channel == PWM_CHANNEL_1) {
        return;
    }

    if (edgeMode == PWM_SINGLE_EDGE) {
        LPC_PWM1->PCR &= ~PWM_PCR_PWMSELn(channel);
    } else {
        LPC_PWM1->PCR |= PWM_PCR_PWMSELn(channel);
    }
}

void PWM_ChannelCmd(PWM_CHANNEL channel, FunctionalState newState) {
    CHECK_PARAM(PARAM_PWM_CHANNEL(channel));

    if (newState == ENABLE) {
        LPC_PWM1->PCR |= PWM_PCR_PWMENAn(channel);
    } else {
        LPC_PWM1->PCR &= ~PWM_PCR_PWMENAn(channel);
    }
}

void PWM_Cmd(FunctionalState newState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE) {
        LPC_PWM1->TCR |= PWM_TCR_PWM_ENABLE;
    } else {
        LPC_PWM1->TCR &= ~PWM_TCR_PWM_ENABLE;
    }
}

void PWM_CounterCmd(FunctionalState newState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));
    if (newState == ENABLE) {
        LPC_PWM1->TCR |= PWM_TCR_COUNTER_ENABLE;
    } else {
        LPC_PWM1->TCR &= ~PWM_TCR_COUNTER_ENABLE;
    }
}

void PWM_ResetCounter(void) {
    LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
    LPC_PWM1->TCR &= ~PWM_TCR_COUNTER_RESET;
}

void PWM_ConfigMatch(const PWM_MATCHCFG_Type* pwmMatchCfg) {
    CHECK_PARAM(PARAM_PWM_MATCH_OPT(pwmMatchCfg->matchChannel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(pwmMatchCfg->intOnMatch));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(pwmMatchCfg->resetOnMatch));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(pwmMatchCfg->stopOnMatch));

    LPC_PWM1->MCR &= ~PWM_MCR_CHANNEL_MASKBIT(pwmMatchCfg->matchChannel);

    if (pwmMatchCfg->intOnMatch == ENABLE) {
        LPC_PWM1->MCR |= PWM_MCR_INT(pwmMatchCfg->matchChannel);
    }

    if (pwmMatchCfg->resetOnMatch == ENABLE) {
        LPC_PWM1->MCR |= PWM_MCR_RESET(pwmMatchCfg->matchChannel);
    }

    if (pwmMatchCfg->stopOnMatch == ENABLE) {
        LPC_PWM1->MCR |= PWM_MCR_STOP(pwmMatchCfg->matchChannel);
    }

    volatile uint32_t* MR[]        = {&LPC_PWM1->MR0, &LPC_PWM1->MR1, &LPC_PWM1->MR2, &LPC_PWM1->MR3,
                                      &LPC_PWM1->MR4, &LPC_PWM1->MR5, &LPC_PWM1->MR6};
    *MR[pwmMatchCfg->matchChannel] = pwmMatchCfg->matchValue;
}

void PWM_MatchUpdate(PWM_MATCH_OPT channel, uint32_t newMatchValue, PWM_UPDATE_OPT updateType) {
    CHECK_PARAM(PARAM_PWM_MATCH_OPT(channel));
    CHECK_PARAM(PARAM_PWM_UPDATE_OPT(updateType));

    volatile uint32_t* MR[] = {&LPC_PWM1->MR0, &LPC_PWM1->MR1, &LPC_PWM1->MR2, &LPC_PWM1->MR3,
                               &LPC_PWM1->MR4, &LPC_PWM1->MR5, &LPC_PWM1->MR6};
    *MR[channel]            = newMatchValue;

    LPC_PWM1->LER |= _BIT(channel);

    if (updateType == PWM_UPDATE_NOW) {
        LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
        LPC_PWM1->TCR &= ~PWM_TCR_COUNTER_RESET;
    }
}

void PWM_ClearIntPending(PWM_INT_TYPE intFlag) {
    CHECK_PARAM(PARAM_PWM_INT_TYPE(intFlag));

    LPC_PWM1->IR = _BIT(intFlag);
}

FlagStatus PWM_GetIntStatus(PWM_INT_TYPE intFlag) {
    CHECK_PARAM(PARAM_PWM_INT_TYPE(intFlag));

    return ((LPC_PWM1->IR & _BIT(intFlag)) ? SET : RESET);
}

void PWM_ConfigCapture(PWM_CAPTURECFG_Type* capCfg) {
    CHECK_PARAM(PARAM_PWM_CAPTURE(capCfg->captureChannel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(capCfg->risingEdge));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(capCfg->fallingEdge));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(capCfg->intOnCapture));

    LPC_PWM1->CCR &= ~PWM_CCR_CHANNEL_MASKBIT(capCfg->captureChannel);

    if (capCfg->risingEdge == ENABLE) {
        LPC_PWM1->CCR |= PWM_CCR_CAP_RISING(capCfg->captureChannel);
    }

    if (capCfg->fallingEdge == ENABLE) {
        LPC_PWM1->CCR |= PWM_CCR_CAP_FALLING(capCfg->captureChannel);
    }

    if (capCfg->intOnCapture == ENABLE) {
        LPC_PWM1->CCR |= PWM_CCR_INT_ON_CAP(capCfg->captureChannel);
    }
}

uint32_t PWM_GetCaptureValue(PWM_CAPTURE capChannel) {
    CHECK_PARAM(PARAM_PWM_CAPTURE(capChannel));

    switch (capChannel) {
        case PWM_CAPTURE_0: return LPC_PWM1->CR0;

        case PWM_CAPTURE_1: return LPC_PWM1->CR1;

        default: return 0;
    }
}

/**
 * @}
 */

#endif /* _PWM */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
