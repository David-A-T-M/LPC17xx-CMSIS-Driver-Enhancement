/**
 * @file        lpc17xx_pinsel.c
 * @brief       Contains all functions support for Pin connect block firmware
 *              library on LPC17xx
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
 * Date: 07/07/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @addtogroup PINSEL
 * @{
 */

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx_pinsel.h"
#include "lpc17xx_libcfg_default.h"

/* ---------------------- Private Function Prototypes ----------------------- */
/**
 * @brief       Sets up the pin selection function.
 *
 * @param[in]   portNum  PINSEL_PORT_x, where x is in the range [0,4].
 * @param[in]   pinNum   PINSEL_PIN_x, where x is in the range [0,31].
 * @param[in]   funcNum  PINSEL_FUNC_x, where x is in the range [0,3].
 */
static void set_PinFunc(uint8_t portNum, uint8_t pinNum, uint8_t funcNum);

/**
 * @brief       Configures the resistor mode for a pin.
 *
 * @param[in]   portNum  PINSEL_PORT_x, where x is in the range [0,4].
 * @param[in]   pinNum   PINSEL_PIN_x, where x is in the range [0,31].
 * @param[in]   pinMode: Must be one of:
 *                       - PINSEL_PULLUP    : Internal pull-up resistor.
 *                       - PINSEL_REPEATER  : Repeater mode.
 *                       - PINSEL_TRISTATE  : Tri-state.
 *                       - PINSEL_PULLDOWN  : Internal pull-down resistor.
 */
static void set_ResistorMode(uint8_t portNum, uint8_t pinNum, uint8_t pinMode);

/**
 * @brief       Configures the open-drain mode for a pin.
 *
 * @param[in]   portNum   PINSEL_PORT_x, where x is in the range [0,4].
 * @param[in]   pinNum    PINSEL_PIN_x, where x is in the range [0,31].
 * @param[in]   openDrain Must be one of:
 *                        - PINSEL_OD_NORMAL     : Normal mode (not open-drain).
 *                        - PINSEL_OD_OPENDRAIN  : Open-drain mode.
 */
static void set_OpenDrainMode(uint8_t portNum, uint8_t pinNum, uint8_t openDrain);
/* ------------------- End of Private Function Prototypes ------------------- */

/* --------------------------- Private Functions ---------------------------- */
static void set_PinFunc(uint8_t portNum, uint8_t pinNum, uint8_t funcNum) {
    uint32_t pinNum_t      = pinNum;
    uint32_t pinselReg_idx = 2 * portNum;
    uint32_t* pPinCon      = (uint32_t*)&LPC_PINCON->PINSEL0;

    if (pinNum_t >= 16) {
        pinNum_t -= 16;
        pinselReg_idx++;
    }
    *(uint32_t*)(pPinCon + pinselReg_idx) &= ~(PINSEL_FUNC_MASK << (pinNum_t * 2));
    *(uint32_t*)(pPinCon + pinselReg_idx) |= ((uint32_t)funcNum) << (pinNum_t * 2);
}

static void set_ResistorMode(uint8_t portNum, uint8_t pinNum, uint8_t pinMode) {
    uint32_t pinNum_t       = pinNum;
    uint32_t pinmodeReg_idx = 2 * portNum;
    uint32_t* pPinCon       = (uint32_t*)&LPC_PINCON->PINMODE0;

    if (pinNum_t >= 16) {
        pinNum_t -= 16;
        pinmodeReg_idx++;
    }

    *(uint32_t*)(pPinCon + pinmodeReg_idx) &= ~(PINSEL_FUNC_MASK << (pinNum_t * 2));
    *(uint32_t*)(pPinCon + pinmodeReg_idx) |= ((uint32_t)pinMode) << (pinNum_t * 2);
}

static void set_OpenDrainMode(uint8_t portNum, uint8_t pinNum, uint8_t openDrain) {
    uint32_t* pPinCon = (uint32_t*)&LPC_PINCON->PINMODE_OD0;

    if (openDrain == PINSEL_OD_OPENDRAIN) {
        *(uint32_t*)(pPinCon + portNum) |= (PINSEL_PIN_MASK << pinNum);
    } else {
        *(uint32_t*)(pPinCon + portNum) &= ~(PINSEL_PIN_MASK << pinNum);
    }
}
/* ------------------------ End of Private Functions ------------------------ */

/* ---------------------------- Public Functions ---------------------------- */
/** @addtogroup PINSEL_Public_Functions
 * @{
 */

void PINSEL_ConfigPin(const PINSEL_CFG_Type* pinCfg) {
    CHECK_PARAM(PARAM_PINSEL_PORT(pinCfg->portNum));
    CHECK_PARAM(PARAM_PINSEL_PIN(pinCfg->pinNum));
    CHECK_PARAM(PARAM_PINSEL_FUNC(pinCfg->funcNum));
    CHECK_PARAM(PARAM_PINSEL_PINMODE(pinCfg->pinMode));
    CHECK_PARAM(PARAM_PINSEL_OD(pinCfg->openDrain));

    set_PinFunc(pinCfg->portNum, pinCfg->pinNum, pinCfg->funcNum);
    set_ResistorMode(pinCfg->portNum, pinCfg->pinNum, pinCfg->pinMode);
    set_OpenDrainMode(pinCfg->portNum, pinCfg->pinNum, pinCfg->openDrain);
}

void PINSEL_ConfigMultiplePins(const PINSEL_CFG_Type* pinCfg, uint32_t pins) {
    CHECK_PARAM(PARAM_PINSEL_PORT(pinCfg->portNum));
    CHECK_PARAM(PARAM_PINSEL_FUNC(pinCfg->funcNum));

    PINSEL_CFG_Type tempCfg = *pinCfg;
    for (uint8_t pin = 0; pin < 32; pin++) {
        if (pins & (1U << pin)) {
            tempCfg.pinNum = pin;
            PINSEL_ConfigPin(&tempCfg);
        }
    }
}

void PINSEL_ConfigTraceFunc(FunctionalState newState) {
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE) {
        LPC_PINCON->PINSEL10 |= (PINSEL_PIN_MASK << PINSEL_TRACE_POS);
    } else {

        LPC_PINCON->PINSEL10 &= ~(PINSEL_PIN_MASK << PINSEL_TRACE_POS);
    }
}

void PINSEL_SetI2CPins(PINSEL_I2C_MODE driveMode, FunctionalState filterSlewRate) {
    CHECK_PARAM(PARAM_PINSEL_I2C_MODE(driveMode));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(filterSlewRate));

    uint32_t regVal = 0;

    if (driveMode == PINSEL_I2C_FAST) {
        regVal = PINSEL_I2CPADCFG_SCLDRV0 | PINSEL_I2CPADCFG_SDADRV0;
    }

    if (filterSlewRate == DISABLE) {
        regVal = PINSEL_I2CPADCFG_SCLI2C0 | PINSEL_I2CPADCFG_SDAI2C0;
    }

    LPC_PINCON->I2CPADCFG = regVal;
}

/**
 * @}
 */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
