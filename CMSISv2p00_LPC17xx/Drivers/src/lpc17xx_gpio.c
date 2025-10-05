/**
 * @file        lpc17xx_gpio.c
 * @brief        Contains all functions support for GPIO firmware library on LPC17xx
 * @version        2.0
 * @date        21. May. 2010
 * @author        NXP MCU SW Application Team
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
/** @addtogroup GPIO
 * @{
 */

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx_gpio.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _GPIO

/* ---------------------- Private Function Prototypes ----------------------- */
/**
 * @brief       Returns a pointer to the GPIO peripheral structure for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to GPIO peripheral, or NULL if portNum is invalid.
 */
static LPC_GPIO_TypeDef* GPIO_GetPointer(uint8_t portNum);

/**
 * @brief       Returns a pointer to the FIO peripheral structure halfword
 *              accessible for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to FIO peripheral, or NULL if portNum is invalid.
 */
static GPIO_HalfWord_TypeDef* FIO_HalfWordGetPointer(uint8_t portNum);

/**
 * @brief       Returns a pointer to the FIO peripheral structure byte
 *              accessible for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to FIO peripheral, or NULL if portNum is invalid.
 */
static GPIO_Byte_TypeDef* FIO_ByteGetPointer(uint8_t portNum);
/* ------------------- End of Private Function Prototypes ------------------- */

/* --------------------------- Private Functions ---------------------------- */
static LPC_GPIO_TypeDef* GPIO_GetPointer(uint8_t portNum) {
    LPC_GPIO_TypeDef* pGPIO = NULL;

    switch (portNum) {
        case 0: pGPIO = LPC_GPIO0; break;
        case 1: pGPIO = LPC_GPIO1; break;
        case 2: pGPIO = LPC_GPIO2; break;
        case 3: pGPIO = LPC_GPIO3; break;
        case 4: pGPIO = LPC_GPIO4; break;
        default: break;
    }

    return pGPIO;
}

static GPIO_HalfWord_TypeDef* FIO_HalfWordGetPointer(uint8_t portNum) {
    GPIO_HalfWord_TypeDef* pFIO = NULL;

    switch (portNum) {
        case 0: pFIO = GPIO0_HalfWord; break;
        case 1: pFIO = GPIO1_HalfWord; break;
        case 2: pFIO = GPIO2_HalfWord; break;
        case 3: pFIO = GPIO3_HalfWord; break;
        case 4: pFIO = GPIO4_HalfWord; break;
        default: break;
    }

    return pFIO;
}

static GPIO_Byte_TypeDef* FIO_ByteGetPointer(uint8_t portNum) {
    GPIO_Byte_TypeDef* pFIO = NULL;

    switch (portNum) {
        case 0: pFIO = GPIO0_Byte; break;
        case 1: pFIO = GPIO1_Byte; break;
        case 2: pFIO = GPIO2_Byte; break;
        case 3: pFIO = GPIO3_Byte; break;
        case 4: pFIO = GPIO4_Byte; break;
        default: break;
    }

    return pFIO;
}
/* ------------------------ End of Private Functions ------------------------ */

/* ---------------------------- Public Functions ---------------------------- */
/** @addtogroup GPIO_Public_Functions
 * @{
 */

/* ------------------------------- GPIO style ------------------------------- */
void GPIO_SetDir(GPIO_PORT portNum, uint32_t pins, GPIO_DIR dir) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_DIR(dir));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        if (dir == GPIO_OUTPUT) {
            pGPIO->FIODIR |= pins;
        } else {
            pGPIO->FIODIR &= ~pins;
        }
    }
}

void GPIO_SetPins(GPIO_PORT portNum, uint32_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOSET = pins;
    }
}

void GPIO_ClearPins(GPIO_PORT portNum, uint32_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOCLR = pins;
    }
}

void GPIO_WriteValue(GPIO_PORT portNum, uint32_t newValue) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOCLR = ~newValue;
        pGPIO->FIOSET = newValue;
    }
}

uint32_t GPIO_ReadValue(GPIO_PORT portNum) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        return pGPIO->FIOPIN;
    }

    return 0;
}

void GPIO_TogglePins(GPIO_PORT portNum, uint32_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));

    LPC_GPIO_TypeDef* pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        const uint32_t current = pGPIO->FIOPIN;

        pGPIO->FIOSET = (~current) & pins;
        pGPIO->FIOCLR = current & pins;
    }
}

void GPIO_SetMask(GPIO_PORT portNum, uint32_t pins, FunctionalState newState) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    LPC_GPIO_TypeDef* pFIO = GPIO_GetPointer(portNum);

    if (pFIO != NULL) {
        if (newState) {
            pFIO->FIOMASK |= pins;
        } else {
            pFIO->FIOMASK &= ~pins;
        }
    }
}

void GPIO_IntCmd(GPIO_PORT portNum, uint32_t newValue, GPIO_INT_EDGE edgeState) {
    CHECK_PARAM(PARAM_GPIO_INT_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_INT_EDGE(edgeState));

    if ((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_RISING)) {
        LPC_GPIOINT->IO0IntEnR = newValue;

    } else if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_RISING)) {
        LPC_GPIOINT->IO2IntEnR = newValue;

    } else if ((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_FALLING)) {
        LPC_GPIOINT->IO0IntEnF = newValue;

    } else if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_FALLING)) {
        LPC_GPIOINT->IO2IntEnF = newValue;
    }
}

FunctionalState GPIO_GetPortIntStatus(GPIO_PORT portNum) {
    CHECK_PARAM(PARAM_GPIO_INT_PORT(portNum));

    if (portNum == GPIO_PORT_0) {
        return (FunctionalState)((LPC_GPIOINT->IntStatus >> 0) & 0x1);
    }

    return (FunctionalState)((LPC_GPIOINT->IntStatus >> 2) & 0x1);
}

FunctionalState GPIO_GetPinIntStatus(GPIO_PORT portNum, uint32_t pinNum, GPIO_INT_EDGE edgeState) {
    CHECK_PARAM(PARAM_GPIO_INT_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_INT_EDGE(edgeState));

    if ((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_RISING)) {
        return (FunctionalState)(((LPC_GPIOINT->IO0IntStatR) >> pinNum) & 0x1);
    }
    if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_RISING)) {
        return (FunctionalState)(((LPC_GPIOINT->IO2IntStatR) >> pinNum) & 0x1);
    }
    if ((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_FALLING)) {
        return (FunctionalState)(((LPC_GPIOINT->IO0IntStatF) >> pinNum) & 0x1);
    }
    if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_FALLING)) {
        return (FunctionalState)(((LPC_GPIOINT->IO2IntStatF) >> pinNum) & 0x1);
    }

    return DISABLE;
}

void GPIO_ClearInt(GPIO_PORT portNum, uint32_t pins) {
    CHECK_PARAM(PARAM_GPIO_INT_PORT(portNum));

    if (portNum == 0) {
        LPC_GPIOINT->IO0IntClr = pins;
    } else {
        LPC_GPIOINT->IO2IntClr = pins;
    }
}

/* ---------------------- FIO (word-accessible) style ----------------------- */
void FIO_SetDir(GPIO_PORT portNum, uint32_t pins, GPIO_DIR dir) {
    GPIO_SetDir(portNum, pins, dir);
}

void FIO_SetPins(GPIO_PORT portNum, uint32_t pins) {
    GPIO_SetPins(portNum, pins);
}

void FIO_ClearPins(GPIO_PORT portNum, uint32_t pins) {
    GPIO_ClearPins(portNum, pins);
}

void FIO_WriteValue(GPIO_PORT portNum, uint32_t newValue) {
    GPIO_WriteValue(portNum, newValue);
}

uint32_t FIO_ReadValue(GPIO_PORT portNum) {
    return (GPIO_ReadValue(portNum));
}

void FIO_TogglePins(GPIO_PORT portNum, uint32_t pins) {
    GPIO_TogglePins(portNum, pins);
}

void FIO_SetMask(GPIO_PORT portNum, uint32_t pins, FunctionalState newState) {
    GPIO_SetMask(portNum, pins, newState);
}

void FIO_IntCmd(GPIO_PORT portNum, uint32_t newValue, GPIO_INT_EDGE edgeState) {
    GPIO_IntCmd(portNum, newValue, edgeState);
}

FunctionalState FIO_GetPortIntStatus(GPIO_PORT portNum) {
    return (GPIO_GetPortIntStatus(portNum));
}

FunctionalState FIO_GetPinIntStatus(GPIO_PORT portNum, uint32_t pinNum, GPIO_INT_EDGE edgeState) {
    return (GPIO_GetPinIntStatus(portNum, pinNum, edgeState));
}

void FIO_ClearInt(GPIO_PORT portNum, uint32_t pins) {
    GPIO_ClearInt(portNum, pins);
}

/* -------------------- FIO (halfword-accessible) style --------------------- */
void FIO_HalfWordSetDir(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins, GPIO_DIR dir) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));
    CHECK_PARAM(PARAM_GPIO_DIR(dir));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (dir == GPIO_OUTPUT) {
            if (halfwordNum == GPIO_HALFWORD_HIGH) {
                pFIO->FIODIRU |= pins;
            } else {
                pFIO->FIODIRL |= pins;
            }
        } else {
            if (halfwordNum == GPIO_HALFWORD_HIGH) {
                pFIO->FIODIRU &= ~pins;
            } else {
                pFIO->FIODIRL &= ~pins;
            }
        }
    }
}

void FIO_HalfWordSetPins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == GPIO_HALFWORD_HIGH) {
            pFIO->FIOSETU = pins;
        } else {
            pFIO->FIOSETL = pins;
        }
    }
}

void FIO_HalfWordClearPins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == GPIO_HALFWORD_HIGH) {
            pFIO->FIOCLRU = pins;
        } else {
            pFIO->FIOCLRL = pins;
        }
    }
}

void FIO_HalfWordWriteValue(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t newValue) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == GPIO_HALFWORD_HIGH) {
            pFIO->FIOCLRU = ~newValue;
            pFIO->FIOSETU = newValue;
        } else {
            pFIO->FIOCLRL = ~newValue;
            pFIO->FIOSETL = newValue;
        }
    }
}

uint16_t FIO_HalfWordReadValue(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == GPIO_HALFWORD_HIGH) {
            return (pFIO->FIOPINU);
        }

        return (pFIO->FIOPINL);
    }
    return (0);
}

void FIO_HalfWordTogglePins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == GPIO_HALFWORD_HIGH) {
            const uint16_t current = pFIO->FIOPINU;

            pFIO->FIOSETU = (~current) & pins;
            pFIO->FIOCLRU = current & pins;
        } else {
            const uint16_t current = pFIO->FIOPINL;

            pFIO->FIOSETL = (~current) & pins;
            pFIO->FIOCLRL = current & pins;
        }
    }
}

void FIO_HalfWordSetMask(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins, FunctionalState newState) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_HALFWORD(halfwordNum));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    GPIO_HalfWord_TypeDef* pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (newState == ENABLE) {
            if (halfwordNum == GPIO_HALFWORD_HIGH) {
                pFIO->FIOMASKU |= pins;
            } else {
                pFIO->FIOMASKL |= pins;
            }
        } else {
            if (halfwordNum == GPIO_HALFWORD_HIGH) {
                pFIO->FIOMASKU &= ~pins;
            } else {
                pFIO->FIOMASKL &= ~pins;
            }
        }
    }
}

/* ---------------------- FIO (byte-accessible) style ----------------------- */
void FIO_ByteSetDir(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins, GPIO_DIR dir) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));
    CHECK_PARAM(PARAM_GPIO_DIR(dir));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (dir == GPIO_OUTPUT) {
            pFIO->FIODIR[byteNum] |= pins;
        } else {
            pFIO->FIODIR[byteNum] &= ~pins;
        }
    }
}

void FIO_ByteSetPins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        pFIO->FIOSET[byteNum] = pins;
    }
}

void FIO_ByteClearPins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        pFIO->FIOCLR[byteNum] = pins;
    }
}

void FIO_ByteWriteValue(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t newValue) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        pFIO->FIOCLR[byteNum] = ~newValue;
        pFIO->FIOSET[byteNum] = newValue;
    }
}

uint8_t FIO_ByteReadValue(GPIO_PORT portNum, GPIO_BYTE byteNum) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        return (pFIO->FIOPIN[byteNum]);
    }
    return (0);
}

void FIO_ByteTogglePins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        const uint8_t current = pFIO->FIOPIN[byteNum];
        pFIO->FIOSET[byteNum] = (~current) & pins;
        pFIO->FIOCLR[byteNum] = current & pins;
    }
}

void FIO_ByteSetMask(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins, FunctionalState newState) {
    CHECK_PARAM(PARAM_GPIO_PORT(portNum));
    CHECK_PARAM(PARAM_GPIO_BYTE(byteNum));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    GPIO_Byte_TypeDef* pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (newState == ENABLE) {
            pFIO->FIOMASK[byteNum] |= pins;
        } else {
            pFIO->FIOMASK[byteNum] &= ~pins;
        }
    }
}

/**
 * @}
 */

#endif /* _GPIO */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
