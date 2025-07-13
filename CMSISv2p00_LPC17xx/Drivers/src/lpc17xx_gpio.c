/***********************************************************************//**
 * @file        lpc17xx_gpio.c
 * @brief        Contains all functions support for GPIO firmware library on LPC17xx
 * @version        2.0
 * @date        21. May. 2010
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
 **********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup GPIO
 * @{
 */

/* Includes ------------------------------------------------------------------- */
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

/* Private Functions ---------------------------------------------------------- */
/*********************************************************************//**
 * @brief       Returns a pointer to the GPIO peripheral structure for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to GPIO peripheral, or NULL if portNum is invalid.
 **********************************************************************/
static LPC_GPIO_TypeDef *GPIO_GetPointer(uint8_t portNum)
{
    LPC_GPIO_TypeDef *pGPIO = NULL;

    switch (portNum) {
    case 0:
        pGPIO = LPC_GPIO0;
        break;
    case 1:
        pGPIO = LPC_GPIO1;
        break;
    case 2:
        pGPIO = LPC_GPIO2;
        break;
    case 3:
        pGPIO = LPC_GPIO3;
        break;
    case 4:
        pGPIO = LPC_GPIO4;
        break;
    default:
        break;
    }

    return pGPIO;
}

/*********************************************************************//**
 * @brief       Returns a pointer to the FIO peripheral structure halfword
 *              accessible for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to FIO peripheral, or NULL if portNum is invalid.
 **********************************************************************/
static GPIO_HalfWord_TypeDef *FIO_HalfWordGetPointer(uint8_t portNum)
{
    GPIO_HalfWord_TypeDef *pFIO = NULL;

    switch (portNum) {
    case 0:
        pFIO = GPIO0_HalfWord;
        break;
    case 1:
        pFIO = GPIO1_HalfWord;
        break;
    case 2:
        pFIO = GPIO2_HalfWord;
        break;
    case 3:
        pFIO = GPIO3_HalfWord;
        break;
    case 4:
        pFIO = GPIO4_HalfWord;
        break;
    default:
        break;
    }

    return pFIO;
}

/*********************************************************************//**
 * @brief       Returns a pointer to the FIO peripheral structure byte
 *              accessible for the given port number.
 * @param[in]   portNum GPIO_PORT_x, where x is in the range [0,4].
 * @return      Pointer to FIO peripheral, or NULL if portNum is invalid.
 **********************************************************************/
static GPIO_Byte_TypeDef *FIO_ByteGetPointer(uint8_t portNum)
{
    GPIO_Byte_TypeDef *pFIO = NULL;

    switch (portNum) {
    case 0:
        pFIO = GPIO0_Byte;
        break;
    case 1:
        pFIO = GPIO1_Byte;
        break;
    case 2:
        pFIO = GPIO2_Byte;
        break;
    case 3:
        pFIO = GPIO3_Byte;
        break;
    case 4:
        pFIO = GPIO4_Byte;
        break;
    default:
        break;
    }

    return pFIO;
}
/* End of Private Functions --------------------------------------------------- */

/* Public Functions ----------------------------------------------------------- */
/** @addtogroup GPIO_Public_Functions
 * @{
 */

/* GPIO ------------------------------------------------------------------------------ */
void GPIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        if (dir == OUTPUT) {
            pGPIO->FIODIR |= bitValue;
        }
        else if (dir == INPUT) {
            pGPIO->FIODIR &= ~bitValue;
        }
    }
}

void GPIO_SetValue(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOSET = bitValue;
    }
}

void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOCLR = bitValue;
    }
}

void GPIO_WriteValue(uint8_t portNum, uint32_t newValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOCLR = ~newValue;
        pGPIO->FIOSET = newValue;
    }
}

uint32_t GPIO_ReadValue(uint8_t portNum)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        return pGPIO->FIOPIN;
    }

    return (0);
}

void GPIO_TogglePins(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        const uint32_t current = pGPIO->FIOPIN;
        const uint32_t toSet = (~current) & bitValue;
        const uint32_t toClear = current & bitValue;
        pGPIO->FIOSET = toSet;
        pGPIO->FIOCLR = toClear;
    }
}

void GPIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState)
{
    LPC_GPIO_TypeDef *pFIO = GPIO_GetPointer(portNum);
    if(pFIO != NULL) {
        if (newState){
            pFIO->FIOMASK |= bitValue;
        }
        else {
            pFIO->FIOMASK &= ~bitValue;
        }
    }
}

void GPIO_IntCmd(uint8_t portNum, uint32_t newValue, uint8_t edgeState)
{
    if((portNum == GPIO_PORT_0)&&(edgeState == GPIO_INT_RISING))
        LPC_GPIOINT->IO0IntEnR = newValue;
    else if ((portNum == GPIO_PORT_2)&&(edgeState == GPIO_INT_RISING))
        LPC_GPIOINT->IO2IntEnR = newValue;
    else if ((portNum == GPIO_PORT_0)&&(edgeState == GPIO_INT_FALLING))
        LPC_GPIOINT->IO0IntEnF = newValue;
    else if ((portNum == GPIO_PORT_2)&&(edgeState == GPIO_INT_FALLING))
        LPC_GPIOINT->IO2IntEnF = newValue;
}

FunctionalState GPIO_GetPortIntStatus(uint8_t portNum)
{
    if (portNum == GPIO_PORT_0)
        return (FunctionalState)((LPC_GPIOINT->IntStatus >> 0) & 0x1);
    if (portNum == GPIO_PORT_2)
        return (FunctionalState)((LPC_GPIOINT->IntStatus >> 2) & 0x1);
    return DISABLE;
}

FunctionalState GPIO_GetPinIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState)
{
    if((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_RISING))
        return (FunctionalState)(((LPC_GPIOINT->IO0IntStatR)>>pinNum)& 0x1);
    if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_RISING))
        return (FunctionalState)(((LPC_GPIOINT->IO2IntStatR)>>pinNum)& 0x1);
    if ((portNum == GPIO_PORT_0) && (edgeState == GPIO_INT_FALLING))
        return (FunctionalState)(((LPC_GPIOINT->IO0IntStatF)>>pinNum)& 0x1);
    if ((portNum == GPIO_PORT_2) && (edgeState == GPIO_INT_FALLING))
        return (FunctionalState)(((LPC_GPIOINT->IO2IntStatF)>>pinNum)& 0x1);
    return DISABLE;
}

void GPIO_ClearInt(uint8_t portNum, uint32_t bitValue)
{
    if(portNum == 0)
        LPC_GPIOINT->IO0IntClr = bitValue;
    else if (portNum == 2)
        LPC_GPIOINT->IO2IntClr = bitValue;
}

/* FIO word accessible ----------------------------------------------------------------- */
/* Stub function for FIO (word-accessible) style */
void FIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir)
{
    GPIO_SetDir(portNum, bitValue, dir);
}

void FIO_SetValue(uint8_t portNum, uint32_t bitValue)
{
    GPIO_SetValue(portNum, bitValue);
}

void FIO_ClearValue(uint8_t portNum, uint32_t bitValue)
{
    GPIO_ClearValue(portNum, bitValue);
}

void FIO_WriteValue(uint8_t portNum, uint32_t newValue)
{
    GPIO_WriteValue(portNum, newValue);
}

uint32_t FIO_ReadValue(uint8_t portNum)
{
    return (GPIO_ReadValue(portNum));
}

void FIO_TogglePins(uint8_t portNum, uint32_t bitValue)
{
    GPIO_TogglePins(portNum, bitValue);
}

void FIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState)
{
    GPIO_SetMask(portNum, bitValue, newState);
}

void FIO_IntCmd(uint8_t portNum, uint32_t bitValue, uint8_t edgeState)
{
    GPIO_IntCmd(portNum, bitValue, edgeState);
}

FunctionalState FIO_GetPortIntStatus(uint8_t portNum)
{
    return (GPIO_GetPortIntStatus(portNum));
}

FunctionalState FIO_GetPinIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState)
{
    return (GPIO_GetPinIntStatus(portNum, pinNum, edgeState));
}

void FIO_ClearInt(uint8_t portNum, uint32_t bitValue)
{
    GPIO_ClearInt(portNum, bitValue);
}

/* FIO halfword accessible ------------------------------------------------------------- */

void FIO_HalfWordSetDir(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, uint8_t dir)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (dir == OUTPUT) {
            if(halfwordNum == HIGH_HALFWORD) {
                pFIO->FIODIRU |= bitValue;
            }
            else if (halfwordNum == LOW_HALFWORD) {
                pFIO->FIODIRL |= bitValue;
            }
        }
        else if (dir == INPUT) {
            if (halfwordNum == HIGH_HALFWORD) {
                pFIO->FIODIRU &= ~bitValue;
            }
            else if (halfwordNum == LOW_HALFWORD) {
                pFIO->FIODIRL &= ~bitValue;
            }
        }
    }
}

void FIO_HalfWordSetValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            pFIO->FIOSETU = bitValue;
        }
        else if(halfwordNum == LOW_HALFWORD) {
            pFIO->FIOSETL = bitValue;
        }
    }
}

void FIO_HalfWordClearValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            pFIO->FIOCLRU = bitValue;
        }
        else if (halfwordNum == LOW_HALFWORD) {
            pFIO->FIOCLRL = bitValue;
        }
    }
}

void FIO_HalfWordWriteValue(uint8_t portNum, uint8_t halfwordNum, uint16_t newValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            pFIO->FIOCLRU = ~newValue;
            pFIO->FIOSETU = newValue;
        }
        else if (halfwordNum == LOW_HALFWORD) {
            pFIO->FIOCLRL = ~newValue;
            pFIO->FIOSETL = newValue;
        }
    }
}

uint16_t FIO_HalfWordReadValue(uint8_t portNum, uint8_t halfwordNum)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            return (pFIO->FIOPINU);
        }
        if (halfwordNum == LOW_HALFWORD) {
            return (pFIO->FIOPINL);
        }
    }
    return (0);
}

void FIO_HalfWordTogglePins(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            const uint16_t current = pFIO->FIOPINU;
            const uint16_t toSet = (~current) & bitValue;
            const uint16_t toClear = current & bitValue;
            pFIO->FIOSETU = toSet;
            pFIO->FIOCLRU = toClear;
        }
        else if (halfwordNum == LOW_HALFWORD) {
            const uint16_t current = pFIO->FIOPINL;
            const uint16_t toSet = (~current) & bitValue;
            const uint16_t toClear = current & bitValue;
            pFIO->FIOSETL = toSet;
            pFIO->FIOCLRL = toClear;
        }
    }
}

void FIO_HalfWordSetMask(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, FunctionalState newState)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (newState == ENABLE) {
            if (halfwordNum == HIGH_HALFWORD) {
                pFIO->FIOMASKU |= bitValue;
            }
            else if (halfwordNum == LOW_HALFWORD) {
                pFIO->FIOMASKL |= bitValue;
            }
        }
        else {
            if (halfwordNum == HIGH_HALFWORD) {
                pFIO->FIOMASKU &= ~bitValue;
            }
            else if (halfwordNum == LOW_HALFWORD) {
                pFIO->FIOMASKL &= ~bitValue;
            }
        }
    }
}


/* FIO Byte accessible ------------------------------------------------------------ */

void FIO_ByteSetDir(uint8_t portNum, uint8_t byteNum, uint8_t bitValue, uint8_t dir)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if(pFIO != NULL) {
        if (dir == OUTPUT) {
            if (byteNum <= 3) {
                pFIO->FIODIR[byteNum] |= bitValue;
            }
        }
        else if (dir == INPUT) {
            if (byteNum <= 3) {
                pFIO->FIODIR[byteNum] &= ~bitValue;
            }
        }
    }
}

void FIO_ByteSetValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOSET[byteNum] = bitValue;
        }
    }
}

void FIO_ByteClearValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOCLR[byteNum] = bitValue;
        }
    }
}

void FIO_ByteWriteValue(uint8_t portNum, uint8_t byteNum, uint8_t newValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL && byteNum <= 3) {
        pFIO->FIOCLR[byteNum] = ~newValue;
        pFIO->FIOSET[byteNum] = newValue;
    }
}

uint8_t FIO_ByteReadValue(uint8_t portNum, uint8_t byteNum)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            return (pFIO->FIOPIN[byteNum]);
        }
    }
    return (0);
}

void FIO_ByteTogglePins(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL && byteNum <= 3) {
        uint8_t current = pFIO->FIOPIN[byteNum];
        uint8_t toSet = (~current) & bitValue;
        uint8_t toClear = current & bitValue;
        pFIO->FIOSET[byteNum] = toSet;
        pFIO->FIOCLR[byteNum] = toClear;
    }
}

void FIO_ByteSetMask(uint8_t portNum, uint8_t byteNum, uint8_t bitValue, FunctionalState newState)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if(pFIO != NULL) {
        if (newState == ENABLE) {
            if (byteNum <= 3) {
                pFIO->FIOMASK[byteNum] |= bitValue;
            }
        }
        else if (newState == DISABLE) {
            if (byteNum <= 3) {
                pFIO->FIOMASK[byteNum] &= ~bitValue;
            }
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

/* --------------------------------- End Of File ------------------------------ */
