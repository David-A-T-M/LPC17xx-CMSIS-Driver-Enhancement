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

static LPC_GPIO_TypeDef *GPIO_GetPointer(uint8_t portNum);
static GPIO_HalfWord_TypeDef *FIO_HalfWordGetPointer(uint8_t portNum);
static GPIO_Byte_TypeDef *FIO_ByteGetPointer(uint8_t portNum);

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

/*********************************************************************//**
 * @brief        Sets the direction for the specified GPIO port pins.
 *
 * @param[in]    portNum    GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    bitValue   Bitmask of pins to configure (0x0 to 0xFFFFFFFF).
 *                          Example: value 0x5 to set direction for bit 0 and bit 2.
 * @param[in]    dir        Must be:
 *                          - INPUT : Input direction.
 *                          - OUTPUT : Output direction.
 *
 * @note - Pins not selected in bitValue are not affected.
 * @note - If portNum or dir is invalid, the function has no effect.
 *
 * @return      None
 **********************************************************************/
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


/*********************************************************************//**
 * @brief       Sets the specified output pins to high on a given GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]   bitValue  Bitmask specifying which pins to set high (1 = set).
 *                        Example: 0x5 sets pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum is invalid, the function has no effect.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 *
 * @return      None
 **********************************************************************/
void GPIO_SetValue(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOSET = bitValue;
    }
}

/*********************************************************************//**
 * @brief       Clears the specified output pins to low on a given GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]   bitValue  Bitmask specifying which pins to set low (1 = clear).
 *                        Example: 0x5 clears pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum is invalid, the function has no effect.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 *
 * @return      None
 **********************************************************************/
void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOCLR = bitValue;
    }
}

/*********************************************************************//**
 * @brief       Writes a value to all pins of the specified GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]   newValue  Value to be written to the FIOPIN register.
 *                        Each bit corresponds to a pin (1 = high, 0 = low).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 * @note - If portNum is invalid, the function has no effect.
 *
 * @return      None
 **********************************************************************/
void GPIO_WriteValue(uint8_t portNum, uint32_t newValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOPIN = newValue;
    }
}

/*********************************************************************//**
 * @brief       Reads the current state of all pins on the specified GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 *
 * @note - The returned value contains the logic state of each pin (bit) on the port,
 *         regardless of whether the pin is configured as input or output.
 * @note - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 * @note - If portNum is invalid, the function returns 0.
 *
 * @return      32-bit value representing the current state of all port pins.
 **********************************************************************/
uint32_t GPIO_ReadValue(uint8_t portNum)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        return pGPIO->FIOPIN;
    }

    return (0);
}

/*********************************************************************//**
 * @brief       Toggles the state of specified pins on the given GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]   bitValue  Bitmask specifying which pins to toggle (1 = toggle).
 *                        Example: 0x5 toggles pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum is invalid, the function has no effect.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 *
 * @return      None
 **********************************************************************/
void GPIO_TogglePins(uint8_t portNum, uint32_t bitValue)
{
    LPC_GPIO_TypeDef *pGPIO = GPIO_GetPointer(portNum);

    if (pGPIO != NULL) {
        pGPIO->FIOPIN ^= bitValue;
    }
}

/*********************************************************************//**
 * @brief       Sets or clears the mask for specified pins on the given GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]   bitValue  Bitmask specifying which pins to mask or unmask (1 = select).
 *                        Example: 0x5 selects pins 0 and 2.
 * @param[in]   newState  FunctionalState value:
 *                        - ENABLE: Mask the selected pins (access disabled).
 *                        - DISABLE: Unmask the selected pins (access enabled).
 *
 * @note - Only the pins selected in bitValue are affected.
 * @note - If portNum is invalid, the function has no effect.
 * @note - After masking, write/read operations to masked pins will have no effect or return 0.
 *
 * @return      None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief       Sets the interrupt enable mask for GPIO pins on the given port.
 *
 * @param[in]   portNum     GPIO_PORT_x, must be 0 or 2.
 * @param[in]   newValue    Bitmask written directly to the interrupt enable register.
 *                          Each bit: 1 = enable interrupt, 0 = disable interrupt.
 *                          Example: 0x5 enables interrupt for pins 0 and 2, disables others.
 * @param[in]   edgeState   Interrupt edge selection:
 *                          - GPIO_INT_RISING:  Rising edge interrupt.
 *                          - GPIO_INT_FALLING: Falling edge interrupt.
 *
 * @note - The entire interrupt enable register is overwritten; all pins not set in
 *         newValue will have their interrupts disabled.
 * @note - Only pins P0.0-P0.11, P0.15-P0.30, and P2.0-P2.13 support interrupts.
 * @note - If portNum or edgeState is invalid, the function has no effect.
 *
 * @return      None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief       Gets the interrupt status for the entire GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x, must be 0 or 2.
 *
 * @note - Only port 0 and port 2 support interrupts.
 * @note - If portNum is not 0 or 2, the function returns DISABLE.
 *
 * @return      ENABLE if any interrupt is pending on the selected port,
 *              DISABLE otherwise.
 **********************************************************************/
FunctionalState GPIO_GetPortIntStatus(uint8_t portNum)
{
    if (portNum == GPIO_PORT_0)
        return (FunctionalState)((LPC_GPIOINT->IntStatus >> 0) & 0x1);
    if (portNum == GPIO_PORT_2)
        return (FunctionalState)((LPC_GPIOINT->IntStatus >> 2) & 0x1);
    return DISABLE;
}

/*********************************************************************//**
 * @brief       Gets the interrupt status for a specific GPIO pin and edge.
 *
 * @param[in]   portNum     GPIO_PORT_x, must be 0 or 2.
 * @param[in]   pinNum      Only pins 0-11, 15-30 for port 0 and 0-13 for port 2 support interrupts.
 * @param[in]   edgeState   Interrupt edge selection:
 *                          - GPIO_INT_RISING:  Rising edge interrupt status.
 *                          - GPIO_INT_FALLING: Falling edge interrupt status.
 *
 * @note - If portNum or edgeState is invalid, the function returns DISABLE.
 *
 * @return      ENABLE if an interrupt has been generated for the selected pin and edge,
 *              DISABLE otherwise.
 **********************************************************************/
FunctionalState GPIO_GetIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState)
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

/*********************************************************************//**
 * @brief       Clears the interrupt status for selected GPIO pins.
 *
 * @param[in]   portNum    GPIO_PORT_x, must be 0 or 2.
 * @param[in]   bitValue   Bitmask specifying which pins to clear interrupt status.
 *                         Example: 0x5 clears interrupt for pins 0 and 2.
 *
 * @note - Only pins P0.0-P0.11, P0.15-P0.30, and P2.0-P2.13 support interrupts.
 * @note - If portNum is not 0 or 2, the function has no effect.
 *
 * @return      None
 **********************************************************************/
void GPIO_ClearInt(uint8_t portNum, uint32_t bitValue)
{
    if(portNum == 0)
        LPC_GPIOINT->IO0IntClr = bitValue;
    else if (portNum == 2)
        LPC_GPIOINT->IO2IntClr = bitValue;
}

/* FIO word accessible ----------------------------------------------------------------- */
/* Stub function for FIO (word-accessible) style */

/**
 * @brief The same as GPIO_SetDir().
 */
void FIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir)
{
    GPIO_SetDir(portNum, bitValue, dir);
}

/**
 * @brief The same as GPIO_SetValue()
 */
void FIO_SetValue(uint8_t portNum, uint32_t bitValue)
{
    GPIO_SetValue(portNum, bitValue);
}

/**
 * @brief The same as GPIO_ClearValue()
 */
void FIO_ClearValue(uint8_t portNum, uint32_t bitValue)
{
    GPIO_ClearValue(portNum, bitValue);
}

/**
 * @brief The same as GPIO_WriteValue()
 */
void FIO_WriteValue(uint8_t portNum, uint32_t newValue)
{
    GPIO_WriteValue(portNum, newValue);
}

/**
 * @brief The same with GPIO_ReadValue()
 */
uint32_t FIO_ReadValue(uint8_t portNum)
{
    return (GPIO_ReadValue(portNum));
}

/**
 * @brief The same with GPIO_TogglePins()
 */
void FIO_TogglePins(uint8_t portNum, uint32_t bitValue)
{
    GPIO_TogglePins(portNum, bitValue);
}

/**
 * @brief The same with GPIO_SetMask()
 */
void FIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState)
{
    GPIO_SetMask(portNum, bitValue, newState);
}

/**
 * @brief The same with GPIO_IntCmd()
 */
void FIO_IntCmd(uint8_t portNum, uint32_t bitValue, uint8_t edgeState)
{
    GPIO_IntCmd(portNum, bitValue, edgeState);
}

/**
 * @brief The same with GPIO_GetPortIntStatus()
 */
FunctionalState FIO_GetPortIntStatus(uint8_t portNum)
{
    return (GPIO_GetPortIntStatus(portNum));
}

/**
 * @brief The same with GPIO_GetIntStatus()
 */
FunctionalState FIO_GetIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState)
{
    return (GPIO_GetIntStatus(portNum, pinNum, edgeState));
}

/**
 * @brief The same with GPIO_ClearInt()
 */
void FIO_ClearInt(uint8_t portNum, uint32_t bitValue)
{
    GPIO_ClearInt(portNum, bitValue);
}


/* FIO halfword accessible ------------------------------------------------------------- */

/*********************************************************************//**
 * @brief        Sets the direction of specified pins for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    bitValue       Bitmask indicating which pins to configure (0x0 to 0xFFFF).
 *                              Example: 0x0005 configures bits 0 and 2.
 * @param[in]    dir            Must be:
 *                              - INPUT
 *                              - OUTPUT
 *
 * @note - Pins not selected in bitValue are not affected.
 * @note - If portNum or halfwordNum or dir are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Sets the specified output pins to high for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    bitValue       Bitmask indicating which pins to set high (0x0 to 0xFFFF).
 *                              Example: 0x0005 sets pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or halfwordNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Clears the specified output pins to low for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    bitValue       Bitmask indicating which pins to set low (0x0 to 0xFFFF).
 *                              Example: 0x0005 clears pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or halfwordNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Writes a value to all pins of the specified FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to write:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    newValue       Value to be written to the FIO halfword register (0x0 to 0xFFFF).
 *                              Each bit corresponds to a pin (1 = high, 0 = low).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 * @note - If portNum or halfwordNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_HalfWordWriteValue(uint8_t portNum, uint8_t halfwordNum, uint16_t newValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            pFIO->FIOPINU = newValue;
        }
        else if (halfwordNum == LOW_HALFWORD) {
            pFIO->FIOPINL = newValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Reads the current state of all pins on the specified FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to read:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 *
 * @note - The returned value contains the logic state of each pin (bit) in the selected halfword,
 *         regardless of whether the pin is configured as input or output.
 * @note - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 * @note - If portNum or halfwordNum are invalid, the function returns 0.
 *
 * @return       16-bit value representing the current state of all pins in the selected halfword.
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Toggles the state of specified pins for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    bitValue       Bitmask indicating which pins to toggle (0x0 to 0xFFFF).
 *                              Example: 0x0005 toggles pins 0 and 2.
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or halfwordNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_HalfWordTogglePins(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue)
{
    GPIO_HalfWord_TypeDef *pFIO = FIO_HalfWordGetPointer(portNum);
    if (pFIO != NULL) {
        if (halfwordNum == HIGH_HALFWORD) {
            pFIO->FIOPINU ^= bitValue;
        }
        else if (halfwordNum == LOW_HALFWORD) {
            pFIO->FIOPINL ^= bitValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Sets or clears the mask for selected bits in a FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - LOW_HALFWORD : bits 0-15.
 *                              - HIGH_HALFWORD : bits 16-31.
 * @param[in]    bitValue       Bitmask indicating which bits to mask or unmask (0x0 to 0xFFFF).
 * @param[in]    newState       FunctionalState value:
 *                              - ENABLE: Mask the selected bits (access disabled).
 *                              - DISABLE: Unmask the selected bits (access enabled).
 *
 * @note - Only the bits selected in bitValue are affected.
 * @note - After masking, read/write operations to masked bits will have no effect or return 0.
 * @note - If portNum or halfwordNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Sets the direction for specified pins in a FIO port byte.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Must be
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    bitValue    Bitmask indicating which bits to configure (0x0 to 0xFF).
 * @param[in]    dir         Must be:
 *                           - INPUT
 *                           - OUTPUT
 *
 * @note - Only the bits selected in bitValue are affected.
 * @note - If portNum, byteNum, or dir are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Sets the specified output pins to high for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to configure:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    bitValue    Bitmask specifying which pins to set high (0x0 to 0xFF).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or byteNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_ByteSetValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOSET[byteNum] = bitValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Clears the specified output pins to low for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to configure:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    bitValue    Bitmask specifying which pins to set low (0x0 to 0xFF).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or byteNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_ByteClearValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOCLR[byteNum] = bitValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Writes a value to all pins of the specified FIO port byte.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to write:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    newValue    Value to be written to the FIO byte register (0x0 to 0xFF).
 *                           Each bit corresponds to a pin (1 = high, 0 = low).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins masked in the FIOMASK register will not be affected by this operation.
 * @note - If portNum or byteNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_ByteWriteValue(uint8_t portNum, uint8_t byteNum, uint8_t newValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOPIN[byteNum] = newValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Reads the current state of all pins on the specified FIO port byte.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to read:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 *
 * @note - The returned value contains the logic state of each pin (bit) in the selected byte,
 *         regardless of whether the pin is configured as input or output.
 * @note - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 * @note - If portNum or byteNum are invalid, the function returns 0.
 *
 * @return       8-bit value representing the current state of all pins in the selected byte.
 **********************************************************************/
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

/*********************************************************************//**
 * @brief        Toggles the state of specified pins for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to configure:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    bitValue    Bitmask indicating which pins to toggle (0x0 to 0xFF).
 *
 * @note - Only pins configured as output are affected; input pins are not changed.
 * @note - Pins not selected in bitValue remain unchanged.
 * @note - If portNum or byteNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
void FIO_ByteTogglePins(uint8_t portNum, uint8_t byteNum, uint8_t bitValue)
{
    GPIO_Byte_TypeDef *pFIO = FIO_ByteGetPointer(portNum);
    if (pFIO != NULL) {
        if (byteNum <= 3){
            pFIO->FIOPIN[byteNum] ^= bitValue;
        }
    }
}

/*********************************************************************//**
 * @brief        Sets or clears the mask for selected bits in a FIO port byte.
 *
 * @param[in]    portNum     GPIO_PORT_x, where x is in the range [0,4].
 * @param[in]    byteNum     Byte part to configure:
 *                           - BYTE0 : bits 0-7.
 *                           - BYTE1 : bits 8-15.
 *                           - BYTE2 : bits 16-23.
 *                           - BYTE3 : bits 24-31.
 * @param[in]    bitValue    Bitmask indicating which bits to mask or unmask (0x0 to 0xFF).
 * @param[in]    newState    FunctionalState value:
 *                           - ENABLE: Mask the selected bits (access disabled).
 *                           - DISABLE: Unmask the selected bits (access enabled).
 *
 * @note - Only the bits selected in bitValue are affected.
 * @note - After masking, read/write operations to masked bits will have no effect or return 0.
 * @note - If portNum or byteNum are invalid, the function has no effect.
 *
 * @return       None
 **********************************************************************/
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
