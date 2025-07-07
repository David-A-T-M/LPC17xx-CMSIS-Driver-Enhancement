/***********************************************************************//**
 * @file        lpc17xx_gpio.h
 * @brief        Contains all macro definitions and function prototypes
 *                 support for GPIO firmware library on LPC17xx
 * @version        3.0
 * @date        18. June. 2010
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
 **************************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup GPIO GPIO
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_GPIO_H_
#define LPC17XX_GPIO_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* Public Macros -------------------------------------------------------------- */
/** @defgroup GPIO_Public_Macros GPIO Public Macros
 * @{
 */

/** Fast GPIO port 0 byte accessible definition.*/
#define GPIO0_Byte    ((GPIO_Byte_TypeDef *)(LPC_GPIO0_BASE))
/** Fast GPIO port 1 byte accessible definition.*/
#define GPIO1_Byte    ((GPIO_Byte_TypeDef *)(LPC_GPIO1_BASE))
/** Fast GPIO port 2 byte accessible definition.*/
#define GPIO2_Byte    ((GPIO_Byte_TypeDef *)(LPC_GPIO2_BASE))
/** Fast GPIO port 3 byte accessible definition.*/
#define GPIO3_Byte    ((GPIO_Byte_TypeDef *)(LPC_GPIO3_BASE))
/** Fast GPIO port 4 byte accessible definition.*/
#define GPIO4_Byte    ((GPIO_Byte_TypeDef *)(LPC_GPIO4_BASE))


/** Fast GPIO port 0 half-word accessible definition.*/
#define GPIO0_HalfWord    ((GPIO_HalfWord_TypeDef *)(LPC_GPIO0_BASE))
/** Fast GPIO port 1 half-word accessible definition.*/
#define GPIO1_HalfWord    ((GPIO_HalfWord_TypeDef *)(LPC_GPIO1_BASE))
/** Fast GPIO port 2 half-word accessible definition.*/
#define GPIO2_HalfWord    ((GPIO_HalfWord_TypeDef *)(LPC_GPIO2_BASE))
/** Fast GPIO port 3 half-word accessible definition.*/
#define GPIO3_HalfWord    ((GPIO_HalfWord_TypeDef *)(LPC_GPIO3_BASE))
/** Fast GPIO port 4 half-word accessible definition.*/
#define GPIO4_HalfWord    ((GPIO_HalfWord_TypeDef *)(LPC_GPIO4_BASE))

/*********************************************************************/ /**
 *!< Macros define for port selection.
 ***********************************************************************/
#define GPIO_PORT_0         ((0))   /**< Port 0.*/
#define GPIO_PORT_1         ((1))   /**< Port 1.*/
#define GPIO_PORT_2         ((2))   /**< Port 2.*/
#define GPIO_PORT_3         ((3))   /**< Port 3.*/
#define GPIO_PORT_4         ((4))   /**< Port 4.*/

/*********************************************************************/ /**
 *!< Macros define for direction configuration.
 ***********************************************************************/
#define INPUT               ((0))   /**< Input direction.*/
#define OUTPUT              ((1))   /**< Output direction.*/

/*********************************************************************/ /**
 *!< Macros define for interrupt edge configuration.
 ***********************************************************************/
#define GPIO_INT_RISING     ((0))   /**< Rising edge interrupt.*/
#define GPIO_INT_FALLING    ((1))   /**< Falling edge interrupt.*/

/*********************************************************************/ /**
 *!< Macros define for halfword port access.
 ***********************************************************************/
#define LOW_HALFWORD        ((0))   /**< Lower halfword access.*/
#define HIGH_HALFWORD       ((1))   /**< Upper halfword access.*/

/*********************************************************************/ /**
 *!< Macros define for byte port access.
 ***********************************************************************/
#define BYTE0               ((0))   /**< Byte 0 access (bits 0-7).*/
#define BYTE1               ((1))   /**< Byte 1 access (bits 8-15).*/
#define BYTE2               ((2))   /**< Byte 2 access (bits 16-23).*/
#define BYTE3               ((3))   /**< Byte 3 access (bits 24-31).*/

/**
 * @}
 */

/* Public Types --------------------------------------------------------------- */
/** @defgroup GPIO_Public_Types GPIO Public Types
 * @{
 */

/**
 * @brief Fast GPIO port byte type definition.
 */
typedef struct {
    __IO uint8_t FIODIR[4];     /**< FIO direction register in byte-align.*/
        uint32_t RESERVED0[3];  /**< Reserved.*/
    __IO uint8_t FIOMASK[4];    /**< FIO mask register in byte-align.*/
    __IO uint8_t FIOPIN[4];     /**< FIO pin register in byte align.*/
    __IO uint8_t FIOSET[4];     /**< FIO set register in byte-align.*/
    __O  uint8_t FIOCLR[4];     /**< FIO clear register in byte-align.*/
} GPIO_Byte_TypeDef;

/**
 * @brief Fast GPIO port half-word type definition
 */
typedef struct {
    __IO uint16_t FIODIRL;      /**< FIO direction register lower halfword part.*/
    __IO uint16_t FIODIRU;      /**< FIO direction register upper halfword part.*/
         uint32_t RESERVED0[3]; /**< Reserved.*/
    __IO uint16_t FIOMASKL;     /**< FIO mask register lower halfword part.*/
    __IO uint16_t FIOMASKU;     /**< FIO mask register upper halfword part.*/
    __IO uint16_t FIOPINL;      /**< FIO pin register lower halfword part.*/
    __IO uint16_t FIOPINU;      /**< FIO pin register upper halfword part.*/
    __IO uint16_t FIOSETL;      /**< FIO set register lower halfword part.*/
    __IO uint16_t FIOSETU;      /**< FIO set register upper halfword part.*/
    __O  uint16_t FIOCLRL;      /**< FIO clear register lower halfword part.*/
    __O  uint16_t FIOCLRU;      /**< FIO clear register upper halfword part.*/
} GPIO_HalfWord_TypeDef;

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup GPIO_Public_Functions GPIO Public Functions
 * @{
 */

/* GPIO style ---------------------------------------------------------- */
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
 **********************************************************************/
void GPIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir);

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
 **********************************************************************/
void GPIO_SetValue(uint8_t portNum, uint32_t bitValue);

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
 **********************************************************************/
void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue);

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
 **********************************************************************/
void GPIO_WriteValue(uint8_t portNum, uint32_t newValue);

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
uint32_t GPIO_ReadValue(uint8_t portNum);

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
 **********************************************************************/
void GPIO_TogglePins(uint8_t portNum, uint32_t bitValue);

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
 **********************************************************************/
void GPIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState);

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
 **********************************************************************/
void GPIO_IntCmd(uint8_t portNum, uint32_t newValue, uint8_t edgeState);

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
FunctionalState GPIO_GetPortIntStatus(uint8_t portNum);

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
FunctionalState GPIO_GetPinIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState);

/*********************************************************************//**
 * @brief       Clears the interrupt status for selected GPIO pins.
 *
 * @param[in]   portNum    GPIO_PORT_x, must be 0 or 2.
 * @param[in]   bitValue   Bitmask specifying which pins to clear interrupt status.
 *                         Example: 0x5 clears interrupt for pins 0 and 2.
 *
 * @note - Only pins P0.0-P0.11, P0.15-P0.30, and P2.0-P2.13 support interrupts.
 * @note - If portNum is not 0 or 2, the function has no effect.
 **********************************************************************/
void GPIO_ClearInt(uint8_t portNum, uint32_t bitValue);

/* FIO (word-accessible) style ----------------------------------------- */
/**
 * @brief The same as GPIO_SetDir().
 */
void FIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir);

/**
 * @brief The same as GPIO_SetValue()
 */
void FIO_SetValue(uint8_t portNum, uint32_t bitValue);

/**
 * @brief The same as GPIO_ClearValue()
 */
void FIO_ClearValue(uint8_t portNum, uint32_t bitValue);

/**
 * @brief The same as GPIO_WriteValue()
 */
void FIO_WriteValue(uint8_t portNum, uint32_t newValue);

/**
 * @brief The same with GPIO_ReadValue()
 */
uint32_t FIO_ReadValue(uint8_t portNum);

/**
 * @brief The same with GPIO_TogglePins()
 */
void FIO_TogglePins(uint8_t portNum, uint32_t bitValue);

/**
 * @brief The same with GPIO_SetMask()
 */
void FIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState);

/**
 * @brief The same with GPIO_IntCmd()
 */
void FIO_IntCmd(uint8_t portNum, uint32_t newValue, uint8_t edgeState);

/**
 * @brief The same with GPIO_GetPortIntStatus()
 */
FunctionalState FIO_GetPortIntStatus(uint8_t portNum);

/**
 * @brief The same with GPIO_GetPinIntStatus()
 */
FunctionalState FIO_GetPinIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState);

/**
 * @brief The same with GPIO_ClearInt()
 */
void FIO_ClearInt(uint8_t portNum, uint32_t pinNum);

/* FIO (halfword-accessible) style ------------------------------------- */
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
 **********************************************************************/
void FIO_HalfWordSetDir(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, uint8_t dir);

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
 **********************************************************************/
void FIO_HalfWordSetValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);

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
 **********************************************************************/
void FIO_HalfWordClearValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);

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
 **********************************************************************/
void FIO_HalfWordWriteValue(uint8_t portNum, uint8_t halfwordNum, uint16_t newValue);

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
uint16_t FIO_HalfWordReadValue(uint8_t portNum, uint8_t halfwordNum);

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
 **********************************************************************/
void FIO_HalfWordTogglePins(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);

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
 **********************************************************************/
void FIO_HalfWordSetMask(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, FunctionalState newState);

/* FIO (byte-accessible) style ----------------------------------------- */
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
 **********************************************************************/
void FIO_ByteSetDir(uint8_t portNum, uint8_t byteNum, uint8_t bitValue, uint8_t dir);

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
 **********************************************************************/
void FIO_ByteSetValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);

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
 **********************************************************************/
void FIO_ByteClearValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);

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
 **********************************************************************/
void FIO_ByteWriteValue(uint8_t portNum, uint8_t byteNum, uint8_t newValue);

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
uint8_t FIO_ByteReadValue(uint8_t portNum, uint8_t byteNum);

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
 **********************************************************************/
void FIO_ByteTogglePins(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);

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
 **********************************************************************/
void FIO_ByteSetMask(uint8_t portNum, uint8_t byteNum, uint8_t bitValue, FunctionalState newState);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_GPIO_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
