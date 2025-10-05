/**
 * @file        lpc17xx_gpio.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for GPIO firmware library on LPC17xx
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
 *
 * @par Refactor:
 * Date: 07/07/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup GPIO GPIO
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_GPIO_H_
#define LPC17XX_GPIO_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup GPIO_Private_Macros GPIO Private Macros
 * @{
 */

/* -------------------- MACROS BYTE POINTER DEFINITIONS --------------------- */
/** Fast GPIO port 0 byte accessible definition. */
#define GPIO0_Byte ((GPIO_Byte_TypeDef*)(LPC_GPIO0_BASE))
/** Fast GPIO port 1 byte accessible definition. */
#define GPIO1_Byte ((GPIO_Byte_TypeDef*)(LPC_GPIO1_BASE))
/** Fast GPIO port 2 byte accessible definition. */
#define GPIO2_Byte ((GPIO_Byte_TypeDef*)(LPC_GPIO2_BASE))
/** Fast GPIO port 3 byte accessible definition. */
#define GPIO3_Byte ((GPIO_Byte_TypeDef*)(LPC_GPIO3_BASE))
/** Fast GPIO port 4 byte accessible definition. */
#define GPIO4_Byte ((GPIO_Byte_TypeDef*)(LPC_GPIO4_BASE))

/* ------------------ MACROS HALF-WORD POINTER DEFINITIONS ------------------ */
/** Fast GPIO port 0 half-word accessible definition. */
#define GPIO0_HalfWord ((GPIO_HalfWord_TypeDef*)(LPC_GPIO0_BASE))
/** Fast GPIO port 1 half-word accessible definition. */
#define GPIO1_HalfWord ((GPIO_HalfWord_TypeDef*)(LPC_GPIO1_BASE))
/** Fast GPIO port 2 half-word accessible definition. */
#define GPIO2_HalfWord ((GPIO_HalfWord_TypeDef*)(LPC_GPIO2_BASE))
/** Fast GPIO port 3 half-word accessible definition. */
#define GPIO3_HalfWord ((GPIO_HalfWord_TypeDef*)(LPC_GPIO3_BASE))
/** Fast GPIO port 4 half-word accessible definition. */
#define GPIO4_HalfWord ((GPIO_HalfWord_TypeDef*)(LPC_GPIO4_BASE))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup GPIO_Public_Types GPIO Public Types
 * @{
 */

/**
 * @brief Port selection for GPIO pins.
 */
typedef enum {
    GPIO_PORT_0 = 0,
    GPIO_PORT_1,
    GPIO_PORT_2,
    GPIO_PORT_3,
    GPIO_PORT_4
} GPIO_PORT;
/** Check GPIO port option parameter. */
#define PARAM_GPIO_PORT(PORT)     ((PORT) >= GPIO_PORT_0 && (PORT) <= GPIO_PORT_4)
/** Check GPIO port option parameter for interrupt. */
#define PARAM_GPIO_INT_PORT(PORT) ((PORT) == GPIO_PORT_0 || (PORT) == GPIO_PORT_2)

/**
 * @brief Half-word access for GPIO ports.
 */
typedef enum {
    GPIO_HALFWORD_LOW = 0,
    GPIO_HALFWORD_HIGH
} GPIO_HALFWORD;
/** Check GPIO half-word option parameter. */
#define PARAM_GPIO_HALFWORD(HW) ((HW) == GPIO_HALFWORD_LOW || (HW) == GPIO_HALFWORD_HIGH)

/**
 * @brief Byte access for GPIO ports.
 */
typedef enum {
    GPIO_BYTE_0 = 0,
    GPIO_BYTE_1,
    GPIO_BYTE_2,
    GPIO_BYTE_3
} GPIO_BYTE;
/** Check GPIO byte option parameter. */
#define PARAM_GPIO_BYTE(BYTE) ((BYTE) >= GPIO_BYTE_0 && (BYTE) <= GPIO_BYTE_3)

/**
 * @brief Direction selection for GPIO pins.
 */
typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT
} GPIO_DIR;
/** Check GPIO direction option parameter. */
#define PARAM_GPIO_DIR(DIR) ((DIR) == GPIO_INPUT || (DIR) == GPIO_OUTPUT)

/**
 * @brief Edge selection for GPIO interrupts.
 */
typedef enum {
    GPIO_INT_RISING = 0,
    GPIO_INT_FALLING
} GPIO_INT_EDGE;
/** Check GPIO interrupt edge option parameter. */
#define PARAM_GPIO_INT_EDGE(EDGE) ((EDGE) == GPIO_INT_RISING || (EDGE) == GPIO_INT_FALLING)

/**
 * @brief Fast GPIO port byte type definition.
 */
typedef struct {
    __IO uint8_t FIODIR[4];  /**< FIO direction register in byte-align. */
    uint32_t RESERVED0[3];   /**< Reserved. */
    __IO uint8_t FIOMASK[4]; /**< FIO mask register in byte-align. */
    __IO uint8_t FIOPIN[4];  /**< FIO pin register in byte align. */
    __IO uint8_t FIOSET[4];  /**< FIO set register in byte-align. */
    __O uint8_t FIOCLR[4];   /**< FIO clear register in byte-align. */
} GPIO_Byte_TypeDef;

/**
 * @brief Fast GPIO port half-word type definition
 */
typedef struct {
    __IO uint16_t FIODIRL;  /**< FIO direction register lower halfword part. */
    __IO uint16_t FIODIRU;  /**< FIO direction register upper halfword part. */
    uint32_t RESERVED0[3];  /**< Reserved. */
    __IO uint16_t FIOMASKL; /**< FIO mask register lower halfword part. */
    __IO uint16_t FIOMASKU; /**< FIO mask register upper halfword part. */
    __IO uint16_t FIOPINL;  /**< FIO pin register lower halfword part. */
    __IO uint16_t FIOPINU;  /**< FIO pin register upper halfword part. */
    __IO uint16_t FIOSETL;  /**< FIO set register lower halfword part. */
    __IO uint16_t FIOSETU;  /**< FIO set register upper halfword part. */
    __O uint16_t FIOCLRL;   /**< FIO clear register lower halfword part. */
    __O uint16_t FIOCLRU;   /**< FIO clear register upper halfword part. */
} GPIO_HalfWord_TypeDef;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup GPIO_Public_Functions GPIO Public Functions
 * @{
 */

/* ------------------------------- GPIO style ------------------------------- */
/**
 * @brief        Sets the direction for the specified GPIO port pins.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    pins       Bitmask of pins to configure (0x0 to 0xFFFFFFFF).
 *                          Example: value 0x5 to set direction for bit 0 and bit 2.
 * @param[in]    dir        Must be:
 *                          - GPIO_INPUT : Input direction.
 *                          - GPIO_OUTPUT : Output direction.
 *
 * @note:
 * - Pins not selected in pins are not affected.
 */
void GPIO_SetDir(GPIO_PORT portNum, uint32_t pins, GPIO_DIR dir);

/**
 * @brief       Sets the specified output pins to high on a given GPIO port.
 *
 * @param[in]   portNum GPIO_PORT_x [0...4].
 * @param[in]   pins    Bitmask specifying which pins to set high (1 = set).
 *                      Example: 0x5 sets pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void GPIO_SetPins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief       Clears the specified output pins to low on a given GPIO port.
 *
 * @param[in]   portNum GPIO_PORT_x [0...4].
 * @param[in]   pins    Bitmask specifying which pins to set low (1 = clear).
 *                      Example: 0x5 clears pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void GPIO_ClearPins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief       Writes a value to all pins of the specified GPIO port.
 *
 * @param[in]   portNum     GPIO_PORT_x [0...4].
 * @param[in]   newValue    Value to be written to the FIOPIN register.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void GPIO_WriteValue(GPIO_PORT portNum, uint32_t newValue);

/**
 * @brief       Reads the current state of all pins on the specified GPIO port.
 *
 * @param[in]   portNum     GPIO_PORT_x [0...4].
 *
 * @note:
 * - The returned value contains the logic state of each pin (bit) on the port,
 *   regardless of whether the pin is configured as input or output.
 * - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 *
 * @return      32-bit value representing the current state of all port pins.
 */
uint32_t GPIO_ReadValue(GPIO_PORT portNum);

/**
 * @brief       Toggles the state of specified pins on the given GPIO port.
 *
 * @param[in]   portNum GPIO_PORT_x [0...4].
 * @param[in]   pins    Bitmask specifying which pins to toggle (1 = toggle).
 *                      Example: 0x5 toggles pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void GPIO_TogglePins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief       Sets or clears the mask for specified pins on the given GPIO port.
 *
 * @param[in]   portNum   GPIO_PORT_x [0...4].
 * @param[in]   pins      Bitmask specifying which pins to mask or unmask (1 = select).
 *                        Example: 0x5 selects pins 0 and 2.
 * @param[in]   newState  FunctionalState value:
 *                        - ENABLE: Mask the selected pins (access disabled).
 *                        - DISABLE: Unmask the selected pins (access enabled).
 *
 * @note:
 * - Only the pins selected in pins are affected.
 * - After masking, write/read operations to masked pins will have no effect.
 */
void GPIO_SetMask(GPIO_PORT portNum, uint32_t pins, FunctionalState newState);

/**
 * @brief       Sets the interrupt enable mask for GPIO pins on the given port.
 *
 * @param[in]   portNum     GPIO_PORT_x [0, 2].
 * @param[in]   newValue    Bitmask written directly to the interrupt enable register.
 *                          Each bit: 1 = enable interrupt, 0 = disable interrupt.
 *                          Example: 0x5 enables interrupt for pins 0 and 2, disables others.
 * @param[in]   edgeState   Interrupt edge selection:
 *                          - GPIO_INT_RISING:  Rising edge interrupt.
 *                          - GPIO_INT_FALLING: Falling edge interrupt.
 *
 * @note:
 * - The entire interrupt enable register is overwritten; all pins not set in
 *   newValue will have their interrupts disabled.
 * - Only pins P0.0-P0.11, P0.15-P0.30, and P2.0-P2.13 support interrupts.
 */
void GPIO_IntCmd(GPIO_PORT portNum, uint32_t newValue, GPIO_INT_EDGE edgeState);

/**
 * @brief       Gets the interrupt status for the entire GPIO port.
 *
 * @param[in]   portNum GPIO_PORT_x [0, 2].
 *
 * @note:
 * - Only port 0 and port 2 support interrupts.
 *
 * @return      ENABLE if any interrupt is pending on the selected port,
 *              DISABLE otherwise.
 */
FunctionalState GPIO_GetPortIntStatus(GPIO_PORT portNum);

/**
 * @brief       Gets the interrupt status for a specific GPIO pin and edge.
 *
 * @param[in]   portNum     GPIO_PORT_x [0, 2].
 * @param[in]   pinNum      Only pins 0-11, 15-30 for port 0 and 0-13 for port 2 support interrupts.
 * @param[in]   edgeState   Interrupt edge selection:
 *                          - GPIO_INT_RISING:  Rising edge interrupt status.
 *                          - GPIO_INT_FALLING: Falling edge interrupt status.
 *
 * @return      ENABLE if an interrupt has been generated for the selected pin and edge,
 *              DISABLE otherwise.
 */
FunctionalState GPIO_GetPinIntStatus(GPIO_PORT portNum, uint32_t pinNum, GPIO_INT_EDGE edgeState);

/**
 * @brief       Clears the interrupt status for selected GPIO pins.
 *
 * @param[in]   portNum    GPIO_PORT_x [0, 2].
 * @param[in]   pins       Bitmask specifying which pins to clear interrupt status.
 *                         Example: 0x5 clears interrupt for pins 0 and 2.
 *
 * @note - Only pins P0.0-P0.11, P0.15-P0.30, and P2.0-P2.13 support interrupts.
*/
void GPIO_ClearInt(GPIO_PORT portNum, uint32_t pins);

/* ---------------------- FIO (word-accessible) style ----------------------- */
/**
 * @brief The same as GPIO_SetDir().
 */
void FIO_SetDir(GPIO_PORT portNum, uint32_t pins, GPIO_DIR dir);

/**
 * @brief The same as GPIO_SetPins().
 */
void FIO_SetPins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief The same as GPIO_ClearPins().
 */
void FIO_ClearPins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief The same as GPIO_WriteValue().
 */
void FIO_WriteValue(GPIO_PORT portNum, uint32_t newValue);

/**
 * @brief The same with GPIO_ReadValue().
 */
uint32_t FIO_ReadValue(GPIO_PORT portNum);

/**
 * @brief The same with GPIO_TogglePins().
 */
void FIO_TogglePins(GPIO_PORT portNum, uint32_t pins);

/**
 * @brief The same with GPIO_SetMask().
 */
void FIO_SetMask(GPIO_PORT portNum, uint32_t pins, FunctionalState newState);

/**
 * @brief The same with GPIO_IntCmd().
 */
void FIO_IntCmd(GPIO_PORT portNum, uint32_t newValue, GPIO_INT_EDGE edgeState);

/**
 * @brief The same with GPIO_GetPortIntStatus().
 */
FunctionalState FIO_GetPortIntStatus(GPIO_PORT portNum);

/**
 * @brief The same with GPIO_GetPinIntStatus().
 */
FunctionalState FIO_GetPinIntStatus(GPIO_PORT portNum, uint32_t pinNum, GPIO_INT_EDGE edgeState);

/**
 * @brief The same with GPIO_ClearInt().
 */
void FIO_ClearInt(GPIO_PORT portNum, uint32_t pins);

/* -------------------- FIO (halfword-accessible) style --------------------- */
/**
 * @brief        Sets the direction of specified pins for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    pins           Bitmask indicating which pins to configure (0x0 to 0xFFFF).
 *                              Example: 0x0005 configures bits 0 and 2.
 * @param[in]    dir            Must be:
 *                              - GPIO_INPUT : Input direction.
 *                              - GPIO_OUTPUT : Output direction.
 *
 * @note - Pins not selected in pins are not affected.
 */
void FIO_HalfWordSetDir(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins, GPIO_DIR dir);

/**
 * @brief        Sets the specified output pins to high for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    pins           Bitmask indicating which pins to set high (0x0 to 0xFFFF).
 *                              Example: 0x0005 sets pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_HalfWordSetPins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins);

/**
 * @brief        Clears the specified output pins to low for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    pins           Bitmask indicating which pins to set low (0x0 to 0xFFFF).
 *                              Example: 0x0005 clears pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_HalfWordClearPins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins);

/**
 * @brief        Writes a value to all pins of the specified FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to write:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    newValue       Value to be written to the FIO halfword register (0x0 to 0xFFFF).
 *                              Each bit corresponds to a pin (1 = high, 0 = low).
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void FIO_HalfWordWriteValue(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t newValue);

/**
 * @brief        Reads the current state of all pins on the specified FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to read:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 *
 * @note:
 * - The returned value contains the logic state of each pin (bit) in the selected halfword,
 *   regardless of whether the pin is configured as input or output.
 * - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 *
 * @return       16-bit value representing the current state of all pins in the selected halfword.
 */
uint16_t FIO_HalfWordReadValue(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum);

/**
 * @brief        Toggles the state of specified pins for a FIO port in halfword-accessible mode.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    pins           Bitmask indicating which pins to toggle (0x0 to 0xFFFF).
 *                              Example: 0x0005 toggles pins 0 and 2.
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_HalfWordTogglePins(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins);

/**
 * @brief        Sets or clears the mask for selected bits in a FIO port halfword.
 *
 * @param[in]    portNum        GPIO_PORT_x [0...4].
 * @param[in]    halfwordNum    Halfword part to configure:
 *                              - GPIO_HALFWORD_LOW : bits 0-15.
 *                              - GPIO_HALFWORD_HIGH : bits 16-31.
 * @param[in]    pins           Bitmask indicating which bits to mask or unmask (0x0 to 0xFFFF).
 * @param[in]    newState       FunctionalState value:
 *                              - ENABLE: Mask the selected bits (access disabled).
 *                              - DISABLE: Unmask the selected bits (access enabled).
 *
 * @note:
 * - Only the bits selected in pins are affected.
 * - After masking, read/write operations to masked bits will have no effect or return 0.
*/
void FIO_HalfWordSetMask(GPIO_PORT portNum, GPIO_HALFWORD halfwordNum, uint16_t pins, FunctionalState newState);

/* ---------------------- FIO (byte-accessible) style ----------------------- */
/**
 * @brief        Sets the direction for specified pins in a FIO port byte.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 * @param[in]    pins       Bitmask indicating which bits to configure (0x0 to 0xFF).
 * @param[in]    dir        Must be:
 *                              - GPIO_INPUT : Input direction.
 *                              - GPIO_OUTPUT : Output direction.
 *
 * @note:
 * - Only the bits selected in pins are affected.
 */
void FIO_ByteSetDir(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins, GPIO_DIR dir);

/**
 * @brief        Sets the specified output pins to high for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 * @param[in]    pins       Bitmask specifying which pins to set high (0x0 to 0xFF).
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_ByteSetPins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins);

/**
 * @brief        Clears the specified output pins to low for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 * @param[in]    pins       Bitmask specifying which pins to set low (0x0 to 0xFF).
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_ByteClearPins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins);

/**
 * @brief        Writes a value to all pins of the specified FIO port byte.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 * @param[in]    newValue   Value to be written to the FIO byte register (0x0 to 0xFF).
 *                          Each bit corresponds to a pin (1 = high, 0 = low).
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins masked in the FIOMASK register will not be affected by this operation.
 */
void FIO_ByteWriteValue(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t newValue);

/**
 * @brief        Reads the current state of all pins on the specified FIO port byte.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 *
 * @note:
 * - The returned value contains the logic state of each pin (bit) in the selected byte,
 *   regardless of whether the pin is configured as input or output.
 * - Pins masked in the FIOMASK register will return 0 in the corresponding bits.
 *
 * @return       8-bit value representing the current state of all pins in the selected byte.
 */
uint8_t FIO_ByteReadValue(GPIO_PORT portNum, GPIO_BYTE byteNum);

/**
 * @brief        Toggles the state of specified pins for a FIO port in byte-accessible mode.
 *
 * @param[in]    portNum    GPIO_PORT_x [0...4].
 * @param[in]    byteNum    GPIO_BYTE_x [0...3].
 * @param[in]    pins       Bitmask indicating which pins to toggle (0x0 to 0xFF).
 *
 * @note:
 * - Only pins configured as output are affected; input pins are not changed.
 * - Pins not selected in pins remain unchanged.
 */
void FIO_ByteTogglePins(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins);

/**
 * @brief        Sets or clears the mask for selected bits in a FIO port byte.
 *
 * @param[in]    portNum     GPIO_PORT_x [0...4].
 * @param[in]    byteNum     GPIO_BYTE_x [0...3].
 * @param[in]    pins        Bitmask indicating which bits to mask or unmask (0x0 to 0xFF).
 * @param[in]    newState    FunctionalState value:
 *                           - ENABLE: Mask the selected bits (access disabled).
 *                           - DISABLE: Unmask the selected bits (access enabled).
 *
 * @note:
 * - Only the bits selected in pins are affected.
 * - After masking, read/write operations to masked bits will have no effect or return 0.
 */
void FIO_ByteSetMask(GPIO_PORT portNum, GPIO_BYTE byteNum, uint8_t pins, FunctionalState newState);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  // LPC17XX_GPIO_H_

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
