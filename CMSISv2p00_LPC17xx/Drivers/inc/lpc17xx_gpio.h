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
void GPIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir);
void GPIO_SetValue(uint8_t portNum, uint32_t bitValue);
void GPIO_ClearValue(uint8_t portNum, uint32_t bitValue);
void GPIO_WriteValue(uint8_t portNum, uint32_t newValue);
uint32_t GPIO_ReadValue(uint8_t portNum);
void GPIO_TogglePins(uint8_t portNum, uint32_t bitValue);
void GPIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState);
void GPIO_IntCmd(uint8_t portNum, uint32_t newValue, uint8_t edgeState);
FunctionalState GPIO_GetPortIntStatus(uint8_t portNum);
FunctionalState GPIO_GetIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState);
void GPIO_ClearInt(uint8_t portNum, uint32_t bitValue);

/* FIO (word-accessible) style ----------------------------------------- */
void FIO_SetDir(uint8_t portNum, uint32_t bitValue, uint8_t dir);
void FIO_SetValue(uint8_t portNum, uint32_t bitValue);
void FIO_ClearValue(uint8_t portNum, uint32_t bitValue);
void FIO_WriteValue(uint8_t portNum, uint32_t newValue);
uint32_t FIO_ReadValue(uint8_t portNum);
void FIO_TogglePins(uint8_t portNum, uint32_t bitValue);
void FIO_SetMask(uint8_t portNum, uint32_t bitValue, FunctionalState newState);
void FIO_IntCmd(uint8_t portNum, uint32_t newValue, uint8_t edgeState);
FunctionalState FIO_GetIntStatus(uint8_t portNum, uint32_t pinNum, uint8_t edgeState);
void FIO_ClearInt(uint8_t portNum, uint32_t pinNum);

/* FIO (halfword-accessible) style ------------------------------------- */
void FIO_HalfWordSetDir(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, uint8_t dir);
void FIO_HalfWordSetValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);
void FIO_HalfWordClearValue(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);
void FIO_HalfWordWriteValue(uint8_t portNum, uint8_t halfwordNum, uint16_t newValue);
uint16_t FIO_HalfWordReadValue(uint8_t portNum, uint8_t halfwordNum);
void FIO_HalfWordTogglePins(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue);
void FIO_HalfWordSetMask(uint8_t portNum, uint8_t halfwordNum, uint16_t bitValue, FunctionalState newState);

/* FIO (byte-accessible) style ----------------------------------------- */
void FIO_ByteSetDir(uint8_t portNum, uint8_t byteNum, uint8_t bitValue, uint8_t dir);
void FIO_ByteSetValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);
void FIO_ByteClearValue(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);
void FIO_ByteWriteValue(uint8_t portNum, uint8_t byteNum, uint8_t newValue);
uint8_t FIO_ByteReadValue(uint8_t portNum, uint8_t byteNum);
void FIO_ByteTogglePins(uint8_t portNum, uint8_t byteNum, uint8_t bitValue);
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
