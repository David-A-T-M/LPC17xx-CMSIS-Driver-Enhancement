/**
 * @file        lpc17xx_uartx.h
 * @brief       Contains all macro definitions and function prototypes
 *              support for UART firmware library on LPC17xx
 * @version     1.0
 * @date        02. November. 2025
 * @author      David Trujillo Medina
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

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup UARTX UARTX
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_UARTX_H
#define LPC17XX_UARTX_H

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup UARTX_Private_Macros UART Private Macros
 * @{
 */

/** UARTX time-out definitions for functions with Blocking Flag mode. */
#define UARTX_BLOCKING_TIMEOUT (0xFFFFFFFFUL)

/** Accepted Error baud rate value (in percent unit). */
#define UARTX_ACCEPTED_BAUDRATE_ERROR (3)

/** Macro for loading least significant halfs of divisors. */
#define UARTX_LOAD_DLL(div) ((div) & 0xFF)
/** Macro for loading most significant halfs of divisors. */
#define UARTX_LOAD_DLM(div) (((div) >> 8) & 0xFF)

/** Baud-rate generation pre-scaler divisor. */
#define UARTX_FDR_DIVADDVAL(n) ((uint32_t)(n & 0x0F))
/** Baud-rate pre-scaler multiplier value. */
#define UARTX_FDR_MULVAL(n)    ((uint32_t)((n << 4) & 0xF0))
/** UART Fractional Divider register bit mask. */
#define UARTX_FDR_BITMASK      ((uint32_t)(0xFF))

/* ---------------------------- BIT DEFINITIONS ----------------------------- */
/** UARTX Received Buffer mask bit (8 bits). */
#define UARTX_RBR_MASKBIT ((uint8_t)0xFF)
/** UARTX Transmit Holding mask bit (8 bits). */
#define UARTX_THR_MASKBIT ((uint8_t)0xFF)

/** UARTX FIFO enable. */
#define UARTX_FCR_FIFO_EN     ((1 << 0))
/** UARTX FIFO RX reset. */
#define UARTX_FCR_RX_RS       ((1 << 1))
/** UARTX FIFO TX reset. */
#define UARTX_FCR_TX_RS       ((1 << 2))
/** UARTX DMA mode selection. */
#define UARTX_FCR_DMAMODE_SEL ((1 << 3))
/** UARTX FIFO control bit mask */
#define UARTX_FCR_BITMASK     ((0xCF))
/** UARTX FIFO size. */
#define UARTX_TX_FIFO_SIZE    (16)

/** Line status register: Receive data ready. */
#define UARTX_LSR_RDR     ((1 << 0))
/** Line status register: Transmit holding register empty. */
#define UARTX_LSR_THRE    ((1 << 5))
/** Line status register: Transmitter empty. */
#define UARTX_LSR_TEMT    ((1 << 6))
/** UARTX Line status bit mask. */
#define UARTX_LSR_BITMASK ((0xFF))

/** Transmit enable bit. */
#define UARTX_TER_TXEN    ((1 << 7))
/** UARTX Transmit Enable Register bit mask. */
#define UARTX_TER_BITMASK ((0x80))

/** UARTX Two Stop Bits Select. */
#define UARTX_LCR_STOPBIT_1   ((0 << 2))
/** UARTX Two Stop Bits Select. */
#define UARTX_LCR_STOPBIT_2   ((1 << 2))
/** UARTX Parity Enable. */
#define UARTX_LCR_PARITY_EN   ((1 << 3))
/** UARTX Odd Parity Select. */
#define UARTX_LCR_PARITY_ODD  ((0 << 4))
/** UARTX Even Parity Select. */
#define UARTX_LCR_PARITY_EVEN ((1 << 4))
/** UARTX force 1 stick parity. */
#define UARTX_LCR_PARITY_F_1  ((2 << 4))
/** UARTX force 0 stick parity. */
#define UARTX_LCR_PARITY_F_0  ((3 << 4))
/** UARTX Transmission Break enable. */
#define UARTX_LCR_BREAK_EN    ((1 << 6))
/** UARTX Divisor Latches Access bit enable. */
#define UARTX_LCR_DLAB_EN     ((1 << 7))
/** UART line control bit mask. */
#define UARTX_LCR_BITMASK     ((0xFF))

/** RBR Interrupt enable. */
#define UARTX_IER_RBRINT_EN  ((1 << 0))
/** THR Interrupt enable. */
#define UARTX_IER_THREINT_EN ((1 << 1))
/** RX line status interrupt enable. */
#define UARTX_IER_RLSINT_EN  ((1 << 2))
/** End of auto-baud interrupt enable. */
#define UARTX_IER_ABEOINT_EN ((1 << 8))
/** Auto-baud time-out interrupt enable. */
#define UARTX_IER_ABTOINT_EN ((1 << 9))
/** UARTX interrupt enable register bit mask */
#define UARTX_IER_BITMASK    ((0x307))

/** End of auto-baud interrupt. */
#define UARTX_IIR_ABEO_INT ((1 << 8))
/** Auto-baud time-out interrupt. */
#define UARTX_IIR_ABTO_INT ((1 << 9))

/** UARTX Auto-baud start. */
#define UARTX_ACR_START        ((1 << 0))
/** UARTX Auto baudrate Mode 1. */
#define UARTX_ACR_MODE         ((1 << 1))
/** UARTX Auto baudrate restart. */
#define UARTX_ACR_AUTO_RESTART ((1 << 2))

/** IrDA mode enable. */
#define UARTX_ICR_IRDAEN      ((1 << 0))
/** IrDA serial input inverted. */
#define UARTX_ICR_IRDAINV     ((1 << 1))
/** IrDA fixed pulse width mode. */
#define UARTX_ICR_FIXPULSE_EN ((1 << 2))
/** PulseDiv - Configures the pulse when FixPulseEn = 1 */
#define UARTX_ICR_PULSEDIV(n) ((uint32_t)((n & 0x07) << 3))
/** UARTX IRDA bit mask. */
#define UARTX_ICR_BITMASK     ((0x3F))

/* ---------------------- CHECK PARAMETER DEFINITIONS ----------------------- */
/** Check UARTX parameter. */
#define PARAM_UARTX(n)                                                                       \
    (((uintptr_t)(n) == (uintptr_t)LPC_UART0) || ((uintptr_t)(n) == (uintptr_t)LPC_UART2) || \
     ((uintptr_t)(n) == (uintptr_t)LPC_UART3))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup UARTX_Public_Types UARTX Public Types
 * @{
 */

/**
 * @brief UARTX Parity type definitions
 */
typedef enum {
    UARTX_PARITY_NONE = 0, /*!< No parity. */
    UARTX_PARITY_ODD,      /*!< Odd parity. */
    UARTX_PARITY_EVEN,     /*!< Even parity. */
    UARTX_PARITY_1,        /*!< Forced "1". */
    UARTX_PARITY_0         /*!< Forced "0". */
} UARTX_PARITY;
/** Check UARTX Parity option parameter */
#define PARAM_UARTX_PARITY(OPT) (((OPT) >= UARTX_PARITY_NONE && (OPT) <= UARTX_PARITY_0))

/**
 * @brief UARTX Databit type definitions
 */
typedef enum {
    UARTX_DATABIT_5 = 0, /*!< 5 bit character length. */
    UARTX_DATABIT_6,     /*!< 6 bit character length. */
    UARTX_DATABIT_7,     /*!< 7 bit character length. */
    UARTX_DATABIT_8      /*!< 8 bit character length. */
} UARTX_CHAR_LENGTH;
/** Check UARTX Databit option parameter */
#define PARAM_UARTX_CHAR_LENGTH(OPT) (((OPT) >= UARTX_DATABIT_5 && (OPT) <= UARTX_DATABIT_8))

/**
 * @brief UARTX Stop bit type definitions
 */
typedef enum {
    UARTX_STOPBITS_1 = 0, /* 1 stop bit. */
    UARTX_STOPBITS_2,     /* 2 stop bits. */
} UARTX_STOPBITS;
/** Check UARTX Stop bit option parameter */
#define PARAM_UARTX_STOPBITS(OPT) (((OPT) >= UARTX_STOPBITS_1 && (OPT) <= UARTX_STOPBITS_2))

/**
 * @brief FIFO Level type definitions
 */
typedef enum {
    UARTX_FIFO_TRG_1 = 0, /*!< FIFO trigger level: 1 character. */
    UARTX_FIFO_TRG_4,     /*!< FIFO trigger level: 4 character. */
    UARTX_FIFO_TRG_8,     /*!< FIFO trigger level: 8 character. */
    UARTX_FIFO_TRG_14     /*!< FIFO trigger level: 14 character. */
} UARTX_FIFO_LEVEL;
/** Check UARTX FIFO Level option parameter */
#define PARAM_UARTX_FIFO_LEVEL(OPT) (((OPT) >= UARTX_FIFO_TRG_1 && (OPT) <= UARTX_FIFO_TRG_14))

/**
 * @brief UARTX Auto-baudrate mode type definition
 */
typedef enum {
    UARTX_AUTOBAUD_MODE0 = 0, /**< UARTX Auto baudrate Mode 0. */
    UARTX_AUTOBAUD_MODE1,     /**< UARTX Auto baudrate Mode 1. */
} UARTX_AB_MODE;
/** Check UARTX Auto-baudrate mode option parameter */
#define PARAM_UARTX_AB_MODE(OPT) (((OPT) >= UARTX_AUTOBAUD_MODE0 && (OPT) <= UARTX_AUTOBAUD_MODE1))

/**
 * @brief UARTX Interrupt Type definitions
 */
typedef enum {
    UARTX_INT_RBR = 0, /*!< RBR Interrupt enable. */
    UARTX_INT_THRE,    /*!< THR Interrupt enable. */
    UARTX_INT_RXLS,    /*!< RX line status interrupt enable. */
    UARTX_INT_ABEO,    /*!< End of auto-baud interrupt enable. */
    UARTX_INT_ABTO     /*!< Time-out of auto-baud interrupt enable. */
} UARTX_INT;
/** Check UARTX Interrupt option parameter */
#define PARAM_UARTX_INT_CFG(OPT) (((OPT) >= UARTX_INT_RBR) && ((OPT) <= UARTX_INT_ABTO))
/** Check UARTX Auto-baudrate interrupt option parameter */
#define PARAM_UARTX_AB_INT(OPT)  (((OPT) == UARTX_INT_ABEO) || ((OPT) != UARTX_INT_ABTO))

/**
 * UART IrDA Control type Definition
 */
typedef enum {
    UARTX_IrDA_PULSEDIV2 = 0, /**< Pulse width = 2 * Tpclk. */
    UARTX_IrDA_PULSEDIV4,     /**< Pulse width = 4 * Tpclk. */
    UARTX_IrDA_PULSEDIV8,     /**< Pulse width = 8 * Tpclk. */
    UARTX_IrDA_PULSEDIV16,    /**< Pulse width = 16 * Tpclk. */
    UARTX_IrDA_PULSEDIV32,    /**< Pulse width = 32 * Tpclk. */
    UARTX_IrDA_PULSEDIV64,    /**< Pulse width = 64 * Tpclk. */
    UARTX_IrDA_PULSEDIV128,   /**< Pulse width = 128 * Tpclk. */
    UARTX_IrDA_PULSEDIV256    /**< Pulse width = 256 * Tpclk. */
} UARTX_IrDA_PULSE_Type;
/** Check UARTX IrDA Pulse Divider option parameter */
#define PARAM_UARTX_IrDA_PULSE_DIV(OPT) (((OPT) >= UARTX_IrDA_PULSEDIV2) && ((OPT) <= UARTX_IrDA_PULSEDIV256))

/**
 * @brief UARTX Configuration Structure definition
 */
typedef struct {
    uint32_t baudRate;            /**< Baud rate. */
    UARTX_PARITY parity;          /**< Should be:
                                  - UARTX_PARITY_NONE:.
                                  - UARTX_PARITY_ODD:.
                                  - UARTX_PARITY_EVEN:.
                                  - UARTX_PARITY_SP_1:.
                                  - UARTX_PARITY_SP_0:. */
    UARTX_CHAR_LENGTH charLength; /**< Should be:
                                  - UARTX_DATABIT_5.
                                  - UARTX_DATABIT_6.
                                  - UARTX_DATABIT_7.
                                  - UARTX_DATABIT_8. */
    UARTX_STOPBITS stopBits;      /**< Should be:
                                  - UARTX_STOPBITS_1.
                                  - UARTX_STOPBITS_2. */
} UARTX_CFG_Type;

/**
 * @brief UARTX FIFO Configuration Structure definition
 */
typedef struct {
    FunctionalState FIFOResetRxBuf; /**< Should be:
                                    - ENABLE: Reset Rx FIFO in UART.
                                    - DISABLE: Do not reset Rx FIFO in UART. */
    FunctionalState FIFOResetTxBuf; /**< Should be:
                                    - ENABLE: Reset Tx FIFO in UART.
                                    - DISABLE: Do not reset Tx FIFO in UART. */
    FunctionalState FIFODMAMode;    /**< Should be:
                                    - ENABLE: Enable DMA mode in UART.
                                    - DISABLE: Disable DMA mode in UART. */
    UARTX_FIFO_LEVEL FIFOLevel;     /**< Should be:
                                    - UARTX_FIFO_TRG_1: Rx FIFO trigger level: 1 character.
                                    - UARTX_FIFO_TRG_4: Rx FIFO trigger level: 4 character.
                                    - UARTX_FIFO_TRG_8: Rx FIFO trigger level: 8 character.
                                    - UARTX_FIFO_TRG_14: Rx FIFO trigger level: 14 character. */
} UARTX_FIFO_CFG_Type;

/**
 * @brief Auto Baudrate mode configuration type definition
 */
typedef struct {
    UARTX_AB_MODE abMode;        /**< Autobaudrate mode. */
    FunctionalState autoRestart; /**< Auto Restart state. */
} UARTX_AB_CFG_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup ADC_Public_Functions ADC Public Functions
 * @{
 */

/**
 * @brief      Initializes the specified UARTX peripheral.
 *
 * This function enables the peripheral power/clock for the selected UART, configures
 * and resets FIFOs, flushes pending RX data, briefly enables the transmitter
 * to flush THR, clears interrupt and control registers, programs baud-rate divisors
 * and configures the Line Control Register.
 *
 * @param[in]  UARTx   Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param[in]  uartCfg Pointer to a UARTX_CFG_Type structure.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 * - The function may block while waiting for transmitter (THR) to flush.
 * - TXEN bit in TER register is disabled at the end of this function.
 */
void UARTX_Init(LPC_UARTX_TypeDef* UARTx, const UARTX_CFG_Type* uartCfg);

/**
 * @brief      De-initializes the specified UARTX peripheral.
 *
 * This function disables the UART transmitter, and removes peripheral
 * power/clock for the selected UART instance.
 *
 * @param[in]  UARTx  Pointer to the UART peripheral (UARTx [0,2,3]).
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 * - Transmitter is disabled via TER.
 * - The function disables peripheral power using CLKPWR_ConfigPPWR for LPC_UARTx.
 * - After this call the UART peripheral must be re-initialized before use.
 */
void UARTX_DeInit(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief      Configures the UART FIFO control register (FCR).
 *
 * This function composes and writes the FCR value for the specified UART peripheral
 * to enable/disable FIFOs, reset RX/TX FIFO buffers, select DMA mode and set the
 * FIFO trigger level.
 *
 * @param[in]  UARTx   Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param[in]  fifoCfg Pointer to a UARTX_FIFO_CFG_Type structure.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_FIFOConfig(LPC_UARTX_TypeDef* UARTx, const UARTX_FIFO_CFG_Type* fifoCfg);

/**
 * @brief      Sends a single byte through the specified UARTX peripheral.
 *
 * This function writes one byte to the UART Transmit Holding Register (THR).
 * The value is masked with UARTX_THR_MASKBIT before being written to ensure
 * only defined data bits are transmitted. The call returns immediately and
 * does not poll the line status.
 *
 * @param[in]  UARTx  Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param[in]  byte   Byte value to be transmitted.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 * - This function does not wait for the Transmit Holding Register Empty (THRE).
 */
void UARTX_SendByte(LPC_UARTX_TypeDef* UARTx, uint8_t byte);

/**
 * @brief      Receives a single byte from the specified UARTX peripheral.
 *
 * This function reads one byte from the UART Receive Buffer Register (RBR).
 * The value is masked with UARTX_RBR_MASKBIT before being returned to ensure
 * only defined data bits are received.
 *
 * @param[in]  UARTx  Pointer to the UART peripheral (UARTx [0,2,3]).
 * @return     The received byte.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
uint8_t UARTX_ReceiveByte(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief     Sends a block of data via UARTX peripheral.
 *
 * This function sends a block of data via the specified UART peripheral.
 * Depending on the 'blocking' parameter, the function operates in either
 * blocking or non-blocking mode. In blocking mode, the function waits until
 * all data is sent or a timeout occurs. In non-blocking mode, the function
 * sends as much data as possible without waiting.
 *
 * @param UARTx     Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param txBuff    Pointer to transmit buffer.
 * @param buffLen   Length of transmit buffer.
 * @param blocking  Blocking mode enable/disable:
 *                  - TRUE: Blocking mode enabled.
 *                  - FALSE: Non-blocking mode.
 *
 * @return          Number of bytes sent.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
uint32_t UARTX_Send(LPC_UARTX_TypeDef* UARTx, const uint8_t* txBuff, uint32_t buffLen, Bool blocking);

/**
 * @brief     Receives a block of data via UARTX peripheral.
 *
 * This function receives a block of data via the specified UART peripheral.
 * Depending on the 'blocking' parameter, the function operates in either
 * blocking or non-blocking mode. In blocking mode, the function waits until
 * all data is received or a timeout occurs. In non-blocking mode, the function
 * receives as much data as available without waiting.
 *
 * @param UARTx     Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param rxBuff    Pointer to receive buffer.
 * @param buffLen   Amount of data to be received (should be less than
 *                  the size of rxBuff).
 * @param blocking  Blocking mode enable/disable:
 *                  - TRUE: Blocking mode enabled.
 *                  - FALSE: Non-blocking mode.
 *
 * @return          Number of bytes successfully received.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
uint32_t UARTX_Receive(LPC_UARTX_TypeDef* UARTx, uint8_t* rxBuff, uint32_t buffLen, Bool blocking);

/**
 * @brief       Get Interrupt Identification value
 *
 * @param UARTx Pointer to the UART peripheral (UARTx [0,2,3]).
 *
 * @return      Current value of UART UIIR register in UART peripheral.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
uint32_t UARTX_GetIntId(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief       Get Line Status value
 *
 * @param UARTx Pointer to the UART peripheral (UARTx [0,2,3]).
 *
 * @return      Current value of UART LSR register in UART peripheral.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
uint8_t UARTX_GetLineStatus(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief       Enables or Disables the specified UARTX interrupt.
 *
 * @param UARTx   Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param option  UARTX interrupt type, should be:
 *                 - UARTX_INT_RBR: RBR Interrupt enable.
 *                 - UARTX_INT_THRE: THR Interrupt enable.
 *                 - UARTX_INT_RXLS: RX line status interrupt enable.
 *                 - UARTX_INT_ABEO: End of auto-baud interrupt enable.
 *                 - UARTX_INT_ABTO: Time-out of auto-baud interrupt enable.
 *
 * @param newState Should be:
 *                 - ENABLE: Enable this UARTX interrupt type.
 *                 - DISABLE: Disable this UARTX interrupt type.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_IntConfig(LPC_UARTX_TypeDef* UARTx, UARTX_INT option, FunctionalState newState);

/**
 * @brief       Enable/Disable transmission on UARTX TxD pin
 *
 * @param UARTx    Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param newState Should be:
 *                 - ENABLE: Enable this function.
 *                 - DISABLE: Disable this function.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_TxCmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState);

/**
 * @brief       Check whether if UARTX transmitter is busy or not.
 *
 * @param UARTx Pointer to the UART peripheral (UARTx [0,2,3]).
 *
 * @return      RESET if UARTX is not busy, otherwise return SET.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
FlagStatus UARTX_CheckBusy(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief       Force a break condition on UARTX TxD pin
 *
 * @param UARTx Pointer to the UART peripheral (UARTx [0,2,3]).
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_ForceBreak(LPC_UARTX_TypeDef* UARTx);

/**
 * @brief       Clear Auto-baudrate Interrupt Pending
 *
 * @param UARTx      Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param abIntType  Type of auto-baud interrupt, should be:
 *                    - UARTX_INT_ABEO: End of Auto-baud interrupt.
 *                    - UARTX_INT_ABTO: Auto-baud time out interrupt.
 */
void UARTX_ABClearIntPending(LPC_UARTX_TypeDef* UARTx, UARTX_INT abIntType);

/**
 * @brief       Enable/Disable Auto-baudrate function
 *
 * @param UARTx     Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param abCfg     Pointer to a UARTX_AB_CFG_Type structure.
 * @param newState  Should be:
 *                   - ENABLE: Enable this function.
 *                   - DISABLE: Disable this function.
 *
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_ABCmd(LPC_UARTX_TypeDef* UARTx, UARTX_AB_CFG_Type* abCfg, FunctionalState newState);

/**
 * @brief       Enable/Disable IrDA serial input inversion
 *
 * @param UARTx     Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param newState  Should be:
 *                   - ENABLE: Enable this function.
 *                   - DISABLE: Disable this function.
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_IrDAInvtInputCmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState);

/**
 * @brief       Enable/Disable IrDA function
 *
 * @param UARTx     Pointer to the UART peripheral (UARTx [0,2,3]).
 * @param newState  Should be:
 *                   - ENABLE: Enable this function.
 *                   - DISABLE: Disable this function.
 * @note:
 * - Supported UART instances are LPC_UART0, LPC_UART2 and LPC_UART3.
 */
void UARTX_IrDACmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState);
void UARTX_IrDAPulseDivConfig(LPC_UARTX_TypeDef* UARTx, UARTX_IrDA_PULSE_Type pulseDiv);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  //LPC17XX_UARTX_H

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
