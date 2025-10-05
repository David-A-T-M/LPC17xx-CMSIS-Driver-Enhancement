/**
 * @file    lpc17xx_gpdma.h
 * @brief   Contains all macro definitions and function prototypes
 *          support for GPDMA firmware library on LPC17xx
 * @version 2.0
 * @date    21. May. 2010
 * @author  NXP MCU SW Application Team
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
 * Date: 28/09/2025, Author: David Trujillo Medina
 */

/* ---------------------------- Peripheral group ---------------------------- */
/** @defgroup GPDMA GPDMA (General Purpose Direct Memory Access)
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_GPDMA_H_
#define LPC17XX_GPDMA_H_

/* -------------------------------- Includes -------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Private Macros ----------------------------- */
/** @defgroup GPDMA_Private_Macros GPDMA Private Macros
 * @{
 */

#define GPDMA_NUM_CHANNELS ((8))

/* ---------------------------- BIT DEFINITIONS ----------------------------- */
/** Bit mask for an especific channel. */
#define GPDMA_ChannelBit(n)       ((1UL << n))
/** Bit mask for an especific request source. */
#define GPDMA_DMACSoftBReq_Src(n) ((1UL << n))

/** DMACConfig register, enable bit. */
#define GPDMA_DMACConfig_E ((0x01))
/** DMACConfig register, endianness configuration bit. */
#define GPDMA_DMACConfig_M ((0x02))

/** Transfer size bits mask. */
#define GPDMA_DMACCxControl_TransferSize(n) (((n & 0xFFF) << 0))
/** Source burst size bits mask. */
#define GPDMA_DMACCxControl_SBSize(n)       (((n & 0x07) << 12))
/** Destination burst size bits mask. */
#define GPDMA_DMACCxControl_DBSize(n)       (((n & 0x07) << 15))
/** Source transfer width bits mask. */
#define GPDMA_DMACCxControl_SWidth(n)       (((n & 0x07) << 18))
/** Destination transfer width bits mask. */
#define GPDMA_DMACCxControl_DWidth(n)       (((n & 0x07) << 21))
/** Source increment control bit. */
#define GPDMA_DMACCxControl_SI              ((1UL << 26))
/** Destination increment control bit. */
#define GPDMA_DMACCxControl_DI              ((1UL << 27))
/** Terminal count interrupt enable bit. */
#define GPDMA_DMACCxControl_I               ((1UL << 31))

/** Channel enable bit. */
#define GPDMA_DMACCxConfig_E                 ((1UL << 0))
/** Source peripheral bits mask. */
#define GPDMA_DMACCxConfig_SrcPeripheral(n)  (((n) << 1))
/** Destination peripheral bits mask. */
#define GPDMA_DMACCxConfig_DestPeripheral(n) (((n) << 6))
/** Transfer type bits mask. */
#define GPDMA_DMACCxConfig_TransferType(n)   (((n) << 11))
/** Interrupt error mask. */
#define GPDMA_DMACCxConfig_IE                ((1UL << 14))
/** Interrupt terminal count mask. */
#define GPDMA_DMACCxConfig_ITC               ((1UL << 15))
/** Active status bit. */
#define GPDMA_DMACCxConfig_A                 ((1UL << 17))
/** Halt status bit. */
#define GPDMA_DMACCxConfig_H                 ((1UL << 18))

/* ---------------------------- MASK DEFINITIONS ---------------------------- */
#define GPDMA_DMACIntStat_ALL ((0xFF))

/**
 * @}
 */

/* ------------------------------ Public Types ------------------------------ */
/** @defgroup GPDMA_Public_Types GPDMA Public Types
 * @{
 */

/**
 * @brief GPDMA Channel enumeration.
 */
typedef enum {
    GPDMA_CHANNEL_0 = 0,
    GPDMA_CHANNEL_1,
    GPDMA_CHANNEL_2,
    GPDMA_CHANNEL_3,
    GPDMA_CHANNEL_4,
    GPDMA_CHANNEL_5,
    GPDMA_CHANNEL_6,
    GPDMA_CHANNEL_7
} GPDMA_CHANNEL;
/** Check GPDMA channel option parameter. */
#define PARAM_GPDMA_CHANNEL(CHANNEL) ((CHANNEL >= GPDMA_CHANNEL_0) && (CHANNEL <= GPDMA_CHANNEL_7))

/**
 * @brief GPDMA Connection type enumeration.
 */
typedef enum {
    GPDMA_SSP0_Tx = 0,
    GPDMA_SSP0_Rx,
    GPDMA_SSP1_Tx,
    GPDMA_SSP1_Rx,
    GPDMA_ADC,
    GPDMA_I2S_C0,
    GPDMA_I2S_C1,
    GPDMA_DAC,
    GPDMA_UART0_Tx,
    GPDMA_UART0_Rx,
    GPDMA_UART1_Tx,
    GPDMA_UART1_Rx,
    GPDMA_UART2_Tx,
    GPDMA_UART2_Rx,
    GPDMA_UART3_Tx,
    GPDMA_UART3_Rx,
    GPDMA_MAT0_0,
    GPDMA_MAT0_1,
    GPDMA_MAT1_0,
    GPDMA_MAT1_1,
    GPDMA_MAT2_0,
    GPDMA_MAT2_1,
    GPDMA_MAT3_0,
    GPDMA_MAT3_1
} GPDMA_CONNECTION;
/** Check GPDMA connection option parameter. */
#define PARAM_GPDMA_CONNECTION(CONN) ((CONN >= GPDMA_SSP0_Tx) && (CONN <= GPDMA_MAT3_1))

/**
 * @brief GPDMA Transfer type enumeration.
 */
typedef enum {
    GPDMA_M2M = 0,
    GPDMA_M2P,
    GPDMA_P2M,
    GPDMA_P2P
} GPDMA_TRANSFER_TYPE;
/** Check GPDMA transfer type option parameter. */
#define PARAM_GPDMA_TRANSFER_TYPE(TYPE) ((TYPE >= GPDMA_M2M) && (TYPE <= GPDMA_P2P))

/**
 * @brief Burst size in Source and Destination enumeration.
 */
typedef enum {
    GPDMA_BSIZE_1 = 0,
    GPDMA_BSIZE_4,
    GPDMA_BSIZE_8,
    GPDMA_BSIZE_16,
    GPDMA_BSIZE_32,
    GPDMA_BSIZE_64,
    GPDMA_BSIZE_128,
    GPDMA_BSIZE_256
} GPDMA_BURST_SIZE;
/** Check GPDMA burst size option parameter. */
#define PARAM_GPDMA_BURST_SIZE(SIZE) ((SIZE >= GPDMA_BSIZE_1) && (SIZE <= GPDMA_BSIZE_256))

/**
 * @brief Transfer width in Source and Destination enumeration.
 */
typedef enum {
    GPDMA_BYTE = 0,
    GPDMA_HALFWORD,
    GPDMA_WORD
} GPDMA_TRANSFER_WIDTH;
/** Check GPDMA transfer width option parameter. */
#define PARAM_GPDMA_TRANSFER_WIDTH(WIDTH) ((WIDTH >= GPDMA_BYTE) && (WIDTH <= GPDMA_WORD))

/**
 * @brief GPDMA Request select enumeration.
 */
typedef enum {
    GPDMA_UART = 0,
    GPDMA_TIMER
} GPDMA_REQUEST_SELECT;
/** Check GPDMA request select option parameter. */
#define PARAM_GPDMA_REQUEST_SELECT(SEL) ((SEL == GPDMA_UART) || (SEL == GPDMA_TIMER))

/**
 * @brief GPDMA Status type enumeration
 */
typedef enum {
    GPDMA_INT,        /**< GPDMA Interrupt Status */
    GPDMA_INTTC,      /**< GPDMA Interrupt Terminal Count Request Status */
    GPDMA_INTERR,     /**< GPDMA Interrupt Error Status */
    GPDMA_RAW_INTTC,  /**< GPDMA Raw Interrupt Terminal Count Status */
    GPDMA_RAW_INTERR, /**< GPDMA Raw Error Interrupt Status */
    GPDMA_ENABLED_CH  /**< GPDMA Enabled Channel Status */
} GPDMA_STATUS_TYPE;
/** Check GPDMA status type option parameter. */
#define PARAM_GPDMA_STAT(TYPE) ((TYPE >= GPDMA_INT) && (TYPE <= GPDMA_ENABLED_CH))

/**
 * @brief GPDMA Interrupt clear status enumeration
 */
typedef enum {
    GPDMA_CLR_INTTC, /**< GPDMA Interrupt Terminal Count Request Clear */
    GPDMA_CLR_INTERR /**< GPDMA Interrupt Error Clear */
} GPDMA_CLEAR_INT;
/** Check GPDMA clear interrupt option parameter. */
#define PARAM_GPDMA_CLEAR_INT(TYPE) ((TYPE == GPDMA_CLR_INTTC) || (TYPE == GPDMA_CLR_INTERR))

/**
 * @brief GPDMA Channel configuration structure.
 */
typedef struct {
    GPDMA_CHANNEL channelNum;           /**< GPDMA_CHANNEL_x [0...7]. */
    uint32_t transferSize;              /**< Block transfer size (4095 max). */
    GPDMA_TRANSFER_WIDTH transferWidth; /**< Used only for GPDMA_M2M only. Should be:
                                            - GPDMA_BYTE: 8-bit transfer.
                                            - GPDMA_HALFWORD: 16-bit transfer.
                                            - GPDMA_WORD: 32-bit transfer. */
    uint32_t srcMemAddr;                /**< Physical source address. Used only if source is memory */
    uint32_t dstMemAddr;                /**< Physical Destination Address. Used only if destination is memory */
    GPDMA_TRANSFER_TYPE transferType;   /**< Should be:
                                            - GPDMA_M2M: Memory to Memory transfer.
                                            - GPDMA_M2P: Memory to Peripheral transfer.
                                            - GPDMA_P2M: Peripheral to Memory transfer.
                                            - GPDMA_P2P: Peripheral to Peripheral transfer. */
    GPDMA_CONNECTION srcConn;           /**< Source peripheral connection. Used only if source is peripheral. */
    GPDMA_CONNECTION dstConn; /**< Destination peripheral connection. Used only if destination is peripheral. */
    uint32_t linkedList;      /**< Linker List Item, otherwise set to '0' */
} GPDMA_Channel_CFG_Type;

/**
 * @brief GPDMA Linker List Item structure.
 */
typedef struct {
    uint32_t srcAddr; /**< Source Address */
    uint32_t dstAddr; /**< Destination address */
    uint32_t nextLLI; /**< Next LLI address, otherwise set to '0' */
    uint32_t control; /**< GPDMA Control of this LLI */
} GPDMA_LLI_Type;

/**
 * @}
 */

/* ---------------------------- Public Functions ---------------------------- */
/** @defgroup GPDMA_Public_Functions GPDMA Public Functions
 * @{
 */

/**
 * @brief      Initializes the GPDMA controller.
 *
 * This function enables power for the GPDMA peripheral, clears and resets configuration
 * and control registers for all DMA channels to their default state, clears any pending
 * interrupt flags, and enables the DMA controller. The DMA is left ready for further
 * configuration and use.
 *
 * @note:
 * - DMA is enabled after calling this function.
 * - It is recommended to initialize DMA before any further configuration.
 */
void GPDMA_Init(void);

/**
 * @brief      Configures and sets up a GPDMA channel according to the provided configuration.
 *
 * This function validates the selected DMA channel, resets its registers, sets up the linked list,
 * configures channel registers and DMA request selection, and finally sets the channel configuration
 * for the transfer type and peripheral connections.
 *
 * @param[in]  dmaCfg  Pointer to a GPDMA_Channel_CFG_Type structure containing the channel configuration.
 *
 * @return     Status
 *             - SUCCESS : Channel setup completed successfully.
 *             - ERROR   : Channel is busy or configuration failed.
 *
 * @note:
 * - The DMA must be initialized before calling this function.
 * - Only one transfer can be active per channel at a time.
 */
Status GPDMA_Setup(const GPDMA_Channel_CFG_Type* dmaCfg);

/**
 * @brief      Enables or disables the specified GPDMA channel.
 *
 * This function sets or clears the enable bit in the DMACCConfig register of the given DMA channel,
 * effectively starting or stopping the DMA transfer for that channel.
 *
 * @param[in]  channel     DMA channel to control (GPDMA_CHANNEL_x [0...7]).
 * @param[in]  newState    Functional state:
 *                         - ENABLE  : Start the DMA channel.
 *                         - DISABLE : Stop the DMA channel.
 *
 * @note:
 * - The channel must be properly configured before enabling.
 * - Disabling the channel will halt any ongoing transfer.
 * - Disabling a channel finishes the current AHB transfer, but FIFO data is lost.
 * - Restarting a channel by just setting Enable is not recommended; re-configure the channel.
 * - The channel is also disabled automatically when the last LLI is reached, the transfer completes, or on error.
 */
void GPDMA_ChannelCmd(GPDMA_CHANNEL channel, FunctionalState newState);

/**
 * @brief      Gets the interrupt status for the specified GPDMA channel.
 *
 * This function checks if the interrupt flag for the given DMA channel is set,
 * according to the selected status type.
 *
 * @param[in]  type     Status type to check:
 *                      - GPDMA_INT        : General interrupt status
 *                      - GPDMA_INTTC      : Terminal count interrupt status
 *                      - GPDMA_INTERR     : Error interrupt status
 *                      - GPDMA_RAW_INTTC  : Raw terminal count status
 *                      - GPDMA_RAW_INTERR : Raw error status
 *                      - GPDMA_ENABLED_CH : Channel enabled status
 * @param[in]  channel  DMA channel to check (GPDMA_CHANNEL_x [0...7]).
 *
 * @return     IntStatus
 *             - SET   : The corresponding channel has an active interrupt request or is enabled.
 *             - RESET : No active interrupt request or channel is disabled.
 *
 * @note
 * - Use this function to check the status of DMA interrupts or channel enable state.
 * - The status is read directly from the DMA controller registers.
 */
IntStatus GPDMA_IntGetStatus(GPDMA_STATUS_TYPE type, GPDMA_CHANNEL channel);

/**
 * @brief      Clears the pending interrupt flag for the specified GPDMA channel.
 *
 * This function clears the pending interrupt flag for the given DMA channel,
 * according to the interrupt type: terminal count or error.
 *
 * @param[in]  type     Interrupt type to clear:
 *                      - GPDMA_CLR_INTTC  : Terminal count interrupt
 *                      - GPDMA_CLR_INTERR : Error interrupt
 * @param[in]  channel  DMA channel to clear (GPDMA_CHANNEL_x [0...7]).
 *
 * @note
 * - Uses DMACIntTCClear or DMACIntErrClr registers to clear the flag.
 * - Useful for managing DMA transfer interrupts.
 */
void GPDMA_ClearIntPending(GPDMA_CLEAR_INT type, GPDMA_CHANNEL channel);

/**
 * @brief      Generates a software DMA request for the specified peripheral connection.
 *
 * This function writes to the DMACSoftSReq register to initiate a DMA transfer
 * on the given peripheral connection line, without requiring a hardware event.
 *
 * @param[in]  line  Peripheral connection line (GPDMA_CONNECTION).
 *
 * @note
 * - Useful for starting DMA transfers from software.
 * - The `line` parameter must be a valid GPDMA_CONNECTION value.
 */
static __INLINE void DMA_SoftRequest(GPDMA_CONNECTION line) {
    if (line > 15) {
        LPC_GPDMA->DMACSoftSReq = GPDMA_DMACSoftBReq_Src((line - 8));
    } else {
        LPC_GPDMA->DMACSoftSReq = GPDMA_DMACSoftBReq_Src(line);
    }
}

/**
 * @brief      Generates a software burst DMA request for the specified peripheral connection.
 *
 * This function writes to the DMACSoftBReq register to initiate a DMA burst transfer
 * on the given peripheral connection line, without requiring a hardware event.
 *
 * @param[in]  line  Peripheral connection line (GPDMA_CONNECTION).
 *
 * @note
 * - Useful for starting DMA bursts from software.
 * - The `line` parameter must be a valid GPDMA_CONNECTION value.
 */
static __INLINE void DMA_SoftBurstRequest(GPDMA_CONNECTION line) {
    if (line > 15) {
        LPC_GPDMA->DMACSoftBReq = GPDMA_DMACSoftBReq_Src((line - 8));
    } else {
        LPC_GPDMA->DMACSoftBReq = GPDMA_DMACSoftBReq_Src(line);
    }
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_GPDMA_H_ */

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
