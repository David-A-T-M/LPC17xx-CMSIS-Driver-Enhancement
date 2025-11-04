/**
 * @file        lpc17xx_uartx.c
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
/** @addtogroup UARTX
 * @{
 */

/* -------------------------------- Includes -------------------------------- */
#include "lpc17xx_uartx.h"
#include "lpc17xx_clkpwr.h"

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */
#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

#ifdef _UART
/* ---------------------- Private Function Prototypes ----------------------- */
/* ------------------- End of Private Function Prototypes ------------------- */

/* --------------------------- Private Functions ---------------------------- */
/**
 * @brief        Determines best dividers to get a target clock rate
 * @param[in]    UARTx    Pointer to selected UART peripheral, should be:
 *                        - LPC_UART0: UART0 peripheral
 *                        - LPC_UART2: UART2 peripheral
 *                        - LPC_UART3: UART3 peripheral
 * @param[in]    baudRate Desired UART baud rate.
 * @return         Error status, could be:
 *                 - SUCCESS
 *                 - ERROR
 */
static Status uartSetDivisors(LPC_UARTX_TypeDef* UARTx, uint32_t baudRate) {
    Status errorStatus = ERROR;

    uint32_t uClk         = 0;
    uint32_t calcBaudrate = 0;
    uint32_t temp         = 0;

    uint32_t diviser           = 0;
    uint32_t mulFracDivOptimal = 1;
    uint32_t dividerAddOptimal = 0;
    uint32_t diviserOptimal    = 0;

    uint32_t relativeError        = 0;
    uint32_t relativeOptimalError = 100000;

    /* get UART block clock */
    switch ((uintptr_t)UARTx) {
        case (uintptr_t)LPC_UART0: uClk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_UART0); break;
        case (uintptr_t)LPC_UART2: uClk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_UART2); break;
        case (uintptr_t)LPC_UART3: uClk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_UART3); break;
        default: break;
    }

    uClk /= 16;

    /* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
    * The formula is :
    * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
    * It involves floating point calculations. That's the reason the formulae are adjusted with
    * Multiply and divide method.*/
    /* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
    * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
    for (uint32_t mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++) {
        for (uint32_t dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++) {
            temp = (mulFracDiv * uClk) / ((mulFracDiv + dividerAddFracDiv));

            diviser = temp / baudRate;
            if ((temp % baudRate) > (baudRate / 2))
                diviser++;

            if (diviser > 2 && diviser < 65536) {
                calcBaudrate = temp / diviser;

                if (calcBaudrate <= baudRate)
                    relativeError = baudRate - calcBaudrate;
                else
                    relativeError = calcBaudrate - baudRate;

                if ((relativeError < relativeOptimalError)) {
                    mulFracDivOptimal    = mulFracDiv;
                    dividerAddOptimal    = dividerAddFracDiv;
                    diviserOptimal       = diviser;
                    relativeOptimalError = relativeError;
                    if (relativeError == 0)
                        break;
                }
            } /* End of if */
        } /* end of inner for loop */
        if (relativeError == 0)
            break;
    } /* end of outer for loop  */

    if (relativeOptimalError < ((baudRate * UARTX_ACCEPTED_BAUDRATE_ERROR) / 100)) {
        UARTx->LCR |= UARTX_LCR_DLAB_EN;
        UARTx->DLM = UARTX_LOAD_DLM(diviserOptimal);
        UARTx->DLL = UARTX_LOAD_DLL(diviserOptimal);
        /* Then reset DLAB bit */
        UARTx->LCR &= (~UARTX_LCR_DLAB_EN) & UARTX_LCR_BITMASK;
        UARTx->FDR = (UARTX_FDR_MULVAL(mulFracDivOptimal) | UARTX_FDR_DIVADDVAL(dividerAddOptimal)) & UARTX_FDR_BITMASK;
        errorStatus = SUCCESS;
    }

    return errorStatus;
}
/* ------------------------ End of Private Functions ------------------------ */

/* ---------------------------- Public Functions ---------------------------- */
/** @addtogroup UARTX_Public_Functions
 * @{
 */

void UARTX_Init(LPC_UARTX_TypeDef* UARTx, const UARTX_CFG_Type* uartCfg) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_UARTX_PARITY(uartCfg->parity));
    CHECK_PARAM(PARAM_UARTX_CHAR_LENGTH(uartCfg->charLength));
    CHECK_PARAM(PARAM_UARTX_STOPBITS(uartCfg->stopBits));

    switch ((uintptr_t)UARTx) {
        case (uintptr_t)LPC_UART0: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0, ENABLE); break;
        case (uintptr_t)LPC_UART2: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART2, ENABLE); break;
        case (uintptr_t)LPC_UART3: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART3, ENABLE); break;
        default: break;
    }

    UARTx->FCR = (UARTX_FCR_FIFO_EN | UARTX_FCR_RX_RS | UARTX_FCR_TX_RS);
    UARTx->FCR = 0;

    while (UARTx->LSR & UARTX_LSR_RDR) {
        (void)UARTx->RBR;
    }

    UARTx->TER = UARTX_TER_TXEN;
    while (!(UARTx->LSR & UARTX_LSR_THRE))
        ;
    UARTx->TER = 0;

    UARTx->IER = 0;
    UARTx->LCR = 0;
    UARTx->ACR = 0;
    (void)UARTx->LSR;
    UARTx->ICR = 0;

    uartSetDivisors(UARTx, (uartCfg->baudRate));

    uint32_t lineCtrl = UARTx->LCR & (UARTX_LCR_DLAB_EN | UARTX_LCR_BREAK_EN);

    switch (uartCfg->charLength) {
        case UARTX_DATABIT_5: lineCtrl |= UARTX_DATABIT_5; break;
        case UARTX_DATABIT_6: lineCtrl |= UARTX_DATABIT_6; break;
        case UARTX_DATABIT_7: lineCtrl |= UARTX_DATABIT_7; break;
        case UARTX_DATABIT_8: lineCtrl |= UARTX_DATABIT_8; break;
        default: break;
    }

    if (uartCfg->parity != UARTX_PARITY_NONE) {
        lineCtrl |= UARTX_LCR_PARITY_EN;
        switch (uartCfg->parity) {
            case UARTX_PARITY_ODD: lineCtrl |= UARTX_LCR_PARITY_ODD; break;
            case UARTX_PARITY_EVEN: lineCtrl |= UARTX_LCR_PARITY_EVEN; break;
            case UARTX_PARITY_1: lineCtrl |= UARTX_LCR_PARITY_F_1; break;
            case UARTX_PARITY_0: lineCtrl |= UARTX_LCR_PARITY_F_0; break;
            default: break;
        }
    }

    switch (uartCfg->stopBits) {
        case UARTX_STOPBITS_1: lineCtrl |= UARTX_LCR_STOPBIT_1; break;
        case UARTX_STOPBITS_2: lineCtrl |= UARTX_LCR_STOPBIT_2; break;
        default: break;
    }

    UARTx->LCR = lineCtrl & UARTX_LCR_BITMASK;
}

void UARTX_DeInit(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    UARTX_TxCmd(UARTx, DISABLE);

    switch ((uintptr_t)UARTx) {
        case (uintptr_t)LPC_UART0: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0, DISABLE); break;
        case (uintptr_t)LPC_UART2: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART2, DISABLE); break;
        case (uintptr_t)LPC_UART3: CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART3, DISABLE); break;
        default: break;
    }
}

void UARTX_FIFOConfig(LPC_UARTX_TypeDef* UARTx, const UARTX_FIFO_CFG_Type* fifoCfg) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_UARTX_FIFO_LEVEL(fifoCfg->FIFOLevel));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(fifoCfg->FIFODMAMode));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(fifoCfg->FIFOResetRxBuf));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(fifoCfg->FIFOResetTxBuf));

    uint8_t tmp = UARTX_FCR_FIFO_EN;

    switch (fifoCfg->FIFOLevel) {
        case UARTX_FIFO_TRG_1: tmp |= UARTX_FIFO_TRG_1 << 6; break;
        case UARTX_FIFO_TRG_4: tmp |= UARTX_FIFO_TRG_4 << 6; break;
        case UARTX_FIFO_TRG_8: tmp |= UARTX_FIFO_TRG_8 << 6; break;
        case UARTX_FIFO_TRG_14: tmp |= UARTX_FIFO_TRG_1 << 6; break;
        default: break;
    }

    if (fifoCfg->FIFOResetTxBuf) {
        tmp |= UARTX_FCR_TX_RS;
    }
    if (fifoCfg->FIFOResetRxBuf) {
        tmp |= UARTX_FCR_RX_RS;
    }
    if (fifoCfg->FIFODMAMode) {
        tmp |= UARTX_FCR_DMAMODE_SEL;
    }

    UARTx->FCR = tmp & UARTX_FCR_BITMASK;
}

void UARTX_SendByte(LPC_UARTX_TypeDef* UARTx, uint8_t byte) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    UARTx->THR = byte & UARTX_THR_MASKBIT;
}

uint8_t UARTX_ReceiveByte(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    return UARTx->RBR & UARTX_RBR_MASKBIT;
}

uint32_t UARTX_Send(LPC_UARTX_TypeDef* UARTx, const uint8_t* txBuff, uint32_t buffLen, Bool blocking) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    uint32_t fifoCnt;
    const uint8_t* pChar = txBuff;
    uint32_t bytesToSend = buffLen;
    uint32_t bytesSent   = 0;

    if (blocking == TRUE) {
        while (bytesToSend) {
            uint32_t timeOut = UARTX_BLOCKING_TIMEOUT;

            while (!(UARTx->LSR & UARTX_LSR_THRE)) {
                if (timeOut == 0) {
                    break;
                }
                timeOut--;
            }

            if (timeOut == 0) {
                break;
            }

            fifoCnt = UARTX_TX_FIFO_SIZE;
            while (fifoCnt && bytesToSend) {
                UARTX_SendByte(UARTx, *pChar++);
                fifoCnt--;
                bytesToSend--;
                bytesSent++;
            }
        }
    } else {
        while (bytesToSend) {
            if (!(UARTx->LSR & UARTX_LSR_THRE)) {
                break;
            }

            fifoCnt = UARTX_TX_FIFO_SIZE;
            while (fifoCnt && bytesToSend) {
                UARTX_SendByte(UARTx, *pChar++);
                bytesToSend--;
                fifoCnt--;
                bytesSent++;
            }
        }
    }
    return bytesSent;
}

uint32_t UARTX_Receive(LPC_UARTX_TypeDef* UARTx, uint8_t* rxBuff, uint32_t buffLen, Bool blocking) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    uint8_t* pChar = rxBuff;

    uint32_t bytesToRecv = buffLen;
    uint32_t bytesRecv   = 0;

    if (blocking == TRUE) {
        while (bytesToRecv) {
            uint32_t timeOut = UARTX_BLOCKING_TIMEOUT;
            while (!(UARTx->LSR & UARTX_LSR_RDR)) {
                if (timeOut == 0)
                    break;
                timeOut--;
            }

            if (timeOut == 0) {
                break;
            }

            *pChar++ = UARTX_ReceiveByte(UARTx);
            bytesToRecv--;
            bytesRecv++;
        }
    } else {
        while (bytesToRecv) {
            if (!(UARTx->LSR & UARTX_LSR_RDR)) {
                break;
            }

            *pChar++ = UARTX_ReceiveByte(UARTx);
            bytesRecv++;
            bytesToRecv--;
        }
    }
    return bytesRecv;
}

uint32_t UARTX_GetIntId(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    return UARTx->IIR & 0x03CF;
}

uint8_t UARTX_GetLineStatus(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    return UARTx->LSR & UARTX_LSR_BITMASK;
}

void UARTX_IntConfig(LPC_UARTX_TypeDef* UARTx, UARTX_INT option, FunctionalState newState) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));
    CHECK_PARAM(PARAM_UARTX_INT_CFG(option));

    uint32_t intEnable = 0;

    switch (option) {
        case UARTX_INT_RBR: intEnable = UARTX_IER_RBRINT_EN; break;
        case UARTX_INT_THRE: intEnable = UARTX_IER_THREINT_EN; break;
        case UARTX_INT_RXLS: intEnable = UARTX_IER_RLSINT_EN; break;
        case UARTX_INT_ABEO: intEnable = UARTX_IER_ABEOINT_EN; break;
        case UARTX_INT_ABTO: intEnable = UARTX_IER_ABTOINT_EN; break;
        default: break;
    }

    if (newState == ENABLE) {
        UARTx->IER |= intEnable;
    } else {
        UARTx->IER &= ~intEnable & UARTX_IER_BITMASK;
    }
}

void UARTX_TxCmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE) {
        UARTx->TER |= UARTX_TER_TXEN;
    } else {
        UARTx->TER &= ~UARTX_TER_TXEN & UARTX_TER_BITMASK;
    }
}

FlagStatus UARTX_CheckBusy(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    if (UARTx->LSR & UARTX_LSR_TEMT) {
        return RESET;
    }
    return SET;
}

void UARTX_ForceBreak(LPC_UARTX_TypeDef* UARTx) {
    CHECK_PARAM(PARAM_UARTX(UARTx));

    UARTx->LCR |= UARTX_LCR_BREAK_EN;
}

void UARTX_ABClearIntPending(LPC_UARTX_TypeDef* UARTx, UARTX_INT abIntType) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_UARTX_AB_INT(abIntType));

    if (abIntType == UARTX_INT_ABEO) {
        UARTx->ACR |= UARTX_IIR_ABEO_INT;
    } else {
        UARTx->ACR |= UARTX_IIR_ABTO_INT;
    }
}

void UARTX_ABCmd(LPC_UARTX_TypeDef* UARTx, UARTX_AB_CFG_Type* abCfg, FunctionalState newState) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    uint32_t baudCtrl = 0;
    if (newState == ENABLE) {
        if (abCfg->abMode == UARTX_AUTOBAUD_MODE1) {
            baudCtrl |= UARTX_ACR_MODE;
        }

        if (abCfg->autoRestart == ENABLE) {
            baudCtrl |= UARTX_ACR_AUTO_RESTART;
        }
    }

    if (newState == ENABLE) {
        UARTx->LCR |= UARTX_LCR_DLAB_EN;
        UARTx->DLL = 0;
        UARTx->DLM = 0;

        UARTx->LCR &= ~UARTX_LCR_DLAB_EN;

        UARTx->FDR = 0x10;
        UARTx->ACR = UARTX_ACR_START | baudCtrl;
    } else {
        UARTx->ACR = 0;
    }
}

void UARTX_IrDAInvtInputCmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE) {
        UARTx->ICR |= UARTX_ICR_IRDAINV;
    } else {
        UARTx->ICR &= (~UARTX_ICR_IRDAINV) & UARTX_ICR_BITMASK;
    }
}

void UARTX_IrDACmd(LPC_UARTX_TypeDef* UARTx, FunctionalState newState) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_FUNCTIONALSTATE(newState));

    if (newState == ENABLE) {
        UARTx->ICR |= UARTX_ICR_IRDAEN;
    } else {
        UARTx->ICR &= (~UARTX_ICR_IRDAEN) & UARTX_ICR_BITMASK;
    }
}

void UARTX_IrDAPulseDivConfig(LPC_UARTX_TypeDef* UARTx, UARTX_IrDA_PULSE_Type pulseDiv) {
    CHECK_PARAM(PARAM_UARTX(UARTx));
    CHECK_PARAM(PARAM_UARTX_IrDA_PULSE_DIV(pulseDiv));

    uint32_t tmp1 = UARTX_ICR_PULSEDIV(pulseDiv);
    uint32_t tmp = UARTx->ICR & (~UARTX_ICR_PULSEDIV(7));
    tmp |= tmp1 | UARTX_ICR_FIXPULSE_EN;
    UARTx->ICR = tmp & UARTX_ICR_BITMASK;
}

/**
 * @}
 */

#endif  // _UART

/**
 * @}
 */

/* ------------------------------ End Of File ------------------------------- */
