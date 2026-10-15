/***************************************************************************//**
* \file cy_debug.c
* \version 1.0
*
* Implements a generic debug logging mechanism for use on the FX device.
*
*******************************************************************************
* \copyright
* (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <string.h>
#include <stdarg.h>
#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_usbfs_cdc.h"
#include "cy_usb_cdc.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_cal_drv.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Use DMAC to send data to the UART. */
#define LOGGER_DMAC_ENABLE      (0u)

static cy_stc_debug_context_t glDbgCtx = {0};
static cy_stc_usb_cdc_ctxt_t  glUsbCdcCtx = {0};

/*******************************************************************************
 * Function name: Cy_USB_CdcDmaCallback
 ****************************************************************************//**
 *
 * USB CDC DMA channel callback function.
 *
 * \param pHandle
 * DMA channel handle
 *
 * \param type
 * Event type
 *
 * \param pbufStat
 * DMA buffer pointer
 *
 * \param userCtx
 * User context
 *
 *******************************************************************************/
void
Cy_USB_CdcDmaCallback (struct cy_stc_hbdma_channel *pHandle,
                       cy_en_hbdma_cb_type_t type,
                       cy_stc_hbdma_buff_status_t* pbufStat, void *userCtx)
{
    cy_stc_debug_recv_context_t rcvContext;
    cy_stc_hbdma_buff_status_t  buffStat;
    cy_en_hbdma_mgr_status_t    dmaStat;
    uint32_t intMask;
    uint32_t rcvSize = 0;

    /* Callback for data receive operation on OUT endpoint. */
    if (type == CY_HBDMA_CB_XFER_CPLT) {
        dmaStat = Cy_HBDma_Channel_WaitForReceiveCplt(pHandle, 0, &rcvSize);
        if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
            rcvContext = glDbgCtx.dbgRcvInfo;
            glDbgCtx.dbgRcvInfo.pReadBuffer = NULL;
            glDbgCtx.dbgRcvInfo.readDoneCb  = NULL;
            glDbgCtx.dbgRcvInfo.readCount   = 0;

            if (rcvSize != 0) {
                /* Read is done. Send the callback. */
                rcvContext.readDoneCb(rcvContext.pReadBuffer, (uint16_t)rcvSize, rcvContext.pUserCtxt);
            }
        }
    }

    /* Callback for data send complete on IN endpoint. */
    if (type == CY_HBDMA_CB_CONS_EVENT)
    {
        intMask = Cy_SysLib_EnterCriticalSection();
        if (glDbgCtx.pMsg == 0) {
            dmaStat = Cy_HBDma_Channel_GetBuffer(pHandle, &buffStat);
            if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
                glDbgCtx.pMsg = buffStat.pBuffer;
            }
        }
        Cy_SysLib_ExitCriticalSection(intMask);
    }
}

/*******************************************************************************
 * Function name: Cy_Debug_LogInit
 ****************************************************************************//**
 *
 * The API initializes the debug logger module with the desired configuration.
 *
 * \param pDbgCfg
 * Debug module config parameters.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_Debug_LogInit (cy_stc_debug_config_t *pDbgCfg)
{
    uint32_t dmaTrigSrc;

    glDbgCtx.pMsg       = pDbgCfg->pBuffer;
    glDbgCtx.dbgLevel   = pDbgCfg->traceLvl;
    glDbgCtx.bufSize    = pDbgCfg->bufSize;
    glDbgCtx.rdPtr      = 0;
    glDbgCtx.wrPtr      = 0;
    glDbgCtx.intfc      = pDbgCfg->dbgIntfce;
    glDbgCtx.printNow   = pDbgCfg->printNow;
    glDbgCtx.inProgress = false;
    glDbgCtx.pDbgScb    = NULL;
    glDbgCtx.maxDmaSize = 64u;

    glUsbCdcCtx.cdcEpIn           = pDbgCfg->cdcEpIn;
    glUsbCdcCtx.cdcEpOut          = pDbgCfg->cdcEpOut;
    glUsbCdcCtx.bufCount          = pDbgCfg->bufCount;
    glUsbCdcCtx.pUsbdCtxt         = pDbgCfg->pUsbCtxt;
    glUsbCdcCtx.recvEnabled       = pDbgCfg->recvEnabled;
    glUsbCdcCtx.cdcSendHandle     = NULL;
    glUsbCdcCtx.cdcRecvHandle     = NULL;
    glUsbCdcCtx.pCfgBuffer        = NULL;

    glDbgCtx.usbCdcInfo = &glUsbCdcCtx;

    glDbgCtx.dbgRcvInfo.pReadBuffer = NULL;
    glDbgCtx.dbgRcvInfo.readDoneCb  = NULL;
    glDbgCtx.dbgRcvInfo.pUserCtxt   = NULL;
    glDbgCtx.dbgRcvInfo.requestSize = 0;
    glDbgCtx.dbgRcvInfo.readCount   = 0;
    glDbgCtx.debugInit              = 0;

    (void)dmaTrigSrc;

    switch (pDbgCfg->dbgIntfce)
    {
        case CY_DEBUG_INTFCE_UART_SCB0:
            glDbgCtx.pDbgScb    = (void *)SCB0;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[0].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB0, Cy_SCB_GetFifoSize(SCB0) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB0_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_UART_SCB1:
            glDbgCtx.pDbgScb    = (void *)SCB1;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[1].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB1, Cy_SCB_GetFifoSize(SCB1) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB1_TX_REQ;
            glDbgCtx.debugInit = 1;

            break;

#if (FX2G3_EN != 1)
        case CY_DEBUG_INTFCE_UART_SCB2:
            glDbgCtx.pDbgScb    = (void *)SCB2;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[2].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB2, Cy_SCB_GetFifoSize(SCB2) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB2_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_UART_SCB3:
            glDbgCtx.pDbgScb    = (void *)SCB3;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[3].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB3, Cy_SCB_GetFifoSize(SCB3) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB3_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;
#endif /* (!FX2G3_EN) */

        case CY_DEBUG_INTFCE_UART_SCB4:
            glDbgCtx.pDbgScb    = (void *)SCB4;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[4].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB4, Cy_SCB_GetFifoSize(SCB4) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB4_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_UART_SCB5:
            glDbgCtx.pDbgScb    = (void *)SCB5;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[5].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB5, Cy_SCB_GetFifoSize(SCB5) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB5_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_UART_SCB6:
            glDbgCtx.pDbgScb    = (void *)SCB6;
            glDbgCtx.maxDmaSize = 32u;

            /* Configure the TX_FIFO trigger level for the SCB to the maxDmaSize value and
             * connect the SCB[6].tr_tx_req to the trigger input of DMAC channel 5.
             */
            Cy_SCB_SetTxFifoLevel(SCB6, Cy_SCB_GetFifoSize(SCB6) - glDbgCtx.maxDmaSize);
            dmaTrigSrc = TRIG_IN_MUX_5_SCB6_TX_REQ;
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_USBFS_CDC:
            glDbgCtx.maxDmaSize = 64u;
            CyUsbFsCdc_ControlDataReceive(pDbgCfg->recvEnabled);

            /* Enable the USBFS interface for CDC enumeration. */
            CyUsbFsCdc_Init();
            CyUsbFsCdc_Enable();
            glDbgCtx.debugInit = 1;
            break;

        case CY_DEBUG_INTFCE_USB_CDC:
            /* In the case of USB CDC based logging, the actual interface initialization can only be
             * done when the USB link is up. If the API is being called too soon, we just do parameter
             * validation to ensure that the interface can function. The actual initialization is
             * performed when we receive a callback from USBD indicating that the USB link is up.
             */
            glDbgCtx.maxDmaSize = glDbgCtx.bufSize;
            glDbgCtx.debugInit = Cy_USB_CdcValidateParameters(&glDbgCtx);
            break;

        case CY_DEBUG_INTFCE_RAM:
        default:
            break;
    }

#if LOGGER_DMAC_ENABLE
    /* Enable the DMAC block. */
    if (glDbgCtx.pDbgScb != NULL) {
        Cy_DMAC_Enable(DMAC);
        Cy_TrigMux_Connect(dmaTrigSrc, TRIG_OUT_MUX_5_MDMA_TR_IN5, false, TRIGGER_TYPE_LEVEL);
    }
#endif /* LOGGER_DMAC_ENABLE */
}

/*******************************************************************************
 * Function name: Cy_Debug_LogDeInit
 ****************************************************************************//**
 *
 * The API de-initializes the debug logger module .
 *
 *
 * \return
 * void
 *******************************************************************************/
cy_en_debug_status_t Cy_Debug_LogDeInit (void)
{
    if (glDbgCtx.debugInit != 1) {
        return CY_DEBUG_STATUS_FAILURE;
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) {
       if (!Cy_USB_CdcDeInit (&glDbgCtx)) {
            return CY_DEBUG_STATUS_FAILURE;
       } else {
           /* Clear the USBD link status change callback. */
           Cy_USBD_RegisterUsbDebugCallback((cy_stc_usb_usbd_ctxt_t *)glUsbCdcCtx.pUsbdCtxt, NULL, NULL);
       }
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USBFS_CDC) {
        CyUsbFsCdc_Disable();
    }

    /* Clear the debug context data structure. */
    memset(&glDbgCtx, 0, sizeof(glDbgCtx));

    return CY_DEBUG_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_Debug_CdcPrintNow
 ****************************************************************************//**
 *
 * This API allows the immediate print feature of the debug module to be updated
 * at runtime.
 *
 * \param startPrint
 * Whether immediate (blocking) print function is to be enabled or disabled.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_Debug_CdcPrintNow (bool startPrint)
{
    /* Stale function. Not used any more. */
    (void)startPrint;
}

/*******************************************************************************
 * Function name: Cy_Debug_SetPrintNow
 ****************************************************************************//**
 *
 * This API allows the immediate print feature of the debug module to be updated
 * at runtime.
 *
 * \param immed_print_en
 * Whether immediate (blocking) print function is to be enabled or disabled.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_Debug_SetPrintNow(bool immed_print_en)
{
    glDbgCtx.printNow = immed_print_en;
}

/* Internal function used to convert integers to decimal or hexadecimal
 * string format. Only 32 bit numbers are supported.
 */
static uint8_t *Cy_Debug_IntToStr(uint8_t *convertedString,
                             uint32_t num,
                             uint8_t base)
{
    uint8_t *str_p, i = 10;
    str_p = convertedString;
    str_p[i] = '\0';
    do {
        str_p[--i] = "0123456789ABCDEF"[num % base];
        num /= base;
    } while (num != 0);

    return (&str_p[i]);
}

/* Internal function used to add a formatted message to the circular log buffer. */
static cy_en_debug_status_t Cy_Debug_AddToBuffer (char *message, va_list argp)
{
    uint8_t  *string_p;
    uint8_t  *argStr = NULL;
    bool      copyReqd = false;
    uint16_t  i, j;
    int32_t   intArg;
    uint32_t  intrState, uintArg;
    uint8_t   convertedString[11];

    /* Make sure we have a valid buffer to store the log data into. */
    if (glDbgCtx.pMsg == NULL) {
        return CY_DEBUG_STATUS_FAILURE;
    }

    /* Parse the string and copy into the buffer for sending out. */
    for (string_p = (uint8_t *)message, i = 0; (*string_p != '\0'); string_p++) {
        if (*string_p != '%') {
            glDbgCtx.curMsg[i++] = *string_p;
            continue;
        }

        string_p++;
        switch (*string_p) {
        case '%' :
            {
                glDbgCtx.curMsg[i++] = '%';
            }
            break;

        case 'c' :
            {
                glDbgCtx.curMsg[i++] = (uint8_t)va_arg (argp, int32_t);
            }
            break;

        case 'd' :
            {
                intArg = va_arg (argp, int32_t);
                if (intArg < 0) {
                    glDbgCtx.curMsg[i++] = '-';
                    intArg = -intArg;
                }

                argStr = Cy_Debug_IntToStr (convertedString, intArg, 10);
                copyReqd = true;
            }
            break;

        case 's':
            {
                argStr = va_arg (argp, uint8_t *);
                copyReqd = true;
            }
            break;

        case 'u':
            {
                uintArg = va_arg (argp, uint32_t);
                argStr = Cy_Debug_IntToStr (convertedString, uintArg, 10);
                copyReqd = true;
            }
            break;

        case 'X':
        case 'x':
            {
                uintArg = va_arg (argp, uint32_t);
                argStr = Cy_Debug_IntToStr (convertedString, uintArg, 16);
                copyReqd = true;
            }
            break;

        default:
            return CY_DEBUG_STATUS_FAILURE;
        }

        if (copyReqd) {
            j = (uint16_t)strlen((char *)argStr);

            if ((i + j + 1) >= MAX_DBG_MSG_SIZE) {
                /* Avoid buffer overflow. */
                return CY_DEBUG_STATUS_FAILURE;
            }

            memcpy((uint8_t *)(glDbgCtx.curMsg + i), (uint8_t *)argStr, j);
            i += j;

            copyReqd = false;
        }

        if ((i + 1) >= MAX_DBG_MSG_SIZE) {
            /* Avoid buffer overflow. */
            return CY_DEBUG_STATUS_FAILURE;
        }
    }

    glDbgCtx.curMsg[i++] = 0;

    /* Copy the data into the common log buffer within a critical section. */
    intrState = Cy_SysLib_EnterCriticalSection();

    j = glDbgCtx.bufSize - glDbgCtx.wrPtr;
    if (j < i) {
        j = glDbgCtx.bufSize - glDbgCtx.wrPtr;
        memcpy((uint8_t *)(glDbgCtx.pMsg + glDbgCtx.wrPtr), glDbgCtx.curMsg, j);
        memcpy((uint8_t *)glDbgCtx.pMsg, glDbgCtx.curMsg + j, i - j);
        glDbgCtx.wrPtr = i - j;
    } else {
        memcpy((uint8_t *)(glDbgCtx.pMsg + glDbgCtx.wrPtr), glDbgCtx.curMsg, i);
        glDbgCtx.wrPtr += i;
        if (glDbgCtx.wrPtr == glDbgCtx.bufSize) {
            glDbgCtx.wrPtr = 0;
        }
    }
    Cy_SysLib_ExitCriticalSection(intrState);

    return CY_DEBUG_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_Debug_AddToLog
 ****************************************************************************//**
 *
 * This is the main logging function supported by this debug module. It takes
 * in a variable list of arguments like the printf function, but only the "%c",
 * "%d", "%x" format specifiers are supported. A verbosity level is associated
 * with each message to be printed.
 *
 * \param dbgLevel
 * Verbosity level associated with the message.
 *
 * \param message
 * Format string.
 *
 * \return
 * Success/Failure return code.
 *******************************************************************************/
cy_en_debug_status_t Cy_Debug_AddToLog (uint8_t dbgLevel,
                                          char *message, ...)
{
    cy_en_debug_status_t status;
    va_list   argp;

    if ((glDbgCtx.pMsg == NULL) || (glDbgCtx.debugInit == 0)) {
        /* Debug module status check. */
        return CY_DEBUG_STATUS_FAILURE;
    }

    if (dbgLevel > glDbgCtx.dbgLevel) {
        /* Return success if this message is irrelevant. */
        return CY_DEBUG_STATUS_SUCCESS;
    }

    if (
            (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) &&
            (glUsbCdcCtx.cdcSendHandle == NULL)
       ) {
        /* Check if the logging interface has been setup properly. */
        return CY_DEBUG_STATUS_FAILURE;
    }

    va_start(argp, message);
    status = Cy_Debug_AddToBuffer(message, argp);
    va_end(argp);

    if ((status == CY_DEBUG_STATUS_SUCCESS) && (glDbgCtx.printNow == true)) {
        Cy_Debug_PrintLog();
    }

    return status;
}

#if LOGGER_DMAC_ENABLE
/* Internal function which sets up the DMAC descriptors used for log data output. */
static void Cy_Debug_ConfigDmaDscr (uint32_t *dmadscr_p, uint16_t rdPtr, uint16_t size, bool havePendingData)
{
    uint16_t maxSize  = glDbgCtx.maxDmaSize;
    uint16_t sizemask = maxSize - 1;

    dmadscr_p[1] = (uint32_t)glDbgCtx.pMsg + rdPtr;
    dmadscr_p[2] = (uint32_t)&((CySCB_Type *)glDbgCtx.pDbgScb)->TX_FIFO_WR;

    if (size > maxSize) {
        if ((size & sizemask) != 0) {
            dmadscr_p[0] = 0x2800007EUL;
            dmadscr_p[3] = sizemask;
            dmadscr_p[4] = 0x00000001UL;
            dmadscr_p[5] = (size / maxSize) - 1;
            dmadscr_p[6] = maxSize;
            dmadscr_p[7] = (uint32_t)(dmadscr_p + 8);

            dmadscr_p[8]  = 0x1900007EUL;
            dmadscr_p[9]  = dmadscr_p[1] + (size & ~sizemask);
            dmadscr_p[10] = dmadscr_p[2];
            dmadscr_p[11] = (size & sizemask) - 1;
            dmadscr_p[12] = 0x00000001UL;
            dmadscr_p[13] = 0x00000000UL;

            if (havePendingData) {
                dmadscr_p[8]  = 0x1800007EUL;
                dmadscr_p[13] = (uint32_t)(dmadscr_p + 16);
            }
        } else {
            dmadscr_p[0] = 0x2900007EUL;
            dmadscr_p[3] = sizemask;
            dmadscr_p[4] = 0x00000001UL;
            dmadscr_p[5] = (size / maxSize) - 1;
            dmadscr_p[6] = maxSize;
            dmadscr_p[7] = 0x00000000UL;

            if (havePendingData) {
                dmadscr_p[0] = 0x2800007EUL;
                dmadscr_p[7] = (uint32_t)(dmadscr_p + 16);
            }
        }
    } else {
        dmadscr_p[0] = 0x1900007EUL;
        dmadscr_p[3] = size - 1;
        dmadscr_p[4] = 0x00000001UL;
        dmadscr_p[5] = 0x00000000UL;

        if (havePendingData) {
            dmadscr_p[0] = 0x1800007EUL;
            dmadscr_p[5] = (uint32_t)(dmadscr_p + 16);
        }
    }
}
#endif /* LOGGER_DMAC_ENABLE */

/* Internal function used to output log data through USBFS CDC IN endpoint. */
static uint16_t Cy_Debug_LogtoUsbFS (uint16_t rdPtr, uint16_t limit)
{
    bool     usbstat;
    uint16_t size;

    while (limit > rdPtr) {
        size = ((limit - rdPtr) > glDbgCtx.maxDmaSize) ? glDbgCtx.maxDmaSize : (limit - rdPtr);
        usbstat = CyUsbFsCdc_EpDataWrite(1, glDbgCtx.pMsg + rdPtr, size);
        if (!usbstat) {
            break;
        }

        rdPtr += size;
    }

    return (rdPtr);
}

/*******************************************************************************
 * Function name: Cy_Debug_PrintLog
 ****************************************************************************//**
 *
 * This API is used to get the logger module to incrementally output a part of the
 * saved log messages to the selected interfaces such as UART or Virtual COM port.
 *
 * \return
 * void
 *******************************************************************************/
#if LOGGER_DMAC_ENABLE
void Cy_Debug_PrintLog(void)
{
    uint16_t size;
    uint32_t intrState;
    uint32_t *dmadscr_p = glDbgCtx.dbgDmaDscr[0];
    cy_stc_dmac_channel_config_t chnConf;

    /* Prevent multiple-execution of print tasks in parallel. */
    intrState = Cy_SysLib_EnterCriticalSection();
    if (glDbgCtx.inProgress) {
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }
    glDbgCtx.inProgress = true;
    Cy_SysLib_ExitCriticalSection(intrState);

    chnConf.priority   = 2UL;
    chnConf.enable     = true;
    chnConf.bufferable = false;
    chnConf.descriptor = (cy_stc_dmac_descriptor_t *)dmadscr_p;

    if (glDbgCtx.rdPtr == glDbgCtx.wrPtr) {
        glDbgCtx.inProgress = false;
        return;
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USBFS_CDC) {
        intrState = Cy_SysLib_EnterCriticalSection();
        if (glDbgCtx.rdPtr > glDbgCtx.wrPtr) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsbFS(glDbgCtx.rdPtr, glDbgCtx.bufSize);
            if (glDbgCtx.rdPtr == glDbgCtx.bufSize) {
                glDbgCtx.rdPtr = 0;
            }
        }

        if (glDbgCtx.rdPtr < glDbgCtx.wrPtr) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsbFS(glDbgCtx.rdPtr, glDbgCtx.wrPtr);
        }
        glDbgCtx.inProgress = false;
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }

    /* If the DMA channel is busy, wait for a short time to let it complete. */
    while (Cy_DMAC_Channel_IsEnabled(DMAC, 5)) {
        Cy_SysLib_DelayUs(10);
    }

    /* Prevent race conditions by not allowing other task/intr to execute in the middle of the logging procedure. */
    intrState = Cy_SysLib_EnterCriticalSection();

    if (glDbgCtx.rdPtr < glDbgCtx.wrPtr) {
        size = (glDbgCtx.wrPtr - glDbgCtx.rdPtr);
        Cy_Debug_ConfigDmaDscr(dmadscr_p, glDbgCtx.rdPtr, size, false);
    } else {
        size = (glDbgCtx.bufSize - glDbgCtx.rdPtr);
        Cy_Debug_ConfigDmaDscr(dmadscr_p, glDbgCtx.rdPtr, size, (glDbgCtx.wrPtr > 0));

        if (glDbgCtx.wrPtr != 0) {
            dmadscr_p += 16;
            Cy_Debug_ConfigDmaDscr(dmadscr_p, 0, glDbgCtx.wrPtr, false);
        }
    }

    Cy_DMAC_Channel_Init(DMAC, 5, &chnConf);
    glDbgCtx.rdPtr = glDbgCtx.wrPtr;
    glDbgCtx.inProgress = false;

    Cy_SysLib_ExitCriticalSection(intrState);
}
#else
void Cy_Debug_PrintLog(void)
{
    uint16_t size;
    uint16_t index;
    uint32_t intrState;

    /* Prevent multiple-execution of print tasks in parallel. */
    intrState = Cy_SysLib_EnterCriticalSection();
    if (glDbgCtx.inProgress) {
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }
    glDbgCtx.inProgress = true;
    Cy_SysLib_ExitCriticalSection(intrState);

    if (glDbgCtx.rdPtr == glDbgCtx.wrPtr) {
        glDbgCtx.inProgress = false;
        return;
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USBFS_CDC) {
        intrState = Cy_SysLib_EnterCriticalSection();
        if (glDbgCtx.rdPtr > glDbgCtx.wrPtr) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsbFS(glDbgCtx.rdPtr, glDbgCtx.bufSize);
            if (glDbgCtx.rdPtr == glDbgCtx.bufSize) {
                glDbgCtx.rdPtr = 0;
            }
        }

        if (glDbgCtx.rdPtr < glDbgCtx.wrPtr) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsbFS(glDbgCtx.rdPtr, glDbgCtx.wrPtr);
        }
        glDbgCtx.inProgress = false;
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) {
        intrState = Cy_SysLib_EnterCriticalSection();
        if (glDbgCtx.rdPtr > glDbgCtx.wrPtr) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsb(&glDbgCtx,glDbgCtx.rdPtr, glDbgCtx.bufSize);
            if (glDbgCtx.rdPtr == glDbgCtx.bufSize) {
                glDbgCtx.rdPtr = 0;
            }
        }

        if ((glDbgCtx.rdPtr < glDbgCtx.wrPtr)) {
            glDbgCtx.rdPtr = Cy_Debug_LogtoUsb(&glDbgCtx,glDbgCtx.rdPtr, glDbgCtx.wrPtr);
        }
        glDbgCtx.inProgress = false;
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }

    /* Prevent race conditions by not allowing other task/intr to execute in the middle of the logging procedure. */
    intrState = Cy_SysLib_EnterCriticalSection();

    if (glDbgCtx.rdPtr < glDbgCtx.wrPtr) {
        size = (glDbgCtx.wrPtr - glDbgCtx.rdPtr);
    } else {
        size = (glDbgCtx.bufSize - glDbgCtx.rdPtr);
        size += glDbgCtx.wrPtr;
    }

    for (index = 0; index < size; index++) {
        /* Block until there is space in the TX FIFO. */
        while (Cy_SCB_UART_GetNumInTxFifo((CySCB_Type *)glDbgCtx.pDbgScb) >= 100);

        Cy_SCB_WriteTxFifo((CySCB_Type *)glDbgCtx.pDbgScb, (uint32_t)glDbgCtx.pMsg[glDbgCtx.rdPtr++]);

        if(glDbgCtx.rdPtr == glDbgCtx.bufSize)
            glDbgCtx.rdPtr = 0;
    }

    glDbgCtx.inProgress = false;
    Cy_SysLib_ExitCriticalSection(intrState);
}
#endif /* LOGGER_DMAC_ENABLE */

/*******************************************************************************
 * Function name: Cy_Debug_ChangeLogLevel
 ****************************************************************************//**
 *
 * This API can be used to change the verbosity level of messages enabled by the
 * logger module at runtime.
 *
 * \param level
 * New verbosity level to be set.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_Debug_ChangeLogLevel(uint8_t level)
{
    if((level >= 1) && (level <= 4)) {
        glDbgCtx.dbgLevel = level;
    }
}

/*******************************************************************************
 * Function name: Cy_Debug_QueueDataRead
 ****************************************************************************//**
 *
 * If a CDC interface is being used for data logging, this function queues a read
 * on the corresponding OUT endpoint. The doneCbk will be called once the requested
 * amount of data is received or a SLP/ZLP is received on the OUT endpoint.
 *
 * \param pReadBuffer
 * Pointer to buffer into which data is to be read.
 *
 * \param dataLength
 * Maximum size of data to be read. Expected to be a multiple of 64.
 *
 * \param doneCbk
 * Callback function to be called on read completion.
 *
 * \param pUserCtxt
 * User context structure to be passed to the callback.
 *
 * \return
 * true if read operation is queued, false otherwise.
 *
 *******************************************************************************/
bool Cy_Debug_QueueDataRead (uint8_t *pReadBuffer, uint16_t dataLength,
                             cy_cb_debug_data_recv_cb_t doneCbk, void *pUserCtxt)
{
    cy_stc_usb_cdc_ctxt_t  *usbCdcInfo = NULL;

    /* Operation only supported when USBFS CDC & USBHS CDC is used for debug and connection
     * is present.
     */
    if ((glDbgCtx.intfc != CY_DEBUG_INTFCE_USB_CDC) && (glDbgCtx.intfc != CY_DEBUG_INTFCE_USBFS_CDC)) {
        return false;
    }
    /* If parameters are not valid or if read is already queued, return error. */
    if (
            (pReadBuffer == NULL) || (doneCbk == NULL) || (dataLength == 0) ||
            (glDbgCtx.dbgRcvInfo.pReadBuffer != NULL) || (glDbgCtx.dbgRcvInfo.readDoneCb != NULL)
    ) {
        return false;
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) {
        if (
                (glDbgCtx.usbCdcInfo == NULL) ||
                (((cy_stc_usb_cdc_ctxt_t *)glDbgCtx.usbCdcInfo)->pUsbdCtxt == NULL) ||
                (((cy_stc_usb_cdc_ctxt_t *)glDbgCtx.usbCdcInfo)->cdcRecvHandle == NULL)
           ) {
            return false;
        }
    }

    /* Store the request parameters. */
    glDbgCtx.dbgRcvInfo.pReadBuffer = pReadBuffer;
    glDbgCtx.dbgRcvInfo.requestSize = dataLength;
    glDbgCtx.dbgRcvInfo.readDoneCb  = doneCbk;
    glDbgCtx.dbgRcvInfo.readCount   = 0;
    glDbgCtx.dbgRcvInfo.pUserCtxt   = pUserCtxt;

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USBFS_CDC)
    {
        /* Queue data read on OUT endpoint. */
        CyUsbFsCdc_QueueEpRead(2);
    }

    if (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) {
        usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)glDbgCtx.usbCdcInfo;

        if (Cy_HBDma_Channel_ReceiveData(usbCdcInfo->cdcRecvHandle, 0,
                    glDbgCtx.dbgRcvInfo.pReadBuffer, glDbgCtx.dbgRcvInfo.requestSize, NULL) != CY_HBDMA_MGR_SUCCESS) {
            glDbgCtx.dbgRcvInfo.pReadBuffer = NULL;
            glDbgCtx.dbgRcvInfo.readDoneCb  = NULL;
            glDbgCtx.dbgRcvInfo.readCount   = 0;
            return false;
        }
    }

    return true;
}

/*******************************************************************************
 * Function name: Cy_Debug_HandleReadIntr
 ****************************************************************************//**
 *
 * Function called from USBFS ISR corresponding to CDC OUT endpoint to read
 * the data and pass on to user callback.
 *
 *******************************************************************************/
void Cy_Debug_HandleReadIntr (void)
{
    cy_stc_debug_recv_context_t rcvContext;
    uint8_t curCount = 0;

    CyUsbFsCdc_EpDataRead(2, glDbgCtx.dbgRcvInfo.pReadBuffer + glDbgCtx.dbgRcvInfo.readCount,
            &curCount);

    glDbgCtx.dbgRcvInfo.readCount += curCount;
    if ((glDbgCtx.dbgRcvInfo.readCount == glDbgCtx.dbgRcvInfo.requestSize) || (curCount < USBFS_EP_MAX_PKT_SIZE)) {
        rcvContext = glDbgCtx.dbgRcvInfo;
        glDbgCtx.dbgRcvInfo.pReadBuffer = NULL;
        glDbgCtx.dbgRcvInfo.readDoneCb  = NULL;
        glDbgCtx.dbgRcvInfo.readCount   = 0;

        /* Read is done. Send the callback. */
        rcvContext.readDoneCb(rcvContext.pReadBuffer, rcvContext.readCount, rcvContext.pUserCtxt);
    } else {
        /* Queue further read. */
        CyUsbFsCdc_QueueEpRead(2);
    }
}

/*******************************************************************************
 * Function name: Cy_Debug_SetCdcCtrlIntfNumber
 ****************************************************************************//**
 *
 * Store the USB CDC control interface number in the context structure for use
 * while handling USB control requests.
 *
 * \param usbSpeed
 * USB device connection speed
 *
 * \param cdcStartIntf
 * Interface number for the CDC control interface
 *
 *******************************************************************************/
void Cy_Debug_SetCdcCtrlIntfNumber (cy_en_usb_speed_t usbSpeed, uint8_t cdcStartIntf)
{
    switch (usbSpeed) {
        case CY_USBD_USB_DEV_SS_GEN1:
        case CY_USBD_USB_DEV_SS_GEN2:
        case CY_USBD_USB_DEV_SS_GEN1X2:
        case CY_USBD_USB_DEV_SS_GEN2X2:
            glUsbCdcCtx.ssCtrlIntf = cdcStartIntf;
            break;

        case CY_USBD_USB_DEV_HS:
            glUsbCdcCtx.hsCtrlIntf = cdcStartIntf;
            break;

        case CY_USBD_USB_DEV_FS:
            glUsbCdcCtx.fsCtrlIntf = cdcStartIntf;
            break;

        default:
            /* Nothing to do */
            break;
    }
}

/*******************************************************************************
 * Function name: Cy_Debug_HandleControlRequests
 ****************************************************************************//**
 *
 * Function to be called from the setup callback to handle class specific control
 * requests addressed to the USB CDC interface.
 *
 * \return
 * true if a valid control request was handled locally, false otherwise. A false
 * return would indicate that the request needs to be handled at the application
 * level including STALL of unknown requests.
 *
 *******************************************************************************/
bool Cy_Debug_HandleControlRequests (void)
{
    if (
            (glDbgCtx.debugInit) && (glDbgCtx.intfc == CY_DEBUG_INTFCE_USB_CDC) &&
            (glUsbCdcCtx.pCfgBuffer != NULL) && (glUsbCdcCtx.pUsbdCtxt != NULL)
       ) {
        return Cy_USB_CdcHandleControlRequests(&glUsbCdcCtx, (cy_stc_usb_usbd_ctxt_t *)(glUsbCdcCtx.pUsbdCtxt));
    }

    return false;
}

#if defined(__cplusplus)
}
#endif

/*[]*/

