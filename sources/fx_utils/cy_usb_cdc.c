/***************************************************************************//**
* \file cy_usb_cdc.c
* \version 1.0
*
* Provided API source for the CDC Debug Interface. Device Class implementation using
* the USB IP block.
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


#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_usb_cdc.h"
#include "cy_usb_common.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usbhs_cal_drv.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_USB_CdcChannelReset
 ****************************************************************************//**
 *
 * Function to de-initialize CDC interface for debug logs
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * True if deinitialization is successful else False
 *
 *******************************************************************************/
static bool Cy_USB_CdcChannelReset(cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    if ((pDbgCtxt != NULL) && (pDbgCtxt->usbCdcInfo != NULL)) {
        usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
        pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

        if ((usbCdcInfo->cdcSendHandle != NULL) && (pUsbdCtxt != NULL)) {
            Cy_HBDma_Channel_Reset(usbCdcInfo->cdcSendHandle);
            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN);
            Cy_USBD_ResetEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN, true);

            if ((usbCdcInfo->recvEnabled) && (usbCdcInfo->cdcRecvHandle != NULL)) {
                Cy_HBDma_Channel_Reset(usbCdcInfo->cdcRecvHandle);
                Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT);
                Cy_USBD_ResetEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT, true);
            }

            return true;
        }
    }

    return false;
}

/*******************************************************************************
 * Function name: Cy_USB_CdcChannelEnable
 *******************************************************************************
 *
 * The function enables the Send/Receive DMA channels
 *
 * \param pDbgCtxt
 * Debug Context.
 *
 *******************************************************************************/
bool Cy_USB_CdcChannelEnable (cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_hbdma_buff_status_t dmaBufStat;
    cy_en_hbdma_mgr_status_t   hbdma_stat = CY_HBDMA_MGR_SUCCESS;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    if (usbCdcInfo->cdcSendHandle != NULL) {
        hbdma_stat = Cy_HBDma_Channel_Enable(usbCdcInfo->cdcSendHandle, 0);
        if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
            return false;
        }

        hbdma_stat = Cy_HBDma_Channel_GetBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);
        if (hbdma_stat == CY_HBDMA_MGR_SUCCESS) {
            pDbgCtxt->pMsg = dmaBufStat.pBuffer;
        }
    }

    return true;
}

/*******************************************************************************
 * Function name: Cy_Debug_LogtoUsb
 ****************************************************************************//**
 *
 * The function used to output log data through USBFS CDC IN endpoint
 *
 * \param pDbgCtxt
 * Debug Context.
 *
 * \param rdPtr
 * Pointer to read data
 *
 * \param limit
 * Number of bytes to be read
 *
 * \return
 * return rdPtr pointer value
 *
 *******************************************************************************/
uint16_t Cy_Debug_LogtoUsb (cy_stc_debug_context_t *pDbgCtxt, uint16_t rdPtr, uint16_t limit)
{
    uint16_t size;
    cy_stc_hbdma_buff_status_t dmaBufStat;
    cy_en_hbdma_mgr_status_t   hbdma_stat = CY_HBDMA_MGR_SUCCESS;
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->pMsg == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return 0;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    if ((usbCdcInfo->pUsbdCtxt == NULL) || (usbCdcInfo->cdcSendHandle == NULL)) {
        return 0;
    }

    while (limit > rdPtr) {
        size = ((limit - rdPtr) > pDbgCtxt->maxDmaSize) ? pDbgCtxt->maxDmaSize : (limit - rdPtr);

        dmaBufStat.pBuffer = pDbgCtxt->pMsg+rdPtr;
        dmaBufStat.size    = pDbgCtxt->bufSize;
        dmaBufStat.count   = size;
        dmaBufStat.status  = 0;
        hbdma_stat = Cy_HBDma_Channel_CommitBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);

        if (hbdma_stat == CY_HBDMA_MGR_SUCCESS) {
            hbdma_stat = Cy_HBDma_Channel_GetBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);
            if (hbdma_stat == CY_HBDMA_MGR_SUCCESS) {
                pDbgCtxt->pMsg = dmaBufStat.pBuffer;
            } else {
                pDbgCtxt->pMsg = NULL;
                break;
            }
        }

        if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
            Cy_USB_CdcChannelReset(pDbgCtxt);
            Cy_USB_CdcChannelEnable(pDbgCtxt);
        }

        rdPtr += size;
    }

    rdPtr = 0;
    pDbgCtxt->wrPtr = 0;
    return (rdPtr);
}

/*******************************************************************************
 * Function name: Cy_USB_CdcLinkStateChangeCbk
 ****************************************************************************//**
 *
 * USB stack callback to debug logger module to enable/disable logging function
 * through CDC function implemented through the active USB link.
 *
 * \param pUsbdCtxt
 * USB stack context pointer.
 * \param isLinkUp
 * Whether the USB link is going up or down.
 * \param pUserCtxt
 * Application specific context variable.
 *
 *
 *******************************************************************************/
static void
Cy_USB_CdcLinkStateChangeCbk (
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        bool isLinkUp,
        void *pUserCtxt)
{
    cy_stc_debug_context_t *pDbgCtxt = (cy_stc_debug_context_t *)pUserCtxt;
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo = NULL;

    if ((pUsbdCtxt != NULL) && (pDbgCtxt != NULL) && (pDbgCtxt->usbCdcInfo != NULL)) {
        usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;

        if (isLinkUp) {
            /* Use validity of DMA channel handle to determine whether CDCInit was already called. */
            if (usbCdcInfo->cdcSendHandle == NULL) {
                if (Cy_USB_CdcInit(pDbgCtxt)) {
                    Cy_USB_CdcChannelEnable(pDbgCtxt);
                }
            }
        } else {
            /* Use validity of DMA channel handle to determine whether CDCInit was already called. */
            if (usbCdcInfo->cdcSendHandle != NULL) {
                Cy_USB_CdcDeInit(pDbgCtxt);
            }
        }
    }
}

/*******************************************************************************
 * Function name: Cy_USB_CdcValidateParameters
 ****************************************************************************//**
 *
 * Function which checks if the various CDC function parameters received are valid.
 * If all parameters are valid, a callback is registered with the USB stack which
 * allows the CDC functionality to be enabled or disabled at appropriate times.
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * true if the parameters are valid, false in case of missing or bad parameters
 * passed to Cy_Debug_LogInit.
 *
 *******************************************************************************/
bool Cy_USB_CdcValidateParameters (cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_usb_cdc_ctxt_t    *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t   *pUsbdCtxt;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

    /* Parameter validation: Verify that we have a valid USBD context as well as valid IN/OUT endpoints.
     * Both endpoints need to be valid even if data receive operation is not enabled.
     */
    if (
            (pUsbdCtxt == NULL) ||
            ((usbCdcInfo->cdcEpIn <= 0u) || (usbCdcInfo->cdcEpIn >= CY_USB_MAX_ENDP_NUMBER)) ||
            ((usbCdcInfo->cdcEpOut <= 0u) || (usbCdcInfo->cdcEpOut >= CY_USB_MAX_ENDP_NUMBER))
       ) {
        return false;
    }

    /* Register a callback function to be notified about USB link state changes. */
    if (CY_USBD_STATUS_SUCCESS == Cy_USBD_RegisterUsbDebugCallback(
                pUsbdCtxt, Cy_USB_CdcLinkStateChangeCbk, (void *)pDbgCtxt)) {
        return true;
    }

    return false;
}

/*******************************************************************************
 * Function name: Cy_USB_CdcInit
 ****************************************************************************//**
 *
 * Function to initialize CDC interface for debug logging. This is triggered
 * through callback from USBD stack when the USB device has been configured.
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * true if initialization is successful, else false
 *
 *******************************************************************************/
bool Cy_USB_CdcInit (cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t  hbdma_stat = CY_HBDMA_MGR_SUCCESS;
    cy_stc_usb_cdc_ctxt_t    *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t   *pUsbdCtxt;
    cy_stc_usb_endp_config_t  epConfig;
    uint16_t epPktSize = CY_CDC_BULK_EP_SIZE_SS;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

    /* Parameter validation */
    if (
            (pUsbdCtxt == NULL) ||
            (pUsbdCtxt->pHBDmaMgr == NULL) ||
            (pUsbdCtxt->devSpeed < CY_USBD_USB_DEV_FS) ||
            (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_SS_GEN2X2) ||
            ((usbCdcInfo->cdcEpIn <= 0u) || (usbCdcInfo->cdcEpIn >= CY_USB_MAX_ENDP_NUMBER)) ||
            ((usbCdcInfo->cdcEpOut <= 0u) || (usbCdcInfo->cdcEpOut >= CY_USB_MAX_ENDP_NUMBER))
       ) {
        return false;
    }

    /* Make sure we have a valid buffer allocated for control transfer handling. */
    if (usbCdcInfo->pCfgBuffer == NULL) {
        usbCdcInfo->pCfgBuffer = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pUsbdCtxt->pHBDmaMgr->pBufMgr, 64u);
        if (usbCdcInfo->pCfgBuffer == NULL) {
            return false;
        }
    }

    /* Fill the buffer with default CDC configuration data. */
    usbCdcInfo->pCfgBuffer[0] = 0x00u;
    usbCdcInfo->pCfgBuffer[1] = 0x10u;
    usbCdcInfo->pCfgBuffer[2] = 0x0Eu;
    usbCdcInfo->pCfgBuffer[3] = 0x00u; /* Baud rate = 0x000E1000 = 921600 */
    usbCdcInfo->pCfgBuffer[4] = 0x00u; /* 1 stop bit */
    usbCdcInfo->pCfgBuffer[5] = 0x00u; /* No parity */
    usbCdcInfo->pCfgBuffer[6] = 0x08u; /* 8 data bits */
    usbCdcInfo->pCfgBuffer[7] = 0x00u;

    /* If the channel is already created, destroy it and start over. */
    if (usbCdcInfo->cdcSendHandle != NULL) {
        Cy_HBDma_Channel_Disable(usbCdcInfo->cdcSendHandle);
        Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcSendHandle);
        usbCdcInfo->cdcSendHandle = NULL;
    }

    /* Create channel which will move data from SRAM to EPM. */
    dmaConfig.bufferMode     = false;                               /* DMA buffer mode disabled */
    dmaConfig.size           = pDbgCtxt->bufSize;
    dmaConfig.prodBufSize    = pDbgCtxt->bufSize;
    dmaConfig.prodHdrSize    = 0;                                   /* No header being added. */
    dmaConfig.count          = usbCdcInfo->bufCount;                /* DMA Buffer Count */
    dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;             /* DMA Channel type: from HB-RAM to USB-IP */
    dmaConfig.endpAddr       = 0x80 | usbCdcInfo->cdcEpIn;          /* Endpoint address */
    dmaConfig.usbMaxPktSize  = CY_CDC_BULK_EP_SIZE_SS;
    dmaConfig.prodSckCount   = 1;                                   /* No. of producer sockets */
    dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
    dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;             /* Producer Socket ID: None */
    dmaConfig.consSckCount   = 1;                                   /* No. of consumer Sockets */
    dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + usbCdcInfo->cdcEpIn);
    dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;             /* Consumer Socket ID: None */
    dmaConfig.cb             = Cy_USB_CdcDmaCallback;
    dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
    dmaConfig.eventEnable    = 0;                                   /* Events not supported in MEM to IP channels. */
    dmaConfig.userCtx        = (void *)pUsbdCtxt;

    /* Parameter adjustment for USB 2.x modes. */
    if (pUsbdCtxt->devSpeed < CY_USBD_USB_DEV_SS_GEN1) {
        dmaConfig.consSck[0] = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + usbCdcInfo->cdcEpIn);
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
            dmaConfig.usbMaxPktSize = CY_CDC_BULK_EP_SIZE_FS;
        } else {
            dmaConfig.usbMaxPktSize = CY_CDC_BULK_EP_SIZE_HS;
        }
    }

    epPktSize = dmaConfig.usbMaxPktSize;

    hbdma_stat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr, &(usbCdcInfo->endpInChannel), &dmaConfig);
    if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
        /* Not able to create Channel then return. */
        return false;
    } else {
        usbCdcInfo->cdcSendHandle = (void *)(&(usbCdcInfo->endpInChannel));
    }

    if (usbCdcInfo->recvEnabled == 1) {
        /* If the channel is already created, destroy it and start over. */
        if (usbCdcInfo->cdcRecvHandle != NULL) {
            Cy_HBDma_Channel_Disable(usbCdcInfo->cdcRecvHandle);
            Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcRecvHandle);
            usbCdcInfo->cdcRecvHandle = NULL;
        }

        /* If data receive is enabled, create a channel to read the data from the EPM. */
        dmaConfig.bufferMode    = false;
        dmaConfig.size          = 1024;
        dmaConfig.prodHdrSize   = 0;
        dmaConfig.prodBufSize   = 1024;
        dmaConfig.count         = 1;
        dmaConfig.chType        = CY_HBDMA_TYPE_IP_TO_MEM;
        dmaConfig.endpAddr      = usbCdcInfo->cdcEpOut;                 /* Endpoint address */
        dmaConfig.usbMaxPktSize = epPktSize;
        dmaConfig.prodSckCount  = 1;
        dmaConfig.prodSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + usbCdcInfo->cdcEpOut);
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;
        dmaConfig.consSckCount  = 1;
        dmaConfig.consSck[0]    = CY_HBDMA_VIRT_SOCKET_RD;
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;
        dmaConfig.cb            = Cy_USB_CdcDmaCallback;
        dmaConfig.intrEnable    = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk;
        dmaConfig.eventEnable   = 0;
        dmaConfig.userCtx       = (void *)pUsbdCtxt;

        /* Parameter adjustment for USB 2.x modes. */
        if (pUsbdCtxt->devSpeed < CY_USBD_USB_DEV_SS_GEN1) {
            dmaConfig.prodSck[0] = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_OUT_EP_00 + usbCdcInfo->cdcEpOut);
        }

        hbdma_stat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr,
                                             &(usbCdcInfo->endpOutChannel),
                                             &dmaConfig);
        if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
            /* Not able to create Channel then return. */
            return false;
        } else {
            usbCdcInfo->cdcRecvHandle = (void *)(&usbCdcInfo->endpOutChannel);
        }
    }

    /* Disable and then configure the CDC bulk endpoints. */

    epConfig.endpType           = CY_USB_ENDP_TYPE_BULK;
    epConfig.valid              = true;
    epConfig.isoPkts            = 0;
    epConfig.interval           = 0;
    epConfig.allowNakTillDmaRdy = true;
    epConfig.maxPktSize         = epPktSize;
    epConfig.burstSize          = 1;
    epConfig.streamID           = 0;

    if (usbCdcInfo->recvEnabled == 1) {
        Cy_USBD_EnableEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT, false);
        epConfig.endpDirection  = CY_USB_ENDP_DIR_OUT;
        epConfig.endpNumber     = usbCdcInfo->cdcEpOut;
        Cy_USB_USBD_EndpConfig(pUsbdCtxt, epConfig);
    }

    Cy_USBD_EnableEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN, false);
    epConfig.endpDirection      = CY_USB_ENDP_DIR_IN;
    epConfig.endpNumber         = usbCdcInfo->cdcEpIn;
    Cy_USB_USBD_EndpConfig(pUsbdCtxt, epConfig);

    return true;
}

/*******************************************************************************
 * Function name: Cy_USB_CdcDeInit
 ****************************************************************************//**
 *
 * Function to de-initialize CDC interface for debug logging. This is triggered
 * through callback from USB stack whenever the USB device is de-initialized or
 * reset.
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * true if deinitialization is successful, else false
 *
 *******************************************************************************/
bool Cy_USB_CdcDeInit (cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;
    if (pUsbdCtxt == NULL) {
        return false;
    }

    if (usbCdcInfo->cdcSendHandle != NULL) {
        Cy_HBDma_Channel_Disable(usbCdcInfo->cdcSendHandle);
        Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcSendHandle);
        usbCdcInfo->cdcSendHandle = NULL;

        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN);
        }
    }

    if ((usbCdcInfo->recvEnabled) && (usbCdcInfo->cdcRecvHandle != NULL)) {
        Cy_HBDma_Channel_Disable(usbCdcInfo->cdcRecvHandle);
        Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcRecvHandle);
        usbCdcInfo->cdcRecvHandle = NULL;

        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT);
        }
    }

    /* Disable the CDC bulk endpoints. */
    if (usbCdcInfo->recvEnabled) {
        /* Cancel state related to any ongoing read operation. */
        pDbgCtxt->dbgRcvInfo.pReadBuffer = NULL;
        pDbgCtxt->dbgRcvInfo.readDoneCb  = NULL;
        pDbgCtxt->dbgRcvInfo.readCount   = 0;

        Cy_USBD_EnableEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT, false);
    }
    Cy_USBD_EnableEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN, false);

    if (usbCdcInfo->pCfgBuffer != NULL) {
        /* Free buffer allocated for control request handling. */
        Cy_HBDma_BufMgr_Free(pUsbdCtxt->pHBDmaMgr->pBufMgr, usbCdcInfo->pCfgBuffer);
        usbCdcInfo->pCfgBuffer = NULL;
    }

    return true;
}

#define CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET        (2u)
#define CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET        (3u)
#define CYFX_CDC_CFG_NUMINTF_OFFSET             (4u)

#define CYFX_CDC_SS_CFG_DSCR_LEN                (71u)
#define CYFX_CDC_SS_IAD_INTFNUM_OFFSET          (2u)
#define CYFX_CDC_SS_CTRL_INTF_NUM_OFFSET        (10u)
#define CYFX_CDC_SS_CTRL_INTF_DATAINTF_OFFSET   (35u)
#define CYFX_CDC_SS_DATA_INTF_NUM_OFFSET        (38u)
#define CYFX_CDC_SS_OUTEP_NUM_OFFSET            (47u)
#define CYFX_CDC_SS_INEP_NUM_OFFSET             (60u)

const uint8_t CdcIntfDescriptor_SS[CYFX_CDC_SS_CFG_DSCR_LEN] = {
    /* Interface association descriptor: Need to update interface number field. */
    0x08,
    0x0B,
    0x01,
    0x02,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC control interface descriptor: Need to update interface number field. */
    0x09,
    0x04,
    0x01,
    0x00,
    0x00,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC class descriptors: No change required */
    0x05,
    0x24,
    0x00,
    0x10,
    0x01,
    0x04,
    0x24,
    0x02,
    0x02,
    0x05,
    0x24,
    0x06,
    0x00,
    0x01,
    0x05,
    0x24,
    0x01,
    0x00,
    0x01,

    /* CDC data interface descriptor: Interface number to be updated */
    0x09,
    0x04,
    0x02,
    0x00,
    0x02,
    0x0A,
    0x00,
    0x00,
    0x00,

    /* Bulk OUT endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x0F,
    0x02,
    0x00,
    0x04,
    0x00,
    0x06,
    0x30,
    0x00,
    0x00,
    0x00,
    0x00,

    /* Bulk IN endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x8F,
    0x02,
    0x00,
    0x04,
    0x00,
    0x06,
    0x30,
    0x00,
    0x00,
    0x00,
    0x00
};

#define CYFX_CDC_HS_CFG_DSCR_LEN                (59u)
#define CYFX_CDC_HS_IAD_INTFNUM_OFFSET          (2u)
#define CYFX_CDC_HS_CTRL_INTF_NUM_OFFSET        (10u)
#define CYFX_CDC_HS_CTRL_INTF_DATAINTF_OFFSET   (35u)
#define CYFX_CDC_HS_DATA_INTF_NUM_OFFSET        (38u)
#define CYFX_CDC_HS_OUTEP_NUM_OFFSET            (47u)
#define CYFX_CDC_HS_INEP_NUM_OFFSET             (54u)

const uint8_t CdcIntfDescriptor_HS[CYFX_CDC_HS_CFG_DSCR_LEN] = {
    /* Interface association descriptor: Need to update interface number field. */
    0x08,
    0x0B,
    0x01,
    0x02,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC control interface descriptor: Need to update interface number field. */
    0x09,
    0x04,
    0x01,
    0x00,
    0x00,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC class descriptors: No change required */
    0x05,
    0x24,
    0x00,
    0x10,
    0x01,
    0x04,
    0x24,
    0x02,
    0x02,
    0x05,
    0x24,
    0x06,
    0x00,
    0x01,
    0x05,
    0x24,
    0x01,
    0x00,
    0x01,

    /* CDC data interface descriptor: Interface number to be updated */
    0x09,
    0x04,
    0x02,
    0x00,
    0x02,
    0x0A,
    0x00,
    0x00,
    0x00,

    /* Bulk OUT endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x0F,
    0x02,
    0x00,
    0x02,
    0x00,

    /* Bulk IN endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x8F,
    0x02,
    0x00,
    0x02,
    0x00
};

const uint8_t CdcIntfDescriptor_FS[CYFX_CDC_HS_CFG_DSCR_LEN] = {
    /* Interface association descriptor: Need to update interface number field. */
    0x08,
    0x0B,
    0x01,
    0x02,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC control interface descriptor: Need to update interface number field. */
    0x09,
    0x04,
    0x01,
    0x00,
    0x00,
    0x02,
    0x02,
    0x01,
    0x00,

    /* CDC class descriptors: No change required */
    0x05,
    0x24,
    0x00,
    0x10,
    0x01,
    0x04,
    0x24,
    0x02,
    0x02,
    0x05,
    0x24,
    0x06,
    0x00,
    0x01,
    0x05,
    0x24,
    0x01,
    0x00,
    0x01,

    /* CDC data interface descriptor: Interface number to be updated */
    0x09,
    0x04,
    0x02,
    0x00,
    0x02,
    0x0A,
    0x00,
    0x00,
    0x00,

    /* Bulk OUT endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x0F,
    0x02,
    0x40,
    0x00,
    0x00,

    /* Bulk IN endpoint descriptor: endpoint number to be updated. */
    0x07,
    0x05,
    0x8F,
    0x02,
    0x40,
    0x00,
    0x00
};

/*******************************************************************************
 * Function name: Cy_USB_CdcAddConfigDescriptors
 ****************************************************************************//**
 *
 * Add descriptors for the USB CDC interface into the descriptors registered with the
 * USB stack. Please note that the descriptor arrays are expected to have the required
 * space for these descriptors: 84 bytes in case of USB 3.x and 66 bytes in case of
 * USB 2.x.
 *
 * \param pDbgCfg
 * Debug configuration structure.
 *
 * \return
 * true if descriptors are updated properly, false otherwise.
 *
 *******************************************************************************/
bool Cy_USB_CdcAddConfigDescriptors (cy_stc_debug_config_t *pDbgCfg)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    uint32_t descLength;
    uint8_t *pCfgDscr;
    uint8_t  numIntf;
    bool retVal = false;

    /* Note: We have no way of knowing the actual size of the descriptor buffer allocated.
     *       Size expectations are documented. Here, we will assume that there is sufficient space to add
     *       the CDC descriptors.
     */

    if ((pDbgCfg == NULL) || (pDbgCfg->pUsbCtxt == NULL) || (pDbgCfg->dbgIntfce != CY_DEBUG_INTFCE_USB_CDC)) {
        return false;
    }

    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)pDbgCfg->pUsbCtxt;

    if (pUsbdCtxt != NULL) {
        /* SS configuration update */
        if (pUsbdCtxt->dscrs.pUsbSsCfgDscr != NULL) {
            pCfgDscr   = (uint8_t *)pUsbdCtxt->dscrs.pUsbSsCfgDscr;
            numIntf    = pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET];
            descLength = ((pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] << 8) |
                         (pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET]));

            memcpy((pCfgDscr + descLength), CdcIntfDescriptor_SS, CYFX_CDC_SS_CFG_DSCR_LEN);

            /* Update required fields in the configuration descriptor. */
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET] = ((descLength + CYFX_CDC_SS_CFG_DSCR_LEN) & 0xFFu);
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] = ((descLength + CYFX_CDC_SS_CFG_DSCR_LEN) >> 8u);
            pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET]      = numIntf + 2;

            /* Update required fields in the CDC descriptor. */
            pCfgDscr[descLength + CYFX_CDC_SS_IAD_INTFNUM_OFFSET]        = numIntf;
            pCfgDscr[descLength + CYFX_CDC_SS_CTRL_INTF_NUM_OFFSET]      = numIntf;
            pCfgDscr[descLength + CYFX_CDC_SS_CTRL_INTF_DATAINTF_OFFSET] = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_SS_DATA_INTF_NUM_OFFSET]      = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_SS_OUTEP_NUM_OFFSET]          = pDbgCfg->cdcEpOut;
            pCfgDscr[descLength + CYFX_CDC_SS_INEP_NUM_OFFSET]           = pDbgCfg->cdcEpIn | 0x80u;

            Cy_Debug_SetCdcCtrlIntfNumber(CY_USBD_USB_DEV_SS_GEN1, numIntf);
            retVal = true;
        }

        /* HS configuration update */
        if (pUsbdCtxt->dscrs.pUsbHsCfgDscr != NULL) {
            pCfgDscr   = (uint8_t *)pUsbdCtxt->dscrs.pUsbHsCfgDscr;
            numIntf    = pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET];
            descLength = ((pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] << 8) |
                         (pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET]));

            memcpy((pCfgDscr + descLength), CdcIntfDescriptor_HS, CYFX_CDC_HS_CFG_DSCR_LEN);

            /* Update required fields in the configuration descriptor. */
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET] = ((descLength + CYFX_CDC_HS_CFG_DSCR_LEN) & 0xFFu);
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] = ((descLength + CYFX_CDC_HS_CFG_DSCR_LEN) >> 8u);
            pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET]      = numIntf + 2;

            /* Update required fields in the CDC descriptor. */
            pCfgDscr[descLength + CYFX_CDC_HS_IAD_INTFNUM_OFFSET]        = numIntf;
            pCfgDscr[descLength + CYFX_CDC_HS_CTRL_INTF_NUM_OFFSET]      = numIntf;
            pCfgDscr[descLength + CYFX_CDC_HS_CTRL_INTF_DATAINTF_OFFSET] = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_HS_DATA_INTF_NUM_OFFSET]      = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_HS_OUTEP_NUM_OFFSET]          = pDbgCfg->cdcEpOut;
            pCfgDscr[descLength + CYFX_CDC_HS_INEP_NUM_OFFSET]           = pDbgCfg->cdcEpIn | 0x80u;

            Cy_Debug_SetCdcCtrlIntfNumber(CY_USBD_USB_DEV_HS, numIntf);
            retVal = true;
        }

        /* FS configuration update */
        if (pUsbdCtxt->dscrs.pUsbFsCfgDscr != NULL) {
            pCfgDscr   = (uint8_t *)pUsbdCtxt->dscrs.pUsbFsCfgDscr;
            numIntf    = pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET];
            descLength = ((pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] << 8) |
                         (pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET]));

            memcpy((pCfgDscr + descLength), CdcIntfDescriptor_FS, CYFX_CDC_HS_CFG_DSCR_LEN);

            /* Update required fields in the configuration descriptor. */
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_LSB_OFFSET] = ((descLength + CYFX_CDC_HS_CFG_DSCR_LEN) & 0xFFu);
            pCfgDscr[CYFX_CDC_CFG_DSCR_LEN_MSB_OFFSET] = ((descLength + CYFX_CDC_HS_CFG_DSCR_LEN) >> 8u);
            pCfgDscr[CYFX_CDC_CFG_NUMINTF_OFFSET]      = numIntf + 2;

            /* Update required fields in the CDC descriptor. */
            pCfgDscr[descLength + CYFX_CDC_HS_IAD_INTFNUM_OFFSET]        = numIntf;
            pCfgDscr[descLength + CYFX_CDC_HS_CTRL_INTF_NUM_OFFSET]      = numIntf;
            pCfgDscr[descLength + CYFX_CDC_HS_CTRL_INTF_DATAINTF_OFFSET] = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_HS_DATA_INTF_NUM_OFFSET]      = numIntf + 1;
            pCfgDscr[descLength + CYFX_CDC_HS_OUTEP_NUM_OFFSET]          = pDbgCfg->cdcEpOut;
            pCfgDscr[descLength + CYFX_CDC_HS_INEP_NUM_OFFSET]           = pDbgCfg->cdcEpIn | 0x80u;

            Cy_Debug_SetCdcCtrlIntfNumber(CY_USBD_USB_DEV_FS, numIntf);
            retVal = true;
        }
    }

    /* Return true if at least one configuration descriptor was updated. */
    return retVal;
}

/* CDC requests which are known and handled at this level. */
#define CDC_RQT_SET_LINE_CODING         (0x20)
#define CDC_RQT_GET_LINE_CODING         (0x21)
#define CDC_RQT_SET_CTRL_LINE_STATE     (0x22)

/*******************************************************************************
 * Function name: Cy_USB_CdcHandleControlRequests
 ****************************************************************************//**
 *
 * Function to be called from the setup callback to handle class specific control
 * requests addressed to the USB CDC interface.
 *
 * \param pCdcCtxt
 * USB CDC device context structure
 * \param pUsbdCtxt
 * USB stack context structure
 *
 * \return
 * true if a valid control request was handled locally, false otherwise.
 *
 *******************************************************************************/
bool Cy_USB_CdcHandleControlRequests (
        cy_stc_usb_cdc_ctxt_t  *pCdcCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    uint16_t wLength, wIndex;
    uint8_t  bRequest, bReqType, bTarget;
    uint8_t  cdcCtrlIf = 0;
    bool     isReqHandled = false;

    if ((pUsbdCtxt == NULL) || (pCdcCtxt == NULL) || (pCdcCtxt->pCfgBuffer == NULL)) {
        return false;
    }

    /* Identify the CDC control interface number for the active USB connection type. */
    if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        cdcCtrlIf = pCdcCtxt->ssCtrlIntf;
    }
    else {
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
            cdcCtrlIf = pCdcCtxt->hsCtrlIntf;
        } else {
            cdcCtrlIf = pCdcCtxt->fsCtrlIntf;
        }
    }

    /* Extract details about control request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wLength  = pUsbdCtxt->setupReq.wLength;
    wIndex   = pUsbdCtxt->setupReq.wIndex;
    bTarget  = (bReqType & CY_USB_CTRL_REQ_RECIPENT_MASK);

    /* We will only handle class specific requests addressed to either of the CDC control or data interfaces. */
    if (
            (((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS) == CY_USB_CTRL_REQ_CLASS) &&
            ((wIndex == cdcCtrlIf) || (wIndex == (cdcCtrlIf + 1))) &&
            (bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF)
       ) {
        switch (bRequest) {
            case CDC_RQT_GET_LINE_CODING:
                {
                    if (wLength <= 8) {
                        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, pCdcCtxt->pCfgBuffer, wLength);
                        if (retStatus == CY_USBD_STATUS_SUCCESS) {
                            isReqHandled = true;
                        }
                    } else {
                        /* Invalid wLength. Stall the request. */
                        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN, true);
                        isReqHandled = true;
                    }
                }
                break;

            case CDC_RQT_SET_LINE_CODING:
                {
                    if (wLength <= 8) {
                        /* Since this is a virtual CDC device, there is no actual interface to be configured.
                         * When new configuration info is received from host, it is just applied in the local
                         * RAM structure and returned as the current configuration to the host.
                         */
                        retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, pCdcCtxt->pCfgBuffer, wLength);
                        if (retStatus == CY_USBD_STATUS_SUCCESS) {
                            isReqHandled = true;
                        }
                    } else {
                        /* Invalid wLength. Stall the request. */
                        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0, CY_USB_ENDP_DIR_OUT, true);
                        isReqHandled = true;
                    }
                }
                break;

            case CDC_RQT_SET_CTRL_LINE_STATE:
                {
                    if (wLength == 0) {
                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                        isReqHandled = true;
                    } else {
                        /* Invalid wLength. Stall the request. */
                        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN, true);
                        isReqHandled = true;
                    }
                }
                break;

            default: /* Unknown request: Pass it back to stack level for error handling. */
                isReqHandled = false;
                break;

        }
    }

    return isReqHandled;
}

#if defined(__cplusplus)
}
#endif

/*[]*/

