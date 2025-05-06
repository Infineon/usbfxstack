/***************************************************************************//**
* \file cy_usb_cdc.c
* \version 1.0
*
* Provided API source for the CDC Debug Interface. Device Class implementation using
* the USB IP block.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
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
 * Function name: Cy_USB_CdcQueueRead
 ****************************************************************************//**
 *
 * The function queue read on OUT endpoint.
 *
 * \param pDbgCtxt
 * Debug Context.
 *
 * \param endpNumber
 * Out Endpoint number to queue the read
 *
 * \param pBuffer
 * Buffer pointer to store the read data.
 *
 * \param dataSize
 * Bytes to read on OUT endpoint
 *
 *
 *******************************************************************************/
void
Cy_USB_CdcQueueRead (cy_stc_debug_context_t *pDbgCtxt, uint8_t endpNumber,
                    uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    /* Null pointer checks. */
    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL) || (pBuffer == NULL)) {
        return;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t *)pDbgCtxt->usbCdcInfo;
    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t *)usbCdcInfo->pUsbdCtxt;

    if ((pUsbdCtxt == NULL) || (usbCdcInfo->pCpuDw0Base == NULL)) {
        return;
    }

    if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)
    {
        /*
        * Verify that the selected endpoint is valid and
        * dataSize is non-zero.
        */
        dmaset_p = &(usbCdcInfo->endpOutDma);
        if ((dmaset_p->valid == 0) || (dataSize == 0))
        {
            return;
        }

        if (Cy_USBHS_App_QueueRead(dmaset_p, pBuffer, dataSize))
        {
            /* Set transfer size for the endpoint and clear NAK status to allow data to be received. */
            if (dataSize < dmaset_p->maxPktSize) {
                /* We are trying to read out data that has already been received. Force EP NAK. */
                Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
            } else {
                /* Set the NAK bit so that the IP can see the bit transition from 1->0 after XFER_CNT is set. */
                Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
                Cy_USBD_UpdateXferCount(pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, dataSize);
                Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, false);
            }
        }
    }
} /* end of function */

/*******************************************************************************
 * Function name: Cy_USB_CdcQueueWrite
 ****************************************************************************//**
 *
 * The function queue Write on IN endpoint.
 *
 * \param pDbgCtxt
 * Debug Context.
 *
 * \param endpNumber
 * IN Endpoint number to queue the write
 *
 * \param pBuffer
 * Buffer pointer to write the data.
 *
 * \param dataSize
 * Bytes to write on IN endpoint
 *
 *
 *******************************************************************************/
bool Cy_USB_CdcQueueWrite(cy_stc_debug_context_t *pDbgCtxt, uint8_t endpNumber,
    uint8_t* pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t* dmaset_p;
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    bool stat = 0;

    /* Null pointer checks. */
    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL) || (pBuffer == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    if (usbCdcInfo->pCpuDw1Base == NULL) {
        return false;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(usbCdcInfo->endpInDma);
    if ((dmaset_p->valid == 0) || (dataSize == 0)) {
        return false;
    }

    stat = Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
    return stat;
} /* end of function */


/*******************************************************************************
 * Function name: Cy_USB_CdcChannelReset
 ****************************************************************************//**
 *
 * Function to de-initialize CDC interface for debug los
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

        if (usbCdcInfo->cdcSendHandle != NULL && pUsbdCtxt != NULL)
        {
            if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
                Cy_HBDma_Channel_Reset(usbCdcInfo->cdcSendHandle);
            } else {
                Cy_USBHS_App_ResetEpDma(&(usbCdcInfo->endpInDma));
            }

            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN);
            Cy_USBD_ResetEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN, true);

            if((usbCdcInfo->recvEnabled) && (usbCdcInfo->cdcRecvHandle != NULL))
            {
                if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
                    Cy_HBDma_Channel_Reset(usbCdcInfo->cdcRecvHandle);
                } else {
                    Cy_USBHS_App_ResetEpDma(&(usbCdcInfo->endpOutDma));
                }

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
 *
 *******************************************************************************/
bool Cy_USB_CdcChannelEnable(cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_hbdma_buff_status_t dmaBufStat;
    cy_en_hbdma_mgr_status_t   hbdma_stat = CY_HBDMA_MGR_SUCCESS;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL))
    {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;

    if(usbCdcInfo->cdcSendHandle != NULL)
    {
        hbdma_stat = Cy_HBDma_Channel_Enable(usbCdcInfo->cdcSendHandle, 0);
        if(hbdma_stat != CY_HBDMA_MGR_SUCCESS)
        {
            return false;
        }

        hbdma_stat = Cy_HBDma_Channel_GetBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);
        if (hbdma_stat == CY_HBDMA_MGR_SUCCESS)
        {
            pDbgCtxt->pMsg = dmaBufStat.pBuffer;
        }

        if(usbCdcInfo->cdcRecvHandle != NULL)
        {
            hbdma_stat = Cy_HBDma_Channel_Enable(usbCdcInfo->cdcRecvHandle, 0);
            if(hbdma_stat != CY_HBDMA_MGR_SUCCESS)
            {
                return false;
            }
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
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

    if ((pUsbdCtxt == NULL) || (pDbgCtxt->pMsg == NULL) || (usbCdcInfo->cdcSendHandle == NULL))
    {
        return 0;
    }

    while (limit > rdPtr)
    {
        size = ((limit - rdPtr) > pDbgCtxt->maxDmaSize) ? pDbgCtxt->maxDmaSize : (limit - rdPtr);
        if(pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)
        {
            dmaBufStat.pBuffer = pDbgCtxt->pMsg+rdPtr;
            dmaBufStat.size    = pDbgCtxt->bufSize;
            dmaBufStat.count   = size;
            dmaBufStat.status  = 0;
            hbdma_stat = Cy_HBDma_Channel_CommitBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);
        }
        else
        {
           Cy_USB_CdcQueueWrite(pDbgCtxt, usbCdcInfo->cdcEpIn, pDbgCtxt->pMsg + rdPtr, size);
        }

        if(hbdma_stat == CY_HBDMA_MGR_SUCCESS )
        {
            hbdma_stat = Cy_HBDma_Channel_GetBuffer(usbCdcInfo->cdcSendHandle, &dmaBufStat);
            pDbgCtxt->pMsg = dmaBufStat.pBuffer;
        }

        if(hbdma_stat != CY_HBDMA_MGR_SUCCESS)
        {
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
 * Function name: Cy_USB_CdcClearDmaInterrupt
 ****************************************************************************//**
 *
 * The function clear the DMA interrrupt associated to the endpoint
 *
 * \param pDbgCtxt
 * Debug Context.
 *
 * \param endpDirection
 * IN or OUT endpoint
 *
 * \param endpNumber
 *Endpoint Number
 *
 *
 *******************************************************************************/
void Cy_USB_CdcClearDmaInterrupt(cy_stc_debug_context_t* pDbgCtxt, uint8_t endpDirection, uint32_t endpNumber)
{
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;

    if ((pDbgCtxt != NULL) && (pDbgCtxt->usbCdcInfo != NULL)) {
        usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;

        if ((usbCdcInfo->cdcEpIn > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
            if (endpDirection == CY_USB_ENDP_DIR_IN) {
                Cy_USBHS_App_ClearDmaInterrupt(&(usbCdcInfo->endpInDma));
            } else {
                Cy_USBHS_App_ClearDmaInterrupt(&(usbCdcInfo->endpOutDma));
            }
        }
    }
} /* end of function. */

/*******************************************************************************
 * Function name: Cy_USB_CdcInitDmaInterrupt
 ****************************************************************************//**
 *
 * The function clear the DMA interrrupt associated to the endpoint
 *
 * \param endpDirection
 * IN or OUT endpoint
 *
 * \param endpNumber
 *Endpoint Number
 *
 * \param userIsr
 * USer ISR to register for the DMa interrupt.
 *
 *******************************************************************************/
void Cy_USB_CdcInitDmaInterrupt(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
    cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;

    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
#if (!CY_CPU_CORTEX_M4)
        intrCfg.intrPriority = 3;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
            intrCfg.intrSrc = NvicMux6_IRQn;
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
            intrCfg.intrSrc = NvicMux7_IRQn;
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
} /* end of function. */


/*******************************************************************************
 * Function name: Cy_USB_CdcInit
 ****************************************************************************//**
 *
 * Function to initialize CDC interface for debug los
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * True if initialization is successful else False
 *
 *******************************************************************************/
bool Cy_USB_CdcInit (cy_stc_debug_context_t *pDbgCtxt)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t   hbdma_stat = CY_HBDMA_MGR_SUCCESS;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    cy_stc_usb_cdc_ctxt_t *usbCdcInfo;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    if ((pDbgCtxt == NULL) || (pDbgCtxt->usbCdcInfo == NULL)) {
        return false;
    }

    usbCdcInfo = (cy_stc_usb_cdc_ctxt_t*)pDbgCtxt->usbCdcInfo;
    pUsbdCtxt  = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

    if ((pUsbdCtxt == NULL) || (usbCdcInfo->cdcEpIn <= 0u && usbCdcInfo->cdcEpIn >= CY_USB_MAX_ENDP_NUMBER)
       || ((usbCdcInfo->recvEnabled == 1) && (usbCdcInfo->cdcEpOut <= 0u && usbCdcInfo->cdcEpOut >= CY_USB_MAX_ENDP_NUMBER)))
    {
        return false;
    }

    dmaConfig.bufferMode     = false;                               /* DMA buffer mode disabled */
    dmaConfig.prodHdrSize    = 0;                                   /* No header being added. */
    dmaConfig.prodSckCount   = 1;                                   /* No. of producer sockets */
    dmaConfig.consSckCount   = 1;                                   /* No. of consumer Sockets */
    dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;             /* Producer Socket ID: None */
    dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;             /* Consumer Socket ID: None */
    dmaConfig.eventEnable    = 0;                                   /* TR_ASSIST connects to events from the adapter. */
    dmaConfig.userCtx        = NULL;                                /* Pass the application context as user context. */

    /* Create channel which will move data from SRAM to USBSS EPM. */
    dmaConfig.size           = pDbgCtxt->bufSize;
    dmaConfig.count          = usbCdcInfo->bufCount;                /* DMA Buffer Count */
    dmaConfig.prodBufSize    = pDbgCtxt->bufSize;
    dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;             /* DMA Channel type: from HB-RAM to USB3-IP */
    dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
    dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + usbCdcInfo->cdcEpIn);
    dmaConfig.cb             = Cy_USB_CdcDmaCallback;
    dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;

    hbdma_stat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr,
                                        &(usbCdcInfo->endpInDma.hbDmaChannel),
                                        &dmaConfig);
    if (hbdma_stat != CY_HBDMA_MGR_SUCCESS)
    {
        /* Not able to create Channel then return. */
        return false;
    }

    usbCdcInfo->cdcSendHandle  =  &usbCdcInfo->endpInDma.hbDmaChannel;

    /* CDC Bulk Endpoint Max packet size */
    usbCdcInfo->endpInDma.maxPktSize = 1024;
    usbCdcInfo->endpInDma.valid      = 0x01;

    if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)
    {
        /* CDC Bulk Endpoint Max packet size */
        usbCdcInfo->endpInDma.maxPktSize = 512;
        usbCdcInfo->endpInDma.valid      = 0x01;

        pEndpDmaSet = &(usbCdcInfo->endpInDma);
        Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, usbCdcInfo->pCpuDw1Base, usbCdcInfo->cdcEpIn,
                                    usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN, 512);

        /* Enable DMA interrupts for the VCOM OUT and IN endpoints. */
        Cy_USB_CdcInitDmaInterrupt(usbCdcInfo->cdcEpIn,CY_USB_ENDP_DIR_IN,Cy_USB_CdcEpInDmaISR);
    }

    if(usbCdcInfo->recvEnabled == 1)
    {
        dmaConfig.bufferMode     = false;                               /* DMA buffer mode disabled */
        dmaConfig.prodHdrSize    = 0;                                   /* No header being added. */
        dmaConfig.prodSckCount   = 1;                                   /* No. of producer sockets */
        dmaConfig.consSckCount   = 1;                                   /* No. of consumer Sockets */
        dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;             /* Producer Socket ID: None */
        dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;             /* Consumer Socket ID: None */
        dmaConfig.eventEnable    = 0;                                   /* TR_ASSIST connects to events from the adapter. */
        dmaConfig.userCtx        = NULL;                                /* Pass the application context as user context. */
        /* Create channel which moves data from USB ingress endpoint into HBW SRAM. */
        dmaConfig.size        = 1024;                                   /* DMA Buffer Size in bytes */
        dmaConfig.count       = usbCdcInfo->bufCount;                   /* DMA Buffer Count */
        dmaConfig.prodBufSize = 1024;
        dmaConfig.chType      = CY_HBDMA_TYPE_IP_TO_MEM;
        dmaConfig.prodSck[0]  = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + usbCdcInfo->cdcEpOut);
        dmaConfig.consSck[0]  = CY_HBDMA_VIRT_SOCKET_RD;
        dmaConfig.cb          = Cy_USB_CdcDmaCallback;                  /* HB-DMA callback */
        dmaConfig.intrEnable  = USB32DEV_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk;

        hbdma_stat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr,
                                             &(usbCdcInfo->endpOutDma.hbDmaChannel),
                                             &dmaConfig);
        if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
            /* Not able to create Channel then return. */
            return false;
        }

        usbCdcInfo->cdcRecvHandle  =  &usbCdcInfo->endpOutDma.hbDmaChannel;

        usbCdcInfo->endpOutDma.maxPktSize = 1024;
        usbCdcInfo->endpOutDma.valid      = 0x01;

        if(pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)
        {
            usbCdcInfo->endpOutDma.maxPktSize = 512;
            usbCdcInfo->endpOutDma.valid      = 0x01;

            Cy_USBD_DebugRegisterCallback(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_USBD_CB_SLP, Cy_USB_CdcSlpCallback);
            pEndpDmaSet = &(usbCdcInfo->endpOutDma);

            Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, usbCdcInfo->pCpuDw0Base,
                    usbCdcInfo->cdcEpOut, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT, 512);

            /* Enable DMA interrupts for the OUT endpoint */
            Cy_USB_CdcInitDmaInterrupt(usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT, Cy_USB_CdcEpOutDmaISR);
        }
    }
    return true;
}


/*******************************************************************************
 * Function name: Cy_USB_CdcDeInit
 ****************************************************************************//**
 *
 * Function to de-initialize CDC interface for debug logging.
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * True if deinitialization is successful else False
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
    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t*)usbCdcInfo->pUsbdCtxt;

    if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)
    {
        if (usbCdcInfo->endpInDma.valid)
        {
            /* DeInit the DMA channel and disconnect the triggers. */
            Cy_USBHS_App_DisableEpDmaSet(&(usbCdcInfo->endpInDma));
        }

        if (usbCdcInfo->endpOutDma.valid) {
            /* DeInit the DMA channel and disconnect the triggers. */
            Cy_USBHS_App_DisableEpDmaSet(&(usbCdcInfo->endpOutDma));
        }
    }

    if (usbCdcInfo->cdcSendHandle != NULL)
    {
        Cy_HBDma_Channel_Disable(usbCdcInfo->cdcSendHandle);
        Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcSendHandle);
        usbCdcInfo->cdcSendHandle = NULL;

        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpIn, CY_USB_ENDP_DIR_IN);
        }
    }

    if ((usbCdcInfo->recvEnabled) && (usbCdcInfo->cdcRecvHandle != NULL))
    {
        Cy_HBDma_Channel_Disable(usbCdcInfo->cdcSendHandle);
        Cy_HBDma_Channel_Destroy(usbCdcInfo->cdcSendHandle);
        usbCdcInfo->cdcRecvHandle = NULL;

        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)
        {
            Cy_USBD_FlushEndp(pUsbdCtxt, usbCdcInfo->cdcEpOut, CY_USB_ENDP_DIR_OUT);
        }
    }

    return true;
}

#if defined(__cplusplus)
}
#endif

/*[]*/

