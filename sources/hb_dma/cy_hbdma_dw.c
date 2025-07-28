/***************************************************************************//**
* \file cy_hbdma_dw.c
* \version 1.0
*
* Implements a set of wrapper functions which enable the use of DataWire
* DMA channels to transfer data from/to the USBHS block endpoint memories.
*
* Limitations associated with trigger connections require that only specific
* DataWire channels be used to transfer data from/to each specific USB
* endpoint.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_device.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_debug.h"
#include "cy_usb_common.h"
#include "cy_usb_usbd.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_HBDma_DW_Configure
 ****************************************************************************//**
 *
 * Configure the trigger connections for the DataWire used for transfers
 * through a USB High-Speed Endpoint.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param enable
 * Whether trigger connections are to be enabled or disabled.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_Configure (
        cy_stc_hbdma_channel_t *pHandle,
        bool enable)
{
    cy_en_hbdma_mgr_status_t stat = CY_HBDMA_MGR_BAD_PARAM;
    uint32_t i, epNum;

    if (pHandle != NULL) {
        /* Make trigger connections for USB-HS producer (Ingress) channels. */
        if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP) {
            for (i = 0; i < pHandle->prodSckCount; i++) {
                if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[i])) {
                    epNum = pHandle->prodSckId[i] - CY_HBDMA_USBHS_OUT_EP_00;

                    if (enable) {
                        /* Make the trigger connections from USB endpoint to DataWire. */
                        Cy_TrigMux_Connect(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + epNum,
                                TRIG_OUT_MUX_0_PDMA0_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                        /* Make the trigger connections from DataWire to USB endpoint. */
                        Cy_TrigMux_Connect(TRIG_IN_MUX_8_PDMA0_TR_OUT0 + epNum,
                                TRIG_OUT_MUX_8_USBHSDEV_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                    } else {
                        /* Make sure the DataWire channel is disabled. */
                        Cy_DMA_Channel_Disable(DW0, epNum);
                        Cy_DMA_Channel_DeInit(DW0, epNum);

                        /* Break the trigger connections. */
                        Cy_TrigMux_Connect((TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 & 0xFFFFFF00UL),
                                TRIG_OUT_MUX_0_PDMA0_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                        Cy_TrigMux_Connect((TRIG_IN_MUX_8_PDMA0_TR_OUT0 & 0xFFFFFF00UL),
                                TRIG_OUT_MUX_8_USBHSDEV_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                    }

                    pHandle->ingressDWRqtQueued[i] = false;
                    pHandle->curIngressXferSize[i] = 0;
                }
            }
        }

        /* Make trigger connections for USB-HS consumer (Egress) channels. */
        if (pHandle->type != CY_HBDMA_TYPE_IP_TO_MEM) {
            for (i = 0; i < pHandle->consSckCount; i++) {
                if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[i])) {
                    epNum = pHandle->consSckId[i] - CY_HBDMA_USBHS_IN_EP_00;

                    if (enable) {
                        /* Make the trigger connections from USB endpoint to DataWire. */
                        Cy_TrigMux_Connect(TRIG_IN_MUX_1_USBHSDEV_TR_OUT16 + epNum,
                                TRIG_OUT_MUX_1_PDMA1_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                        /* Make the trigger connections from DataWire to USB endpoint. */
                        Cy_TrigMux_Connect(TRIG_IN_MUX_8_PDMA1_TR_OUT0 + epNum,
                                TRIG_OUT_MUX_8_USBHSDEV_TR_IN16 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                    } else {
                        /* Make sure the DataWire channel is disabled. */
                        Cy_DMA_Channel_Disable(DW1, epNum);
                        Cy_DMA_Channel_DeInit(DW1, epNum);

                        /* Break the trigger connections. */
                        Cy_TrigMux_Connect((TRIG_IN_MUX_1_USBHSDEV_TR_OUT16 & 0xFFFFFF00UL),
                                TRIG_OUT_MUX_1_PDMA1_TR_IN0 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                        Cy_TrigMux_Connect((TRIG_IN_MUX_8_PDMA1_TR_OUT0 & 0xFFFFFF00UL),
                                TRIG_OUT_MUX_8_USBHSDEV_TR_IN16 + epNum,
                                false, TRIGGER_TYPE_LEVEL);
                    }

                    pHandle->egressDWRqtQueued[i] = false;
                    pHandle->pCurEgressDataBuf[i] = NULL;
                    pHandle->curEgressXferSize[i] = 0;
                }
            }
        }
    }

    return stat;
}

/*******************************************************************************
 * Function name: Cy_HBDma_DW_QueueRead
 ****************************************************************************//**
 *
 * Function which queues read operation using DataWire DMA on USBHS OUT endpoint
 * corresponding to a DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param prodIndex
 * Index of the producer from which to read data.
 * \param pBuffer
 * Pointer to the data buffer to read data into.
 * \param dataSize
 * Size of data expected. This should be a multiple of the max packet size.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_QueueRead (
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 prodIndex,
        uint8_t                *pBuffer,
        uint16_t                dataSize)
{
    cy_stc_dma_descriptor_t        *pFirstDscr;
    cy_stc_dma_descriptor_config_t  dmaDscrConfig;
    cy_stc_dma_channel_config_t     dmaChannelConfig;
    uint32_t                       *epmAddr;
    uint16_t                        residue;
    uint16_t                        nFullPkts;
    uint8_t                         epNum;
    cy_stc_usb_usbd_ctxt_t         *pUsbdCtxt = NULL;

    /* Parameter validity checks. */
    if ((pHandle == NULL) || (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) || (prodIndex >= pHandle->prodSckCount) ||
            (!CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[prodIndex])) || (pBuffer == NULL) || (dataSize == 0)) {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* DMA channel status check. */
    if (((pHandle->state != CY_HBDMA_CHN_ACTIVE) && (pHandle->state != CY_HBDMA_CHN_OVERRIDE)) ||
            (pHandle->ingressDWRqtQueued[prodIndex])) {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)(pHandle->pContext->pUsbStackCtx);

    /* Maximum number of packets per descriptor is 256. */
    if (dataSize >= (pHandle->epMaxPktSize << 8U)) {
        nFullPkts = 256U;
    } else {
        nFullPkts = (dataSize / pHandle->epMaxPktSize);
    }

    residue    = (dataSize % pHandle->epMaxPktSize);
    epNum      = pHandle->prodSckId[prodIndex] - CY_HBDMA_USBHS_OUT_EP_00;
    epmAddr    = Cy_USBHS_CalculateEpmAddr(epNum, CY_USB_ENDP_DIR_OUT);
    pFirstDscr = &(pHandle->pProdDwDscr[prodIndex][1]);

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMA_Channel_Disable(DW0, epNum);

    /* If 1 or more full packets are to be read out, set up the 2-D DMA descriptor. */
    if (nFullPkts != 0) {
        pFirstDscr = &(pHandle->pProdDwDscr[prodIndex][0]);

        dmaDscrConfig.retrigger        = CY_DMA_WAIT_FOR_REACT;
        dmaDscrConfig.interruptType    = CY_DMA_DESCR_CHAIN;
        dmaDscrConfig.triggerInType    = CY_DMA_X_LOOP;
        dmaDscrConfig.triggerOutType   = CY_DMA_X_LOOP;
        dmaDscrConfig.channelState     = (residue != 0) ? CY_DMA_CHANNEL_ENABLED : CY_DMA_CHANNEL_DISABLED;
        dmaDscrConfig.dataSize         = CY_DMA_WORD;
        dmaDscrConfig.srcTransferSize  = CY_DMA_TRANSFER_SIZE_DATA;
        dmaDscrConfig.dstTransferSize  = CY_DMA_TRANSFER_SIZE_DATA;
        dmaDscrConfig.descriptorType   = CY_DMA_2D_TRANSFER;
        dmaDscrConfig.srcAddress       = (void *)epmAddr;
        dmaDscrConfig.srcXincrement    = 1u;
        dmaDscrConfig.srcYincrement    = 0;
        dmaDscrConfig.dstAddress       = (void *)pBuffer;
        dmaDscrConfig.dstXincrement    = 1u;
        dmaDscrConfig.dstYincrement    = (pHandle->epMaxPktSize >> 2u);
        dmaDscrConfig.xCount           = (pHandle->epMaxPktSize >> 2u);
        dmaDscrConfig.yCount           = nFullPkts;
        if (residue != 0) {
            if (residue >= 0x04U) {
                dmaDscrConfig.nextDescriptor = &(pHandle->pProdDwDscr[prodIndex][1]);
            } else {
                dmaDscrConfig.nextDescriptor = &(pHandle->pProdDwDscr[prodIndex][2]);
            }
        } else {
            dmaDscrConfig.nextDescriptor = NULL;
        }

        Cy_DMA_Descriptor_Init(pFirstDscr, &dmaDscrConfig);
    } else {
        /* Zero out descriptor[0] since it is not being used. */
        memset((uint8_t *)(pHandle->pProdDwDscr[prodIndex]), 0, sizeof(cy_stc_dma_descriptor_t));
    }

    /* If there is a short packet to be read at the end, set up descriptors as required. */
    if (residue != 0) {
        if (residue >= 0x04U) {
            dmaDscrConfig.retrigger            = CY_DMA_WAIT_FOR_REACT;
            dmaDscrConfig.interruptType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.dataSize             = CY_DMA_WORD;
            dmaDscrConfig.srcTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMA_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = epmAddr;
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(pBuffer + nFullPkts * pHandle->epMaxPktSize);
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue >> 2u);
            dmaDscrConfig.yCount               = 0;
            if ((residue & 0x03U) != 0) {
                dmaDscrConfig.channelState     = CY_DMA_CHANNEL_ENABLED;
                dmaDscrConfig.nextDescriptor   = &(pHandle->pProdDwDscr[prodIndex][2]);
            } else {
                dmaDscrConfig.channelState     = CY_DMA_CHANNEL_DISABLED;
                dmaDscrConfig.nextDescriptor   = 0;
            }
            Cy_DMA_Descriptor_Init(&(pHandle->pProdDwDscr[prodIndex][1]), &dmaDscrConfig);
        } else {
            if (nFullPkts == 0) {
                pFirstDscr = &(pHandle->pProdDwDscr[prodIndex][2]);
            }
        }

        if ((residue & 0x03U) != 0) {
            dmaDscrConfig.retrigger            = CY_DMA_RETRIG_IM;
            dmaDscrConfig.interruptType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.channelState         = CY_DMA_CHANNEL_DISABLED;
            dmaDscrConfig.dataSize             = CY_DMA_BYTE;
            dmaDscrConfig.srcTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMA_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(epmAddr + (residue >> 2u));
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(pBuffer + (dataSize & 0xFFFCU));
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue & 0x03U);
            dmaDscrConfig.yCount               = 0;
            dmaDscrConfig.nextDescriptor       = NULL;
            Cy_DMA_Descriptor_Init(&(pHandle->pProdDwDscr[prodIndex][2]), &dmaDscrConfig);
        }
    }

    dmaChannelConfig.descriptor  = pFirstDscr;
    dmaChannelConfig.priority    = 3;
    dmaChannelConfig.enable      = false;
    dmaChannelConfig.bufferable  = false;
    dmaChannelConfig.preemptable = false;
    Cy_DMA_Channel_Init(DW0, epNum, &dmaChannelConfig);
    Cy_DMA_Channel_SetInterruptMask(DW0, epNum, CY_DMA_INTR_MASK);

    pHandle->ingressDWRqtQueued[prodIndex] = true;
    if (nFullPkts != 0) {
        pHandle->curIngressXferSize[prodIndex] = dataSize;
    }
    Cy_DMA_Channel_Enable(DW0, epNum);

    if (Cy_USBD_GetEndpointType(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT) != CY_USB_ENDP_TYPE_ISO) {
        /* Set transfer size for the endpoint and clear NAK status to allow data to be received. */
        if (dataSize < pHandle->epMaxPktSize) {
            /* We are trying to read out data that has already been received. Force EP NAK. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT, true);
        } else {
            /* Rounding dataSize upto nearest multiple of max. packet size. */
            uint16_t xferCnt = ((nFullPkts + (residue != 0)) * pHandle->epMaxPktSize);

            /* Set the NAK bit so that the IP can see the bit transition from 1->0 after XFER_CNT is set. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT, true);
            Cy_USBD_UpdateXferCount(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT, xferCnt);
            Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT, false);
        }
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_HBDma_DW_CompleteShortRead
 ****************************************************************************//**
 *
 * Function which terminates ongoing USBHS ingress transfer when a short packet
 * has been received on the endpoint.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param prodIndex
 * Index of the producer from which to read data.
 * \param shortPktSize
 * Size of the short packet received in bytes.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_CompleteShortRead (
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 prodIndex,
        uint16_t                shortPktSize)
{
    cy_stc_dma_descriptor_t *activeDscr;
    cy_stc_dma_descriptor_t *prevDscr;
    uint8_t                 *pDataBuf = NULL;
    uint16_t                 dataSize = 0;
    uint8_t                  epNum;
    cy_stc_usb_usbd_ctxt_t  *pUsbdCtxt = NULL;

    /* Parameter validity checks. */
    if ((pHandle == NULL) || (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) || (prodIndex >= pHandle->prodSckCount) ||
            (!CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[prodIndex]))) {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* A read operation should already have been queued. */
    if (pHandle->ingressDWRqtQueued[prodIndex] == false) {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)(pHandle->pContext->pUsbStackCtx);
    epNum = pHandle->prodSckId[prodIndex] - CY_HBDMA_USBHS_OUT_EP_00;

    if (Cy_USBD_GetEndpointType(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT) != CY_USB_ENDP_TYPE_ISO) {
        /* NAK the endpoint until the next DMA request is queued. */
        Cy_USB_USBD_EndpSetClearNakNrdy(pUsbdCtxt, epNum, CY_USB_ENDP_DIR_OUT, true);
    }

    /* Get the current descriptor and check whether it is the 2-D descriptor. */
    activeDscr = Cy_DMA_Channel_GetCurrentDescriptor(DW0, epNum);
    if (
            (activeDscr != &(pHandle->pProdDwDscr[prodIndex][0])) &&
            (activeDscr != &(pHandle->pProdDwDscr[prodIndex][1]))
       ) {
        DBG_HBDMA_ERR("DW0 channel %d pointing to unexpected DMA descriptor %x\r\n", epNum, activeDscr);
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    if (activeDscr == &(pHandle->pProdDwDscr[prodIndex][0])) {
        /*
         * Channel is still working on Descriptor #0. Calculate the size of data received so far by
         * checking the current Y index of the channel. Then disable and re-enable the DMA channel with
         * the appropriate buffer address and short packet size.
         */
        dataSize = Cy_DMA_Channel_GetCurrentYIndex(DW0, epNum) * pHandle->epMaxPktSize;
        pDataBuf = (uint8_t *)Cy_DMA_Descriptor_GetDstAddress(activeDscr);
        pDataBuf += dataSize;
        dataSize += shortPktSize;
    } else {
        prevDscr = &(pHandle->pProdDwDscr[prodIndex][0]);

        /*
         * Descriptor #0 is complete. We can take all the data corresponding to Descriptor #0 as received.
         * Read is to be resumed from the destination address programmed in Descriptor #1.
         */
        if (prevDscr->ctl != 0) {
            dataSize = Cy_DMA_Descriptor_GetYloopDataCount(prevDscr) * pHandle->epMaxPktSize;
        } else {
            /* Descriptor #0 was not in use. */
            dataSize = 0;
        }

        pDataBuf = (uint8_t *)Cy_DMA_Descriptor_GetDstAddress(activeDscr);
        dataSize += shortPktSize;
    }

    /* Save the updated data transfer size in the channel structure. */
    pHandle->curIngressXferSize[prodIndex] = dataSize;

    if (shortPktSize != 0) {
        /* Clear the transfer pending flag at this point as QueueRead will set the flag again. */
        pHandle->ingressDWRqtQueued[prodIndex] = false;
        Cy_HBDma_DW_QueueRead(pHandle, prodIndex, pDataBuf, shortPktSize);

        /* Send trigger to DataWire channel so that the partial packet is read out. */
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + epNum, CY_TRIGGER_TWO_CYCLES);
    } else {
        /* Disable the DMA channel. */
        Cy_DMA_Channel_Disable(DW0, epNum);

        /* We need to trigger the DMA completion interrupt manually here. */
        Cy_DMA_Channel_SetInterrupt(DW0, epNum);
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_HBDma_DW_QueueWrite
 ****************************************************************************//**
 *
 * Function which queues write operation using DataWire DMA on USBHS IN endpoint
 * corresponding to a DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param consIndex
 * Index of the consumer to write data into.
 * \param pBuffer
 * Pointer to the data buffer containing the data.
 * \param dataSize
 * Size of data to be transferred.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_QueueWrite (
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 consIndex,
        uint8_t                *pBuffer,
        uint16_t                dataSize)
{
    cy_stc_dma_descriptor_t        *pFirstDscr;
    cy_stc_dma_descriptor_config_t  dmaDscrConfig;
    cy_stc_dma_channel_config_t     dmaChannelConfig;
    uint32_t                       *epmAddr;
    uint16_t                        residue;
    uint16_t                        nFullPkts;
    uint8_t                         epNum;
    cy_stc_usb_usbd_ctxt_t         *pUsbdCtxt = NULL;

    /* Parameter validity checks. */
    if ((pHandle == NULL) || (pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) || (consIndex >= pHandle->consSckCount) ||
            (!CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[consIndex])) || (pBuffer == NULL)) {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* DMA channel status check. */
    if (((pHandle->state != CY_HBDMA_CHN_ACTIVE) && (pHandle->state != CY_HBDMA_CHN_OVERRIDE)) ||
            (pHandle->egressDWRqtQueued[consIndex])) {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    /* Store the current transfer details. */
    pHandle->pCurEgressDataBuf[consIndex] = pBuffer;
    pHandle->curEgressXferSize[consIndex] = dataSize;

    if (dataSize == 0) {
        pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pHandle->pContext->pUsbStackCtx;
        epNum     = pHandle->consSckId[consIndex] - CY_HBDMA_USBHS_IN_EP_00;

        pHandle->egressDWRqtQueued[consIndex] = true;
        Cy_USBD_SendEgressZLP(pUsbdCtxt, epNum);

        /* We need to trigger an interrupt on the DataWire channel to indicate transfer completion. */
        Cy_DMA_Channel_SetInterruptMask(DW1, epNum, CY_DMA_INTR_MASK);
        Cy_DMA_Channel_SetInterrupt(DW1, epNum);
        return CY_HBDMA_MGR_SUCCESS;
    }

    /* Maximum number of packets per descriptor is 256. */
    if (dataSize >= (pHandle->epMaxPktSize << 8U)) {
        nFullPkts = 256U;
    } else {
        nFullPkts = (dataSize / pHandle->epMaxPktSize);
    }

    epNum      = pHandle->consSckId[consIndex] - CY_HBDMA_USBHS_IN_EP_00;
    residue    = (dataSize % pHandle->epMaxPktSize);
    epmAddr    = Cy_USBHS_CalculateEpmAddr(epNum, CY_USB_ENDP_DIR_IN);
    pFirstDscr = &(pHandle->pConsDwDscr[consIndex][1]);

    /* If the DMA channel is already enabled, disable it. */
    Cy_DMA_Channel_Disable(DW1, epNum);

    /* If 1 or more full packets are to be written out, set up the 2-D DMA descriptor. */
    if (nFullPkts != 0) {
        pFirstDscr = &(pHandle->pConsDwDscr[consIndex][0]);

        dmaDscrConfig.retrigger            = CY_DMA_WAIT_FOR_REACT;
        dmaDscrConfig.interruptType        = CY_DMA_DESCR_CHAIN;
        dmaDscrConfig.triggerInType        = CY_DMA_X_LOOP;
        dmaDscrConfig.triggerOutType       = CY_DMA_X_LOOP;
        dmaDscrConfig.channelState         = (residue != 0) ? CY_DMA_CHANNEL_ENABLED : CY_DMA_CHANNEL_DISABLED;
        dmaDscrConfig.dataSize             = CY_DMA_WORD;
        dmaDscrConfig.srcTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
        dmaDscrConfig.dstTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
        dmaDscrConfig.descriptorType       = CY_DMA_2D_TRANSFER;
        dmaDscrConfig.srcAddress           = (void *)pBuffer;
        dmaDscrConfig.srcXincrement        = 1u;
        dmaDscrConfig.srcYincrement        = (pHandle->epMaxPktSize >> 2u);
        dmaDscrConfig.dstAddress           = (void *)epmAddr;
        dmaDscrConfig.dstXincrement        = 1u;
        dmaDscrConfig.dstYincrement        = 0;
        dmaDscrConfig.xCount               = (pHandle->epMaxPktSize >> 2u);
        dmaDscrConfig.yCount               = nFullPkts;
        if (residue != 0) {
            if (residue >= 4U) {
                dmaDscrConfig.nextDescriptor = &(pHandle->pConsDwDscr[consIndex][1]);
            } else {
                dmaDscrConfig.nextDescriptor = &(pHandle->pConsDwDscr[consIndex][2]);
            }
        } else {
            dmaDscrConfig.nextDescriptor   = NULL;
        }

        Cy_DMA_Descriptor_Init(pFirstDscr, &dmaDscrConfig);
    }

    /* If there is a short packet to be read at the end, set up descriptors as required. */
    if (residue != 0) {
        if (residue >= 0x04U) {
            dmaDscrConfig.retrigger            = CY_DMA_WAIT_FOR_REACT;
            dmaDscrConfig.interruptType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.dataSize             = CY_DMA_WORD;
            dmaDscrConfig.srcTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMA_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(pBuffer + nFullPkts * pHandle->epMaxPktSize);
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)epmAddr;
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue >> 2u);
            dmaDscrConfig.yCount               = 0;
            if ((residue & 0x03U) != 0) {
                dmaDscrConfig.channelState     = CY_DMA_CHANNEL_ENABLED;
                dmaDscrConfig.nextDescriptor   = &(pHandle->pConsDwDscr[consIndex][2]);
            } else {
                dmaDscrConfig.channelState     = CY_DMA_CHANNEL_DISABLED;
                dmaDscrConfig.nextDescriptor   = NULL;
            }
            Cy_DMA_Descriptor_Init(&(pHandle->pConsDwDscr[consIndex][1]), &dmaDscrConfig);
        } else {
            if (nFullPkts == 0) {
                pFirstDscr = &(pHandle->pConsDwDscr[consIndex][2]);
            }
        }

        if ((residue & 0x03U) != 0) {
            dmaDscrConfig.retrigger            = CY_DMA_RETRIG_IM;
            dmaDscrConfig.interruptType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMA_DESCR_CHAIN;
            dmaDscrConfig.channelState         = CY_DMA_CHANNEL_DISABLED;
            dmaDscrConfig.dataSize             = CY_DMA_BYTE;
            dmaDscrConfig.srcTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMA_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMA_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(pBuffer + (dataSize & 0xFFFCU));
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(epmAddr + (residue >> 2u));
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue & 0x03U);
            dmaDscrConfig.yCount               = 0;
            dmaDscrConfig.nextDescriptor       = NULL;
            Cy_DMA_Descriptor_Init(&(pHandle->pConsDwDscr[consIndex][2]), &dmaDscrConfig);
        }
    }

    dmaChannelConfig.descriptor  = pFirstDscr;
    dmaChannelConfig.priority    = 3;
    dmaChannelConfig.enable      = false;
    dmaChannelConfig.bufferable  = false;
    dmaChannelConfig.preemptable = false;
    Cy_DMA_Channel_Init(DW1, epNum, &dmaChannelConfig);
    Cy_DMA_Channel_SetInterruptMask(DW1, epNum, CY_DMA_INTR_MASK);

    /* Make sure the HBW SRAM read cache is flushed when starting a new DMA operation. */
    if (((uint32_t)pBuffer >= 0x1C000000UL) && ((uint32_t)pBuffer < 0x1C100000UL)) {
        MAIN_REG->CTRL |= MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk;
    }

    pHandle->egressDWRqtQueued[consIndex] = true;
    Cy_DMA_Channel_Enable(DW1, epNum);

    /* For the first transfer on an IN endpoint, we need to assert the trigger manually. */
    if (pHandle->egressDWTrigDone[consIndex] == false) {
        pHandle->egressDWTrigDone[consIndex] = true;
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_1_USBHSDEV_TR_OUT16 + epNum, CY_TRIGGER_TWO_CYCLES);
    }

    return CY_HBDMA_MGR_SUCCESS;
}

#if defined(__cplusplus)
}
#endif

/*[]*/

