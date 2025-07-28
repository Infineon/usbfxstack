/***************************************************************************//**
* \file cy_hbdma_mgr.c
* \version 1.0
*
* Implements the High BandWidth DMA manager library.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_hbdma_version.h"
#include "cy_debug.h"
#include "cy_gpio.h"
#include "cy_usb_usbd.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#endif /* FREERTOS_ENABLE */

#if defined(CY_IP_MXS40LVDS2USB32SS)

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_GetVersion
 ****************************************************************************//**
 *
 * Returns the High BandWidth Manager Middleware version information in the
 * form of a 32-bit word including the major, minor, patch and build numbers.
 *
 * \return
 * Version information for this library in the format:
 *         b31:28 -> Major Version
 *         b27:24 -> Minor Version
 *         b23:16 -> Patch Version
 *         b15:00 -> Build Number
 *******************************************************************************/
uint32_t
Cy_HBDma_Mgr_GetVersion(void)
{
    return HBDMA_VERSION_NUM;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_Init
 ****************************************************************************//**
 *
 * Initialize the High BandWidth DMA manager library.
 *
 * \param pContext
 * Pointer to the DMA manager context structure.
 *
 * \param pDrvContext
 * Pointer to the HBDMA driver context structure.
 *
 * \param pDscrPool
 * Pointer to the HBDMA descriptor list.
 *
 * \param pBufMgr
 * Pointer to the HBDMA buffer manager context.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the DMA manager init was successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_Init (
    cy_stc_hbdma_mgr_context_t *pContext,
    cy_stc_hbdma_context_t     *pDrvContext,
    cy_stc_hbdma_dscr_list_t   *pDscrPool,
    cy_stc_hbdma_buf_mgr_t     *pBufMgr)
{
#if FREERTOS_ENABLE
    BaseType_t status = pdFAIL;
#endif /* FREERTOS_ENABLE */

    if ((pContext == NULL) || (pDrvContext == NULL) || (pDscrPool == NULL) || (pBufMgr == NULL))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Zero out the context structure. */
    memset((void *)pContext, 0, sizeof(cy_stc_hbdma_mgr_context_t));

    pContext->pDrvContext  = pDrvContext;
    pContext->pDscrPool    = pDscrPool;
    pContext->pBufMgr      = pBufMgr;
    pContext->en_64k       = Cy_HBDma_Is64KBufferEnabled(pDrvContext);
    pContext->pUsbStackCtx = NULL;

    /* Allocate list of DataWire descriptors to be used for all USBHS channels. We need three
     * descriptors per DMA channel (ingress and egress). */
    pContext->dwDscrList = (cy_stc_dma_descriptor_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr,
            CY_HBDMA_DW_ADAP_CNT * CY_HBDMA_SOCK_PER_ADAPTER * CY_HBDMA_DW_DSCR_PER_CHN * sizeof(cy_stc_dma_descriptor_t));
    if (pContext->dwDscrList == NULL) {
        return CY_HBDMA_MGR_MEMORY_ERROR;
    }

    /* Register the callback to be called when DMA interrupts are received. */
    Cy_HBDma_SetInterruptCallback(pDrvContext, Cy_HBDma_Channel_Cb, (void*)pContext);

#if FREERTOS_ENABLE
    pContext->isrCtrlCb          = NULL;
    pContext->intrDisabled       = false;
    pContext->cbFromISREnable    = false;
    pContext->queueOverflowCount = 0;
    pContext->dmaIntrQueue       = xQueueCreate(CY_HBDMA_INTR_QUEUE_ENTRIES, sizeof(cy_stc_hbdma_intr_msg_t));

    if (pContext->dmaIntrQueue != NULL) {
        vQueueAddToRegistry(pContext->dmaIntrQueue, "HbDmaIntrQueue");
        status = xTaskCreate(Cy_HBDma_Mgr_TaskHandler, "HbDmaTask", CY_HBDMA_TASK_STACK_DEPTH, (void *)pContext,
                CY_HBDMA_TASK_PRIORITY, &(pContext->dmaMgrTask));
        if (status != pdPASS) {
            DBG_HBDMA_ERR("xTaskCreate failed: %d\r\n", status);
        }
    } else {
        DBG_HBDMA_ERR("xQueueCreate failed\r\n");
    }

    if (status != pdPASS) {
        Cy_HBDma_BufMgr_Free(pBufMgr, pContext->dwDscrList);
        pContext->dwDscrList = NULL;
        return CY_HBDMA_MGR_MEMORY_ERROR;
    }

    DBG_HBDMA_INFO("Created HbDmaTask\r\n");
#endif /* FREERTOS_ENABLE */

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_RegisterISRCtrlCallback
 ****************************************************************************//**
 *
 * Register a function callback that the DMA manager can call to dynamically
 * enable or disable the HBDMA interrupts based on the state of the interrupt
 * notification message queue.
 *
 * \param pContext
 * Pointer to the DMA manager context structure.
 *
 * \param cb
 * Pointer to function callback.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_HBDma_Mgr_RegisterISRCtrlCallback (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_cb_hbdma_mgr_intr_ctrl_cb_t cb)
{
#if FREERTOS_ENABLE
    if (pDmaMgr != NULL) {
        /* If callback is being unregistered and interrupt was disabled, make sure to enable it. */
        if ((cb == NULL) && (pDmaMgr->isrCtrlCb != NULL) && (pDmaMgr->intrDisabled == true)) {
            pDmaMgr->isrCtrlCb(true);
            pDmaMgr->intrDisabled = false;
        }

        pDmaMgr->isrCtrlCb = cb;
    }
#endif /* FREERTOS_ENABLE */
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_DeInit
 ****************************************************************************//**
 *
 * De-initialize the High BandWidth DMA manager library.
 *
 * \param pContext
 * Pointer to the DMA manager context structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the manager de-init is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the context structure passed is invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_DeInit (
    cy_stc_hbdma_mgr_context_t *pContext)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_BAD_PARAM;
    uint32_t i;

    if (pContext != NULL) {
        status = CY_HBDMA_MGR_SUCCESS;

        for (i = 0; i < CY_HBDMA_MAX_ADAP_CNT; i++) {
            if (pContext->socketUsed[i] != 0) {
                status = CY_HBDMA_MGR_SOCK_BUSY;
                break;
            }
        }

        if (status == CY_HBDMA_MGR_SUCCESS) {
            /* Free the DMA descriptor list. */
            if (pContext->dwDscrList != NULL) {
                Cy_HBDma_BufMgr_Free(pContext->pBufMgr, pContext->dwDscrList);
                pContext->dwDscrList = NULL;
            }

            /* Clear the ISR callback registered with the driver. */
            Cy_HBDma_SetInterruptCallback(pContext->pDrvContext, NULL, NULL);
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_RegisterUsbContext
 ****************************************************************************//**
 *
 * Register the USB stack context pointer with the High BandWidth manager. A valid
 * stack context is required to make use of USB-HS endpoints and DataWire channels
 * with the High BandWidth channel API.
 *
 * \param context_p
 * Pointer to the DMA manager context structure.
 * \param pUsbStackCtx
 * Pointer to USB stack context structure passed as an opaque pointer.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_RegisterUsbContext (
        cy_stc_hbdma_mgr_context_t *context_p,
        void *pUsbStackCtx)
{
    if (context_p != NULL) {
        context_p->pUsbStackCtx = pUsbStackCtx;
    }
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_UpdateMultEn
 ****************************************************************************//**
 *
 * Update the MULT setting for sockets associated with USB endpoints. When the
 * MULT feature is enabled at the socket level, the USB32DEV endpoint memory
 * is allowed to combine the data from multiple DMA buffers into one transfer
 * burst, thereby getting better data throughput in typical use cases.
 *
 * @warning This function is expected to be called from the USB32 device stack
 * and not directly by the user.
 *
 * \param pContext
 * Pointer to the DMA manager context structure.
 *
 * \param endpNumber
 * USB endpoint index.
 *
 * \param isEgressEp
 * Whether this is an Egress endpoint.
 *
 * \param multEnable
 * Whether to enable the MULT feature.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the MULT setting is updated correctly.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_UpdateMultEn (
        cy_stc_hbdma_mgr_context_t *pContext,
        uint32_t endpNumber,
        bool isEgressEp,
        bool multEnable)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_BAD_PARAM;

    if ((pContext != NULL) && (endpNumber < 16UL))
    {
        /*
         * Update the bit-map that stores the MULT enable setting for the endpoints.
         * It is expected that the socket will only be updated after the API is called and the relevant bits
         * can be set then.
         */
        if (isEgressEp)
        {
            pContext->usbEgrMultEnable &= ~(1 << endpNumber);
            pContext->usbEgrMultEnable |= (multEnable << endpNumber);
        }
        else
        {
            pContext->usbIngMultEnable &= ~(1 << endpNumber);
            pContext->usbIngMultEnable |= (multEnable << endpNumber);
        }

        status = CY_HBDMA_MGR_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_HBDma_SetSocketChannel
****************************************************************************//**
*
* Update DMA channel pHandle associated with a HBW socket.
*
* \param pDmaMgr HBDMA manager context.
* \param socket ID of socket whose mapping is to be updated.
* \param chHandle DMA channel pHandle. May be NULL.
*
* \return None
*******************************************************************************/
static void
Cy_HBDma_SetSocketChannel(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_hbdma_socket_id_t socket,
        cy_stc_hbdma_channel_t *chHandle)
{
    uint32_t adapterIndex, socketIndex;

    CY_ASSERT_L1((pDmaMgr != NULL) && (CY_HBDMA_IS_SOCKET_VALID(socket)));
    adapterIndex = ((uint32_t)socket >> 4U);
    socketIndex  = ((uint32_t)socket & (CY_HBDMA_SOCK_PER_ADAPTER - 1U));
    pDmaMgr->sckChannelMap[adapterIndex - 1U][socketIndex] = chHandle;
}

/*******************************************************************************
* Function Name: Cy_HBDma_GetSocketChannel
****************************************************************************//**
*
* Get handle to DMA channel mapped to the specified socket.
*
* \param pDmaMgr HBDMA manager context.
* \param socket ID of socket whose mapping is to be retrieved.
*
* \return Pointer to the socket control registers if valid,
*         NULL otherwise.
*******************************************************************************/
static cy_stc_hbdma_channel_t *
Cy_HBDma_GetSocketChannel(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_hbdma_socket_id_t socket)
{
    uint32_t adapterIndex, socketIndex;

    CY_ASSERT_L1((pDmaMgr != NULL) && (CY_HBDMA_IS_SOCKET_VALID(socket)));
    adapterIndex = ((uint32_t)socket >> 4U);
    socketIndex  = ((uint32_t)socket & (CY_HBDMA_SOCK_PER_ADAPTER - 1U));
    return (pDmaMgr->sckChannelMap[adapterIndex - 1U][socketIndex]);
}

/*******************************************************************************
* Function Name: Cy_HBDma_FreeDscrChain
****************************************************************************//**
*
* Free the DMA descriptors and buffers starting for the specified descriptor.
*
* \param pDmaMgr Pointer to the DMA manager context structure.
* \param dscrIndex Start of the descriptor chain to be freed.
* \param count Number of descriptors to free.
* \param hdrSize Size of producer data header to be accounted for.
* \param isProdChain Whether to follow next write descriptor chain.
* \param freeBuffer Whether DMA buffers need to be freed.
*
* \return CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid,
*         CY_HBDMA_MGR_SUCCESS otherwise.
*******************************************************************************/
static void
Cy_HBDma_FreeDscrChain (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint16_t                    dscrIndex,
        uint16_t                    count,
        uint16_t                    hdrSize,
        bool                        isProdChain,
        bool                        freeBuffer)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t nextIndex;
    uint8_t  *buf_p;

    while ((count--) != 0)
    {
        Cy_HBDma_GetDescriptor (dscrIndex, &dscr);
        if ((freeBuffer) && (dscr.pBuffer != NULL))
        {
            buf_p = CY_HBDMA_GET_BUFFER_ADDRESS(dscr.pBuffer);
            if ((isProdChain) && (hdrSize != 0))
            {
                buf_p -= hdrSize;
            }
            Cy_HBDma_BufMgr_Free(pDmaMgr->pBufMgr, (void *)buf_p);
        }

        if (isProdChain)
        {
            nextIndex = ((dscr.chain & 0xFFFF0000UL) >> 16U);
        }
        else
        {
            nextIndex = (dscr.chain & 0x0000FFFFUL);
        }

        Cy_HBDma_DscrList_Put(pDmaMgr->pDscrPool, dscrIndex);
        dscrIndex = nextIndex;
    }

    return;
}

/*******************************************************************************
* Function Name: Cy_HBDma_AllocDscrChain
****************************************************************************//**
*
* Allocate High BandWidth DMA descriptor chain.
*
* \param pDmaMgr Pointer to the DMA manager context structure.
* \param dscrIndex_p Pointer to return the first descriptor index through.
* \param count Number of descriptors to allocate.
* \param bufferSize Size of DMA buffer to be allocated.
* \param dscrSync Descriptor sync value to be programmed.
*
* \return CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid,
*         CY_HBDMA_MGR_SUCCESS otherwise.
*******************************************************************************/
static cy_en_hbdma_mgr_status_t
Cy_HBDma_AllocDscrChain (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint16_t *dscrIndex_p,
        uint16_t count,
        uint32_t bufferSize,
        uint32_t dscrSync)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint16_t curIndex, nextIndex, i;
    cy_stc_hbdma_desc_t dscr;

    /* Parameter validity check. */
    if ((pDmaMgr == NULL) || (dscrIndex_p == NULL) || (count == 0) || ((bufferSize & 0x0FU) != 0))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Create the chain */
    i = count;
    status = Cy_HBDma_DscrList_Get(pDmaMgr->pDscrPool, dscrIndex_p);
    if (status == CY_HBDMA_MGR_SUCCESS)
    {
        curIndex = *dscrIndex_p;

        /* Allocate and update the required set of descriptors */
        while ((i--) != 0)
        {
            if (i != 0)
            {
                status = Cy_HBDma_DscrList_Get(pDmaMgr->pDscrPool, &nextIndex);
            }
            else
            {
                /* Create a loop back. */
                nextIndex = *dscrIndex_p;
            }

            if ((bufferSize != 0) && (status == CY_HBDMA_MGR_SUCCESS))
            {
                dscr.pBuffer = Cy_HBDma_BufMgr_Alloc(pDmaMgr->pBufMgr, bufferSize);
                dscr.size    = CY_HBDMA_DSCR_SET_BUFSIZE(pDmaMgr->en_64k, 0, bufferSize);
                if (dscr.pBuffer == NULL)
                {
                    status = CY_HBDMA_MGR_MEMORY_ERROR;
                }
            }
            else
            {
                dscr.pBuffer = NULL;
                dscr.size     = 0;
            }

            dscr.sync   = dscrSync;

            /* Update both the consumer and producer chains with the same descriptor. */
            dscr.chain = nextIndex | (nextIndex << 16U);
            Cy_HBDma_SetDescriptor(curIndex, &dscr);
            curIndex = nextIndex;
        }

        /* If there was a memory allocation error, clean up all the allocated
         * memory and free the descriptor chain. */
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            Cy_HBDma_FreeDscrChain(pDmaMgr, *dscrIndex_p, count, 0, true, true);
        }
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_HBDma_SetSocketDefaultState
****************************************************************************//**
*
* Set all sockets associated with a DMA channel to their default state.
*
* \param pDmaMgr Pointer to the DMA manager context structure.
* \param pHandle Handle to the DMA channel structure.
*
* \return None
*******************************************************************************/
static void
Cy_HBDma_SetSocketDefaultState(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t *pHandle)
{
    cy_stc_hbdma_sockconfig_t sckConf;
    uint16_t i;

    if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP)
    {
        for (i = 0; i < pHandle->prodSckCount; i++)
        {
            if (!CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[i])) {
                /* Disable and Configure the producer socket. */
                Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);

                /* Disable any event triggers coming into the socket. */
                Cy_HbDma_DisconnectEventTriggers(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);

                sckConf.actDscrIndex = pHandle->firstProdDscrIndex[i];
                sckConf.reqXferSize  = 0;
                sckConf.status       = (
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk |
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_PROD_EVENTS_Msk |
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk);
                sckConf.intrMask     = 0x00000000UL;
                Cy_HBDma_SetSocketConfig(pDmaMgr->pDrvContext, pHandle->prodSckId[i], &sckConf);
            }

            /* Update the list that maps socket to channel. */
            Cy_HBDma_SetSocketChannel(pDmaMgr, pHandle->prodSckId[i], pHandle);
        }
    }

    if (pHandle->type != CY_HBDMA_TYPE_IP_TO_MEM)
    {
        for (i = 0; i < pHandle->consSckCount; i++)
        {
            if (!CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[i])) {
                /* Disable and Configure the consumer socket. */
                Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->consSckId[i]);

                /* Disable any event triggers coming into the socket. */
                Cy_HbDma_DisconnectEventTriggers(pDmaMgr->pDrvContext, pHandle->consSckId[i]);

                sckConf.actDscrIndex = pHandle->firstConsDscrIndex[i];
                sckConf.reqXferSize  = 0;
                sckConf.status       = (
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk |
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_CONS_EVENTS_Msk |
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk);
                sckConf.intrMask     = 0x00000000UL;
                Cy_HBDma_SetSocketConfig(pDmaMgr->pDrvContext, pHandle->consSckId[i], &sckConf);
            }

            /* Update the list that maps socket to channel. */
            Cy_HBDma_SetSocketChannel(pDmaMgr, pHandle->consSckId[i], pHandle);
        }
    }
}


/*******************************************************************************
* Function Name: Cy_HBDma_SetSocketEnabled
****************************************************************************//**
*
* Enable the sockets associated with a DMA channel.
*
* \param pDmaMgr Pointer to the DMA manager context structure.
* \param pHandle Handle to the DMA channel structure.
* \param xferSize Amount of data to be transferred through the channel.
*
* \return None
*******************************************************************************/
static void
Cy_HBDma_SetSocketEnabled(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t *pHandle,
        uint32_t xferSize)
{
    cy_stc_hbdma_sockconfig_t sckConf;
    cy_stc_hbdma_sock_t       sckStat;
    cy_stc_hbdma_desc_t       dscrInfo;
    uint16_t i, j;

    /* Configure and enable producer sockets associated with the channel. */
    if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP)
    {
        for (i = 0; i < pHandle->prodSckCount; i++)
        {
            if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[i])) {
                /* Queue a read operation on the endpoint using buffer information from the current descriptor. */
                Cy_HBDma_GetDescriptor(pHandle->curProdDscrIndex[i], &dscrInfo);
                if ((dscrInfo.pBuffer != NULL) && (dscrInfo.size != 0)) {
                    Cy_HBDma_DW_QueueRead(pHandle, i,
                            CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                            CY_HBDMA_DSCR_GET_BUFSIZE(pDmaMgr->en_64k, dscrInfo.size));
                }

                continue;
            }

            /* Disable the socket first. */
            Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);

            /* Configure the socket transfer size and status. */
            Cy_HBDma_GetSocketStatus(pDmaMgr->pDrvContext, pHandle->prodSckId[i], &sckStat);
            sckConf.actDscrIndex = sckStat.actDscrIndex;
            sckConf.reqXferSize  = xferSize;
            sckConf.intrMask     = pHandle->notification & ~USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_CONSUME_EVENT_Msk;
            sckConf.status       = LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_PROD_EVENTS_Msk;

            /* Enable sending of produce events based on channel configuration. */
            if (pHandle->eventEnable) {

                if (pHandle->type == CY_HBDMA_TYPE_IP_TO_IP) {
                    if (pHandle->prodSckCount == 1U)
                    {
                        for (j = 0; j < pHandle->consSckCount; j++)
                        {
                            /* Make trigger connection from the one Producer Socket to all Consumer Sockets. */
                            Cy_HbDma_ConnectEventTrigger(pDmaMgr->pDrvContext,
                                    pHandle->prodSckId[0], pHandle->consSckId[j], 0);
                        }
                    }
                    else
                    {
                        /*
                         * Make trigger connection from all Producer Sockets to the one Consumer Socket.
                         */
                        Cy_HbDma_ConnectEventTrigger(pDmaMgr->pDrvContext,
                                pHandle->prodSckId[i], pHandle->consSckId[0], i);
                    }
                } else {
                    /* Enable event received interrupt for the producer socket. */
                    sckConf.intrMask |= USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_EVENT_RCVD_Msk;
                }
            }

            if (xferSize != 0UL)
            {
                /* Set Truncate ON for finite data transfers. */
                sckConf.status |= LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk;

                /* Enable SUSPEND and TRANS_DONE interrupts when setting a finite transfer size. */
                sckConf.intrMask |= (USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_SUSPEND_Msk |
                        USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk);
            }
            if (pHandle->bufferMode)
            {
                /* Set buffer mode bit where required. */
                sckConf.status |= LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_UNIT_Msk;
            }

            /* Enable MULT feature where applicable. */
            if (
                    (CY_HBDMA_IS_USB_IN_SOCK(pHandle->prodSckId[i])) &&
                    ((pDmaMgr->usbIngMultEnable & (1UL << (pHandle->prodSckId[i] - CY_HBDMA_USBIN_SOCKET_00))) != 0)
               )
            {
                if (pHandle->count > 0x1FU) {
                    sckConf.status |= (LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_ENABLE_Msk |
                            (0x1FU << LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_COUNT_Pos));
                } else {
                    sckConf.status |= (LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_ENABLE_Msk |
                            (pHandle->count << LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_COUNT_Pos));
                }
            }

            Cy_HBDma_SetSocketConfig(pDmaMgr->pDrvContext, pHandle->prodSckId[i], &sckConf);

            /* Finally re-enable the socket. */
            Cy_HBDma_SocketEnable(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);
            DBG_HBDMA_INFO("Enabled prod_sck %x %x\r\n", pHandle->prodSckId[i], sckConf.status);
        }
    }

    /* Configure and enable consumer sockets associated with the channel. */
    if (pHandle->type != CY_HBDMA_TYPE_IP_TO_MEM)
    {
        /* In case of IP to IP channels, transfer size limit shall only be set on the producer sockets. */
        if (pHandle->type == CY_HBDMA_TYPE_IP_TO_IP)
        {
            xferSize = 0;
        }

        for (i = 0; i < pHandle->consSckCount; i++)
        {
            if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[i])) {
                continue;
            }

            /* Disable the socket first. */
            Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->consSckId[i]);

            /* Configure the socket transfer size and status. */
            Cy_HBDma_GetSocketStatus(pDmaMgr->pDrvContext, pHandle->consSckId[i], &sckStat);
            sckConf.actDscrIndex   = sckStat.actDscrIndex;
            sckConf.reqXferSize  = xferSize;
            sckConf.intrMask  = pHandle->notification & ~USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_PRODUCE_EVENT_Msk;
            sckConf.status     = LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_CONS_EVENTS_Msk;

            /* Enable sending of consume events based on channel configuration. */
            if (pHandle->eventEnable) {

                if (pHandle->type == CY_HBDMA_TYPE_IP_TO_IP) {
                    if (pHandle->consSckCount == 1U)
                    {
                        for (j = 0; j < pHandle->prodSckCount; j++)
                        {
                            /* Make trigger connection from the one Consumer Socket to all Producer Sockets. */
                            Cy_HbDma_ConnectEventTrigger(pDmaMgr->pDrvContext,
                                    pHandle->consSckId[0], pHandle->prodSckId[j], 0);
                        }
                    }
                    else
                    {
                        /*
                         * Make trigger connection from all Consumer Sockets to the one Producer Socket.
                         */
                        Cy_HbDma_ConnectEventTrigger(pDmaMgr->pDrvContext,
                                pHandle->consSckId[i], pHandle->prodSckId[0], i);
                    }
                } else {
                    /* Enable event received interrupt for the socket. */
                    sckConf.intrMask |= USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_EVENT_RCVD_Msk;
                }
            }

            if (xferSize != 0UL)
            {
                /* Set Truncate ON for finite data transfers. */
                sckConf.status |= LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk;

                /* Enable SUSPEND and TRANS_DONE interrupts when setting a finite transfer size. */
                sckConf.intrMask |= (USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_SUSPEND_Msk |
                        USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk);
            }
            if (pHandle->bufferMode)
            {
                /* Set buffer mode bit where required. */
                sckConf.status |= LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_UNIT_Msk;
            }

            /* Enable MULT feature where applicable. */
            if (
                    (CY_HBDMA_IS_USB_EG_SOCK(pHandle->consSckId[i])) &&
                    ((pDmaMgr->usbEgrMultEnable & (1UL << (pHandle->consSckId[i] - CY_HBDMA_USBEG_SOCKET_00))) != 0)
               )
            {
                if (pHandle->count > 0x1FU) {
                    sckConf.status |= (LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_ENABLE_Msk |
                            (0x1FU << LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_COUNT_Pos));
                } else {
                    sckConf.status |= (LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_ENABLE_Msk |
                            (pHandle->count << LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_AVL_COUNT_Pos));
                }
            }

            Cy_HBDma_SetSocketConfig(pDmaMgr->pDrvContext, pHandle->consSckId[i], &sckConf);

            /* Finally re-enable the socket. */
            Cy_HBDma_SocketEnable(pDmaMgr->pDrvContext, pHandle->consSckId[i]);
            DBG_HBDMA_INFO("Enabled cons_sck %x %x\r\n", pHandle->consSckId[i], sckConf.status);
        }
    }
}

/*******************************************************************************
* Function Name: Cy_HBDma_UpdateDscrChain
****************************************************************************//**
*
* Update descriptor chain with correct next descriptor index for 1:2 and 2:1
* DMA channels.
*
* \param pHandle Handle to the DMA channel structure.
*
* \return CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid,
*         CY_HBDMA_MGR_SUCCESS otherwise.
*******************************************************************************/
static cy_en_hbdma_mgr_status_t
Cy_HBDma_UpdateDscrChain (
    cy_stc_hbdma_channel_t *pHandle)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    cy_stc_hbdma_desc_t      dscrInfo[2];
    uint16_t                 dscrIndex[2];
    uint8_t                  i, j;

    /* Parameter validity check. */
    if ((pHandle == NULL) ||
            ((pHandle->prodSckCount > 1) && (pHandle->consSckCount > 1)))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->prodSckCount > 1)
    {
        i = 0;
        j = 0;

        dscrIndex[0] = pHandle->firstProdDscrIndex[0];
        Cy_HBDma_GetDescriptor(dscrIndex[0], &dscrInfo[0]);
        dscrIndex[1] = ((dscrInfo[0].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) >>
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos);
        Cy_HBDma_GetDescriptor(dscrIndex[1], &dscrInfo[1]);

        pHandle->firstProdDscrIndex[1] = dscrIndex[1];
        pHandle->curProdDscrIndex[1]   = dscrIndex[1];
        pHandle->firstConsDscrIndex[1] = pHandle->firstProdDscrIndex[0];
        pHandle->curConsDscrIndex[1]   = pHandle->firstProdDscrIndex[0];

        do {
            if (j != 0)
            {
                /* Update sync field of descriptor to point to the correct producer socket. */
                dscrInfo[j].sync = (
                        (dscrInfo[j].sync & ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_PROD_SCK_Msk |
                                              LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_PROD_IP_Msk)) |
                        CY_HBDMA_PROD_SOCK_TO_SYNC(pHandle->prodSckId[1]));
            }

            /* Update next write descriptor to point to the next descriptor for this socket. */
            dscrInfo[j].chain = (
                    (dscrInfo[j].chain & ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)) |
                    ((dscrInfo[j ^ 1].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) <<
                     LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos));

            /* If we have a header to be added in firmware, adjust the buffer size and start address. */
            if ((pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) && (pHandle->prodHdrSize != 0))
            {
                dscrInfo[j].pBuffer += pHandle->prodHdrSize;
                dscrInfo[j].size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pHandle->pContext->en_64k, 0, pHandle->prodBufSize);
            }

            /* Write the modified descriptor back. */
            Cy_HBDma_SetDescriptor(dscrIndex[j], &dscrInfo[j]);

            dscrIndex[j] = ((dscrInfo[j ^ 1].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) >>
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos);
            Cy_HBDma_GetDescriptor(dscrIndex[j], &dscrInfo[j]);

            j = j ^ 1;
            i++;
        } while (i < (pHandle->count * 2));
    }
    else
    {
        if ((pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) && (pHandle->prodHdrSize != 0))
        {
            dscrIndex[0] = pHandle->firstProdDscrIndex[0];

            for (i = 0; i < pHandle->count; i++)
            {
                /* Run through the DMA descriptors and update them to skip the header portion of the buffer. */
                Cy_HBDma_GetDescriptor(dscrIndex[0], &dscrInfo[0]);
                dscrInfo[0].pBuffer += pHandle->prodHdrSize;
                dscrInfo[0].size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pHandle->pContext->en_64k, 0, pHandle->prodBufSize);
                Cy_HBDma_SetDescriptor(dscrIndex[0], &dscrInfo[0]);

                dscrIndex[0] = ((dscrInfo[0].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                        >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
            }
        }
    }

    if (pHandle->consSckCount > 1)
    {
        i = 0;
        j = 0;

        dscrIndex[0] = pHandle->firstProdDscrIndex[0];
        Cy_HBDma_GetDescriptor(dscrIndex[0], &dscrInfo[0]);
        dscrIndex[1] = ((dscrInfo[0].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) >>
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
        Cy_HBDma_GetDescriptor(dscrIndex[1], &dscrInfo[1]);

        pHandle->firstProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];
        pHandle->curProdDscrIndex[1]   = pHandle->firstProdDscrIndex[0];
        pHandle->firstConsDscrIndex[1] = dscrIndex[1];
        pHandle->curConsDscrIndex[1]   = dscrIndex[1];

        do {
            if (j != 0)
            {
                /* Update sync field of descriptor to point to the correct consumer socket. */
                dscrInfo[j].sync = (
                        (dscrInfo[j].sync & ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_CONS_SCK_Msk |
                                              LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_CONS_IP_Msk)) |
                        CY_HBDMA_CONS_SOCK_TO_SYNC(pHandle->consSckId[1]));
            }

            /* Update next read descriptor to point to the next descriptor for this socket. */
            dscrInfo[j].chain = (
                    (dscrInfo[j].chain & ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk)) |
                    ((dscrInfo[j ^ 1].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) >>
                     LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos));

            /* Write the modified descriptor back. */
            Cy_HBDma_SetDescriptor(dscrIndex[j], &dscrInfo[j]);

            dscrIndex[j] = ((dscrInfo[j ^ 1].chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) >>
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
            Cy_HBDma_GetDescriptor(dscrIndex[j], &dscrInfo[j]);

            j = j ^ 1;
            i++;
        } while (i < (pHandle->count * 2));
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_HBDma_MultiChn_DscrSetup
****************************************************************************//**
*
* Allocate and configure descriptor chains for Manual 1:2 and 2:1 DMA channels.
*
* \param pDmaMgr Handle to the DMA manager context structure.
* \param pHandle Handle to the DMA channel structure.
* \param dscrSync Sync field setting to be used in the descriptors.
*
* \return CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid,
*         CY_HBDMA_MGR_SUCCESS otherwise.
*******************************************************************************/
static cy_en_hbdma_mgr_status_t
Cy_HBDma_MultiChn_DscrSetup (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t     *pHandle,
        uint32_t                    dscrSync)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    cy_stc_hbdma_desc_t srcDesc, dstDesc;
    uint16_t srcIdx, dstIdx0, dstIdx1;
    uint32_t i;

    /* For a manual IP -> IP channel, we need to create a separate set of descriptors. */
    if (pHandle->prodSckCount > 1)
    {
        /* Save the first descriptor index as the one to be used with the consumer. */
        pHandle->firstConsDscrIndex[0] = pHandle->firstProdDscrIndex[0];
        pHandle->curConsDscrIndex[0]   = pHandle->firstProdDscrIndex[0];
        pHandle->firstProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];
        pHandle->curConsDscrIndex[1]   = pHandle->firstProdDscrIndex[0];

        /*
         * Allocate separate descriptor chains with no associated buffers for each
         * producer socket, then update the descriptors with the buffers allocated
         * earlier.
         */
        status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->curProdDscrIndex[0]),
                pHandle->count, 0, dscrSync);
        if (status == CY_HBDMA_MGR_SUCCESS)
        {
            dscrSync &= ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_PROD_SCK_Msk |
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_PROD_IP_Msk);
            dscrSync |= CY_HBDMA_PROD_SOCK_TO_SYNC(pHandle->prodSckId[1]);

            status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->curProdDscrIndex[1]),
                    pHandle->count, 0, dscrSync);

            if (status == CY_HBDMA_MGR_SUCCESS)
            {
                /*
                 * The original descriptor chain should be used as the consumer chain.
                 *
                 * We can take the buffer pointers from that chain and update in the newly created per-producer
                 * chains. The RD_NEXT pointer in each descriptor is set to point to the next produce descriptor
                 * on the alternate producer socket.
                 */
                srcIdx  = pHandle->curConsDscrIndex[0];
                dstIdx0 = pHandle->curProdDscrIndex[0];
                dstIdx1 = pHandle->curProdDscrIndex[1];

                for (i = 0; i < pHandle->count; i++)
                {
                    Cy_HBDma_GetDescriptor(srcIdx, &srcDesc);
                    Cy_HBDma_GetDescriptor(dstIdx0, &dstDesc);
                    dstDesc.chain   = ((dstDesc.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) |
                            (dstIdx1 << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos));
                    dstDesc.pBuffer = srcDesc.pBuffer;
                    dstDesc.size    = CY_HBDMA_DSCR_SET_BUFSIZE(pDmaMgr->en_64k, 0, pHandle->size);
                    if (pHandle->prodHdrSize != 0)
                    {
                        /*
                         * In case header addition is expected, update the base pointer and buffer size in
                         * the producer side descriptors.
                         */
                        dstDesc.pBuffer += pHandle->prodHdrSize;
                        dstDesc.size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pDmaMgr->en_64k, 0, pHandle->prodBufSize);
                    }
                    Cy_HBDma_SetDescriptor(dstIdx0, &dstDesc);

                    dstIdx0 = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dstDesc.chain);
                    srcIdx  = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(srcDesc.chain);

                    Cy_HBDma_GetDescriptor(srcIdx, &srcDesc);
                    srcDesc.sync = dscrSync;
                    Cy_HBDma_SetDescriptor(srcIdx, &srcDesc);
                    Cy_HBDma_GetDescriptor(dstIdx1, &dstDesc);
                    dstDesc.chain   = ((dstDesc.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) |
                            (dstIdx0 << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos));
                    dstDesc.pBuffer = srcDesc.pBuffer;
                    dstDesc.size    = CY_HBDMA_DSCR_SET_BUFSIZE(pDmaMgr->en_64k, 0, pHandle->size);
                    if (pHandle->prodHdrSize != 0)
                    {
                        /*
                         * In case header addition is expected, update the base pointer and buffer size in
                         * the producer side descriptors.
                         */
                        dstDesc.pBuffer += pHandle->prodHdrSize;
                        dstDesc.size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pDmaMgr->en_64k, 0, pHandle->prodBufSize);
                    }
                    Cy_HBDma_SetDescriptor(dstIdx1, &dstDesc);

                    dstIdx1 = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dstDesc.chain);
                    srcIdx  = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(srcDesc.chain);
                }

                /* Update descriptor indices of interest for the channel. */
                pHandle->firstProdDscrIndex[0] = pHandle->curProdDscrIndex[0];
                pHandle->firstProdDscrIndex[1] = pHandle->curProdDscrIndex[1];
                pHandle->nextProdDscr          = pHandle->curProdDscrIndex[0];
                pHandle->lastProdDscr          = pHandle->curProdDscrIndex[0];
                pHandle->nextConsDscr          = pHandle->curConsDscrIndex[0];
            }
            else
            {
                /* Free the descriptor chain just created and return error. */
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->curProdDscrIndex[0], pHandle->count, 0,
                        true, false);
            }
        }
    }
    else
    {
        pHandle->curProdDscrIndex[0] = pHandle->firstProdDscrIndex[0];
        pHandle->curProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];

        status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->curConsDscrIndex[0]),
                pHandle->count, 0, dscrSync);
        if (status == CY_HBDMA_MGR_SUCCESS)
        {
            dscrSync &= ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_CONS_SCK_Msk |
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_CONS_IP_Msk);
            dscrSync |= CY_HBDMA_CONS_SOCK_TO_SYNC(pHandle->consSckId[1]);

            status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->curConsDscrIndex[1]),
                    pHandle->count, 0, dscrSync);

            if (status == CY_HBDMA_MGR_SUCCESS)
            {
                /*
                 * The original descriptor chain should be used as the consumer chain.
                 *
                 * We can take the buffer pointers from that chain and update in the newly created per-producer
                 * chains. The RD_NEXT pointers in the descriptors are updated to point to the next descriptor
                 * associated with the alternate socket.
                 */
                srcIdx  = pHandle->firstProdDscrIndex[0];
                dstIdx0 = pHandle->curConsDscrIndex[0];
                dstIdx1 = pHandle->curConsDscrIndex[1];

                for (i = 0; i < pHandle->count; i++)
                {
                    Cy_HBDma_GetDescriptor(srcIdx, &srcDesc);
                    Cy_HBDma_GetDescriptor(dstIdx0, &dstDesc);
                    dstDesc.pBuffer = srcDesc.pBuffer;
                    dstDesc.chain   = ((dstDesc.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) |
                            (dstIdx1 << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos));
                    dstDesc.size    = CY_HBDMA_DSCR_SET_BUFSIZE(pDmaMgr->en_64k, 0, pHandle->size);
                    Cy_HBDma_SetDescriptor(dstIdx0, &dstDesc);
                    if (pHandle->prodHdrSize != 0)
                    {
                        /*
                         * In case header addition is expected, update the base pointer and buffer size in
                         * the producer side descriptors.
                         */
                        srcDesc.pBuffer += pHandle->prodHdrSize;
                        srcDesc.size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pDmaMgr->en_64k, 0, pHandle->prodBufSize);
                        Cy_HBDma_SetDescriptor(srcIdx, &srcDesc);
                    }

                    dstIdx0 = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dstDesc.chain);
                    srcIdx  = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(srcDesc.chain);

                    Cy_HBDma_GetDescriptor(srcIdx, &srcDesc);
                    Cy_HBDma_GetDescriptor(dstIdx1, &dstDesc);
                    dstDesc.pBuffer = srcDesc.pBuffer;
                    dstDesc.chain   = ((dstDesc.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) |
                            (dstIdx0 << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos));
                    dstDesc.size    = CY_HBDMA_DSCR_SET_BUFSIZE(pDmaMgr->en_64k, 0, pHandle->size);
                    Cy_HBDma_SetDescriptor(dstIdx1, &dstDesc);
                    if (pHandle->prodHdrSize != 0)
                    {
                        /*
                         * In case header addition is expected, update the base pointer and buffer size in
                         * the producer side descriptors.
                         */
                        srcDesc.pBuffer += pHandle->prodHdrSize;
                        srcDesc.size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pDmaMgr->en_64k, 0, pHandle->prodBufSize);
                    }
                    srcDesc.sync = dscrSync;
                    Cy_HBDma_SetDescriptor(srcIdx, &srcDesc);

                    dstIdx1 = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dstDesc.chain);
                    srcIdx  = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(srcDesc.chain);
                }

                /* Update descriptor indices of interest for the channel. */
                pHandle->firstConsDscrIndex[0] = pHandle->curConsDscrIndex[0];
                pHandle->firstConsDscrIndex[1] = pHandle->curConsDscrIndex[1];
                pHandle->nextConsDscr          = pHandle->firstConsDscrIndex[0];
                pHandle->nextProdDscr          = pHandle->firstProdDscrIndex[0];
                pHandle->lastProdDscr          = pHandle->firstProdDscrIndex[0];
            }
            else
            {
                /* Free the descriptor chain just created and return error. */
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->curConsDscrIndex[0], pHandle->count, 0,
                        false, false);
            }
        }
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_HBDma_DelayedProdEvt_Handler
****************************************************************************//**
*
* Function used to handle delayed produce events in the case of 2:1 manual
* channels. Called from the main HBDma interrupt event handler.
*
* \param pDmaMgr DMA manager context structure pointer.
* \param channelHandle Handle to the DMA channel structure.
* \param prodSckIndex The producer socket index to be checked and handled.
*
* \return true if valid produce event was found.
*******************************************************************************/
static bool Cy_HBDma_DelayedProdEvt_Handler (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t     *channelHandle,
        uint8_t                     prodSckIndex)
{
    cy_stc_hbdma_buff_status_t  bufStat = {NULL, 0, 0, 0};
    cy_stc_hbdma_desc_t         dscrInfo;
    uint16_t                    dscrIndex;
    bool                        retVal = false;

    /* Handle delayed produce event on the second producer socket of a 2:1 channel. */
    dscrIndex = channelHandle->curProdDscrIndex[prodSckIndex];
    Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

    if (
            ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0) &&
            (!CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
       ) {
        bufStat.pBuffer  = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
        bufStat.size     = CY_HBDMA_DSCR_GET_BUFSIZE(pDmaMgr->en_64k, dscrInfo.size);
        bufStat.count    = CY_HBDMA_DSCR_GET_BYTECNT(pDmaMgr->en_64k, dscrInfo.size);
        bufStat.status   = (dscrInfo.size & 0x0EUL);

        /* Move the current produce descriptor for the socket. */
        channelHandle->curProdDscrIndex[prodSckIndex] = (
                (dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                >> (LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos)
                );

        /* Raise produce event callback. */
        if (channelHandle->cbFunc != NULL) {
            channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_PROD_EVENT, &bufStat, channelHandle->cbCtx);
        }

        /* The next produce event is expected on the other socket. */
        channelHandle->nextProdSck = !prodSckIndex;
        retVal = true;
    }

    return retVal;
}

/*******************************************************************************
* Function Name: Cy_HBDma_IngressDW_IntrHandler
****************************************************************************//**
*
* Function used to handle DataWire interrupts corresponding to USBHS OUT
* endpoints.
*
* \param pDmaMgr DMA manager context structure pointer.
* \param channelHandle Handle to the DMA channel structure.
* \param prodSckIndex The producer socket index to be checked and handled.
*
* \return true if valid produce event was found.
*******************************************************************************/
static bool Cy_HBDma_IngressDW_IntrHandler (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t     *channelHandle,
        uint8_t                     prodSckIndex)
{
    cy_stc_hbdma_buff_status_t  bufStat = {NULL, 0, 0, 0};
    cy_stc_hbdma_desc_t         dscrInfo;
    uint16_t                    dscrIndex;
    bool                        retVal = false;

    /* Handle delayed produce event on the second producer socket of a 2:1 channel. */
    dscrIndex = channelHandle->curProdDscrIndex[prodSckIndex];
    Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

    /* At this stage, the descriptor should be unoccupied and unmarked. */
    if (
            ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) &&
            (!CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
       ) {
        /* Update the byte count and set the occupied bit in the DSCR_SIZE field. */
        dscrInfo.size = (
                (dscrInfo.size & 0x0000FFFFUL) |
                (channelHandle->curIngressXferSize[prodSckIndex] << 16UL) |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk);
        Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);

        bufStat.pBuffer  = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
        bufStat.size     = CY_HBDMA_DSCR_GET_BUFSIZE(pDmaMgr->en_64k, dscrInfo.size);
        bufStat.count    = CY_HBDMA_DSCR_GET_BYTECNT(pDmaMgr->en_64k, dscrInfo.size);
        bufStat.status   = (dscrInfo.size & 0x0EUL);

        /* Move the current produce descriptor for the socket. */
        channelHandle->curProdDscrIndex[prodSckIndex] = (
                (dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                >> (LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos)
                );

        /* If the next descriptor is not occupied or marked, we can queue another read operation. */
        dscrIndex = channelHandle->curProdDscrIndex[prodSckIndex];
        Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
        if (
                ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) &&
                (!CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
           ) {
            Cy_HBDma_DW_QueueRead(channelHandle, prodSckIndex,
                    CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                    CY_HBDMA_DSCR_GET_BUFSIZE(pDmaMgr->en_64k, dscrInfo.size));
        }

        /* Raise produce event callback. */
        if (channelHandle->cbFunc != NULL) {
            channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_PROD_EVENT, &bufStat, channelHandle->cbCtx);
        }

        if (channelHandle->prodSckCount > 1) {
            /* The next produce event is expected on the other socket. */
            channelHandle->nextProdSck = !prodSckIndex;
        }

        retVal = true;
    }

    return retVal;
}

/*******************************************************************************
* Function Name: Cy_HBDma_IngressDW_HandleConsEvent
****************************************************************************//**
*
* Function used to update Ingress DataWire status when a consume event
* is received on the egress socket associated with it.
*
* \param pDmaMgr DMA manager context structure pointer.
* \param channelHandle Handle to the DMA channel structure.
* \param consSckIndex The consumer socket index to be checked and handled.
*
* \return true if valid produce event was found.
*******************************************************************************/
static bool Cy_HBDma_IngressDW_HandleConsEvent (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t     *channelHandle,
        cy_hbdma_socket_id_t        sockIdx)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint16_t dscrIndex;
    uint8_t actSckIndex = 0;
    bool retVal = false;

    /* If the producer socket is not USBHS OUT EP, return error. */
    if ((channelHandle == NULL) || (channelHandle->type != CY_HBDMA_TYPE_IP_TO_IP) ||
            (!CY_HBDMA_IS_USBHS_OUT_EP(sockIdx))) {
        return retVal;
    }

    /* Find the producer socket index. */
    if ((channelHandle->prodSckCount > 1) && (channelHandle->prodSckId[1] == sockIdx)) {
        actSckIndex = 1;
    } else {
        if (channelHandle->prodSckId[0] == sockIdx) {
            actSckIndex = 0;
        } else {
            return retVal;
        }
    }

    /* If read is not queued on the DataWire channel. */
    if (channelHandle->ingressDWRqtQueued[actSckIndex] == false) {
        /* Get the next producer descriptor and verify that it is empty and unmarked. */
        dscrIndex = channelHandle->curProdDscrIndex[actSckIndex];
        Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
        if (
                ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) &&
                (!CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
           ) {
            /* Queue read operation. */
            Cy_HBDma_DW_QueueRead(channelHandle, actSckIndex,
                    CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                    CY_HBDMA_DSCR_GET_BUFSIZE(channelHandle->pContext->en_64k, dscrInfo.size));
            retVal = true;
        }
    }

    return retVal;
}

/*******************************************************************************
* Function Name: Cy_HBDma_EgressDW_ConsEvtHandler
****************************************************************************//**
*
* Generic helper function to take care of descriptor and socket updates when
* there is a transfer completion on Egress DataWire path associated with a DMA
* channel.
*
* \param pDmaMgr DMA manager context structure pointer.
* \param channelHandle Handle to the DMA channel structure.
* \param consSckIndex The consumer socket index to be checked and handled.
*
*******************************************************************************/
static void Cy_HBDma_EgressDW_ConsEvtHandler (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t     *channelHandle,
        uint8_t                     consSckIndex)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint16_t dscrIndex;

    /* Read the current consumer descriptor. */
    dscrIndex = channelHandle->curConsDscrIndex[consSckIndex];
    Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

    /* Mark the current consumer descriptor unoccupied and unmarked. */
    dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
    dscrInfo.size    = dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
    Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);

    /* Move to the next consumer descriptor. */
    channelHandle->curConsDscrIndex[consSckIndex] = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);

    /* Read the lastProdDscr. */
    dscrIndex = channelHandle->lastProdDscr;
    Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

    /* Mark it empty and unoccupied as well. */
    dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
    dscrInfo.size    = dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
    Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);

    if (channelHandle->type == CY_HBDMA_TYPE_IP_TO_IP) {
        /* Notify the producer that buffer is empty. */
        if (CY_HBDMA_IS_USBHS_OUT_EP(CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync))) {
            Cy_HBDma_IngressDW_HandleConsEvent(pDmaMgr, channelHandle, CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync));
        } else {
            Cy_HBDma_SendSocketEvent(pDmaMgr->pDrvContext, CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync), false);
        }
    }

    /* Move lastProdDscr to the next one. */
    if (channelHandle->prodSckCount > 1) {
        channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
    } else {
        channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
    }
}

/*******************************************************************************
* Function Name: Cy_HBDma_Channel_IntrHandler
****************************************************************************//**
*
* High BandWidth DMA channel callback function implementation.
*
* \param pDmaMgr DMA manager context structure pointer.
* \param intrInfo DMA interrupt information.
*
* \return void
*******************************************************************************/
static void Cy_HBDma_Channel_IntrHandler (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_intr_msg_t    *intrInfo_p)
{
    cy_stc_hbdma_channel_t     *channelHandle;
    cy_stc_hbdma_sockconfig_t   sckConf;
    cy_stc_hbdma_sock_t         sckStat;
    cy_hbdma_socket_id_t        socketId = (cy_hbdma_socket_id_t)intrInfo_p->socketId;
    cy_hbdma_socket_id_t        expSockId;
    cy_stc_hbdma_buff_status_t  bufStat = {NULL, 0, 0, 0};
    cy_stc_hbdma_desc_t         dscrInfo;
    uint16_t                    dscrIndex;
    uint8_t                     actSckIndex = 0;

    if ((pDmaMgr == NULL) || (intrInfo_p == NULL))
    {
        DBG_HBDMA_ERR("NULL parameters\r\n");
        return;
    }

    channelHandle = Cy_HBDma_GetSocketChannel(pDmaMgr, socketId);
    if (channelHandle == NULL)
    {
        DBG_HBDMA_ERR("No channel mapped to socket %x\r\n", socketId);
        return;
    }

    switch(intrInfo_p->intrType)
    {
        case CY_HBDMA_SOCK_PRODUCE_EVT:
            if ((channelHandle->prodSckCount > 1) && (channelHandle->prodSckId[0] != socketId))
            {
                actSckIndex = 1;
            }
            else
            {
                actSckIndex = 0;
            }

            /*
             * Multiple produce events might have happened back-to-back. We have to keep raising
             * events until the next producer descriptor matches the current descriptor that the socket
             * is using.
             */
            do {
                dscrIndex = channelHandle->curProdDscrIndex[actSckIndex];
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

                if (
                        ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) ||
                        (CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
                   )
                {
                    /*
                     * It is possible that we see a second produce event interrupt after the corresponding
                     * descriptor was already processed during the previous one. This is not an error
                     * condition.
                     */
                    break;
                }
                else
                {
                    /* In case of a 2:1 channel, handle produce events only in the order expected. */
                    if (channelHandle->prodSckCount > 1) {
                        if (actSckIndex != channelHandle->nextProdSck) {
                            /* Keep this event pending until we service the valid produce event on the other socket. */
                            channelHandle->pendingEvtCnt++;
                            break;
                        }
                    }

                    bufStat.pBuffer  = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                    bufStat.size     = CY_HBDMA_DSCR_GET_BUFSIZE(pDmaMgr->en_64k, dscrInfo.size);
                    bufStat.count    = CY_HBDMA_DSCR_GET_BYTECNT(pDmaMgr->en_64k, dscrInfo.size);
                    bufStat.status   = (dscrInfo.size & 0x0EUL);

                    /* Move the current produce descriptor for the socket. */
                    channelHandle->curProdDscrIndex[actSckIndex] = (
                            (dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                            >> (LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos)
                            );

                    /* Raise produce event callback. */
                    if (channelHandle->cbFunc != NULL) {
                        channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_PROD_EVENT, &bufStat, channelHandle->cbCtx);
                    }

                    /* Next produce event is expected on the other producer socket. */
                    if (channelHandle->prodSckCount > 1) {
                        channelHandle->nextProdSck = !actSckIndex;

                        /* If there were delayed produce events, handle them. Only one can be handled after
                         * each event on the active socket. */
                        if (channelHandle->pendingEvtCnt > 0) {
                            if (!Cy_HBDma_DelayedProdEvt_Handler(pDmaMgr, channelHandle, !actSckIndex)) {
                                channelHandle->pendingEvtCnt = 0;
                            }
                        }
                    }
                }
            } while (channelHandle->curProdDscrIndex[actSckIndex] != intrInfo_p->curDscr);
            break;

        case CY_HBDMA_SOCK_CONSUME_EVT:
            if ((channelHandle->consSckCount > 1) && (channelHandle->consSckId[0] != socketId))
            {
                actSckIndex = 1;
            }
            else
            {
                actSckIndex = 0;
            }

            /*
             * Multiple consume events might have happened back-to-back. We have to keep raising events
             * until the next consumer descriptor for this socket matches the current descriptor that
             * the socket is using.
             */
            do {
                dscrIndex = channelHandle->curConsDscrIndex[actSckIndex];
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

                if ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0)
                {
                    /*
                     * At present, we are getting consume event interrupt when sending a produce event
                     * to the socket. Just ignore these interrupts when the current descriptor is still
                     * occupied.
                     */
                    break;
                }
                else
                {
                    /*
                     * Keeping track of buffers which are yet to be consumed.
                     */
                    if (channelHandle->commitCnt[actSckIndex] != 0)
                    {
                        channelHandle->commitCnt[actSckIndex]--;
                    } else {
                        /*
                         * In cases where consume/produce events are being handled by FW, only process as many
                         * consume events as are expected.
                         */
                        if (channelHandle->eventEnable == false)
                        {
                            break;
                        }
                    }

                    /* Move the current consume descriptor for the socket. */
                    channelHandle->curConsDscrIndex[actSckIndex] = (
                            (dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk)
                            >> (LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos)
                            );

                    if ((channelHandle->type != CY_HBDMA_TYPE_IP_TO_IP) || (channelHandle->eventEnable == false))
                    {
                        Cy_HBDma_GetDescriptor(channelHandle->lastProdDscr, &dscrInfo);
                        expSockId = CY_HBDMA_SYNC_TO_CONS_SOCK(dscrInfo.sync);

                        /* In case of a 1:2 channel, it is possible that the consume event is received
                         * out of order. In such case, we just store the consume event to be handled later. */
                        if ((channelHandle->consSckCount == 1) || (socketId == expSockId))
                        {
                            /* Mark the last produce descriptor empty and send consume event to the socket. */
                            dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                            dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                            Cy_HBDma_SetDescriptor(channelHandle->lastProdDscr, &dscrInfo);

                            if (channelHandle->type == CY_HBDMA_TYPE_IP_TO_IP)
                            {
                                if (CY_HBDMA_IS_USBHS_OUT_EP(CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync))) {
                                    Cy_HBDma_IngressDW_HandleConsEvent(pDmaMgr, channelHandle,
                                            CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync));
                                } else {
                                    /* Send a consume event to the socket corresponding to this descriptor. */
                                    Cy_HBDma_SendSocketEvent(pDmaMgr->pDrvContext,
                                            CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync), false);
                                }
                            }

                            /* Move the lastProdDscr to the next descriptor. */
                            if (channelHandle->prodSckCount > 1)
                            {
                                channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                            }
                            else
                            {
                                channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                            }

                            /* Raise consume event callback. */
                            if (channelHandle->cbFunc != NULL) {
                                channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_CONS_EVENT, NULL,
                                        channelHandle->cbCtx);
                            }

                            /* Process any pending consume events prematurely received from the alternate socket. */
                            if ((channelHandle->consSckCount > 1) && (channelHandle->pendingEvtCnt != 0))
                            {
                                channelHandle->pendingEvtCnt--;

                                /* Mark the last produce descriptor empty and send consume event to the socket. */
                                Cy_HBDma_GetDescriptor(channelHandle->lastProdDscr, &dscrInfo);
                                dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                                dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                                Cy_HBDma_SetDescriptor(channelHandle->lastProdDscr, &dscrInfo);

                                if (channelHandle->type == CY_HBDMA_TYPE_IP_TO_IP) {
                                    if (CY_HBDMA_IS_USBHS_OUT_EP(CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync))) {
                                        Cy_HBDma_IngressDW_HandleConsEvent(pDmaMgr, channelHandle,
                                                CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync));
                                    } else {
                                        /* Send a consume event to the socket corresponding to this descriptor. */
                                        Cy_HBDma_SendSocketEvent(pDmaMgr->pDrvContext,
                                                CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync), false);
                                    }
                                }

                                /* Move the lastProdDscr to the next descriptor. */
                                if (channelHandle->prodSckCount > 1)
                                {
                                    channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                                }
                                else
                                {
                                    channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                                }

                                /* Raise consume event callback. */
                                if (channelHandle->cbFunc != NULL) {
                                    channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_CONS_EVENT, NULL,
                                            channelHandle->cbCtx);
                                }
                            }
                        }
                        else
                        {
                            channelHandle->pendingEvtCnt++;
                        }
                    }
                }
            } while (channelHandle->curConsDscrIndex[actSckIndex] != intrInfo_p->curDscr);
            break;

        case CY_HBDMA_SOCK_SUSPEND_EVT:
            /* Send notification if enabled for the channel. */
            if (
                    (channelHandle->cbFunc != NULL) &&
                    ((channelHandle->notification & USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_SUSPEND_Msk) != 0)
               ) {
                channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_SUSPENDED, NULL, channelHandle->cbCtx);
            }
            break;

        case CY_HBDMA_SOCK_ERROR_EVT:
            if (channelHandle->cbFunc != NULL) {
                channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_ERROR, NULL, channelHandle->cbCtx);
            }
            break;

        case CY_HBDMA_SOCK_XFERDONE_EVT:
            if (channelHandle->state == CY_HBDMA_CHN_ACTIVE) {
                /* Disable the socket once the requested amount of data has been transferred. */
                Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, socketId);
            }

            /* Update the status of the channel to idle so that a repeated operation can be performed. */
            channelHandle->state = CY_HBDMA_CHN_CONFIGURED;

            /* Send notification if enabled for the channel. */
            if (
                    (channelHandle->cbFunc != NULL) &&
                    ((channelHandle->notification & USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk) != 0)
               ) {
                channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_XFER_CPLT, NULL, channelHandle->cbCtx);
            }
            break;

        case CY_HBDMA_SOCK_EVT_RCVD:
            /* For an IP to Mem channel, this interrupt is equivalent to a DiscardBuffer API call. */
            if (channelHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) {
                Cy_HBDma_GetDescriptor(channelHandle->nextProdDscr, &dscrInfo);
                if (channelHandle->prodSckCount > 1) {
                    channelHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                    channelHandle->activeSckIndex ^= 1;
                } else {
                    channelHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                }
            }

            /* For a Mem to IP channel, this interrupt is equivalent to a CommitBuffer API call. */
            if (channelHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) {
                Cy_HBDma_GetDescriptor(channelHandle->nextConsDscr, &dscrInfo);
                if (channelHandle->consSckCount > 1) {
                    channelHandle->nextConsDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                    channelHandle->activeSckIndex ^= 1;
                } else {
                    channelHandle->nextConsDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                }
            }
            break;

        case CY_HBDMA_SOCK_STALL_EVT:
            if (channelHandle->eventEnable)
            {
                if (channelHandle->state == CY_HBDMA_CHN_DATADROP)
                {
                    /* Mark all the descriptors empty and send an event to the socket. */
                    Cy_HBDma_GetSocketStatus(pDmaMgr->pDrvContext, socketId, &sckStat);
                    dscrIndex = (uint16_t)(sckStat.actDscrIndex & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_DSCR_NUMBER_Msk);
                    Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
                    while ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0)
                    {
                        dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                        Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);
                        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                    }

                    /* Send a consume event to the producer socket. */
                    Cy_HBDma_SendSocketEvent(pDmaMgr->pDrvContext, socketId, false);
                }
            }
            else
            {
                /* STALL interrupt is only enabled on consumer sockets when there is a pending discard operation. */
                if ((channelHandle->consSckCount > 1) && (channelHandle->consSckId[0] != socketId))
                {
                    actSckIndex = 1;
                }
                else
                {
                    actSckIndex = 0;
                }

                dscrIndex = channelHandle->curConsDscrIndex[actSckIndex];
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);

                Cy_HBDma_GetSocketStatus(pDmaMgr->pDrvContext, socketId, &sckStat);

                if (
                        (CY_HBDMA_STATUS_TO_SOCK_STATE(sckStat.status) == CY_HBDMA_SOCK_STATE_STALL) &&
                        (CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer)) &&
                        (channelHandle->discardCnt[actSckIndex] != 0)
                   )
                {
                    Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, channelHandle->consSckId[actSckIndex]);

                    /* Clear the marker on the descriptor and ensure it is empty. */
                    dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                    dscrInfo.size    = dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                    Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);

                    /* Move to the next descriptor. */
                    channelHandle->curConsDscrIndex[actSckIndex] = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);

                    /* Jump the current descriptor on the socket to the next one. */
                    Cy_HBDma_GetSocketStatus(pDmaMgr->pDrvContext, channelHandle->consSckId[actSckIndex], &sckStat);
                    sckConf.actDscrIndex = channelHandle->curConsDscrIndex[actSckIndex];
                    sckConf.reqXferSize  = sckStat.reqXferSize - sckStat.compXferCount;
                    sckConf.intrMask     = sckStat.intrMask;
                    sckConf.status       = (sckStat.status & ~(
                                USB32DEV_ADAPTER_DMA_SCK_STATUS_WRAPUP_Msk |
                                USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_SUSPEND_Msk |
                                USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk));

                    channelHandle->discardCnt[actSckIndex]--;
                    if (channelHandle->discardCnt[actSckIndex] == 0)
                    {
                        sckConf.intrMask &= ~USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_STALL_Msk;
                    }

                    Cy_HBDma_SetSocketConfig(pDmaMgr->pDrvContext, channelHandle->consSckId[actSckIndex], &sckConf);
                    Cy_HBDma_SocketEnable(pDmaMgr->pDrvContext, channelHandle->consSckId[actSckIndex]);

                    /* Mark the last produce descriptor empty and send consume event to the socket. */
                    Cy_HBDma_GetDescriptor(channelHandle->lastProdDscr, &dscrInfo);
                    dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                    dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                    Cy_HBDma_SetDescriptor(channelHandle->lastProdDscr, &dscrInfo);

                    if (CY_HBDMA_IS_USBHS_OUT_EP(CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync))) {
                        Cy_HBDma_IngressDW_HandleConsEvent(pDmaMgr, channelHandle,
                                CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync));
                    } else {
                        /* Send a consume event to the socket corresponding to this descriptor. */
                        Cy_HBDma_SendSocketEvent(pDmaMgr->pDrvContext,
                                CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync), false);
                    }

                    /* Move the lastProdDscr to the next descriptor. */
                    if (channelHandle->prodSckCount > 1)
                    {
                        channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                    }
                    else
                    {
                        channelHandle->lastProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                    }
                }
            }
            break;

        case CY_HBDMA_DATAWIRE0_INTERRUPT:
            {
                if (channelHandle->prodSckCount > 1) {
                    if (channelHandle->prodSckId[0] != socketId) {
                        actSckIndex = 1;
                    } else {
                        actSckIndex = 0;
                    }
                }

                /* Clear the DW request queued flag. */
                channelHandle->ingressDWRqtQueued[actSckIndex] = false;

                /* If channel is in override state, send XFER_CPLT callback and stop reading data. */
                if (channelHandle->state == CY_HBDMA_CHN_OVERRIDE) {
                    channelHandle->state = CY_HBDMA_CHN_CONFIGURED;

                    /* Send notification if enabled for the channel. */
                    if (
                            (channelHandle->cbFunc != NULL) &&
                            ((channelHandle->notification & USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk) != 0)
                       ) {
                        channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_XFER_CPLT, NULL, channelHandle->cbCtx);
                    }

                    break;
                }

                /* Transfer completed on USBHS OUT endpoint. Handle based on PRODUCE_EVT. */
                if (channelHandle->prodSckCount > 1) {
                    /* Verify that transfer completed on the expected endpoint. */
                    if (actSckIndex != channelHandle->nextProdSck) {
                        /* Keep this event pending until we service the valid produce event on the other socket. */
                        channelHandle->pendingEvtCnt++;
                        break;
                    }
                }

                /* Handle the interrupt. */
                if (Cy_HBDma_IngressDW_IntrHandler(pDmaMgr, channelHandle, actSckIndex)) {

                    /* If there was a delayed produce event, handle that as well. */
                    if (channelHandle->pendingEvtCnt > 0) {
                        channelHandle->pendingEvtCnt = 0;
                        Cy_HBDma_IngressDW_IntrHandler(pDmaMgr, channelHandle, !actSckIndex);
                    }
                }
            }
            break;

        case CY_HBDMA_DATAWIRE1_INTERRUPT:
            {
                /* Identify the correct socket index. */
                if ((channelHandle->consSckCount > 1) && (channelHandle->consSckId[0] != socketId)) {
                    actSckIndex = 1;
                } else {
                    actSckIndex = 0;
                }

                /* Clear the request queued flag first. */
                channelHandle->egressDWRqtQueued[actSckIndex] = false;

                /* If channel is in override state, send XFER_CPLT callback and disable datawire. */
                if (channelHandle->state == CY_HBDMA_CHN_OVERRIDE) {
                    channelHandle->state = CY_HBDMA_CHN_CONFIGURED;

                    /* Send notification if enabled for the channel. */
                    if (
                            (channelHandle->cbFunc != NULL) &&
                            ((channelHandle->notification & USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk) != 0)
                       ) {
                        channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_XFER_CPLT, NULL, channelHandle->cbCtx);
                    }

                    break;
                }

                /* Find the socket on which next consume event was expected. */
                Cy_HBDma_GetDescriptor(channelHandle->lastProdDscr, &dscrInfo);
                expSockId = CY_HBDMA_SYNC_TO_CONS_SOCK(dscrInfo.sync);

                /* In case of a 1:2 channel, it is possible that the consume event is received
                 * out of order. In such case, we just store the consume event to be handled later. */
                if ((channelHandle->consSckCount == 1) || (socketId == expSockId)) {
                    /* Decrement commit count. */
                    if (channelHandle->commitCnt[actSckIndex] != 0) {
                        channelHandle->commitCnt[actSckIndex]--;
                    }

                    /* Update the descriptors and notify producer socket. */
                    Cy_HBDma_EgressDW_ConsEvtHandler(pDmaMgr, channelHandle, actSckIndex);

                    /* Skip over any discarded buffers. */
                    Cy_HBDma_GetDescriptor(channelHandle->curConsDscrIndex[actSckIndex], &dscrInfo);
                    while (
                            (channelHandle->discardCnt[actSckIndex] != 0) &&
                            ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) &&
                            (CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
                          ) {
                        channelHandle->discardCnt[actSckIndex]--;
                        Cy_HBDma_EgressDW_ConsEvtHandler(pDmaMgr, channelHandle, actSckIndex);
                        Cy_HBDma_GetDescriptor(channelHandle->curConsDscrIndex[actSckIndex], &dscrInfo);
                    }

                    if (channelHandle->commitCnt[actSckIndex] != 0) {
                        /* At least one more buffer has been committed. Queue the write operation for it. */
                        if ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0) {
                            Cy_HBDma_DW_QueueWrite(channelHandle, actSckIndex,
                                    CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                                    CY_HBDMA_DSCR_GET_BYTECNT(pDmaMgr->en_64k, dscrInfo.size));
                        }
                    }

                    /* Raise consume event callback. */
                    if (channelHandle->cbFunc != NULL) {
                        channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_CONS_EVENT, NULL,
                                channelHandle->cbCtx);
                    }

                    /* Process any pending consume events prematurely received from the alternate socket. */
                    if ((channelHandle->consSckCount > 1) && (channelHandle->pendingEvtCnt != 0)) {
                        channelHandle->pendingEvtCnt--;

                        actSckIndex = !actSckIndex;

                        /* Decrement commit count. */
                        if (channelHandle->commitCnt[actSckIndex] != 0) {
                            channelHandle->commitCnt[actSckIndex]--;
                        }

                        /* Update the descriptors and notify producer socket. */
                        Cy_HBDma_EgressDW_ConsEvtHandler(pDmaMgr, channelHandle, actSckIndex);

                        /* Skip over any discarded buffers. */
                        Cy_HBDma_GetDescriptor(channelHandle->curConsDscrIndex[actSckIndex], &dscrInfo);
                        while (
                                (channelHandle->discardCnt[actSckIndex] != 0) &&
                                ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0) &&
                                (CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrInfo.pBuffer))
                              ) {
                            channelHandle->discardCnt[actSckIndex]--;
                            Cy_HBDma_EgressDW_ConsEvtHandler(pDmaMgr, channelHandle, actSckIndex);
                            Cy_HBDma_GetDescriptor(channelHandle->curConsDscrIndex[actSckIndex], &dscrInfo);
                        }

                        if (channelHandle->commitCnt[actSckIndex] != 0) {
                            /* At least one more buffer has been committed. Queue the write operation for it. */
                            if ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0) {
                                Cy_HBDma_DW_QueueWrite(channelHandle, actSckIndex,
                                        CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                                        CY_HBDMA_DSCR_GET_BYTECNT(pDmaMgr->en_64k, dscrInfo.size));
                            }
                        }

                        /* Raise consume event callback. */
                        if (channelHandle->cbFunc != NULL) {
                            channelHandle->cbFunc(channelHandle, CY_HBDMA_CB_CONS_EVENT, NULL, channelHandle->cbCtx);
                        }
                    }
                } else {
                    channelHandle->pendingEvtCnt++;
                }
            }
            break;

        default:
            /* Nothing to be done. */
            break;
    }
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_DmaCallbackConfigure
 ****************************************************************************//**
 *
 * Function to configure context from which DMA callback functions are generated.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param callbackFromISREnable
 * Whether sending of DMA callbacks directly from ISR is enabled.
 *******************************************************************************/
void Cy_HBDma_Mgr_DmaCallbackConfigure (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        bool callbackFromISREnable)
{
    if (pDmaMgr != NULL) {
#if FREERTOS_ENABLE
        pDmaMgr->cbFromISREnable = callbackFromISREnable;
#endif /* FREERTOS_ENABLE */
    }
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_GetIntrDropCount
 ****************************************************************************//**
 *
 * Function to get count of DMA interrupt messages dropped due to message queue
 * overflow.
 *
 * If the count is non-zero at any stage, it indicates that DMA interrupts are
 * being generated too fast for the CPU to handle. Either larger DMA buffers
 * should be used to reduce interrupt frequency or the
 * Cy_HBDma_Mgr_RegisterISRCtrlCallback() API should be used to dynamically
 * throttle DMA interrupts.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \return
 * Count of DMA interrupt messages which have been dropped so far.
 *******************************************************************************/
uint32_t Cy_HBDma_Mgr_GetIntrDropCount (
        cy_stc_hbdma_mgr_context_t *pDmaMgr)
{
    uint32_t count = 0;

#if FREERTOS_ENABLE
    if (pDmaMgr != NULL) {
        count = pDmaMgr->queueOverflowCount;
    }
#endif /* FREERTOS_ENABLE */

    return count;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Cb
 ****************************************************************************//**
 *
 * High BandWidth DMA channel callback function implementation.
 *
 * \param socketId
 * Socket on which interrupt was received.
 *
 * \param intrType
 * Type of interrupt which was received.
 *
 * \param curDscr
 * Active descriptor for the socket.
 *
 * \param userCtx
 * User context for the callback.
 *
 * \return
 * true if further interrupt processing is to be disabled.
 *******************************************************************************/
bool Cy_HBDma_Channel_Cb (
        cy_hbdma_socket_id_t   socketId,
        cy_en_hbdma_sock_evt_t intrType,
        uint32_t               curDscr,
        void                  *userCtx)
{
    cy_stc_hbdma_mgr_context_t *pDmaMgr = (cy_stc_hbdma_mgr_context_t *)userCtx;
    cy_stc_hbdma_intr_msg_t intrMsg;
#if FREERTOS_ENABLE
    BaseType_t status;
    BaseType_t wakeTask = pdFALSE;
    bool haltIntr = false;
#endif /* FREERTOS_ENABLE */

    intrMsg.socketId = (uint32_t)socketId;
    intrMsg.intrType = (uint32_t)intrType;
    intrMsg.curDscr  = curDscr;

#if FREERTOS_ENABLE
    if (pDmaMgr->cbFromISREnable) {
        Cy_HBDma_Channel_IntrHandler(pDmaMgr, &intrMsg);
    } else {
        status = xQueueSendFromISR(pDmaMgr->dmaIntrQueue, &intrMsg, &wakeTask);
        if (status != pdPASS) {
            pDmaMgr->queueOverflowCount++;
        } else {
            /*
             * If the interrupt queue is nearly full and we have an application callback which allows DMA
             * interrupts to be disabled, block interrupts for now.
             */
            if (
                    (uxQueueMessagesWaitingFromISR(pDmaMgr->dmaIntrQueue) >= CY_HBDMA_INTRQ_FULL_THRESHOLD) &&
                    (pDmaMgr->isrCtrlCb != NULL)
               ) {
                /* Cause all HBDMA interrupts to be disabled until the queue is freed up. */
                DBG_HBDMA_TRACE("Intr disable\r\n");
                pDmaMgr->isrCtrlCb(false);
                pDmaMgr->intrDisabled = true;
                haltIntr = true;
            }
        }
    }

    return haltIntr;
#else
    Cy_HBDma_Channel_IntrHandler(pDmaMgr, &intrMsg);
    return false;
#endif /* FREERTOS_ENABLE */
}

#if FREERTOS_ENABLE
/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_TaskHandler
 ****************************************************************************//**
 *
 * High BandWidth DMA manager task.
 *
 * \param pTaskParam
 * Pointer to the HBDma manager context structure.
 *
 * \return
 * void
 *******************************************************************************/
void Cy_HBDma_Mgr_TaskHandler (
        void *pTaskParam)
{
    cy_stc_hbdma_mgr_context_t *pDmaMgr = (cy_stc_hbdma_mgr_context_t *)pTaskParam;
    cy_stc_hbdma_intr_msg_t     intrInfo;
    TickType_t                  waitPeriod = 10;
    BaseType_t                  xStatus;
    BaseType_t                  xWakeTask;
    UBaseType_t                 nMsgs;
    uint32_t                    lock;

    xStatus = xQueueReceive(pDmaMgr->dmaIntrQueue, &intrInfo, waitPeriod);
    do {
        lock = Cy_SysLib_EnterCriticalSection();
        if (xStatus == pdPASS) {
            if ((pDmaMgr->isrCtrlCb != NULL) && (pDmaMgr->intrDisabled)) {
                /* If sufficient space has been freed up, re-enable DMA interrupts. */
                if (uxQueueMessagesWaiting(pDmaMgr->dmaIntrQueue) < CY_HBDMA_INTRQ_FREE_THRESHOLD) {
                    DBG_HBDMA_TRACE("Intr enable\r\n");
                    pDmaMgr->isrCtrlCb(true);
                    pDmaMgr->intrDisabled = false;
                }
            }

            Cy_SysLib_ExitCriticalSection(lock);
            Cy_HBDma_Channel_IntrHandler(pDmaMgr, &intrInfo);

            /*
             * If another message is present in the queue when we return
             * from the pHandler, fetch it immediately and process it.
             */
            lock = Cy_SysLib_EnterCriticalSection();
            nMsgs = uxQueueMessagesWaiting(pDmaMgr->dmaIntrQueue);
            if (nMsgs != 0) {
                xStatus = xQueueReceiveFromISR(pDmaMgr->dmaIntrQueue, &intrInfo, &xWakeTask);
                Cy_SysLib_ExitCriticalSection(lock);
            } else {
                Cy_SysLib_ExitCriticalSection(lock);
                xStatus = xQueueReceive(pDmaMgr->dmaIntrQueue, &intrInfo, waitPeriod);
            }
        } else {
            Cy_SysLib_ExitCriticalSection(lock);
            xStatus = xQueueReceive(pDmaMgr->dmaIntrQueue, &intrInfo, waitPeriod);
        }
    } while (1);
}
#endif /* FREERTOS_ENABLE */

static void
ModifyDescriptorHeaderSize(
        uint16_t startDscr,
        uint16_t count,
        uint16_t hdrSize,
        uint32_t bufSize,
        bool     en_64k)
{
    cy_stc_hbdma_desc_t dscrInfo;

    while (count--) {
        Cy_HBDma_GetDescriptor(startDscr, &dscrInfo);
        dscrInfo.pBuffer += hdrSize;
        dscrInfo.size = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(en_64k, 0, bufSize);
        Cy_HBDma_SetDescriptor(startDscr, &dscrInfo);

        startDscr = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
    }
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Create
 ****************************************************************************//**
 *
 * Create a High BandWidth DMA channel based on the parameters specified in the
 * config structure.
 *
 * \param pDmaMgr
 * Pointer to the DMA manager context structure.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param config
 * Desired DMA channel configuration.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel creation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Create (
    cy_stc_hbdma_mgr_context_t *pDmaMgr,
    cy_stc_hbdma_channel_t *pHandle,
    cy_stc_hbdma_chn_config_t *config)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_BAD_PARAM;
    uint32_t dscrSync = 0, count = 0;
    uint16_t i;
    bool hs_ep_used = false;

    if ((pDmaMgr != NULL) && (pHandle != NULL) && (config != NULL))
    {
        /* Assume success by default. */
        status = CY_HBDMA_MGR_SUCCESS;

        /* Basic configuration parameter validity checks. */
        if (
                (config->chType > CY_HBDMA_TYPE_MEM_TO_IP) ||
                (config->size > CY_HBDMA_MAX_BUFFER_SIZE(pDmaMgr->en_64k)) ||
                ((config->chType != CY_HBDMA_TYPE_MEM_TO_IP) && (config->prodSckCount == 0)) ||
                ((config->chType != CY_HBDMA_TYPE_IP_TO_MEM) && (config->consSckCount == 0)) ||
                (config->prodSckCount > 2U) ||
                (config->consSckCount > 2U) ||
                ((config->prodSckCount > 1U) && (config->consSckCount > 1U)) ||
                ((config->prodSckCount > 1U) && ((config->prodSck[0] & 0xF0U) != (config->prodSck[1] & 0xF0U))) ||
                ((config->consSckCount > 1U) && ((config->consSck[0] & 0xF0U) != (config->consSck[1] & 0xF0U))) ||
                ((config->intrEnable != 0) && (config->cb == NULL))
           )
        {
            status = CY_HBDMA_MGR_BAD_PARAM;
        }

        /* Ensure that the consumer sockets to be used for the channel are not already in use. */
        if (
                (status == CY_HBDMA_MGR_SUCCESS) &&
                (config->chType != CY_HBDMA_TYPE_MEM_TO_IP)
           )
        {
            for (i = 0; i < config->prodSckCount; i++)
            {
                if (CY_HBDMA_IS_USBHS_OUT_EP(config->prodSck[i])) {
                    hs_ep_used = true;
                }

                if (Cy_HBDma_GetSocketChannel(pDmaMgr, config->prodSck[i]) != NULL)
                {
                    status = CY_HBDMA_MGR_SOCK_BUSY;
                }
            }
        }

        /* Ensure that the producer sockets to be used for the channel are not already in use. */
        if (
                (status == CY_HBDMA_MGR_SUCCESS) &&
                (config->chType != CY_HBDMA_TYPE_IP_TO_MEM)
           )
        {
            for (i = 0; i < config->consSckCount; i++)
            {
                if (CY_HBDMA_IS_USBHS_IN_EP(config->consSck[i])) {
                    hs_ep_used = true;
                }

                if (Cy_HBDma_GetSocketChannel(pDmaMgr, config->consSck[i]) != NULL)
                {
                    status = CY_HBDMA_MGR_SOCK_BUSY;
                }
            }
        }

        if ((status == CY_HBDMA_MGR_SUCCESS) && (hs_ep_used)) {
            /* A valid USB stack context pointer is required when any USB-HS endpoints are used. */
            if (pDmaMgr->pUsbStackCtx == NULL) {
                status = CY_HBDMA_MGR_NOT_SUPPORTED;
            } else {
                /*
                 * When USBHS endpoint is used in the channel, we need a valid max. packet size setting and
                 * events must not be enabled.
                 */
                if ((config->eventEnable) || (config->usbMaxPktSize == 0) || (config->usbMaxPktSize > 1024)) {
                    status = CY_HBDMA_MGR_BAD_PARAM;
                }
            }
        }
    }

    if (status == CY_HBDMA_MGR_SUCCESS)
    {
        /* Zero out the channel structure first. */
        memset((void *)pHandle, 0, sizeof(cy_stc_hbdma_channel_t));

        pHandle->pContext        = pDmaMgr;
        pHandle->type            = config->chType;
        pHandle->bufferMode      = config->bufferMode;
        pHandle->prodSckCount    = config->prodSckCount;
        pHandle->consSckCount    = config->consSckCount;
        pHandle->prodSckId[0]    = config->prodSck[0];
        pHandle->prodSckId[1]    = config->prodSck[1];
        pHandle->consSckId[0]    = config->consSck[0];
        pHandle->consSckId[1]    = config->consSck[1];
        pHandle->size            = config->size;
        pHandle->count           = config->count;
        pHandle->prodHdrSize     = 0;
        pHandle->prodBufSize     = 0;
        pHandle->eventEnable     = config->eventEnable;
        pHandle->notification    = config->intrEnable;
        pHandle->cbFunc          = config->cb;
        pHandle->cbCtx           = config->userCtx;
        pHandle->activeSckIndex  = 0x00;
        pHandle->commitCnt[0]    = 0x00;
        pHandle->commitCnt[1]    = 0x00;
        pHandle->discardCnt[0]   = 0x00;
        pHandle->discardCnt[1]   = 0x00;
        pHandle->overrideCnt     = 0x00;
        pHandle->nextProdSck     = 0x00;
        pHandle->pendingEvtCnt   = 0x00;
        pHandle->endpAddr        = config->endpAddr;

        /* Header addition is only supported in Manual mode IP -> IP channels and IP to Mem Channels. */
        if ((config->chType != CY_HBDMA_TYPE_MEM_TO_IP) && (config->eventEnable == false))
        {
            pHandle->prodHdrSize = config->prodHdrSize;
            pHandle->prodBufSize = config->prodBufSize;
        }

        count = config->count;
        if (((config->prodSckCount) > 1) || ((config->consSckCount) > 1))
        {
            /* If this is a 2:1 or 1:2 channel, we need twice as many descriptors in total. */
            count <<= 1;
        }

        /* Allocate the over-ride descriptor for the channel first. */
        status = Cy_HBDma_DscrList_Get(pDmaMgr->pDscrPool, &(pHandle->overrideDscrIndex));
        if (status == CY_HBDMA_MGR_SUCCESS)
        {
            switch (config->chType)
            {
                case CY_HBDMA_TYPE_IP_TO_IP:
                    /* Handle the One-to-One socket mapping case with events being forwarded. */
                    dscrSync = (
                            (config->consSck[0] & 0x0FU) |
                            ((config->consSck[0] & 0xF0U) << 4U) |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_EVENT_Msk |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_INT_Msk |
                            ((config->prodSck[0] & 0x0FU) << 16U) |
                            ((config->prodSck[0] & 0xF0U) << 20U) |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_EVENT_Msk |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_INT_Msk
                            );

                    status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->firstProdDscrIndex[0]),
                            count, config->size, dscrSync);
                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        /* Save the descriptor index in all relevant fields. */
                        pHandle->firstProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[0] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[1] = pHandle->firstProdDscrIndex[0];

                        /* In case of a 1:2 or 2:1 channel, update the descriptors to point to the correct sockets. */
                        if ((pHandle->prodSckCount > 1) || (pHandle->consSckCount > 1))
                        {
                            if (config->eventEnable != false)
                            {
                                /*
                                 * If this is an AUTO channel, we just need to update the descriptors
                                 * in interleaved fashion.
                                 */
                                status = Cy_HBDma_UpdateDscrChain(pHandle);
                            }
                            else
                            {
                                /*
                                 * For a Manual channel, we need to create a separate set of descriptors for
                                 * each producer/consumer socket and link them properly.
                                 */
                                status = Cy_HBDma_MultiChn_DscrSetup(pDmaMgr, pHandle, dscrSync);
                            }
                        }
                        else
                        {
                            if (
                                    (config->chType == CY_HBDMA_TYPE_IP_TO_IP) &&
                                    (config->eventEnable == false)
                               )
                            {
                                /*
                                 * If header addition is required, walk through the producer descriptor chain and
                                 * change the buffer pointer and size.
                                 */
                                if (pHandle->prodHdrSize != 0)
                                {
                                    ModifyDescriptorHeaderSize(pHandle->firstProdDscrIndex[0], count,
                                            pHandle->prodHdrSize, pHandle->prodBufSize, pHandle->pContext->en_64k);
                                }

                                /*
                                 * If events are disabled (manual channel operation), we need separate descriptors on
                                 * the producer and consumer sides. No DMA buffers are to be allocated.
                                 */
                                status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->firstConsDscrIndex[0]),
                                        count, 0, dscrSync);
                            }
                        }
                    }

                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        pHandle->curProdDscrIndex[0] = pHandle->nextProdDscr =
                            pHandle->lastProdDscr = pHandle->firstProdDscrIndex[0];
                        pHandle->curProdDscrIndex[1] = pHandle->firstProdDscrIndex[1];
                        pHandle->curConsDscrIndex[0] = pHandle->nextConsDscr = pHandle->firstConsDscrIndex[0];
                        pHandle->curConsDscrIndex[1] = pHandle->firstConsDscrIndex[1];
                    }
                    break;

                case CY_HBDMA_TYPE_IP_TO_MEM:
                    /* Currently handling single producer socket case. */
                    dscrSync = (
                            (0x3FUL << 8U) |
                            ((config->prodSck[0] & 0x0FU) << 16U) |
                            ((config->prodSck[0] & 0xF0U) << 20U) |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_EVENT_Msk |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_INT_Msk
                            );
                    status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->firstProdDscrIndex[0]),
                            count, config->size, dscrSync);
                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        /* Save the descriptor index in all relevant fields. */
                        pHandle->firstProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[0] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[1] = pHandle->firstProdDscrIndex[0];

                        status = Cy_HBDma_UpdateDscrChain(pHandle);
                    }
                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        pHandle->curProdDscrIndex[0] = pHandle->nextProdDscr =
                            pHandle->lastProdDscr = pHandle->firstProdDscrIndex[0];
                        pHandle->curProdDscrIndex[1] = pHandle->firstProdDscrIndex[1];
                        pHandle->curConsDscrIndex[0] = pHandle->nextConsDscr = pHandle->firstConsDscrIndex[0];
                        pHandle->curConsDscrIndex[1] = pHandle->firstConsDscrIndex[1];
                    }
                    break;

                case CY_HBDMA_TYPE_MEM_TO_IP:
                    /* Currently handling single consumer socket case. */
                    dscrSync = (
                            (config->consSck[0] & 0x0FU) |
                            ((config->consSck[0] & 0xF0U) << 4U) |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_EVENT_Msk |
                            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_INT_Msk |
                            (0x3FUL << 24U)
                            );
                    status = Cy_HBDma_AllocDscrChain(pDmaMgr, &(pHandle->firstProdDscrIndex[0]),
                            count, config->size, dscrSync);
                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        /* Save the descriptor index in all relevant fields. */
                        pHandle->firstProdDscrIndex[1] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[0] = pHandle->firstProdDscrIndex[0];
                        pHandle->firstConsDscrIndex[1] = pHandle->firstProdDscrIndex[0];

                        status = Cy_HBDma_UpdateDscrChain(pHandle);
                    }
                    if (status == CY_HBDMA_MGR_SUCCESS)
                    {
                        pHandle->curProdDscrIndex[0] = pHandle->nextProdDscr =
                            pHandle->lastProdDscr = pHandle->firstProdDscrIndex[0];
                        pHandle->curProdDscrIndex[1] = pHandle->firstProdDscrIndex[1];
                        pHandle->curConsDscrIndex[0] = pHandle->nextConsDscr = pHandle->firstConsDscrIndex[0];
                        pHandle->curConsDscrIndex[1] = pHandle->firstConsDscrIndex[1];
                    }
                    break;

                default:
                    status = CY_HBDMA_MGR_BAD_PARAM;
                    break;
            }
        }

        /* If all steps above are successful, update channel state. Otherwise, zero out the structure. */
        if (status == CY_HBDMA_MGR_SUCCESS)
        {
            /* Set all sockets into their default state. */
            Cy_HBDma_SetSocketDefaultState(pDmaMgr, pHandle);
            pHandle->state = CY_HBDMA_CHN_CONFIGURED;
        }
        else
        {
            /* Zero out the whole structure. */
            memset((void *)pHandle, 0, sizeof(cy_stc_hbdma_channel_t));
        }
    }

    if (status == CY_HBDMA_MGR_SUCCESS) {
        pHandle->epMaxPktSize          = config->usbMaxPktSize;
        pHandle->pProdDwDscr[0]        = NULL;
        pHandle->pProdDwDscr[1]        = NULL;
        pHandle->pConsDwDscr[0]        = NULL;
        pHandle->pConsDwDscr[1]        = NULL;
        pHandle->pCurEgressDataBuf[0]  = NULL;
        pHandle->pCurEgressDataBuf[1]  = NULL;
        pHandle->curEgressXferSize[0]  = 0;
        pHandle->curEgressXferSize[1]  = 0;
        pHandle->curIngressXferSize[0] = 0;
        pHandle->curIngressXferSize[1] = 0;

        if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP) {
            if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[0])) {
                pHandle->pProdDwDscr[0] = &(pDmaMgr->dwDscrList[3 * (pHandle->prodSckId[0] & 0x0F)]);
                if ((pHandle->prodSckCount > 1U) && (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[1]))) {
                    pHandle->pProdDwDscr[1] = &(pDmaMgr->dwDscrList[3 * (pHandle->prodSckId[1] & 0x0F)]);
                }
            }
        }

        if (pHandle->type != CY_HBDMA_TYPE_IP_TO_MEM) {
            if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[0])) {
                pHandle->pConsDwDscr[0] = &(pDmaMgr->dwDscrList[48 + 3 * (pHandle->consSckId[0] & 0x0F)]);
                if ((pHandle->consSckCount > 1U) && (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[1]))) {
                    pHandle->pConsDwDscr[1] = &(pDmaMgr->dwDscrList[48 + 3 * (pHandle->consSckId[1] & 0x0F)]);
                }
            }
        }

        /* Configure DataWire resources. */
        Cy_HBDma_DW_Configure(pHandle, true);

        /* Clear the trigger done flags initially. */
        pHandle->egressDWTrigDone[0] = false;
        pHandle->egressDWTrigDone[1] = false;
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Destroy
 ****************************************************************************//**
 *
 * Destroy a High BandWidth DMA channel. The implementation makes sure that the
 * DMA sockets associated with the channel are disabled in addition to cleaning
 * up the data structures and freeing memory elements.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel destroy operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Destroy (
    cy_stc_hbdma_channel_t *pHandle)
{
    cy_stc_hbdma_mgr_context_t *pDmaMgr;
    uint16_t i;

    if ((pHandle == NULL) || (pHandle->pContext == NULL))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    pDmaMgr = pHandle->pContext;

    /* Free up DataWire resources. */
    Cy_HBDma_DW_Configure(pHandle, false);

    /* Clear the trigger done flags on destroy. */
    pHandle->egressDWTrigDone[0] = false;
    pHandle->egressDWTrigDone[1] = false;

    /* First disable the sockets associated with the channel and mark them free. */
    if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP)
    {
        for (i = 0; i < pHandle->prodSckCount; i++)
        {
            if (!CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[i])) {
                Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);
                Cy_HbDma_DisconnectEventTriggers(pDmaMgr->pDrvContext, pHandle->prodSckId[i]);
            }
            Cy_HBDma_SetSocketChannel(pDmaMgr, pHandle->prodSckId[i], NULL);
        }
    }

    if (pHandle->type != CY_HBDMA_TYPE_IP_TO_MEM)
    {
        for (i = 0; i < pHandle->consSckCount; i++)
        {
            if (!CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[i])) {
                Cy_HBDma_SocketDisable(pDmaMgr->pDrvContext, pHandle->consSckId[i]);
                Cy_HbDma_DisconnectEventTriggers(pDmaMgr->pDrvContext, pHandle->consSckId[i]);
            }
            Cy_HBDma_SetSocketChannel(pDmaMgr, pHandle->consSckId[i], NULL);
        }
    }

    if (pHandle->prodSckCount > 1)
    {
        /* Free the descriptor chains for each of the producer sockets. */
        Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstProdDscrIndex[1], pHandle->count, pHandle->prodHdrSize,
                true, true);
        Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstProdDscrIndex[0], pHandle->count, pHandle->prodHdrSize,
                true, true);

        /* If this is a manual channel (consumer descriptor is different from producer descriptor),
         * free the consumer descriptor chain as well. This will have 2X the number of descriptors. */
        if (pHandle->firstConsDscrIndex[0] != pHandle->firstProdDscrIndex[0])
        {
            Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[0], pHandle->count * 2, 0,
                    false, false);
        }
    }
    else
    {
        if (pHandle->consSckCount > 1)
        {
            if (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP)
            {
                /* Free the descriptor chain for the producer socket. This will have 2X the number
                 * of descriptors. */
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstProdDscrIndex[0], pHandle->count * 2,
                        pHandle->prodHdrSize, true, true);

                /* If this is a manual channel, we need to free the consumer descriptor chains as well. */
                if (pHandle->firstConsDscrIndex[0] != pHandle->firstProdDscrIndex[0])
                {
                    Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[0], pHandle->count, 0,
                            false, false);
                    Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[1], pHandle->count, 0,
                            false, false);
                }
            }
            else
            {
                /* Free the descriptor chains for each of the consumer sockets. */
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[0], pHandle->count, 0,
                        false, true);
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[1], pHandle->count, 0,
                        false, true);
            }
        }
        else
        {
            /* Free the descriptor chain for the producer socket. */
            Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstProdDscrIndex[0], pHandle->count, pHandle->prodHdrSize,
                    (pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP), true);

            /* If this is a manual channel (consumer descriptor is different from producer descriptor),
             * free the consumer descriptor chain as well. */
            if (pHandle->firstConsDscrIndex[0] != pHandle->firstProdDscrIndex[0])
            {
                Cy_HBDma_FreeDscrChain(pDmaMgr, pHandle->firstConsDscrIndex[0], pHandle->count, 0,
                        false, false);
            }
        }
    }

    /* Free the over-ride descriptor. */
    Cy_HBDma_DscrList_Put(pDmaMgr->pDscrPool, pHandle->overrideDscrIndex);

    /* Zero out the channel pHandle structure. */
    memset((void *)pHandle, 0, sizeof(cy_stc_hbdma_channel_t));

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Enable
 ****************************************************************************//**
 *
 * Enable a High BandWidth DMA channel for data transfer. Any sockets associated
 * with the DMA channel will be enabled. If a non-zero xferSize is specified,
 * the channel gets disabled automatically after transferring the specified
 * amount of data.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param xferSize
 * Size of data to be transferred through the channel.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel enable operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Enable (
    cy_stc_hbdma_channel_t *pHandle,
    uint32_t xferSize)
{
    if ((pHandle == NULL) || (pHandle->pContext == NULL) || (pHandle->count == 0))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /*
     * Only infinite transfer size is supported when we are using USBHS endpoints as
     * producer or consumer.
     */
    if (((pHandle->pProdDwDscr[0] != NULL) || (pHandle->pConsDwDscr[0] != NULL)) && (xferSize != 0)) {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->state != CY_HBDMA_CHN_CONFIGURED)
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    /* Store the transfer size requested. */
    pHandle->xferSize = xferSize;
    pHandle->state = CY_HBDMA_CHN_ACTIVE;
    pHandle->pendingEvtCnt = 0;
    pHandle->nextProdSck   = 0;
    pHandle->commitCnt[0]  = 0;
    pHandle->commitCnt[1]  = 0;
    pHandle->discardCnt[0] = 0;
    pHandle->discardCnt[1] = 0;
    pHandle->overrideCnt   = 0;

    /* Make sure DataWire trigger connections are made. */
    Cy_HBDma_DW_Configure(pHandle, true);

    /* Configure and enable the sockets. */
    Cy_HBDma_SetSocketEnabled(pHandle->pContext, pHandle, xferSize);

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Disable
 ****************************************************************************//**
 *
 * Disable a High BandWidth DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel disable operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Disable (
    cy_stc_hbdma_channel_t *pHandle)
{
    return Cy_HBDma_Channel_Reset(pHandle);
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Reset
 ****************************************************************************//**
 *
 * Reset a High BandWidth DMA channel. This leaves all the DMA buffers associated
 * with the channel in the empty state and the channel itself in the disabled state.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel reset operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Reset (
        cy_stc_hbdma_channel_t *pHandle)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint16_t dscrIndex, i, j;
    uint16_t dscrCount;

    if ((pHandle == NULL) || (pHandle->pContext == NULL))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Make sure DataWire trigger connections are broken. */
    Cy_HBDma_DW_Configure(pHandle, false);

    /* Clear the trigger done flags on channel reset. */
    pHandle->egressDWTrigDone[0] = false;
    pHandle->egressDWTrigDone[1] = false;

    /* Restore all sockets to their default state. */
    Cy_HBDma_SetSocketDefaultState(pHandle->pContext, pHandle);

    /* Go through all the descriptors and mark them unoccupied. */
    for (j = 0; j < pHandle->prodSckCount; j++)
    {
        dscrIndex = pHandle->firstProdDscrIndex[j];
        dscrCount = (pHandle->consSckCount > 1) ? (pHandle->count * 2) : pHandle->count;
        for (i = 0; i < dscrCount; i++)
        {
            Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
            dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
            dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
            Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);
            dscrIndex = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                    >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
        }
    }

    if (pHandle->firstProdDscrIndex[0] != pHandle->firstConsDscrIndex[0])
    {
        for (j = 0; j < pHandle->consSckCount; j++)
        {
            dscrIndex = pHandle->firstConsDscrIndex[j];
            dscrCount = (pHandle->prodSckCount > 1) ? (pHandle->count * 2) : pHandle->count;
            for (i = 0; i < dscrCount; i++)
            {
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
                dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
                dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
                Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);
                dscrIndex = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk)
                        >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos);
            }
        }
    }

    /* Mark the override descriptor unoccupied. */
    Cy_HBDma_GetDescriptor(pHandle->overrideDscrIndex, &dscrInfo);
    dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
    dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
    Cy_HBDma_SetDescriptor(pHandle->overrideDscrIndex, &dscrInfo);

    /* Reset current descriptors for all sockets. */
    pHandle->curProdDscrIndex[0] = pHandle->nextProdDscr =
        pHandle->lastProdDscr = pHandle->firstProdDscrIndex[0];
    pHandle->curProdDscrIndex[1] = pHandle->firstProdDscrIndex[1];
    pHandle->curConsDscrIndex[0] = pHandle->nextConsDscr = pHandle->firstConsDscrIndex[0];
    pHandle->curConsDscrIndex[1] = pHandle->firstConsDscrIndex[1];

    /* Reset channel state. */
    pHandle->activeSckIndex = 0x00;
    pHandle->commitCnt[0]   = 0x00;
    pHandle->commitCnt[1]   = 0x00;
    pHandle->discardCnt[0]  = 0x00;
    pHandle->discardCnt[1]  = 0x00;
    pHandle->overrideCnt    = 0x00;
    pHandle->nextProdSck    = 0x00;
    pHandle->pendingEvtCnt  = 0x00;
    pHandle->state          = CY_HBDMA_CHN_CONFIGURED;

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_SetWrapUp
 ****************************************************************************//**
 *
 *  This API is used to forcibly commit a DMA buffer to the consumer, and is
 *  useful in the case where data transfer has abruptly stopped without the
 *  producer being able to commit the data buffer.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckOffset
 *  Socket id to wrapup.
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel reset operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the DMA channel is not in the required state.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_SetWrapUp (cy_stc_hbdma_channel_t *pHandle, uint8_t sckOffset)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    if ((pHandle == NULL) || (pHandle->pContext == NULL) ||
            (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) || (sckOffset >= pHandle->prodSckCount)) {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* This API can be called only if the channel is in active mode */
    if (pHandle->state != CY_HBDMA_CHN_ACTIVE) {
        status = CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    if ((status == CY_HBDMA_MGR_SUCCESS) && (!CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[sckOffset]))) {
        /* If this is a high bandwidth DMA socket, trigger wrap-up operation. */
        Cy_HBDma_SocketSetWrapUp(pHandle->pContext->pDrvContext,pHandle->prodSckId[sckOffset]);
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetBuffer
 ****************************************************************************//**
 *
 * Get the status of the active DMA buffer associated with the DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Return parameter to pass the buffer status through.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the API is called when the descriptor is not
 * in the expected state.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_GetBuffer (
        cy_stc_hbdma_channel_t *pHandle,
        cy_stc_hbdma_buff_status_t *bufStat_p)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t dscrIndex;
    bool     occupied = false;
    bool     marked = false;

    if (((pHandle == NULL) || (bufStat_p == NULL)) ||
        ((pHandle->prodSckCount > 1) && (pHandle->consSckCount > 1)))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP)
    {
        dscrIndex = pHandle->nextConsDscr;
    }
    else
    {
        dscrIndex = pHandle->nextProdDscr;
    }

    /* Fetch the descriptor content. If MARKER is set in the descriptor, it has already been processed and
     * should be treated as invalid. */
    Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
    occupied = ((dscr.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) != 0);
    marked   = CY_HBDMA_IS_DESCRIPTOR_MARKED(dscr.pBuffer);

    /*
     * If this is a IP -> MEM or IP -> IP channel, the buffer should be occupied and unmarked.
     * If this is a MEM -> IP channel, the buffer should be empty and unmarked.
     */
    if (
            ((pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) && ((occupied) || (marked))) ||
            ((pHandle->type != CY_HBDMA_TYPE_MEM_TO_IP) && ((!occupied) || (marked)))
       )
    {
        /* Return error as descriptor is not in the expected state. */
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    bufStat_p->pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscr.pBuffer);
    bufStat_p->size    = CY_HBDMA_DSCR_GET_BUFSIZE(pHandle->pContext->en_64k, dscr.size);
    bufStat_p->count   = CY_HBDMA_DSCR_GET_BYTECNT(pHandle->pContext->en_64k, dscr.size);
    bufStat_p->status  = (dscr.size & 0x0EUL);

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_CommitBuffer
 ****************************************************************************//**
 *
 * Mark a DMA buffer occupied on the consumer side of the DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Information about the buffer to be committed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the commit operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_CommitBuffer (
        cy_stc_hbdma_channel_t *pHandle,
        cy_stc_hbdma_buff_status_t *bufStat_p)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint32_t dscrSize = 0;
    uint8_t consSckIndex = 0;

    /* Don't allow commit operation if event signalling between sockets is enabled. */
    if ((pHandle == NULL) || (bufStat_p == NULL) || (pHandle->pContext == NULL) ||
            (bufStat_p->count > CY_HBDMA_MAX_BUFFER_SIZE(pHandle->pContext->en_64k)) ||
            (pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) || (pHandle->eventEnable != false))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->consSckCount > 1)
    {
        consSckIndex = pHandle->activeSckIndex;
    }

    /* Update the next consume descriptor with the desired buffer pointer, size and status. */
    Cy_HBDma_GetDescriptor(pHandle->nextConsDscr, &dscrInfo);

    dscrSize = (bufStat_p->status & 0x0EU) | LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk;
    dscrSize = CY_HBDMA_DSCR_SET_BYTECNT(pHandle->pContext->en_64k, dscrSize, bufStat_p->count);
    if (bufStat_p->size < bufStat_p->count)
    {
        /* If count is larger than size (header addition), update the size field to match the count value. */
        dscrSize = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pHandle->pContext->en_64k, dscrSize, bufStat_p->count);
    }
    else
    {
        dscrSize = CY_HBDMA_DSCR_SET_BUFSIZE(pHandle->pContext->en_64k, dscrSize, bufStat_p->size);
    }

    /* Update the descriptor with the correct data pointer and size and mark it occupied. */
    dscrInfo.pBuffer = bufStat_p->pBuffer;
    dscrInfo.size    = dscrSize;

    /* Set marker bit in case of MEM_TO_IP channel. */
    if (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP)
    {
        dscrInfo.pBuffer = CY_HBDMA_MARK_DSCR_BUFFER(dscrInfo.pBuffer);
    }

    Cy_HBDma_SetDescriptor(pHandle->nextConsDscr, &dscrInfo);

    /* Need to move the next consume descriptor index. */
    if (pHandle->consSckCount > 1)
    {
        /* Follow the next WR_NEXT pointer in case this is a 1:2 channel. */
        pHandle->nextConsDscr = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
        pHandle->activeSckIndex ^= 1;
    }
    else
    {
        pHandle->nextConsDscr = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk)
                >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos);
    }

    /* If this is an IP -> IP channel, move the next produce descriptor as well. */
    if (pHandle->type == CY_HBDMA_TYPE_IP_TO_IP)
    {
        /* Set the marker bit in the descriptor to indicate it has been processed already. */
        Cy_HBDma_GetDescriptor(pHandle->nextProdDscr, &dscrInfo);
        dscrInfo.pBuffer = CY_HBDMA_MARK_DSCR_BUFFER(dscrInfo.pBuffer);
        Cy_HBDma_SetDescriptor(pHandle->nextProdDscr, &dscrInfo);

        if (pHandle->prodSckCount > 1)
        {
            /* Follow the RD_NEXT pointer in case this is a 2:1 channel. */
            pHandle->nextProdDscr = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk)
                    >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos);
        }
        else
        {
            pHandle->nextProdDscr = ((dscrInfo.chain & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk)
                    >> LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos);
        }
    }

    /* Increment the commit count for the consumer socket. */
    pHandle->commitCnt[consSckIndex] = pHandle->commitCnt[consSckIndex] + 1;

    if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[consSckIndex])) {
        /* If there is no ongoing write operation on the endpoint, queue the data write. */
        if (pHandle->egressDWRqtQueued[consSckIndex] == false) {
            Cy_HBDma_DW_QueueWrite(pHandle, consSckIndex, bufStat_p->pBuffer, bufStat_p->count);
        }
    } else {
        /* Send a produce event to the appropriate consumer socket. */
        Cy_HBDma_SendSocketEvent(pHandle->pContext->pDrvContext, pHandle->consSckId[consSckIndex], true);
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_DiscardBuffer
 ****************************************************************************//**
 *
 * Mark a DMA buffer empty on the producer side of the DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Information about the buffer to be dropped.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the discard operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_DiscardBuffer (
        cy_stc_hbdma_channel_t *pHandle,
        cy_stc_hbdma_buff_status_t *bufStat_p)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint32_t dscrChain;
    uint8_t consSckIndex = 0;

    /* Operation only valid on IP to MEM and IP to IP channels. */
    if ((pHandle == NULL) || (bufStat_p == NULL) || (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Discard is to be done by marking the active descriptor empty and moving to the next one. */
    Cy_HBDma_GetDescriptor(pHandle->nextProdDscr, &dscrInfo);

    if ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0)
    {
        DBG_HBDMA_ERR("ChnDiscard: Buffer empty\r\n");
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    /* Return information about the buffer that is being discarded. */
    bufStat_p->pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
    bufStat_p->size    = CY_HBDMA_DSCR_GET_BUFSIZE(pHandle->pContext->en_64k, dscrInfo.size);
    bufStat_p->count   = CY_HBDMA_DSCR_GET_BYTECNT(pHandle->pContext->en_64k, dscrInfo.size);

    if (pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM)
    {
        /* Mark the buffer as free */
        dscrInfo.pBuffer = CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer);
        dscrInfo.size    = dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
        Cy_HBDma_SetDescriptor(pHandle->nextProdDscr, &dscrInfo);
        dscrChain = dscrInfo.chain;

        if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[pHandle->activeSckIndex])) {
            /* If we do not have a read operation queued on the endpoint, queue one now. */
            if (pHandle->ingressDWRqtQueued[pHandle->activeSckIndex] == false) {
                Cy_HBDma_GetDescriptor(pHandle->curProdDscrIndex[pHandle->activeSckIndex], &dscrInfo);
                Cy_HBDma_DW_QueueRead(pHandle, pHandle->activeSckIndex,
                        CY_HBDMA_GET_BUFFER_ADDRESS(dscrInfo.pBuffer),
                        CY_HBDMA_DSCR_GET_BUFSIZE(pHandle->pContext->en_64k, dscrInfo.size));
            }
        } else {
            /* Send a consume event to the socket where buffer is being discarded. */
            Cy_HBDma_SendSocketEvent(pHandle->pContext->pDrvContext, pHandle->prodSckId[pHandle->activeSckIndex], false);
        }

        /* Move the active descriptor to the next one. */
        if (pHandle->prodSckCount > 1)
        {
            pHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrChain);

            /* Switch active socket index if we have more than 1 producer. */
            pHandle->activeSckIndex ^= 1;
        }
        else
        {
            pHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrChain);
        }
    }
    else
    {
        /*
         * Discarding a buffer on a Manual Channel is a bit tricky.
         * It is possible that the consumer socket is still working on a previous descriptor and it might
         * switch to the new descriptor when we are doing the updates required to do buffer discard.
         * To avoid race conditions, the following procedure is used:
         * 1. The consume descriptor corresponding to this buffer is left unoccupied with the marker bit set.
         * 2. STALL interrupt is enabled for the consumer socket.
         * 3. Socket will STALL and raise interrupt when it reaches the correct descriptor.
         * 4. At this point, we can modify the socket to point to the next descriptor.
         * 5. A count of discard calls made by the user is maintained to handle cases where multiple
         *    Discard and/or Commit calls are made back to back.
         */
        /* Set the marker bit in the descriptor to indicate it has been processed already. */
        dscrInfo.pBuffer = CY_HBDMA_MARK_DSCR_BUFFER(dscrInfo.pBuffer);
        Cy_HBDma_SetDescriptor(pHandle->nextProdDscr, &dscrInfo);

        /* Move to the next produce descriptor index. */
        if (pHandle->prodSckCount > 1)
        {
            /* Follow the RD_NEXT pointer in case this is a 2:1 channel. */
            pHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
        }
        else
        {
            pHandle->nextProdDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
        }

        if (pHandle->consSckCount > 1)
        {
            consSckIndex = pHandle->activeSckIndex;
        }

        /* Get the next consumer descriptor. */
        Cy_HBDma_GetDescriptor(pHandle->nextConsDscr, &dscrInfo);

        /* Leave the descriptor unoccupied and set the marker bit. */
        dscrInfo.size    = (dscrInfo.size & ~LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk);
        dscrInfo.pBuffer = CY_HBDMA_MARK_DSCR_BUFFER(dscrInfo.pBuffer);
        Cy_HBDma_SetDescriptor(pHandle->nextConsDscr, &dscrInfo);

        /* If this is the first discard call, enable the STALL interrupt for the socket. */
        if (pHandle->discardCnt[consSckIndex] == 0)
        {
            /* Increment the pending discard count for this socket. */
            pHandle->discardCnt[consSckIndex]++;

            if (!CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[consSckIndex])) {
                Cy_HBDma_UpdateSockIntrMask(pHandle->pContext->pDrvContext,
                        pHandle->consSckId[consSckIndex],
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_MASK_STALL_Msk,
                        true);
            }
        }
        else
        {
            pHandle->discardCnt[consSckIndex]++;
        }

        /* Move to the next consume descriptor index. */
        if (pHandle->consSckCount > 1)
        {
            /* Follow the next WR_NEXT pointer in case this is a 1:2 channel. */
            pHandle->nextConsDscr = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
            pHandle->activeSckIndex ^= 1;
        }
        else
        {
            pHandle->nextConsDscr = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
        }
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_SendData
 ****************************************************************************//**
 *
 * Prepares to send data through a DMA channel. This API should be called when
 * the channel is in the disabled state, and will return after the DMA operation
 * has been queued. The Cy_HBDma_Channel_WaitForSendCplt function can be called
 * to wait until the transfer is completed.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of consumer socket through which data is to be sent.
 *
 * \param dataBuf_p
 * Pointer to the buffer containing data to be sent.
 *
 * \param dataSize
 * Size of data to be sent.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the data to be sent is queued successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not idle.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_SendData (
        cy_stc_hbdma_channel_t *pHandle,
        uint16_t sckIdx,
        uint8_t *dataBuf_p,
        uint32_t dataSize)
{
    cy_stc_hbdma_sockconfig_t sckConf;
    cy_stc_hbdma_desc_t dscrInfo;
    uint32_t dscrSync;
    uint32_t dscrSize = 0;

    if ((pHandle == NULL)  || (pHandle->pContext == NULL)  ||
            (pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) || (sckIdx >= pHandle->consSckCount) ||
            (dataSize > CY_HBDMA_MAX_BUFFER_SIZE(pHandle->pContext->en_64k)) || (dataBuf_p == NULL) ||
            ((uint32_t)dataBuf_p <= CY_HBW_SRAM_BASE_ADDR) || ((uint32_t)dataBuf_p > CY_HBW_SRAM_LAST_ADDR))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->state != CY_HBDMA_CHN_CONFIGURED)
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    dscrSync = (
            (pHandle->consSckId[sckIdx] & 0x0FU) |
            ((pHandle->consSckId[sckIdx] & 0xF0U) << 4U) |
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_EVENT_Msk |
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_CONS_INT_Msk |
            (0x3FUL << 24U)
            );

    dscrSize = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pHandle->pContext->en_64k, dscrSize, dataSize);
    dscrSize = CY_HBDMA_DSCR_SET_BYTECNT(pHandle->pContext->en_64k, dscrSize, dataSize);
    dscrSize |= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk;

    /* Configure the over-ride DMA descriptor. */
    dscrInfo.pBuffer  = dataBuf_p;
    dscrInfo.sync     = dscrSync;
    dscrInfo.chain    = 0x0000FFFFUL | (pHandle->overrideCnt << 16U);
    dscrInfo.size     = dscrSize;

    if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[sckIdx])) {
        if (dataSize > CY_HBDMA_MAX_BUFFER_SIZE(0)) {
            return CY_HBDMA_MGR_BAD_PARAM;
        }

        /* Set channel state as active. */
        pHandle->overrideCnt++;
        pHandle->state = CY_HBDMA_CHN_OVERRIDE;

        Cy_HBDma_DW_Configure(pHandle, true);
        Cy_HBDma_DW_QueueWrite(pHandle, sckIdx, dataBuf_p, (uint16_t)dataSize);
    } else {
        /* Make sure the socket is disabled before queueing the next write. */
        Cy_HBDma_SocketDisable(pHandle->pContext->pDrvContext, pHandle->consSckId[sckIdx]);

        /* Set channel state as active. */
        pHandle->overrideCnt++;
        pHandle->state = CY_HBDMA_CHN_OVERRIDE;

        /* Delay added to ensure synchronization. */
        Cy_SysLib_DelayUs(50);

        Cy_HBDma_SetDescriptor(pHandle->overrideDscrIndex, &dscrInfo);

        /* Point the consumer socket to the override descriptor and enable it. */
        sckConf.actDscrIndex  = pHandle->overrideDscrIndex;
        sckConf.reqXferSize = 1;
        sckConf.intrMask = (pHandle->cbFunc != NULL) ? USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk : 0UL;
        sckConf.status    = (
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_CONS_EVENTS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_UNIT_Msk);
        Cy_HBDma_SetSocketConfig(pHandle->pContext->pDrvContext, pHandle->consSckId[sckIdx], &sckConf);
        Cy_HBDma_SocketEnable(pHandle->pContext->pDrvContext, pHandle->consSckId[sckIdx]);
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_WaitForSendCplt
 ****************************************************************************//**
 *
 * Waits until the DMA transfer requested using Cy_HBDma_Channel_SendData API
 * has been completed.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of consumer socket through which data is to be sent.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the send operation was completed.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if SendData has not been called previously.
 * CY_HBDMA_MGR_TIMEOUT if the operation times out on the consumer side.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_WaitForSendCplt (
        cy_stc_hbdma_channel_t *pHandle,
        uint16_t  sckIdx)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    cy_stc_hbdma_sock_t sckStat;
    uint32_t pollCnt = 0;

    if ((pHandle == NULL) || (pHandle->pContext == NULL) ||
            (pHandle->type == CY_HBDMA_TYPE_IP_TO_MEM) || (sckIdx >= pHandle->consSckCount))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if ((pHandle->state != CY_HBDMA_CHN_OVERRIDE) && (pHandle->state != CY_HBDMA_CHN_CONFIGURED))
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    if (CY_HBDMA_IS_USBHS_IN_EP(pHandle->consSckId[sckIdx])) {
        /* Wait until DMA request has been completed. */
        do {
            pollCnt++;
            if (pollCnt > 100) {
#if FREERTOS_ENABLE
                vTaskDelay(1);
#else
                Cy_SysLib_Delay(1);
#endif /* FREERTOS_ENABLE */

                if (pollCnt > 2500) {
                    status = CY_HBDMA_MGR_TIMEOUT;
                    break;
                }
            }
        } while (pHandle->egressDWRqtQueued[sckIdx] == true);

        if (status == CY_HBDMA_MGR_TIMEOUT) {
            Cy_HBDma_DW_Configure(pHandle, false);
        }
    } else {
        /* Wait until the DMA transfer is finished. We cannot rely on Consume Event interrupt due to
         * spurious interrupts being raised. So, waiting for socket to get suspended. */
        do {
            Cy_HBDma_GetSocketStatus(pHandle->pContext->pDrvContext, pHandle->consSckId[sckIdx], &sckStat);

            pollCnt++;
            if (pollCnt > 100) {
#if FREERTOS_ENABLE
                vTaskDelay(1);
#else
                Cy_SysLib_Delay(1);
#endif /* FREERTOS_ENABLE */

                if (pollCnt > 2500) {
                    DBG_HBDMA_ERR("SendData TO %x %x %x %x %x %x\r\n",
                            sckStat.status, sckStat.actDscr.size,
                            USB32DEV->USB32DEV_EPM.EEPM_ENDPOINT[0],
                            USB32DEV->USB32DEV_PROT.PROT_CS,
                            USB32DEV->USB32DEV_PROT.PROT_EPI_CS1[0], USB32DEV->USB32DEV_PROT.PROT_EPI_CS2[0]);

                    status = CY_HBDMA_MGR_TIMEOUT;
                    break;
                }
            }
        } while (
                ((sckStat.status & LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk) != 0) &&
                ((sckStat.intrStatus & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_SUSPEND_Msk) == 0)
                );

        /* Disable the socket and reset channel state. */
        Cy_HBDma_SocketDisable(pHandle->pContext->pDrvContext, pHandle->consSckId[sckIdx]);
    }

    pHandle->state = CY_HBDMA_CHN_CONFIGURED;
    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_ReceiveData
 ****************************************************************************//**
 *
 * Prepare to receive a specific amount of data using a DMA channel. This API
 * should be called while the channel is in the disabled state and will return
 * as soon as channel is enabled for data transfer.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of the producer socket within the channel through which data will be received.
 *
 * \param dataBuf_p
 * Pointer of buffer where the received data should be placed.
 *
 * \param bufferSize
 * Maximum amount of data that may be received.
 *
 * \param actualSize_p
 * Return parameter to get the actual received data size through (can be NULL).
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the receive operation is queued successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not idle.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_ReceiveData (
        cy_stc_hbdma_channel_t *pHandle,
        uint16_t  sckIdx,
        uint8_t  *dataBuf_p,
        uint32_t  bufferSize,
        uint32_t *actualSize_p)
{
    cy_stc_hbdma_sockconfig_t sckConf;
    cy_stc_hbdma_desc_t dscrInfo;
    uint32_t dscrSync;

    if ((pHandle == NULL) || (pHandle->pContext == NULL) ||
            (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) || (sckIdx >= pHandle->prodSckCount) ||
            (dataBuf_p == NULL) || (bufferSize == 0) ||
            (bufferSize > CY_HBDMA_MAX_BUFFER_SIZE(pHandle->pContext->en_64k)) ||
            ((uint32_t)dataBuf_p <= CY_HBW_SRAM_BASE_ADDR) || ((uint32_t)dataBuf_p > CY_HBW_SRAM_LAST_ADDR))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->state != CY_HBDMA_CHN_CONFIGURED)
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    dscrSync = (
            (0x3FUL << 8U) |
            ((pHandle->prodSckId[sckIdx] & 0x0FU) << 16U) |
            ((pHandle->prodSckId[sckIdx] & 0xF0U) << 20U) |
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_EVENT_Msk |
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SYNC_EN_PROD_INT_Msk
            );

    /* Configure the over-ride DMA descriptor. */
    dscrInfo.pBuffer  = dataBuf_p;
    dscrInfo.sync     = dscrSync;
    dscrInfo.chain    = 0xFFFF0000UL | pHandle->overrideCnt;
    dscrInfo.size     = CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(pHandle->pContext->en_64k, 0, bufferSize);

    if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[sckIdx])) {
        if (bufferSize > CY_HBDMA_MAX_BUFFER_SIZE(0)) {
            return CY_HBDMA_MGR_BAD_PARAM;
        }

        /* Set channel state as active. */
        pHandle->overrideCnt++;
        pHandle->state = CY_HBDMA_CHN_OVERRIDE;

        Cy_HBDma_DW_Configure(pHandle, true);
        Cy_HBDma_DW_QueueRead(pHandle, sckIdx, dataBuf_p, bufferSize);
    } else {
        /* Make sure the socket is disabled before queueing the next write. */
        Cy_HBDma_SocketDisable(pHandle->pContext->pDrvContext, pHandle->prodSckId[sckIdx]);

        /* Delay added to ensure synchronization. */
        Cy_SysLib_DelayUs(50);

        /* Set channel state as active. */
        pHandle->overrideCnt++;
        pHandle->state = CY_HBDMA_CHN_OVERRIDE;

        Cy_HBDma_SetDescriptor(pHandle->overrideDscrIndex, &dscrInfo);

        /* Point the consumer socket to the override descriptor and enable it. */
        sckConf.actDscrIndex  = pHandle->overrideDscrIndex;
        sckConf.reqXferSize = 1;
        sckConf.intrMask = (pHandle->cbFunc != NULL) ? USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk : 0UL;
        sckConf.status    = (
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_TRUNCATE_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_EN_PROD_EVENTS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_SUSP_TRANS_Msk |
                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_UNIT_Msk);
        Cy_HBDma_SetSocketConfig(pHandle->pContext->pDrvContext, pHandle->prodSckId[sckIdx], &sckConf);
        Cy_HBDma_SocketEnable(pHandle->pContext->pDrvContext, pHandle->prodSckId[sckIdx]);
    }

    (void)actualSize_p;

    return CY_HBDMA_MGR_SUCCESS;
}

/** Get dscr_size value from the dscr_no DMA descriptor. */
#define CY_HBDMA_GET_DESC_SIZE(dscr_no) \
    (*((uint32_t *)((CY_HBW_SRAM_BASE_ADDR) + ((dscr_no) * CY_HBDMA_DESC_SIZE) + 0x0CU)))

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_WaitForReceiveCplt
 ****************************************************************************//**
 *
 * Wait until a HBW DMA operation initiated using Cy_HBDma_Channel_ReceiveData
 * has been completed. The actual amount of data read is returned through the
 * actualSize_p parameter.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of the producer socket in the channel to be used for data read.
 *
 * \param actualSize_p
 * Optional return parameter to get the actual size of data received.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the data is received successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if a previous ReceiveData call is not pending.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_WaitForReceiveCplt (
        cy_stc_hbdma_channel_t *pHandle,
        uint16_t  sckIdx,
        uint32_t *actualSize_p)
{
    cy_stc_hbdma_sock_t sckStat;
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint32_t tmp = 0;
    uint32_t intState;
    uint32_t pollCnt = 0;

    if ((pHandle == NULL) || (pHandle->pContext == NULL) ||
            (pHandle->type == CY_HBDMA_TYPE_MEM_TO_IP) || (sckIdx >= pHandle->prodSckCount))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Allow API to go through immediately after the transfer has been completed on the channel. */
    if ((pHandle->state != CY_HBDMA_CHN_OVERRIDE) && (pHandle->state != CY_HBDMA_CHN_CONFIGURED))
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    if (CY_HBDMA_IS_USBHS_OUT_EP(pHandle->prodSckId[sckIdx])) {
        /* Wait until DMA request has been completed. */
        do {
            pollCnt++;
            if (pollCnt > 100) {
#if FREERTOS_ENABLE
                vTaskDelay(1);
#else
                Cy_SysLib_Delay(1);
#endif /* FREERTOS_ENABLE */

                if (pollCnt > 2500) {
                    status = CY_HBDMA_MGR_TIMEOUT;
                    break;
                }
            }
        } while (pHandle->ingressDWRqtQueued[sckIdx] == true);

        if (status == CY_HBDMA_MGR_TIMEOUT) {
            Cy_HBDma_DW_Configure(pHandle, false);
        } else {
            if (actualSize_p != NULL) {
                *actualSize_p = pHandle->curIngressXferSize[sckIdx];
            }
        }
    } else {
        /* Wait until the DMA transfer is finished. */
        do {
            Cy_HBDma_GetSocketStatus(pHandle->pContext->pDrvContext, pHandle->prodSckId[sckIdx], &sckStat);

            tmp++;
            if (tmp > 10) {
#if FREERTOS_ENABLE
                vTaskDelay(1);
#else
                Cy_SysLib_Delay(1);
#endif /* FREERTOS_ENABLE */

                if (tmp > 2500) {
                    status = CY_HBDMA_MGR_TIMEOUT;
                    break;
                }
            }
        } while (
                ((sckStat.status & LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk) != 0) &&
                ((sckStat.intrStatus & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk) == 0)
                );

        /* Disable the socket and reset channel state. */
        Cy_HBDma_SocketDisable(pHandle->pContext->pDrvContext, pHandle->prodSckId[sckIdx]);

        if (status == CY_HBDMA_MGR_SUCCESS) {
            intState = Cy_SysLib_EnterCriticalSection();
            if (actualSize_p != NULL) {
                __DMB();
                __DSB();
                tmp = CY_HBDMA_GET_DESC_SIZE(pHandle->overrideDscrIndex);
                *actualSize_p = CY_HBDMA_DSCR_GET_BYTECNT(pHandle->pContext->en_64k, tmp);
            }
            Cy_SysLib_ExitCriticalSection(intState);
        }
    }

    pHandle->state = CY_HBDMA_CHN_CONFIGURED;
    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetBufferInfo
 ****************************************************************************//**
 *
 * Retrieve the set of DMA buffers associated with a HBDma channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param pBufPtrs
 * Return array to be filled with the DMA buffer pointers.
 *
 * \param bufferCnt
 * Number of buffer pointers to be fetched.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the buffer query is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_GetBufferInfo (
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t **pBufPtrs,
        uint8_t bufferCnt)
{
    cy_stc_hbdma_desc_t dscrInfo;
    uint16_t dscrIndex;
    uint8_t i;

    if ((pHandle == NULL) || (pHandle->pContext == NULL) || (pBufPtrs == NULL) || (bufferCnt == 0))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /*
     * Walk through the descriptor chain and retrieve the buffer pointers from each descriptor.
     * If all the descriptors for the channel have been completed, store NULL for the rest of the indices.
     */
    dscrIndex = pHandle->firstProdDscrIndex[0];
    if ((pHandle->prodSckCount > 1) || (pHandle->consSckCount > 1))
    {
        for (i = 0; i < bufferCnt; i++)
        {
            if (i < (pHandle->count << 1U))
            {
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
                pBufPtrs[i] = dscrInfo.pBuffer;
                if (pHandle->prodSckCount > 1)
                {
                    /* In case of a 2:1 channel, the RD_NEXT pointers should be followed to get the linear chain. */
                    dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
                }
                else
                {
                    /* In case of a 1:2 channel, the WR_NEXT pointers should be followed to get the linear chain. */
                    dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
                }
            }
            else
            {
                pBufPtrs[i] = NULL;
            }
        }
    }
    else
    {
        for (i = 0; i < bufferCnt; i++)
        {
            if (i < pHandle->count)
            {
                Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
                pBufPtrs[i] = dscrInfo.pBuffer;
                dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscrInfo.chain);
            }
            else
            {
                pBufPtrs[i] = NULL;
            }
        }
    }

    return CY_HBDMA_MGR_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_AutoDropData
 ****************************************************************************//**
 *
 * This function allows an active Auto DMA channel to be configured to drop
 * all data that is being received on the producer side. The channel has to be
 * reset and then enabled to restore normal operation.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel has been set up to drop the incoming data.
 * CY_HBDMA_MGR_BAD_PARAM if the channel is not valid or not an AUTO channel.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not active.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_AutoDropData (
        cy_stc_hbdma_channel_t *pHandle)
{
    cy_stc_hbdma_context_t *pDrvCtxt;
    cy_stc_hbdma_sock_t     sckStat;
    cy_stc_hbdma_desc_t     dscrInfo;
    uint16_t                dscrIndex;
    uint32_t                mask;

    /* Operation is only valid on IP to IP channels with event enabled.
     * Operation is not supported on 1:2 DMA channels.
     */
    if ((pHandle == NULL) || (pHandle->type != CY_HBDMA_TYPE_IP_TO_IP) ||
            (pHandle->eventEnable == 0) || (pHandle->consSckCount > 1))
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    if (pHandle->state != CY_HBDMA_CHN_ACTIVE)
    {
        return CY_HBDMA_MGR_SEQUENCE_ERROR;
    }

    pDrvCtxt = pHandle->pContext->pDrvContext;

    /* Change channel state to indicate incoming data should be dropped. */
    pHandle->state = CY_HBDMA_CHN_DATADROP;

    /* Enable STALL interrupts on all producer sockets. */
    Cy_HBDma_UpdateSockIntrMask(pDrvCtxt,
            pHandle->prodSckId[0],
            LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_STALL_Msk,
            true);

    if (pHandle->prodSckCount > 1)
    {
        Cy_HBDma_UpdateSockIntrMask(pDrvCtxt,
                pHandle->prodSckId[0],
                LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_STALL_Msk,
                true);
    }

    mask = Cy_SysLib_EnterCriticalSection();

    /* If the consumer socket is in the ACTIVE state, keep freeing up the descriptors
     * and sending consume events to the respective producer sockets.
     */
    Cy_HBDma_GetSocketStatus(pDrvCtxt, pHandle->consSckId[0], &sckStat);
    Cy_HBDma_SocketDisable(pDrvCtxt, pHandle->consSckId[0]);

    if (CY_HBDMA_STATUS_TO_SOCK_STATE(sckStat.status) == CY_HBDMA_SOCK_STATE_ACTIVE)
    {
        dscrIndex = (uint16_t)(sckStat.actDscrIndex & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_DSCR_NUMBER_Msk);

        do {
            Cy_HBDma_GetDescriptor(dscrIndex, &dscrInfo);
            if ((dscrInfo.size & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_OCCUPIED_Msk) == 0)
            {
                /* Descriptor is not marked occupied. Exit the loop. */
                break;
            }

            dscrInfo.size &= LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk;
            Cy_HBDma_SetDescriptor(dscrIndex, &dscrInfo);
            Cy_HBDma_SendSocketEvent(pDrvCtxt, CY_HBDMA_SYNC_TO_PROD_SOCK(dscrInfo.sync), false);

            dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(dscrInfo.chain);
        } while (1);
    }

    Cy_SysLib_ExitCriticalSection(mask);

    return CY_HBDMA_MGR_SUCCESS;
}


/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetChannelState
 ****************************************************************************//**
 *
 * Retrieve the state of DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * channel state.
 *******************************************************************************/
cy_en_hbdma_chn_state_t
Cy_HBDma_Channel_GetChannelState (cy_stc_hbdma_channel_t *pHandle)
{
    if (pHandle != NULL) {
        return (pHandle->state);
    } else {
        return CY_HBDMA_CHN_NOT_CONFIGURED;
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_SetUsbEgressAdapterDelay
 ****************************************************************************//**
 *
 * Update the number of cycles of delay to be applied between consecutive AXI
 * data fetches made by the USB egress DMA adapter. The function is meant to
 * be used by the USB stack based on current USB speed.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param gblDelayCycles
 * Number of delay cycles to be applied in the range of 0 to 15.
 *
 *******************************************************************************/
void Cy_HBDma_Mgr_SetUsbEgressAdapterDelay (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint8_t gblDelayCycles)
{
    if ((pDmaMgr != NULL) && (pDmaMgr->pDrvContext != NULL)) {
        Cy_HBDma_SetUsbEgressAdapterDelay(pDmaMgr->pDrvContext, gblDelayCycles);
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_SetLvdsAdapterIngressMode
 ****************************************************************************//**
 *
 * This function enables or disables the support for egress data transfers from
 * RAM buffers which are not 16-byte aligned based on whether the specified
 * LVDS DMA adapters are working in Ingress only mode or not. For any ingress-only
 * adapter, this support can be disabled to gain better DMA performance.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param isAdap0Ingress
 * Whether adapter 0 (sockets 0 to 15) is being used only in ingress direction.
 *
 * \param isAdap1Ingress
 * Whether adapter 1 (sockets 16 to 31) is being used only in ingress direction.
 *
 *******************************************************************************/
void Cy_HBDma_Mgr_SetLvdsAdapterIngressMode (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        bool isAdap0Ingress,
        bool isAdap1Ingress)
{
    if ((pDmaMgr != NULL) && (pDmaMgr->pDrvContext != NULL)) {
        Cy_HBDma_SetLvdsAdapterIngressMode(pDmaMgr->pDrvContext, isAdap0Ingress, isAdap1Ingress);
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleDW0Interrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles transfer completion interrupt from any of the
 * DataWire channels associated with non EP0 USB-HS OUT endpoints (channels 1 to 15).
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleDW0Interrupt (
        cy_stc_hbdma_mgr_context_t *pDmaMgr)
{
    uint32_t epNum;
    cy_hbdma_socket_id_t sockId;

    if (pDmaMgr != NULL) {
        for (epNum = 1; epNum < 16; epNum++) {
            if (Cy_DMA_Channel_GetInterruptStatusMasked(DW0, epNum) != 0) {
                Cy_DMA_Channel_ClearInterrupt(DW0, epNum);
                sockId = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_OUT_EP_00 + epNum);
                Cy_HBDma_Channel_Cb(sockId, CY_HBDMA_DATAWIRE0_INTERRUPT, 0, (void *)pDmaMgr);
            }
        }
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleDW1Interrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles transfer completion interrupt from any of the
 * DataWire channels associated with non EP0 USB-HS IN endpoints (channels 1 to 15).
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleDW1Interrupt (
        cy_stc_hbdma_mgr_context_t *pDmaMgr)
{
    uint32_t epNum;
    cy_hbdma_socket_id_t sockId;

    if (pDmaMgr != NULL) {
        for (epNum = 1; epNum < 16; epNum++) {
            if (Cy_DMA_Channel_GetInterruptStatusMasked(DW1, epNum) != 0) {
                Cy_DMA_Channel_ClearInterrupt(DW1, epNum);
                sockId = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + epNum);
                Cy_HBDma_Channel_Cb(sockId, CY_HBDMA_DATAWIRE1_INTERRUPT, 0, (void *)pDmaMgr);
            }
        }
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleUsbShortInterrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles SLP or ZLP interrupts from USB-HS OUT
 * endpoints. This is expected to be triggered from interrupt callback provided
 * by the USB stack.
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 * \param epNum
 * Endpoint number on which SLP/ZLP was received.
 * \param pktSize
 * Actual size (in bytes) of the packet received.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleUsbShortInterrupt (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint8_t                     epNum,
        uint16_t                    pktSize)
{
    cy_stc_hbdma_channel_t *pHandle;
    cy_hbdma_socket_id_t sockId;

    if ((pDmaMgr != NULL) && (epNum < 16)) {
        /* Look up the channel structure corresponding to this endpoint. */
        sockId = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_OUT_EP_00 + epNum);
        pHandle = Cy_HBDma_GetSocketChannel(pDmaMgr, sockId);

        if (pHandle != NULL) {
            /* Get the DMA operation to complete. */
            if (pHandle->prodSckId[0] == sockId) {
                Cy_HBDma_DW_CompleteShortRead(pHandle, 0, pktSize);
            }

            if ((pHandle->prodSckCount == 2) && (pHandle->prodSckId[1] == sockId)) {
                Cy_HBDma_DW_CompleteShortRead(pHandle, 1, pktSize);
            }

            if ((pktSize == 0) && (pDmaMgr->pUsbStackCtx != NULL)) {
                Cy_USBD_ClearZlpSlpIntrEnableMask((cy_stc_usb_usbd_ctxt_t *)pDmaMgr->pUsbStackCtx,
                        epNum, CY_USB_ENDP_DIR_OUT, true);
            }
        } else {
            DBG_HBDMA_ERR("Channel handle for HS EP %d-IN not found\r\n", epNum);
        }
    }
}

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS) */

/* [] END OF FILE */
