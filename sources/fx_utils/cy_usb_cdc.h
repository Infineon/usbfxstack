/***************************************************************************//**
* \file cy_usb_cdc.h
* \version 1.0
*
* Provided API declarations for the CDC Debug Interface Device Class
* implementation using the USB IP block.
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

/**
 * \addtogroup group_usbfxstack_fx_utils
 * \{
 */

/**
 * \addtogroup group_usbfxstack_fx_utils_macros
 * \{
 */


#ifndef CY_USB_CDC_H

/** Indicates the use of cy_usb_cdc */
#define CY_USB_CDC_H

#include "stdint.h"
#include "stdbool.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

/**
 * @brief Structure containing the information for USB CDC interface
 */
typedef struct cy_stc_usb_cdc_ctxt_
{
    uint8_t                  cdcEpIn;                   /**< IN endpoint used for CDC interface. */
    uint8_t                  cdcEpOut;                  /**< OUT endpoint used for CDC interface. */
    uint8_t                  bufCount;                  /**< Number of RAM buffers to be used for logging. */
    uint8_t                  recvEnabled;               /**< Whether data receive through OUT endpoint is enabled. */
    uint8_t                  rxFreeBufferCount;         /**< Count of free data receive buffers available. */
    DW_Type                  *pCpuDw0Base;              /**< Pointer to DataWire-0 IP registers. */
    DW_Type                  *pCpuDw1Base;              /**< Pointer to DataWire-1 IP registers. */
    cy_stc_app_endp_dma_set_t endpInDma;                /**< DMA status information for IN endpoints. */
    cy_stc_app_endp_dma_set_t endpOutDma;               /**< DMA status information for OUT endpoints. */
    void                      *pUsbdCtxt;               /**< USBD stack context pointer. */
    void                     *cdcSendHandle;            /**< Pointer to DMA channel used for data logging. */
    void                     *cdcRecvHandle;            /**< Pointer to DMA channel used to receive CDC data. */
} cy_stc_usb_cdc_ctxt_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/*******************************************************************************
 * Function name: Cy_USB_CdcInit
 ****************************************************************************//**
 *
 * Function to initialize CDC interface for debug logging.
 *
 * \param pDbgCtxt
 * Debug Context
 *
 * \return
 * True if initialization is successful, else False
 *
 *******************************************************************************/
bool Cy_USB_CdcInit(cy_stc_debug_context_t *pDbgCtxt);

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
 * True if deinitialization is successful, else False
 *
 *******************************************************************************/
bool Cy_USB_CdcDeInit(cy_stc_debug_context_t *pDbgCtxt);

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
uint16_t Cy_Debug_LogtoUsb(cy_stc_debug_context_t *pDbgCtxt, uint16_t rdPtr, uint16_t limit);

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
 * Endpoint Number
 *
 *******************************************************************************/
void Cy_USB_CdcClearDmaInterrupt(cy_stc_debug_context_t* pDbgCtxt, uint8_t endpDirection, uint32_t endpNumber);


/*******************************************************************************
 * Function name: Cy_USB_CdcDmaCallback
 ****************************************************************************//**
 *
 * DMA channel Callback used by the USB CDC logging interface.
 *
 * \param pHandle
 * CDC Channel Handle.
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
void Cy_USB_CdcDmaCallback(struct cy_stc_hbdma_channel *pHandle,
                           cy_en_hbdma_cb_type_t type,
                           cy_stc_hbdma_buff_status_t* pbufStat, void *userCtx);

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
 *******************************************************************************/
void Cy_USB_CdcQueueRead(cy_stc_debug_context_t *pDbgCtxt, uint8_t endpNumber,
                         uint8_t *pBuffer, uint16_t dataSize);

/*******************************************************************************
 * Function name: Cy_USB_CdcSlpCallback
 ****************************************************************************//**
 *
 * The Function will be called by USBD layer when SLP message comes on the
 * CDC OUT endpoint.
 *
 * \param pAppCtxt
 * User context
 *
 * \param pUsbdCtxt
 * USBD context
 *
 * \param pMsg
 * Message
 *
 *******************************************************************************/
void Cy_USB_CdcSlpCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
 * Function name: Cy_USB_CdcChannelEnable
 ****************************************************************************//**
 *
 * The function enables the CDC Send/Receive DMA channels
 *
 * \param pDbgCtxt
 * Debug context
 *
 * \return
 * true if channel was enabled successfully, false otherwise.
 *******************************************************************************/
bool Cy_USB_CdcChannelEnable(cy_stc_debug_context_t *pDbgCtxt);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_USB_CDC_H */

/** \} group_usbfxstack_fx_utils */

/*[]*/

