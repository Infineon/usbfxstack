/***************************************************************************//**
* \file cy_usb_cdc.h
* \version 1.0
*
* Provided API declarations for the CDC Debug Interface Device Class
* implementation using the USB IP block.
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

#include <stdint.h>
#include <stdbool.h>
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define CY_CDC_BULK_EP_SIZE_FS          (64u)           /**< Maximum packet size of CDC bulk endpoint in FS mode */
#define CY_CDC_BULK_EP_SIZE_HS          (512u)          /**< Maximum packet size of CDC bulk endpoint in HS mode */
#define CY_CDC_BULK_EP_SIZE_SS          (1024u)         /**< Maximum packet size of CDC bulk endpoint in SS mode */

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
    cy_stc_hbdma_channel_t   endpInChannel;             /**< DMA channel for CDC Bulk-IN endpoint. */
    cy_stc_hbdma_channel_t   endpOutChannel;            /**< DMA channel for CDC Bulk-OUT endpoint. */
    void                     *pUsbdCtxt;                /**< USBD stack context pointer. */
    void                     *cdcSendHandle;            /**< Pointer to DMA channel used for data logging. */
    void                     *cdcRecvHandle;            /**< Pointer to DMA channel used to receive CDC data. */
    uint8_t                  ssCtrlIntf;                /**< Control interface index for USB-SS */
    uint8_t                  hsCtrlIntf;                /**< Control interface index for USB-HS */
    uint8_t                  fsCtrlIntf;                /**< Control interface index for USB-FS */
    uint8_t                  *pCfgBuffer;               /**< Buffer used to handle CDC control requests. 64 byte buffer
                                                             is allocated internally when the CDC interface is enabled
                                                             and freed when it is disabled. */
} cy_stc_usb_cdc_ctxt_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

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
bool Cy_USB_CdcValidateParameters(cy_stc_debug_context_t *pDbgCtxt);

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
bool Cy_USB_CdcInit(cy_stc_debug_context_t *pDbgCtxt);

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
bool Cy_USB_CdcAddConfigDescriptors(cy_stc_debug_config_t *pDbgCfg);

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
bool Cy_USB_CdcHandleControlRequests(
        cy_stc_usb_cdc_ctxt_t  *pCdcCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_USB_CDC_H */

/** \} group_usbfxstack_fx_utils */

/*[]*/

