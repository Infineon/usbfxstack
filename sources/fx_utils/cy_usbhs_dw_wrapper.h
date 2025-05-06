/***************************************************************************//**
* \file cy_usbhs_dw_wrapper.h
* \version 1.0
*
* Defines the wrapper functions that enable the use of DataWire DMA channels
* to read/write data from/to the USBHS endpoint memories.
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

/**
 * \addtogroup group_usbfxstack_fx_utils
 * \{
 */

/**
 * \addtogroup group_usbfxstack_fx_utils_macros
 * \{
 */

#ifndef _CY_USBHS_DW_WRAPPER_H_

/** Indicates the use of cy_usb_hs_wrapper */
#define _CY_USBHS_DW_WRAPPER_H_

#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** Number of DMA descriptors required per DataWire channel:
 *  1. First one is a 2-D descriptor which is used to move full data packets.
 *  2. Second one is a 1-D descriptor which moves the DWORD aligned portion of any short packet which
 *     completes the transfer.
 *  3. Third one is a 1-D descriptor used to transfer the remaining non-DWORD aligned data (1, 2 or 3 bytes)
 *     where required.
 */
#define DSCRS_PER_CHN                   (3u)

/** Retrieve the DataWire DMA descriptor from the USBHS DMA wrapper data structure. */
#define Cy_UsbHS_App_GetDmaDesc(dmaset,n)       (cy_stc_dma_descriptor_t *)(&((dmaset)->dmaXferDscr[n]))

/** Retrieve the DMAC descriptor from the USBHS DMA wrapper data structure. */
#define Cy_UsbHS_App_GetDmaCDesc(dmaset,n)      (cy_stc_dmac_descriptor_t *)(&((dmaset)->dmaXferDscr[n]))

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

/**
 * This structure represents a DMAC or DataWire transfer descriptor.
 */
typedef struct
{
    uint32_t dscrArray[8];              /**< Array of 8 DWORDs that covers the largest DMAC descriptor. */
} cy_stc_usbhs_dma_desc_t;

/** 
 * This structure collects all the information associated with the DMA configuration for a
 * USB OUT/IN endpoint.
 */
typedef struct cy_stc_app_endp_dma_set_
{
    void                    *pDwStruct;                         /**< DataWire register structure pointer. */
    cy_stc_usbhs_dma_desc_t  dmaXferDscr[DSCRS_PER_CHN];        /**< DMA descriptors used to transfer data. */
    cy_stc_hbdma_channel_t   hbDmaChannel;                      /**< High bandwidth DMA channel structure. */
    uint16_t                 maxPktSize;                        /**< Maximum packet size for the endpoint. */
    uint8_t                  channel;                           /**< DataWire channel number. */
    uint8_t                  epNumber;                          /**< Endpoint number. */
    cy_en_usb_endp_dir_t     epDir;                             /**< Endpoint direction. */
    bool                     valid;                             /**< Whether endpoint is valid (enabled). */
    bool                     firstRqtDone;                      /**< Whether first transfer on the ep is complete. */
    cy_en_usb_endp_type_t    endpType;                          /**< Endpoint Type. */

    uint8_t                 *pCurDataBuffer;                    /** RAM buffer used for current transfer. */
    uint16_t                 curDataSize;                       /** Current DMA transfer size. */
} cy_stc_app_endp_dma_set_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/******************* Function Prototypes *********************/

/*******************************************************************************
 * Function name: Cy_USBHS_App_EnableEpDmaSet
 ****************************************************************************//**
 *
 * Configure the DMA resources associated with a USBHS endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 * \param pDwStruct
 * Pointer to DataWire instance to be used for this endpoint.
 *
 * \param channelNum
 * DataWire channel number to be used.
 *
 * \param epNumber
 * USB endpoint number.
 *
 * \param epDir
 * USB endpoint direction.
 *
 * \param maxPktSize
 * Maximum packet size for the endpoint.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool
Cy_USBHS_App_EnableEpDmaSet(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        DW_Type                   *pDwStruct,
        uint8_t                    channelNum,
        uint8_t                    epNumber,
        cy_en_usb_endp_dir_t       epDir,
        uint16_t                   maxPktSize);


/*******************************************************************************
 * Function name: Cy_USBHS_App_DisableEpDmaSet
 ****************************************************************************//**
 *
 * De-init DMA resources related to an USBHS endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 *******************************************************************************/
void
Cy_USBHS_App_DisableEpDmaSet(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

/*******************************************************************************
 * Function name: Cy_USBHS_App_ResetEpDma
 ****************************************************************************//**
 *
 * Reset the DMA resources corresponding to an endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 *******************************************************************************/
void
Cy_USBHS_App_ResetEpDma(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

/*******************************************************************************
 * Function name: Cy_USBHS_App_QueueRead
 ****************************************************************************//**
 *
 * Function to queue read operation on an OUT endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 * \param pBuffer
 * Pointer to buffer into which the data should be read.
 *
 * \param dataSize
 * Amount of data (in bytes) to be read.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool
Cy_USBHS_App_QueueRead(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize);

/*******************************************************************************
 * Function name: Cy_USBHS_App_ReadShortPacket
 ****************************************************************************//**
 *
 * Function to modify an ongoing DMA read operation to take care of a short packet.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 * \param pktSize
 * Size (in bytes) of the short packet which has been received.
 *
 * \return
 * Total amount of data received on the endpoint including any previous full
 * packets.
 *******************************************************************************/
uint16_t
Cy_USBHS_App_ReadShortPacket(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint16_t                   pktSize);

/*******************************************************************************
 * Function name: Cy_USBHS_App_QueueWrite
 ****************************************************************************//**
 *
 * Function to queue write operation on an IN endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 * \param pBuffer
 * Pointer to buffer containing write data.
 *
 * \param dataSize
 * Amount of data to be transferred (in bytes).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool
Cy_USBHS_App_QueueWrite(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize);

/*******************************************************************************
 * Function name: Cy_USBHS_App_ClearDmaInterrupt
 ****************************************************************************//**
 *
 * Disable any pending DataWire channel interrupts for an endpoint.
 *
 * \param pEpDmaSet
 * USBHS endpoint DMA wrapper structure
 *
 *******************************************************************************/
void
Cy_USBHS_App_ClearDmaInterrupt(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USBHS_DW_WRAPPER_H_ */

/** \} group_usbfxstack_fx_utils */

/*[EOF]*/

