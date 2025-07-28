/***************************************************************************//**
* \file cy_usb_usbd.h
* \version 1.0
*
* Defines the programming interfaces of the USB Device Stack.
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
 * \addtogroup group_usbfxstack_usb_common
 * \{
 */

/**
 * \addtogroup group_usbfxstack_usb_common_macros
 * \{
 */

#if !defined(CY_USB_USBD_H)

/** Indicates the use of cy_usb_usbd */
#define CY_USB_USBD_H

#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#endif /* FREERTOS_ENABLE */

#if defined(__cplusplus)
extern "C" {
#endif

/** Offset of descriptor length field in any USB descriptor. */
#define CY_USB_DSCR_OFFSET_LEN               (0x00)
/** Offset of type field in any USB descriptor. */
#define CY_USB_DSCR_OFFSET_TYPE              (0x01)

/** Length of standard device descriptor. */
#define CY_USB_DEVICE_DSCR_LEN              (18)
/** Length of device qualifier descriptor. */
#define CY_USB_DEVICE_QUAL_DSCR_LEN         (10)
/** Length of SS endpoint companion descriptor. */
#define CY_USB_ENDP_SS_COMP_DSCR_LEN        (6)
/** Length of SSP isochronous endpoint companion descriptor. */
#define CY_USB_ENDP_SSP_ISO_COMP_DSCR_LEN   (8)

/** Mask to get bmRequestType field from control request data. */
#define CY_USB_BMREQUEST_SETUP0_MASK      (0x000000FFu)
/** Bit position of bmRequestType field in control request data. */
#define CY_USB_BMREQUEST_SETUP0_POS       (0u)
/** Mask to get bRequest field from control request data. */
#define CY_USB_BREQUEST_SETUP0_MASK       (0x0000FF00u)
/** Bit position of bRequest field in control request data. */
#define CY_USB_BREQUEST_SETUP0_POS        (8u)
/** Mask to get wValue field from control request data. */
#define CY_USB_WVALUE_SETUP0_MASK         (0xFFFF0000u)
/** Bit position of wValue field in control request data. */
#define CY_USB_WVALUE_SETUP0_POS          (16u)
/** Mask to get wIndex field from control request data. */
#define CY_USB_WINDEX_SETUP1_MASK         (0x0000FFFFu)
/** Bit position of wIndex field in control request data. */
#define CY_USB_WINDEX_SETUP1_POS          (0u)
/** Mask to get wLength field from control request data. */
#define CY_USB_WLENGTH_SETUP1_MASK         (0xFFFF0000u)
/** Bit position of wLength field in control request data. */
#define CY_USB_WLENGTH_SETUP1_POS          (16u)

#ifndef DOXYGEN

#define CY_USB_CTRL_REQ_TYPE_MASK              (0x60)
#define CY_USB_CTRL_REQ_TYPE_POS               (5u)
#define CY_USB_CTRL_REQ_STD                    (0x00u)
#define CY_USB_CTRL_REQ_CLASS                  (0x01u)
#define CY_USB_CTRL_REQ_VENDOR                 (0x02u)
#define CY_USB_CTRL_REQ_RESERVED               (0x03u)

/* Defines to decode recipient from bmRequest type. */
#define CY_USB_CTRL_REQ_RECIPENT_MASK          (0x1Fu)
#define CY_USB_CTRL_REQ_RECIPENT_POS           (0u)
#define CY_USB_CTRL_REQ_RECIPENT_DEVICE        (0u)
#define CY_USB_CTRL_REQ_RECIPENT_INTF          (1u)
#define CY_USB_CTRL_REQ_RECIPENT_ENDP          (2u)
#define CY_USB_CTRL_REQ_RECIPENT_OTHERS        (3u)
#define CY_USB_CTRL_REQ_RECIPENT_RSVD          (4u)

/* Extracting descriptor type and index from wValue. */
#define CY_USB_CTRL_REQ_DSCR_TYPE_MASK          (0xFF00u)
#define CY_USB_CTRL_REQ_DSCR_TYPE_POS           (8u)
#define CY_USB_CTRL_REQ_DSCR_INDEX_MASK         (0x00FFu)
#define CY_USB_CTRL_REQ_DSCR_INDEX_POS          (0u)


/* Defines for GET STATUS command  */
#define CY_USB_GET_STATUS_DEV_SELF_POWER_POS          (0u)
#define CY_USB_GET_STATUS_DEV_SELF_POWER              (0x01u)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP_MASK      (0x0002)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP_POS       (1u)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP           (0x02u)
#define CY_USB_GET_STATUS_DEV_U1_ENABLE_POS           (2u)
#define CY_USB_GET_STATUS_DEV_U1_ENABLE               (0x04u)
#define CY_USB_GET_STATUS_DEV_U2_ENABLE_POS           (3u)
#define CY_USB_GET_STATUS_DEV_U2_ENABLE               (0x08u)

#define CY_USB_GET_STATUS_ENDP_HALT_POS               (0u)
#define CY_USB_GET_STATUS_ENDP_HALT                   (1u)

/* various offset in device descriptor */
#define CY_USB_DEVICE_DSCR_OFFSET_MAX_PKT_SIZE      (0x07)
#define CY_USB_DEVICE_DSCR_OFFSET_NUM_CONFIG        (0x08)

/* various offset in config descriptor */
#define CY_USB_CFG_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_CFG_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN            (0x02)
#define CY_USB_CFG_DSCR_OFFSET_NUM_INTF             (0x04)
#define CY_USB_CFG_DSCR_OFFSET_CFG_VALUE            (0x05)
#define CY_USB_CFG_DSCR_OFFSET_ATTRIBUTE            (0x07)
#define CY_USB_CFG_DSCR_OFFSET_MAX_POWER            (0x08)

#define CY_USB_CFG_DSCR_SELF_POWER_MASK             ((uint8_t)0x40)
#define CY_USB_CFG_DSCR_REMOTE_WAKEUP_MASK          ((uint8_t)0x20)

/* various offset in interface descriptor  */
#define CY_USB_INTF_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_INTF_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_INTF_DSCR_OFFSET_NUMBER               (0x02)
#define CY_USB_INTF_DSCR_OFFSET_ALT_SETTING          (0x03)
#define CY_USB_INTF_DSCR_OFFSET_NUM_ENDP             (0x04)

/* various offset in endpoint descriptor  */
#define CY_USB_ENDP_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_ENDP_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_ENDP_DSCR_OFFSET_ADDRESS              (0x02)
#define CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE            (0x03)
#define CY_USB_ENDP_DSCR_OFFSET_MAX_PKT              (0x04)
#define CY_USB_ENDP_DSCR_OFFSET_INTERVAL             (0x06)
#define CY_USB_ENDP_DSCR_OFFSET_BYTES_PER_INTVL      (0x04)

/* various offset in SS endpoint Companion descriptor  */
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_LEN                (0x00)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_TYPE               (0x01)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMAX_BURST         (0x02)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMATTRIBUTE        (0x03)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL (0x04)

/* various offset in SS PLUS ISO endpoint Companion descriptor  */
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_LEN                (0x00)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_TYPE               (0x01)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_RES                (0x02)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL (0x04)

/* various offset in BOS descriptor  */
#define CY_USB_BOS_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_BOS_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_BOS_DSCR_OFFSET_TOTAL_LEN            (0x02)
#define CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP           (0x04)

/* various offset in BOS CAPABILITY descriptor  */
#define CY_USB_CAP_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_OFFSET_CAP_TYPE             (0x02)

/* various Device capability code  */
#define CY_USB_DEVICE_CAP_TYPE_WUSB                 (0x01u)
#define CY_USB_DEVICE_CAP_TYPE_USB2_EXT             (0x02u)
#define CY_USB_DEVICE_CAP_TYPE_SS_USB               (0x03u)
#define CY_USB_DEVICE_CAP_TYPE_CONTAINTER_ID        (0x04u)
#define CY_USB_DEVICE_CAP_TYPE_SS_PLUS              (0x0Au)
#define CY_USB_DEVICE_CAP_TYPE_PTM                  (0x0Bu)

/* various offset in USB2.0 EXT descriptor  */
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE            (0x03)


/* Various offset in SS capability descriptor */
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_ATTRIBUTE            (0x03)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_SPEED_SUPPORT        (0x04)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_FUNC_SUPPORT         (0x06)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_U1_EXIT_LAT          (0x07)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_U2_EXIT_LAT          (0x08)

/* Container ID OFFSET */
#define CY_USB_CAP_DSCR_CID_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_CID_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_CID_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_CID_OFFSET_RESERVE              (0x03)
#define CY_USB_CAP_DSCR_CID_OFFSET_ID                   (0x04)

/* various offset in SS PLUS Device capability descriptor */
#define CY_USB_CAP_DSCR_SSP_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_SSP_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_SSP_OFFSET_CAP_TYPE             (0x02)

/* wMaxPacketSize mask. */
#define CY_USB_ENDP_MAX_PKT_SIZE_MASK   (0x07FFU)

/* Additional transactions per microframe mask. */
#define CY_USB_ENDP_ADDL_XN_MASK        (0x18U)

/* Additional transactions per microframe field bit position. */
#define CY_USB_ENDP_ADDL_XN_POS         (3U)

#endif /* DOXYGEN */

/** Depth of message queue used by USBD stack to get messages from USBHS/USBSS CAL drivers. */
#define CY_USB_USBD_MSG_QUEUE_SIZE              (32)

/** Size of each message in the USBD message queue. */
#define CY_USB_USBD_MSG_SIZE                    (sizeof(cy_stc_usb_cal_msg_t))

/** Maximum number of interfaces supported in the configuration. */
#define CY_USB_MAX_INTF                         (17)

/** Stack size reserved for USBD task in 32-bit words. Budgeting extra space to cover for callbacks. */
#define CY_USBD_TASK_STACK_DEPTH                (512)

/** USBD task priority level. */
#define CY_USBD_TASK_PRIORITY                   (10)

#ifndef DOXYGEN
#if (!EP0_DATAWIRE_EN)

/* Map USBD DMA types and function names to DMAC types and functions. */
typedef cy_stc_dmac_descriptor_t cy_stc_usbd_dma_descr_t;
typedef cy_stc_dmac_descriptor_config_t cy_stc_usbd_dma_descr_conf_t;
typedef cy_stc_dmac_channel_config_t cy_stc_usbd_dma_chn_conf_t;

#define CY_USBD_DMA_CHN_DISABLED CY_DMAC_CHANNEL_DISABLED
#define CY_USBD_DMA_CHN_ENABLED CY_DMAC_CHANNEL_ENABLED
#define CY_USBD_DMA_RETRIG_IM CY_DMAC_RETRIG_IM
#define CY_USBD_DMA_WAIT_FOR_REACT CY_DMAC_WAIT_FOR_REACT
#define CY_USBD_DMA_DESCR_CHAIN CY_DMAC_DESCR_CHAIN
#define CY_USBD_DMA_X_LOOP CY_DMAC_X_LOOP
#define CY_USBD_DMA_BYTE CY_DMAC_BYTE
#define CY_USBD_DMA_WORD CY_DMAC_WORD
#define CY_USBD_DMA_XFER_SIZE_DATA CY_DMAC_TRANSFER_SIZE_DATA
#define CY_USBD_DMA_1D_XFER CY_DMAC_1D_TRANSFER
#define CY_USBD_DMA_2D_XFER CY_DMAC_2D_TRANSFER

#define Cy_USBD_DMADesc_SetSrcAddress Cy_DMAC_Descriptor_SetSrcAddress
#define Cy_USBD_DMADesc_SetDstAddress Cy_DMAC_Descriptor_SetDstAddress
#define Cy_USBD_DMADesc_SetXloopDataCount Cy_DMAC_Descriptor_SetXloopDataCount
#define Cy_USBD_DMADesc_SetYloopDataCount Cy_DMAC_Descriptor_SetYloopDataCount
#define Cy_USBD_DMADesc_SetYloopSrcIncrement Cy_DMAC_Descriptor_SetYloopSrcIncrement
#define Cy_USBD_DMADesc_SetYloopDstIncrement Cy_DMAC_Descriptor_SetYloopDstIncrement
#define Cy_USBD_DMADesc_SetNextDescriptor Cy_DMAC_Descriptor_SetNextDescriptor
#define Cy_USBD_DMADesc_SetChannelState Cy_DMAC_Descriptor_SetChannelState
#define Cy_USBD_DMADesc_Init Cy_DMAC_Descriptor_Init
#define Cy_USBD_DMAChn_Init Cy_DMAC_Channel_Init
#define Cy_USBD_DMAChn_SetDesc Cy_DMAC_Channel_SetDescriptor
#define Cy_USBD_DMAChn_Enable Cy_DMAC_Channel_Enable
#define Cy_USBD_DMAChn_Disable Cy_DMAC_Channel_Disable
#define Cy_USBD_DMAChn_IsEnabled Cy_DMAC_Channel_IsEnabled
#define Cy_USBD_DMAChn_EnableIntr(dmabase, channel) Cy_DMAC_Channel_SetInterruptMask((dmabase), (channel), CY_DMAC_INTR_MASK)
#define Cy_USBD_DMAChn_DisableIntr(dmabase, channel) Cy_DMAC_Channel_SetInterruptMask((dmabase), (channel), 0)
#define Cy_USBD_DMAChn_GetIntrStatus Cy_DMAC_Channel_GetInterruptStatus
#define Cy_USBD_DMAChn_ClearIntr(dmabase, channel, value) Cy_DMAC_Channel_ClearInterrupt((dmabase), (channel), (value))

#define Cy_USBD_EP0In_DmaBase(pUsbdCtxt)        pUsbdCtxt->pCpuDmacBase
#define Cy_USBD_EP0Out_DmaBase(pUsbdCtxt)       pUsbdCtxt->pCpuDmacBase

#define CY_USBD_EGREP_OUT_TRIG          TRIG_IN_MUX_5_USBHSDEV_TR_OUT16
#define CY_USBD_EGREP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN16
#define CY_USBD_EGREP_DMA_TRIG_BASE     TRIG_OUT_MUX_5_MDMA_TR_IN0
#define CY_USBD_EGREP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_MDMA_TR_OUT0
#define CY_USBD_INGEP_OUT_TRIG          TRIG_IN_MUX_5_USBHSDEV_TR_OUT0
#define CY_USBD_INGEP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN0
#define CY_USBD_INGEP_DMA_TRIG_BASE     TRIG_OUT_MUX_5_MDMA_TR_IN0
#define CY_USBD_INGEP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_MDMA_TR_OUT0

#else

/* Map USBD DMA types and function names to DW types and functions. */
typedef cy_stc_dma_descriptor_t cy_stc_usbd_dma_descr_t;
typedef cy_stc_dma_descriptor_config_t cy_stc_usbd_dma_descr_conf_t;
typedef cy_stc_dma_channel_config_t cy_stc_usbd_dma_chn_conf_t;

#define CY_USBD_DMA_CHN_DISABLED CY_DMA_CHANNEL_DISABLED
#define CY_USBD_DMA_CHN_ENABLED CY_DMA_CHANNEL_ENABLED
#define CY_USBD_DMA_RETRIG_IM CY_DMA_RETRIG_IM
#define CY_USBD_DMA_WAIT_FOR_REACT CY_DMA_WAIT_FOR_REACT
#define CY_USBD_DMA_DESCR_CHAIN CY_DMA_DESCR_CHAIN
#define CY_USBD_DMA_X_LOOP CY_DMA_X_LOOP
#define CY_USBD_DMA_BYTE CY_DMA_BYTE
#define CY_USBD_DMA_WORD CY_DMA_WORD
#define CY_USBD_DMA_XFER_SIZE_DATA CY_DMA_TRANSFER_SIZE_DATA
#define CY_USBD_DMA_1D_XFER CY_DMA_1D_TRANSFER
#define CY_USBD_DMA_2D_XFER CY_DMA_2D_TRANSFER

#define Cy_USBD_DMADesc_SetSrcAddress Cy_DMA_Descriptor_SetSrcAddress
#define Cy_USBD_DMADesc_SetDstAddress Cy_DMA_Descriptor_SetDstAddress
#define Cy_USBD_DMADesc_SetXloopDataCount Cy_DMA_Descriptor_SetXloopDataCount
#define Cy_USBD_DMADesc_SetYloopDataCount Cy_DMA_Descriptor_SetYloopDataCount
#define Cy_USBD_DMADesc_SetYloopSrcIncrement Cy_DMA_Descriptor_SetYloopSrcIncrement
#define Cy_USBD_DMADesc_SetYloopDstIncrement Cy_DMA_Descriptor_SetYloopDstIncrement
#define Cy_USBD_DMADesc_SetNextDescriptor Cy_DMA_Descriptor_SetNextDescriptor
#define Cy_USBD_DMADesc_SetChannelState Cy_DMA_Descriptor_SetChannelState
#define Cy_USBD_DMADesc_Init Cy_DMA_Descriptor_Init
#define Cy_USBD_DMAChn_Init Cy_DMA_Channel_Init
#define Cy_USBD_DMAChn_SetDesc Cy_DMA_Channel_SetDescriptor
#define Cy_USBD_DMAChn_Enable Cy_DMA_Channel_Enable
#define Cy_USBD_DMAChn_Disable Cy_DMA_Channel_Disable
#define Cy_USBD_DMAChn_IsEnabled Cy_DMA_Channel_IsEnabled
#define Cy_USBD_DMAChn_EnableIntr(dmabase, channel) Cy_DMA_Channel_SetInterruptMask((dmabase), (channel), CY_DMA_INTR_MASK)
#define Cy_USBD_DMAChn_DisableIntr(dmabase, channel) Cy_DMA_Channel_SetInterruptMask((dmabase), (channel), 0)
#define Cy_USBD_DMAChn_GetIntrStatus Cy_DMA_Channel_GetInterruptStatus
#define Cy_USBD_DMAChn_ClearIntr(dmabase, channel, value) Cy_DMA_Channel_ClearInterrupt((dmabase), (channel))

#define Cy_USBD_EP0In_DmaBase(pUsbdCtxt)        pUsbdCtxt->pCpuDw1Base
#define Cy_USBD_EP0Out_DmaBase(pUsbdCtxt)       pUsbdCtxt->pCpuDw0Base

#define CY_USBD_EGREP_OUT_TRIG          TRIG_IN_MUX_1_USBHSDEV_TR_OUT16
#define CY_USBD_EGREP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN16
#define CY_USBD_EGREP_DMA_TRIG_BASE     TRIG_OUT_MUX_1_PDMA1_TR_IN0
#define CY_USBD_EGREP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_PDMA1_TR_OUT0
#define CY_USBD_INGEP_OUT_TRIG          TRIG_IN_MUX_0_USBHSDEV_TR_OUT0
#define CY_USBD_INGEP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN0
#define CY_USBD_INGEP_DMA_TRIG_BASE     TRIG_OUT_MUX_0_PDMA0_TR_IN0
#define CY_USBD_INGEP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_PDMA0_TR_OUT0

#endif /* EP0_DATAWIRE_EN */
#endif /* DOXYGEN */

/** \} group_usbfxstack_usb_common_macros */

/**
 * \addtogroup group_usbfxstack_usb_common_enums
 * \{
 */

/**
 * @typedef cy_en_usb_std_dscr_type_t
 * @brief Standard descriptor types defined by USB specification.
 */
typedef enum {
    CY_USB_DSCR_TYPE_DEVICE = 0x01u,                    /**< 0x01: Device descriptor. */
    CY_USB_DSCR_TYPE_CFG,                               /**< 0x02: Configuration descriptor. */
    CY_USB_DSCR_TYPE_STR,                               /**< 0x03: String descriptor. */
    CY_USB_DSCR_TYPE_INTF,                              /**< 0x04: Interface descriptor. */
    CY_USB_DSCR_TYPE_ENDP,                              /**< 0x05: Endpoint descriptor. */
    CY_USB_DSCR_TYPE_DEVICE_QUALIFIER,                  /**< 0x06: Device qualifier descriptor. */

    CY_USB_DSCR_TYPE_INTF_ASSOC = 0x0Bu,                /**< 0x0B: Interface association descriptor. */
    CY_USB_DSCR_TYPE_BOS = 0x0Fu,                       /**< 0x0F: BOS descriptor. */
    CY_USB_DSCR_TYPE_DEVICE_CAP = 0x10u,                /**< 0x10: Device capability descriptor. */
    CY_USB_DSCR_TYPE_SS_ENDP_COMP = 0x30u,              /**< 0x30: SS endpoint companion descriptor. */
    CY_USB_DSCR_TYPE_SSP_ISO_ENDP_COMP = 0x31u          /**< 0x31: SSP isochronous EP companion descriptor. */
} cy_en_usb_std_dscr_type_t;

/**
 * @typedef cy_en_usb_set_dscr_type_t
 * @brief List of descriptor types that can be registered with USB stack.
 */
typedef enum {
    CY_USB_SET_SS_DEVICE_DSCR,                  /**< USB 3.x device descriptor. */
    CY_USB_SET_HS_DEVICE_DSCR,                  /**< USB 2.x device descriptor. */
    CY_USB_SET_DEVICE_QUAL_DSCR,                /**< Device qualifier descriptor. */
    CY_USB_SET_FS_CONFIG_DSCR,                  /**< Full Speed Configuration descriptor. */
    CY_USB_SET_HS_CONFIG_DSCR,                  /**< High Speed Configuration descriptor. */
    CY_USB_SET_SS_CONFIG_DSCR,                  /**< Super Speed Configuration descriptor. */
    CY_USB_SET_STRING_DSCR,                     /**< String descriptor. */
    CY_USB_SET_HS_BOS_DSCR,                     /**< USB 2.x BOS descriptor. */
    CY_USB_SET_SS_BOS_DSCR                      /**< USB 3.x BOS descriptor. */
} cy_en_usb_set_dscr_type_t;

/**
 * @typedef cy_en_usb_usbd_cb_t
 * @brief List of event callbacks made by USBD stack to the application.
 */
typedef enum {
    CY_USB_USBD_CB_RESET,                       /**< USB bus reset event. */
    CY_USB_USBD_CB_RESET_DONE,                  /**< USB reset complete event. */
    CY_USB_USBD_CB_BUS_SPEED,                   /**< Full Speed to High Speed transition event. */
    CY_USB_USBD_CB_SETUP,                       /**< Control request to be handled by application. */
    CY_USB_USBD_CB_SUSPEND,                     /**< USB link suspended. */
    CY_USB_USBD_CB_RESUME,                      /**< USB link resumed from suspend. */
    CY_USB_USBD_CB_SET_CONFIG,                  /**< SET_CONFIGURATION request received. */
    CY_USB_USBD_CB_SET_INTF,                    /**< SET_INTERFACE request received. */
    CY_USB_USBD_CB_STATUS_STAGE_COMP,           /**< Control request acknowledged. */
    CY_USB_USBD_CB_L1_SLEEP,                    /**< USB 2.x L1 state entered. */
    CY_USB_USBD_CB_L1_RESUME,                   /**< USB 2.x L1 state exited. */
    CY_USB_USBD_CB_ZLP,                         /**< ZLP transfer done on endpoint. USB 2.x only. */
    CY_USB_USBD_CB_SLP,                         /**< SLP transfer done on endpoint. USB 2.x only. */
    CY_USB_USBD_CB_DONE,                        /**< Requested transfer completed on endpoint. USB 2.x only. */
    CY_USB_USBD_CB_DISCONNECT,                  /**< USB disconnect detected. USB 3.x only. */
    CY_USB_USBD_CB_SETADDR,                     /**< SET_ADDRESS command completed. */
    CY_USB_USBD_CB_EP0_RCV_DONE,                /**< EP0 out transfer has been completed. */
    CY_USB_USBD_CB_SOF_ITP,                     /**< USB3 ITP or USB2 SOF received. */
    CY_USB_USBD_CB_SET_INVALID                  /**< Invalid callback type. */
} cy_en_usb_usbd_cb_t;

/**
 * @typedef cy_en_usb_device_state_t
 * @brief List of USB device states.
 */
typedef enum {
    CY_USB_DEVICE_STATE_DISABLE = 0x00,         /**< When device is not visible on BUS */
    CY_USB_DEVICE_STATE_ENABLE,                 /**< When device is visible on BUS. */
    CY_USB_DEVICE_STATE_ATTACHED,               /**< When Device visible and VBUS detected. */
    CY_USB_DEVICE_STATE_POWER,                  /**< When VBUS visible. */
    CY_USB_DEVICE_STATE_RESET,                  /**< when Bus reset applied. */
    CY_USB_DEVICE_STATE_DEFAULT,                /**< when RESET complete. */
    CY_USB_DEVICE_STATE_ADDRESS,                /**< When SET_ADDRESS complete. */
    CY_USB_DEVICE_STATE_CONFIGURED,             /**< When device is in configured state. */
    CY_USB_DEVICE_STATE_SUSPEND,                /**< When USB link is suspended (L2/U3) */
    CY_USB_DEVICE_STATE_HS_L1,                  /**< When USBHS link is in L1 state. */
    CY_USB_DEVICE_STATE_INVALID,                /**< Unused state. */
}cy_en_usb_device_state_t;

/**
 * @typedef cy_en_usb_endp0_state_t
 * @brief List of USB control endpoint states.
 */
typedef enum
{
    CY_USB_ENDP0_STATE_IDLE = 0x00,             /**< Control endpoint is idle. */
    CY_USB_ENDP0_STATE_SETUP,                   /**< Control request received. */
    CY_USB_ENDP0_STATE_DATAIN,                  /**< IN transfer pending. */
    CY_USB_ENDP0_STATE_DATAOUT,                 /**< OUT transfer pending. */
    CY_USB_ENDP0_STATE_STATUS,                  /**< Status handshake pending. */
    CY_USB_ENDP0_STATE_STALL                    /**< Control endpoint stalled. */
}cy_en_usb_endp0_state_t;

/**
 * @typedef cy_en_usb_std_req_t
 * @brief Various standard request as per USB specification.
 */
typedef enum
{
    CY_USB_SC_GET_STATUS = 0x00,                /**< Get status */
    CY_USB_SC_CLEAR_FEATURE,                    /**< Clear feature */
    CY_USB_SC_RESERVED,                         /**< Undefined request */
    CY_USB_SC_SET_FEATURE,                      /**< Set feature */
    CY_USB_SC_SET_ADDRESS = 0x05,               /**< Set Address */
    CY_USB_SC_GET_DESCRIPTOR,                   /**< Get Descriptor */
    CY_USB_SC_SET_DESCRIPTOR,                   /**< Set Descriptor */
    CY_USB_SC_GET_CONFIGURATION,                /**< Get Configuration */
    CY_USB_SC_SET_CONFIGURATION,                /**< Set Configuration */
    CY_USB_SC_GET_INTERFACE,                    /**< Get Interface */
    CY_USB_SC_SET_INTERFACE,                    /**< Set Interface */
    CY_USB_SC_SYNC_FRAME,                       /**< Sync Frame */
    CY_USB_SC_SET_SEL = 0x30,       /**< Set system exit latency. */
    CY_USB_SC_SET_ISOC_DELAY        /**< Set isochronous delay. */
} cy_en_usb_std_req_t;

/**
 * @typedef cy_en_usb_dscr_type_t
 * @brief Various standard descriptor as per USB secification.
 */
typedef enum {
    CY_USB_DEVICE_DSCR = 0x01,                  /**< Device descriptor. */
    CY_USB_CONFIG_DSCR,                         /**< Configuration descriptor. */
    CY_USB_STRING_DSCR,                         /**< String descriptor. */
    CY_USB_INTR_DSCR,                           /**< Interface descriptor. */
    CY_USB_ENDP_DSCR,                           /**< Endpoint descriptor. */
    CY_USB_DEVICE_QUAL_DSCR,                    /**< Device qualifier descriptor. */
    CY_USB_OTHERSPEED_DSCR,                     /**< Other Speed configuration descriptor. */
    CY_USB_BOS_DSCR = 0x0F,                     /**< BOS descriptor. */
    CY_DEVICE_CAPB_DSCR,                        /**< Device capability descriptor. */
    CY_SS_ENDP_COMPN_DSCR = 0x30,               /**< SS endpoint companion descriptor. */
    CY_SSPLUS_ISO_ENDP_COMPN_DSCR = 0x31        /**< SSP isochronous endpoint companion descriptor. */
} cy_en_usb_dscr_type_t;

/**
 * @typedef cy_en_usb_feature_selector_t
 * @brief Defines used in set/clear feature command.
 */
typedef enum {
    CY_USB_FEATURE_ENDP_HALT = 0,               /**< Endpoint Halt: Addressed to endpoint. */
    CY_USB_FEATURE_FUNC_SUSPEND = 0,            /**< Function Suspend: Addressed to interface. */
    CY_USB_FEATURE_DEVICE_REMOTE_WAKE = 1,      /**< Remote wake */
    CY_USB_FEATURE_DEVICE_TEST_MODE = 2,        /**< USB 2.x electrical test mode. */
    CY_USB_FEATURE_U1_ENABLE   = 48,            /**< USB 3.x U1 Enable   */
    CY_USB_FEATURE_U2_ENABLE   = 49,            /**< USB 3.x U2 Enable   */
    CY_USB_FEATURE_LTM_ENABLE  = 50,            /**< USB 3.x LTM Enable  */
    CY_USB_FEATURE_LDM_ENABLE  = 53             /**< USB 3.x LDM Enable  */
}cy_en_usb_feature_selector_t;

/**
 * @typedef cy_en_usb_enum_method_t
 * @brief Define for enumeration method.
 */
typedef enum {
    CY_USB_ENUM_METHOD_FAST = 0,                /**< USBD layer handles enumeration process. */
    CY_USB_ENUM_METHOD_APPLICATION,             /**< Application layer will handle enumeration. */
}cy_en_usb_enum_method_t;

/**
 * @typedef cy_en_usbd_ret_code_t
 * @brief USBD layer return code shared between USBD layer and Application layer.
 */
typedef enum {
    CY_USBD_STATUS_SUCCESS=0,                   /**< Function completed successfully. */
    CY_USBD_STATUS_FAILURE,                     /**< Unexpected failure condition. */
    CY_USBD_STATUS_BAD_PARAM,                   /**< Invalid parameter passed to API. */
    CY_USBD_STATUS_CTXT_NULL,                   /**< USBD context pointer is NULL. */
    CY_USBD_STATUS_PTR_NULL,                    /**< NULL pointer passed for necessary parameters. */
    CY_USBD_STATUS_INVALID_CALLBACK_TYPE,       /**< Invalid callback type specified. */
    CY_USBD_STATUS_INVALID_DSCR_TYPE,           /**< Invalid descriptor type specified. */
    CY_USBD_STATUS_INVALID_INDEX,               /**< Invalid descriptor index specified. */
    CY_USBD_STATUS_INVALID_CONFIG_NUMBER,       /**< Invalid configuration index specified. */
    CY_USBD_STATUS_MALLOC_FAILED,               /**< Memory allocation failed. */
    CY_USBD_STATUS_MSG_SEND_FAIL,               /**< Sending of message to application layer failed. */
    CY_USBD_STATUS_CTRL_REQ_HANDLE_FAIL,        /**< Stack was unable to handle a control request. */
    CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM,   /**< Invalid endpoint configuration specified. */
    CY_USBD_STATUS_ENDP_CONFIG_FAIL,            /**< Failed to configure endpoint. */
    CY_USBD_STATUS_TIMEOUT,                     /**< Operation timed out. */
    CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED,    /**< Invalid descriptor field queried. */
}cy_en_usbd_ret_code_t;

/**
 * @typedef cy_en_usbd_notification_type_t
 * @brief Types of device notifications applicable to USB3.2 devices. */
typedef enum {
    CY_USBD_NOTIF_RESERVED = 0,         /**< Invalid value. */
    CY_USBD_NOTIF_FUNC_WAKE,            /**< Function wake notification. */
    CY_USBD_NOTIF_LTM,                  /**< Latency tolerance message. */
    CY_USBD_NOTIF_BIAM,                 /**< Bus interval adjustment message. */
    CY_USBD_NOTIF_RSVD4,                /**< Host Role Request message is reserved and should not be used. */
    CY_USBD_NOTIF_SBLNK_SPEED           /**< Sublink Speed notification. */
}cy_en_usbd_notification_type_t;

/** \} group_usbfxstack_usb_common_enums */

#ifndef DOXYGEN
typedef struct cy_stc_usb_usbd_ctxt_t cy_stc_usb_usbd_ctxt_t;
#endif /* DOXYGEN */

/**
 * \addtogroup group_usbfxstack_usb_common_typedefs
 * \{
*/

/**
 * USB stack to application callback prototype.
 * A function of this type is used by the USBD stack to notify the application
 * about events of interest.
 */
typedef void (* cy_usb_usbd_callback_t) (
        void *pAppCtxt,                         /**< Application context pointer. */
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,      /**< USBD stack context pointer. */
        cy_stc_usb_cal_msg_t *pMsg              /**< Message pointer. */
        );

/** \} group_usbfxstack_usb_common_typedefs */

/**
 * \addtogroup group_usbfxstack_usb_common_structs
 * \{
 */

/**
 * @brief Application registers required descriptors through given API and descriptors
 *  will be stored in this data structure.
 */
typedef struct
{
    uint8_t     *pUsbDevDscr;                   /**< Pointer to USB 2.x device descriptor. */
    uint8_t     *pUsbSsDevDscr;                 /**< Pointer to USB 3.x device descriptor. */
    uint8_t     *pUsbDevQualDscr;               /**< Pointer to device qualifier descriptor. */
    uint8_t     *pUsbCfgDscr;                   /**< Pointer to active configuration descriptor. */
    uint8_t     *pUsbOtherSpeedCfgDscr;         /**< Pointer to other speed configuration descriptor. */
    uint8_t     *pUsbFsCfgDscr;                 /**< Pointer to full-speed configuration descriptor. */
    uint8_t     *pUsbHsCfgDscr;                 /**< Pointer to high-speed configuration descriptor. */
    uint8_t     *pUsbSsCfgDscr;                 /**< Pointer to USB 3.x configuration descriptor. */
    uint8_t     *pUsbStringDescr[CY_USBD_MAX_STR_DSCR_INDX + 1]; /**< Pointers to string descriptors. */
    uint8_t     *pUsbSsBosDscr;                 /**< Pointer to USB 3.x BOS descriptor. */
    uint8_t     *pUsbHsBosDscr;                 /**< Pointer to USB 2.x BOS descriptor. */
}cy_stc_usb_set_dscr_ptrs_t;

/**
 * @brief Endpoint related configuration and status information is stored in this
 * structure.
 */
typedef struct
{
    bool valid;                                 /**< Whether endpoint is valid (has been enabled) */
    bool mapped;                                /**< Whether endpoint has been mapped to hardware resource. */
    bool halt;                                  /**< Whether endpoint has been stalled. */
    bool allowNakTillDmaRdy;                    /**< Whether OUT endpoint should NAK transfers when DMA is not ready */
    cy_en_usb_endp_type_t endpType;             /**< Type of endpoint. */
    uint32_t maxPktSize;                        /**< Maximum packet size. */
    uint32_t mult;                              /**< Mult (number of bursts per interval) for isochronous EPs. */
    uint32_t burstSize;                         /**< Burst size in number of packets. */
    uint32_t streamID;                          /**< Active stream number for USB 3.x bulk endpoints. */
    uint16_t retryBufOffset;                    /**< Offset to retry buffer region allocated for the endpoint. */
}cy_stc_usb_endp_info_t;

/**
 * @brief Structure containing all information managed by the USBD device stack.
 */
struct cy_stc_usb_usbd_ctxt_t
{
    cy_en_usb_device_state_t devState;                  /**< Current device state. */
    cy_en_usb_device_state_t prevDevState;              /**< Previous device state. */
    cy_en_usb_speed_t devSpeed;                         /**< Current device speed. */
    cy_en_usb_speed_t startSsDevSpeed;                  /**< Desired USB 3.x connection speed. */
    cy_en_usb_speed_t startHsDevSpeed;                  /**< Desired USB 2.x connection speed. */
    cy_en_usb_enum_method_t enumMethod;                 /**< Method of handling control requests. */
    cy_en_usb_endp0_state_t endp0State;                 /**< State of control endpoint. */

    cy_stc_usb_set_dscr_ptrs_t dscrs;                   /**< USB descriptors registered with the stack. */
    uint8_t *pActiveCfgDscr;                            /**< Pointer to the active configuration descriptor. */
    uint8_t devAddr;                                    /**< Current USB device address. */
    uint8_t activeCfgNum;                               /**< Active configuration index. */
    uint8_t numOfIntfInActiveCfg;                       /**< Number of interfaces in the active configuration. */
    uint32_t intfRemoteWakeEnabled;                     /**< Whether function wake is enabled. */
    uint16_t altSettings[CY_USB_MAX_INTF];              /**< Active alternate setting for each interface. */
    uint16_t numOfAltSettings[CY_USB_MAX_INTF];         /**< Number of alternate settings in each interface. */

    uint8_t selfPowered;                                /**< Whether device is self powered. */
    uint8_t remoteWakeupAbility;                        /**< Whether remote wake is supported. */
    uint8_t remoteWakeupEnable;                         /**< Whether remote wake is enabled. */
    uint8_t usbDeviceStat;                              /**< Device status to be sent in response to GET_STATUS. */

    cy_stc_usb_setup_req_t setupReq;                    /**< Active control request information. */
    bool setupReqActive;                                /**< Whether any control request is active. */
    bool ep0SendDone;                                   /**< Whether any EP0-IN request has been responded to. */

    bool lpmDisabled;                                   /**< Whether application has disabled LPM entry. */
    bool lpmEnablePending;                              /**< Whether LPM re-enable is pending. */

    cy_stc_usb_endp_info_t endpInfoIn[CY_USB_MAX_ENDP_NUMBER];  /**< Status of IN endpoints. */
    cy_stc_usb_endp_info_t endpInfoOut[CY_USB_MAX_ENDP_NUMBER]; /**< Status of OUT endpoints. */

    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr0;          /**< DMA descriptor used for EP0-OUT transfer. */
    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr1;          /**< DMA descriptor used for EP0-OUT transfer. */
    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr2;          /**< DMA descriptor used for EP0-OUT transfer. */
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr0;         /**< DMA descriptor used for EP0-IN transfer. */
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr1;         /**< DMA descriptor used for EP0-IN transfer. */
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr2;         /**< DMA descriptor used for EP0-IN transfer. */

    uint32_t channel0;                                  /**< DMA channel index used for EP0-OUT transfer. */
    uint32_t channel1;                                  /**< DMA channel index used for EP0-IN transfer. */

    cy_stc_hbdma_mgr_context_t *pHBDmaMgr;              /**< High BandWidth DMA manager context pointer. */

    cy_stc_hbdma_channel_t inEp0DmaUsb3Ch;              /**< DMA channel used for EP0-IN transfers. */
    cy_stc_hbdma_channel_t outEp0DmaUsb3Ch;             /**< DMA channel used for EP0-OUT transfers. */
    uint32_t *inEp0ScratchBuffer;                       /**< Temporary buffer used to copy data sent on EP0 */

    cy_usb_usbd_callback_t busResetCb;                  /**< Bus reset callback function. */
    cy_usb_usbd_callback_t busResetDoneCb;              /**< Reset done callback function. */
    cy_usb_usbd_callback_t busSpeedCb;                  /**< FS to HS speed change callback function. */
    cy_usb_usbd_callback_t setupCb;                     /**< Control request callback function. */
    cy_usb_usbd_callback_t suspendCb;                   /**< Suspend entry callback function. */
    cy_usb_usbd_callback_t resumeCb;                    /**< Suspend exit callback function. */
    cy_usb_usbd_callback_t setConfigCb;                 /**< SetConfig callback function. */
    cy_usb_usbd_callback_t setIntfCb;                   /**< SetInterface callback function. */
    cy_usb_usbd_callback_t statusStageComplCb;          /**< Control request ACK callback function. */
    cy_usb_usbd_callback_t l1SleepCb;                   /**< USB 2.x L1 entry callback function. */
    cy_usb_usbd_callback_t l1ResumeCb;                  /**< USB 2.x L1 exit callback function. */
    cy_usb_usbd_callback_t zlpCb;                       /**< USB 2.x ZLP transfer callback function. */
    cy_usb_usbd_callback_t slpCb;                       /**< USB 2.x SLP transfer callback function. */
    cy_usb_usbd_callback_t doneCb;                      /**< USB 2.x transfer done callback function. */
    cy_usb_usbd_callback_t DisconnectCb;                /**< USB disconnect callback function. */
    cy_usb_usbd_callback_t setAddrCb;                   /**< SetAddress callback function. */
    cy_usb_usbd_callback_t ep0RecvCb;                   /**< Callback for EP0 OUT transfer completion. */
    cy_usb_usbd_callback_t debugSlpCb;                  /**< USB 2.x Debug endpoint SLP transfer callback function. */
    cy_usb_usbd_callback_t itpCb;                       /**< Callback for SOF/ITP interrupt. */

    DMAC_Type *pCpuDmacBase;                            /**< Pointer to DMAC register structure. */

    cy_stc_usb_cal_ctxt_t *pCalCtxt;                    /**< Pointer to USBHS CAL driver context structure. */
    cy_stc_usbss_cal_ctxt_t *pSsCalCtxt;                /**< Pointer to USBSS CAL driver context structure. */
    void *pAppCtxt;                                     /**< Application context to be passed to callbacks. */

#if FREERTOS_ENABLE
    TaskHandle_t usbdTaskHandle;                        /**< USBD stack task. */
    QueueHandle_t usbdMsgQueue;                         /**< Message queue to get messages from HS/SS CAL. */
    TimerHandle_t usbdTimerHandle;                      /**< Timer instance used by the stack. */
    TimerHandle_t usb3ConnectTimer;                     /**< Timer instance used to enable USB3 connection. */
    bool usb3ConnectTimerStarted;                       /**< Whether usb3ConnectTimer has been started. */
    bool lpbkStateTimerStarted;                         /**< Whether loopback state timer is running. */
#else
    uint32_t nextTaskTick;                              /**< The time tick at which next USBD task should be run. */
#endif /* FREERTOS_ENABLE */

    uint32_t otherSpeedCfgDscrBuf[128U];                /**< Copy of other speed configuration descriptor. */
    bool EnumerationDone;                               /**< Whether enumeration is complete. */
    bool strDscrAvailable;                              /**< Whether any string descriptors have been registered. */

    uint8_t ssRetryBufStatus[8];                        /**< Bitmap used for USB 3.x retry buffer allocation. */

    bool    disableHsOnComplianceEntry;                 /**< Unused configuration flag: reserved. */
    uint8_t termDetectCount;                            /**< Number of USB 3.x connection attempts made. */
    bool    isHsEnabled;                                /**< Whether USBHS block is enabled. */

    uint32_t *pUsbEvtBuf;                               /**< Memory buffer used to save USB driver and stack status. */
    uint16_t  usbEvtLogSize;                            /**< Size of debug memory buffer in DWORDS. */
    uint16_t  usbEvtLogIdx;                             /**< Next debug memory buffer index. */

    bool handleRxFailure;                               /**< Enable work-around for intermittent Gen2 training failure. */
    uint8_t gen2_lnk_retry_limit;                       /**< Number of Gen2 re-connect attempts to be made. */
    uint8_t gen2_lnk_retry_cnt;                         /**< Number of Gen2 re-connect attempts made so far. */

    uint8_t *pEp0ReceiveBuffer;                         /**< Buffer into which EP0 OUT data should be received. */
    uint16_t ep0ExpRcvSize;                             /**< Size of expected transfer on EP0-OUT. */

    uint8_t debugOutEp;                                 /**< Debug OUT Endpoint Number*/
    bool ssOnBusResetDisable;                           /**< Whether SS connect on USB 2.x Bus Reset is disabled. */
    bool underrunWarningDone;                           /**< Whether underrun warning has been issued. */
};

/** \} group_usbfxstack_usb_common_structs */

/**
 * \addtogroup group_usbfxstack_usb_common_functions
 * \{
 */

/*******************************************************************************
* Function name: Cy_USBD_GetVersion
****************************************************************************//**
*
* Get the USBD middleware version information.
*
* \return
* 32-bit version information including major, minor, patch and build numbers.
*
*******************************************************************************/
uint32_t Cy_USBD_GetVersion(void);

/*******************************************************************************
* Function name: Cy_USB_USBD_Init
****************************************************************************//**
*
* This function initializes USBD layer and activates CAL layer initialization
* function for SS and HS controller.
*
* \param pAppCtxt
* application layer context pointer
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCpuDmacBase
* Base address for CPU DMA.
*
* \param pCalCtxt
* CAL layer context pointer for HS controller.
*
* \param pSsCalCtxt
* CAL layer context pointer for SS controller.
*
* \param pHbDmaMgrCtxt
* High Bandwidth DMA MgrCtxt.
*
* \return
* CY_USBD_STATUS_SUCCESS if the operation is successful.
* CY_USBD_STATUS_FAILURE if the opration is failed.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_Init(void *pAppCtxt,
                                       cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       DMAC_Type *pCpuDmacBase,
                                       cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                       cy_stc_usbss_cal_ctxt_t *pSsCalCtxt,
                                       cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);


/*******************************************************************************
* Function name: Cy_USBD_DebugRegisterCallback
****************************************************************************//**
*
*  This API will be used by CDC (Debug) interface to register required callback
*
* \param pUsbdCtxt
* USBD Context
*
* \param debugOutEp
* CDC (Debug) OUT endpoint number.
*
* \param callBackType
* USBD callback type
*
* \param callBackFunc
* Registered callback function
*
* \return
* CY_USBD_STATUS_SUCCESS if the operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if ednpoint number is out of range
* CY_USBD_STATUS_INVALID_CALLBACK_TYPE if callback type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_DebugRegisterCallback (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint8_t debugOutEp,
                          cy_en_usb_usbd_cb_t callBackType,
                          cy_usb_usbd_callback_t callBackFunc);


/*******************************************************************************
* Function name: Cy_USBD_RegisterCallback
****************************************************************************//**
*
* This API will be used by application to register required callback.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param callBackType
* Base address for CPU DMA.
*
* \param callBackFunc
* Base address for DATAWIRE0.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_INVALID_CALLBACK_TYPE if callback type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_RegisterCallback
                                        (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_en_usb_usbd_cb_t callBackType,
                                         cy_usb_usbd_callback_t callBackFunc);

/*******************************************************************************
* Function name: Cy_USBD_SetDscr
****************************************************************************//**
*
* This API will be used by application to provide various standard descriptors
* to USBD layer and USBD layer will use this descriptor in "fast enumeration".
* It is callers responsibility to provide proper descriptor with right index.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param dscrType
* Type of descriptor.
*
* \param dscrIndex
* Descriptor Index.
*
* \param pDscr
* Pointer to descriptor data based on type.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to descriptor is NULL.
* CY_USBD_STATUS_INVALID_DSCR_TYPE if descriptor type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SetDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                      cy_en_usb_set_dscr_type_t dscrType,
                                      uint8_t dscrIndex, uint8_t *pDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetDeviceSpeed
****************************************************************************//**
*
* This API returns device's operating speed.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_USB_DEV_NOT_CONNECTED if device is not connected.
* cy_en_usb_speed_t speed of device.
*
*******************************************************************************/
cy_en_usb_speed_t Cy_USBD_GetDeviceSpeed(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_SetDeviceSpeed
****************************************************************************//**
*
* This API force operating speed of dvice.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param speed
* speed of device.
*
*******************************************************************************/
void Cy_USBD_SetDeviceSpeed(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_en_usb_speed_t speed);

/*******************************************************************************
* Function name: Cy_USBD_SetDeviceSpeedAtUSBDOnly
****************************************************************************//**
*
* This API is used to set device speed only at USBD layer.
* This API should be used when application wants to fetch different speed
* dependent data even before enumaration is done. During enumeration speed
* field will be overwritten at USBD layer.
* It is application's responsibility to pass right speed.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param speed
* speed of device.
*
*******************************************************************************/
void Cy_USBD_SetDeviceSpeedAtUSBDOnly(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                      cy_en_usb_speed_t speed);


/*******************************************************************************
* Function name: Cy_USBD_GetDeviceState
****************************************************************************//**
*
* This API returns present state of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USB_DEVICE_STATE_INVALID if usbd context NULL.
* cy_en_usb_device_state_t present state of device.
*
*******************************************************************************/
cy_en_usb_device_state_t Cy_USBD_GetDeviceState
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);


/*******************************************************************************
* Function name: Cy_USBD_FindEndp0MaxPktSize
****************************************************************************//**
*
* This API finds endpoint 0 max packet size from device descriptor. device
* descriptors are different at different speed.
*
* \param pDevDscr
* Pointer to device descriptor.
*
* \param devSpeed
* Speed of device for which endpoint 0 size is required.
*
* \param pMaxPktSize
* Max packet size of endpoint 0 will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM in all other case.
* cy_en_usb_device_state_t present state of device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_FindEndp0MaxPktSize(uint8_t *pDevDscr,
                                                  cy_en_usb_speed_t devSpeed,
                                                  uint32_t *pMaxPktSize);

/*******************************************************************************
* Function name: Cy_USBD_GetLpmBosUSBExt
****************************************************************************//**
*
*  Get information about LPM support from USB ext descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pLpmSupp
* Information about LPM support is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not register OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetLpmBosUSBExt(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              uint8_t *pLpmSupp);


/*******************************************************************************
* Function name: Cy_USBD_GetAttributeBosUSBExt
****************************************************************************//**
*
*  Get attribute from USB ext descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pAttribute
* Required information related to attribute will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not register OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetAttributeBosUSBExt(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                    uint32_t *pAttribute);

/*******************************************************************************
* Function name: Cy_USBD_GetAttributeBosSS
****************************************************************************//**
*
*  Get attribute from Super speed BoS descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pAttribute
* Attribute field from BOS descriptor will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetAttributeBosSS(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                uint8_t *pAttribute);

/*******************************************************************************
* Function name: Cy_USBD_GetSpeedSuppBosSS
****************************************************************************//**
*
*  Get Speed supported from Super speed descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pSpeedSupp
* speed information is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetSpeedSuppBosSS(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                uint32_t *pSpeedSupp);

/*******************************************************************************
* Function name: Cy_USBD_GetU1U2ExitLatBosSS
****************************************************************************//**
*
*   Get U1 and U2 exit latency  from Super speed BoS descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pU1ExitLat
* U1 exit latency will be stored here.
*
* \param pU2ExitLat
* U2 exit latency will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetU1U2ExitLatBosSS
                                  (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                   uint8_t *pU1ExitLat, uint16_t *pU2ExitLat);

/*******************************************************************************
* Function name: Cy_USBD_GetAttributeBosSSP
****************************************************************************//**
*
*  Get attribute from Super speed plus descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pAttribute
* attribute will be store here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetAttributeBosSSP(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 uint32_t *pAttribute);

/*******************************************************************************
* Function name: Cy_USBD_GetFunctSupportBosSSP
****************************************************************************//**
*
*  Get functionality supported from Super speed plus descriptor
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pFuncSupp
* Information about functionality supported is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetFunctSupportBosSSP(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                    uint32_t *pFuncSupp);

/*******************************************************************************
* Function name: Cy_USBD_isCfgValid
****************************************************************************//**
*
*  This function checks validity of given configuration.
*
* \param cfgNum
* Configuration Number.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if config is valid.
* FALSE if config is not valid.
*
*******************************************************************************/
bool Cy_USBD_isCfgValid(uint8_t cfgNum, const uint8_t *pCfgDscr);

/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveCfgNum
****************************************************************************//**
*
* This API called by application to know active configuration number.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCfgNum
* config number will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_GetActiveCfgNum
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint8_t *pCfgNum);

/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveCfgDscr
****************************************************************************//**
*
* This API called by application to get active configuration descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* Pointer to active configuration descriptor.
*
*******************************************************************************/
uint8_t *Cy_USB_USBD_GetActiveCfgDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_FindNumOfIntf
****************************************************************************//**
*
*  This function finds number of interface supported in given configation.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* 0x00 if Configuration descriptor is NULL.
* Number of interface supported by configuration.
*
*******************************************************************************/
uint8_t Cy_USBD_FindNumOfIntf(const uint8_t *pCfgDscr);

/*******************************************************************************
* Function name: Cy_USBD_FindSelfPower
****************************************************************************//**
*
*  This function finds device supports Self power or BUS power.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if device supports self power.
* FALSE if device supports only BUS power.
*
*******************************************************************************/
bool Cy_USBD_FindSelfPower(const uint8_t *pCfgDscr);

/*******************************************************************************
* Function name: Cy_USBD_FindRemoteWakeupAbility
****************************************************************************//**
*
*  This function finds device supports remote wakeup or not.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if device supports remote wakeup.
* FALSE if device does not support remote wakeup.
*
*******************************************************************************/
bool Cy_USBD_FindRemoteWakeupAbility(uint8_t *pCfgDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetRemoteWakeupStatus
****************************************************************************//**
*
*  This function checks L2 remote wakeup status and return information.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* TRUE if remote wakeup enabled in HW register.
* FALSE if remote wakeup disabled in HW register.
*
*******************************************************************************/
bool Cy_USBD_GetRemoteWakeupStatus(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_SignalRemoteWakeup
****************************************************************************//**
*
*  This function triggers HW to start/end remotewakeup signaling.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param startEnd
* True to start signaling and false to end signaling.
*
*******************************************************************************/
void Cy_USBD_SignalRemoteWakeup(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                bool startEnd);


/* Interface descriptor related functions. */
/*******************************************************************************
* Function name: Cy_USBD_GetIntfDscr
****************************************************************************//**
*
*  This function called by application to get interface descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* Interface number.
*
* \param altSetting
* Alternate setting number.
*
* \return
* NULL if USBD context pointer is NULL.
* pointer to Interface descriptor.
*
*******************************************************************************/
uint8_t *Cy_USBD_GetIntfDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t intfNum,
                             uint8_t altSetting);

/*******************************************************************************
* Function name: Cy_USBD_isIntfValid
****************************************************************************//**
*
*  This function checks given interface is part of configuration descriptor or not.
*
* \param intf
* Interface number.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if given interfae number is part of configuration descriptor.
* FALSEif given interface is not part of configuration descriptor.
*
*******************************************************************************/
bool Cy_USBD_isIntfValid(uint16_t intf, uint8_t *pCfgDscr);

/*******************************************************************************
* Function name: Cy_USBD_FindNumOfEndp
****************************************************************************//**
*
*  This function  gives number of Endpoints supported by given interface.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* 0x00 if interface descriptor is NULL.
* Number of endpoints.
*
*******************************************************************************/
uint8_t Cy_USBD_FindNumOfEndp(uint8_t *pIntfDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpDscr
****************************************************************************//**
*
*  This function to fetch endpoint descriptor for a given interface descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to endpoint descriptors.
*
*******************************************************************************/
uint8_t *Cy_USBD_GetEndpDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pIntfDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetSsEndpCompDscr
****************************************************************************//**
*
*  This function to fetch super speed endpoint companion descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *Cy_USBD_GetSsEndpCompDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetSspIsoCompDscr
****************************************************************************//**
*
*  This function to fetch super speed plus Iso endpoint companion descriptor from
*  given companion descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCompDscr
* pointer to endpoint companion descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to Iso endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *Cy_USBD_GetSspIsoCompDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    uint8_t *pCompDscr);


/*******************************************************************************
* Function name: Cy_USBD_GetSspIsoEndpCompDscr
****************************************************************************//**
*
*  This function to fetch super speed plus Iso endpoint companion descriptor from
*  given endpoint descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to super speed plus Iso endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *Cy_USBD_GetSspIsoEndpCompDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveAltSetting
****************************************************************************//**
*
*  This function called by application to know active altSetting for a given
*  interface.
*
* \param pUsbdCtxt
* USBD layer context pointer.

* \param intfNum
* interface number.
*
* \param pAltSetting
* required alt setting pointer.
*
* \return
* CY_USBD_STATUS_CTXT_NULL when context is null.
* CY_USBD_STATUS_FAILURE when active config descriptor is null.
* CY_USBD_STATUS_SUCCESS whem able to fetch altsetting number.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_GetActiveAltSetting
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint8_t intfNum,
                                           uint8_t *pAltSetting);

/*******************************************************************************
* Function name: Cy_USBD_FindAltSetting
****************************************************************************//**
*
* This API gives information about alt setting supported by given interface.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* number of alt setting supported by an interface.
*
*******************************************************************************/
uint8_t Cy_USBD_FindAltSetting(uint8_t *pIntfDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetNumOfAltSetting
****************************************************************************//**
*
* This API gets number of alt setting in an interface from USBD context data structure.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* interface number.
*
* \return
* number of alt setting supported by interface.
*
*******************************************************************************/
uint8_t Cy_USBD_GetNumOfAltSetting(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                   uint8_t intfNum);

/*******************************************************************************
* Function name: Cy_USBD_UpdateNumOfAltSetting
****************************************************************************//**
*
* This API gets number of alt setting in an interface.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* interface number.
*
* \return
* number of alt setting supported by interface.
*
*******************************************************************************/
uint8_t Cy_USBD_UpdateNumOfAltSetting(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                      uint8_t intfNum);

/* Endpoint related functions. */
/*******************************************************************************
* Function name: Cy_USBD_EndpDscrValid
****************************************************************************//**
*
*  This API confirms given descriptor is endpoint descriptor or not.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* TRUE if given descriptor is endpoint descriptor.
* FALSE if given descriptor is not endpoint descriptor.
*
*******************************************************************************/
bool Cy_USBD_EndpDscrValid(uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpNumMaxPktDir
****************************************************************************//**
*
*  This function fetches endpoint number, maxPktSize and direction from endpoint
*  descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pEndpNum
* endpoint number will be stored here.
*
* \param pMaxPktSize
* max packet size of endpoint will be stored here.
*
* \param pDir
* Direction of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_PTR_NULL if any of the parameter is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpNumMaxPktDir(uint8_t *pEndpDscr,
                                                  uint32_t *pEndpNum,
                                                  uint16_t *pMaxPktSize,
                                                  uint32_t *pDir);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpMaxPktSize
****************************************************************************//**
*
*  This function fetches maxPktSize from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pMaxPktSize
* max packet size of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpMaxPktSize(uint8_t *pEndpDscr,
                                                uint16_t *pMaxPktSize);

/*******************************************************************************
* Function name: Cy_USBD_GetIsoBytesPerIntvl
****************************************************************************//**
*
*  This function fetches max bytes per interval for Iso endpoint from
*  iso companion endpoint descriptor.
*
* \param pIsoCompDscr
* pointer to Iso endpoint companion descriptor.
*
* \param pBytesPerIntvl
* Bytes per interval field from the descriptor will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if ompanion descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetIsoBytesPerIntvl (uint8_t *pIsoCompDscr,
                                                  uint32_t *pBytesPerIntvl);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpAttribute
****************************************************************************//**
*
*  This function fetches endpoint attribute from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pAttribute
* atrribute of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpAttribute(uint8_t *pEndpDscr,
                                               uint8_t *pAttribute);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpType
****************************************************************************//**
*
*  This function fetches endpoint type from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pEndpType
*  endpoint type is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to endpoint type is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpType(uint8_t *pEndpDscr,
                                          uint32_t *pEndpType);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpInterval
****************************************************************************//**
*
*  This function fetches bInterval from endpoint descriptor.
*
* \param pEndpDscr
* Pointer to endpoint descriptor.
*
* \param pInterval
* Endpoint polling interval is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to interval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpInterval(uint8_t *pEndpDscr,
                                              uint8_t *pInterval);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnMaxburst
****************************************************************************//**
*
*  This function fetches bMaxBurst from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pMaxBust
*  max burst information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pMaxBurst is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpCompnMaxburst(uint8_t *pEndpCompnDscr,
                                                   uint8_t *pMaxBust);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnMaxStream
****************************************************************************//**
*
*  This function fetches bMaxStream from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pMaxStream
*  max stream information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pMaxStream is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpCompnMaxStream(uint8_t *pEndpCompnDscr,
                                                    uint8_t *pMaxStream);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnAttribute
****************************************************************************//**
*
*  This function fetches attribute from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pAttribute
*  attribute information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pAttribute is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpCompnAttribute(uint8_t *pEndpCompnDscr,
                                                    uint8_t *pAttribute);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnBytePerInterval
****************************************************************************//**
*
*  This function fetches bytePerInterval from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pBytePerInterval
*  Byte per interval information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBytePerInterval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpCompnBytePerInterval
                                                   (uint8_t *pEndpCompnDscr,
                                                    uint16_t *pBytePerInterval);

/*******************************************************************************
* Function name: Cy_USBD_GetEndpSspIsoCompnBytePerInterval
****************************************************************************//**
*
*  This function fetches bytePerInterval from endpoint SS Iso companion descriptor.
*
* \param pSspIsoEndpCompnDscr
* pointer to super speed Iso endpoint companion descriptor.
*
* \param pBytePerInterval
*  Bytes per interval information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_PTR_NULL if pBytePerInterval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetEndpSspIsoCompnBytePerInterval
                                               (uint8_t *pSspIsoEndpCompnDscr,
                                                uint32_t *pBytePerInterval);

/*******************************************************************************
* Function name: Cy_USBD_EnableEndp
****************************************************************************//**
*
*  This function enable/disable endpoint. Based on speed, this function will
*  call appropriate HW function to enable/disable endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param enable
* true for enabling endpoint and false for disabling endpoint.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_EnableEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         uint32_t endpNum,
                                         cy_en_usb_endp_dir_t endpDir,
                                         bool enable);

/*******************************************************************************
* Function name: Cy_USBD_SetEpBurstMode
****************************************************************************//**
*
*  This function enables/disables the MULT (allow burst across multiple DMA buffers)
*  feature for the specified USB endpoints. Enabling the feature can improve the data
*  transfer throughput on the respective endpoints. The settings need to be updated
*  in the USB IP as well as at the DMA socket level.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \param endpDirection
* endpoint direction.
*
* \param enable
* true for enabling burst mode and false for disabling burst mode.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SetEpBurstMode
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        bool enable);

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpConfig
****************************************************************************//**
*
*  This function configures endpoint based on user requirement.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpConfig
* configuration parameters passed by user.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM if any of the configuration parameter invalid.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpConfig(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_endp_config_t endpConfig);


/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearStall
****************************************************************************//**
*
*  This function enable or disable STALL condition for the endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param setClear
* true for set and false for clear stall condition.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpSetClearStall
                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                           cy_en_usb_endp_dir_t endpDir, bool setClear);

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearNakNrdy
****************************************************************************//**
*
*  This function enable or disable NAK/NRDY condition for the endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param setClear
* true for set and false for clear NAK condition.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpSetClearNakNrdy
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNum,
                                        cy_en_usb_endp_dir_t endpDir,
                                        bool setClear);

/*******************************************************************************
* Function name: Cy_USBD_ResetEndp
****************************************************************************//**
*
*  This function resets endpoint related paramters by calling appropriate function
*  at HW level. This clears sticky bits e.g retry bit, flowcontrol bit.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param preserveSeqNo
* true to preserve sequence number and false to reset sequence number.
*
*******************************************************************************/
void Cy_USBD_ResetEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                       cy_en_usb_endp_dir_t endpDir, bool preserveSeqNo);

/*******************************************************************************
* Function name: Cy_USBD_FlushEndp
****************************************************************************//**
*
*  This function flush endpoint data present in EPM.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \param endpDirection
* endpoint direction.
*
*******************************************************************************/
void Cy_USBD_FlushEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       uint32_t endpNumber,
                       cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function name: Cy_USBD_FlushEndpAll
****************************************************************************//**
*
*  This function flush all endpoints data endpoint data present in EPM.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpDir
* Parameter is unused. Both IN and OUT endpoints will be flushed.
*
*******************************************************************************/
void Cy_USBD_FlushEndpAll(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function name: Cy_USBD_FlushResetEndpAll
****************************************************************************//**
*
*  This function flush/reset all endpoint with perticular direction based on
*  speed of device controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpDirection
* endpoint direction.
*
*******************************************************************************/
void Cy_USBD_FlushResetEndpAll(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearNakNrdyAll
****************************************************************************//**
*
*  This function enable or disable NAK/NRDY condition on all endpoints based on
*  speed of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* set ie true to enable NAK/NRDY and clear ie false to disable NAK/NRDY.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpSetClearNakNrdyAll
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        bool setClear);
/*******************************************************************************
* Function name: Cy_USBD_EndpIsNakNrdySet
****************************************************************************//**
*
*  This function checks whether the specified endpoint currently has the NAK/NRDY bit set.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \return
* TRUE if NAK/NRDY bit is set
* FALSE if NAK/NRDY bit is reset
*
*******************************************************************************/
bool Cy_USBD_EndpIsNakNrdySet(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              uint32_t endpNum,
                              cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function name: Cy_USBD_EndpIsStallSet
****************************************************************************//**
*
*  This function checks whether the specified endpoint currently has the STALL bit set.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDirection
* endpoint direction.
*
* \return
* TRUE if STALL bit is set
* FALSE if STALL bit is reset
*
*******************************************************************************/
bool Cy_USBD_EndpIsStallSet(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function name: Cy_USBD_EndpMapStream
****************************************************************************//**
*
* This function maps an unused USB Ingress/Egress socket to the specified stream
* associated with a bulk endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \param endpDirection
* endpoint direction.
*
* \param streamId
* stream number.
*
* \param socketNum
* socket number to be mapped.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_EndpMapStream
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        uint16_t streamId,
                                        uint32_t socketNum);

/*******************************************************************************
* Function name: Cy_USBD_EndpUnmapStream
****************************************************************************//**
*
* This function unmaps the USB ingress/egress socket to bulk stream mapping
* for a bulk endpoint and socket.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param socketNum
* socket number to be mapped.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EndpUnmapStream(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                       cy_en_usb_endp_dir_t endpDir, uint32_t socketNum);

/*******************************************************************************
* Function name: Cy_USBD_EndpSetPktsPerBuffer
****************************************************************************//**
*
*  This function is used to update the USB block with the number of max. sized
* packets received on a SuperSpeed OUT endpoint that can fit into one DMA buffer.
* Setting this value is required for proper operation in cases where the maximum
* packet size of the endpoint is not a power of 2.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \param pktsPerBuffer
* Number of packets that can fit in one DMA buffer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_EndpSetPktsPerBuffer
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        uint8_t pktsPerBuffer);

/*******************************************************************************
* Function name: Cy_USBD_UpdateXferCount
****************************************************************************//**
*
* This function will update xfer count based on speed of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \param endpDirection
* endpoint direction.
*
* \param bufferSize
* buffer size.
*
*******************************************************************************/
void Cy_USBD_UpdateXferCount(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint32_t endpNumber,
                             cy_en_usb_endp_dir_t endpDirection,
                             uint32_t bufferSize);

/*******************************************************************************
* Function name: Cy_USBD_CtrlEndp0DataOutAck
****************************************************************************//**
*
* Data stage ACK for endpoint0 is controlled by this function. In USBHS,
* when "CONT_TO_DATA" bit is set then controller won't send ACK till data
* is validated.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* true for set and false for clear.
*
*******************************************************************************/
void Cy_USBD_CtrlEndp0DataOutAck(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                 bool setClear);

/*******************************************************************************
* Function name: Cy_USBD_EnableStatusCtrl
****************************************************************************//**
*
* This function enable/disable status control feature which is part of SS controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param enable
* true for enable and false for disable.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_EnableStatusCtrl
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            bool enable);
/*******************************************************************************
* Function name: Cy_USBD_ClearStatusClrBusy
****************************************************************************//**
*
* This function write "1" to STATUS_CLR_BUSY which clears the bit to initiate
* STATUS response.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_ClearStatusClrBusy
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_SendAckSetupDataStatusStage
****************************************************************************//**
*
* Allow chipset to send ACK to complete control transfer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_SendAckSetupDataStatusStage(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

#ifndef DOXYGEN
#define Cy_USBD_SendACkSetupDataStatusStage(pCtxt) Cy_USBD_SendAckSetupDataStatusStage(pCtxt)
#endif /* DOXYGEN */

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0Data
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0. Based on speed of device,
*  SS or HS function will be invoked.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0Data
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0DataSs
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0 of SS device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0DataSs
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0DataHs
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0 of HS device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0DataHs
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);

/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0Data
****************************************************************************//**
*
*  This function receives  data overthe DMA for endpoint 0. Based on speed of
*  device, SS or HS function will be invoked.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_RecvEndp0Data
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);
/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0DataSs
****************************************************************************//**
*
*  This function to  recieve data for endpoint 0 through DMA for SS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0DataSs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint8_t *pBuffer, uint32_t bufferSize);

/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0DataHs
****************************************************************************//**
*
*  This function to  recieve data for endpoint 0 through DMA for HS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0DataHs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint8_t *pBuffer, uint32_t bufferSize);


/*******************************************************************************
* Function name: Cy_USBD_IsEp0ReceiveDone
****************************************************************************//**
*
* This function checks whether a EP0 data receive operation is pending.
* Please note that the function will return true to indicate transfer done
* if there was no pending transfer in the first place.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* true if there is no pending EP0 OUT data transfer.
* false if there is a pending EP0 OUT data transfer.
*******************************************************************************/
bool Cy_USBD_IsEp0ReceiveDone(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_HandleGetDscr
****************************************************************************//**
*
*  This function handles all get descriptor command coming from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleGetDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq,
                                              cy_stc_usb_cal_msg_t *pMsg);
/*******************************************************************************
* Function name: Cy_USBD_HandleGetStatus
****************************************************************************//**
*
*  This function handles get status request coming from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleGetStatus
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq);

/*******************************************************************************
* Function name: Cy_USBD_HandleSetFeature
****************************************************************************//**
*
*  This function handles set-feature request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSetFeature
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleClearFeature
****************************************************************************//**
*
*  This function handles clear-feature request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleClearFeature
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleSetConfiguration
****************************************************************************//**
*
*  This function handles set configuration request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSetConfiguration
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_setup_req_t setupReq,
                                           cy_stc_usb_cal_msg_t *pMsg);
/*******************************************************************************
* Function name: Cy_USBD_HandleGetConfiguration
****************************************************************************//**
*
*  This function handles get configuration request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleGetConfiguration
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_stc_usb_setup_req_t setupReq);

/*******************************************************************************
* Function name: Cy_USBD_HandleSetInterface
****************************************************************************//**
*
*  This function handles set interface request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSetInterface
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleGetInterface
****************************************************************************//**
*
*  This function handles get interface request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleGetInterface
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq);

/*******************************************************************************
* Function name: Cy_USBD_HandleReset
****************************************************************************//**
*
*  This function handles bus reset.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleRxFailure
****************************************************************************//**
*
* Enables USB stack level work-around to attempt Gen2 connection start-up multiple
* times in case of training failure in Polling.Active or Polling.Config states.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param enable
* Whether work-around is to be enabled.
*
* \param retry_cnt
* Number of connection attempts to be made in case of Gen2 training failure.
*******************************************************************************/
void Cy_USBD_HandleRxFailure(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool enable, uint8_t retry_cnt);

/*******************************************************************************
* Function name: Cy_USBD_HandleSsReset
****************************************************************************//**
*
*  This function handles reset in SS mode.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSsReset(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleHsGrant
****************************************************************************//**
*
*  High speed related switching is handled here.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleHsGrant(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleResetDone
****************************************************************************//**
*
*  This function handles reset-done message.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleResetDone
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);
/*******************************************************************************
* Function name: Cy_USBD_HandleCtrlXfrSetupStage
****************************************************************************//**
*
*  This function handles setup stage of control transfer. It also prepare for
*  data/ack stage based on request.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleCtrlXfrSetupStage
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleStatusStage
****************************************************************************//**
*
*  This function handles status stage of control transfer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleStatusStage
                                        (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleSuspend
****************************************************************************//**
*
*  This function handles suspend functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSuspend(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleResume
****************************************************************************//**
*
*  This function handles resume functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleResume(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleL1Sleep
****************************************************************************//**
*
*  This function handles L1-Sleep functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleL1Sleep(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleL1Resume
****************************************************************************//**
*
*  This function handles L1-Resume functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleL1Resume(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleZlp
****************************************************************************//**
*
*  This function handles Zero-Length-Packet. This functionality applicable to HS
*  controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleZlp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleDone
****************************************************************************//**
*
*  This function handles done interrupt for an endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleDone(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleRateChange
****************************************************************************//**
*
*  This function handles rate change. This functionality is applicable to USB3
*  controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleRateChange(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleSlp
****************************************************************************//**
*
*  This function handles short length packet functionality. This functionality is
*  applicable to HS controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSlp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleSsDisconnect
****************************************************************************//**
*
*  This function handles disconnect event/message came from bottom layer.
*  Applicable to SS controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleSsDisconnect
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_HandleMsg
****************************************************************************//**
*
* This function handles all messages comes from bottom layer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_HandleMsg(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);

/*******************************************************************************
* Function name: Cy_USBD_InitUsbDscrPtrs
****************************************************************************//**
*
* This function initializes all descriptor pointers to NULL.
*
* \param pDscr
* pointer to data structure where all descriptors are stored.
*
*******************************************************************************/
void  Cy_USBD_InitUsbDscrPtrs(cy_stc_usb_set_dscr_ptrs_t *pDscr);

/*******************************************************************************
* Function name: Cy_USBD_ConnectHsDevice
****************************************************************************//**
*
* This functions connects HS device to BUS and make it visible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_ConnectHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_ConnectSsDevice
****************************************************************************//**
*
* This functions connects SS device to BUS and make it visible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_ConnectSsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_ConnectDevice
****************************************************************************//**
*
* This functions connects USB device with given speed and make it visible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param usbSpeed
* usb device speed which need to be configured in controller HW.
*
*******************************************************************************/
void Cy_USBD_ConnectDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_en_usb_speed_t usbSpeed);

/*******************************************************************************
* Function name: Cy_USBD_ResetUsbdCommonDs
****************************************************************************//**
*
* This functions reset all USBD common data structure.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_ResetUsbdCommonDs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_DisconnectHsDevice
****************************************************************************//**
*
* This functions dis-connects HS device from BUS and make it invisible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_DisconnectHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_DisconnectSsDevice
****************************************************************************//**
*
* This functions dis-connects SS device from BUS and make it invisible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_DisconnectSsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_DisconnectDevice
****************************************************************************//**
*
* Disconnect device from BUS. This function should be called by module/layer
* who detects vbus insertion/removal.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USBD_DisconnectDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/* Set of initialization functions related to EP0-DMA */

/*******************************************************************************
* Function name: Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig
****************************************************************************//**
*
*It initializes DMA descriptor for endpoint 0 IN transfer.
*
* \param pEndp0InCpuDmaDscrConfig
* dma descriptor configuration.
*
* \param first
* first or other descriptor.
*
*******************************************************************************/
void Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig(cy_stc_usbd_dma_descr_conf_t
                                             *pEndp0InCpuDmaDscrConfig,
                                             bool first);

/*******************************************************************************
* Function name: Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig
****************************************************************************//**
*
* It initializes DMA descriptor for EP0-OUT transfers.
*
* \param pEndp0OutdscrConfig
* dma descriptor configuration.
*
* \param first
* first or other descriptor.
*
*******************************************************************************/
void Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig(cy_stc_usbd_dma_descr_conf_t
                                              *pEndp0OutdscrConfig, bool first);

/*******************************************************************************
* Function name: Cy_USB_USBD_InitCpuDmaChannelCfg
****************************************************************************//**
*
* It initializes DMA channel config.
*
* \param pDmaChCfg
* dma channel configuration.
*
* \param pDmaDscr
* pointer to DMA descriptor.
*
*******************************************************************************/
void Cy_USB_USBD_InitCpuDmaChannelCfg(cy_stc_usbd_dma_chn_conf_t *pDmaChCfg,
                                            cy_stc_usbd_dma_descr_t *pDmaDscr);

/*******************************************************************************
* Function name: Cy_USB_USBD_cpuDmaInit
****************************************************************************//**
*
* It initializes Cpu DMA functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_cpuDmaInit(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);


/*******************************************************************************
* Function name: Cy_USB_USBD_EndpInit
****************************************************************************//**
*
* This function initializes endpoint 0 and other endpoints for HS and SS controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other case.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpInit(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USB_USBD_DisableHsDevice
****************************************************************************//**
*
* This function disable all HS device interrupt and make device invisible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USB_USBD_DisableHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USB_USBD_EnableHsDevice
****************************************************************************//**
*
* This function enable HS device and make it visible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USB_USBD_EnableHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_CtrlHSEnableOnCompliance
****************************************************************************//**
*
* Function used to enable/disable switch to USB-HS (2.0) mode on entry into
* the USB 3.x Compliance LTSSM state. This transition is left enabled on
* start-up and can be enabled through this API when device configurations
* that do not support USB-HS are being used.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param hsEnable
*  Whether transition into USB-HS is enabled.
*
*******************************************************************************/
void Cy_USBD_CtrlHSEnableOnCompliance(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                      bool hsEnable);

/*******************************************************************************
* Function name: Cy_USBD_TaskHandler
****************************************************************************//**
*
* This function handles msg coming from CAL layer.
*
* \param pTaskParam
* void pointer given to task handler.
*
*******************************************************************************/
void Cy_USBD_TaskHandler(void *pTaskParam);

/*******************************************************************************
* Function name: Cy_USBD_ProcessMsg
****************************************************************************//**
*
* This function sends msg to USBD thread.
*
* \param pUsbd
* void pointer carry USBD context.
*
* \param pCalMgs
* message prepared by CAL layer.
*
* \return
* Whether context switch is required at the end of the ISR which generated the
* message.
*
*******************************************************************************/
bool Cy_USBD_ProcessMsg(void *pUsbd, void *pCalMgs);

/*******************************************************************************
* Function name: Cy_USBD_SendEgressZLP
****************************************************************************//**
*
* This function sends ZLP for given endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNumber
* endpoint number.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SendEgressZLP(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            uint32_t endpNumber);

/*******************************************************************************
* Function name: Cy_USBD_ClearZlpSlpIntrEnableMask
****************************************************************************//**
*
* This function clears endpoint interrupt for ingrress and egresss and enables
* respective mask register.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param zlpSlp
* true for zlp and false for slp.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_ClearZlpSlpIntrEnableMask
                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                       cy_en_usb_endp_dir_t endpDir, bool zlpSlp);

/*******************************************************************************
* Function name: Cy_USB_LpmSetClearNYET
****************************************************************************//**
*
* This is a stack internal function used to enable/disable LPM support.
* Applications should always use Cy_USBD_LpmEnable and Cy_USBD_LpmDisable
* for this purpose.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* true for set and false for clear.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if status update failed.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_LpmSetClearNYET(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             bool setClear);

/*******************************************************************************
* Function name: Cy_USBD_GetUSBLinkActive
****************************************************************************//**
*
* This function ensures that the USB link is brought into the L0 (USB2) or
* U0 (USB3) state when the respective low power states.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_GetUSBLinkActive(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_LpmDisable
****************************************************************************//**
*
*  This function disables all further transitions into a USB low power
*  (U1, U2, or L1)  state.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_LpmDisable(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_LpmEnable
****************************************************************************//**
*
* This function re-enables state based transitions into USB low power
* (U1, U2 and L1) modes.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_LpmEnable(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0DataSs
****************************************************************************//**
*
*  This function will disable dma channel for SS which was submitted to recieve
*  data.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
*******************************************************************************/
void Cy_USB_USBD_RetireRecvEndp0DataSs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0DataHs
****************************************************************************//**
*
*  This function will disable dma channel for HS which was submitted to recieve
*  data.
*
* \param pUsbdCtxt
* Type of notification to be sent.
*
*******************************************************************************/
void Cy_USB_USBD_RetireRecvEndp0DataHs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0Data
****************************************************************************//**
*
*  This function will disable dma channel which was submitted to recieve data.
*  Common function for HS and SS.
*
* \param pUsbdCtxt
* Type of notification to be sent.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USB_USBD_RetireRecvEndp0Data(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function name: Cy_USBD_SendSSDeviceNotification
****************************************************************************//**
*
*  This function sends a Device Notification Transaction Packet to the host
*  controller on a USB 3.2 link.
*
* \param pUsbdCtxt
* Type of notification to be sent.
*
* \param type
* type of notification.
*
* \param param0
* 24-bits of data to be added in DWORD1 of the TP.
*
* \param param1
* Data to be added in DWORD2 of the TP.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if type is not valid.
*
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SendSSDeviceNotification
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_en_usbd_notification_type_t type,
                                           uint32_t param0, uint32_t param1);

/*******************************************************************************
 * Function Name: Cy_USBD_InitEventLog
 ****************************************************************************//**
 *
 * Enables capture of USB HS/SS driver events and state changes into RAM buffer.
 *
 * \param pUsbdCtxt
 * The pointer to the USBD context structure \ref cy_stc_usb_usbd_ctxt_t
 * allocated by the user.
 *
 * \param pEvtLogBuf
 * RAM buffer pointer to log the data into. Can be NULL if logging is to
 * be disabled.
 *
 * \param evtLogSize
 * Size of RAM buffer in 32-bit words.
 *
 * \return
 * The status code of the function execution \ref cy_en_usbd_ret_code_t.
 *******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_InitEventLog(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint32_t *pEvtLogBuf,
                                           uint16_t evtLogSize);

/*******************************************************************************
 * Function Name: Cy_USBD_AddEvtToLog
 ****************************************************************************//**
 *
 * Add USB event/state to the RAM log buffer.
 *
 * \param pUsbdCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usb_usbd_ctxt_t.
 *
 * \param evtId
 * Event ID to be logged.
 *
 *******************************************************************************/
void Cy_USBD_AddEvtToLog(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         uint32_t evtId);

/*******************************************************************************
 * Function Name: Cy_USBD_GetEvtLogIndex
 ****************************************************************************//**
 *
 * Returns the current USB driver event log index.
 *
 * \param pUsbdCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usb_usbd_ctxt_t.
 *
 * \return
 * Current event log index.
 *******************************************************************************/
uint16_t Cy_USBD_GetEvtLogIndex(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
 * Function Name: Cy_USBD_TickIncrement
 ****************************************************************************//**
 *
 * Increment the timer tick variable used to timestamp the USB event logs.
 * Should be called from SysTick interrupt handler.
 *
 * \param pUsbdCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usb_usbd_ctxt_t.
 *
 *******************************************************************************/
void Cy_USBD_TickIncrement(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
 * Function Name: Cy_USBD_GetTimerTick
 ****************************************************************************//**
 *
 * Retrieve the current value of the timer tick.
 *
 * \return
 * Current value of timestamp variable.
 *******************************************************************************/
uint32_t Cy_USBD_GetTimerTick(void);

/*******************************************************************************
 * Function Name: Cy_USBD_ResetTimerTick
 ****************************************************************************//**
 *
 * Clear the timer tick value maintained in the USB stack.
 *
 *******************************************************************************/
void Cy_USBD_ResetTimerTick(void);

/*******************************************************************************
* Function name: Cy_USBD_EP0OutDma_IntrHandler
****************************************************************************//**
*
* Handler for interrupt associated with EP0-OUT DMA transfer in USB 2.x modes.
* Should be called from the ISR registered for DMAC channel 1 which is used for
* EP0-OUT transfers.
*
* \param pUsbdCtxt
* USB stack context pointer.
*
*******************************************************************************/
void Cy_USBD_EP0OutDma_IntrHandler(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
 * Function Name: Cy_USBD_SetDmaClkFreq
 ****************************************************************************//**
 *
 * Function which sets the desired DMA clock frequency during USB 3.x operation
 * on the FX3G2 device. This function should be called before USB connection is
 * enabled. The default clock selected is 240 MHz derived from USB2 PLL.
 *
 * \param pUsbdCtxt
 * USB stack context pointer.
 *
 * \param dmaFreq
 * Desired DMA clock frequency.
 *
 * \return
 * Status code
 *******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SetDmaClkFreq(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_en_hbdma_clk_freq_t dmaFreq);


/*******************************************************************************
* Function Name: Cy_USBD_DisableLPMDeviceExit
****************************************************************************//**
*
* Function to disable support for device initiated exit from USB 3.x low power modes
* completely.
*
* @param pUsbdCtxt
* USB stack context pointer.
*
* @param devExitDisable
* true to disable device initiated exit from LPM states.
*******************************************************************************/
void
Cy_USBD_DisableLPMDeviceExit(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool devExitDisable);

/*******************************************************************************
* Function Name: Cy_USBD_DisableSSOnBusReset
****************************************************************************//**
*
* Function which disables the USB 3.x reconnection when USB 2.x Bus Reset
* is received. This function is only provided for testing of link robustness
* and not expected to be used in an application.
*
* \param pUsbdCtxt
* USB Stack context structure.
*******************************************************************************/
void Cy_USBD_DisableSSOnBusReset(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/*******************************************************************************
* Function Name: Cy_USBD_SelectConfigLane
****************************************************************************//**
*
* Function to select the USB 3.x Configuration Lane. This API can be used in cases
* where the USB connection orientation is being detected externally instead of
* through CC1/CC2 voltage measurement by the controller itself.
*
* \param pUsbdCtxt
* USB Stack context structure.
* \param laneSel
* Desired configuration lane selection.
*
* \return
* Whether the orientation selection has been registered successfully.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SelectConfigLane(
                                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                               cy_en_usb_config_lane_t laneSel);

/*******************************************************************************
 * Function Name: Cy_USBD_GetEndpointType
 ****************************************************************************//**
 *
 * Function to identify the type of endpoint based on its index.
 *
 * \param pUsbdCtxt
 * USB Stack context structure.
 * \param endpNumber
 * Endpoint index.
 * \param endpDirection
 * Endpoint direction.
 *
 * \return
 * Type of the endpoint.
 *******************************************************************************/
cy_en_usb_endp_type_t Cy_USBD_GetEndpointType(
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        uint32_t endpNumber,
        cy_en_usb_endp_dir_t endpDirection);

/** \} group_usbfxstack_usb_common_functions */

#if defined(__cplusplus)
}
#endif

#endif /* (CY_USB_USBD_H) */

/** \} group_usbfxstack_usb_common */

/* [EOF] */

