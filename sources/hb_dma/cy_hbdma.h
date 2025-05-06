/***************************************************************************//**
* \file cy_hbdma.h
* \version 1.00
*
* Provides common API declarations of the High BandWidth DMA driver.
*
********************************************************************************
* \copyright
* Copyright (2025) Cypress Semiconductor Corporation
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
 * \addtogroup group_usbfxstack_hb_dma
 * \{
 * HBDma is initialized and configured using the DMA manager, part of the usbfxstack middleware library.
 *
 * All data movement within the High BandWidth subsystem happens through temporary buffers located in the
 * buffer RAM. When a DMA datapath is being setup between the LVCMOS and USBHS interfaces, the firmware
 * needs to prepare RAM buffers which will be used for the transfers and configure a set of descriptors which track
 * the state of these RAM buffers. This is performed using a set of convenience API provided as part of the High
 * BandWidth DMA manager.
 * \defgroup group_usbfxstack_hb_dma_macros Macros
 * \defgroup group_usbfxstack_hb_dma_enums Enumerated Types
 * \defgroup group_usbfxstack_hb_dma_structs Data Structures
 * \defgroup group_usbfxstack_hb_dma_functions Functions
 * \defgroup group_usbfxstack_hb_dma_typedefs Type Definitions
 */

/**
 * \addtogroup group_usbfxstack_hb_dma_macros
 * \{
 */

#if !defined(CY_HBDMA_H)

/** Indicates the use of High-bandwidth DMA */
#define CY_HBDMA_H

#include "cy_device.h"
#include "ip/cyip_usb32dev.h"
#include "ip/cyip_lvdsss.h"

#if defined (CY_IP_MXS40LVDS2USB32SS)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "cy_syslib.h"
#include "cy_syspm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
*                        API Constants
*******************************************************************************/

/** Driver major version */
#define CY_HBDMA_DRV_VERSION_MAJOR      (1)

/** Driver minor version */
#define CY_HBDMA_DRV_VERSION_MINOR      (0)

/** HBDMA driver identifier */
#define CY_HBDMA_ID                     CY_PDL_DRV_ID(0x80U)

/** Base address of HBW SRAM. */
#define CY_HBW_SRAM_BASE_ADDR           (0x1C000000UL)

/** Upper limit of the HBW SRAM. */
#define CY_HBW_SRAM_LAST_ADDR           (0x1C100000UL)

/** Size of each High Bandwidth DMA descriptor. */
#define CY_HBDMA_DESC_SIZE              (0x10U)

/** Maximum number of descriptors supported. */
#define CY_HBDMA_MAX_DSCR_CNT           (4096U)

/** Get address of nth DMA descriptor. */
#define CY_HBDMA_GET_DESC_ADDR(dscr_no) ((CY_HBW_SRAM_BASE_ADDR) + ((dscr_no) * CY_HBDMA_DESC_SIZE))

/** Check whether socket id passed is valid. */
#define CY_HBDMA_IS_SOCKET_VALID(id) ( ((id) <= CY_HBDMA_LVDS_SOCKET_31) || \
                                       (((id) >= CY_HBDMA_USBIN_SOCKET_00) && ((id) <= CY_HBDMA_USBIN_SOCKET_15)) || \
                                       (((id) >= CY_HBDMA_USBEG_SOCKET_00) && ((id) <= CY_HBDMA_USBEG_SOCKET_15)) || \
                                       ((id) == CY_HBDMA_VIRT_SOCKET_WR) || \
                                       ((id) == CY_HBDMA_VIRT_SOCKET_RD) \
                                     )

/** Check whether the socket is a valid USB ingress socket. */
#define CY_HBDMA_IS_USB_IN_SOCK(id)  (((id) >= CY_HBDMA_USBIN_SOCKET_00) && ((id) <= CY_HBDMA_USBIN_SOCKET_15))

/** Check whether the socket is a valid USB egress socket. */
#define CY_HBDMA_IS_USB_EG_SOCK(id)  (((id) >= CY_HBDMA_USBEG_SOCKET_00) && ((id) <= CY_HBDMA_USBEG_SOCKET_15))

/** Mask for write-able fields of socket status. */
#define CY_HBDMA_SOCK_STATUS_WR_MASK    (0x2FE007FFUL)

/** Maximum number of sockets supported per DMA adapter. */
#define CY_HBDMA_SOCK_PER_ADAPTER       (16u)

/** Socket stall status value. */
#define CY_HBDMA_SOCK_STATE_STALL       (1u)

/** Socket active status value. */
#define CY_HBDMA_SOCK_STATE_ACTIVE      (2u)

#ifndef DOXYGEN

/** Compute Producer Socket encoding in the DSCR_SYNC field. */
#define CY_HBDMA_PROD_SOCK_TO_SYNC(sck_id)      \
    ((((sck_id) & 0x0FU) | (((sck_id) & 0xF0U) << 4U)) << 16U)

/** Compute Consumer Socket encoding in the DSCR_SYNC field. */
#define CY_HBDMA_CONS_SOCK_TO_SYNC(sck_id)      \
    (((sck_id) & 0x0FU) | (((sck_id) & 0xF0U) << 4U))

/** Retrieve producer socket ID from descriptor sync field. */
#define CY_HBDMA_SYNC_TO_PROD_SOCK(sync)        \
    (cy_hbdma_socket_id_t)((((sync) >> 16U) & 0x0FU) | (((sync) >> 20U) & 0xF0U))

/** Retrieve consumer socket ID from descriptor sync field. */
#define CY_HBDMA_SYNC_TO_CONS_SOCK(sync)        \
    (cy_hbdma_socket_id_t)(((sync) & 0x0FU) | (((sync) >> 4U) & 0xF0U))

/** Get next WR descriptor index from descriptor chain field. */
#define CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(chain)    \
    (uint16_t)(((chain) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Msk) >> \
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_WR_NEXT_DSCR_Pos)

/** Get next RD descriptor index from descriptor chain field. */
#define CY_HBDMA_CHAIN_TO_NEXT_RD_IDX(chain)    \
    (uint16_t)(((chain) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Msk) >> \
            LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_CHAIN_RD_NEXT_DSCR_Pos)

/** Get the socket current state from the status register. */
#define CY_HBDMA_STATUS_TO_SOCK_STATE(status)   \
    (((status) & LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE_Msk) >> LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE_Pos)

/** Get DMA buffer address from descriptor buffer register. */
#define CY_HBDMA_GET_BUFFER_ADDRESS(dscrBuffer) \
    ((uint8_t *)((uint32_t)(dscrBuffer) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_BUFFER_BUFFER_ADDR_Msk))

/** Check whether marker bit is set on descriptor. */
#define CY_HBDMA_IS_DESCRIPTOR_MARKED(dscrBuffer)       \
    ((((uint32_t)(dscrBuffer)) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_BUFFER_MARKER_Msk) != 0)

/** Update the marker bit in descriptor buffer pointer. */
#define CY_HBDMA_MARK_DSCR_BUFFER(dscrBuffer)   \
    ((uint8_t *)((uint32_t)(dscrBuffer) | LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_BUFFER_MARKER_Msk))

/** Get DMA buffer size field from the descriptor. */
#define CY_HBDMA_DSCR_GET_BUFSIZE(en_64k, dscrSize)                                             \
    (                                                                                           \
     ((en_64k) != 0) ?                                                                          \
        (((dscrSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk) << 1U) :          \
        ((dscrSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk)                    \
    )

/** Get byte count field from the descriptor. */
#define CY_HBDMA_DSCR_GET_BYTECNT(en_64k, dscrSize)                                             \
    (                                                                                           \
     ((en_64k) != 0) ?                                                                          \
        ((((dscrSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk) >>                \
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Pos) |                 \
         (((dscrSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_MSB_Msk) << 16U)       \
        ) :                                                                                     \
        (((dscrSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk) >>                 \
                       LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Pos)                    \
    )

/** Set the DMA buffer size field in the descriptor. */
#define CY_HBDMA_DSCR_SET_BUFSIZE(en_64k, dscrSize, bufSize)                                    \
    (                                                                                           \
     ((en_64k) != 0) ?                                                                          \
        (((dscrSize) & ~LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk) |                \
         (((bufSize) >> 1U) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk)            \
        ) :                                                                                     \
        (((dscrSize) & ~LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk) |                \
         ((bufSize) & LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BUFFER_SIZE_Msk))                   \
    )

/** Update DMA buffer size by rounding up the byte count value. */
#define CY_HBDMA_DSCR_SET_SIZE_BY_COUNT(en_64k, dscrSize, byteCnt)                              \
    (                                                                                           \
     ((en_64k) != 0) ?                                                                          \
        (CY_HBDMA_DSCR_SET_BUFSIZE((en_64k), (dscrSize), ((byteCnt) + 0x1FU))) :                \
        (CY_HBDMA_DSCR_SET_BUFSIZE((en_64k), (dscrSize), ((byteCnt) + 0x0FU)))                  \
    )

/** Set the byte count field in the descriptor. */
#define CY_HBDMA_DSCR_SET_BYTECNT(en_64k, dscrSize, byteCnt)                                    \
    (                                                                                           \
     ((en_64k) != 0) ?                                                                          \
        (((dscrSize) & ~LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk) |                 \
         (((byteCnt) << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Pos) &                 \
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk) |                 \
         ((byteCnt) >> 16U)) :                                                                  \
        (((dscrSize) & ~LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk) |                 \
         (((byteCnt) << LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Pos) &                 \
                        LVDSSS_LVDS_ADAPTER_DMA_SCK_DSCR_SIZE_BYTE_COUNT_Msk))                  \
    )

#endif /* DOXYGEN */

/** \} group_usbfxstack_hb_dma_macros */

/**
 * \addtogroup group_usbfxstack_hb_dma_enums
 * \{
 */

/*******************************************************************************
*                         Enumerated Data Types
*******************************************************************************/

/**
 * @typedef cy_hbdma_socket_id_t
 * @brief List of High BandWidth DMA sockets supported in the platform.
 */
typedef enum
{
    CY_HBDMA_LVDS_SOCKET_00 = 0x10,     /**< LVDS socket #00 */
    CY_HBDMA_LVDS_SOCKET_01,            /**< LVDS socket #01 */
    CY_HBDMA_LVDS_SOCKET_02,            /**< LVDS socket #02 */
    CY_HBDMA_LVDS_SOCKET_03,            /**< LVDS socket #03 */
    CY_HBDMA_LVDS_SOCKET_04,            /**< LVDS socket #04 */
    CY_HBDMA_LVDS_SOCKET_05,            /**< LVDS socket #05 */
    CY_HBDMA_LVDS_SOCKET_06,            /**< LVDS socket #06 */
    CY_HBDMA_LVDS_SOCKET_07,            /**< LVDS socket #07 */
    CY_HBDMA_LVDS_SOCKET_08,            /**< LVDS socket #08 */
    CY_HBDMA_LVDS_SOCKET_09,            /**< LVDS socket #09 */
    CY_HBDMA_LVDS_SOCKET_10,            /**< LVDS socket #10 */
    CY_HBDMA_LVDS_SOCKET_11,            /**< LVDS socket #11 */
    CY_HBDMA_LVDS_SOCKET_12,            /**< LVDS socket #12 */
    CY_HBDMA_LVDS_SOCKET_13,            /**< LVDS socket #13 */
    CY_HBDMA_LVDS_SOCKET_14,            /**< LVDS socket #14 */
    CY_HBDMA_LVDS_SOCKET_15,            /**< LVDS socket #15 */

    CY_HBDMA_LVDS_SOCKET_16 = 0x20U,    /**< LVDS socket #16 */
    CY_HBDMA_LVDS_SOCKET_17,            /**< LVDS socket #17 */
    CY_HBDMA_LVDS_SOCKET_18,            /**< LVDS socket #18 */
    CY_HBDMA_LVDS_SOCKET_19,            /**< LVDS socket #19 */
    CY_HBDMA_LVDS_SOCKET_20,            /**< LVDS socket #20 */
    CY_HBDMA_LVDS_SOCKET_21,            /**< LVDS socket #21 */
    CY_HBDMA_LVDS_SOCKET_22,            /**< LVDS socket #22 */
    CY_HBDMA_LVDS_SOCKET_23,            /**< LVDS socket #23 */
    CY_HBDMA_LVDS_SOCKET_24,            /**< LVDS socket #24 */
    CY_HBDMA_LVDS_SOCKET_25,            /**< LVDS socket #25 */
    CY_HBDMA_LVDS_SOCKET_26,            /**< LVDS socket #26 */
    CY_HBDMA_LVDS_SOCKET_27,            /**< LVDS socket #27 */
    CY_HBDMA_LVDS_SOCKET_28,            /**< LVDS socket #28 */
    CY_HBDMA_LVDS_SOCKET_29,            /**< LVDS socket #29 */
    CY_HBDMA_LVDS_SOCKET_30,            /**< LVDS socket #30 */
    CY_HBDMA_LVDS_SOCKET_31,            /**< LVDS socket #31 */

    CY_HBDMA_USBEG_SOCKET_00 = 0x30U,   /**< USB egress socket #00 */
    CY_HBDMA_USBEG_SOCKET_01,           /**< USB egress socket #01 */
    CY_HBDMA_USBEG_SOCKET_02,           /**< USB egress socket #02 */
    CY_HBDMA_USBEG_SOCKET_03,           /**< USB egress socket #03 */
    CY_HBDMA_USBEG_SOCKET_04,           /**< USB egress socket #04 */
    CY_HBDMA_USBEG_SOCKET_05,           /**< USB egress socket #05 */
    CY_HBDMA_USBEG_SOCKET_06,           /**< USB egress socket #06 */
    CY_HBDMA_USBEG_SOCKET_07,           /**< USB egress socket #07 */
    CY_HBDMA_USBEG_SOCKET_08,           /**< USB egress socket #08 */
    CY_HBDMA_USBEG_SOCKET_09,           /**< USB egress socket #09 */
    CY_HBDMA_USBEG_SOCKET_10,           /**< USB egress socket #10 */
    CY_HBDMA_USBEG_SOCKET_11,           /**< USB egress socket #11 */
    CY_HBDMA_USBEG_SOCKET_12,           /**< USB egress socket #12 */
    CY_HBDMA_USBEG_SOCKET_13,           /**< USB egress socket #13 */
    CY_HBDMA_USBEG_SOCKET_14,           /**< USB egress socket #14 */
    CY_HBDMA_USBEG_SOCKET_15,           /**< USB egress socket #15 */

    CY_HBDMA_USBIN_SOCKET_00 = 0x40U,   /**< USB ingress socket #00 */
    CY_HBDMA_USBIN_SOCKET_01,           /**< USB ingress socket #01 */
    CY_HBDMA_USBIN_SOCKET_02,           /**< USB ingress socket #02 */
    CY_HBDMA_USBIN_SOCKET_03,           /**< USB ingress socket #03 */
    CY_HBDMA_USBIN_SOCKET_04,           /**< USB ingress socket #04 */
    CY_HBDMA_USBIN_SOCKET_05,           /**< USB ingress socket #05 */
    CY_HBDMA_USBIN_SOCKET_06,           /**< USB ingress socket #06 */
    CY_HBDMA_USBIN_SOCKET_07,           /**< USB ingress socket #07 */
    CY_HBDMA_USBIN_SOCKET_08,           /**< USB ingress socket #08 */
    CY_HBDMA_USBIN_SOCKET_09,           /**< USB ingress socket #09 */
    CY_HBDMA_USBIN_SOCKET_10,           /**< USB ingress socket #10 */
    CY_HBDMA_USBIN_SOCKET_11,           /**< USB ingress socket #11 */
    CY_HBDMA_USBIN_SOCKET_12,           /**< USB ingress socket #12 */
    CY_HBDMA_USBIN_SOCKET_13,           /**< USB ingress socket #13 */
    CY_HBDMA_USBIN_SOCKET_14,           /**< USB ingress socket #14 */
    CY_HBDMA_USBIN_SOCKET_15,           /**< USB ingress socket #15 */

    CY_HBDMA_VIRT_SOCKET_RD,            /**< Virtual (CPU) access socket for data read. */
    CY_HBDMA_VIRT_SOCKET_WR             /**< Virtual (CPU) access socket for data write. */
} cy_hbdma_socket_id_t;

/**
 * @typedef cy_hbdma_adapter_id_t
 * @brief List of High BandWidth DMA adapters supported in the platform.
 */
typedef enum
{
    CY_HBDMA_ADAP_LVDS_0 = 1,           /**< LVDS DMA adapter #0. Handles sockets CY_HBDMA_LVDS_SOCKET_00 to
                                             CY_HBDMA_LVDS_SOCKET_15 */
    CY_HBDMA_ADAP_LVDS_1,               /**< LVDS DMA adapter #1. Handles sockets CY_HBDMA_LVDS_SOCKET_16 to
                                             CY_HBDMA_LVDS_SOCKET_31 */
    CY_HBDMA_ADAP_USB_IN,               /**< USB32 Ingress DMA adapter. Handles sockets CY_HBDMA_USBIN_SOCKET_00
                                             to CY_HBDMA_USBIN_SOCKET_15 */
    CY_HBDMA_ADAP_USB_EG                /**< USB32 Egress DMA adapter. Handles sockets CY_HBDMA_USBEG_SOCKET_00
                                             to CY_HBDMA_USBEG_SOCKET_15 */
} cy_hbdma_adapter_id_t;

/**
 * @typedef cy_en_hbdma_status_t
 * @brief List of return values provided by HBW DMA driver API.
 */
typedef enum
{
    CY_HBDMA_SUCCESS = 0U,                                              /**< Operation completed successfully. */
    CY_HBDMA_BAD_PARAM = (CY_HBDMA_ID | CY_PDL_STATUS_ERROR | 1U),      /**< Bad API parameters. */
    CY_HBDMA_NOT_READY = (CY_HBDMA_ID | CY_PDL_STATUS_ERROR | 2U)       /**< DMA adapter not ready. */
} cy_en_hbdma_status_t;

/**
 * @typedef cy_en_hbdma_sock_evt_t
 * @brief List of interrupt callback types reported by the HBW DMA driver to the DMA manager
 * middleware.
 */
typedef enum
{
    CY_HBDMA_SOCK_PRODUCE_EVT = 0,      /**< Produce event interrupt received. */
    CY_HBDMA_SOCK_CONSUME_EVT,          /**< Consume event interrupt received. */
    CY_HBDMA_SOCK_STALL_EVT,            /**< Socket stalled interrupt received. */
    CY_HBDMA_SOCK_SUSPEND_EVT,          /**< Socket suspended interrupt received. */
    CY_HBDMA_SOCK_ERROR_EVT,            /**< Socket error interrupt received. */
    CY_HBDMA_SOCK_XFERDONE_EVT,         /**< Transfer done interrupt received. */
    CY_HBDMA_SOCK_EVT_RCVD              /**< Event received interrupt: Identifies trigger coming from Central DMA. */
} cy_en_hbdma_sock_evt_t;

/**
 * @typedef cy_en_hbdma_clk_freq_t
 * @brief List of High BandWidth DMA clock frequencies supported on the device
 * family.
 */
typedef enum
{
    CY_HBDMA_CLK_150_MHZ = 0,           /**< 150 MHz clock derived from system level clk_hf4 input. */
    CY_HBDMA_CLK_160_MHZ,               /**< 160 MHz clock divided from 480 MHz provided by USB2 PLL. */
    CY_HBDMA_CLK_240_MHZ,               /**< 240 MHz clock divided from 480 MHz provided by USB2 PLL. */
    CY_HBDMA_CLK_SSPHY_CLK              /**< Clock derived from USB3 PHY. 125 MHz in Gen1 operation and
                                             312.5 MHz in Gen2 operation. */
} cy_en_hbdma_clk_freq_t;

/** \} group_usbfxstack_hb_dma_enums */

/**
 * \addtogroup group_usbfxstack_hb_dma_typedefs
 * \{
 */

/*******************************************************************************
*                              Type Definitions
*******************************************************************************/

/**
 * High BandWidth DMA socket interrupt callback prototype.
 * This is an internal callback function used by the socket ISRs to send
 * information to the DMA manager for processing.
 *
 * \return
 * Indicates whether further interrupt processing is to be paused to avoid
 * over-running the interrupt queue.
 */
typedef bool (*cy_cb_hbdma_intr_callback_t) (
        cy_hbdma_socket_id_t socketId,          /**< Socket on which the interrupt was received. */
        cy_en_hbdma_sock_evt_t intrType,        /**< Type of interrupt received. */
        uint32_t curDscr,                       /**< Current descriptor on the socket after the
                                                    interrupt has been received. */
        void *userCtx                           /**< Opaque variable used to pass the DMA manager
                                                    context structure pointer. */
        );

/** \} group_usbfxstack_hb_dma_typedefs */

/**
 * \addtogroup group_usbfxstack_hb_dma_structs
 * \{
 */

/*******************************************************************************
*                            Data Structures
*******************************************************************************/

#ifndef DOXYGEN

/**
 * @brief Structure representing the control registers for a High BandWidth DMA socket.
 * This is a copy of the relevant register groups maintained in the USB32DEV
 * and LVDS IP blocks.
 */
typedef struct {
  __IOM uint32_t SCK_DSCR;              /** Descriptor Chain Pointer (offset 0x00000000) */
  __IOM uint32_t SCK_SIZE;              /** Transfer Size Register (offset 0x00000004) */
  __IOM uint32_t SCK_COUNT;             /** Transfer Count Register (offset 0x00000008) */
  __IOM uint32_t SCK_STATUS;            /** Socket Status Register (offset 0x0000000C) */
  __IOM uint32_t SCK_INTR;              /** Socket Interrupt Request Register (offset 0x00000010) */
  __IOM uint32_t SCK_INTR_MASK;         /** Socket Interrupt Mask Register (offset 0x00000014) */
   __IM uint32_t RESERVED[2];
  __IOM uint32_t DSCR_BUFFER;           /** Descriptor buffer base address register (offset 0x00000020) */
  __IOM uint32_t DSCR_SYNC;             /** Descriptor synchronization pointers register (offset 0x00000024) */
  __IOM uint32_t DSCR_CHAIN;            /** Descriptor chain pointers register (offset 0x00000028) */
  __IOM uint32_t DSCR_SIZE;             /** Descriptor size register (offset 0x0000002C) */
   __IM uint32_t RESERVED1[19];
   __OM uint32_t EVENT;                 /** Event communication register (offset 0x0000007C) */
} HBDMA_SCK_Type;

/**
 * @brief Structure representing the control registers for a High BandWidth DMA adapter.
 * This is a copy of the relevant register groups maintained in the USB32DEV
 * and LVDS IP blocks.
 */
typedef struct {
   __IM uint32_t SCK_INTR;              /** Socket interrupt request register (offset 0x00000000) */
   __IM uint32_t RESERVED[59];
  __IOM uint32_t ADAPTER_CTRL;          /** Adapter control register (offset 0x000000F0) */
   __IM uint32_t ADAPTER_DEBUG;         /** Adapter debug observation register (offset 0x000000F4) */
  __IOM uint32_t ADAPTER_CONF;          /** Adapter configuration register (offset 0x000000F8) */
   __IM uint32_t ADAPTER_STATUS;        /** Adapter global status fields (offset 0x000000FC) */
} HBDMA_SCK_GBL_Type;

#endif /* DOXYGEN */

/**
 * @brief Structure encapsulating the context information for the High BandWidth DMA driver.
 * The driver provides functions to initialize the DMA adapters and sockets as well as
 * to handle interrupts raised by any of the adapters.
 *
 * @note The High BandWidth DMA register structures linked in the context structure are
 * specific to the product family.
 */
typedef struct {
    HBDMA_SCK_Type           *LVDSAD0_SCK;      /**< Pointer to LVDS DMA Adapter 0 Socket array. */
    HBDMA_SCK_GBL_Type       *LVDSAD0_SCK_GBL;  /**< Pointer to LVDS DMA Adapter 0 SCK_GBL structure. */
    HBDMA_SCK_Type           *LVDSAD1_SCK;      /**< Pointer to LVDS DMA Adapter 1 Socket array. */
    HBDMA_SCK_GBL_Type       *LVDSAD1_SCK_GBL;  /**< Pointer to LVDS DMA Adapter 1 SCK_GBL structure. */
    HBDMA_SCK_Type           *USBIN_SCK;        /**< Pointer to USB Ingress Socket array. */
    HBDMA_SCK_GBL_Type       *USBIN_SCK_GBL;    /**< Pointer to USB Ingress SCK_GBL structure. */
    HBDMA_SCK_Type           *USBEG_SCK;        /**< Pointer to USB Egress Socket array. */
    HBDMA_SCK_GBL_Type       *USBEG_SCK_GBL;    /**< Pointer to USB Egress SCK_GBL structure. */
    cy_cb_hbdma_intr_callback_t intrCallback;   /**< Callback function pointer for interrupt notification. */
    void                     *cbContext;        /**< Caller context to be passed to the callback function. */
    bool                      usb3Enabled;      /**< Whether USB32DEV adapter is supported. */
    bool                      lvdsEnabled;      /**< Whether LVDS adapter is supported. */
} cy_stc_hbdma_context_t;

/**
 * @brief Structure representing the High BandWidth DMA descriptor structure which is
 * maintained in the device RAM memory. A copy of the active descriptor being worked
 * on by each of the sockets is maintained in the socket register table itself.
 */
typedef struct cy_stc_hbdma_desc_t
{
    uint8_t    *pBuffer;                        /**< Pointer to RAM buffer used. */
    uint32_t    sync;                           /**< Points to the producer and consumer sockets. */
    uint32_t    chain;                          /**< Next descriptor links. */
    uint32_t    size;                           /**< Buffer size and current data count. */
} cy_stc_hbdma_desc_t;

/**
 * @brief Structure representing the control and status registers associated with
 * a High BandWidth DMA socket. This includes the socket specific registers as well
 * as the copy of the active DMA descriptor which is maintained for each socket.
 */
typedef struct cy_stc_hbdma_sock_t
{
    uint32_t actDscrIndex;                      /**< Index of active descriptor. */
    uint32_t reqXferSize;                       /**< The transfer size requested for this socket. */
    uint32_t compXferCount;                     /**< The completed transfer count for this socket. */
    uint32_t status;                            /**< Socket configuration and status register. */
    uint32_t intrStatus;                        /**< Interrupt status register. */
    uint32_t intrMask;                          /**< Interrupt mask register. */
    cy_stc_hbdma_desc_t actDscr;                /**< Copy of active descriptor. */
    uint32_t sckEvent;                          /**< Register to generate DMA event triggers. */
} cy_stc_hbdma_sock_t;

/**
 * @brief Structure used to provide the configuration settings associated with a
 * High BandWidth DMA socket.
 */
typedef struct cy_stc_hbdma_sockconfig_t
{
    uint32_t actDscrIndex;                      /**< Index of first descriptor to be used. */
    uint32_t reqXferSize;                       /**< The transfer size requested for this socket. */
    uint32_t intrMask;                          /**< Interrupt mask register. */
    uint32_t status;                            /**< Socket status to be set. */
} cy_stc_hbdma_sockconfig_t;

/** \} group_usbfxstack_hb_dma_structs */

/**
 * \addtogroup group_usbfxstack_hb_dma_functions
 * \{
 */

/*******************************************************************************
*                            Function Prototypes
*******************************************************************************/

/*******************************************************************************
 * Function name: Cy_HBDma_Init
 ****************************************************************************//**
 *
 * Initializes the High BandWidth DMA driver. Pointers to the USB32DEV and
 * LVDS control register blocks need to be passed as parameters while initializing
 * the driver.
 *
 * \param lvds_base
 * Pointer to the LVDS IP control registers.
 *
 * \param usbss_base
 * Pointer to the USB32DEV IP control registers.
 *
 * \param pContext
 * Pointer to the HBDma driver context structure.
 *
 * \param usb_egrs_fq_depth
 * Fetch queue depth setting to be set for the USB32DEV egress DMA adapter.
 *
 * \param usb_egrs_rq_ctrl
 * Read queue empty threshold to be set for the USB32DEV egress DMA adapter.
 *
 * \return
 * CY_HBDMA_SUCCESS if driver initialization is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_Init(
        LVDSSS_LVDS_Type *lvds_base,
        USB32DEV_Type *usbss_base,
        cy_stc_hbdma_context_t *pContext,
        uint32_t usb_egrs_fq_depth,
        uint32_t usb_egrs_rq_ctrl
        );

/*******************************************************************************
 * Function name: Cy_HBDma_DeInit
 ****************************************************************************//**
 *
 * De-initialize the High BandWidth DMA adapter. This function also disables
 * the DMA adapters associated with the LVDS and USB32DEV IP blocks.
 *
 * \param pContext
 * Pointer to the HBDma driver context structure.
 *******************************************************************************/
void
Cy_HBDma_DeInit(
        cy_stc_hbdma_context_t *pContext
        );

/*******************************************************************************
 * Function name: Cy_HBDma_GetDescriptor
 ****************************************************************************//**
 *
 * Reads and returns the contents of the High BandWidth DMA descriptor with the
 * specified index.
 *
 * \param dscrIndex
 * Index of the DMA descriptor to be fetched.
 *
 * \param dscr_p
 * Pointer to structure to be filled with the descriptor information.
 *
 * \return
 * CY_HBDMA_SUCCESS if descriptor fetch is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_GetDescriptor(
        uint16_t dscrIndex,
        cy_stc_hbdma_desc_t *dscr_p
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SetDescriptor
 ****************************************************************************//**
 *
 * Updates the High BandWidth DMA descriptor at a specified index with the
 * desired values.
 *
 * \param dscrIndex
 * DMA descriptor index.
 *
 * \param dscr_p
 * Structure containing the values to be updated in the DMA descriptor.
 *
 * \return
 * CY_HBDMA_SUCCESS if descriptor update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetDescriptor(
        uint16_t dscrIndex,
        cy_stc_hbdma_desc_t *dscr_p
        );

/*******************************************************************************
 * Function name: Cy_HBDma_GetSocketStatus
 ****************************************************************************//**
 *
 * Read and return the contents of all control and status registers associated
 * with a High BandWidth DMA socket. This function can be used to check the
 * current socket status, and also as the first step during a read-modify-write
 * update of the socket configuration.
 *
 * \param pContext
 * Pointer to the driver context structure.
 *
 * \param sock_id
 * ID of the socket whose status is to be retrieved.
 *
 * \param sckConf_p
 * Pointer to structure to be filled with the socket status.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket status read is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_GetSocketStatus(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        cy_stc_hbdma_sock_t *sckConf_p
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SetSocketConfig
 ****************************************************************************//**
 *
 * Updates the configuration of a HBDma socket with the desired fields from the
 * config structure passed in as parameter. This API can be used to update the
 * current descriptor, the interrupt masks and the various config bits associated
 * with the socket functionality.
 *
 * \param pContext
 * Pointer to the driver context structure.
 *
 * \param sock_id
 * ID of the socket whose configuration is to be updated.
 *
 * \param conf
 * Pointer to the desired socket configuration.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetSocketConfig(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        cy_stc_hbdma_sockconfig_t *conf
        );

/*******************************************************************************
 * Function name: Cy_HBDma_UpdateSockIntrMask
 ****************************************************************************//**
 *
 * Special function provided to update only the interrupt mask associated
 * with a HBDma socket. This is a subset of the functionality provided by the
 * Cy_HBDma_SetSocketConfig API.
 *
 * \param pContext
 * Pointer to the HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be updated.
 *
 * \param intrMap
 * Bit map specifying the interrupt mask bits to be updated.
 *
 * \param enable
 * Whether the interrupts specified in intrMap are to be enabled or disabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_UpdateSockIntrMask(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        uint32_t intrMap,
        bool enable
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SocketEnable
 ****************************************************************************//**
 *
 * Function to enable a High BandWidth DMA socket after it has been configured
 * using the Cy_HBDma_SetSocketConfig API. If the socket has not been configured
 * properly before it is enabled, it can result in unexpected transfers or
 * errors.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be enabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketEnable(
        cy_stc_hbdma_context_t *pContext,      /* DMA driver context pointer. */
        cy_hbdma_socket_id_t sock_id            /* Id of the socket to be enabled. */
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SocketDisable
 ****************************************************************************//**
 *
 * Function to disable a High BandWidth DMA socket. This function causes any
 * ongoing DMA transfer on the socket to be aborted and blocks until the socket
 * has moved into the disabled state.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be disabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketDisable(
        cy_stc_hbdma_context_t *pContext,      /* DMA driver context pointer. */
        cy_hbdma_socket_id_t sock_id            /* Id of the socket to be disabled. */
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SocketSetWrapUp
 ****************************************************************************//**
 *
 * This function sets the bit that forces a socket with a partially filled buffer
 * to wrap up the buffer. The function does not wait for the socket to wrap up.
 * This API should be called after ensuring that the socket in question is not
 * actively receiving data. Otherwise, this can result in data loss.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be wraped up.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketSetWrapUp (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id);

/*******************************************************************************
 * Function name: Cy_HBDma_SendSocketEvent
 ****************************************************************************//**
 *
 * Function to send an event notification to the specified socket. Either a
 * produce event or a consume event can be sent based on the use case and type
 * of the socket.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to which the event is to be sent.
 *
 * \param isProduceEvent
 * Set to true for sending a produce event, false for consume event.
 *
 * \return
 * CY_HBDMA_SUCCESS if sending the event is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SendSocketEvent(
        cy_stc_hbdma_context_t *pContext,        /* DMA driver context pointer. */
        cy_hbdma_socket_id_t sock_id,           /* Id of the socket to be enabled. */
        bool isProduceEvent                     /* Type of event to be sent: Produce(1) or Consume(2) */
        );

/*******************************************************************************
 * Function name: Cy_HbDma_ConnectEventTrigger
 ****************************************************************************//**
 *
 * Function to connect the event trigger output from one socket to the input of
 * another socket. The same mechanism is used for sending produce events from an
 * ingress socket to an egress socket as well as for sending consume events from
 * an egress socket to an ingress socket.
 *
 * If the DMA configuration used is a 1:2 or a 2:1 channel, trigger outputs from
 * multiple sockets can be connected to the input of one socket. The trigNum
 * parameter is used to specify the index of the event trigger input on the
 * destination side.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param src_sock
 * ID of the socket which generates the produce or consume event trigger.
 *
 * \param dst_sock
 * ID of the socket which receives the event trigger.
 *
 * \param trigNum
 * Index of the trigger input on the destination socket. Can be 0 or 1.
 *
 * \return
 * CY_HBDMA_SUCCESS if the trigger connection is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HbDma_ConnectEventTrigger(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t src_sock,
        cy_hbdma_socket_id_t dst_sock,
        uint8_t              trigNum
        );

/*******************************************************************************
 * Function name: Cy_HbDma_DisconnectEventTriggers
 ****************************************************************************//**
 *
 * Function to break all event trigger connections coming to a socket.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param dst_sock
 * ID of the socket which receives the event trigger.
 *
 * \return
 * CY_HBDMA_SUCCESS if the operation is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HbDma_DisconnectEventTriggers(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t dst_sock);

/*******************************************************************************
 * Function name: Cy_HBDma_HandleInterrupts
 ****************************************************************************//**
 *
 * Function which handles the interrupts generated by any of the sockets associated
 * with a High BandWidth DMA adapter. This function is expected to be called from
 * the ISR for the corresponding interrupt vector.
 *
 * The interrupt handling in the driver is only a top-half implementation which
 * notifies the HBDma manager layer and then clears the interrupt.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param adapter
 * Identifies the adapter for which the interrupts are to be serviced.
 *
 * \return
 * CY_HBDMA_SUCCESS if the adapter specified is valid.
 * CY_HBDMA_BAD_PARAM if invalid parameters are passed in.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_HandleInterrupts(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_adapter_id_t adapter
        );

/*******************************************************************************
 * Function name: Cy_HBDma_SetInterruptCallback
 ****************************************************************************//**
 *
 * Function that registers a callback which can be called the HBDma driver to
 * provide notification of socket interrupts. The callback will be registered by
 * the HBDma manager layer and will be called when the socket interrupts are
 * serviced through the Cy_HBDma_HandleInterrupts function.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param cb_p
 * Callback function pointer.
 *
 * \param cbContext
 * Opaque user data to be passed to the callback function.
 *
 * \return
 * CY_HBDMA_SUCCESS if callback registration is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetInterruptCallback(
        cy_stc_hbdma_context_t *pContext,
        cy_cb_hbdma_intr_callback_t cb_p,
        void *cbContext
        );

/*******************************************************************************
 * Function name: Cy_HBDma_EvictReadCache
 ****************************************************************************//**
 *
 * Evicts the read cache used while masters are using either the Fast AHB
 * bus (Cortex-M4) or the Slow AHB bus (Cortex-M0+, DataWire, DMAC or Crypto)
 * to read content from the High BandWidth RAM area. This operation is required
 * whenever a master is reading from the RAM while there is a possibility of
 * the corresponding memory having been modified by DMA operations.
 *
 * \param isCm4Access
 * Whether the read cache used by Cortex-M4 is to be cleared. Set to false
 * for all other masters.
 *
 *******************************************************************************/
void Cy_HBDma_EvictReadCache(
        bool isCm4Access);

/*******************************************************************************
 * Function name: Cy_HBDma_SetClockFrequency
 ****************************************************************************//**
 *
 * Sets the high bandwidth DMA clock frequency to the desired value. Please note
 * that parameters in the USB block need to be set based on the operating USB
 * speed and selected clock frequency. Hence, this function is expected to be
 * called from the USB stack and not expected to be called directly.
 *
 * \param dmaFreq
 * Desired DMA domain clock frequency.
 *
 * \return
 * CY_HBDMA_SUCCESS if clock setup is successful
 * CY_HBDMA_BAD_PARAM if the parameter passed in is invalid.
 *******************************************************************************/
cy_en_hbdma_status_t Cy_HBDma_SetClockFrequency(
        cy_en_hbdma_clk_freq_t dmaFreq);

/*******************************************************************************
 * Function name: Cy_HBDma_Is64KBufferEnabled
 ****************************************************************************//**
 *
 * Check whether 64KB DMA buffer support is enabled on the device. This feature
 * is not supported on A0 silicon revision of FX10.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \return
 * true if 64KB DMA buffers are supported.
 * false if 64KB DMA buffers are not supported.
 *******************************************************************************/
bool Cy_HBDma_Is64KBufferEnabled(
        cy_stc_hbdma_context_t *pDrvContext);

/*******************************************************************************
 * Function name: Cy_HBDma_SetUsbEgressAdapterDelay
 ****************************************************************************//**
 *
 * Update the number of cycles of delay to be applied between consecutive AXI
 * data fetches made by the USB egress DMA adapter. The function is meant to
 * be used by the USB stack based on current USB speed.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \param gblDelayCycles
 * Number of delay cycles to be applied in the range of 0 to 15.
 *
 *******************************************************************************/
void Cy_HBDma_SetUsbEgressAdapterDelay(
        cy_stc_hbdma_context_t *pDrvContext,
        uint8_t gblDelayCycles);

/*******************************************************************************
 * Function name: Cy_HBDma_SetLvdsAdapterIngressMode
 ****************************************************************************//**
 *
 * This function enables or disables the support for egress data transfers from
 * RAM buffers which are not 16-byte aligned based on whether the specified
 * LVDS DMA adapters are working in Ingress only mode or not. For any ingress-only
 * adapter, this support can be disabled to gain better DMA performance.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \param isAdap0Ingress
 * Whether adapter 0 (sockets 0 to 15) is being used only in ingress direction.
 *
 * \param isAdap1Ingress
 * Whether adapter 1 (sockets 16 to 31) is being used only in ingress direction.
 *
 *******************************************************************************/
void Cy_HBDma_SetLvdsAdapterIngressMode(
        cy_stc_hbdma_context_t *pDrvContext,
        bool isAdap0Ingress,
        bool isAdap1Ingress);

/** \} group_usbfxstack_hb_dma_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS */

#endif /* (CY_HBDMA_H) */

/** \} group_usbfxstack_hb_dma */

/* [] END OF FILE */

