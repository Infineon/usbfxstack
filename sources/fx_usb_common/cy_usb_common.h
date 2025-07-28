/***************************************************************************//**
* \file cy_usb_common.h
* \version 1.0
*
* Provides common definitions used in the USB Device Stack.
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
 * FX USB Common, part of the usbfxstack library provides drivers for various operation modes, ports and other components of the FX devices.
 * \defgroup group_usbfxstack_usb_common_macros Macros
 * \defgroup group_usbfxstack_usb_common_enums Enumerated Types
 * \defgroup group_usbfxstack_usb_common_structs Data Structures
 * \defgroup group_usbfxstack_usb_common_typedefs Type Definitions
 * \defgroup group_usbfxstack_usb_common_functions Functions
 */

/**
 * \addtogroup group_usbfxstack_usb_common_macros
 * \{
 */

#ifndef _CY_USB_COMMON_H_

/** Indicates the use of cy_usb_common */
#define _CY_USB_COMMON_H_

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef DOXYGEN

/* This file will be shared across all the layers ie CAL/USBD/APPLICATION. */
#ifndef NULL

/** Specifies NULL with value as 0 */
#define NULL 0
#endif /* NULL */

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#define CY_USBHS_EGR_EPM_BASE_ADDR  0x30000000UL
#define CY_USBHS_ING_EPM_BASE_ADDR  0x30004000UL

#define CY_USB_ENDP_0    0
#define CY_USB_ENDP_1    1
#define CY_USB_ENDP_2    2
#define CY_USB_ENDP_3    3
#define CY_USB_ENDP_4    4
#define CY_USB_ENDP_5    5
#define CY_USB_ENDP_6    6
#define CY_USB_ENDP_7    7
#define CY_USB_ENDP_8    8
#define CY_USB_ENDP_9    9
#define CY_USB_ENDP_10   10
#define CY_USB_ENDP_11   11
#define CY_USB_ENDP_12   12
#define CY_USB_ENDP_13   13
#define CY_USB_ENDP_14   14
#define CY_USB_ENDP_15   15

#endif /* DOXYGEN */

#define CY_USBD_ENDP_DIR_MASK     (uint8_t)0x80 /**< Mask used to get endpoint direction bit from descriptor. */
#define CY_USBD_ENDP_NUM_MASK     (uint8_t)0x7F /**< Mask used to get endpoint index from descriptor. */

#define CY_USB_SET      0x01                    /**< Value used to set control fields in registers. */
#define CY_USB_CLEAR    0x00                    /**< Value used to clear control fields in registers. */

/** Maximum number of endpoints supported by the device. */
#define CY_USB_MAX_ENDP_NUMBER     16

/** Maximum number of string descriptors which are managed by the USBD stack. */
#define CY_USBD_MAX_STR_DSCR_INDX  16

/** Number of SS termination detect attempts to be made by the device. */
#define CY_USB_MAX_TERMINATION_DETECTION_COUNT 0x05

/** Parameter passed to Cy_USBD_ClearZlpSlpIntrEnableMask API to select ZLP interrupt. */
#define CY_USB_ARG_ZLP 0x01

/** Parameter passed to Cy_USBD_ClearZlpSlpIntrEnableMask API to select SLP interrupt. */
#define CY_USB_ARG_SLP 0x00

/** Get minimum from among two numbers. */
#define CY_USB_MIN(arg1,arg2) (((arg1) <= (arg2)) ? (arg1): (arg2))

/** Get maximum from among two numbers. */
#define CY_USB_MAX(arg1,arg2) (((arg1) >= (arg2)) ? (arg1): (arg2))

/** Retrieves byte 0 from a 32 bit number. */
#define CY_USB_DWORD_GET_BYTE0(d)        ((uint8_t)((d) & 0xFF))

/** Retrieves byte 1 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE1(d)        ((uint8_t)(((d) >>  8) & 0xFF))

/** Retrieves byte 2 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE2(d)        ((uint8_t)(((d) >> 16) & 0xFF))

/** Retrieves byte 3 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE3(d)        ((uint8_t)(((d) >> 24) & 0xFF))

/** Get the LS byte from a 16-bit number */
#define CY_USB_GET_LSB(w)        ((uint8_t)((w) & UINT8_MAX))

/** Get the LS word from a 32-bit number */
#define CY_USB_GET_LSW(d)        ((uint16_t)((d) & UINT16_MAX))

/** Get the MS byte from a 16-bit number */
#define CY_USB_GET_MSB(w)        ((uint8_t)((w) >> 8))

/** Get the MS word from a 32-bit number */
#define CY_USB_GET_MSW(d)      ((uint16_t)((d) >> 16))

/** Get the most significant two bytes from a 32-bit number. */
#define DWORD_MSB_BYTES(val)   CY_USB_DWORD_GET_BYTE2(val),\
                               CY_USB_DWORD_GET_BYTE3(val)

/** Get the least significant two bytes from a 32-bit number. */
#define DWORD_LSB_BYTES(val)   CY_USB_DWORD_GET_BYTE0(val),\
                               CY_USB_DWORD_GET_BYTE1(val)

/** Get all the bytes from a 32-bit number in little endian order. */
#define LITTLE_ENDIAN_32(val)   CY_USB_DWORD_GET_BYTE0(val),\
                                CY_USB_DWORD_GET_BYTE1(val), \
                                CY_USB_DWORD_GET_BYTE2(val), \
                                CY_USB_DWORD_GET_BYTE3(val)

/** Get all the bytes from a 64-bit number in little endian order. */
#define LITTLE_ENDIAN_64(msb,lsb)  LITTLE_ENDIAN_32(lsb),\
                                   LITTLE_ENDIAN_32(msb)

/** Combine two byte values into a 16-bit integer. */
#define CY_USB_BUILD_2B_WORD(byte1, byte0)  \
                             ((uint16_t)((byte1 << 8) | byte0))

/** Combine four byte values into a 32-bit integer. */
#define CY_USB_BUILD_4B_WORD(byte3,byte2,byte1,byte0) \
             (uint32_t)((((byte3) << 24) | ((byte2) << 16) | \
                         ((byte1) << 8) | (byte0)))

/** Field mask that indicates a 32-bit debug value in the event log buffer. */
#define CY_EVT_LOG_DBG_MSG              (0x80000000UL)

/** Add a 32-bit debug value to the event log buffer. */
#define CY_LOG_VALUE(logmsg)            (Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_EVT_LOG_DBG_MSG | (logmsg)))

/** Add an 8-bit event code along with timestamp to the event log buffer. */
#define CY_LOG_EVENT(event)             (Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, event))

/** \} group_usbfxstack_usb_common_macros */

/**
 * \addtogroup group_usbfxstack_usb_common_typedefs
 * \{
 */

/**
 * USB driver to stack callback prototype.
 * A function of this type is used by the USBHS and USBSS drivers to notify the
 * USBD stack about relevant interrupts.
 *
 * \return
 * Return value from stack indicates whether a context switch should be initiated
 * when the ISR returns.
 */
typedef bool (* cy_usb_cal_msg_callback_t) (
        void *pUsbdCtxt,                        /**< Pointer to the USBD stack context structure. */
        void *pMsg                              /**< Pointer to the message structure. */
        );

/** \} group_usbfxstack_usb_common_typedefs */

/**
 * \addtogroup group_usbfxstack_usb_common_enums
 * \{
 */

/**
 * @typedef cy_en_usb_cal_ret_code_t
 * @brief List of API error codes returned by CAL APIs in various
 * conditions.
 */
typedef enum {
    CY_USB_CAL_STATUS_SUCCESS=0,                        /**< Function completed successfully. */
    CY_USB_CAL_STATUS_FAILURE,                          /**< Hardware level error encountered. */
    CY_USB_CAL_STATUS_BAD_PARAM,                        /**< Invalid parameter passed to function. */
    CY_USB_CAL_STATUS_CAL_CTXT_NULL,                    /**< Driver context is NULL. */
    CY_USB_CAL_STATUS_MALLOC_FAILED,                    /**< Memory allocation for driver structures failed. */
    CY_USB_CAL_STATUS_MSG_SEND_FAIL,                    /**< Sending of message to USBD stack failed. */
    CY_USB_CAL_STATUS_CAL_BASE_NULL,                    /**< Pointer to USBHS device register set is NULL. */
    CY_USB_CAL_STATUS_TOGGLE_FAILED,                    /**< Failed to update data toggle on USB 2.x endpoint. */
    CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL,               /**< Pointer to USB32 protocol register set is NULL. */
    CY_USB_CAL_STATUS_CAL_LNK_BASE_NULL,                /**< Pointer to USB32 link register set is NULL. */
    CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL,                /**< Pointer to USB32 EPM register set is NULL. */
    CY_USB_CAL_STATUS_CAL_MAIN_BASE_NULL,               /**< Pointer to USB32 register set is NULL. */
    CY_USB_CAL_STATUS_TIMEOUT                           /**< Operation timed out. */
}cy_en_usb_cal_ret_code_t;

/**
 * @typedef cy_en_usb_cal_msg_type_t
 * @brief Types of messages sent the by the USBHS and/or USBSS CAL drivers
 * to the USBD stack.
 */
typedef enum {
    CY_USB_CAL_MSG_INVALID=0,                           /**< Invalid message type. */
    CY_USB_CAL_MSG_RESET,                               /**< USB 2.x reset event. */
    CY_USB_CAL_MSG_HSGRANT,                             /**< USB 2.x high speed chirp handshake complete. */
    CY_USB_CAL_MSG_RESET_DONE,                          /**< USB 2.x reset complete event. */
    CY_USB_CAL_MSG_SUDAV,                               /**< USB 2.x control request received. */
    CY_USB_CAL_MSG_STATUS_STAGE,                        /**< USB 2.x control request acknowledged. */
    CY_USB_CAL_MSG_SETADDR,                             /**< SET_ADDRESS request acknowledged. */
    CY_USB_CAL_MSG_SUSP,                                /**< USB link suspended. */
    CY_USB_CAL_MSG_RESUME_START,                        /**< USB 2.x resume signalling started. */
    CY_USB_CAL_MSG_RESUME_END,                          /**< USB 2.x resume signalling completed. */
    CY_USB_CAL_MSG_DEEP_SLEEP_EXIT,                     /**< Deep sleep exit detected. */
    CY_USB_CAL_MSG_L1_SLEEP,                            /**< USB 2.x link entered L1 state. */
    CY_USB_CAL_MSG_L1_URESUME,                          /**< USB 2.x link exited L1 state. */
    CY_USB_CAL_MSG_IN_ZLP,                              /**< ZLP interrupt received on IN endpoint. */
    CY_USB_CAL_MSG_IN_SLP,                              /**< SLP interrupt received on IN endpoint. */
    CY_USB_CAL_MSG_OUT_ZLP,                             /**< ZLP interrupt received on OUT endpoint. */
    CY_USB_CAL_MSG_OUT_SLP,                             /**< SLP interrupt received on OUT endpoint. */
    CY_USB_CAL_MSG_OUT_DONE,                            /**< Expected amount of data received on OUT endpoint. */
    CY_USB_CAL_MSG_SOF_ITP,                             /**< Start-of-Frame or ITP packet received. */
    CY_USB_CAL_MSG_ERRLIMIT,                            /**< USB 2.x device raised error limit interrupt. */
    CY_USBSS_CAL_MSG_LNK_RESET,                         /**< USB 3.x link reset: Warm or Hot Reset. */
    CY_USBSS_CAL_MSG_LNK_CONNECT,                       /**< USB 3.x far-end terminations detected. */
    CY_USBSS_CAL_MSG_LNK_DISCONNECT,                    /**< USB 3.x link entered SS.Disabled state. */
    CY_USBSS_CAL_MSG_LNK_INT,                           /**< USB 3.x link interrupt received. */
    CY_USBSS_CAL_MSG_LNK_ERR_LIMIT,                     /**< Too many USB 3.x link errors detected. */
    CY_USBSS_CAL_MSG_LNK_RX_DETECT_ACTIVE,              /**< USB 3.x link entered Rx.Detect.Active state. */
    CY_USBSS_CAL_MSG_LNK_SS_DISABLE,                    /**< USB 3.x link entered SS.Disabled state. */
    CY_USBSS_CAL_MSG_LNK_COMPLIANCE,                    /**< USB 3.x link entered Compliance state. */
    CY_USBSS_CAL_MSG_USB3_LNKFAIL,                      /**< USB 3.x link unstable. */
    CY_USBSS_CAL_MSG_EP_INT,                            /**< USB 2.x endpoint interrupt received. */
    CY_USBSS_CAL_MSG_PROT_INT,                          /**< USB 3.x protocol interrupt received. */
    CY_USB_CAL_MSG_PROT_SETADDR_0,                      /**< Device address set to 0 */
    CY_USB_CAL_MSG_PROT_SUTOK,                          /**< USB 3.x control request received. */
    CY_USB_CAL_MSG_PROT_HOST_ERR,                       /**< USB 3.x host error indication received. */
    CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP,               /**< Port Capability LMP handshake timed out. */
    CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG,               /**< Port Configuration LMP handshake timed out. */
    CY_USB_CAL_MSG_PROT_TIMEOUT_PING,                   /**< Ping timeout. */
    CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CAP,           /**< Invalid Port Capability LMP received. */
    CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CFG,           /**< Invalid Port Configuration LMP received. */
    CY_USBSS_CAL_MSG_PROT_EP_INT,                       /**< USB 3.x endpoint interrupt received. */
    CY_USBSS_CAL_MSG_VBUS_CHANGE,                       /**< VBus voltage change detected. */
    CY_USBSS_CAL_MSG_USB3_WARM_RESET,                   /**< USB 3.x warm reset detected. */
    CY_USBSS_CAL_MSG_USB3_U3_SUSPEND,                   /**< USB 3.x link entered U3 */
    CY_USBSS_CAL_MSG_USB3_U0_RESUME,                    /**< USB 3.x link re-entered U0 from U3 */
    CY_USBSS_CAL_MSG_UX_REENABLE,                       /**< Indication to re-enable USB 3.x LPM transitions. */
    CY_USBSS_CAL_MSG_EPM_UNDERRUN,                      /**< Underrun detected by the USB 3.x endpoint memory. */
    CY_USBSS_CAL_MSG_USB3_LMP_FAIL,                     /**< LMP handshake failure. */
    CY_USBSS_CAL_MSG_USB3_RATE_CHG,                     /**< USB 3.x data rate changed. */
    CY_USBSS_CAL_MSG_ABORT_UX_ENABLE,                   /**< Cancel re-enable of USB 3.x LPM transitions. */
    CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE,             /**< Handle Gen2 link training failure. */
    CY_USB_CAL_MSG_EP0_RCV_DONE,                        /**< EP0-OUT receive complete. */
    CY_USBSS_CAL_MSG_PORT_CONFIGURED,                   /**< Port Configuration LMP handshake completed. */
    CY_USBSS_CAL_MSG_LPBK_FORCED,                       /**< Link has been forced into Loopback state. */
    CY_USB_CAL_MSG_MAX                                  /**< Invalid message. */
}cy_en_usb_cal_msg_type_t;

/**
 * @typedef cy_en_usbss_cal_evt_type_t
 * @brief List of USB events which are logged into the RAM based time-stamped event buffer
 * for debug support.
 */
typedef enum {
    CY_SSCAL_EVT_NONE = 0x00,           /**< 0x00: No event indication. */
    CY_SSCAL_EVT_DRV_EN,                /**< 0x01: USBSS driver/IP enable. */
    CY_SSCAL_EVT_CONN_EN,               /**< 0x02: USBSS connection enable. */
    CY_SSCAL_EVT_CONN_DIS,              /**< 0x03: USBSS connection disable. */
    CY_SSCAL_EVT_CONNECT,               /**< 0x04: Link connect interrupt. */
    CY_SSCAL_EVT_DISCONNECT,            /**< 0x05: Link disconnect interrupt. */
    CY_SSCAL_EVT_RESET,                 /**< 0x06: Link reset interrupt. */
    CY_SSCAL_EVT_PSVP_RECONNECT,        /**< 0x07: PSVP reconnect attempt after polling failure. */
    CY_SSCAL_EVT_PORT_CAP_HNDSHK,       /**< 0x08: Port Capability handshake complete. */
    CY_SSCAL_EVT_PORT_CFG_HNDSHK,       /**< 0x09: Port Configuration handshake complete. */
    CY_SSCAL_EVT_LMP_TIMEOUT,           /**< 0x0A: Port Cap/Cfg handshake timeout. */
    CY_SSCAL_EVT_VBUS_PRESENT,          /**< 0x0B: VBus presence detected. */
    CY_SSCAL_EVT_VBUS_ABSENT,           /**< 0x0C: VBus absence detected. */
    CY_SSCAL_EVT_WARM_RESET,            /**< 0x0D: Warm Reset detected. */
    CY_SSCAL_EVT_DBG_1,                 /**< 0x0E: App specific debug event #1 */
    CY_SSCAL_EVT_DBG_2,                 /**< 0x0F: App specific debug event #2 */
    CY_SSCAL_EVT_VBUS_CHG_INTR,         /**< 0x10: VBus state change interrupt. */
    CY_SSCAL_EVT_SETADDR_INTR,          /**< 0x11: SET_ADDRESS request received. */
    CY_SSCAL_EVT_SUTOK_INTR,            /**< 0x12: New Control request received. */
    CY_SSCAL_EVT_EP0STAT_INTR,          /**< 0x13: Status TP received. */
    CY_SSCAL_EVT_RATE_GEN1,             /**< 0x14: Data rate changed to Gen1 */
    CY_SSCAL_EVT_RATE_GEN2,             /**< 0x15: Data rate changed to Gen2 */
    CY_SSCAL_EVT_NEXT_CP_SEL,           /**< 0x16: Shift to next compliance pattern. */
    CY_SSCAL_EVT_PHY0_SELECT,           /**< 0x17: PHY0 instance selected for connection. */
    CY_SSCAL_EVT_PHY1_SELECT,           /**< 0x18: PHY1 instance selected for connection. */
    CY_SSCAL_EVT_POLLING_CDR_RST,       /**< 0x19: CDR reset due to long stay in Polling.Active. */
    CY_SSCAL_EVT_RECOV_DELAY,           /**< 0x1A: Recovery is going on for a long time. */
    CY_SSCAL_EVT_CDR_RESET_1,           /**< 0x1B: CDR reset on entry into Polling.RxEq. */

    CY_SSCAL_EVT_HP_TIMEOUT,            /**< 0x1C: PENDING_HP_TIMER timeout. */
    CY_SSCAL_EVT_U3_PHY_DISABLED,       /**< 0x1D: USB3 PHY disabled since link is in U3 */
    CY_SSCAL_EVT_U3_WAKE_PHY_ENABLE,    /**< 0x1E: USB3 PHY re-enabled as part of U3 exit. */
    CY_SSCAL_EVT_DEV_U3_EXIT,           /**< 0x1F: Device initiated U3 exit. */

    CY_SSCAL_EVT_LNK_SS_DISABLE = 0x20, /**< 0x20: LTSSM state change to SS.Disable. */
    CY_SSCAL_EVT_LNK_RX_DET_RES,        /**< 0x21: LTSSM state change to Rx.Detect.Reset. */
    CY_SSCAL_EVT_LNK_RX_DET_ACT,        /**< 0x22: LTSSM state change to Rx.Detect.Active. */
    CY_SSCAL_EVT_LNK_RX_DET_QUIET,      /**< 0x23: LTSSM state change to Rx.Detect.Quiet. */
    CY_SSCAL_EVT_LNK_SS_INACT_QUIET,    /**< 0x24: LTSSM state change to SS.Inactive.Quiet */
    CY_SSCAL_EVT_LNK_SS_INACT_DISC,     /**< 0x25: LTSSM state change to SS.Inactive.Disconnect.Detect */
    CY_SSCAL_EVT_LNK_HOTRST_ACT,        /**< 0x26: LTSSM state change to HotReset.Active. */
    CY_SSCAL_EVT_LNK_HOTRST_EXIT,       /**< 0x27: LTSSM state change to HotReset.Exit. */
    CY_SSCAL_EVT_LNK_POLLING_LFPS,      /**< 0x28: LTSSM state change to Polling.LFPS. */
    CY_SSCAL_EVT_LNK_POLLING_RXEQ,      /**< 0x29: LTSSM state change to Polling.RxEQ. */
    CY_SSCAL_EVT_LNK_POLLING_ACT,       /**< 0x2A: LTSSM state change to Polling.Active. */
    CY_SSCAL_EVT_LNK_POLLING_CFG,       /**< 0x2B: LTSSM state change to Polling.Configuration. */
    CY_SSCAL_EVT_LNK_POLLING_IDLE,      /**< 0x2C: LTSSM state change to Polling.Idle. */
    CY_SSCAL_EVT_LNK_POLLING_PLUS,      /**< 0x2D: LTSSM state change to Polling.LFPSPlus. */
    CY_SSCAL_EVT_LNK_POLLING_PMATCH,    /**< 0x2E: LTSSM state change to Polling.PortMatch. */
    CY_SSCAL_EVT_LNK_POLLING_PCFG,      /**< 0x2F: LTSSM state change to Polling.PortConfig */
    CY_SSCAL_EVT_LNK_U0,                /**< 0x30: LTSSM state change to U0 */
    CY_SSCAL_EVT_LNK_U1,                /**< 0x31: LTSSM state change to U1 */
    CY_SSCAL_EVT_LNK_U2,                /**< 0x32: LTSSM state change to U2 */
    CY_SSCAL_EVT_LNK_U3,                /**< 0x33: LTSSM state change to U3 */
    CY_SSCAL_EVT_LNK_LPBK_ACT,          /**< 0x34: LTSSM state change to Loopback.Active */
    CY_SSCAL_EVT_LNK_LPBK_EXIT,         /**< 0x35: LTSSM state change to Loopback.Exit */
    CY_SSCAL_EVT_LNK_UNDEF0,            /**< 0x36: Undefined link state. */
    CY_SSCAL_EVT_LNK_COMPLIANCE,        /**< 0x37: LTSSM state change to Compliance */
    CY_SSCAL_EVT_LNK_RECOV_ACT,         /**< 0x38: LTSSM state change to Recovery.Active */
    CY_SSCAL_EVT_LNK_RECOV_CFG,         /**< 0x39: LTSSM state change to Recovery.Configuration */
    CY_SSCAL_EVT_LNK_RECOV_IDLE,        /**< 0x3A: LTSSM state change to Recovery.Idle */

    CY_USBD_EVT_SS_TO_HS = 0x40,        /**< 0x40: SS to HS fallback. */
    CY_USBD_EVT_SS_REEN,                /**< 0x41: SS Re-enable due to USB 2.x reset. */
    CY_USBD_EVT_SS_DISB,                /**< 0x42: SS disable from USBD. */
    CY_USBD_EVT_HS_DISB,                /**< 0x43: HS disable due to SS receiver detect. */
    CY_USBD_EVT_HS_RESET_SKIP,          /**< 0x44: SS re-enable skipped on HS reset. */


    CY_HSCAL_EVT_CONN_EN = 0x50,          /**< 0x50: USBHS connection enable. */
    CY_HSCAL_EVT_CONN_DIS,                /**< 0x51: USBHS connection disable. */
    CY_HSCAL_EVT_PHY_COMM_INIT,           /**< 0x52: USBHS Common PHY INIT done. */
    CY_HSCAL_EVT_PHY_FSHSMODE_INIT,       /**< 0x53: USBHS FSHS PHY INIT done. */
    CY_HSCAL_EVT_CAL_INIT_DONE,           /**< 0x54: USBHS CAL init done. */
    CY_HSCAL_EVT_PLL_INIT,                /**< 0x55: USBHS PLL init done. */
    CY_HSCAL_EVT_PLL_DEINIT,              /**< 0x56: USBHS PLL DEINIT done. */
    CY_HSCAL_EVT_SUSP,                    /**< 0x57: USBHS suspend interrupt. */
    CY_HSCAL_EVT_URESET,                  /**< 0x58: USBHS URESET  interrupt. */
    CY_HSCAL_EVT_HSGRANT,                 /**< 0x59: USBHS HSGRANT interrupt. */
    CY_HSCAL_EVT_SUDAV,                   /**< 0x5A: USBHS SUDAV   interrupt. */
    CY_HSCAL_EVT_ERRLIMIT,                /**< 0x5B: USBHS ERRLIMIT interrupt. */
    CY_HSCAL_EVT_URESUME,                 /**< 0x5C: USBHS URESUME interrupt. */
    CY_HSCAL_EVT_STATUS_STAGE,            /**< 0x5D: USBHS STATUS STAGE interrupt. */
    CY_HSCAL_EVT_L1_SLEEP_REQ,            /**< 0x5E: USBHS L1SLEEP_REQ interrupt. */
    CY_HSCAL_EVT_L1_URESUME,              /**< 0x5F: USBHS L1URESUME   interrupt. */
    CY_HSCAL_EVT_RESET_DONE,              /**< 0x60: USBHS RESET DONE  interrupt. */
    CY_HSCAL_EVT_SET_ADDR,                /**< 0x61: USBHS SET ADDR     interrupt. */
    CY_HSCAL_EVT_HOST_URESUME_ARR,        /**< 0x62: USBHS HOST URESUME interrupt. */
    CY_HSCAL_EVT_DPSLP,                   /**< 0x63: USBHS DPSLP interrupt. */
    CY_HSCAL_EVT_PID_MISMATCH,            /**< 0x64: USBHS PID MISMATCH interrupt. */
    CY_HSCAL_EVT_START_REMOTE_WAKE_SGNAL, /**< 0x65: USBHS REMOTE_WAKE SIGNAL interrupt. */

    CY_HSCAL_EVT_USBD_INIT = 0x70,        /**< 0x70: USBD INIT DONE. */

    CY_USBD_DATA_RATE_UNCONNECTED = 0x80, /**< 0x80: USB connection not present. */
    CY_USBD_DATA_RATE_FS,                 /**< 0x81: Data rate changed to USB-FS */
    CY_USBD_DATA_RATE_HS,                 /**< 0x82: Data rate changed to USB-HS */
    CY_USBD_DATA_RATE_GEN1X1,             /**< 0x83: Data rate changed to Gen1x1 (5Gbps) */
    CY_USBD_DATA_RATE_GEN1X2,             /**< 0x84: Data rate changed to Gen1x2 (9Gbps) */
    CY_USBD_DATA_RATE_GEN2X1,             /**< 0x85: Data rate changed to Gen2x1 (10Gbps) */
    CY_USBD_DATA_RATE_GEN2X2,             /**< 0x86: Data rate changed to Gen2x2 (20Gbps) */

    CY_USB_UVC_EVT_SET_CFG  = 0x90,       /**< 0x90: UVC application SET_CONFIG request. */
    CY_USB_UVC_EVT_SET_INTF,              /**< 0x91: UVC application SET_INTERFACE request. */
    CY_USB_UVC_EVT_VSTREAM_START,         /**< 0x92: UVC video streaming start request. */
    CY_USB_UVC_EVT_SET_CUR_REQ,           /**< 0x93: UVC SET_CUR request. */

    CY_USB_EVT_INIT_LVDS_LB_EN = 0xA0,    /**< 0xA0: LVDS link loopback enabled. */
    CY_USB_EVT_PPORT0_EN,                 /**< 0xA1: LVDS port #0 enabled. */
    CY_USB_EVT_PPORT1_EN,                 /**< 0xA2: LVDS port #1 enabled. */
    CY_USB_EVT_LVCMOS_EN,                 /**< 0xA3: LVCMOS mode selected. */
    CY_USB_EVT_LVDS_EN,                   /**< 0xA4: LVDS mode selected. */
    CY_USB_EVT_INMD_EN,                   /**< 0xA5: Metadata insertion enabled. */
    CY_USB_EVT_WL_EN,                     /**< 0xA6: LVDS WideLink mode selected. */

    CY_USB_U3V_EVT_EPHALT_DCI_CMD = 0xB0, /**< 0xB0: U3V DCI command endpoint halted. */
    CY_USB_U3V_EVT_EPHALT_DCI_RSP,        /**< 0xB1: U3V DCI response endpoint halted. */
    CY_USB_U3V_EVT_U3V_CMD,               /**< 0xB2: U3V DCI command received. */
    CY_USB_U3V_EVT_U3V_RESP_SENT,         /**< 0xB3: U3V DCI response sent. */

    CY_LNK_EVENT_DBG_0            = 0xF0, /**< 0xF0: */
    CY_LNK_EVENT_SET_U1,                  /**< 0xF1: */
    CY_LNK_EVENT_SET_U2,                  /**< 0xF2: */
    CY_LNK_EVENT_CLR_U1,                  /**< 0xF3: */
    CY_LNK_EVENT_CLR_U2,                  /**< 0xF4: */
    CY_LNK_EVENT_SET_FORCE_ACCEPT_LPM,    /**< 0xF5: LPM enable forced due to Link Management Packet. */
    CY_LNK_EVENT_CLR_FORCE_ACCEPT_LPM,    /**< 0xF6: Forced LPM enable cleared due to Link Management Packet. */
    CY_LNK_EVENT_CDR_RST_DECODE_ERR,      /**< 0xF7: Decode error reported by PHY for config lane. */
    CY_LNK_EVENT_CDR_RST_DECODE_ERR_PHY_1,/**< 0xF8: Decode error reported by PHY for non-config lane. */
    CY_LNK_EVENT_SS_DISABLED_ENTRY,       /**< 0xF9: SS.Disabled state entered. */
    CY_LNK_EVENT_TRIG_DISCON_EVENT,       /**< 0xFA: LTSSM Disconnect interrupt forced by firmware. */
    CY_LNK_EVENT_LANE_POLARITY_INV,       /**< 0xFB: Lane Polarity inverted */
    CY_LNK_EVENT_LNK_FORCE_U0,            /**< 0xFC: Link forced into U0 state during Ux exit sequence. */
    CY_LNK_EVENT_UX_EXIT_TS1_TIMEOUT      /**< 0xFD: Transition into Recovery.Config did not happen within expected time. */
} cy_en_usbss_cal_evt_type_t;

/**
 * @typedef cy_en_usb_speed_t
 * @brief List of USB connection speeds which can be supported on the device.
 */
typedef enum
{
    CY_USBD_USB_DEV_NOT_CONNECTED = 0x00,               /**< USB connection is inactive. */
    CY_USBD_USB_DEV_FS,                                 /**< Full-Speed USB connection. */
    CY_USBD_USB_DEV_HS,                                 /**< High-Speed USB connection. */
    CY_USBD_USB_DEV_SS_GEN1,                            /**< USB 3.2 Gen1x1 (5 Gbps) USB connection. */
    CY_USBD_USB_DEV_SS_GEN1X2,                          /**< USB 3.2 Gen1x2 (10 Gbps) USB connection. */
    CY_USBD_USB_DEV_SS_GEN2,                            /**< USB 3.2 Gen2x1 (10 Gbps) USB connection. */
    CY_USBD_USB_DEV_SS_GEN2X2,                          /**< USB 3.2 Gen2x2 (20 Gbps) USB connection. */
} cy_en_usb_speed_t;

/**
 * @typedef cy_en_usb_config_lane_t
 * @brief Options for USB 3.x Configuration Lane Selection.
 */
typedef enum
{
    CY_USB_CFG_LANE_AUTODETECT = 0,     /**< Configuration lane to be detected through CC voltage measurement. */
    CY_USB_CFG_LANE_0,                  /**< Lane-0 to be used as Configuration lane. */
    CY_USB_CFG_LANE_1                   /**< Lane-1 to be used as Configuration lane. */
} cy_en_usb_config_lane_t;

/**
 * @brief List of USB endpoint types.
 */
typedef enum {
    CY_USB_ENDP_TYPE_CTRL=0,                            /**< Control endpoint. */
    CY_USB_ENDP_TYPE_ISO,                               /**< Isochronous endpoint. */
    CY_USB_ENDP_TYPE_BULK,                              /**< Bulk endpoint. */
    CY_USB_ENDP_TYPE_INTR,                              /**< Interrupt endpoint. */
    CY_USB_ENDP_TYPE_INVALID                            /**< Invalid endpoint type. */
}cy_en_usb_endp_type_t;

/**
 * @brief Defines direction of USB endpoints.
 */
typedef enum {
    CY_USB_ENDP_DIR_OUT=0,                              /**< OUT endpoint: host to device. */
    CY_USB_ENDP_DIR_IN,                                 /**< IN endpoint: device to host. */
    CY_USB_ENDP_DIR_INVALID                             /**< Invalid endpoint direction. */
}cy_en_usb_endp_dir_t;

/** \} group_usbfxstack_usb_common_enums */

/**
 * \addtogroup group_usbfxstack_usb_common_structs
 * \{
 */

/**
 * @brief Structure used by the HS and SS CAL drivers to send messages to the
 * USBD stack layer.
 */
typedef struct cy_stc_usb_cal_msg_t {
    cy_en_usb_cal_msg_type_t type;                      /**< Type of message. */
    uint32_t data[2];                                   /**< Data associated with the message. */
}cy_stc_usb_cal_msg_t;

#ifndef DOXYGEN
/**
 * @brief Message structure for HOST layer.
 */
typedef struct cy_stc_usb_host_msg_t {
    uint32_t type;                                      /**< Type of message. */
    uint32_t data[32];                                  /**< Data associated with the message. */
}cy_stc_usb_host_msg_t;
#endif /* DOXYGEN */

/**
 * @brief Structure used by the USBD stack to send messages to the application
 * layer.
 */
typedef struct cy_stc_usbd_app_msg_t {
    uint32_t type;                                      /**< Type of message. */
    uint32_t data[2];                                   /**< Data associated with the message. */
}cy_stc_usbd_app_msg_t;

/**
 * @brief Fields which are part of the 8 byte data associated with the control request
 * received on endpoint 0.
 */
typedef struct cy_stc_usb_setup_req_t {
    uint8_t bmRequest;                                  /**< bmRequestType field from the USB specification. */
    uint8_t bRequest;                                   /**< bRequest field from the USB specification. */
    uint16_t wValue;                                    /**< wValue field from the USB specification. */
    uint16_t wIndex;                                    /**< wIndex field from the USB specification. */
    uint16_t wLength;                                   /**< wLength field from the USB specification. */
}cy_stc_usb_setup_req_t;

/**
 * @brief Structure which encapsulates all properties of USB endpoints on the
 * device.
 */
typedef struct
{
    cy_en_usb_endp_type_t endpType;             /**< Type of endpoint. */
    cy_en_usb_endp_dir_t endpDirection;         /**< Endpoint direction. */
    bool valid;                                 /**< Whether endpoint is to be enabled or disabled. */
    uint8_t isoPkts;                            /**< MULT setting for periodic endpoints. */
    uint8_t interval;                           /**< Service interval for periodic endpoints. */
    bool allowNakTillDmaRdy;                    /**< Whether OUT EP should NAK when DMA is not ready. Only HS/FS. */
    uint32_t endpNumber;                        /**< Endpoint index: 0 to 15 */
    uint32_t maxPktSize;                        /**< Maximum packet size in bytes. */
    uint32_t burstSize;                         /**< Maximum burst setting: 1 to 16 */
    uint32_t streamID;                          /**< Number of streams supported on BULK endpoint. */
} cy_stc_usb_endp_config_t;

/** \} group_usbfxstack_usb_common_structs */

/**
 * \addtogroup group_usbfxstack_usb_common_functions
 * \{
 */

/*******************************************************************************
* Function Name: Cy_USBHS_CalculateEpmAddr
****************************************************************************//**
*
* Returns the base address of the Endpoint Memory region corresponding
* to a USBHSDEV endpoint.
*
* \param endpNum
* Endpoint index (valid range is 0 to 15).
*
* \param endpDirection
* Direction of the endpoint
*
* \return
* The base address of the endpoint memory region for the endpoint.
*
*******************************************************************************/
static inline uint32_t *
Cy_USBHS_CalculateEpmAddr (uint32_t endpNum, cy_en_usb_endp_dir_t endpDirection)
{
    if (endpNum >= CY_USB_MAX_ENDP_NUMBER)
    {
        return NULL;
    }

    if (endpDirection == CY_USB_ENDP_DIR_IN)
    {
        return((uint32_t *)(CY_USBHS_EGR_EPM_BASE_ADDR + (endpNum << 10U)));
    }

    return((uint32_t *)(CY_USBHS_ING_EPM_BASE_ADDR + (endpNum << 10U)));
}

/** \} group_usbfxstack_usb_common_functions */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_COMMON_H_ */

/** \} group_usbfxstack_usb_common */

/* [] EOF */
