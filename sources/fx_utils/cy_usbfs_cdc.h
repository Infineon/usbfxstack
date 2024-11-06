/***************************************************************************//**
* \file cy_usbfs_cdc.h
* \version 1.0
*
* Provided API declarations for the Comm. Device Class implementation using
* the USBFS IP block.
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

/**
 * \addtogroup group_usbfxstack_fx_utils
 * \{
 */

/**
 * \addtogroup group_usbfxstack_fx_utils_macros
 * \{
 */

#ifndef CY_USBFS_CDC_H

/** Indicates the use of cy_usbfs_cdc */
#define CY_USBFS_CDC_H

#include "stdint.h"
#include "stdbool.h"
#include "cy_device.h"
#include "cy_device_headers.h"
#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** Number of endpoints supported. */
#define USBFS_NUM_EP                              (8)

/** Maximum packet size for the endpoints. */
#define USBFS_EP_MAX_PKT_SIZE                     (64)

/** Value to be written into BUF_SIZE register to select max. pkt. size for endpoints as 64 bytes. */
#define USBFS_BUF_SIZE_64BYTE                     (0x66)

/** Maximum packet size for EP0 */
#define USBFS_EP0_SIZE                            (8)

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_enums
 * \{
 */

/**
 * @typedef cy_en_usbfs_devstate_t
 * @brief List of USBFS device mode states.
 */
typedef enum
{
    USBFS_STATE_INACTIVE = 0,           /**< Un-initialized state. */
    USBFS_STATE_DISABLED,               /**< Disabled state. */
    USBFS_STATE_CONNECTED,              /**< Connected to an external USB host. */
    USBFS_STATE_RESET,                  /**< When the device has received a reset. */
    USBFS_STATE_ADDRESSED,              /**< Device address has been set. */
    USBFS_STATE_CONFIGURED,             /**< A valid device configuration is selected. */
} cy_en_usbfs_devstate_t;

/**
 * @typedef cy_en_usbfs_ep0state_t
 * @brief List of USB EP0 (control endpoint) states.
 */
typedef enum
{
    USB_EP0_STATE_DISABLED = 0,         /**< EP0 is disabled. */
    USB_EP0_STATE_SETUP,                /**< EP0 in setup phase. */
    USB_EP0_STATE_DATA_IN,              /**< EP0 in data IN phase. */
    USB_EP0_STATE_DATA_OUT,             /**< EP0 in data OUT phase. */
    USB_EP0_STATE_STATUS_IN,            /**< EP0 in status IN ZLP phase. */
    USB_EP0_STATE_STATUS_OUT,           /**< EP0 in status OUT ZLP phase. */
    USB_EP0_STATE_STALL                 /**< EP0 in stall phase. */
} cy_en_usbfs_ep0state_t;

/** \} group_usbfxstack_fx_utils_enums */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

/**
 * @brief USB endpoint handle.
 * This structure holds information about the USB endpoint. The structure is
 * only for internal use and not expected to be created/used by user code.
 */
typedef struct
{
    bool is_out;                /**< Direction of endpoint. */
    bool toggle;                /**< Active data toggle. */
    bool enabled;               /**< Whether the endpoint is enabled. */
} cy_stc_usbfs_epinfo_t;

/**
 * @brief Structure representing the 8-byte data associated with a USB control
 * request which is received after a SETUP token. The format of the structure
 * is defined by the USB specification.
 */
typedef struct
{
    uint8_t attrib;             /**< Request attributes:
                                     Bit7:   0=Host to device, 1=Device to host
                                     Bits6:5 0=Standard, 1=Class, 2=Vendor,
                                             3=Reserved
                                     Bits4:0 0=Device, 1=Interface, 2=Endpoint,
                                             3=Other, 4-31=Reserved */
    uint8_t cmd;                /**< Request command */
    uint16_t value;             /**< Value parameter for command */
    uint16_t index;             /**< Index parameter for command */
    uint16_t length;            /**< Number of bytes to be transferred in data phase */

} cy_stc_usb_setup_pkt_t;

/**
 * @brief USB device controller handle. This structure holds information
 * about the simplified USBFS device stack and its state.
 */
typedef struct
{
    uint16_t dev_stat;                  /**< Device status to be returned during GET_STATUS call. */
    cy_en_usbfs_devstate_t state;       /**< Current USB state. */
    cy_en_usbfs_devstate_t prev_state;  /**< Previous USB state. */
    cy_stc_usb_setup_pkt_t setup_pkt;   /**< Current USB setup packet. */
    cy_en_usbfs_ep0state_t ep0_state;   /**< Current EP0 transfer state. */
    bool ep0_toggle;                    /**< Current EP0 toggle state. */
    uint8_t *ep0_buffer;                /**< Current EP0 data transfer buffer. */
    uint16_t ep0_length;                /**< Pending transfer size for EP0 */
    bool ep0_zlp_rqd;                   /**< Whether an ZLP need to be sent on EP0 to indicate a short transfer. */
    bool ep0_last;                      /**< Whether the current EP0 read / write request is the last. */
    uint8_t active_cfg;                 /**< Current configuration value set by SET_CONFIG. */

    cy_stc_usbfs_epinfo_t ep_handle[USBFS_NUM_EP + 1];        /**< Endpoint handles for internal state machine. */

    uint8_t cdcConfig[8];               /**< CDC SET/GET line coding data buffer. */
    bool    cdcRecvEnabled;             /**< Whether CDC data receive is enabled by user. */
} cy_usbfs_devhandle_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_enums
 * \{
 */

/**
 * @typedef cy_en_usbfs_ep_mode_t
 * @brief USB endpoint modes. The enumeration lists all the possible register configurations
 * for endpoint states. This list is dependent on the hardware and should not be changed.
 */
typedef enum
{
    USBDEV_EP_MODE_DISABLE,             /* Ignore all USB traffic to this EP */
    USBDEV_EP_MODE_NAK_INOUT,           /* SETUP: Accept, IN: NAK, OUT: NAK */
    USBDEV_EP_MODE_STATUS_OUT_ONLY,     /* SETUP: Accept, IN: STALL, OUT: ACK 0B, NAK others */
    USBDEV_EP_MODE_STALL_INOUT,         /* SETUP: Accept, IN: STALL, OUT: STALL */
    USBDEV_EP_MODE_ISO_OUT = 5,         /* SETUP: Ignore, IN: Ignore, OUT: Accept ISO */
    USBDEV_EP_MODE_STATUS_IN_ONLY,      /* SETUP: Accept, IN: Respond 0B, OUT: STALL */
    USBDEV_EP_MODE_ISO_IN,              /* SETUP: Ignore, IN: Accept ISO, OUT: Ignore */
    USBDEV_EP_MODE_NAK_OUT,             /* SETUP: Ignore, IN: Ignore, OUT: NAK */
    USBDEV_EP_MODE_ACK_OUT,             /* SETUP: Ignore, IN: Ignore, OUT: ACK or STALL
                                           (changes mode to NAK_OUT after ACK) */
    USBDEV_EP_MODE_ACK_OUT_STATUS_IN = 11,
    /* SETUP: Ignore, IN: Respond 0B, OUT: ACK */
    USBDEV_EP_MODE_NAK_IN,              /* SETUP: Ignore, IN: NAK, OUT: Ignore */
    USBDEV_EP_MODE_ACK_IN,              /* SETUP: Ignore, IN: ACK or STALL, OUT: Ignore
                                           (changes mode to NAK_IN after ACK) */
    USBDEV_EP_MODE_ACK_IN_STATUS_OUT = 15,
    /* SETUP: Ignore, IN: Respond, OUT: ACK 0B, NAK others */

} cy_en_usbfs_ep_mode_t;

/**
 * @typedef cy_en_usbfs_intr_priority_t
 * @brief USB interrupt priority options.
 */
typedef enum
{
    USB_INTR_PRIORITY_HIGH, /* Configured to use the USB_INTR_HIGH_VECTOR_LOCATION */
    USB_INTR_PRIORITY_MED,  /* Configured to use the USB_INTR_MED_VECTOR_LOCATION */
    USB_INTR_PRIORITY_LOW   /* Configured to use the USB_INTR_LOW_VECTOR_LOCATION */
} cy_en_usbfs_intr_priority_t;

/** \} group_usbfxstack_fx_utils_enums */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

#ifndef DOXYGEN

/**
 * @brief Structure representing the Serial Interface Engine level control
 * registers associated with a non-control endpoint.
 */
typedef struct
{
    volatile uint32_t sie_epx_cnt0;
    volatile uint32_t sie_epx_cnt1;
    volatile uint32_t sie_epx_cr0;
} USBSIE_REGS_T, *PUSBSIE_REGS_T;

/**
 * @brief Structure representing the arbiter level control registers associated
 * with a non-control endpoint.
 */
typedef struct
{
    volatile uint32_t arb_epx_cfg;
    volatile uint32_t arb_epx_int_en;
    volatile uint32_t arb_epx_sr;
    volatile uint32_t rsrvd0;
    volatile uint32_t arb_rwx_wa;
    volatile uint32_t arb_rwx_wa_msb;
    volatile uint32_t arb_rwx_ra;
    volatile uint32_t arb_rwx_ra_msb;
    volatile uint32_t arb_rwx_dr;
} USBARB_REGS_T, *PUSBARB_REGS_T;

/**
 * @brief Structure representing the 16-bit arbiter level control registers
 * associated with a non-control endpoint.
 */
typedef struct
{
    volatile uint32_t arb_rwx_wa16;
    volatile uint32_t rsrvd0;
    volatile uint32_t arb_rwx_ra16;
    volatile uint32_t rsrvd1;
    volatile uint32_t arb_rwx_dr16;
} USBARB16_REGS_T, *PUSBARB16_REGS_T;

#endif /* DOXYGEN */

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/*******************************************************************************
 * Function name: CyUsbFsCdc_Init
 ****************************************************************************//**
 *
 * The API initializes the USB interface. This is mainly a software state
 * machine initialization. The PHY is not enabled at this point. The API
 * helps to cleanup previous state information.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Init(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_Enable
 ****************************************************************************//**
 *
 * The function initializes the USB hardware and enables the D+/D- lines and does
 * a pull up on the D+ line for the external host to detect the presence of
 * the device. The API expects that the USB block is already initialized.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Enable(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_Disable
 ****************************************************************************//**
 *
 * The function disables the USB hardware and disconnects the D+/D- lines.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Disable(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_GetState
 ****************************************************************************//**
 *
 * The API returns the current state of the USB device module.
 *
 * \return
 * The current USB device/stack state.
 *******************************************************************************/
cy_en_usbfs_devstate_t CyUsbFsCdc_GetState(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_CompleteEp0Status
 ****************************************************************************//**
 *
 * The API completes the status phase of the current EP0 request. The caller
 * is expected to call the function in sequence. The status phase is handled
 * inplicitly when the setup_read and setup_write functions are invoked correctly
 * with the last flag set to true. The function does not wait for the transfer to
 * complete. In case of an error, the EP0 shall be stalled by the USB module.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_CompleteEp0Status(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_Ep0AckWait
 ****************************************************************************//**
 *
 * The function does a blocking wait until the status phase is completed.
 * This function should be invoked only if the task loop can be safely blocked.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Ep0AckWait(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_StallEp0
 ****************************************************************************//**
 *
 * The function stalls endpoint zero to indicate error to current request. The
 * stall is automatically cleared on receiving a new setup request.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_StallEp0(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_SetupEp0Read
 ****************************************************************************//**
 *
 * The API does not wait for the read to complete. The function just updates
 * the state machine and queues the first packet read. The read has to be
 * completed by repeatedly queueing packet read requests. The last parameter
 * can be used to do  multiple partial transfers. For default single tranfers,
 * the last parameter should always be true.
 *
 * \param data
 * Buffer to write the received USB EP0 data into. The caller should ensure
 * that the buffer is capable of receiving upto a size of length bytes.
 *
 * \param length
 * Length of data to be transferred. This has to be a multiple of eight bytes.
 *
 * \param last 
 * Whether the request is a partial transfer or not. Set to true if the stack
 * needs to implicily handle the status phase after completing the transfer.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SetupEp0Read(uint8_t *data, uint16_t length, bool last);

/*******************************************************************************
 * Function name: CyUsbFsCdc_SetupEp0Write
 ****************************************************************************//**
 *
 * The API does not wait for the write to complete. The function just updates
 * the state machine and queues the first packet. The write has to be completed
 * by repeatedly queueing packet requests. The last parameter can be used to do
 * multiple partial transfers. For default single tranfers, the last parameter
 * should always be true.
 *
 * \param data
 * Buffer containing the USB EP0 data to be transferred.
 *
 * \param length
 * Length of data to be transferred.
 *
 * \param last 
 * Whether the request is a partial transfer or not. Set to true if the stack
 * needs to implicily handle the status phase after completing the transfer.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SetupEp0Write(uint8_t *data, uint16_t length, bool last);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpEnable
 ****************************************************************************//**
 *
 * Enable an endpoint with the selected configuration.
 * The endpoint shall be initialized and configured with the provided parameters.
 * The API expects the endpoint to be in disabled state.
 *
 * \param ep_index
 * Index of endpoint to be enabled. Valid values are from 1 to 8 (inclusive).
 *
 * \param is_out
 * true if the endpoint is an OUT endpoint, false for IN endpoint.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpEnable(uint8_t ep_index, bool is_out);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDisable
 ****************************************************************************//**
 *
 * Disables a previously enabled endpoint.
 * The endpoint shall be disabled and all data in the FIFO cleared. The endpoint
 * shall stop responding to requests from USB host.
 *
 * \param ep_index
 * Index of endpoint to be disabled. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDisable(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpSetStall
 ****************************************************************************//**
 *
 * The endpoint shall stall all IN / OUT tokens after the function has been
 * executed.
 *
 * \param ep_index
 * Index of endpoint to be modified. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpSetStall(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpClearStall
 ****************************************************************************//**
 *
 * The function shall clear a previously stalled endpoint. The function shall
 * reset the data toggle even if the endpoint was not previously stalled.
 * This call shall  also reset the endpoint to the default state. So read
 * should be explicitly invoked for an OUT endpoint. The endpoint shall start
 * NAKing all the IN / OUT tokens once the function gets executed.
 *
 * \param ep_index
 * Index of endpoint to be modified. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpClearStall(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_IsEpReady
 ****************************************************************************//**
 *
 * Checks whether the endpoint is ready for data transfer. The API expects that
 * the endpoint is enabled and active.
 *
 * \param ep_index
 * Endpoint to be checked
 *
 * \return
 * Endpoint status:
 *      true  - If this is an IN endpoint, then the EP is ready to send data.
 *              If there was a previous transfer then it has completed successfully.
 *              If this is an OUT endpoint, then the EP has received a packet
 *              of data from the USB host.
 *      false - If this is an IN endpoint, data is being sent out and not completed.
 *              If this is an OUT endpoint, then the data is not yet received.
 *              If the endpoint is not active or USB connection is not active,
 *              this API returns CyFalse.
 *******************************************************************************/
bool CyUsbFsCdc_IsEpReady(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_QueueEpRead
 ****************************************************************************//**
 *
 * The function enables the selected endpoint to receive one packet of data.
 * It does not wait for the data to be received.
 *
 * \param ep_index
 * Index of endpoint on which read is to be queued.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_QueueEpRead(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDataRead
 ****************************************************************************//**
 *
 * Retrieves the data packet available on the endpoint.
 * The function expects that a data is already available and retrieves the
 * packet from the endpoint buffer.
 *
 * \param ep_index
 * Index of endpoint on which read is to be performed.
 *
 * \param data
 * Pointer to buffer into which the data should be read.
 *
 * \param count
 * Size of the data to be read (in bytes).
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDataRead(uint8_t ep_index, uint8_t *data, uint8_t *count);

/*******************************************************************************
 * Function name: CyUsbFsCdc_SendZlp
 ****************************************************************************//**
 *
 * The function sends a zero-length packet on the selected IN endpoint.
 *
 * \param ep_index
 * Index of endpoint on which ZLP is to be sent.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SendZlp(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDataWrite
 ****************************************************************************//**
 *
 * The function copies the data available in the buffer to the endpoint buffer
 * and arms the endpoint for transfer.
 *
 * \param ep_index
 * Index of endpoint on which data is to be transferred.
 *
 * \param data
 * Pointer to buffer containing the data to be written.
 *
 * \param count
 * Size of the data to be transferred (in bytes).
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDataWrite(uint8_t ep_index, uint8_t *data, uint8_t  count);

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpFlush
 ****************************************************************************//**
 *
 * The function resets the endpoint and re-arms an OUT endpoint to receive
 * data if a receive was already queued.
 *
 * \param ep_index
 * Index of endpoint which is to be flushed.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpFlush(uint8_t ep_index);

/*******************************************************************************
 * Function name: CyUsbFsCdc_ControlDataReceive
 ****************************************************************************//**
 *
 * Control the handling of data received through the USBFS CDC interface.
 * When data receive is not enabled, the driver will keep discarding any data
 * received on the OUT endpoint. When data receive is enabled, it is expected
 * that the user will queue read operations to fetch the OUT data as required.
 *
 * \param recvEnable
 * Whether data receive handling is enabled.
 *******************************************************************************/
void CyUsbFsCdc_ControlDataReceive(bool recvEnable);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_USBFS_CDC_H */

/** \} group_usbfxstack_fx_utils */

/*[]*/
