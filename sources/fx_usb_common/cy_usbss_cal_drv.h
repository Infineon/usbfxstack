/***************************************************************************//**
* \file cy_usbss_cal_drv.h
* \version 1.0
*
* Defines the interfaces of the FX USBSS block driver.
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

/* This header is not to be documented for FX2G3 */

/**
 * \addtogroup group_usbfxstack_usb_common_macros
 * \{
 */

#if !defined(CY_USBSS_CAL_DRV_H)

/** Indicates the use of cy_usbss_cal_drv */
#define CY_USBSS_CAL_DRV_H

#include <stdbool.h>
#include "cy_device.h"
#include "cy_hbdma.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef DOXYGEN
/* Some sections of the code are intentionally not documented.
 * Definitions in these sections of code are only used internally
 * within the USBFXStack library. We don't need to document them
 * for end users.
 */

#define CY_USB32DEV_PROT_LMP_OVERRIDE_LMP_SEND \
            ((uint32_t)0x01 << USB32DEV_PROT_LMP_OVERRIDE_LMP_SEND_Pos)

/* Default Elastic Buffer Half Depth setting to be used in Gen2 connection. */
#define FX3G2_DFLT_GEN2_EB_DEPTH                (12u)

/* Number of USB 3.x PHY blocks on FX3G2 device. */
#define FX3G2_MAX_NUM_USB_PHY                   (2u)

/* Position of device address field in USB 3.0 LMP and TP. */
#define CY_USB32DEV_PROT_TP_TYPE_POS                 (0)
#define CY_USB32DEV_PROT_TP_DEVADDR_POS              (25)

/* Position of endpoint number field in USB 3.0 TP. */
#define CY_USB32DEV_PROT_TP_SUBTYPE_POS              (0)
#define CY_USB32DEV_PROT_TP_DIR_POS                 (7)
#define CY_USB32DEV_PROT_TP_ENDP_NUM_POS             (8)
#define CY_USB32DEV_PROT_TP_TT_POS                   (12)
#define CY_USB32DEV_PROT_TP_NUMP_POS                 (16)
#define CY_USB32DEV_PROT_TP_SEQ_NUM_POS              (21)

/*  PROT_CS related defines */
#define CY_USBSS_PROT_CS_SETUP_CLR_BUSY  (0x01u << USB32DEV_PROT_CS_SETUP_CLR_BUSY_Pos)
#define CY_USBSS_PROT_CS_NRDY_ALL        (0x01u << USB32DEV_PROT_CS_NRDY_ALL_Pos)

/* PROT_SEQ_NUM related defines. */
#define CY_USBSS_PROT_SEQ_NUM_ENDP           (0x0Fu << USB32DEV_PROT_SEQ_NUM_ENDPOINT_Pos)
#define CY_USBSS_PROT_SEQ_NUM_DIR_IN            (0x01u << USB32DEV_PROT_SEQ_NUM_DIR_Pos)
#define CY_USBSS_PROT_SEQ_NUM_VALID          (0x01u << USB32DEV_PROT_SEQ_NUM_SEQ_VALID_Pos)
#define CY_USBSS_PROT_SEQ_NUM_COMMAND_WRITE  (0x01u << USB32DEV_PROT_SEQ_NUM_COMMAND_Pos)

#define CY_USBSS_PROT_EPO_CS1_VALID    (0x01u << USB32DEV_PROT_EPO_CS1_VALID_Pos)


/* prot streamErrorStatus  */
#define CY_USBSS_PROT_STREAM_ERROR_STATUS_ERROR_DETECTED        \
    (0x01u << USB32DEV_PROT_STREAM_ERROR_STATUS_ERROR_DETECTED_Pos)


/* prot_epi_cs1 and prot_epo_cs1 related defines. */
#define CY_USBSS_PROT_EPI_CS1_VALID (0x01u << USB32DEV_PROT_EPI_CS1_VALID_Pos)
#define CY_USBSS_PROT_EPO_CS1_VALID (0x01u << USB32DEV_PROT_EPO_CS1_VALID_Pos)

#define CY_USBSS_PROT_EPI_CS1_NRDY (0x01u << USB32DEV_PROT_EPI_CS1_NRDY_Pos)
#define CY_USBSS_PROT_EPO_CS1_NRDY (0x01u << USB32DEV_PROT_EPO_CS1_NRDY_Pos)

#define CY_USBSS_PROT_EPI_CS1_STALL (0x01u << USB32DEV_PROT_EPI_CS1_STALL_Pos)
#define CY_USBSS_PROT_EPO_CS1_STALL (0x01u << USB32DEV_PROT_EPO_CS1_STALL_Pos)


#define CY_USBSS_PROT_EPI_CS1_RESET (0x01u << USB32DEV_PROT_EPI_CS1_EP_RESET_Pos)
#define CY_USBSS_PROT_EPO_CS1_RESET (0x01u << USB32DEV_PROT_EPO_CS1_EP_RESET_Pos)

/* Bring ENDP-IN CS1 endpoint fields to default state */
#define CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPI_CS1 \
                           (~(USB32DEV_PROT_EPI_CS1_VALID_Msk | \
                            USB32DEV_PROT_EPI_CS1_NRDY_Msk | \
                            USB32DEV_PROT_EPI_CS1_STALL_Msk | \
                            USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk | \
                            USB32DEV_PROT_EPI_CS1_EP_RESET_Msk | \
                            USB32DEV_PROT_EPI_CS1_ERDY_FLOWCONTROL_Msk))

#define CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPI_CS1 \
                            (0u)

/* Bring ENDP-IN CS2 endpoint fields to default state */
#define CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPI_CS2 \
                           (~(USB32DEV_PROT_EPI_CS2_EPI_TYPE_Msk | \
                             USB32DEV_PROT_EPI_CS2_SPARE_Msk | \
                             USB32DEV_PROT_EPI_CS2_MAXBURST_Msk | \
                             USB32DEV_PROT_EPI_CS2_ISOINPKS_Msk | \
                             USB32DEV_PROT_EPI_CS2_BINTERVAL_Msk))

#define CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPI_CS2 \
                           ((16 << USB32DEV_PROT_EPI_CS2_SPARE_Pos)  | \
                            (48 << USB32DEV_PROT_EPI_CS2_ISOINPKS_Pos))

/* Bring ENDP-OUT CS1 endpoint fields to default state */
#define CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPO_CS1 \
                         (~(USB32DEV_PROT_EPO_CS1_VALID_Msk | \
                          USB32DEV_PROT_EPO_CS1_NRDY_Msk | \
                          USB32DEV_PROT_EPO_CS1_STALL_Msk | \
                          USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk | \
                          USB32DEV_PROT_EPO_CS1_EP_RESET_Msk | \
                          USB32DEV_PROT_EPO_CS1_EN_PP_ZERO_DET_Msk | \
                          USB32DEV_PROT_EPO_CS1_EN_EOB_ONE_DET_Msk | \
                          USB32DEV_PROT_EPO_CS1_ERDY_FLOWCONTROL_Msk))

/* Setting EN_PP_ZERO_DET to 1 causes pre-mature wrapping up of DMA buffers. Default should
 * be zero for this bit.
 */
#define CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPO_CS1 \
                           (0u)

/* Bring ENDP-OUT CS2 endpoint fields to default state */
#define CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPO_CS2 \
                             (~(USB32DEV_PROT_EPO_CS2_EPO_TYPE_Msk | \
                              USB32DEV_PROT_EPO_CS2_SPARE_Msk |\
                              USB32DEV_PROT_EPO_CS2_MAXBURST_Msk| \
                              USB32DEV_PROT_EPO_CS2_ISOINPKS_Msk))

#define CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPO_CS2 \
                               ((16 << USB32DEV_PROT_EPI_CS2_SPARE_Pos)  | \
                               (48 << USB32DEV_PROT_EPI_CS2_ISOINPKS_Pos))

/* PTM Status related macros */
#define CY_USBSS_PTM_STAT_LDM_ENABLE_Msk        (0x01UL)
#define CY_USBSS_PTM_STAT_LDM_VALID_Msk         (0x02UL)
#define CY_USBSS_PTM_STAT_LDM_VALUE_Msk         (0xFFFF0000UL)
#define CY_USBSS_PTM_STAT_LDM_VALUE_Pos         (16UL)

/* Endpoint control register default values. */
#define PROT_EPO_CS1_DEFAULT_VAL                (0x000000C0UL)
#define PROT_EPO_CS2_DEFAULT_VAL                (0x00600040UL)
#define PROT_EPI_CS1_DEFAULT_VAL                (0x00000000UL)
#define PROT_EPI_CS2_DEFAULT_VAL                (0x00600040UL)

#define UsbBlkInitConfig                        *((volatile uint32_t *)0x40680028UL)

/* USB PHY Capability LBPM Values */
#define CY_USBSS_LBPM_PHY_CAP_MASK              (0x03UL)
#define CY_USBSS_LBPM_PHY_CAP_GEN1x1_VAL        (0x00UL)
#define CY_USBSS_LBPM_PHY_CAP_GEN2X1_VAL        (0x04UL)
#define CY_USBSS_LBPM_PHY_CAP_GEN1X2_VAL        (0x40UL)
#define CY_USBSS_LBPM_PHY_CAP_GEN2X2_VAL        (0x44UL)

#define USBSS_CAL_GET_LINK_STATE(regs)      ((regs)->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk)

#define CY_USBSS_JUMP_TO_LINK_STATE(regs, state)                                                        \
    ((regs)->LNK_LTSSM_STATE = (((state) << USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_VALUE_Pos) |        \
        USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_GO_Msk))

#define CY_USBSS_FORCE_LINK_STATE(regs, state)                                                          \
    ((regs)->LNK_LTSSM_STATE = (((state) << USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_VALUE_Pos) |        \
        USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk))




/* Delay Timer macros */
#define CY_USBSS_CAL_TCPWM_INDEX        (1UL)
#define CY_USBSS_CAL_TCPWM_INTERRUPT    (tcpwm_0_interrupts_1_IRQn)
#define CY_USBSS_CAL_TCPWM_DIV_INDEX    (2UL)
#define CY_USBSS_CAL_TCPWM_CLK          (PCLK_TCPWM0_CLOCKS1) /*CLK 1 for index 1*/
#define CY_USBSS_CAL_TIMER_RUNNING      (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING | CY_TCPWM_COUNTER_STATUS_DOWN_COUNTING)


#endif /* DOXYGEN */

/** \} group_usbfxstack_usb_common_macros */

/**
 * \addtogroup group_usbfxstack_usb_common_enums
 * \{
 */

/**
 * @typedef cy_en_usbss_pkt_type_t
 * @brief Types of USB 3.2 protocol layer packets.
 */
typedef enum
{
    CY_USBSS_PKT_TYPE_LMP = 0x00,         /**< Link Management Packet. */
    CY_USBSS_PKT_TYPE_TP  = 0x04,         /**< Transaction Packet. */
    CY_USBSS_PKT_TYPE_DPH = 0x08,         /**< Data Packet Header. */
    CY_USBSS_TP_PKT_TYPE_ITP = 0x0C       /**< Isochronous Timestamp Packet. */
}cy_en_usbss_pkt_type_t;

/**
 * @typedef cy_en_usbss_tp_pkt_subtype_t
 * @brief Types of USB 3.2 transaction packets.
 */
typedef enum
{
    CY_USBSS_TP_SUBTYPE_RES = 0,             /**< Reserved. */
    CY_USBSS_TP_SUBTYPE_ACK,                 /**< ACK TP. */
    CY_USBSS_TP_SUBTYPE_NRDY,                /**< NRDY TP. */
    CY_USBSS_TP_SUBTYPE_ERDY,                /**< ERDY TP. */
    CY_USBSS_TP_SUBTYPE_STATUS,              /**< STATUS TP. */
    CY_USBSS_TP_SUBTYPE_STALL,               /**< STALL TP. */
    CY_USBSS_TP_SUBTYPE_NOTICE,              /**< DEV_NOTIFICATION TP. */
    CY_USBSS_TP_SUBTYPE_PING,                /**< PING TP. */
    CY_USBSS_TP_SUBTYPE_PINGRSP              /**< PING RESPONSE TP. */
}cy_en_usbss_tp_pkt_subtype_t;

/**
 * @typedef cy_en_usbss_link_state_t
 * @brief List of USB 3.2 link states. This list is non-exhaustive and covers only states of interest.
 */
typedef enum
{
    CY_USBSS_LNK_STATE_SSDISABLED        = 0x00,        /**< SS.Disabled */
    CY_USBSS_LNK_STATE_RXDETECT_RES      = 0x01,        /**< Rx.Detect.Reset */
    CY_USBSS_LNK_STATE_RXDETECT_ACT      = 0x02,        /**< Rx.Detect.Active */
    CY_USBSS_LNK_STATE_RXDETECT_QUT      = 0x03,        /**< Rx.Detect.Quiet */
    CY_USBSS_LNK_STATE_SSINACT_QUT       = 0x04,        /**< SS.Inactive.Quiet */
    CY_USBSS_LNK_STATE_SSINACT_DET       = 0x05,        /**< SS.Inactive.Disconnect.Detect */
    CY_USBSS_LNK_STATE_POLLING_LFPS      = 0x08,        /**< Polling.LFPS */
    CY_USBSS_LNK_STATE_POLLING_RxEQ      = 0x09,        /**< Polling.RxEq */
    CY_USBSS_LNK_STATE_POLLING_ACT       = 0x0A,        /**< Polling.Active */
    CY_USBSS_LNK_STATE_POLLING_CFG       = 0x0B,        /**< Polling.Configuration */
    CY_USBSS_LNK_STATE_POLLING_IDLE      = 0x0C,        /**< Polling.Idle */
    CY_USBSS_LNK_STATE_POLLING_LFPSPLUS  = 0x0D,        /**< Polling.LfpsPlus */
    CY_USBSS_LNK_STATE_POLLING_PORTMATCH = 0x0E,        /**< Polling.PortMatch */
    CY_USBSS_LNK_STATE_POLLING_PORTCONF  = 0x0F,        /**< Polling.PortConfig */
    CY_USBSS_LNK_STATE_U0                = 0x10,        /**< U0 - Active state */
    CY_USBSS_LNK_STATE_U1                = 0x11,        /**< U1 */
    CY_USBSS_LNK_STATE_U2                = 0x12,        /**< U2 */
    CY_USBSS_LNK_STATE_U3                = 0x13,        /**< U3 - Suspend state */
    CY_USBSS_LNK_STATE_LPBK_ACTV         = 0x14,        /**< Loopback.Active state */
    CY_USBSS_LNK_STATE_COMP              = 0x17,        /**< Compliance */
    CY_USBSS_LNK_STATE_RECOV_ACT         = 0x18,        /**< Recovery.Active */
    CY_USBSS_LNK_STATE_RECOV_CNFG        = 0x19,        /**< Recovery.Configuration */
    CY_USBSS_LNK_STATE_RECOV_IDLE        = 0x1A,        /**< Recovery.Idle */
    CY_USBSS_LNK_STATE_ILLEGAL           = 0x1F         /**< Illegal/unknown LTSSM state. */
} cy_en_usbss_link_state_t;

/**
 * @typedef cy_en_usbss_lnk_power_mode_t
 * @brief List of USB 3.2 link power modes.
 */
typedef enum
{
    CY_USBSS_LPM_U0 = 0,                 /**< U0 (active) power state. */
    CY_USBSS_LPM_U1,                     /**< U1 power state. */
    CY_USBSS_LPM_U2,                     /**< U2 power state. */
    CY_USBSS_LPM_U3,                     /**< U3 (suspend) power state. */
    CY_USBSS_LPM_COMP,                   /**< Compliance state. */
    CY_USBSS_LPM_UNKNOWN                 /**< The link is not in any of the Ux
                                              states. This normally happens while
                                              the link training is ongoing. */
} cy_en_usbss_lnk_power_mode_t;

/**
 * @typedef cy_en_usbss_cal_lpm_cfg_t
 * @brief List of possible USB 3.x LPM accept configurations.
 */
typedef enum
{
    CY_SSCAL_LPM_DISABLED = 0,          /**< U1/U2 entry has been disabled by user. */
    CY_SSCAL_LPM_DELAYED,               /**< U1/U2 entry delayed for the instant. */
    CY_SSCAL_LPM_ENABLED                /**< U1/U2 entry is allowed. */
} cy_en_usbss_cal_lpm_cfg_t;

/**
 * @typedef cy_en_usbss_delay_event_t
 * @brief Types of events to handle after delay
 * timer expires.
 */
typedef enum
{
    DELAY_EVENT_NONE           = 0x00,          /**< No events pending. */
    DELAY_EVENT_P0_CHG         = 0x01,          /**< Handle P0 change interrupt */
    DELAY_EVENT_HOLD_POLL_ACT  = 0x02           /**< Handle Polling Active entry event */
}cy_en_usbss_delay_event_t;


/** \} group_usbfxstack_usb_common_enums */

/**
 * \addtogroup group_usbfxstack_usb_common_structs
 * \{
 */

/**
 * @brief Structure which encapsulates all information about the USB 3.2 Controller
 * Adaptation Layer (CAL) or driver for FX3G2.
 */
typedef struct cy_stc_usbss_cal_ctxt_t
{
    USB32DEV_Type      *regBase;                                        /**< USB32DEV IP base pointer. */
    USB32DEV_MAIN_Type *pMainBase;                                      /**< Pointer to USB32DEV_MAIN register section. */
    USB32DEV_PROT_Type *pProtBase;                                      /**< Pointer to USB32DEV_PROT register section. */
    USB32DEV_LNK_Type  *pLinkBase;                                      /**< Pointer to USB32DEV_LINK register section. */
    USB32DEV_EPM_Type  *pEpmBase;                                       /**< Pointer to USB32DEV_EPM register section. */
    void               *pUsbdCtxt;                                      /**< Pointer to USBD stack context. Used while sending events. */
    cy_usb_cal_msg_callback_t msgCb;                                    /**< Event callback function registered by USBD stack. */
    bool                sofEvtEnable;                                   /**< Whether ITP(SOF) interrupt is to be left enabled. */
    bool                forceLPMAccept;                                 /**< Whether host has asked us to always accept U1/U2 entry. */
    cy_en_usbss_cal_lpm_cfg_t lpmConfig;                                /**< Whether Low Power Mode transitions are disabled. */
    bool                pollingCfgDone;                                 /**< Whether link went through Polling.Configuration. */
    bool                pollingLFPSDone;                                /**< Whether link went through Polling.LFPS. */
    bool                linkSuspended;                                  /**< Whether the USB3 link has been suspended (in U3) */
    bool                inCompState;                                    /**< Whether LTSSM is in compliance state. */
    uint8_t             currCompPattern;                                /**< Current compliance pattern being sent. */
    uint8_t             gen2_ebdepth;                                   /**< Elastic Buffer half-depth to be used in Gen2 connection. */
    uint32_t            lastLinkState;                                  /**< The last logged link state. */
    uint32_t            prevLinkState;                                  /**< The last valid link state */
    uint32_t            ptmStatus;                                      /**< PTM STATUS response for GET_STATUS command */
    uint16_t            isochDelay;                                     /**< Isoch delay retrieved from host via SET_ISOC request */
    uint8_t             activePhyIdx;                                   /**< USB-SS PHY index used in current connection. */
    bool                gen2Enabled;                                    /**< Whether USB connection is being run in Gen2 mode. */
    bool                dualLaneEnabled;                                /**< Whether dual USB lanes are enabled. */
    bool                rxAdaptationDone[FX3G2_MAX_NUM_USB_PHY];        /**< Whether receiver adaptation has been completed since last check. */
    uint8_t             phyLutIndex[FX3G2_MAX_NUM_USB_PHY];             /**< The chosen PHY LUT index. */
    bool                connectRcvd;                                    /**< Whether connect interrupt has been received. */
    bool                uxExitActive;                                   /**< Whether Ux exit is ongoing. */
    bool                lpmWarningDone;                                 /**< Whether LPM enable related warning has been printed already. */
    uint8_t             u2TimeoutVal;                                   /**< U2 inactivity timeout value. */
    bool                phyDisabled;                                    /**< Whether PHY has been disabled as part of U3 processing. */
    bool                deepSleepEntered;                               /**< Whether deep sleep was entered. */
    bool                devLpmExitDisabled;                             /**< Whether device initiated exit from LPM states is disabled. */
    cy_en_hbdma_clk_freq_t desiredDmaFreq;                              /**< Desired DMA clock frequency. */
    cy_en_hbdma_clk_freq_t currentDmaFreq;                              /**< Actual DMA clock frequency. */
    uint8_t             phyResult[FX3G2_MAX_NUM_USB_PHY][16];           /**< CTLE equalization algorithm error code for each setting. */
    uint32_t            phyEyeHeight[FX3G2_MAX_NUM_USB_PHY][16];        /**< Eye height recorded for each of the equalizer settings. */
    uint32_t            txAfeSettings[FX3G2_MAX_NUM_USB_PHY][8];        /**< Configuration for the PHY transmitter slices. */
    uint32_t            lbadCounter;                                    /**< Count of LBAD interrupts received. */
    uint8_t             numPhyInitialized;                              /**< Count of PHY instances initialized. */
    uint8_t             lgoodTimeoutCount;                              /**< Count of LGOOD timeout events. */
    uint16_t            lpmExitLfpsDelay;                               /**< Expected LPM exit LFPS TX duration. */
    bool                gen1x1FallbackEn;                               /**< Enable fallback to Gen1x1 if the Host/Hub does not support fallback to x1 speeds */
    bool                compExitDone;                                   /**< Whether Compliance state has been exited. */
    uint32_t            compExitTS;                                     /**< Time stamp when compliance state was exited. */
    uint16_t            activeLutMask;                                  /**< Active CTLE LUT mask value. */
    bool                stopClkOnEpResetEnable;                         /**< Enable stopping clock during EP reset. */
    cy_en_usb_config_lane_t usbConfigLane;                              /**< USB configuration lane selection. */
    bool                gen1SpeedForced;                                /**< Driver has forced Gen1 speed in compliance. */
    bool                inGen1DualMode;                                 /**< Whether link was up in Gen1x2 mode. */
    cy_en_usbss_delay_event_t delayEventMask;                           /**< Event to handle after timer expiry */
}cy_stc_usbss_cal_ctxt_t;

/** \} group_usbfxstack_usb_common_structs */

/**
 * \addtogroup group_usbfxstack_usb_common_functions
 * \{
 */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Init
****************************************************************************//**
*
* Initializes the USBSS IP and relevant variables.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pUsbdCtxt
* The pointer to the USBD context structure.
*
* \param cb
* Callback function registered by the user \ref cy_usb_cal_msg_callback_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_Init
                                (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 void *pUsbdCtxt, cy_usb_cal_msg_callback_t cb);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Connect
****************************************************************************//**
*
* Enables the USB3 RX terminations for connecting to the host and enables
* relevant USB LINK and PROTOCOL interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param usbSpeed
* The desired speed at which the device needs to be connected to the host
* \ref cy_en_usb_speed_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_Connect
                                (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 cy_en_usb_speed_t usbSpeed);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SendMsg
****************************************************************************//**
*
* Sends message to the USBD layer using a callback registered by the user. This
* function should call only ISR safe routine.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pMsg
* Message structure pointer.
*
* \return
* Whether context switch is required at the end of the ISR.
*
*******************************************************************************/
bool Cy_USBSS_Cal_SendMsg(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                          void *pMsg);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_IntrHandler
****************************************************************************//**
*
* Interrupt Handler function for USB3 IP.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_IntrHandler(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_WakeupIntrHandler
****************************************************************************//**
*
* Wakeup Interrupt Handler function for USB3 IP.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_WakeupIntrHandler(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Get_PtmStatus()
****************************************************************************//**
*
* Sends the PTM STATUS to the USBD layer to repsond to GET_STATUS request.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.

* \return uint32_t
* The PTM status .
*
*******************************************************************************/
uint32_t Cy_USBSS_Cal_Get_PtmStatus(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_LPMEnable
****************************************************************************//**
*
* Enables the USB3 Link layer Low Power Mode Handling. The LINK layer can accept
* going into U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param isResume
* Whether delayed LPM entry is being resumed.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_LPMEnable(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                bool isResume);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ForceLPMAccept
****************************************************************************//**
*
* Enables the USB3 Link layer to always accept LPM requests going into
* U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* If true, the device accept LPM requests always.
* If false, the device may accept or reject LPM requests.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_ForceLPMAccept
                                (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 bool enable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_LPMDisable
****************************************************************************//**
*
* Enables the USB3 Link layer to disable accepting LPM requests going into
* U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_LPMDisable(cy_stc_usbss_cal_ctxt_t *pCalCtxt);


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetLinkPowerState
****************************************************************************//**
*
* Forces the USB3 Link layer to a particular Link Power State by sending
* LGO_Ux requests.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param lnkMode
* Link Mode that needs to be set from \ref cy_en_usbss_lnk_power_mode_t
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetLinkPowerState
                                (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 cy_en_usbss_lnk_power_mode_t lnkMode);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetUsbLinkActive
****************************************************************************//**
*
* Get the USB3 Link layer to U0 State if it is in U1/U2 state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_GetUsbLinkActive(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetLinkPowerState
****************************************************************************//**
*
* Retrieves the Link Power State of the device.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param *pMode
* Pointer to a memory where the link power state will be stored
*  from \ref cy_en_usbss_lnk_power_mode_t
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_GetLinkPowerState
                                (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 cy_en_usbss_lnk_power_mode_t *pMode);

/* Transaction Packet related functions. */
/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendTp
****************************************************************************//**
*
* This function will send transaction packet. It is responsibility of caller to
* prepare transaction packet and call this function.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param *pTpData
* Pointer to a memory where transaction packet stored.
*
*******************************************************************************/
void Cy_USBSS_Cal_ProtSendTp(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint32_t *pTpData);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendAckTp
****************************************************************************//**
*
* This function prepares and send ACK transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param numP
* Number of Data packet buffer receiever can accept.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_ProtSendAckTp
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNum,
                                           cy_en_usb_endp_dir_t endpDir,
                                           uint8_t numP, uint16_t bulkStream);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendErdyTp
****************************************************************************//**
*
* This function prepares and send ERDY transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param numP
* Number of Data packet buffer receiever can accept.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_ProtSendErdyTp
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNum,
                                           cy_en_usb_endp_dir_t endpDir,
                                           uint8_t numP, uint16_t bulkStream);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendNrdyTp
****************************************************************************//**
*
* This function prepares and send NRDY transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_ProtSendNrdyTp
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNum,
                                           cy_en_usb_endp_dir_t endpDir,
                                           uint16_t bulkStream);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpReset
****************************************************************************//**
*
* Function to reset an endpoint. This clears sticky bits e.g retry bit,
* flowcontrol bit.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpReset
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint32_t endpNum,
                                         cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetSeqNum
****************************************************************************//**
*
* This function gets the current value of sequence number for an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param pSeqNum
* Current value of sequence number will be store here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_GetSeqNum
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNum,
                                           cy_en_usb_endp_dir_t endpDir,
                                           uint8_t *pSeqNum);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetSeqNum
****************************************************************************//**
*
*  This function sets value of sequence number in hardware for an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param seqNum
* Sequence number need to be written in HW register.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetSeqNum
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNum,
                                           cy_en_usb_endp_dir_t endpDir,
                                           uint8_t seqNum);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetDevAddress
****************************************************************************//**
*
*  This function will get device address assigned by Host.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pDevAddr
* device address will be stored here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_GetDevAddress
                                          (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                           uint8_t *pDevAddr);


/* Endpoint/EPM related functions  */
/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetClearStall
****************************************************************************//**
*
* This function enable or disable NRDY condition in hw. By setting NRDY bit,
* endpoint will keep sending NRDY till NRDY bit is cleared.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpSetClearStall
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint32_t endpNum,
                                         cy_en_usb_endp_dir_t endpDir,
                                         bool setClear);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetClearNrdy
****************************************************************************//**
*
* This function enable or disable NRDY condition in hw. By setting NRDY bit,
* endpoint will keep sending NRDY till NRDY bit is cleared.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpSetClearNrdy
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint32_t endpNum,
                                         cy_en_usb_endp_dir_t endpDir,
                                         bool setClear);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetClearNRDYALL
****************************************************************************//**
*
* This function enable or disable NRDY condition for all endpopint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetClearNrdyAll
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         bool setClear);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpConfig
****************************************************************************//**
*
* This function handles configuration of endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param configParam
* Configuration Parameters stored here \ref cy_stc_usb_endp_config_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpConfig
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        cy_stc_usb_endp_config_t configParam);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetPktsPerBuffer
****************************************************************************//**
*
* This function sets the number of maximum sized packets that can fit into a
* single DMA buffer for an Ingress endpoint. This needs to be called in cases
* where the maximum packet size is not a power of 2, so that the EPM can operate
* correctly.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param pktsPerBuffer
* packats per buffer.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpSetPktsPerBuffer
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        uint8_t pktsPerBuffer);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpMapStream
****************************************************************************//**
*
* This function maps an unused USB Ingress/Egress socket to the specified
* stream associated with a bulk endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param endpDirection
* Endpoint direction.
*
* \param streamId
* stream ID.
*
* \param socketNum
* Socket Number.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EndpMapStream
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        uint16_t streamId,
                                        uint32_t socketNum);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpUnmapStream
****************************************************************************//**
*
* This function unmaps the DMA socket to stream mapping for a bulk endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param socketNum
* Socket Number.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpUnmapStream(cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir,
                            uint32_t socketNum);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableEndp
****************************************************************************//**
*
* This function enables/disable endpoint and set/reset respective interrupt.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param endpDirection
* Endpoint direction.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableEndp
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        bool enable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetEpBurstMode
****************************************************************************//**
*
* This function enables/disables the MULT (allow burst across multiple DMA buffers)
* feature for the specified USB endpoints. Enabling the feature can improve the data
* transfer throughput on the respective endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param endpDirection
* Endpoint direction.
*
* \param enable
* true for enable burst mode and false for disable burst mode.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetEpBurstMode
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        bool enable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetEndpRetryOffset
****************************************************************************//**
*
* This function sets the egress endpoint retry buffer offset for Type-2
* (CTRL and BULK) endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param retryBufOffset
* Retry buffer offset.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetEndpRetryOffset
                                       (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        uint16_t retryBufOffset);

/* Data Transfer related functions  */
/*******************************************************************************
* Function Name: Cy_USBSS_Cal_UpdateXferCount
****************************************************************************//**
*
* This function updates xfer count in an ndpoint register.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param xferCount
* transfer count value stored here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_UpdateXferCount
                                           (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                            uint32_t endpNum,
                                            cy_en_usb_endp_dir_t endpDir,
                                            uint32_t xferCount);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableStatusCtrl
****************************************************************************//**
*
* This function enable/disable Status Control feature mentioned in PROT_CS
* register.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableStatusCtrl
                                           (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                            bool enable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ClearStatusClrBusy
****************************************************************************//**
*
* This function write "1" to STATUS_CLR_BUSY which clears  the bit to initiate
* STATUS response.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_ClearStatusClrBusy
                                           (cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SendAckSetupDataStatusStage
****************************************************************************//**
*
* This function enable HW to send ACK in status stage.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SendAckSetupDataStatusStage(
                                            cy_stc_usbss_cal_ctxt_t *pCalCtxt);

#ifndef DOXYGEN
#define Cy_USBSS_Cal_SendACkSetupDataStatusStage(pCtxt) Cy_USBSS_Cal_SendAckSetupDataStatusStage(pCtxt)
#endif /* DOXYGEN */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushEPM
****************************************************************************//**
*
* This function will flush Ingress and egress both the EPM.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param force
* Whether flush should be forcibly done even if pointers are zero.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_FlushEPM(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                               bool force);


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushEndpSocket
****************************************************************************//**
*
* This function will flush socket associated with an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_FlushEndpSocket
                                            (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                             uint32_t endpNum,
                                             cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushAllEndpSocket
****************************************************************************//**
*
* This function will flush socket associated with all endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_FlushAllEndpSocket(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableMainIntr
****************************************************************************//**
*
* This function will enable/disable main interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableMainIntr
                                            (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                             bool EnableDisable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableLinkIntr
****************************************************************************//**
*
* This function will enable/disable link interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableLinkIntr
                                            (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                             bool EnableDisable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableProtIntr
****************************************************************************//**
*
* This function will enable/disable protocol interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableProtIntr
                                            (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                             bool EnableDisable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpIsNakNrdySet
****************************************************************************//**
*
* This function checks whether the specified endpoint is currently in the NRDYed
* state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* Whether endpoint is currently NRDYed. True means NRDYed state.
*
*******************************************************************************/
bool Cy_USBSS_Cal_EndpIsNakNrdySet(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                   uint32_t endpNum, cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpIsStallSet
****************************************************************************//**
*
* This function checks whether the specified endpoint is currently in the STALLed
* state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNumber
* Endpoint number.
*
* \param endpDirection
* Endpoint direction.
*
* \return
* Whether endpoint is currently STALLed. True means STALLed state.
*
*******************************************************************************/
bool Cy_USBSS_Cal_EndpIsStallSet(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 uint32_t endpNumber,
                                 cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableSsDevice
****************************************************************************//**
*
* This function enable/disable USB device in SS mode.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_EnableSsDevice
                                            (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                             bool enable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DisConnect
****************************************************************************//**
*
*  This function handles disconnect device at CAL layer.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_DisConnect(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_PTMConfig
****************************************************************************//**
*
* This function handles PTM related configuration.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param ptmControl
* Set to 1 for enabling LDM & PTM. Set to 0 for disabling LDM and reset PTM logic.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_PTMConfig(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                bool ptmControl);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnterLinkCompliance
****************************************************************************//**
*
* This function performs the actions required when the USB LTSSM enters the
* Compliance state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_EnterLinkCompliance(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_NextCompliancePattern
****************************************************************************//**
*
* This function causes the LTSSM to move to the next compliance pattern
* once a Ping.LFPS has been detected.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_NextCompliancePattern(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ExitLinkCompliance
****************************************************************************//**
*
* This function performs the required actions when the LTSSM exits the Compliance
* state due to a Warm Reset.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
void Cy_USBSS_Cal_ExitLinkCompliance(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetGen2EBDepth
****************************************************************************//**
*
* Function to update the Elastic Buffer Half Depth setting to be used in Gen2
* USB connection.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param gen2_ebdepth
* Elastic Buffer Half Depth setting: Valid range from 4 to 16.
*
*******************************************************************************/
void Cy_USBSS_Cal_SetGen2EBDepth(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 uint8_t   gen2_ebdepth);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_IsEnabled
****************************************************************************//**
*
* Indicates whether USB 3.x connection is enabled on the device.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* true if connection is enabled, false otherwise.
*
*******************************************************************************/
bool Cy_USBSS_Cal_IsEnabled(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_MeasureCCVoltage
****************************************************************************//**
*
* Measure the voltage on CC line and return the corresponding ADC reading.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param cc2select
* true to measure CC2 voltage, false for CC1.
*
* \return
* ADC reading equivalent to the CC voltage.
*******************************************************************************/
uint8_t
Cy_USBSS_Cal_MeasureCCVoltage(cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool cc2select);

/*******************************************************************************
 * Function Name: Cy_USBSS_Cal_SetDmaClkFreq
 ****************************************************************************//**
 *
 * Function which sets the desired DMA clock frequency on the FX3G2 device.
 * This function should be called before USB connection is enabled. The default
 * clock selected is 240 MHz derived from USB2 PLL.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
 * allocated by the user.
 *
 * \param dmaFreq
 * Desired DMA clock frequency.
 *
 * \return
 * Status code
 *******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetDmaClkFreq(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                    cy_en_hbdma_clk_freq_t dmaFreq);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DeepSleepPrep
****************************************************************************//**
*
* This function prepares the USB32DEV controller IP block for entry into deep
* sleep state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* true if application can proceed with deep sleep entry, false otherwise.
*******************************************************************************/
bool Cy_USBSS_Cal_DeepSleepPrep(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DeepSleepExit
****************************************************************************//**
*
* This function restores the USB32DEV controller IP block state after USB
* link has resumed from U3 (suspend) state. This function is called from the
* driver internally and not intended to be called directly.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*******************************************************************************/
void Cy_USBSS_Cal_DeepSleepExit(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DisableLPMDeviceExit
****************************************************************************//**
*
* Function to disable support for device initiated exit from USB low power modes
* completely.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param devExitDisable
* true to disable device initiated exit from LPM states.
*******************************************************************************/
void
Cy_USBSS_Cal_DisableLPMDeviceExit(cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool devExitDisable);

/*******************************************************************************
 * Function Name: Cy_USBSS_Cal_InitEventLog
 ****************************************************************************//**
 *
 * Enables capture of USBSS CAL event and state changes into RAM buffer. This
 * function is intended to be called by USBD and not directly from the
 * application.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
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
 * The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
 *******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_InitEventLog(cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                   uint32_t *pEvtLogBuf,
                                                   uint16_t  evtLogSize);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_PostEpEnable
****************************************************************************//**
*
* Function to reset egress socket EOB detection function after endpoints have
* been enabled.
*
* \param pCalCtxt
* USBSS Controller context structure.
*******************************************************************************/
void Cy_USBSS_Cal_PostEpEnable(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ReleaseLTSSM
****************************************************************************//**
*
* Function to release the LTSSM from a forced state.
*
* \param pCalCtxt
* USBSS Controller context structure.
*******************************************************************************/
void Cy_USBSS_Cal_ReleaseLTSSM(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ClkStopOnEpRstEnable
****************************************************************************//**
*
* Function to enable stop/restart of USB block clock during EP reset. This functionality
* can be enabled to ensure all state related to an endpoint is cleared properly. A
* possible side effect is that the USB link goes through recovery cycles during the
* Endpoint Reset operation.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param clkStopEn
* Whether to enable or disable the clock stop control.
*******************************************************************************/
void Cy_USBSS_Cal_ClkStopOnEpRstEnable(cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool clkStopEn);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SelectConfigLane
****************************************************************************//**
*
* Function to select the USB 3.x Configuration Lane. This API can be used in cases
* where the USB connection orientation is being detected externally instead of
* through CC1/CC2 voltage measurement by the controller itself.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param laneSel
* Desired configuration lane selection.
*******************************************************************************/
void Cy_USBSS_Cal_SelectConfigLane(cy_stc_usbss_cal_ctxt_t *pCalCtxt, cy_en_usb_config_lane_t laneSel);

/*******************************************************************************
* Function name: Cy_USBSS_Cal_ITPIntrUpdate
****************************************************************************//**
*
* Function to enable or disable the ITP interrupt based on user requirement.
*
* \param pCalCtxt
* CAL layer library context pointer.
*
* \param itpIntrEnable
* Whether the ITP interrupt should be enabled.
*
*******************************************************************************/
void Cy_USBSS_Cal_ITPIntrUpdate(cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool itpIntrEnable);

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetTemperatureReading
****************************************************************************//**
*
* Get the temperature sensitive reading from ADC in the USB block.
*
* \note
* The procedure to convert the ADC reading to actual temperature is TBD.
* The ADC reading will reduce by 1 point for a rise of approximately 5 degrees
* in temperature.
*
* \param pCalCtxt
* USBSS Controller context structure.
*
* \return
* ADC reading equivalent to die temperature.
*******************************************************************************/
uint8_t Cy_USBSS_Cal_GetTemperatureReading(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

#ifndef DOXYGEN

/* Debug function used during Silicon bring-up: To be removed. */
void Cy_USBSS_Cal_PrintCtleResults(cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t phyIndex);

#endif /* DOXYGEN */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_TimerISR
****************************************************************************//**
*
* Function that handles the delay timer expiry based on the event
* This function should be called by the user when the TCPWM ISR is
* invoked.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/

void Cy_USBSS_Cal_TimerISR(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

/** \} group_usbfxstack_usb_common_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_USBSS_CAL_DRV_H */

/** \} group_usbfxstack_usb_common */

/* End of file */
