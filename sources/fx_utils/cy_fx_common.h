/***************************************************************************//**
* \file cy_fx_common.h
* \version 1.0
*
* Provides macros and definitions associated with the FX common utility functions.
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

#ifndef _CY_FX_COMMON_H_

/** Indicates the use of cy_fx_common */
#define _CY_FX_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_enums
 * \{
 */

/**
 * @typedef cy_en_fxusb_dbg_func_sel_t
 * @brief List of EZ-USB FX debug functions which can be routed to the Digital
 * Design for Test (DDFT) IOs of the device.
 */
typedef enum {
    CY_FX_DBG_FUNC_NONE                       = 0x0000,         /**< No debug function selected. */

    CY_FX_DBG_FUNC_USBPHY0_RXEQ_TRAINING      = 0x0002,         /**< USB PHY0 in RxEq training. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RATE_10G      = 0x0003,         /**< USB PHY0 PIPE data rate set to 10Gbps. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RX_TERM       = 0x0004,         /**< USB PHY0 RX termination enable. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RX_ELEC_IDLE  = 0x0005,         /**< USB PHY0 PIPE RX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_TX_ELEC_IDLE  = 0x0006,         /**< USB PHY0 PIPE TX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_CLK_EN        = 0x0007,         /**< USB PHY0 PIPE clock enable. */
    CY_FX_DBG_FUNC_USBPHY0_P3_ENTRY           = 0x0008,         /**< USB PHY0 P3 entry PHY status. */
    CY_FX_DBG_FUNC_USBPHY0_P3_EXIT            = 0x0009,         /**< USB PHY0 P3 exit PHY status. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P0           = 0x000A,         /**< USB PHY0 in P0 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P1           = 0x000B,         /**< USB PHY0 in P1 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P2           = 0x000C,         /**< USB PHY0 in P2 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P3           = 0x000D,         /**< USB PHY0 in P3 state. */
    CY_FX_DBG_FUNC_USBPHY0_TX_SER_EN          = 0x000E,         /**< USB PHY0 Transmit Serialiser enable. */
    CY_FX_DBG_FUNC_USBPHY0_TX_ELEC_IDLE       = 0x000F,         /**< USB PHY0 Transmit Electrical Idle. */
    CY_FX_DBG_FUNC_USBPHY0_TX_LFPS_EN         = 0x0010,         /**< USB PHY0 LFPS Transmit enable. */
    CY_FX_DBG_FUNC_USBPHY0_TX_HS_CLKDRV_EN    = 0x0011,         /**< USB PHY0 Transmit HS clock driver enable. */
    CY_FX_DBG_FUNC_USBPHY0_CDR_LOCK_TO_REF    = 0x0012,         /**< USB PHY0 CDR in Lock to Reference mode. */
    CY_FX_DBG_FUNC_USBPHY0_PROT_LPBK_EN       = 0x0013,         /**< USB PHY0 Protocol level loopback enabled. */
    CY_FX_DBG_FUNC_USBPHY0_TX_PULLUP_EN       = 0x0014,         /**< USB PHY0 TX pull-up enable for Rx Detection. */
    CY_FX_DBG_FUNC_USBPHY0_TX_RXDET_OP        = 0x0015,         /**< USB PHY0 Rx Detection Output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_PRESENT         = 0x0016,         /**< USB PHY0 Receiver Present status. */
    CY_FX_DBG_FUNC_USBPHY0_AFE_ZCAL_COMP_OUT  = 0x0017,         /**< USB PHY0 TX impedance calibration output. */
    CY_FX_DBG_FUNC_USBPHY0_WARM_RST_DET       = 0x0018,         /**< USB PHY0 detecting Warm reset. */
    CY_FX_DBG_FUNC_USBPHY0_MASK_LFPS_DET      = 0x0019,         /**< USB PHY0 LFPS detect status masked. */
    CY_FX_DBG_FUNC_USBPHY0_HS_DATA_PRESENT    = 0x001A,         /**< USB PHY0 High Speed Data Present output. */
    CY_FX_DBG_FUNC_USBPHY0_HS_DATA_IN_U0      = 0x001B,         /**< USB PHY0 HS data in U0 state. */
    CY_FX_DBG_FUNC_USBPHY0_RX_HUNT_FOR_ALIGN  = 0x001C,         /**< USB PHY0 receiver hunting for alignment. */
    CY_FX_DBG_FUNC_USBPHY0_RX_ALIGN_FORCED    = 0x001D,         /**< USB PHY0 receiver data alignment forced. */
    CY_FX_DBG_FUNC_USBPHY0_G1_ELBUF_UNDERFLOW = 0x001E,         /**< USB PHY0 Gen1 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY0_G1_ELBUF_OVERFLOW  = 0x001F,         /**< USB PHY0 Gen1 Elastic Buffer Overflow. */
    CY_FX_DBG_FUNC_USBPHY0_RX_ALIGNED         = 0x0020,         /**< USB PHY0 Receiver Aligned status. */
    CY_FX_DBG_FUNC_USBPHY0_RX_LOCKED          = 0x0021,         /**< USB PHY0 Receiver Locked status. */
    CY_FX_DBG_FUNC_USBPHY0_G1_8B10B_DISP      = 0x0022,         /**< USB PHY0 active Gen1 8B10B Disparity. */
    CY_FX_DBG_FUNC_USBPHY0_G2_ELBUF_UNDERFLOW = 0x0023,         /**< USB PHY0 Gen2 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY0_G3_ELBUF_OVERFLOW  = 0x0024,         /**< USB PHY0 Gen2 Elastic Buffer Overflow. */
    CY_FX_DBG_FUNC_USBPHY0_ADC_COMP_OUT       = 0x0025,         /**< USB PHY0 ADC comparator output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT     = 0x0081,         /**< USB PHY0 Receiver LFPS detector output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_SIGNAL_LOCK     = 0x0083,         /**< USB PHY0 Receiver Signal lock status. */
    CY_FX_DBG_FUNC_USBPHY0_RX_AFE_OSA_DONE    = 0x0085,         /**< USB PHY0 AFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY0_RX_FILTERED_LOCK   = 0x008B,         /**< USB PHY0 Receiver signal lock post filtering. */
    CY_FX_DBG_FUNC_USBPHY0_RX_DFE_OSA_DONE    = 0x008D,         /**< USB PHY0 DFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY0_RX_REFCLK_BY_2     = 0x008E,         /**< USB PHY0 reference clock divided by 2. */

    CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING      = 0x0102,         /**< USB PHY1 in RxEq training. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RATE_10G      = 0x0103,         /**< USB PHY1 PIPE data rate set to 10Gbps. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RX_TERM       = 0x0104,         /**< USB PHY1 RX termination enable. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RX_ELEC_IDLE  = 0x0105,         /**< USB PHY1 PIPE RX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_TX_ELEC_IDLE  = 0x0106,         /**< USB PHY1 PIPE TX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_CLK_EN        = 0x0107,         /**< USB PHY1 PIPE clock enable. */
    CY_FX_DBG_FUNC_USBPHY1_P3_ENTRY           = 0x0108,         /**< USB PHY1 P3 entry PHY status. */
    CY_FX_DBG_FUNC_USBPHY1_P3_EXIT            = 0x0109,         /**< USB PHY1 P3 exit PHY status. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P0           = 0x010A,         /**< USB PHY1 in P0 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P1           = 0x010B,         /**< USB PHY1 in P1 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P2           = 0x010C,         /**< USB PHY1 in P2 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P3           = 0x010D,         /**< USB PHY1 in P3 state. */
    CY_FX_DBG_FUNC_USBPHY1_TX_SER_EN          = 0x010E,         /**< USB PHY1 Transmit Serialiser enable. */
    CY_FX_DBG_FUNC_USBPHY1_TX_ELEC_IDLE       = 0x010F,         /**< USB PHY1 Transmit Electrical Idle. */
    CY_FX_DBG_FUNC_USBPHY1_TX_LFPS_EN         = 0x0110,         /**< USB PHY1 LFPS Transmit enable. */
    CY_FX_DBG_FUNC_USBPHY1_TX_HS_CLKDRV_EN    = 0x0111,         /**< USB PHY1 Transmit HS clock driver enable. */
    CY_FX_DBG_FUNC_USBPHY1_CDR_LOCK_TO_REF    = 0x0112,         /**< USB PHY1 CDR in Lock to Reference mode. */
    CY_FX_DBG_FUNC_USBPHY1_PROT_LPBK_EN       = 0x0113,         /**< USB PHY1 Protocol level loopback enabled. */
    CY_FX_DBG_FUNC_USBPHY1_TX_PULLUP_EN       = 0x0114,         /**< USB PHY1 TX pull-up enable for Rx Detection. */
    CY_FX_DBG_FUNC_USBPHY1_TX_RXDET_OP        = 0x0115,         /**< USB PHY1 Rx Detection Output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_PRESENT         = 0x0116,         /**< USB PHY1 Receiver Present status. */
    CY_FX_DBG_FUNC_USBPHY1_AFE_ZCAL_COMP_OUT  = 0x0117,         /**< USB PHY1 TX impedance calibration output. */
    CY_FX_DBG_FUNC_USBPHY1_WARM_RST_DET       = 0x0118,         /**< USB PHY1 detecting Warm reset. */
    CY_FX_DBG_FUNC_USBPHY1_MASK_LFPS_DET      = 0x0119,         /**< USB PHY1 LFPS detect status masked. */
    CY_FX_DBG_FUNC_USBPHY1_HS_DATA_PRESENT    = 0x011A,         /**< USB PHY1 High Speed Data Present output. */
    CY_FX_DBG_FUNC_USBPHY1_HS_DATA_IN_U0      = 0x011B,         /**< USB PHY1 HS data in U0 state. */
    CY_FX_DBG_FUNC_USBPHY1_RX_HUNT_FOR_ALIGN  = 0x011C,         /**< USB PHY1 receiver hunting for alignment. */
    CY_FX_DBG_FUNC_USBPHY1_RX_ALIGN_FORCED    = 0x011D,         /**< USB PHY1 receiver data alignment forced. */
    CY_FX_DBG_FUNC_USBPHY1_G1_ELBUF_UNDERFLOW = 0x011E,         /**< USB PHY1 Gen1 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY1_G1_ELBUF_OVERFLOW  = 0x011F,         /**< USB PHY1 Gen1 Elastic Buffer Overflow. */
    CY_FX_DBG_FUNC_USBPHY1_RX_ALIGNED         = 0x0120,         /**< USB PHY1 Receiver Aligned status. */
    CY_FX_DBG_FUNC_USBPHY1_RX_LOCKED          = 0x0121,         /**< USB PHY1 Receiver Locked status. */
    CY_FX_DBG_FUNC_USBPHY1_G1_8B10B_DISP      = 0x0122,         /**< USB PHY1 active Gen1 8B10B Disparity. */
    CY_FX_DBG_FUNC_USBPHY1_G2_ELBUF_UNDERFLOW = 0x0123,         /**< USB PHY1 Gen2 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY1_G3_ELBUF_OVERFLOW  = 0x0124,         /**< USB PHY1 Gen2 Elastic Buffer Overflow. */
    CY_FX_DBG_FUNC_USBPHY1_ADC_COMP_OUT       = 0x0125,         /**< USB PHY1 ADC comparator output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT     = 0x0181,         /**< USB PHY1 Receiver LFPS detector output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_SIGNAL_LOCK     = 0x0183,         /**< USB PHY1 Receiver Signal lock status. */
    CY_FX_DBG_FUNC_USBPHY1_RX_AFE_OSA_DONE    = 0x0185,         /**< USB PHY1 AFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY1_RX_FILTERED_LOCK   = 0x018B,         /**< USB PHY1 Receiver signal lock post filtering. */
    CY_FX_DBG_FUNC_USBPHY1_RX_DFE_OSA_DONE    = 0x018D,         /**< USB PHY1 DFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY1_RX_REFCLK_BY_2     = 0x018E,         /**< USB PHY1 reference clock divided by 2. */

    CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE       = 0x0204,         /**< USB Egress Socket #0 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK1_ACTIVE       = 0x0205,         /**< USB Egress Socket #1 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK2_ACTIVE       = 0x0206,         /**< USB Egress Socket #2 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK3_ACTIVE       = 0x0207,         /**< USB Egress Socket #3 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK4_ACTIVE       = 0x0208,         /**< USB Egress Socket #4 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK5_ACTIVE       = 0x0209,         /**< USB Egress Socket #5 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK6_ACTIVE       = 0x020A,         /**< USB Egress Socket #6 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK7_ACTIVE       = 0x020B,         /**< USB Egress Socket #7 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK8_ACTIVE       = 0x020C,         /**< USB Egress Socket #8 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK9_ACTIVE       = 0x020D,         /**< USB Egress Socket #9 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK10_ACTIVE      = 0x020E,         /**< USB Egress Socket #10 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK11_ACTIVE      = 0x020F,         /**< USB Egress Socket #11 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK12_ACTIVE      = 0x0210,         /**< USB Egress Socket #12 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK13_ACTIVE      = 0x0211,         /**< USB Egress Socket #13 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK14_ACTIVE      = 0x0212,         /**< USB Egress Socket #14 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK15_ACTIVE      = 0x0213,         /**< USB Egress Socket #15 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK0_ACTIVE       = 0x0214,         /**< USB Ingress Socket #0 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK1_ACTIVE       = 0x0215,         /**< USB Ingress Socket #1 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK2_ACTIVE       = 0x0216,         /**< USB Ingress Socket #2 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK3_ACTIVE       = 0x0217,         /**< USB Ingress Socket #3 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK4_ACTIVE       = 0x0218,         /**< USB Ingress Socket #4 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK5_ACTIVE       = 0x0219,         /**< USB Ingress Socket #5 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK6_ACTIVE       = 0x021A,         /**< USB Ingress Socket #6 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK7_ACTIVE       = 0x021B,         /**< USB Ingress Socket #7 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK8_ACTIVE       = 0x021C,         /**< USB Ingress Socket #8 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK9_ACTIVE       = 0x021D,         /**< USB Ingress Socket #9 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK10_ACTIVE      = 0x021E,         /**< USB Ingress Socket #10 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK11_ACTIVE      = 0x021F,         /**< USB Ingress Socket #11 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK12_ACTIVE      = 0x0220,         /**< USB Ingress Socket #12 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK13_ACTIVE      = 0x0221,         /**< USB Ingress Socket #13 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK14_ACTIVE      = 0x0222,         /**< USB Ingress Socket #14 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK15_ACTIVE      = 0x0223,         /**< USB Ingress Socket #15 active. */
    CY_FX_DBG_FUNC_USBSS_EP0_T1_SCH_RDY       = 0x0224,         /**< USB Endpoint 0 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP1_T1_SCH_RDY       = 0x0225,         /**< USB Endpoint 1 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP2_T1_SCH_RDY       = 0x0226,         /**< USB Endpoint 2 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP3_T1_SCH_RDY       = 0x0227,         /**< USB Endpoint 3 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP4_T1_SCH_RDY       = 0x0228,         /**< USB Endpoint 4 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP5_T1_SCH_RDY       = 0x0229,         /**< USB Endpoint 5 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP6_T1_SCH_RDY       = 0x022A,         /**< USB Endpoint 6 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP7_T1_SCH_RDY       = 0x022B,         /**< USB Endpoint 7 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP8_T1_SCH_RDY       = 0x022C,         /**< USB Endpoint 8 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP9_T1_SCH_RDY       = 0x022D,         /**< USB Endpoint 9 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP10_T1_SCH_RDY      = 0x022E,         /**< USB Endpoint 10 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP11_T1_SCH_RDY      = 0x022F,         /**< USB Endpoint 11 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP12_T1_SCH_RDY      = 0x0230,         /**< USB Endpoint 12 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP13_T1_SCH_RDY      = 0x0231,         /**< USB Endpoint 13 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP14_T1_SCH_RDY      = 0x0232,         /**< USB Endpoint 14 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP15_T1_SCH_RDY      = 0x0233,         /**< USB Endpoint 15 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP0_T2_SCH_RDY       = 0x0234,         /**< USB Endpoint 0 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP1_T2_SCH_RDY       = 0x0235,         /**< USB Endpoint 1 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP2_T2_SCH_RDY       = 0x0236,         /**< USB Endpoint 2 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP3_T2_SCH_RDY       = 0x0237,         /**< USB Endpoint 3 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP4_T2_SCH_RDY       = 0x0238,         /**< USB Endpoint 4 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP5_T2_SCH_RDY       = 0x0239,         /**< USB Endpoint 5 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP6_T2_SCH_RDY       = 0x023A,         /**< USB Endpoint 6 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP7_T2_SCH_RDY       = 0x023B,         /**< USB Endpoint 7 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP8_T2_SCH_RDY       = 0x023C,         /**< USB Endpoint 8 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP9_T2_SCH_RDY       = 0x023D,         /**< USB Endpoint 9 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP10_T2_SCH_RDY      = 0x023E,         /**< USB Endpoint 10 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP11_T2_SCH_RDY      = 0x023F,         /**< USB Endpoint 11 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP12_T2_SCH_RDY      = 0x0240,         /**< USB Endpoint 12 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP13_T2_SCH_RDY      = 0x0241,         /**< USB Endpoint 13 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP14_T2_SCH_RDY      = 0x0242,         /**< USB Endpoint 14 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP15_T2_SCH_RDY      = 0x0243,         /**< USB Endpoint 15 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_PROT_T2_HDR_FULL     = 0x0275,         /**< USB protocol layer Type-2 transmit header full. */
    CY_FX_DBG_FUNC_USBSS_PROT_HDR_FULL        = 0x0276,         /**< USB protocol layer transmit header full. */
    CY_FX_DBG_FUNC_USBSS_LCW_READY            = 0x0277,         /**< Link Command Word ready for transmit. */
    CY_FX_DBG_FUNC_USBSS_DPP_RETRY            = 0x0278,         /**< USB Data packet payload retry. */
    CY_FX_DBG_FUNC_USBSS_RX_HDR_EMPTY         = 0x0279,         /**< USB protocol receive header empty condition. */
    CY_FX_DBG_FUNC_USBSS_TX_ELECIDLE          = 0x027A,         /**< USB PIPE level signal for TX electrical idle. */
    CY_FX_DBG_FUNC_USBSS_TXDET_LPBK           = 0x027B,         /**< USB PIPE level signal indicating loopback. */
    CY_FX_DBG_FUNC_USBSS_RX_VALID             = 0x027C,         /**< USB PIPE level signal indicating Rx.Valid */
    CY_FX_DBG_FUNC_USBSS_RX_ELECIDLE          = 0x027D,         /**< USB PIPE level signal for RX electrical idle. */
    CY_FX_DBG_FUNC_USBSS_RATE_10G             = 0x027E,         /**< USB PIPE level signal indicating 10G data rate */
    CY_FX_DBG_FUNC_USBSS_PWRDOWN_1            = 0x027F,         /**< USB PIPE power down signal #1 */
    CY_FX_DBG_FUNC_USBSS_PWRDOWN_0            = 0x0280,         /**< USB PIPE power down signal #0 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_COMP           = 0x0282,         /**< USB link state is Compliance. */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U3             = 0x0283,         /**< USB link state is U3 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U2             = 0x0284,         /**< USB link state is U2 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U1             = 0x0285,         /**< USB link state is U1 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U0             = 0x0286,         /**< USB link state is U0 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_LPBK_EXIT      = 0x0287,         /**< USB link state is Loopback.Exit */
    CY_FX_DBG_FUNC_USBSS_LTSSM_LPBK_ACTV      = 0x0288,         /**< USB link state is Loopback.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_HOTRST_EXIT    = 0x0289,         /**< USB link state is HotReset.Exit */
    CY_FX_DBG_FUNC_USBSS_LTSSM_HOTRST_ACTV    = 0x028A,         /**< USB link state is HotReset.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_IDLE     = 0x028B,         /**< USB link state is Recovery.Idle */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_CFG      = 0x028C,         /**< USB link state is Recovery.Configuration */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_ACTV     = 0x028D,         /**< USB link state is Recovery.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_IDLE      = 0x028E,         /**< USB link state is Polling.Idle */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_CFG       = 0x028F,         /**< USB link state is Polling.Configuration */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_ACTV      = 0x0290,         /**< USB link state is Polling.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_RXEQ      = 0x0291,         /**< USB link state is Polling.RxEq */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_PORTCFG   = 0x0292,         /**< USB link state is Polling.PortConfig */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_PORTMATCH = 0x0293,         /**< USB link state is Polling.PortMatch */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_LFPSPLUS  = 0x0294,         /**< USB link state is Polling.LFPSPlus */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_LFPS      = 0x0295,         /**< USB link state is Polling.LFPS */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RXDET_ACTV     = 0x0296,         /**< USB link state is RxDetect.Active */

    CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE     = 0x1000,         /**< LVDS/LVCMOS Link 0 training done. */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAINING_DONE     = 0x1001,         /**< LVDS/LVCMOS Link 1 training done. */
    CY_FX_DBG_FUNC_SIP_LNK0_FIFO_OVERFLOW     = 0x1002,         /**< LVDS/LVCMOS Link 0 FIFO overflow */
    CY_FX_DBG_FUNC_SIP_LNK1_FIFO_OVERFLOW     = 0x1003,         /**< LVDS/LVCMOS Link 1 FIFO overflow */
    CY_FX_DBG_FUNC_SIP_LNK0_FIFO_UNDERFLOW    = 0x1004,         /**< LVDS/LVCMOS Link 0 FIFO underflow */
    CY_FX_DBG_FUNC_SIP_LNK1_FIFO_UNDERFLOW    = 0x1005,         /**< LVDS/LVCMOS Link 1 FIFO underflow */
    CY_FX_DBG_FUNC_SIP_LNK0_TRAIN_BLK_DETECT  = 0x1008,         /**< Training block detected on LVDS/LVCMOS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_TRAIN_BLK_FAILED  = 0x1009,         /**< Training block detection failed on link 0 */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAIN_BLK_DETECT  = 0x100A,         /**< Training block detected on LVDS/LVCMOS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAIN_BLK_FAILED  = 0x100B,         /**< Training block detection failed on link 1 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE0_TRAIN_BLK   = 0x100C,         /**< Training block detected on lane 0 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE1_TRAIN_BLK   = 0x100D,         /**< Training block detected on lane 1 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE2_TRAIN_BLK   = 0x100E,         /**< Training block detected on lane 2 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE3_TRAIN_BLK   = 0x100F,         /**< Training block detected on lane 3 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE4_TRAIN_BLK   = 0x1010,         /**< Training block detected on lane 4 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE5_TRAIN_BLK   = 0x1011,         /**< Training block detected on lane 5 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE6_TRAIN_BLK   = 0x1012,         /**< Training block detected on lane 6 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE7_TRAIN_BLK   = 0x1013,         /**< Training block detected on lane 7 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE8_TRAIN_BLK   = 0x1014,         /**< Training block detected on lane 8 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE0_TRAIN_BLK   = 0x1015,         /**< Training block detected on lane 0 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE1_TRAIN_BLK   = 0x1016,         /**< Training block detected on lane 1 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE2_TRAIN_BLK   = 0x1017,         /**< Training block detected on lane 2 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE3_TRAIN_BLK   = 0x1018,         /**< Training block detected on lane 3 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE4_TRAIN_BLK   = 0x1019,         /**< Training block detected on lane 4 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE5_TRAIN_BLK   = 0x101A,         /**< Training block detected on lane 5 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE6_TRAIN_BLK   = 0x101B,         /**< Training block detected on lane 6 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE7_TRAIN_BLK   = 0x101C,         /**< Training block detected on lane 7 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE8_TRAIN_BLK   = 0x101D,         /**< Training block detected on lane 8 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK0_DATA_IOS_OFF      = 0x101F,         /**< Data IOs of LVDS/LVCMOS link 0 switched off */
    CY_FX_DBG_FUNC_SIP_LNK0_CTRL_IOS_OFF      = 0x1020,         /**< Control IOs of LVDS/LVCMOS link 0 switched off */
    CY_FX_DBG_FUNC_SIP_LNK0_L1_STATE          = 0x1021,         /**< LVDS/LVCMOS link 0 in L1 state. */
    CY_FX_DBG_FUNC_SIP_LNK0_L3_STATE          = 0x1022,         /**< LVDS/LVCMOS link 0 in L3 state. */
    CY_FX_DBG_FUNC_SIP_LNK1_DATA_IOS_OFF      = 0x1023,         /**< Data IOs of LVDS/LVCMOS link 1 switched off */
    CY_FX_DBG_FUNC_SIP_LNK1_CTRL_IOS_OFF      = 0x1024,         /**< Control IOs of LVDS/LVCMOS link 1 switched off */
    CY_FX_DBG_FUNC_SIP_LNK1_L1_STATE          = 0x1025,         /**< LVDS/LVCMOS link 1 in L1 state. */
    CY_FX_DBG_FUNC_SIP_LNK1_L3_STATE          = 0x1026,         /**< LVDS/LVCMOS link 1 in L3 state. */
    CY_FX_DBG_FUNC_SIP_LNK0_TRAIN_DONE_LPBK   = 0x103b,         /**< LVDS/LVCMOS Link 0 training done */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAIN_DONE_LPBK   = 0x103c,         /**< LVDS/LVCMOS Link 1 training done */
    CY_FX_DBG_FUNC_SIP_LNK0_L1_ENTRY          = 0x1070,         /**< LVDS/LVCMOS link 0 in L1 state Entry */
    CY_FX_DBG_FUNC_SIP_LNK0_L1_EXIT           = 0x1071,         /**< LVDS/LVCMOS link 0 in L1 state Exit */
    CY_FX_DBG_FUNC_SIP_LNK0_L3_ENTRY          = 0x1072,         /**< LVDS/LVCMOS link 0 in L3 state Entry */
    CY_FX_DBG_FUNC_SIP_LNK1_L1_ENTRY          = 0x1073,         /**< LVDS/LVCMOS link 1 in L1 state Entry */
    CY_FX_DBG_FUNC_SIP_LNK1_L1_EXIT           = 0x1074,         /**< LVDS/LVCMOS link 1 in L1 state Exit */
    CY_FX_DBG_FUNC_SIP_LNK1_L3_ENTRY          = 0x1075,         /**< LVDS/LVCMOS link 1 in L3 state Entry */

    CY_FX_DBG_FUNC_SIP_LNK0_LANE0_STS         = 0x129e,         /**< Status on Data lane 0 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE1_STS         = 0x129f,         /**< Status on Data lane 1 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE2_STS         = 0x12a0,         /**< Status on Data lane 2 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE3_STS         = 0x12a1,         /**< Status on Data lane 3 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE4_STS         = 0x12a2,         /**< Status on Data lane 4 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE5_STS         = 0x12a3,         /**< Status on Data lane 5 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE6_STS         = 0x12a4,         /**< Status on Data lane 6 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE7_STS         = 0x12a5,         /**< Status on Data lane 7 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_CTRL_STS          = 0x12a6,         /**< Status on CTR Lane of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_CLK_STS           = 0x12a7,         /**< Status on CLK of LVDS link 0 */

    CY_FX_DBG_FUNC_SIP_LNK1_LANE0_STS         = 0x149e,         /**< Status on Data lane 0 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE1_STS         = 0x149f,         /**< Status on Data lane 1 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE2_STS         = 0x14a0,         /**< Status on Data lane 2 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE3_STS         = 0x14a1,         /**< Status on Data lane 3 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE4_STS         = 0x14a2,         /**< Status on Data lane 4 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE5_STS         = 0x14a3,         /**< Status on Data lane 5 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE6_STS         = 0x14a4,         /**< Status on Data lane 6 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE7_STS         = 0x14a5,         /**< Status on Data lane 7 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_CTRL_STS          = 0x14a6,         /**< Status on CTR Lane of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_CLK_STS           = 0x14a7,         /**< Status on CLK of LVDS link 1 */

    CY_FX_DBG_FUNC_SIP_GPIF0_TH0_SKT_NUM      = 0x1610,         /**< Update Thread0 SKT Number on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH1_SKT_NUM      = 0x1611,         /**< Update Thread1 SKT Number on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH2_SKT_NUM      = 0x1612,         /**< Update Thread2 SKT Number on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH3_SKT_NUM      = 0x1613,         /**< Update Thread3 SKT Number on GPIF0 */

    CY_FX_DBG_FUNC_SIP_GPIF0_TH0_FG           = 0x1634,         /**< Empty/Full flag for Thread0 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH0_PARTIAL_FG   = 0x1635,         /**< Partial flag for Thread0 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH1_FG           = 0x1636,         /**< Empty/Full flag for Thread1 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH1_PARTIAL_FG   = 0x1637,         /**< Partial flag for Thread1 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH2_FG           = 0x1638,         /**< Empty/Full flag for Thread2 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH2_PARTIAL_FG   = 0x1639,         /**< Partial flag for Thread2 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH3_FG           = 0x163a,         /**< Empty/Full flag for Thread3 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_TH3_PARTIAL_FG   = 0x163b,         /**< Partial flag for Thread3 on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_CURR_FG          = 0x163c,         /**< Empty/Full flag for Current Thread on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_CURR_PARTIAL_FG  = 0x163d,         /**< Partial flag for Current Thread on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_INVAL_STATE_ERR  = 0x163d,         /**< GPIF INVALID STATE ERR on GPIF0 */

    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_0          = 0x164f,         /**< alpha0 "dq_oen" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_1          = 0x1650,         /**< alpha1 "update_dout" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_2          = 0x1651,         /**< alpha2 "sample_din" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_3          = 0x1652,         /**< alpha3 "sample_ain" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_4          = 0x1653,         /**< alpha4 First early output signal from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_5          = 0x1654,         /**< alpha5 Second early output signal from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_6          = 0x1655,         /**< alpha6 Third early output signal from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_ALPHA_7          = 0x1656,         /**< alpha7 Fourth early output signal from GPIF0 state machine */

    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_0           = 0x1657,         /**< beta0 First delayed output from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_1           = 0x1658,         /**< beta1 Second delayed output from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_2           = 0x1659,         /**< beta2 Third delayed output from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_3           = 0x165a,         /**< beta3 Fourth delayed output from GPIF0 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_4           = 0x165b,         /**< beta4 LS bit of thread number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_5           = 0x165c,         /**< beta5 MS bit of thread number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_6           = 0x165d,         /**< beta6 "rq_pop" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_7           = 0x165e,         /**< beta7 "wq_push" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_8           = 0x165f,         /**< beta8 "arq_pop" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_9           = 0x1660,         /**< beta9 "awq_push" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_10          = 0x1661,         /**< beta10 "a_oen" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_11          = 0x1662,         /**< beta11 "c_oen" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_12          = 0x1663,         /**< beta12 "ctrl_count_incr" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_13          = 0x1664,         /**< beta13 "ctrl_count_reset" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_14          = 0x1665,         /**< beta14 "addr_count_incr" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_15          = 0x1666,         /**< beta15 "addr_count_reset" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_16          = 0x1667,         /**< beta16 "data_count_incr" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_17          = 0x1668,         /**< beta17 "data_count_reset" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_18          = 0x1669,         /**< beta18 "interrupt_cpu" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_20          = 0x166b,         /**< beta20 "update_aout" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_21          = 0x166c,         /**< beta21 "register_access" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_22          = 0x166d,         /**< beta22 "initialize_crc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_23          = 0x166e,         /**< beta23 "calculate_crc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_24          = 0x166f,         /**< beta24 "use_crc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_27          = 0x1672,         /**< beta27 "thread_done" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_28          = 0x1673,         /**< beta28 "thread_done_eot" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_29          = 0x1674,         /**< beta29 "eop" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_BETA_30          = 0x1675,         /**< beta30 "eot" on GPIF0 */

    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_0         = 0x1677,         /**< lambda0 Reflects CTL0 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_1         = 0x1678,         /**< lambda1 Reflects CTL1 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_2         = 0x1679,         /**< lambda2 Reflects CTL2 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_3         = 0x167a,         /**< lambda3 Reflects CTL3 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_4         = 0x167b,         /**< lambda4 Reflects CTL4 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_5         = 0x167c,         /**< lambda5 Reflects CTL5 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_6         = 0x167d,         /**< lambda6 Reflects CTL6 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_7         = 0x167e,         /**< lambda7 Reflects CTL7 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_8         = 0x167f,         /**< lambda8 Reflects CTL8 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_9         = 0x1680,         /**< lambda9 Reflects CTL9 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_10        = 0x1681,         /**< lambda10 Reflects CTL10 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_11        = 0x1682,         /**< lambda11 Reflects CTL11 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_12        = 0x1683,         /**< lambda12 Reflects CTL12 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_13        = 0x1684,         /**< lambda13 Reflects CTL13 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_14        = 0x1685,         /**< lambda14 Reflects CTL14 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_15        = 0x1686,         /**< lambda15 Reflects CTL15 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_16        = 0x1687,         /**< lambda16 "eg_data_valid" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_17        = 0x1688,         /**< lambda17 "in_data_valid" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_18        = 0x1689,         /**< lambda18 "ctrl_count" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_19        = 0x168a,         /**< lambda19 "addr_count" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_20        = 0x168b,         /**< lambda20 "data_count" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_21        = 0x168c,         /**< lambda21 "ctrl_comp" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_22        = 0x168d,         /**< lambda22 "addr_comp" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_23        = 0x168e,         /**< lambda23 "data_comp" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_24        = 0x168f,         /**< lambda24 "dma_watermark" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_25        = 0x1690,         /**< lambda25 "dma_ready_addr" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_26        = 0x1691,         /**< lambda26 "dma_ready_data" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_27        = 0x1692,         /**< lambda27 "crc_error" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_28        = 0x1693,         /**< lambda28 "Boolean One" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_29        = 0x1694,         /**< lambda29 "interrupt_pending" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_30        = 0x1695,         /**< lambda30 "cpu_lambda" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_31        = 0x1696,         /**< lambda31 "eg_addr_valid" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_32        = 0x1697,         /**< lambda32 Reflects CTL16 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_33        = 0x1698,         /**< lambda33 Reflects CTL17 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_34        = 0x1699,         /**< lambda34 Reflects CTL18 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_35        = 0x169a,         /**< lambda35 Reflects CTL19 asserted state for GPIF0 instance */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_36        = 0x169b,         /**< lambda36 "link_err" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_37        = 0x169c,         /**< lambda37 "tseq_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_38        = 0x169d,         /**< lambda38 "link_idle_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_39        = 0x169e,         /**< lambda39 "data_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_40        = 0x169f,         /**< lambda40 "inmd_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_41        = 0x16a0,         /**< lambda41 "eop_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_42        = 0x16a1,         /**< lambda42 "chkdcrc_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_43        = 0x16a2,         /**< lambda43 "initmcrc_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_44        = 0x16a3,         /**< lambda44 "initdcrc_cmd" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_45        = 0x16a4,         /**< lambda45 "lvds_sflg" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_46        = 0x16a5,         /**< lambda46 "lvds_cflg" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_47        = 0x16a6,         /**< lambda47 "lvds_set_tad" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_48        = 0x16a7,         /**< lambda48 "lvds_set_sad" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_49        = 0x16a8,         /**< lambda49 "lvds_set_varwl" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_50        = 0x16a9,         /**< lambda50 "lvds_set_varwh" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_51        = 0x16aa,         /**< lambda51 "lvds_uevc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_52        = 0x16ab,         /**< lambda52 "lvds_cevc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_53        = 0x16ac,         /**< lambda53 "lvds_epc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_54        = 0x16ad,         /**< lambda54 "lvds_hpc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_55        = 0x16ae,         /**< lambda55 "lvds_cpc" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_56        = 0x16af,         /**< lambda56 "lvds_l1_enter" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_57        = 0x16b0,         /**< lambda57 "lvds_l1_exit" on GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_58        = 0x16b1,         /**< lambda58 Bit 0 of socket number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_59        = 0x16b2,         /**< lambda59 Bit 1 of socket number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_60        = 0x16b3,         /**< lambda60 Bit 2 of socket number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_61        = 0x16b4,         /**< lambda61 Bit 3 of socket number selected by GPIF0 */
    CY_FX_DBG_FUNC_SIP_GPIF0_LAMBDA_62        = 0x16b5,         /**< lambda62 Bit 4 of socket number selected by GPIF0 */

    CY_FX_DBG_FUNC_SIP_GPIF1_TH0_SKT_NUM      = 0x1810,         /**< Update Thread0 SKT Number on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH1_SKT_NUM      = 0x1811,         /**< Update Thread1 SKT Number on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH2_SKT_NUM      = 0x1812,         /**< Update Thread2 SKT Number on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH3_SKT_NUM      = 0x1813,         /**< Update Thread3 SKT Number on GPIF1 */

    CY_FX_DBG_FUNC_SIP_GPIF1_TH0_FG           = 0x1834,         /**< Empty/Full flag for Thread0 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH0_PARTIAL_FG   = 0x1835,         /**< Partial flag for Thread0 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH1_FG           = 0x1836,         /**< Empty/Full flag for Thread1 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH1_PARTIAL_FG   = 0x1837,         /**< Partial flag for Thread1 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH2_FG           = 0x1838,         /**< Empty/Full flag for Thread2 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH2_PARTIAL_FG   = 0x1839,         /**< Partial flag for Thread2 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH3_FG           = 0x183a,         /**< Empty/Full flag for Thread3 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_TH3_PARTIAL_FG   = 0x183b,         /**< Partial flag for Thread3 on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_CURR_FG          = 0x183c,         /**< Empty/Full flag for Current Thread on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_CURR_PARTIAL_FG  = 0x183d,         /**< Partial flag for Current Thread on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_INVAL_STATE_ERR  = 0x183d,         /**< GPIF INVALID STATE ERR on GPIF1 */

    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_0          = 0x184f,         /**< alpha0 "dq_oen" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_1          = 0x1850,         /**< alpha1 "update_dout" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_2          = 0x1851,         /**< alpha2 "sample_din" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_3          = 0x1852,         /**< alpha3 "sample_ain" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_4          = 0x1853,         /**< alpha4 First early output signal from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_5          = 0x1854,         /**< alpha5 Second early output signal from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_6          = 0x1855,         /**< alpha6 Third early output signal from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_ALPHA_7          = 0x1856,         /**< alpha7 Fourth early output signal from GPIF1 state machine */

    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_0           = 0x1857,         /**< beta0 First delayed output from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_1           = 0x1858,         /**< beta1 Second delayed output from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_2           = 0x1859,         /**< beta2 Third delayed output from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_3           = 0x185a,         /**< beta3 Fourth delayed output from GPIF1 state machine */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_4           = 0x185b,         /**< beta4 LS bit of thread number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_5           = 0x185c,         /**< beta5 MS bit of thread number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_6           = 0x185d,         /**< beta6 "rq_pop" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_7           = 0x185e,         /**< beta7 "wq_push" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_8           = 0x185f,         /**< beta8 "arq_pop" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_9           = 0x1860,         /**< beta9 "awq_push" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_10          = 0x1861,         /**< beta10 "a_oen" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_11          = 0x1862,         /**< beta11 "c_oen" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_12          = 0x1863,         /**< beta12 "ctrl_count_incr" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_13          = 0x1864,         /**< beta13 "ctrl_count_reset" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_14          = 0x1865,         /**< beta14 "addr_count_incr" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_15          = 0x1866,         /**< beta15 "addr_count_reset" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_16          = 0x1867,         /**< beta16 "data_count_incr" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_17          = 0x1868,         /**< beta17 "data_count_reset" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_18          = 0x1869,         /**< beta18 "interrupt_cpu" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_20          = 0x186b,         /**< beta20 "update_aout" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_21          = 0x186c,         /**< beta21 "register_access" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_22          = 0x186d,         /**< beta22 "initialize_crc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_23          = 0x186e,         /**< beta23 "calculate_crc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_24          = 0x186f,         /**< beta24 "use_crc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_27          = 0x1872,         /**< beta27 "thread_done" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_28          = 0x1873,         /**< beta28 "thread_done_eot" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_29          = 0x1874,         /**< beta29 "eop" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_BETA_30          = 0x1875,         /**< beta30 "eot" on GPIF1 */

    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_0         = 0x1877,         /**< lambda0 Reflects CTL0 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_1         = 0x1878,         /**< lambda1 Reflects CTL1 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_2         = 0x1879,         /**< lambda2 Reflects CTL2 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_3         = 0x187a,         /**< lambda3 Reflects CTL3 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_4         = 0x187b,         /**< lambda4 Reflects CTL4 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_5         = 0x187c,         /**< lambda5 Reflects CTL5 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_6         = 0x187d,         /**< lambda6 Reflects CTL6 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_7         = 0x187e,         /**< lambda7 Reflects CTL7 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_8         = 0x187f,         /**< lambda8 Reflects CTL8 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_9         = 0x1880,         /**< lambda9 Reflects CTL9 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_10        = 0x1881,         /**< lambda10 Reflects CTL10 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_11        = 0x1882,         /**< lambda11 Reflects CTL11 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_12        = 0x1883,         /**< lambda12 Reflects CTL12 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_13        = 0x1884,         /**< lambda13 Reflects CTL13 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_14        = 0x1885,         /**< lambda14 Reflects CTL14 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_15        = 0x1886,         /**< lambda15 Reflects CTL15 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_16        = 0x1887,         /**< lambda16 "eg_data_valid" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_17        = 0x1888,         /**< lambda17 "in_data_valid" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_18        = 0x1889,         /**< lambda18 "ctrl_count" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_19        = 0x188a,         /**< lambda19 "addr_count" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_20        = 0x188b,         /**< lambda20 "data_count" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_21        = 0x188c,         /**< lambda21 "ctrl_comp" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_22        = 0x188d,         /**< lambda22 "addr_comp" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_23        = 0x188e,         /**< lambda23 "data_comp" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_24        = 0x188f,         /**< lambda24 "dma_watermark" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_25        = 0x1890,         /**< lambda25 "dma_ready_addr" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_26        = 0x1891,         /**< lambda26 "dma_ready_data" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_27        = 0x1892,         /**< lambda27 "crc_error" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_28        = 0x1893,         /**< lambda28 "Boolean One" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_29        = 0x1894,         /**< lambda29 "interrupt_pending" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_30        = 0x1895,         /**< lambda30 "cpu_lambda" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_31        = 0x1896,         /**< lambda31 "eg_addr_valid" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_32        = 0x1897,         /**< lambda32 Reflects CTL16 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_33        = 0x1898,         /**< lambda33 Reflects CTL17 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_34        = 0x1899,         /**< lambda34 Reflects CTL18 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_35        = 0x189a,         /**< lambda35 Reflects CTL19 asserted state for GPIF1 instance */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_36        = 0x189b,         /**< lambda36 "link_err" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_37        = 0x189c,         /**< lambda37 "tseq_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_38        = 0x189d,         /**< lambda38 "link_idle_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_39        = 0x189e,         /**< lambda39 "data_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_40        = 0x189f,         /**< lambda40 "inmd_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_41        = 0x18a0,         /**< lambda41 "eop_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_42        = 0x18a1,         /**< lambda42 "chkdcrc_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_43        = 0x18a2,         /**< lambda43 "initmcrc_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_44        = 0x18a3,         /**< lambda44 "initdcrc_cmd" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_45        = 0x18a4,         /**< lambda45 "lvds_sflg" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_46        = 0x18a5,         /**< lambda46 "lvds_cflg" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_47        = 0x18a6,         /**< lambda47 "lvds_set_tad" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_48        = 0x18a7,         /**< lambda48 "lvds_set_sad" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_49        = 0x18a8,         /**< lambda49 "lvds_set_varwl" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_50        = 0x18a9,         /**< lambda50 "lvds_set_varwh" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_51        = 0x18aa,         /**< lambda51 "lvds_uevc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_52        = 0x18ab,         /**< lambda52 "lvds_cevc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_53        = 0x18ac,         /**< lambda53 "lvds_epc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_54        = 0x18ad,         /**< lambda54 "lvds_hpc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_55        = 0x18ae,         /**< lambda55 "lvds_cpc" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_56        = 0x18af,         /**< lambda56 "lvds_l1_enter" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_57        = 0x18b0,         /**< lambda57 "lvds_l1_exit" on GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_58        = 0x18b1,         /**< lambda58 Bit 0 of socket number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_59        = 0x18b2,         /**< lambda59 Bit 1 of socket number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_60        = 0x18b3,         /**< lambda60 Bit 2 of socket number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_61        = 0x18b4,         /**< lambda61 Bit 3 of socket number selected by GPIF1 */
    CY_FX_DBG_FUNC_SIP_GPIF1_LAMBDA_62        = 0x18b5,         /**< lambda62 Bit 4 of socket number selected by GPIF1 */

    CY_FX_DBG_FUNC_SIP_TH0_EOP                = 0x1A04,         /**< End of Packet on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_EOT                = 0x1A05,         /**< End of Transfer on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_DMA_READY          = 0x1A08,         /**< DMA READY on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_DMA_ERROR          = 0x1A0a,         /**< DMA Error on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_OVERRUN            = 0x1A1d,         /**< Write exceeds the space available in the buffer on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_UNDERRUN           = 0x1A1e,         /**< Reads exceeds the byte count of the buffer on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_SOCKET_ACTIVE      = 0x1A2b,         /**< Socket has gone inactive within a DMA Transfer on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_ADAPTER_OVERRUN    = 0x1A61,         /**< Adapter Unable to service write request though buffer is available on Thread0 */
    CY_FX_DBG_FUNC_SIP_TH0_ADAPTER_UNDERRUN   = 0x1A62,         /**< Adapter Unable to service read request though buffer is available on Thread0 */

    CY_FX_DBG_FUNC_SIP_TH1_EOP                = 0x1B04,         /**< End of Packet on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_EOT                = 0x1B05,         /**< End of Transfer on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_DMA_READY          = 0x1B08,         /**< DMA READY on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_DMA_ERROR          = 0x1B0a,         /**< DMA Error on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_OVERRUN            = 0x1B1d,         /**< Write exceeds the space available in the buffer on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_UNDERRUN           = 0x1B1e,         /**< Reads exceeds the byte count of the buffer on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_SOCKET_ACTIVE      = 0x1B2b,         /**< Socket has gone inactive within a DMA Transfer on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_ADAPTER_OVERRUN    = 0x1B61,         /**< Adapter Unable to service write request though buffer is available on Thread1 */
    CY_FX_DBG_FUNC_SIP_TH1_ADAPTER_UNDERRUN   = 0x1B62,         /**< Adapter Unable to service read request though buffer is available on Thread1 */

    CY_FX_DBG_FUNC_SIP_TH2_EOP                = 0x1C04,         /**< End of Packet on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_EOT                = 0x1C05,         /**< End of Transfer on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_DMA_READY          = 0x1C08,         /**< DMA READY on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_DMA_ERROR          = 0x1C0a,         /**< DMA Error on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_OVERRUN            = 0x1C1d,         /**< Write exceeds the space available in the buffer on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_UNDERRUN           = 0x1C1e,         /**< Reads exceeds the byte count of the buffer on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_SOCKET_ACTIVE      = 0x1C2b,         /**< Socket has gone inactive within a DMA Transfer on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_ADAPTER_OVERRUN    = 0x1C61,         /**< Adapter Unable to service write request though buffer is available on Thread2 */
    CY_FX_DBG_FUNC_SIP_TH2_ADAPTER_UNDERRUN   = 0x1C62,         /**< Adapter Unable to service read request though buffer is available on Thread2 */

    CY_FX_DBG_FUNC_SIP_TH3_EOP                = 0x1D04,         /**< End of Packet on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_EOT                = 0x1D05,         /**< End of Transfer on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_DMA_READY          = 0x1D08,         /**< DMA READY on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_DMA_ERROR          = 0x1D0a,         /**< DMA Error on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_OVERRUN            = 0x1D1d,         /**< Write exceeds the space available in the buffer on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_UNDERRUN           = 0x1D1e,         /**< Reads exceeds the byte count of the buffer on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_SOCKET_ACTIVE      = 0x1D2b,         /**< Socket has gone inactive within a DMA Transfer on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_ADAPTER_OVERRUN    = 0x1D61,         /**< Adapter Unable to service write request though buffer is available on Thread3 */
    CY_FX_DBG_FUNC_SIP_TH3_ADAPTER_UNDERRUN   = 0x1D62          /**< Adapter Unable to service read request though buffer is available on Thread3 */

} cy_en_fxusb_dbg_func_sel_t;

/** \} group_usbfxstack_fx_utils_enums */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/*******************************************************************************
 * Function name: Cy_UsbFx_SelectDFTFunctions
 ****************************************************************************//**
 *
 * Function used to select the Design-For-Test debug functions which are to be
 * driven on to the P11.0, P11.1, P9.2, P9.3, P9.4 and P9.5 pins of the EZ-USB FX device.
 *
 * \param dft0_func
 * Select the debug function to be driven on the P11.0 pin. Any USB/LVDS function
 * is supported.
 * \param dft1_func
 * Select the debug function to be driven on the P11.1 pin. Any USB/LVDS function
 * is supported.
 * \param usbdft0_func
 * Select the debug function to be driven on the P9.2 pin. Only USB functions
 * are supported.
 * \param usbdft1_func
 * Select the debug function to be driven on the P9.3 pin. Only USB functions
 * are supported.
 * \param sipdft0_func
 * Select the debug function to be driven on the P9.4 pin. Only LVDS/LVCMOS functions
 * are supported.
 * \param sipdft1_func
 * Select the debug function to be driven on the P9.5 pin. Only LVDS/LVCMOS functions
 * are supported.
 *
 * \return
 * true if DFT configuration is done, false in case of error.
 *******************************************************************************/
bool Cy_UsbFx_SelectDFTFunctions(
        uint32_t dft0_func,
        uint32_t dft1_func,
        uint32_t usbdft0_func,
        uint32_t usbdft1_func,
        uint32_t sipdft0_func,
        uint32_t sipdft1_func);

/*******************************************************************************
 * Function name: Cy_UsbFx_MemSetDword
 ****************************************************************************//**
 *
 * Initialize a memory buffer with the specified 4-byte value.
 *
 * \param pBuffer
 * Pointer to buffer to be initialized.
 *
 * \param value
 * Value to initialize the memory buffer with.
 *
 * \param byteCount
 * Size of the buffer in bytes.
 *
 *******************************************************************************/
void
Cy_UsbFx_MemSetDword(
        uint32_t *pBuffer,
        uint32_t value,
        uint32_t byteCount);

/*******************************************************************************
 * Function name: Cy_UsbFx_MemCpyDword
 ****************************************************************************//**
 *
 * Copy data from source to destination buffer using 4-byte words.
 *
 * \param pDst
 * Pointer to destination buffer.
 *
 * \param pSrc
 * Pointer to source buffer.
 *
 * \param byteCount
 * Size of the buffer in bytes.
 *
 *******************************************************************************/
void
Cy_UsbFx_MemCpyDword(
        uint32_t *pDst,
        const uint32_t *pSrc,
        uint32_t byteCount);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_FX_COMMON_H_ */

/** \} group_usbfxstack_fx_utils */

/*[]*/

