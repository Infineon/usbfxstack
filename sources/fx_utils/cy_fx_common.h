/***************************************************************************//**
* \file cy_fx_common.h
* \version 1.0
*
* Provides macros and definitions associated with the FX common utility functions.
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
    CY_FX_DBG_FUNC_SIP_LNK1_L3_STATE          = 0x1026          /**< LVDS/LVCMOS link 1 in L3 state. */
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
 * Function used to select the Design-For-Test debug functions which are to
 * be driven on to the P11.0, P11.1, P9.2 and P9.3 pins of the EZ-USB FX device.
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
 *
 * \return
 * true if DFT configuration is done, false in case of error.
 *******************************************************************************/
bool Cy_UsbFx_SelectDFTFunctions(
        uint32_t dft0_func,
        uint32_t dft1_func,
        uint32_t usbdft0_func,
        uint32_t usbdft1_func);

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

