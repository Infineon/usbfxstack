/***************************************************************************//**
* \file cy_lvds.h
* \version 1.00
*
* This file provides constants and parameter values for the LVDS driver.
*
********************************************************************************
* \copyright
* Copyright (2024) Cypress Semiconductor Corporation
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
 * \addtogroup group_usbfxstack_lvds_lvcmos
 * \{
 * The LVCMOS_LVDS block is initialized and configured using the LVCMOS/LVDS
 * library which is a part of the usbfxstack middleware library.
 * \defgroup group_usbfxstack_lvds_lvcmos_macros Macros
 * \defgroup group_usbfxstack_lvds_lvcmos_enums Enumerated Types
 * \defgroup group_usbfxstack_lvds_lvcmos_structs Data Structures
 * \defgroup group_usbfxstack_lvds_lvcmos_functions Functions
 */

/**
 * \addtogroup group_usbfxstack_lvds_lvcmos_macros
 * \{
 */

#if !defined(CY_LVDS_H)

/** Indicates the use of LVDS/LVCMOS */
#define CY_LVDS_H

#include "cy_device.h"

#if defined (CY_IP_MXS40LVDS2USB32SS)

#include "stdint.h"
#include <stddef.h>
#include "cy_syslib.h"
#include "ip/cyip_lvdsss.h"


#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
*                        API Constants
*******************************************************************************/

/** Driver major version */
#define CY_LVDS_DRV_VERSION_MAJOR           (1)

/** Driver minor version */
#define CY_LVDS_DRV_VERSION_MINOR           (0)

/** HBDMA driver identifier */
#define CY_LVDS_ID                          CY_PDL_DRV_ID(0x82U)

/** PHY MDLL phase value for LVCMOS DDR mode. */
#define CY_LVDS_DDR_MDLL_PHASE_VAL          (0u)

/** Little endian byte ordering selection. */
#define CY_LVDS_GPIF_LITTLE_ENDIAN          0u
/** Big endian byte ordering selection. */
#define CY_LVDS_GPIF_BIG_ENDIAN             1u
/** Maximum number of states in GPIF state machine. */
#define CY_LVDS_GPIF_MAX_NUM_STATES         (256)
/** Number of transition functions supported by GPIF hardware. */
#define CY_LVDS_GPIF_NUM_TRANS_FUNC         (32)

#if (!LVCMOS_16BIT_SDR)

/** Number of DMA threads supported by the LVDS IP. */
#define CY_LVDS_MAX_THREAD_COUNT            (4)
/** Number of metadata structures supported. */
#define CY_LVDS_MAX_MD_INDEX_COUNT          (4)
/** Size of each metadata structure in 16-bit half-words. */
#define CY_LVDS_MAX_MD_COUNT                (16)
/** Number of LVDS IP PHY instances. */
#define CY_LVDS_MAX_PHY_INSTANCE            (2)
/** Number of GPIF hardware instances. */
#define CY_LVDS_MAX_GPIF_INSTANCE           (2)
/** Number of sockets supported per LVDS port. */
#define CY_LVDS_MAX_SOCKET_COUNT            (16)

#else

/** Number of DMA threads supported by the LVDS IP. */
#define CY_LVDS_MAX_THREAD_COUNT            (2)
/** Number of metadata structures supported. */
#define CY_LVDS_MAX_MD_INDEX_COUNT          (2)
/** Size of each metadata structure in 16-bit half-words. */
#define CY_LVDS_MAX_MD_COUNT                (16)
/** Number of LVDS IP PHY instances. */
#define CY_LVDS_MAX_PHY_INSTANCE            (1)
/** Number of GPIF hardware instances. */
#define CY_LVDS_MAX_GPIF_INSTANCE           (1)
/** Number of sockets supported per LVDS port. */
#define CY_LVDS_MAX_SOCKET_COUNT            (16)

#endif /*LVCMOS_16BIT_SDR*/

/** Timeout loop count used during PHY initialization. */
#define CY_LVDS_MAX_TIMEOUT_COUNT           (2500u)

/** Number of distinct states that are possible in a GPIF state machine when DSS is enabled. */
#define GPIF_MAX_STATE(numDss)              (0 ? 255: ((256/(numDss * 2)) - 1))

/** Offset of thread registers within LVDS IP block. */
#define CY_LVDS_THREAD_OFFSET_ADDRESS           0x0400

/** Offset of metadata variable #0 */
#define CY_LVDS_MD_VARIABLE0                    (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x002C)
/** Offset of metadata variable #1 */
#define CY_LVDS_MD_VARIABLE1                    (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0030)
/** Offset of low word of PTS timer. */
#define CY_LVDS_MD_PTS_TIMER_L                  (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0034)
/** Offset of high word of PTS timer. */
#define CY_LVDS_MD_PTS_TIMER_H                  (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0036)
/** Offset of bytes 1:0 of SCR timer. */
#define CY_LVDS_MD_SCR_TIMER0_L                 (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0038)
/** Offset of bytes 3:2 of SCR timer. */
#define CY_LVDS_MD_SCR_TIMER0_H                 (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x003A)
/** Offset of bytes 5:4 of SCR timer. */
#define CY_LVDS_MD_SCR_TIMER1_L                 (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x003C)
/** Offset of bytes 7:6 of SCR timer. */
#define CY_LVDS_MD_SCR_TIMER1_H                 (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x003E)
/** Offset of metadata flags field. */
#define CY_LVDS_MD_FLAGS                        (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0040)
/** Offset of bytes 1:0 of event counter #0 */
#define CY_LVDS_MD_EVENT_COUNTER0_DW0_L         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0050)
/** Offset of bytes 3:2 of event counter #0 */
#define CY_LVDS_MD_EVENT_COUNTER0_DW0_H         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0052)
/** Offset of bytes 5:4 of event counter #0 */
#define CY_LVDS_MD_EVENT_COUNTER0_DW1_L         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0054)
/** Offset of bytes 7:6 of event counter #0 */
#define CY_LVDS_MD_EVENT_COUNTER0_DW1_H         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0056)
/** Offset of bytes 1:0 of event counter #1 */
#define CY_LVDS_MD_EVENT_COUNTER1_DW0_L         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0058)
/** Offset of bytes 3:2 of event counter #1 */
#define CY_LVDS_MD_EVENT_COUNTER1_DW0_H         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x005A)
/** Offset of bytes 5:4 of event counter #1 */
#define CY_LVDS_MD_EVENT_COUNTER1_DW1_L         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x005C)
/** Offset of bytes 7:6 of event counter #1 */
#define CY_LVDS_MD_EVENT_COUNTER1_DW1_H         (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x005E)
/** Offset of bytes 1:0 of payload counter. */
#define CY_LVDS_MD_PAYLOAD_COUNTER_DW0_L        (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0060)
/** Offset of bytes 3:2 of payload counter. */
#define CY_LVDS_MD_PAYLOAD_COUNTER_DW0_H        (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0062)
/** Offset of bytes 5:4 of payload counter. */
#define CY_LVDS_MD_PAYLOAD_COUNTER_DW1_L        (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0064)
/** Offset of bytes 7:6 of payload counter. */
#define CY_LVDS_MD_PAYLOAD_COUNTER_DW1_H        (CY_LVDS_THREAD_OFFSET_ADDRESS + 0x0066)
/** Position of TWO_BIT_SLV_FF field in LINK_CONFIG register of the A0 Silicon Revision */
#define LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF_A0_Pos   (7UL)
/** Mask for TWO_BIT_SLV_FF field in LINK_CONFIG register of the A0 Silicon Revision */
#define LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF_A0_Msk   (0x80UL)

/** \} group_usbfxstack_lvds_lvcmos_macros */

/**
 * \addtogroup group_usbfxstack_lvds_lvcmos_enums
 * \{
 */

/*******************************************************************************
*                         Enumerated Data Types
*******************************************************************************/

/**
 * @typedef cy_en_lvds_phy_gpio_index_t
 * @brief List of LVDS IP pins which can be used as GPIO when otherwise unused.
 */
typedef enum
{
    CY_LVDS_PHY_GPIO_DQ0 = 0,                   /**< Data pin #0 */
    CY_LVDS_PHY_GPIO_DQ1,                       /**< Data pin #1 */
    CY_LVDS_PHY_GPIO_DQ2,                       /**< Data pin #2 */
    CY_LVDS_PHY_GPIO_DQ3,                       /**< Data pin #3 */
    CY_LVDS_PHY_GPIO_DQ4,                       /**< Data pin #4 */
    CY_LVDS_PHY_GPIO_DQ5,                       /**< Data pin #5 */
    CY_LVDS_PHY_GPIO_DQ6,                       /**< Data pin #6 */
    CY_LVDS_PHY_GPIO_DQ7,                       /**< Data pin #7 */
    CY_LVDS_PHY_GPIO_DQ8,                       /**< Data pin #8 */
    CY_LVDS_PHY_GPIO_DQ9,                       /**< Data pin #9 */
    CY_LVDS_PHY_GPIO_DQ10,                      /**< Data pin #10 */
    CY_LVDS_PHY_GPIO_DQ11,                      /**< Data pin #11 */
    CY_LVDS_PHY_GPIO_DQ12,                      /**< Data pin #12 */
    CY_LVDS_PHY_GPIO_DQ13,                      /**< Data pin #13 */
    CY_LVDS_PHY_GPIO_DQ14,                      /**< Data pin #14 */
    CY_LVDS_PHY_GPIO_DQ15,                      /**< Data pin #15 */
    CY_LVDS_PHY_GPIO_CTL0,                      /**< Control pin #0 */
    CY_LVDS_PHY_GPIO_CTL1,                      /**< Control pin #1 */
    CY_LVDS_PHY_GPIO_CTL2,                      /**< Control pin #2 */
    CY_LVDS_PHY_GPIO_CTL3,                      /**< Control pin #3 */
    CY_LVDS_PHY_GPIO_CTL4,                      /**< Control pin #4 */
    CY_LVDS_PHY_GPIO_CTL5,                      /**< Control pin #5 */
    CY_LVDS_PHY_GPIO_CTL6,                      /**< Control pin #6 */
    CY_LVDS_PHY_GPIO_CTL7,                      /**< Control pin #7 */
    CY_LVDS_PHY_GPIO_CTL8,                      /**< Control pin #8 */
    CY_LVDS_PHY_GPIO_CTL9                       /**< Control pin #9 */
} cy_en_lvds_phy_gpio_index_t;

/**
 * @typedef cy_en_lvds_phy_gpio_dir_t
 * @brief GPIO direction options.
 */
typedef enum
{
    CY_LVDS_PHY_GPIO_INPUT = 0,                 /**< Pin configured as input to device */
    CY_LVDS_PHY_GPIO_OUTPUT                     /**< Pin configured as output from device */
} cy_en_lvds_phy_gpio_dir_t;

/**
 * @typedef cy_en_lvds_phy_gpio_intr_mode_t
 * @brief GPIO interrupt modes.
 */
typedef enum
{
    CY_LVDS_PHY_GPIO_NO_INTERRUPT = 0,          /**< No interrupt detection enabled. */
    CY_LVDS_PHY_GPIO_INTERRUPT_POSEDGE,         /**< Interrupt on rising edge. */
    CY_LVDS_PHY_GPIO_INTERRUPT_NEGEDGE,         /**< Interrupt on falling edge. */
    CY_LVDS_PHY_GPIO_INTERRUPT_ANY_EDGE         /**< Interrupt on both edges. */
} cy_en_lvds_phy_gpio_intr_mode_t;

/**
 *  @typedef cy_en_lvds_status_t
 *  @brief List of status codes returned by LVDS driver.
 */
typedef enum
{
    CY_LVDS_SUCCESS        = 0x00u,                                     /**< API passed. */
    CY_LVDS_BAD_PARAMETER  = CY_LVDS_ID | CY_PDL_STATUS_ERROR | 0x01u,  /**< Bad parameter passed in to the API. */
    CY_LVDS_CONFIG_ERROR   = CY_LVDS_ID | CY_PDL_STATUS_ERROR | 0x02u,  /**< Bad IP configuration specified. */
    CY_LVDS_TIMEOUT_ERROR  = CY_LVDS_ID | CY_PDL_STATUS_ERROR | 0x03u   /**< Operation timed out. */
} cy_en_lvds_status_t;

/**
 *  @typedef cy_en_lvds_md_type_t
 *  @brief LVDS Metadata element types.
 */
typedef enum
{
    CY_LVDS_MD_CONSTANT = 0,            /**< Use constant value for the Metadata field. */
    CY_LVDS_MD_VARIABLE = 1             /**< Use one of the IP variables for the Metadata field. */
} cy_en_lvds_md_type_t;

/**
 *  @typedef cy_en_lvds_phy_ctrl_bus_dir_t
 *  @brief List of control signal direction setting.
 */
typedef enum
{
    CY_LVDS_PHY_CTRL_BUS_DIR_INPUT       = 0,   /**< Configure as an input. */
    CY_LVDS_PHY_CTRL_BUS_DIR_OUTPUT      = 1,   /**< Configure as an output. */
    CY_LVDS_PHY_CTRL_BUS_DIR_BI_DIR      = 2,   /**< Configure as a bi-direction IO. */
    CY_LVDS_PHY_CTRL_BUS_DIR_OPEN_DRAIN  = 3    /**< Configure as an open-drain IO. */
} cy_en_lvds_phy_ctrl_bus_dir_t;

/**
 *  @typedef cy_en_lvds_phy_ctrl_bus_out_signal_t
 *  @brief Control bus output mapping to various signals available in GPIF.
 */
typedef enum
{
    CY_LVDS_PHY_BUS_OUT_ALPHA_4                 = 0,    /**< Connect control bus to Alpha No. 4 */
    CY_LVDS_PHY_BUS_OUT_ALPHA_5                 = 1,    /**< Connect control bus to Alpha No. 5 */
    CY_LVDS_PHY_BUS_OUT_ALPHA_6                 = 2,    /**< Connect control bus to Alpha No. 6 */
    CY_LVDS_PHY_BUS_OUT_ALPHA_7                 = 3,    /**< Connect control bus to Alpha No. 7 */
    CY_LVDS_PHY_BUS_OUT_BETA_0                  = 8,    /**< Connect control bus to Beta No. 0 */
    CY_LVDS_PHY_BUS_OUT_BETA_1                  = 9,    /**< Connect control bus to Beta No. 1 */
    CY_LVDS_PHY_BUS_OUT_BETA_2                  = 10,   /**< Connect control bus to Beta No. 2 */
    CY_LVDS_PHY_BUS_OUT_BETA_3                  = 11,   /**< Connect control bus to Beta No. 3 */
    CY_LVDS_PHY_BUS_OUT_EMPTY_FULL_FLAG_TH_0    = 16,   /**< Connect control bus to Empty/Full flag of thread 0 */
    CY_LVDS_PHY_BUS_OUT_EMPTY_FULL_FLAG_TH_1    = 17,   /**< Connect control bus to Empty/Full flag of thread 1 */
    CY_LVDS_PHY_BUS_OUT_EMPTY_FULL_FLAG_TH_2    = 18,   /**< Connect control bus to Empty/Full flag of thread 2 */
    CY_LVDS_PHY_BUS_OUT_EMPTY_FULL_FLAG_TH_3    = 19,   /**< Connect control bus to Empty/Full flag of thread 3 */
    CY_LVDS_PHY_BUS_OUT_PARTIAL_FLAG_TH_0       = 20,   /**< Connect control bus to partial flag of thread 0 */
    CY_LVDS_PHY_BUS_OUT_PARTIAL_FLAG_TH_1       = 21,   /**< Connect control bus to partial flag of thread 1 */
    CY_LVDS_PHY_BUS_OUT_PARTIAL_FLAG_TH_2       = 22,   /**< Connect control bus to partial flag of thread 2 */
    CY_LVDS_PHY_BUS_OUT_PARTIAL_FLAG_TH_3       = 23,   /**< Connect control bus to partial flag of thread 3 */
    CY_LVDS_PHY_BUS_OUT_EMPTY_FULL_FLAG_CUR_TH  = 24,   /**< Connect control bus to Empty/Full flag of current thread. */
    CY_LVDS_PHY_BUS_OUT_PARTIAL_FLAG_CUR_TH     = 25,   /**< Connect control bus to partial flag of current thread. */
    CY_LVDS_PHY_1_USEC_PULSE_SIGNAL             = 26,   /**< Connect control bus to 1us pulse signal. */
    CY_LVDS_PHY_BUS_OUT_LOGIC_0                 = 27,   /**< Connect control bus to LOGIC 0 */
} cy_en_lvds_phy_ctrl_bus_out_signal_t;

/**
 *  @typedef cy_en_lvds_phy_gear_ratio_t
 *  @brief List of LVDS SIP gearing ratios.
 */
typedef enum
{
    CY_LVDS_PHY_GEAR_RATIO_1_1 = 0,                     /**< Gearing ratio of 1:1 */
#if (!LVCMOS_16BIT_SDR)
    CY_LVDS_PHY_GEAR_RATIO_2_1 = 1,                     /**< Gearing ratio of 2:1 */
    CY_LVDS_PHY_GEAR_RATIO_4_1 = 2,                     /**< Gearing ratio of 4:1 */
    CY_LVDS_PHY_GEAR_RATIO_8_1 = 3                      /**< Gearing ratio of 8:1 */
#endif /* (!LVCMOS_16BIT_SDR) */
} cy_en_lvds_phy_gear_ratio_t;

/**
 *  @typedef cy_en_lvds_gpif_comp_type_t
 *  @brief Types of GPIF comparators.
 */
typedef enum
{
    CY_LVDS_GPIF_COMP_CTRL = 0,                         /**< Control bus comparator. */
    CY_LVDS_GPIF_COMP_ADDR = 1,                         /**< Address bus comparator. */
    CY_LVDS_GPIF_COMP_DATA = 2                          /**< Data bus comparator. */
} cy_en_lvds_gpif_comp_type_t;

/**
 *  @typedef cy_en_lvds_phy_ad_bus_dir_t
 *  @brief Data bus direction configuration.
 */
typedef enum
{
    CY_LVDS_PHY_AD_BUS_DIR_INPUT      = 0,              /**< Data bus direction as input. */
    CY_LVDS_PHY_AD_BUS_DIR_OUTPUT     = 1,              /**< Data bus direction as ouput. */
    CY_LVDS_PHY_AD_BUS_ALPHA_CONTROL  = 2               /**< Data bus direction controlled by Alpha signal. */
} cy_en_lvds_phy_ad_bus_dir_t;

/**
 *  @typedef cy_en_lvds_gpif_slave_master_t
 *  @brief LVCMOS CLK Slave/Master configuration.
 */
typedef enum
{
    CY_LVDS_LVCMOS_CLK_SLAVE      = 0,              /**< LVCMOS CLK as Slave  (LVCMOS clock pin is driven by External FPGA). */
    CY_LVDS_LVCMOS_CLK_MASTER     = 1               /**< LVCMOS CLk as Master (LVCMOS clock pin is driven by DUT). */
} cy_en_lvds_gpif_slave_master_t;

/**
 *  @typedef cy_en_lvds_master_clk_src_t
 *  @brief LVCMOS Master Clk Source
 */
typedef enum
{
    CY_LVDS_MASTER_CLK_SRC_USB2     = 0,              /**< LVCMOS CLK OUT  derived from PLL in USBHS block. */
    CY_LVDS_MASTER_CLK_SRC_HF       = 1,              /**< LVCMOS CLK OUT  derived from FX10 system level high frequency clock.*/
    CY_LVDS_MASTER_CLK_SRC_PLL      = 2               /**< LVCMOS CLK OUT  derived from PLL. */
} cy_en_lvds_master_clk_src_t;

/**
 *  @typedef cy_en_lvds_phy_mode_sel_t
 *  @brief LVDS SIP modes of operation.
 */
typedef enum
{
    CY_LVDS_PHY_MODE_LVCMOS  =  0,                      /**< Sensor interface port configured as LVCMOS. */
    CY_LVDS_PHY_MODE_LVDS    =  1                       /**< Sensor interface port configured as LVDS.  */
} cy_en_lvds_phy_mode_sel_t;

/**
 *  @typedef cy_en_lvds_phy_data_bus_width_t
 *  @brief LVDS SIP data bus width.
 */
typedef enum
{
#if (!LVCMOS_16BIT_SDR)
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_1     = 0,           /**< LVDS data bus width 1 bit. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_2     = 1,           /**< LVDS data bus width 2 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_3     = 2,           /**< LVDS data bus width 3 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_4     = 3,           /**< LVDS data bus width 4 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_5     = 4,           /**< LVDS data bus width 5 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_6     = 5,           /**< LVDS data bus width 6 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_7     = 6,           /**< LVDS data bus width 7 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_8     = 7,           /**< LVDS data bus width 8 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_9     = 8,           /**< LVDS wide data bus width 9 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_10    = 9,           /**< LVDS wide data bus width 10 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_11    = 10,          /**< LVDS wide data bus width 11 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_12    = 11,          /**< LVDS wide data bus width 12 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_13    = 12,          /**< LVDS wide data bus width 13 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_14    = 13,          /**< LVDS wide data bus width 14 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_15    = 14,          /**< LVDS wide data bus width 15 bits. */
    CY_LVDS_PHY_LVDS_MODE_NUM_LANE_16    = 15,          /**< LVDS wide data bus width 16 bits. */
#endif /* LVCMOS_16BIT_SDR */
    CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_8   = 0,           /**< LVCMOS data bus width 8 bit. */
    CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16  = 1,           /**< LVCMOS data bus width 16 bit. */
    CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_24  = 2,           /**< LVCMOS wide data bus width 24 bit. */
    CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_32  = 3            /**< LVCMOS wide data bus width 32 bit. */
} cy_en_lvds_phy_data_bus_width_t;

/**
 *  @typedef cy_en_lvds_gpif_alpha_t
 *  @brief GPIF Alpha signals.
 */
typedef enum
{
    CY_LVDS_GPIF_ALPHA_DQ_OEN   = 0x01,             /**< Bit 00 = Output enable for Data bus. */
    CY_LVDS_GPIF_ALPHA_UPD_DOUT = 0x02,             /**< Bit 01 = Update output data by popping from FIFO. */
    CY_LVDS_GPIF_ALPHA_SAMP_DIN = 0x04,             /**< Bit 02 = Sample the input data. */
    CY_LVDS_GPIF_ALPHA_SAMP_AIN = 0x08,             /**< Bit 03 = Sample the input address. */
    CY_LVDS_GPIF_ALPHA_USER0    = 0x10,             /**< Bit 04 = User selected alpha. */
    CY_LVDS_GPIF_ALPHA_USER1    = 0x20,             /**< Bit 05 = User selected alpha. */
    CY_LVDS_GPIF_ALPHA_USER2    = 0x40,             /**< Bit 06 = User selected alpha. */
    CY_LVDS_GPIF_ALPHA_USER3    = 0x80              /**< Bit 07 = User selected alpha. */
} cy_en_lvds_gpif_alpha_t;

/**
 *  @typedef cy_en_lvds_gpif_beta_t
 *  @brief GPIF Beta signals.
 */
typedef enum
{
    CY_LVDS_GPIF_BETA_USER0      = 0x00000001U,     /**< Bit 00 = User selected beta. */
    CY_LVDS_GPIF_BETA_USER1      = 0x00000002U,     /**< Bit 01 = User selected beta. */
    CY_LVDS_GPIF_BETA_USER2      = 0x00000004U,     /**< Bit 02 = User selected beta. */
    CY_LVDS_GPIF_BETA_USER3      = 0x00000008U,     /**< Bit 03 = User selected beta. */
    CY_LVDS_GPIF_BETA_RESERVED0  = 0x00000010U,     /**< Bit 04 = Thread number field: Set separately. */
    CY_LVDS_GPIF_BETA_RESERVED1  = 0x00000020U,     /**< Bit 05 = Thread number field: Set separately. */
    CY_LVDS_GPIF_BETA_POP_RQ     = 0x00000040U,     /**< Bit 06 = Pop data from read queue. */
    CY_LVDS_GPIF_BETA_PUSH_WQ    = 0x00000080U,     /**< Bit 07 = Push data into write queue. */
    CY_LVDS_GPIF_BETA_POP_ARQ    = 0x00000100U,     /**< Bit 08 = Pop address from read queue. */
    CY_LVDS_GPIF_BETA_PUSH_AWQ   = 0x00000200U,     /**< Bit 09 = Push address into write queue. */
    CY_LVDS_GPIF_BETA_ADDR_OEN   = 0x00000400U,     /**< Bit 10 = Output enable for address. */
    CY_LVDS_GPIF_BETA_CTRL_OEN   = 0x00000800U,     /**< Bit 11 = Output enable for control pins set as outputs. */
    CY_LVDS_GPIF_BETA_INC_CTRL   = 0x00001000U,     /**< Bit 12 = Increment control counter. */
    CY_LVDS_GPIF_BETA_RST_CTRL   = 0x00002000U,     /**< Bit 13 = Reset control counter. */
    CY_LVDS_GPIF_BETA_INC_ADDR   = 0x00004000U,     /**< Bit 14 = Increment address counter. */
    CY_LVDS_GPIF_BETA_RST_ADDR   = 0x00008000U,     /**< Bit 15 = Reset address counter. */
    CY_LVDS_GPIF_BETA_INC_DATA   = 0x00010000U,     /**< Bit 16 = Increment data counter. */
    CY_LVDS_GPIF_BETA_RST_DATA   = 0x00020000U,     /**< Bit 17 = Reset data counter. */
    CY_LVDS_GPIF_BETA_INTR_CPU   = 0x00040000U,     /**< Bit 18 = Raise interrupt to ARM CPU. */
    CY_LVDS_GPIF_BETA_INTR_HOST  = 0x00080000U,     /**< Bit 19 = Raise interrupt to interface host (peer). */
    CY_LVDS_GPIF_BETA_UPD_AOUT   = 0x00100000U,     /**< Bit 20 = Update the address for output. */
    CY_LVDS_GPIF_BETA_REG_ACCESS = 0x00200000U,     /**< Bit 21 = Perform MMIO register access. */
    CY_LVDS_GPIF_BETA_INIT_CRC   = 0x00400000U,     /**< Bit 22 = Initialize CRC computation. */
    CY_LVDS_GPIF_BETA_CALC_CRC   = 0x00800000U,     /**< Bit 23 = Include data in CRC calculation. */
    CY_LVDS_GPIF_BETA_USE_CRC    = 0x01000000U,     /**< Bit 24 = Check CRC against input value, or output CRC. */
    CY_LVDS_GPIF_BETA_SET_DRQ    = 0x02000000U,     /**< Bit 25 = Assert the DRQ output. */
    CY_LVDS_GPIF_BETA_CLR_DRQ    = 0x04000000U,     /**< Bit 26 = De-assert the DRQ output. */
    CY_LVDS_GPIF_BETA_THR_DONE   = 0x08000000U,     /**< Bit 27 = Shut-down thread. */
    CY_LVDS_GPIF_BETA_THR_WRAPUP = 0x10000000U,     /**< Bit 28 = Shut-down thread and wrap up buffer. */
    CY_LVDS_GPIF_BETA_SEND_EOP   = 0x20000000U,     /**< Bit 29 = Send EOP to DMA adapter. */
    CY_LVDS_GPIF_BETA_SEND_EOT   = 0x40000000U,     /**< Bit 30 = Send EOT to DMA adapter. */
} cy_en_lvds_gpif_beta_t;

/**
 *  @typedef cy_en_lvds_gpif_lambda_t
 *  @brief GPIF Lambda signals.
 */
typedef enum
{
    CY_LVDS_GPIF_LAMBDA_CTRL00 = 0,             /**< Bit 00: CTRL[0] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL01,                 /**< Bit 01: CTRL[1] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL02,                 /**< Bit 02: CTRL[2] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL03,                 /**< Bit 03: CTRL[3] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL04,                 /**< Bit 04: CTRL[4] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL05,                 /**< Bit 05: CTRL[5] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL06,                 /**< Bit 06: CTRL[6] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL07,                 /**< Bit 07: CTRL[7] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL08,                 /**< Bit 08: CTRL[8] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL09,                 /**< Bit 09: CTRL[9] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL10,                 /**< Bit 10: CTRL[10] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL11,                 /**< Bit 11: CTRL[11] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL12,                 /**< Bit 12: CTRL[12] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL13,                 /**< Bit 13: CTRL[13] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL14,                 /**< Bit 14: CTRL[14] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL15,                 /**< Bit 15: CTRL[15] input. */
    CY_LVDS_GPIF_LAMBDA_DOUT_VALID,             /**< Bit 16: EGRESS_DATA_VALID */
    CY_LVDS_GPIF_LAMBDA_DIN_VALID,              /**< Bit 17: INGRESS_DATA_VALID */
    CY_LVDS_GPIF_LAMBDA_CTRL_CNT,               /**< Bit 18: Control counter hit limit. */
    CY_LVDS_GPIF_LAMBDA_ADDR_CNT,               /**< Bit 19: Address counter hit limit. */
    CY_LVDS_GPIF_LAMBDA_DATA_CNT,               /**< Bit 20: Data counter hit limit. */
    CY_LVDS_GPIF_LAMBDA_CTRL_CMP,               /**< Bit 21: Control comparator match. */
    CY_LVDS_GPIF_LAMBDA_ADDR_CMP,               /**< Bit 22: Address comparator match. */
    CY_LVDS_GPIF_LAMBDA_DATA_CMP,               /**< Bit 23: Data comparator match. */
    CY_LVDS_GPIF_LAMBDA_DMA_THRES,              /**< Bit 24: Current thread is above DMA threshold (watermark). */
    CY_LVDS_GPIF_LAMBDA_DMA_AREADY,             /**< Bit 25: Address socket is ready to be written into / read from. */
    CY_LVDS_GPIF_LAMBDA_DMA_DREADY,             /**< Bit 26: Data socket is ready to be written into / read from. */
    CY_LVDS_GPIF_LAMBDA_CRC_ERROR,              /**< Bit 27: CRC error has been detected. */
    CY_LVDS_GPIF_LAMBDA_ONE,                    /**< Bit 28: Logic: 1 */
    CY_LVDS_GPIF_LAMBDA_INTR_ACTV,              /**< Bit 29: CPU interrupt is already active (pending). */
    CY_LVDS_GPIF_LAMBDA_CPU_DEF,                /**< Bit 30: CPU controlled lambda signal. */
    CY_LVDS_GPIF_LAMBDA_EG_ADDR_VALID,          /**< Bit 31: Egress address valid. */
    CY_LVDS_GPIF_LAMBDA_CTRL16,                 /**< Bit 32: CTRL[16] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL17,                 /**< Bit 33: CTRL[17] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL18,                 /**< Bit 34: CTRL[18] input. */
    CY_LVDS_GPIF_LAMBDA_CTRL19,                 /**< Bit 35: CTRL[19] input. */
    CY_LVDS_GPIF_LAMBDA_LINK_ERROR,             /**< Bit 36: Link error detected. */
    CY_LVDS_GPIF_LAMBDA_TSEQ,                   /**< Bit 37: Link training sequence detected. */
    CY_LVDS_GPIF_LAMBDA_LINK_IDLE,              /**< Bit 38: Link idle command detected. */
    CY_LVDS_GPIF_LAMBDA_DATA_IN,                /**< Bit 39: Data command detected. */
    CY_LVDS_GPIF_LAMBDA_INSERT_MD,              /**< Bit 40: Insert metadata command detected. */
    CY_LVDS_GPIF_LAMBDA_EOP,                    /**< Bit 41: EOP command detected. */
    CY_LVDS_GPIF_LAMBDA_CHK_DCRC,               /**< Bit 42: Check data CRC command detected. */
    CY_LVDS_GPIF_LAMBDA_INIT_MCRC,              /**< Bit 43: Initialize metadata CRC. */
    CY_LVDS_GPIF_LAMBDA_INIT_DCRC,              /**< Bit 44: Initialize data CRC. */
    CY_LVDS_GPIF_LAMBDA_SET_FLAG,               /**< Bit 45: Set flag command detected. */
    CY_LVDS_GPIF_LAMBDA_CLEAR_FLAG,             /**< Bit 46: Clear flag command detected. */
    CY_LVDS_GPIF_LAMBDA_SET_THREAD,             /**< Bit 47: Select thread command detected. */
    CY_LVDS_GPIF_LAMBDA_SET_SOCKET,             /**< Bit 48: Select socket command detected. */
    CY_LVDS_GPIF_LAMBDA_SET_MD_L,               /**< Bit 49: Set metadata variable low word command. */
    CY_LVDS_GPIF_LAMBDA_SET_MD_H,               /**< Bit 50: Set metadata variable high word command. */
    CY_LVDS_GPIF_LAMBDA_UPDATE_EVENT_CNT,       /**< Bit 51: Update event count command detected. */
    CY_LVDS_GPIF_LAMBDA_CLEAR_EVENT_CNT,        /**< Bit 52: Clear event count command detected. */
    CY_LVDS_GPIF_LAMBDA_EN_PAYLOAD_CNT,         /**< Bit 53: Enable payload count command detected. */
    CY_LVDS_GPIF_LAMBDA_HALT_PAYLOAD_CNT,       /**< Bit 54: Halt payload count command detected. */
    CY_LVDS_GPIF_LAMBDA_CLEAR_PAYLOAD_CNT,      /**< Bit 55: Clear payload count command detected. */
    CY_LVDS_GPIF_LAMBDA_L1_ENTER,               /**< Bit 56: L1 state entry command. */
    CY_LVDS_GPIF_LAMBDA_L1_EXIT,                /**< Bit 57: L1 state exit command. */
    CY_LVDS_GPIF_LAMBDA_SOCKET_B0,              /**< Bit 58: Current socket number bit 0 */
    CY_LVDS_GPIF_LAMBDA_SOCKET_B1,              /**< Bit 59: Current socket number bit 1 */
    CY_LVDS_GPIF_LAMBDA_SOCKET_B2,              /**< Bit 60: Current socket number bit 2 */
    CY_LVDS_GPIF_LAMBDA_SOCKET_B3,              /**< Bit 61: Current socket number bit 3 */
    CY_LVDS_GPIF_LAMBDA_SOCKET_B4,              /**< Bit 62: Current socket number bit 4 */
    CY_LVDS_GPIF_LAMBDA_RATEMATCH               /**< Bit 63: Valid signal for lambda bits from 37 to 57 */
} cy_en_lvds_gpif_lambda_t;

/**
 *  @typedef cy_en_lvds_gpif_event_type_t
 *  @brief GPIF event types.
 */
typedef enum
{
    CY_LVDS_GPIF_EVT_END_STATE = 0,               /**< State machine has reached the designated end state. */
    CY_LVDS_GPIF_EVT_SM_INTERRUPT,                /**< State machine has raised a software interrupt. */
    CY_LVDS_GPIF_EVT_SWITCH_TIMEOUT,              /**< Desired state machine switch has timed out. */
    CY_LVDS_GPIF_EVT_CRC_ERROR,                   /**< Desired state machine switch has timed out. */
    CY_LVDS_GPIF_EVT_ADDR_COUNTER,                /**< Address counter has reached the limit. */
    CY_LVDS_GPIF_EVT_DATA_COUNTER,                /**< Data counter has reached the limit. */
    CY_LVDS_GPIF_EVT_CTRL_COUNTER,                /**< Control counter has reached the limit. */
    CY_LVDS_GPIF_EVT_ADDR_COMP,                   /**< Address comparator match has been obtained. */
    CY_LVDS_GPIF_EVT_DATA_COMP,                   /**< Data comparator match has been obtained. */
    CY_LVDS_GPIF_EVT_CTRL_COMP,                   /**< Control comparator match has been obtained. */
    CY_LVDS_GPIF_EVT_WAVEFORM_BUSY,               /**< CPU accessed waveform memory when it is busy */
    CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH0,           /**< Egress data register is empty for thread 0 */
    CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH1,           /**< Egress data register is empty for thread 1 */
    CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH2,           /**< Egress data register is empty for thread 2 */
    CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH3,           /**< Egress data register is empty for thread 3 */
    CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH0,           /**< Ingress data register is valid for thread 0 */
    CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH1,           /**< Ingress data register is valid for thread 1 */
    CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH2,           /**< Ingress data register is valid for thread 2 */
    CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH3,           /**< Ingress data register is valid for thread 3 */
    CY_LVDS_GPIF_EVT_CONTROL_BYTE_INVALID         /**< Invalid control byte received. */
} cy_en_lvds_gpif_event_type_t;

/**
 *  @typedef cy_en_lvds_gpif_error_t
 *  @brief GPIF error types.
 */
typedef enum
{
    CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE  = 1, /**< Ingress address field over-written. */
    CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID,       /**< Egress read while data not valid. */
    CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR,       /**< DMA data read error. */
    CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR,       /**< DMA data write error. */
    CY_LVDS_GPIF_ERROR_DMA_ADDR_RD_ERROR,       /**< DMA address read error. */
    CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR,       /**< DMA address write error. */
    CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR = 8  /**< Invalid GPIF state error. */
} cy_en_lvds_gpif_error_t;

/**
 *  @typedef cy_en_lvds_gpif_thread_event_t
 *  @brief Header flag set/clear type.
 */
typedef enum
{
    CY_LVDS_GPIF_HDR_FLG0_CLR = 0,              /**< Flag-0 has been cleared. */
    CY_LVDS_GPIF_HDR_FLG1_CLR,                  /**< Flag-1 has been cleared. */
    CY_LVDS_GPIF_HDR_FLG2_CLR,                  /**< Flag-2 has been cleared. */
    CY_LVDS_GPIF_HDR_FLG3_CLR,                  /**< Flag-3 has been cleared. */
    CY_LVDS_GPIF_HDR_FLG0_SET,                  /**< Flag-0 has been set. */
    CY_LVDS_GPIF_HDR_FLG1_SET,                  /**< Flag-1 has been set. */
    CY_LVDS_GPIF_HDR_FLG2_SET,                  /**< Flag-2 has been set. */
    CY_LVDS_GPIF_HDR_FLG3_SET                   /**< Flag-3 has been set. */
} cy_en_lvds_gpif_thread_event_t;

/**
 *  @typedef cy_en_lvds_gpif_thread_error_t
 *  @brief Thread error types.
 */
typedef enum
{
    CY_LVDS_GPIF_THREAD_DIR_ERROR      = 1,     /**< Incorrect DMA transfer direction. */
    CY_LVDS_GPIF_THREAD_WR_OVERFLOW,            /**< DMA write overflow. */
    CY_LVDS_GPIF_THREAD_RD_UNDERRUN,            /**< DMA read underflow. */
    CY_LVDS_GPIF_THREAD_SCK_ACTIVE     = 0xA,   /**< Socket became inactive during transfer. */
    CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW,          /**< Overflow on adapter interface. */
    CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW,         /**< Underflow on adapter interface. */
    CY_LVDS_GPIF_THREAD_READ_FORCE_END,         /**< Socket wrapped up while reading. */
    CY_LVDS_GPIF_THREAD_READ_BURST_ERR          /**< Burst read interrupted. */
} cy_en_lvds_gpif_thread_error_t;

/**
 *  @typedef cy_en_lvds_gpif_thread_no_t
 *  @brief Thread number index.
 */
typedef enum
{
    CY_LVDS_GPIF_THREAD_0 = 0,                  /**< LVDS DMA thread #0 */
    CY_LVDS_GPIF_THREAD_1,                      /**< LVDS DMA thread #1 */
#if (!LVCMOS_16BIT_SDR)
    CY_LVDS_GPIF_THREAD_2,                      /**< LVDS DMA thread #2 */
    CY_LVDS_GPIF_THREAD_3                       /**< LVDS DMA thread #3 */
#endif /* (!LVCMOS_16BIT_SDR) */
} cy_en_lvds_gpif_thread_no_t;

/**
 *  @typedef cy_en_lvds_phy_events_t
 *  @brief LVDS PHY events.
 */
typedef enum
{
    CY_LVDS_PHY_GPIO_INTR_PORT = 0x0,           /**< GPIO edge detect interrupt. */
    CY_LVDS_PHY_MONITOR_FAIL,                   /**< Unable to adjust clock phase selection. Deskew re-run required. */
    CY_LVDS_PHY_PLL_LOCK_LOST,                  /**< PLL lock lost. */
    CY_LVDS_PHY_MDLL_LOCK_LOST,                 /**< Master DLL lock lost. */
    CY_LVDS_PHY_FF_OVERFLOW,                    /**< FIFO overflow. */
    CY_LVDS_PHY_TRAINING_DONE,                  /**< PHY training completed. */
    CY_LVDS_PHY_LNK_TRAIN_BLK_DET,              /**< Link training block detected. */
    CY_LVDS_PHY_LNK_TRAIN_BLK_DET_FAIL,         /**< Link training block missed before training is complete. */
    CY_LVDS_PHY_L1_EXIT,                        /**< L1 state exited. */
    CY_LVDS_PHY_L1_ENTRY,                       /**< L1 state entered. */
    CY_LVDS_PHY_L3_ENTRY                        /**< L3 state entered. */
} cy_en_lvds_phy_events_t;

/**
 *  @typedef cy_en_lvds_low_power_events_t
 *  @brief LVDS L3 events for Sensor Interface Ports.
 */
typedef enum
{
    CY_LVDS_LOW_POWER_LNK0_L3_EXIT = 0x0,       /**< Link-0 (SIP0) exited L3 state. */
    CY_LVDS_LOW_POWER_LNK1_L3_EXIT              /**< Link-1 (SIP1) exited L3 state. */
} cy_en_lvds_low_power_events_t;

/**
 *  @typedef cy_en_lvds_gpif_clk_src_t
 *  @brief GPIF clock source types.
 */
typedef enum
{
    CY_LVDS_GPIF_CLOCK_USB2 = 0,                /**< Clock derived from PLL in USBHS block. */
    CY_LVDS_GPIF_CLOCK_HF = 1,                  /**< Clock derived from device level high frequency clock. */
    CY_LVDS_GPIF_CLOCK_LVCMOS_IF = 2            /**< External clock input received on SIP interface. */
} cy_en_lvds_gpif_clk_src_t;

/**
 *  @typedef cy_en_lvds_gpif_clk_divider_t
 *  @brief GPIF clock divider value.
 */
typedef enum
{
    CY_LVDS_GPIF_CLOCK_DIV_INVALID = 0,         /**< Divide by 1: Can only be used with CY_LVDS_GPIF_CLOCK_HF. */
    CY_LVDS_GPIF_CLOCK_DIV_2,                   /**< Divide by 2 */
    CY_LVDS_GPIF_CLOCK_DIV_3,                   /**< Divide by 3 */
    CY_LVDS_GPIF_CLOCK_DIV_4                    /**< Divide by 4 */
} cy_en_lvds_gpif_clk_divider_t;

/**
 *  @typedef cy_en_thread_intlv_t
 *  @brief Thread interleaving types.
 */
typedef enum
{
    CY_LVDS_TH0_TH1_INTERLEAVED = 0,            /**< Threads 0 and 1 are interleaved. */
#if (!LVCMOS_16BIT_SDR)
    CY_LVDS_TH2_TH3_INTERLEAVED = 1             /**< Threads 2 and 3 are interleaved. */
#endif /* (!LVCMOS_16BIT_SDR) */
} cy_en_thread_intlv_t;

/**
 *  @typedef cy_en_lvds_phy_deskew_algo_t
 *  @brief Deskew algorithm types.
 */
typedef enum
{
    CY_LVDS_PHY_DESKEW_BYPASS   = 0,            /**< No deskew: signals are bypassed. */
    CY_LVDS_PHY_DESKEW_SLOW,                    /**< Slow deskew algorithm. */
    CY_LVDS_PHY_DESKEW_SDR_FAST,                /**< Fast deskew algorithm for SDR interface. */
    CY_LVDS_PHY_DESKEW_DDR_FAST                 /**< Fast deskew algorithm for DDR interface. */
} cy_en_lvds_phy_deskew_algo_t;

/**
 *  @typedef cy_en_lvds_phy_interface_clock_t
 *  @brief LVDS PHY interface clock frequencies type.
 */
typedef enum
{
#if (!LVCMOS_16BIT_SDR)
    CY_LVDS_PHY_INTERFACE_CLK_625_MHZ = 1,      /**< Interface clock at 625 MHz. Used in LVDS mode. */
    CY_LVDS_PHY_INTERFACE_CLK_156_25_MHZ,       /**< Interface clock at 156.25 MHz. Used in LVDS mode. */
    CY_LVDS_PHY_INTERFACE_CLK_148_5_MHZ,        /**< Interface clock at 148.5 MHz. Used in LVDS mode. */
    CY_LVDS_PHY_INTERFACE_CLK_74_25_MHZ,        /**< Interface clock at 74.25 MHz. Used in LVDS mode. */
#endif /* (!LVCMOS_16BIT_SDR) */
    CY_LVDS_PHY_INTERFACE_CLK_160_MHZ = 5,      /**< Interface clock at 160 MHz. Used in LVCMOS mode. */
    CY_LVDS_PHY_INTERFACE_CLK_100_MHZ,          /**< Interface clock at 100 MHz. Used in LVCMOS mode. */
    CY_LVDS_PHY_INTERFACE_CLK_80_MHZ            /**< Interface clock at 80 MHz. Used in LVCMOS mode. */
} cy_en_lvds_phy_interface_clock_t;

/**
 *  @typedef cy_en_lvds_slave_fifo_mode_t
 *  @brief Slave FIFO modes.
 */
typedef enum
{
    CY_LVDS_NORMAL_MODE = 0,                    /**< Normal mode of LVDS interface operation. */
    CY_LVDS_SLAVE_FIFO_2BIT,                    /**< 2-bit slave fifo mode. Master selects DMA thread. */
    CY_LVDS_SLAVE_FIFO_5BIT                     /**< 5-bit slave fifo mode. Master selects DMA socket. */
} cy_en_lvds_slave_fifo_mode_t;

#if (!LVCMOS_16BIT_SDR)
/**
 *  @typedef cy_en_lvds_freq_gear_ratio_t
 *  @brief Slave FIFO modes.
 */
typedef enum
{
    CY_LVDS_CLK_625_MHZ_1_1 = 0,                /**< INTERFACE CLK 625 MHz GEAR RATIO 1:1 */
    CY_LVDS_CLK_625_MHZ_2_1,                    /**< INTERFACE CLK 625 MHz GEAR RATIO 2:1 */
    CY_LVDS_CLK_156_25_MHZ_1_1,                 /**< INTERFACE CLK 156.25 MHz GEAR RATIO 1:1 */
    CY_LVDS_CLK_156_25_MHZ_2_1,                 /**< INTERFACE CLK 156.25 MHz GEAR RATIO 2:1 */
    CY_LVDS_CLK_156_25_MHZ_4_1,                 /**< INTERFACE CLK 156.25 MHz GEAR RATIO 4:1 */
    CY_LVDS_CLK_156_25_MHZ_8_1,                 /**< INTERFACE CLK 156.25 MHz GEAR RATIO 8:1 */
    CY_LVDS_CLK_148_5_MHZ_1_1,                  /**< INTERFACE CLK 148.5 MHz GEAR RATIO 1:1 */
    CY_LVDS_CLK_148_5_MHZ_2_1,                  /**< INTERFACE CLK 148.5 MHz GEAR RATIO 2:1 */
    CY_LVDS_CLK_148_5_MHZ_4_1,                  /**< INTERFACE CLK 148.5 MHz GEAR RATIO 4:1 */
    CY_LVDS_CLK_148_5_MHZ_8_1,                  /**< INTERFACE CLK 148.5 MHz GEAR RATIO 8:1 */
    CY_LVDS_CLK_74_25_MHZ_2_1,                  /**< INTERFACE CLK 74.25 MHz GEAR RATIO 2:1 */
    CY_LVDS_CLK_74_25_MHZ_4_1,                  /**< INTERFACE CLK 74.25 MHz GEAR RATIO 4:1 */
    CY_LVDS_CLK_74_25_MHZ_8_1                   /**< INTERFACE CLK 74.25 MHz GEAR RATIO 8:1 */
} cy_en_lvds_freq_gear_ratio_t;
#endif /* (!LVCMOS_16BIT_SDR) */

/** \} group_usbfxstack_lvds_lvcmos_enums */

/**
 * \addtogroup group_usbfxstack_lvds_lvcmos_structs
 * \{
 */

/***************************************
*       Configuration Structures
***************************************/

/**
 * @brief Structure that defines each 16-bit element used in LVDS metadata structures.
 */
typedef struct
{
    uint16_t metadataValue;                     /**< Value when element is constant, selects field when variable. */
    cy_en_lvds_md_type_t metadataType;          /**< Type of the field: Constant or variable. */
} cy_stc_lvds_md_config_t;

/**
 * @brief GPIF waveform memory configuration.
 */
typedef struct
{
    uint32_t leftData[4];                       /**< 16 byte left transition descriptor. */
    uint32_t rightData[4];                      /**< 16 byte right transition descriptor. */
} cy_stc_lvds_gpif_wavedata_t;

/**
 * @brief GPIF register configuration consisting of register address offset and register value.
 */
typedef struct
{
    uint16_t regAddr;                           /**< Offset of register to be initialized. */
    uint32_t regValue;                          /**< Value to be written into the register. */
} cy_stc_lvds_gpif_reg_data_t;

/**
 * @brief Structure containing all elements required to configure entire GPIF instance.
 */
typedef const struct
{
    uint16_t                      stateCount;           /**< Number of states to be initialized. */
    cy_stc_lvds_gpif_wavedata_t   *stateData;           /**< Pointer to state descriptors structure. */
    uint8_t                       *statePosition;       /**< Pointer to array index -> state number mapping. */
    uint16_t                      functionCount;        /**< Number of transition functions to be initialized. */
    uint16_t                      *functionData;        /**< Pointer to array containing transition function data. */
    uint16_t                      regCount;             /**< Number of GPIF config registers to be initialized. */
    cy_stc_lvds_gpif_reg_data_t   *regData;             /**< Pointer to array containing GPIF register values. */
} cy_stc_lvds_gpif_config_t;


/**
 * @brief Control bus configuration settings.
 */
typedef struct
{
    cy_en_lvds_phy_ctrl_bus_dir_t           direction;          /**< Direction of control bus. */
    bool                                    polarity;           /**< Polarity of control bus. */
    bool                                    defaultValue;       /**< Default value of control bus. */
    bool                                    toggleMode;         /**< Enable toggle mode. */
    cy_en_lvds_phy_ctrl_bus_out_signal_t    ctrlBusSignal;      /**< Ctrl Out signal mapping to omega's. */
} cy_stc_lvds_phy_ctrl_bus_config_t;

/**
 * @brief Structure which defines the configuration for each Sensor Interface Port (SIP)
 * on the device.
 */
typedef struct
{
    bool                                loopbackModeEn;         /**< Enable/Disable loopback mode. */
    bool                                isPutLoopbackMode;      /**< If the port under test in loopback mode. */
    bool                                wideLink;               /**< Enables widelink on the port. */
    uint8_t                             phyTrainingPattern;     /**< PHY training pattern. */
    uint32_t                            ctrlBusBitMap;          /**< CTRL[19:0] pin bit map, input pins should be high */
    uint32_t                            linkTrainingPattern;    /**< LINK training pattern. */
    cy_en_lvds_phy_mode_sel_t           modeSelect;             /**< Select between LVDS or LVCMOS. */
    cy_en_lvds_phy_data_bus_width_t     dataBusWidth;           /**< Data bus width for the port. */
    cy_en_lvds_phy_gear_ratio_t         gearingRatio;           /**< Gearing ratio of the interface. */
    cy_en_lvds_gpif_clk_src_t           clkSrc;                 /**< Select the GPIF clock source. */
    cy_en_lvds_gpif_clk_divider_t       clkDivider;             /**< Clock divider value of GPIF clock */
    cy_en_lvds_phy_ad_bus_dir_t         dataBusDirection;       /**< Configure direction of data bus. */
    cy_en_lvds_gpif_slave_master_t      lvcmosClkMode;          /**< LVCMOS clock pin driven as Slave(FPGA) or Master(DUT) */
    cy_en_lvds_master_clk_src_t         lvcmosMasterClkSrc;     /**< LVCMOS Master Clock Source */
    cy_en_lvds_phy_interface_clock_t    interfaceClock;         /**< Input clock received from external interface. */
    cy_en_lvds_slave_fifo_mode_t        slaveFifoMode;          /**< 2bit or 5bit Slave FIFO mode */
} cy_stc_lvds_phy_config_t;

/**
 * @brief Structure providing configuration for the SIP and GPIF sections of one
 * LVDS port instance on the FX device.
 */
typedef struct
{
    cy_stc_lvds_phy_config_t *phyConfig;                        /**< Pointer to PHY config structure. */
    cy_stc_lvds_gpif_config_t *gpifConfig;                      /**< Pointer to GPIF config structure. */
} cy_stc_lvds_config_t;

/**
 * @brief Structure that encapsulates various callback functions which are invoked by the LVDS
 * IP driver.
 */
typedef struct
{
    void (* gpif_events)(
            uint8_t smNo,
            cy_en_lvds_gpif_event_type_t gpifEvent,
            void *userContext);                         /**< Callback for notification of GPIF events. */

    void (* gpif_error)(
            uint8_t smNo,
            cy_en_lvds_gpif_error_t gpifError,
            void *userContext);                         /**< Callback for notification of GPIF errors. */

    void (* gpif_thread_error)(
            cy_en_lvds_gpif_thread_no_t threadIndex,
            cy_en_lvds_gpif_thread_error_t threadError,
            void *userContext);                         /**< Callback for notification of DMA thread errors. */

    void (* gpif_thread_event)(
            cy_en_lvds_gpif_thread_no_t threadIndex,
            cy_en_lvds_gpif_thread_event_t threadEvent,
            void *userContext);                         /**< Callback for notification of DMA thread events. */

    void (* phy_events)(
            uint8_t sipNo,
            cy_en_lvds_phy_events_t phyEvent,
            void *userContext);                         /**< Callback for notification of PHY events. */

    void (* low_power_events)(
            cy_en_lvds_low_power_events_t lowPowerEvent,
            void *userContext);                         /**< Callback for notification of low power events. */
} cy_stc_lvds_app_cb_t;

/**
 * @brief LVDS driver Context structure.
 */
typedef struct
{
    LVDSSS_LVDS_Type *base;                             /**< Pointer to the control registers for the IP block. */
    void *userContext;                                  /**< Caller context to be passed to the callback function. */
    cy_stc_lvds_app_cb_t *intrCallback;                 /**< Pointer to callback structure required for interrupt notification. */
    cy_stc_lvds_phy_config_t *phyConfigP0;              /**< Pointer to PHY config structure for SIP0 */
    cy_stc_lvds_gpif_config_t *gpifConfigP0;            /**< Pointer to GPIF config structure for SIP0 */
    cy_stc_lvds_phy_config_t *phyConfigP1;              /**< Pointer to PHY config structure for SIP1 */
    cy_stc_lvds_gpif_config_t *gpifConfigP1;            /**< Pointer to GPIF config structure for SIP1 */

} cy_stc_lvds_context_t;

/** \} group_usbfxstack_lvds_lvcmos_structs */

/**
 * \addtogroup group_usbfxstack_lvds_lvcmos_functions
 * \{
 */

/***************************************
*        Function Prototypes
***************************************/

/*******************************************************************************
* Function Name: Cy_LVDS_Init
****************************************************************************//**
*
* Initialises LVDS IP block.
*
* @param base
* Pointer to the LVDS register base address
*
* @param sipNo
* Sensor Interface Port number.
*
* @param lvdsConfig
* Pointer to a structure array specifies all the parameters required to configure
* the LVDS IP block
*
* @param lvdsContext
* LVDS context structure.
*
* @return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_Init(LVDSSS_LVDS_Type * base, uint8_t sipNo,
                                const cy_stc_lvds_config_t * lvdsConfig,
                                cy_stc_lvds_context_t *lvdsContext);

/*******************************************************************************
* Function Name: Cy_LVDS_PhyInit
****************************************************************************//**
*
* Initialises LVDS Sensor interface port and Analog front end.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
* \param config
* Specifies port configuration such as bus direction, bus width etc.
* The value will provided by device configurator.
*
* \param lvdsContext
* LVDS driver context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_PhyInit(LVDSSS_LVDS_Type * base, uint8_t portNo,
                                    const cy_stc_lvds_phy_config_t * config,
                                    cy_stc_lvds_context_t *lvdsContext);

/*******************************************************************************
* Function Name: Cy_LVDS_PhyTrainingStart
****************************************************************************//**
*
* Enable PHY TRAINING in AFE and then wait for DESKEW COMPLETE to get set.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
* \param config
* Specifies port configuration such as bus direction, bus width etc.
* The value will provided by device configurator.
*
* \return cy_en_lvds_status_t
* CY_LVDS_CONFIG_ERROR - If the function is called for LVCMOS mode
* CY_LVDS_TIMEOUT_ERROR - If the DESKEW didn't get completed
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_PhyTrainingStart(LVDSSS_LVDS_Type* base, uint8_t portNo,
                                            const cy_stc_lvds_phy_config_t * config);

/*******************************************************************************
* Function Name: Cy_LVDS_PhyGpioModeEnable
****************************************************************************//**
*
* Enable GPIO mode for LVDS/LVCMOS pins.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor interface port number.
*
* \param pinNo
* GPIO pin ID.
*
* \param dir
* GPIO pin direction.
*
* \param intrMode
* GPIF pin interrupt selection. Only applies when pin is an input.
*
*******************************************************************************/
void Cy_LVDS_PhyGpioModeEnable (LVDSSS_LVDS_Type* base, uint8_t portNo,
                                cy_en_lvds_phy_gpio_index_t pinNo,
                                cy_en_lvds_phy_gpio_dir_t dir,
                                cy_en_lvds_phy_gpio_intr_mode_t intrMode);

/*******************************************************************************
* Function Name: Cy_LVDS_PhyGpioModeDisable
****************************************************************************//**
*
* Disable GPIO mode for LVDS/LVCMOS pins.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor interface port number.
*
* \param pinNo
* GPIO pin ID.
*
*******************************************************************************/
void Cy_LVDS_PhyGpioModeDisable (LVDSSS_LVDS_Type* base, uint8_t portNo,
                                cy_en_lvds_phy_gpio_index_t pinNo);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifEnableComp
****************************************************************************//**
*
* Configures one of the comparators in the GPIF block.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param compType
* Type of comparatore to be initialized.
*
* \param compValue
* The value to compare the signals against.
*
* \param compMask
* Mask that specifies which bits are to be used in the comparison.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifEnableComp(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                            cy_en_lvds_gpif_comp_type_t compType,
                                            void *compValue, void *compMask);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifDisableComp
****************************************************************************//**
*
* Resets one of the comparators in GPIF block.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param compType
* Type of comparator to be disabled.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifDisableComp(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                            cy_en_lvds_gpif_comp_type_t compType);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInitAddrCounter
****************************************************************************//**
*
* Configures GPIF Address counter.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param initValue
* Initial (reset) value for the counter.
*
* \param limit
* Value at which to stop the counter and flag an event.
*
* \param reload
* Whether to reload the counter and continue after limit is hit.
*
* \param upCount
* Whether to count upwards from the initial value.
*
* \param increment
* The value to be incremented/decremented from the counter at each step.
*
*******************************************************************************/
void Cy_LVDS_GpifInitAddrCounter(LVDSSS_LVDS_Type * base, uint8_t smNo,
                                uint32_t initValue, uint32_t limit, bool reload,
                                bool upCount, uint8_t increment);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInitCtrlCounter
****************************************************************************//**
*
* Configures GPIF control counter.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param initValue
* Initial (reset) value for the counter.
*
* \param limit
* Value at which to stop the counter and flag an event.
*
* \param reload
* Whether to reload the counter and continue after limit is hit.
*
* \param upCount
* Whether to count upwards from the initial value.
*
* \param outputBit
* Selects counter bit to be connected to CTRL[9] output.  TODO Check if this is valid
*
*******************************************************************************/
void Cy_LVDS_GpifInitCtrlCounter(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                uint32_t initValue, uint32_t limit, bool reload,
                                bool upCount, uint8_t outputBit);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInitDataCounter
****************************************************************************//**
*
* Configures GPIF data counter.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param initValueMsb
* Higher 4 byte of initial (reset) value for the counter.
*
* \param initValueLsb
* Lower 4 byte of initial (reset) value for the counter.
*
* \param limitMsb
* Higher 4 byte of value at which to stop the counter and flag an event.
*
* \param limitLsb
* Lower 4 byte of value at which to stop the counter and flag an event.
*
* \param reload
* Whether to reload the counter and continue after limit is hit.
*
* \param upCount
* Whether to count upwards from the initial value.
*
* \param increment
* The value to be incremented/decremented from the counter at each step.
*
*******************************************************************************/
void Cy_LVDS_GpifInitDataCounter(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                uint32_t initValueMsb, uint32_t initValueLsb,
                                uint32_t limitMsb, uint32_t limitLsb, bool reload,
                                bool upCount, uint8_t increment);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInitStateCounter
****************************************************************************//**
*
* Configures GPIF State counter.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param limit
* Value at which to stop the counter and flag an event.
*
*******************************************************************************/
void Cy_LVDS_GpifInitStateCounter(LVDSSS_LVDS_Type* base, uint8_t smNo, uint16_t limit);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifReadData
****************************************************************************//**
*
* This API will receive the data into GPIF registers.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \param threadIndex
* Thread number to be used.
*
* \param selectThread
* Enables SW to select thread.
*
* \param numWords
* Number of words to read and store into the buffer.
*
* \param buffer
* Pointer to the buffer where data will be copied.
*
* \param waitOption
* Specifies duration to wait for. Currently unsupported.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifReadData(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                        uint32_t threadIndex, bool selectThread,
                                        uint32_t numWords, uint32_t *buffer, uint32 waitOption);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInit
****************************************************************************//**
*
* Initializes the GPIF.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param gpifConfig
* Structure containing all the configuration values for GPIF.
*
* \param lvdsContext
* LVDS driver context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifInit(LVDSSS_LVDS_Type * base, uint8_t smNo,
                                    const cy_stc_lvds_gpif_config_t *gpifConfig,
                                    cy_stc_lvds_context_t *lvdsContext);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSMStart
****************************************************************************//**
*
* Start the GPIF state machine from the specified state.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param stateIndex
* Index of the state from where to start the state machine.
*
* \param initialAlpha
* Alpha signal to look for once the state machine starts.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
* \note
* This function should be called after initialization both the PHY and LINK layer.
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifSMStart(LVDSSS_LVDS_Type* base, uint8_t smNo,
                                        uint8_t stateIndex, uint8_t initialAlpha);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSMSwitch
****************************************************************************//**
*
* This function is used to start GPIF state machine execution from a desired state.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine number.
*
* \param fromState
* State index from which the switch needs to be initiated.
*
* \param toState
* State index of the destination state.
*
* \param endState
* State index which triggers GPIF_DONE event
*
* \param initialAlpha
* Mask of Alpha signal which needs to be asserted when the new state machine starts
* \ref cy_en_lvds_gpif_alpha_t
*
* \param switchTimeout
* Switch timeout counter value. SWITCH_TIMEOUT interrupt gets asserted once the
* timer elapses after a WAVEFORM swith is initiated.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifSMSwitch(LVDSSS_LVDS_Type * base, uint8_t smNo, uint16_t fromState,
                                        uint16_t toState, uint16_t endState, uint8_t initialAlpha,
                                        uint32_t switchTimeout);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifThreadConfig
****************************************************************************//**
*
* Configures and enables the thread. Thread number and it's various
* connfigurations are passed as parameters.
*
* \param base
* Pointer to the LVDS register base address
*
* \param threadIndex
* Thread number which needs to be configured
*
* \param socketNo
* Socket number which is to be assigned to thread.
*
* \param flagOnData
* false if flag should be asserted when data size is less than watermark, true otherwise.
*
* \param watermark
* Watermark position.
*
* \param burst
* Log2 of burst size.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS_Type* base, uint8_t threadIndex,
                                            uint32_t socketNo, bool flagOnData, uint16_t watermark,
                                            uint8_t burst);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifWaveformLoad
****************************************************************************//**
*
* Initialises the GPIF waveform based on the state machine.
* The waveform is an output from the GPIF designer.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \param stateCnt
* Number of states present in the state machine.
*
* \param firstState
* Index of the first state.
*
* \param stateDataMap
* Mapping of waveform memory with the states.
*
* \param transitionData
* Structure containing Waveform memory configuration.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifWaveformLoad(LVDSSS_LVDS_Type* base, uint8_t smNo, uint8 stateCnt,
                                            uint8_t firstState, uint8_t *stateDataMap,
                                            cy_stc_lvds_gpif_wavedata_t *transitionData);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifInitTransFunction
****************************************************************************//**
*
* Initialises the truth table for the user defined transition function in the
* GPIF State Machine
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine number.
*
* \param fnTable
* Array of values corresponding to transfer function.
* The array is an output of the GPIF designer GPIF state machine number.
*
* \note
* This function should be called after initialization both the PHY and LINK layer.
*
*******************************************************************************/
void Cy_LVDS_GpifInitTransFunction(LVDSSS_LVDS_Type * base, uint8_t smNo, uint16 *fnTable);

/*******************************************************************************
* Function Name: Cy_LVDS_IrqHandler
****************************************************************************//**
*
* LVDS Interrupt handler.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param lvdsContext
* LVDS driver context structure.
*
*******************************************************************************/
void Cy_LVDS_IrqHandler(LVDSSS_LVDS_Type * base, cy_stc_lvds_context_t *lvdsContext);

/*******************************************************************************
* Function Name: Cy_LVDS_LowPowerIrqHandler
****************************************************************************//**
*
* LVDS Low power interrupt handler.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param lvdsContext
* LVDS driver context structure.
*
*******************************************************************************/
void Cy_LVDS_LowPowerIrqHandler(LVDSSS_LVDS_Type * base, cy_stc_lvds_context_t *lvdsContext);

/*******************************************************************************
* Function Name: Cy_LVDS_InitMetadata
****************************************************************************//**
*
* Initialises LVDS Sensor interface port and Analog front end.
*
* \param base
* Pointer to the LVDS register base address
*
* \param threadNo
* There are 4 threads (0, 1, 2, 3) for which metadata insertion can pe performed.
*
* \param mdIndex
* There are 4 Metadata per thread available to be initialized.
*
* \param mdCount
* Total number of metadata which needs to be inserted
*
* \param mdBuffer
* Pointer to a structure array (cy_stc_lvds_md_config_t) specifiying the metadata
* elements which needs to be inserted.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_InitMetadata(LVDSSS_LVDS_Type * base, uint8_t threadNo,
                            uint8_t mdIndex, uint8_t mdCount, cy_stc_lvds_md_config_t * mdBuffer);

/*******************************************************************************
* Function Name: Cy_LVDS_SetLvcmosDDRClockPhase
****************************************************************************//**
*
* Select the DLL clock phase to be used on the receiver side when the Sensor
* Interface Port (SIP) is configured to work in LVCMOS DDR mode.
*
* \param sipNo
* Sensor Interface Port number.
*
* \param clkPhase
* Selected clock phase value in the range of 0 to 15.
*
*******************************************************************************/
void Cy_LVDS_SetLvcmosDDRClockPhase(uint8_t sipNo, uint8_t clkPhase);

/*******************************************************************************
* Function Name: Cy_LVDS_L3_Entry
****************************************************************************//**
*
* Handle entry into the L3 low power state for one of Sensor Interface Ports.
*
* \param base
* Pointer to SIP IP register set.
*
* \param lvdsContext
* Pointer to driver context structure.
*
* \param sipNo
* Sensor Interface Port number.
*******************************************************************************/
void Cy_LVDS_L3_Entry(LVDSSS_LVDS_Type *base,
                      cy_stc_lvds_context_t *lvdsContext,
                      uint8_t sipNo);

/*******************************************************************************
* Function Name: Cy_LVDS_L3_Exit
****************************************************************************//**
*
* Handle exit from L3 low power state for one of Sensor Interface Ports.
*
* \param base
* Pointer to SIP IP register set.
*
* \param lvdsContext
* Pointer to driver context structure.
*
* \param sipNo
* Sensor Interface Port number.
*
* \return
* Status of the L3 exit operation.
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_L3_Exit(LVDSSS_LVDS_Type * base,
                                    cy_stc_lvds_context_t *lvdsContext,
                                    uint8_t sipNo);

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSMControl
****************************************************************************//**
*
* Pause or resume the Gpif State machine.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \param pause
* pause = 1 will pause the state machine,
* pause = 0 will resume the state machine.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifSMControl(LVDSSS_LVDS_Type * base, uint8_t smNo, bool pause);

__STATIC_INLINE void Cy_LVDS_Disable                            (LVDSSS_LVDS_Type * base);
__STATIC_INLINE void Cy_LVDS_Enable                             (LVDSSS_LVDS_Type * base);
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetLambdaStatus             (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t lambdaIndex);
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetAlphaStatus              (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t alphaIndex);
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetBetaStatus               (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t betaIndex);
__STATIC_INLINE void Cy_LVDS_PhyPortEnable                      (LVDSSS_LVDS_Type * base, uint8_t portNo);
__STATIC_INLINE void Cy_LVDS_PhyPortDisable                     (LVDSSS_LVDS_Type * base, uint8_t portNo);
__STATIC_INLINE void Cy_LVDS_PhySetTrainingPattern              (LVDSSS_LVDS_Type * base, uint8_t portNo, uint8_t trainingPattern);
__STATIC_INLINE void Cy_LVDS_SetInterrupt                       (LVDSSS_LVDS_Type * base, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_ClearInterrupt                     (LVDSSS_LVDS_Type * base, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_SetInterruptMask                   (LVDSSS_LVDS_Type * base, uint32_t interruptMask);
__STATIC_INLINE uint32_t Cy_LVDS_GetInterruptStatusMasked       (LVDSSS_LVDS_Type * base);
__STATIC_INLINE uint32_t Cy_LVDS_GetInterruptStatus             (LVDSSS_LVDS_Type * base);
__STATIC_INLINE void Cy_LVDS_PhyClearInterrupt                  (LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_PhySetInterrupt                    (LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_PhySetInterruptMask                (LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interruptMask);
__STATIC_INLINE uint32_t Cy_LVDS_PhyGetInterruptStatusMasked    (LVDSSS_LVDS_Type * base, uint8_t portNo);
__STATIC_INLINE uint32_t Cy_LVDS_PhyGetInterruptStatus          (LVDSSS_LVDS_Type * base, uint8_t portNo);
__STATIC_INLINE void Cy_LVDS_GpifClearInterrupt                 (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_GpifSetInterrupt                   (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interrupt);
__STATIC_INLINE void Cy_LVDS_GpifSetInterruptMask               (LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interruptMask);
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetInterruptStatusMasked   (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetInterruptStatus         (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE void Cy_LVDS_GpifConfigClock                    (LVDSSS_LVDS_Type * base, uint8_t smNo, const cy_stc_lvds_phy_config_t * phyConfig);
__STATIC_INLINE void Cy_LVDS_ThreadIntlvEnable                  (LVDSSS_LVDS_Type * base, cy_en_thread_intlv_t threadNo);
__STATIC_INLINE void Cy_LVDS_GpifSetEndianness                  (LVDSSS_LVDS_Type * base, uint8_t smNo, bool endianType);
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetStatus                  (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetSMState                 (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE void Cy_LVDS_GpifSetFwTrig                      (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE void Cy_LVDS_GpifClearFwTrig                    (LVDSSS_LVDS_Type * base, uint8_t smNo);
__STATIC_INLINE void Cy_LVDS_RegisterCallback                   (LVDSSS_LVDS_Type *base, cy_stc_lvds_app_cb_t *lvdsCallback, cy_stc_lvds_context_t *lvdsContext, void *userContext);
__STATIC_INLINE void Cy_LVDS_SetFlagInterruptMask               (LVDSSS_LVDS_Type * base, uint32_t interruptMask);
__STATIC_INLINE uint32_t Cy_LVDS_GetFlagInterrupt               (LVDSSS_LVDS_Type * base);
__STATIC_INLINE void Cy_LVDS_SetFlagInterrupt                   (LVDSSS_LVDS_Type * base, uint32_t interruptMask);
__STATIC_INLINE void Cy_LVDS_ClearFlagInterrupt                 (LVDSSS_LVDS_Type * base, uint32_t interruptMask);
__STATIC_INLINE uint32_t Cy_LVDS_GetFlagInterruptMasked         (LVDSSS_LVDS_Type * base);
__STATIC_INLINE void Cy_LVDS_EnableLoopbackMode                 (LVDSSS_LVDS_Type * base, uint8_t put);
__STATIC_INLINE void Cy_LVDS_DisableLoopbackMode                (LVDSSS_LVDS_Type * base);
__STATIC_INLINE void Cy_LVDS_PhyGpioSet                         (LVDSSS_LVDS_Type* base, uint8_t portNo, cy_en_lvds_phy_gpio_index_t pinNo);
__STATIC_INLINE void Cy_LVDS_PhyGpioClr                         (LVDSSS_LVDS_Type* base, uint8_t portNo, cy_en_lvds_phy_gpio_index_t pinNo);
__STATIC_INLINE bool Cy_LVDS_PhyGpioRead                        (LVDSSS_LVDS_Type* base, uint8_t portNo, cy_en_lvds_phy_gpio_index_t pinNo);



/*******************************************************************************
* Function Name: Cy_LVDS_Enable
****************************************************************************//**
*
* Global enable for LINK and PHY.
*
* \param base
* Pointer to the LVDS register base address
*
* \note
* This function should be called after initialization both the PHY and LINK
* layer.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_Enable(LVDSSS_LVDS_Type * base)
{
    base->CTL = _BOOL2FLD(LVDSSS_LVDS_CTL_PHY_ENABLED,  1u) |
                _BOOL2FLD(LVDSSS_LVDS_CTL_LINK_ENABLED, 1u) |
                _BOOL2FLD(LVDSSS_LVDS_CTL_IP_ENABLED,   1u);
}

/*******************************************************************************
* Function Name: Cy_LVDS_Disable
****************************************************************************//**
*
* Global disable for LINK and PHY.
*
* \param base
* Pointer to the LVDS register base address
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_Disable(LVDSSS_LVDS_Type * base)
{
    base->CTL = _BOOL2FLD(LVDSSS_LVDS_CTL_PHY_ENABLED,  0u) |
                _BOOL2FLD(LVDSSS_LVDS_CTL_LINK_ENABLED, 0u) |
                _BOOL2FLD(LVDSSS_LVDS_CTL_IP_ENABLED,   0u);
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetLambdaStatus
****************************************************************************//**
*
* Returns the status of Lambda signals.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine number.
*
* \param lambdaIndex
* Lambda signal index for which the status needs to be checked.
*
* \return
* Returns the value of lambda signal corresponding to the index passed
*
*******************************************************************************/
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetLambdaStatus(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t lambdaIndex)
{
    if (lambdaIndex < 32)
    {
        return ((uint8_t)((base->GPIF[smNo].GPIF_LAMBDA_STAT0 >> lambdaIndex) & 0x00000001UL));
    }
    else
    {
        lambdaIndex -= 32;
        return ((uint8_t)((base->GPIF[smNo].GPIF_LAMBDA_STAT1 >> lambdaIndex) & 0x00000001UL));
    }
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetAlphaStatus
****************************************************************************//**
*
* Returns the status of Alpha signals.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \param alphaIndex
* Alpha signal index for which the status needs to be checked.
*
* \return
* Returns the value of Alpha signal corresponding to the index passed
*
*******************************************************************************/
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetAlphaStatus(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t alphaIndex)
{
    return ((uint8_t)((base->GPIF[smNo].GPIF_ALPHA_STAT >> alphaIndex) & 0x00000001UL));
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetBetaStatus
****************************************************************************//**
*
* Returns the status of Beta signals.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF State machine number.
*
* \param betaIndex
* Beta signal index for which the status needs to be checked.
*
* \return
* Returns the value of Beta signal corresponding to the index passed

*******************************************************************************/
__STATIC_INLINE uint8_t Cy_LVDS_GpifGetBetaStatus(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t betaIndex)
{
    return ((uint8_t)((base->GPIF[smNo].GPIF_BETA_STAT >> betaIndex) & 0x00000001UL));
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhyPortEnable
****************************************************************************//**
*
* Enables the Sensor Interface Port (SIP).
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor Interface Port number.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhyPortEnable(LVDSSS_LVDS_Type * base, uint8_t portNo)
{
    base->LINK_CONFIG[portNo] = _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LINK_ENABLE, 1u);
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhyPortDisable
****************************************************************************//**
*
* Disables the Sensor Interface Port (SIP).
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor Interface Port number.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhyPortDisable(LVDSSS_LVDS_Type * base, uint8_t portNo)
{
    base->LINK_CONFIG[portNo] = _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LINK_ENABLE, 0u);
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhySetTrainingPattern
****************************************************************************//**
*
* Sets the training pattern for the Sensor Interface Port.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor Interface Port number.
*
* \param trainingPattern
* The training pattern which will be sent by the FPGA master.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhySetTrainingPattern(LVDSSS_LVDS_Type * base, uint8_t portNo, uint8_t trainingPattern)
{
      base->AFE[portNo].PHY_TRAIN_CONFIG = _VAL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_SEQ, trainingPattern);
}


/*******************************************************************************
* Function Name: Cy_LVDS_SetInterruptMask
****************************************************************************//**
*
* Set global interrupt mask.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interruptMask
* The value determines which status changes will cause an interrupt.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_SetInterruptMask(LVDSSS_LVDS_Type * base, uint32_t interruptMask)
{
    base->LVDS_INTR_MASK_WD0 |= interruptMask;
}


/*******************************************************************************
* Function Name: Cy_LVDS_GetInterruptStatusMasked
****************************************************************************//**
*
* Get global interrupt masked value.
*
* \param base
* Pointer to the LVDS register base address
*
* \return
* The bit fields set indicate which interrupts are raised to the CPU.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GetInterruptStatusMasked(LVDSSS_LVDS_Type * base)
{
    return (base->LVDS_INTR_MASKED_WD0);
}

/*******************************************************************************
* Function Name: Cy_LVDS_GetInterruptStatus
****************************************************************************//**
*
* Returns the value of interrupt register.
*
* \param base
* Pointer to the LVDS register base address
*
* \return
* Value of interrupt status register.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GetInterruptStatus(LVDSSS_LVDS_Type * base)
{
    return (base->LVDS_INTR_WD0);
}

/*******************************************************************************
* Function Name: Cy_LVDS_ClearInterrupt
****************************************************************************//**
*
* Clears the interrupt register.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interrupt
* The bitmask of statuses to clear.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_ClearInterrupt(LVDSSS_LVDS_Type * base, uint32_t interrupt)
{
    base->LVDS_INTR_WD0 = interrupt;
}


/*******************************************************************************
* Function Name: Cy_LVDS_SetInterrupt
****************************************************************************//**
*
* Set the interrupt register.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interrupt
* Bit mask value of the register fields which needs to be set.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_SetInterrupt(LVDSSS_LVDS_Type * base, uint32_t interrupt)
{
    base->LVDS_INTR_SET_WD0 = interrupt;
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhySetInterruptMask
****************************************************************************//**
*
* Set PHY interrupt mask.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS block
*
* \param interruptMask
* The value determines which status changes will cause an interrupt.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhySetInterruptMask(LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interruptMask)
{
    base->AFE[portNo].PHY_INTR_MASK = interruptMask;
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhyGetInterruptStatusMasked
****************************************************************************//**
*
* Get PHY interrupt masked value.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS block
*
* \return
* The bit fields set indicate which interrupts are raised to the CPU.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_PhyGetInterruptStatusMasked(LVDSSS_LVDS_Type * base, uint8_t portNo)
{
    return (base->AFE[portNo].PHY_INTR_MASKED);
}


/*******************************************************************************
* Function Name: Cy_LVDS_PhyGetInterruptStatus
****************************************************************************//**
*
* Get PHY interrupt status.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS block
*
* \return
* Value of LVDS/LVCMOS PHY interrupt status register.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_PhyGetInterruptStatus(LVDSSS_LVDS_Type * base, uint8_t portNo)
{
    return (base->AFE[portNo].PHY_INTR);
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhyClearInterrupt
****************************************************************************//**
*
* Clear PHY interrupt status.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS block
*
* \param interrupt
* The bitmask of statuses to clear.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhyClearInterrupt(LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interrupt)
{
    base->AFE[portNo].PHY_INTR = interrupt;
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhySetInterrupt
****************************************************************************//**
*
*  Set PHY interrupt status.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS block
*
* \param interrupt
* Bit mask value of the register fields which needs to be set.
*
* \note
* This function should be called after initialization both the PHY and LINK
* layer.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhySetInterrupt(LVDSSS_LVDS_Type * base, uint8_t portNo, uint32_t interrupt)
{
    base->AFE[portNo].PHY_INTR_SET = interrupt;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSetInterruptMask
****************************************************************************//**
*
* Set GPIF interrupt mask.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine index: 0 or 1.
*
* \param interruptMask
* The value determines which status changes will cause an interrupt.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifSetInterruptMask(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interruptMask)
{
    base->GPIF[smNo].GPIF_INTR_MASK = interruptMask;
}


/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetInterruptStatusMasked
****************************************************************************//**
*
* Get GPIF interrupt masked value.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine index: 0 or 1.
*
* \return
* The bit fields set indicate which interrupts are raised to the CPU.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetInterruptStatusMasked(LVDSSS_LVDS_Type * base, uint8_t smNo)
{
    return (base->GPIF[smNo].GPIF_INTR_MASKED);
}


/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetInterruptStatus
****************************************************************************//**
*
* Get GPIF interrupt status.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine.
*
* \return
* The bit fields set indicate which interrupts are raised to the CPU.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetInterruptStatus(LVDSSS_LVDS_Type * base, uint8_t smNo)
{
    return (base->GPIF[smNo].GPIF_INTR);
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifClearInterrupt
****************************************************************************//**
*
* Clear GPIF interrupt status.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine.
*
* \param interrupt
* The bitmask of statuses to clear.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifClearInterrupt(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interrupt)
{
    base->GPIF[smNo].GPIF_INTR = interrupt;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSetInterrupt
****************************************************************************//**
*
* Set GPIF interrupt register.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF state machine index: 0 or 1.
*
* \param interrupt
* Bit mask value of the register fields which needs to be set.
*
* \note
* This function should be called after initialization both the PHY and LINK
* layer.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifSetInterrupt(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t interrupt)
{
    base->GPIF[smNo].GPIF_INTR_SET = interrupt;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifConfigClock
****************************************************************************//**
*
* Configures the clock source for GPIF.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* State Machine Number for GPIF.
*
* \param phyConfig
* LVDS PHY config structure which provides the clock information.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifConfigClock(LVDSSS_LVDS_Type* base, uint8_t smNo, const cy_stc_lvds_phy_config_t * phyConfig)
{
    bool isIfClkLvcmosSDR = false;
    if(((phyConfig->interfaceClock == CY_LVDS_PHY_INTERFACE_CLK_80_MHZ) ||
        (phyConfig->interfaceClock == CY_LVDS_PHY_INTERFACE_CLK_100_MHZ)) &&
        (phyConfig->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) && (phyConfig->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_1_1))
    {
        isIfClkLvcmosSDR = true;
    }
    base->GPIF_CLK_SEL[smNo] =  _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_GPIF_CLK_SRC, phyConfig->clkSrc) |
                                _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_USB_CLK_DIV_VAL, phyConfig->clkDivider) |
                                _BOOL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_GATE_WAVEFORM_MEM_CLK, 1) |
                                _BOOL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_LVCMOS_IF_CLK_100MHZ, isIfClkLvcmosSDR) |
                                _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_WAVEFORM_RAM_CG_DURATION, 6);
}


/*******************************************************************************
* Function Name: Cy_LVDS_ThreadIntlvEnable
****************************************************************************//**
*
* Enables resource sharing between threads.
*
* \param base
* Pointer to the LVDS register base address
*
* \param threadNo
* Mode of thread interleaving to be enabled.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_ThreadIntlvEnable(LVDSSS_LVDS_Type * base, cy_en_thread_intlv_t threadNo)
{
    if(threadNo == CY_LVDS_TH0_TH1_INTERLEAVED)
        base->THREAD_INTLV_CTL = _BOOL2FLD(LVDSSS_LVDS_THREAD_INTLV_CTL_TH0_TH1_INTERLEAVED, 1);
    else
        base->THREAD_INTLV_CTL = _BOOL2FLD(LVDSSS_LVDS_THREAD_INTLV_CTL_TH2_TH3_INTERLEAVED, 1);
}


/*******************************************************************************
* Function Name: Cy_LVDS_GpifSetEndianness
****************************************************************************//**
*
* Configure the endinaness of GPIF State machine.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \param endianType
* 0 - Little Endian
* 1 - Big Endian
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifSetEndianness(LVDSSS_LVDS_Type* base, uint8_t smNo, bool endianType)
{
    base->GPIF[smNo].GPIF_CONFIG = _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_CONFIG_ENDIAN, endianType);
}


/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetStatus
****************************************************************************//**
*
* Returns the value of GPIF status register.
*
* \param base
* Pointer to the LVDS register base address.
*
* \param smNo
* GPIF state machine number.
*
* \return
* The bit mask of the GPIF status register.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetStatus(LVDSSS_LVDS_Type* base, uint8_t smNo)
{
    return(base->GPIF[smNo].GPIF_STATUS);
}


/*******************************************************************************
* Function Name: Cy_LVDS_GpifGetSMState
****************************************************************************//**
*
* Returns the current state index of GPIF state machine.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \return
* The current state number of the GPIF state machine.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GpifGetSMState(LVDSSS_LVDS_Type * base, uint8_t smNo)
{
    return (_FLD2VAL(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CURRENT_STATE, (base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT)));
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSetFwTrig
****************************************************************************//**
*
* Sets lambda index "30" in the GPIF State Machine.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifSetFwTrig(LVDSSS_LVDS_Type * base, uint8_t smNo)
{
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT |= LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CPU_LAMBDA_Msk;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifClearFwTrig
****************************************************************************//**
*
* Clear lambda index "30" in the GPIF State Machine.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_GpifClearFwTrig(LVDSSS_LVDS_Type * base, uint8_t smNo)
{
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT &= ~LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CPU_LAMBDA_Msk;
}

/*******************************************************************************
* Function Name: Cy_LVDS_RegisterCallback
****************************************************************************//**
*
* Register callback function for all the callback types.
*
* \param base
* Pointer to the LVDS register base address
*
* \param lvdsCallback
* Pointer to a structure array which specifies all callback functions
*
* \param lvdsContext
* LVDS context structure.
*
* \param userContext
* User context data to be passed back to the callback functions.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_RegisterCallback(LVDSSS_LVDS_Type *base,
                            cy_stc_lvds_app_cb_t *lvdsCallback,
                            cy_stc_lvds_context_t *lvdsContext,
                            void *userContext)
{
    /* Supress compiler warning of unused variable */
    (void) base;
    if(lvdsContext != NULL)
    {
        lvdsContext->intrCallback = lvdsCallback;
        lvdsContext->userContext = userContext;
    }
}

/*******************************************************************************
* Function Name: Cy_LVDS_SetFlagInterruptMask
****************************************************************************//**
*
* Set header flag interrupt mask.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interruptMask
* The value determines which status changes will cause an interrupt.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_SetFlagInterruptMask(LVDSSS_LVDS_Type * base, uint32_t interruptMask)
{
    base->LVDS_INTR_MASK_WD1 = interruptMask;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GetFlagInterrupt
****************************************************************************//**
*
* Get bitmask of header flags which are set.
*
* \param base
* Pointer to the LVDS register base address
*
* \return
* The bit fields set indicate which interrupts are asserted.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GetFlagInterrupt(LVDSSS_LVDS_Type * base)
{
    return (base->LVDS_INTR_WD1);
}

/*******************************************************************************
* Function Name: Cy_LVDS_SetFlagInterrupt
****************************************************************************//**
*
* Set the header flag interrupts register.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interruptMask
* Bit mask value of the register fields which needs to be set.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_SetFlagInterrupt(LVDSSS_LVDS_Type * base, uint32_t interruptMask)
{
    base->LVDS_INTR_SET_WD1 = interruptMask;
}

/*******************************************************************************
* Function Name: Cy_LVDS_ClearFlagInterrupt
****************************************************************************//**
*
* Clears the header flag interrupt register.
*
* \param base
* Pointer to the LVDS register base address
*
* \param interrupt
* The bitmask of statuses to clear.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_ClearFlagInterrupt(LVDSSS_LVDS_Type * base, uint32_t interrupt)
{
    base->LVDS_INTR_WD1 = interrupt;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GetFlagInterruptMasked
****************************************************************************//**
*
* Get Header Flag interrupt masked value.
*
* \param base
* Pointer to the LVDS register base address
*
* \return
* The bit fields set indicate which interrupts are raised to the CPU.
*
*******************************************************************************/
__STATIC_INLINE uint32_t Cy_LVDS_GetFlagInterruptMasked(LVDSSS_LVDS_Type * base)
{
    return (base->LVDS_INTR_MASKED_WD1);
}

/*******************************************************************************
* Function Name: Cy_LVDS_EnableLoopbackMode
****************************************************************************//**
*
* Enable loopback mode.
*
* \param base
* Pointer to the LVDS register base address
*
* \param put
* Port number which is under test in loopback mode.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_EnableLoopbackMode(LVDSSS_LVDS_Type * base, uint8_t put)
{
    if(put == 0)
        base->LOOPBACK_CFG = 0x00000001;
    else
        base->LOOPBACK_CFG = 0x00000002;
}

/*******************************************************************************
* Function Name: Cy_LVDS_DisableLoopbackMode
****************************************************************************//**
*
* Disable link loopback mode.
*
* \param base
* Pointer to the LVDS register base address
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_DisableLoopbackMode(LVDSSS_LVDS_Type * base)
{
    base->LOOPBACK_CFG = 0x00000000;
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhyGpioSet
****************************************************************************//**
*
* Drive LVDS pin high if configured as an output in GPIO mode.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor interface port number
*
* \param pinNo
* GPIO pin ID.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhyGpioSet(LVDSSS_LVDS_Type* base,
                                        uint8_t portNo,
                                        cy_en_lvds_phy_gpio_index_t pinNo)
{
    base->AFE[portNo].PHY_GPIO[pinNo] |= LVDSSS_LVDS_AFE_PHY_GPIO_OUT_VALUE_Msk;
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhyGpioClr
****************************************************************************//**
*
* Drive LVDS pin low if configured as an output in GPIO mode.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor interface port number
*
* \param pinNo
* GPIO pin ID.
*
*******************************************************************************/
__STATIC_INLINE void Cy_LVDS_PhyGpioClr(LVDSSS_LVDS_Type* base,
                                        uint8_t portNo,
                                        cy_en_lvds_phy_gpio_index_t pinNo)
{
    base->AFE[portNo].PHY_GPIO[pinNo] &= ~LVDSSS_LVDS_AFE_PHY_GPIO_OUT_VALUE_Msk;
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhyGpioRead
****************************************************************************//**
*
* Returns the value of pin if configured as an input in GPIO mode.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Sensor interface port number
*
* \param pinNo
* GPIO pin ID.
*
* \return
* Value sampled from the LVCMOS IO pin.
*
*******************************************************************************/
__STATIC_INLINE bool Cy_LVDS_PhyGpioRead(LVDSSS_LVDS_Type* base,
                                        uint8_t portNo,
                                        cy_en_lvds_phy_gpio_index_t pinNo)
{
    return ((base->AFE[portNo].PHY_GPIO[pinNo] & LVDSSS_LVDS_AFE_PHY_GPIO_IN_VALUE_Msk) >> (LVDSSS_LVDS_AFE_PHY_GPIO_IN_VALUE_Pos));
}

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS */

#endif /* (CY_LVDS_H) */

/** \} group_usbfxstack_lvds_lvcmos_functions */

/**
 * \} group_usbfxstack_lvds_lvcmos
*/

/* [] END OF FILE */
