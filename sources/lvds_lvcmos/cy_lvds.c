/***************************************************************************//**
* \file cy_lvds.c
* \version 1.00
*
* Source code file for the LVDS driver.
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

#include "cy_device.h"

#if defined (CY_IP_MXS40LVDS2USB32SS)

#include "cy_lvds.h"
#include "cy_debug.h"
#include "cy_pdl.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define LVDS_MAX_INTERFACE_FREQ_RX_KHZ          (625000)  /* 625 MHz  */
#define LVDS_MIN_INTERFACE_FREQ_RX_KHZ          (74000)   /* 74  MHz  */

#define LVCMOS_DDR_SLAVE_MAX_INTERFACE_FREQ_RX_KHZ    (160000)   /* 160 MHz  */
#define LVCMOS_DDR_SLAVE_MIN_INTERFACE_FREQ_RX_KHZ    (40000)    /* 40 MHz  */

#define LVCMOS_SDR_SLAVE_MAX_INTERFACE_FREQ_RX_KHZ    (160000)   /* 160 MHz  */
#define LVCMOS_SDR_SLAVE_MAX_INTERFACE_FREQ_TX_KHZ    (100000)   /* 100 MHz  */

#define LVCMOS_DDR_MASTER_MAX_INTERFACE_FREQ_TX_KHZ   (80000)   /* 80 MHz */

#define LVCMOS_SDR_MASTER_MAX_INTERFACE_FREQ_RX_KHZ    (100000)   /* 100 MHz  */
#define LVCMOS_SDR_MASTER_MAX_INTERFACE_FREQ_TX_KHZ    (100000)   /* 100 MHz  */

#if (!LVCMOS_16BIT_SDR)

/**
 * @brief Lookup Table for pll_n_in_div, pll_n_serial_div, pll_n_fb_div and pll_n_frame_div
*/

const uint8_t cy_arr_lvds_pll_n_in_div[9] =
{
    0x0,  /* 0 */
    0x0,  /* 1 */
    0x1,  /* 2 */
    0x1,  /* 3 */
    0x2,  /* 4 */
    0x2,  /* 5 */
    0x2,  /* 6 */
    0x2,  /* 7 */
    0x3   /* 8 */
};

const uint8_t cy_arr_lvds_pll_n_serial_div[9] =
{
    0x0,  /* 0 */
    0x0,  /* 1 */
    0x1,  /* 2 */
    0x1,  /* 3 */
    0x2,  /* 4 */
    0x2,  /* 5 */
    0x2,  /* 6 */
    0x2,  /* 7 */
    0x3   /* 8 */
};

const uint8_t cy_arr_lvds_pll_n_frame_div[9] =
{
    0x0,  /* 0 */
    0x0,  /* 1 */
    0x0,  /* 2 */
    0x0,  /* 3 */
    0x0,  /* 4 */
    0x0,  /* 5 */
    0x0,  /* 6 */
    0x0,  /* 7 */
    0x1   /* 8 */
};

const uint8_t cy_arr_lvds_pll_n_fb_div[33] =
{
    0x0,  /* 0 */
    0x1,  /* 1 */
    0x2,  /* 2 */
    0x3,  /* 3 */
    0x4,  /* 4 */
    0x5,  /* 5 */
    0x6,  /* 6 */
    0x7,  /* 7 */
    0x8,  /* 8 */
    0x9,  /* 9 */
    0xA,  /* 10 */
    0xB,  /* 11 */
    0xC,  /* 12 */
    0xC,  /* 13 */
    0xD,  /* 14 */
    0xD,  /* 15 */
    0xE,  /* 16 */
    0xE,  /* 17 */
    0xE,  /* 18 */
    0xE,  /* 19 */
    0xE,  /* 20 */
    0xE,  /* 21 */
    0xE,  /* 22 */
    0xE,  /* 23 */
    0xE,  /* 24 */
    0xE,  /* 25 */
    0xE,  /* 26 */
    0xE,  /* 27 */
    0xE,  /* 28 */
    0xE,  /* 29 */
    0xE,  /* 30 */
    0xE,  /* 31 */
    0xF,  /* 32 */
};

#endif /* (!LVCMOS_16BIT_SDR) */

static void HandlePhyInterrupts        (uint32_t phyIntr, cy_stc_lvds_context_t *lvdsContext);
static void HandleGpifInterrupts       (uint32_t gpifIntr, cy_stc_lvds_context_t *lvdsContext);
static void HandleThreadError          (uint32_t threadError, uint32_t lvdsError, cy_stc_lvds_context_t *lvdsContext);
static void ReportGpifError            (uint32_t smNo, cy_stc_lvds_context_t *lvdsContext);

#if (!LVCMOS_16BIT_SDR)
/* Master DLL clock phase selected for LVCMOS DDR operation. */
static uint8_t glLvcmosClkPhaseSel[2] = {
    CY_LVDS_DDR_MDLL_PHASE_VAL,
    CY_LVDS_DDR_MDLL_PHASE_VAL
};
#endif /* (!LVCMOS_16BIT_SDR) */

/*******************************************************************************
* Function Name: Cy_LVDS_Init
****************************************************************************//**
*
* Initialises LVDS IP block. Only discrete clock frequencies specified through
* \ref cy_en_lvds_phy_interface_clock_t are supported. Use the
* \ref Cy_LVDS_Init_Ext() function to get support for a wider range of
* frequencies.
*
* \param base
* Pointer to the LVDS register base address
*
* \param sipNo
* Sensor Interface Port number.
*
* \param cy_stc_lvds_config_t
* Pointer to a structure array specifies all the parameters required to configure
* the LVDS IP block
*
* \param cy_stc_lvds_context_t
* LVDS context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_Init(LVDSSS_LVDS_Type * base, uint8_t sipNo,
                            const cy_stc_lvds_config_t * lvdsConfig, cy_stc_lvds_context_t *lvdsContext)
{
    cy_en_lvds_status_t status = CY_LVDS_BAD_PARAMETER;

    if ((base == NULL) || (lvdsConfig == NULL) || (sipNo >= CY_LVDS_MAX_PHY_INSTANCE) ||
            (lvdsContext == NULL) || (lvdsConfig->phyConfig == NULL) || (lvdsConfig->gpifConfig == NULL))
    {
        return status;
    }

    /* Check if LVDS AFE is already enabled */
    if(base->CTL & LVDSSS_LVDS_CTL_LINK_ENABLED_Msk)
    {
        DBG_LVDS_ERR(" AFE already enabled\n\r");
        return CY_LVDS_LINK_ALREADY_ENABLED;
    }

    switch (lvdsConfig->phyConfig->interfaceClock)
    {
#if (!LVCMOS_16BIT_SDR)
        case CY_LVDS_PHY_INTERFACE_CLK_625_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 625000UL;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_156_25_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 156250UL;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_148_5_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 148500UL;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_74_25_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 74250UL;
            break;
#endif /* (!LVCMOS_16BIT_SDR) */

        case CY_LVDS_PHY_INTERFACE_CLK_160_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 160000UL;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_100_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 100000UL;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_80_MHZ:
            lvdsContext->interfaceClock_kHz[sipNo] = 80000UL;
            break;

        default:
            DBG_LVDS_ERR("Unsupported InterfaceClock setting\r\n");
            return status;
    }

    status = Cy_LVDS_PhyInit(LVDSSS_LVDS, sipNo, lvdsConfig->phyConfig, lvdsContext);
    if(status != CY_LVDS_SUCCESS)
    {
        DBG_LVDS_ERR("LVDS_PhyInit failed for Port: %d\r\n", sipNo);
        return status;
    }

    DBG_LVDS_INFO("LVDS_PhyInit done for Port: %d\r\n", sipNo);

#if (!LVCMOS_16BIT_SDR)
    /* In case of a wide link port, both PHY needs to be initialized */
    if(lvdsConfig->phyConfig->wideLink == true)
    {
        lvdsContext->interfaceClock_kHz[1] = lvdsContext->interfaceClock_kHz[0];
        status = Cy_LVDS_PhyInit(LVDSSS_LVDS, 1, lvdsConfig->phyConfig, lvdsContext);
        if(status != CY_LVDS_SUCCESS)
        {
            DBG_LVDS_ERR("LVDS_PhyInit failed for Port: 1\r\n");
            return status;
        }

        DBG_LVDS_INFO("LVDS_PhyInit done for Port: 1 \r\n");
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    status = Cy_LVDS_GpifInit(LVDSSS_LVDS, sipNo, lvdsConfig->gpifConfig, lvdsContext);
    if(status != CY_LVDS_SUCCESS)
    {
        return status;
    }
    DBG_LVDS_INFO("LVDS_GpifInit done for Port: %d\r\n", sipNo);
    return status;
}

/*******************************************************************************
* Function Name: Cy_LVDS_Deinit
****************************************************************************//**
*
* De-initializes the Sensor Interface Port IP block.
*
* \param base
* Pointer to the LVDS register base address
*
* \param sipNo
* Sensor Interface Port number.
*
* \param cy_stc_lvds_context_t
* Pointer to driver context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_Deinit (LVDSSS_LVDS_Type *base, uint8_t sipNo, cy_stc_lvds_context_t *lvdsContext)
{
    cy_en_lvds_status_t status = CY_LVDS_BAD_PARAMETER;
    uint32_t intMask;
#if (!LVCMOS_16BIT_SDR)
    bool isWideLink = false;
#endif /* !LVCMOS_16BIT_SDR*/

    if ((base == NULL) || (sipNo >= CY_LVDS_MAX_PHY_INSTANCE) || (lvdsContext == NULL))
    {
        return status;
    }
#if (!LVCMOS_16BIT_SDR)
    /* Save WideLink setting. */
    isWideLink = lvdsContext->phyConfigP0->wideLink;
#endif /* !LVCMOS_16BIT_SDR*/

    /* Disable GPIF state machine first. */
    intMask = Cy_SysLib_EnterCriticalSection();
    base->GPIF[sipNo].GPIF_WAVEFORM_CTRL_STAT |= LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_PAUSE_Msk;
    Cy_SysLib_DelayUs(10);
    base->GPIF[sipNo].GPIF_WAVEFORM_CTRL_STAT = 0;
    base->GPIF[sipNo].GPIF_INTR = 0xFFFFFFFFUL;
    base->GPIF[sipNo].GPIF_INTR_MASK = 0;

    /* Disable the AFE. */
    status = Cy_LVDS_PhyDeinit(LVDSSS_LVDS, sipNo, lvdsContext);

    if (status == CY_LVDS_SUCCESS) {
        /* Disable threads and clear interrupts associated with the port. */
#if (!LVCMOS_16BIT_SDR)
        if (isWideLink) {
            base->THREAD[0].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
            base->THREAD[1].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
            base->THREAD[2].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
            base->THREAD[3].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
            base->LVDS_INTR_WD0 = 0xFFFFFFFFUL;
            base->LVDS_INTR_WD1 = 0xFFFFFFFFUL;
        }
        else
#endif /* (!LVCMOS_16BIT_SDR) */
        {
            if (sipNo == 0) {
                base->THREAD[0].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
                base->THREAD[1].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
                base->LVDS_INTR_WD0 = 0x54035535UL;
                base->LVDS_INTR_WD1 = 0x00FF00FFUL;
#if (!LVCMOS_16BIT_SDR)
            } else {
                base->THREAD[2].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
                base->THREAD[3].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
                base->LVDS_INTR_WD0 = 0xA80CAACAUL;
                base->LVDS_INTR_WD1 = 0xFF00FF00UL;
#endif /* (!LVCMOS_16BIT_SDR) */
            }
        }
    }

    Cy_SysLib_ExitCriticalSection(intMask);

    return status;
}

/*******************************************************************************
* Function Name: Cy_LVDS_Init_Ext
****************************************************************************//**
*
* Initialises LVDS IP block. More flexible version of Cy_LVDS_Init() function
* which allows selection of any interface clock frequency within the valid
* range.
*
* \param base
* Pointer to the LVDS register base address
*
* \param sipNo
* Sensor Interface Port number.
*
* \param cy_stc_lvds_config_t
* Pointer to a structure array specifies all the parameters required to configure
* the LVDS IP block
*
* \param cy_stc_lvds_context_t
* LVDS context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_Init_Ext (LVDSSS_LVDS_Type * base, uint8_t sipNo,
                            const cy_stc_lvds_config_t * lvdsConfig, cy_stc_lvds_context_t *lvdsContext)
{
    cy_en_lvds_status_t status = CY_LVDS_BAD_PARAMETER;
    if((base == NULL) || (lvdsConfig == NULL) || (sipNo >= CY_LVDS_MAX_PHY_INSTANCE) ||
        (lvdsContext == NULL) || (lvdsConfig->phyConfig == NULL) || (lvdsConfig->gpifConfig == NULL))
    {
        return status;
    }

    /* Check if LVDS AFE is already enabled */
    if(base->CTL & LVDSSS_LVDS_CTL_LINK_ENABLED_Msk)
    {
        DBG_LVDS_ERR(" AFE already enabled\n\r");
        return CY_LVDS_LINK_ALREADY_ENABLED;
    }

    /* Store the frequency in the context structure for subsequent use. */
    lvdsContext->interfaceClock_kHz[sipNo] = lvdsConfig->phyConfig->interfaceClock_kHz;

    status = Cy_LVDS_PhyInit(LVDSSS_LVDS, sipNo, lvdsConfig->phyConfig, lvdsContext);
    if(status != CY_LVDS_SUCCESS)
    {
        DBG_LVDS_ERR("LVDS_PhyInit failed for Port: %d\r\n", sipNo);
        return status;
    }

    DBG_LVDS_INFO("LVDS_PhyInit done for Port: %d\r\n", sipNo);

#if (!LVCMOS_16BIT_SDR)
    /* In case of a wide link port, both PHY needs to be initialized */
    if(lvdsConfig->phyConfig->wideLink == true)
    {
        lvdsContext->interfaceClock_kHz[1] = lvdsContext->interfaceClock_kHz[0];
        status = Cy_LVDS_PhyInit(LVDSSS_LVDS, 1, lvdsConfig->phyConfig, lvdsContext);
        if(status != CY_LVDS_SUCCESS)
        {
            DBG_LVDS_ERR("LVDS_PhyInit failed for Port: 1\r\n");
            return status;
        }

        DBG_LVDS_INFO("LVDS_PhyInit done for Port: 1 \r\n");
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    status = Cy_LVDS_GpifInit(LVDSSS_LVDS, sipNo, lvdsConfig->gpifConfig, lvdsContext);
    if(status != CY_LVDS_SUCCESS)
    {
        return status;
    }
    DBG_LVDS_INFO("LVDS_GpifInit done for Port: %d\r\n", sipNo);
    return status;
}

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
* \param mdBuffer
* Pointer to a structure array (cy_stc_lvds_md_config_t) specifiying the metadata
* elements which needs to be inserted.
*
* \param mdIndex
* There are 4 Metadata per thread available to be initialized.
*
* \param mdCount
* Total number of metadata which needs to be inserted
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_InitMetadata(LVDSSS_LVDS_Type *base, uint8_t threadNo,
                            uint8_t mdIndex, uint8_t mdCount, cy_stc_lvds_md_config_t *mdBuffer)
{
    uint32_t i, value;

    if((base == NULL) || (threadNo >= CY_LVDS_MAX_THREAD_COUNT) ||
        (mdIndex >= CY_LVDS_MAX_MD_INDEX_COUNT) || (mdCount > CY_LVDS_MAX_MD_COUNT))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Clear the MDx_CTRL register before initializing it */
    switch(mdIndex)
    {
        case 0:
            base->THREAD[threadNo].MD0_CTRL = 0x00000000;
            break;
        case 1:
            base->THREAD[threadNo].MD1_CTRL = 0x00000000;
            break;
#if (!LVCMOS_16BIT_SDR)
        case 2:
            base->THREAD[threadNo].MD2_CTRL = 0x00000000;
            break;
        case 3:
            base->THREAD[threadNo].MD3_CTRL = 0x00000000;
            break;
#endif /* (!LVCMOS_16BIT_SDR) */
    }

    for(i = 0; i < mdCount; i++)
    {
        if(mdBuffer[i].metadataType == CY_LVDS_MD_VARIABLE)
        {
            switch(mdIndex)
            {
                case 0:
                    base->THREAD[threadNo].MD0_CTRL |= (1 << i);
                    break;
                case 1:
                    base->THREAD[threadNo].MD1_CTRL |= (1 << i);
                    break;
#if (!LVCMOS_16BIT_SDR)
                case 2:
                    base->THREAD[threadNo].MD2_CTRL |= (1 << i);
                    break;
                case 3:
                    base->THREAD[threadNo].MD3_CTRL |= (1 << i);
                    break;
#endif /* (!LVCMOS_16BIT_SDR) */
            }
            value = mdBuffer[i].metadataValue + threadNo*LVDSSS_LVDS_THREAD_SECTION_SIZE;
        }
        else
        {
            value = mdBuffer[i].metadataValue;
        }

        if((threadNo == 0) || (threadNo == 1))
        {
            if(i%2 == 0)
            {
                base->TH0_TH1_METADATA_RAM[8*mdIndex + 32*threadNo + (i/2)] = value;
            }
            else
            {
                base->TH0_TH1_METADATA_RAM[8*mdIndex + 32*threadNo + ((i-1)/2)] |= value << 16;
            }
        }
#if (!LVCMOS_16BIT_SDR)
        else
        {
            if(i%2 == 0)
                base->TH2_TH3_METADATA_RAM[8*mdIndex + 32*(threadNo - 2) + (i/2)] = value;
            else
                base->TH2_TH3_METADATA_RAM[8*mdIndex + 32*(threadNo - 2) + ((i - 1)/2)] |= value << 16;
        }
#endif /* (!LVCMOS_16BIT_SDR) */
    }

    if(mdCount != 16)
    {
        switch(mdIndex)
        {
            case 0:
                base->THREAD[threadNo].MD0_CTRL |= _VAL2FLD(LVDSSS_LVDS_THREAD_MD0_CTRL_MD_SIZE, mdCount<<1);
                break;
            case 1:
                base->THREAD[threadNo].MD1_CTRL |= _VAL2FLD(LVDSSS_LVDS_THREAD_MD1_CTRL_MD_SIZE, mdCount<<1);
                break;
#if (!LVCMOS_16BIT_SDR)
            case 2:
                base->THREAD[threadNo].MD2_CTRL |= _VAL2FLD(LVDSSS_LVDS_THREAD_MD2_CTRL_MD_SIZE, mdCount<<1);
                break;
            case 3:
                base->THREAD[threadNo].MD3_CTRL |= _VAL2FLD(LVDSSS_LVDS_THREAD_MD3_CTRL_MD_SIZE, mdCount<<1);
                break;
#endif /* (!LVCMOS_16BIT_SDR) */
        }
    }

    return CY_LVDS_SUCCESS;
}

#if (!LVCMOS_16BIT_SDR)
/* Constant array that maps LVDS lane count settings from B0 silicon to that of A0 silicon. */
const uint8_t FX_A0_LVDS_LANE_CNT_MAP[] = {
    0,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_1 ==> 1 lane */
    1,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_2 ==> 2 lanes */
    2,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_3 ==> 4 lanes */
    2,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_4 ==> 4 lanes */
    3,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_5 ==> 8 lanes */
    3,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_6 ==> 8 lanes */
    3,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_7 ==> 8 lanes */
    3,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_8 ==> 8 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_9 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_10 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_11 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_12 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_13 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_14 ==> 16 lanes */
    4,          /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_15 ==> 16 lanes */
    4           /* CY_LVDS_PHY_LVDS_MODE_NUM_LANE_16 ==> 16 lanes */
};

/*******************************************************************************
* Function Name: Cy_LVDS_Phy_Lvds_Init
****************************************************************************//**
*
* Initialises LVDS Sensor interface port for LVDS Mode
*
* \param base
* Pointer to the LVDS register base address
*
* \param lvdsContext
* Pointer to driver context structure.
*
* \param config
* Specifies port configuration such as bus direction, bus width etc.
* The value will provided by device configurator.
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_Phy_Lvds_Init (LVDSSS_LVDS_Type * base,
        cy_stc_lvds_context_t *lvdsContext, const cy_stc_lvds_phy_config_t *config, uint8_t portNo)
{

    uint8_t i = 0;
    uint8_t phy_deskew_algo = 0;
    uint32_t timeoutCount = 0;
    uint32_t lane_data_bit_rate = 0;
    uint32_t interface_freq = 0; /* In kHz */
    uint8_t  gear_ratio = 0;
    uint8_t  pll_bypass = 0;
    uint32_t pll_serial_clk =0;     /* pll_serial_clk can range from 74.5MHz to 625MHz. */
                                    /* pll_serial_clk = FIN * (pll_n_fb_div/(pll_n_in_div * pll_n_serial_div)) for pll_fb_sel[1:0] = 00 */
    uint32_t pll_frame_clk =0;      /* pll_frame_clk = pll_serial_clk /pll_n_frame_div */
    uint8_t  pll_n_frame_div = 0 ;  /* Output Frame Clock Divider */
    uint8_t  pll_n_serial_div = 0;  /* Output Serial Clock Divider */
    uint8_t  pll_n_in_div = 0;      /* Input Clock Divider */
    uint8_t  pll_n_fb_div = 0;      /* Feedback Clock Divider */
    uint32_t vco_clk = 0;           /* VCO_CLK = FIN * (pll_n_fb_div/pll_n_in_div) for pll_fb_sel[1:0] = 00; */
                                    /* vco_clk = pll_serial_clk * pll_n_serial_div ; */
    uint32_t pll_input_freq = 0;
    uint32_t cnt_10us24_val = 0;
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;

    if ((base == NULL) || (config == NULL) || (lvdsContext == NULL) || (portNo >= CY_LVDS_MAX_PHY_INSTANCE))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    interface_freq = lvdsContext->interfaceClock_kHz[portNo];
    if ((interface_freq < LVDS_MIN_INTERFACE_FREQ_RX_KHZ) ||  (interface_freq > LVDS_MAX_INTERFACE_FREQ_RX_KHZ))
    {
        DBG_LVDS_ERR("Wrong LVDS Interface Freq:%d kHz \r\n",interface_freq);
        return CY_LVDS_CONFIG_ERROR;
    }

    if (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_1_1)
        gear_ratio = 1;
    else if (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_2_1)
        gear_ratio = 2;
    else if (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_4_1)
        gear_ratio = 4;
    else
        gear_ratio = 8;

    lane_data_bit_rate = interface_freq * gear_ratio;   /* In kHz */

    if (lane_data_bit_rate > MAX_LVDS_LANE_DATA_BIT_RATE)
    {
        DBG_LVDS_ERR("LVDS Data Bit rate more than 1.25Gbps\r\n");
        return CY_LVDS_CONFIG_ERROR;
    }

    /* Pad for V1P1 has to be enabled */
    base->AFE[portNo].LICIO_VCCD_V1P1       |=  _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN, 1u);

    /* Set LVDS_AFE0_PHY_ADC_CONFIG.ISO_N = 1 , Disable output isolation control (Active low) */
    base->AFE[portNo].PHY_ADC_CONFIG        |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N, 1u);



    if (lane_data_bit_rate >= 625000 && lane_data_bit_rate <= MAX_LVDS_LANE_DATA_BIT_RATE)
    {
        phy_deskew_algo = CY_LVDS_PHY_DESKEW_DDR_FAST;
        /* PLL_SERIAL_CLK = FIN * (pll_n_fb_div/(pll_n_in_div * pll_n_serial_div)) for pll_fb_sel[1:0] = 00 */
        pll_serial_clk = lane_data_bit_rate/2;
        pll_n_frame_div = 4;
        if (gear_ratio == 2)
            pll_bypass = 1;
        DBG_LVDS_INFO("LVDS_PHY_DESKEW:DDR_FAST pll_bypass:%d \r\n",pll_bypass);
    }
    else if (lane_data_bit_rate > 200000)
    {
        phy_deskew_algo = CY_LVDS_PHY_DESKEW_SDR_FAST;
        pll_serial_clk = lane_data_bit_rate;
        pll_n_frame_div = 8;
        if (gear_ratio == 1)
            pll_bypass = 1;
        DBG_LVDS_INFO("LVDS_PHY_DESKEW:SDR_FAST pll_bypass:%d \r\n",pll_bypass);
    }
    else
    {
        /* BYPASS Mode */
        phy_deskew_algo = CY_LVDS_PHY_DESKEW_BYPASS;
        base->AFE[portNo].DLL_M_CONFIG      |=   _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_SPEED_MODE, 1u);
        pll_serial_clk = lane_data_bit_rate;
        pll_n_frame_div = 8;
        if (gear_ratio == 1)
            pll_bypass = 1;
        DBG_LVDS_INFO("LVDS_PHY_DESKEW:BYPASS pll_bypass:%d \r\n",pll_bypass);
    }

    if (pll_serial_clk < 295000)  /* In kHz */
    {
        if (pll_serial_clk < 147000)
            pll_n_serial_div = 8;
        else
            pll_n_serial_div = 4;
    }
    else {
        pll_n_serial_div = 2;
    }

    pll_frame_clk = pll_serial_clk/pll_n_frame_div; /* In kHz */

    /* CNT_10US24_VAL = (10.24 us * PLL_FRAME_CLK MHz) + 100 */
    cnt_10us24_val = (10 * (pll_frame_clk/1000)) + 100;

    /* VCO_CLK = FIN * (pll_n_fb_div/pll_n_in_div) for pll_fb_sel[1:0] = 00 */
    vco_clk = pll_serial_clk * pll_n_serial_div;

    if(interface_freq >= 640000){
        pll_n_in_div = 8;
    }
    else if (interface_freq >= 320000)
    {
        pll_n_in_div = 4;
    }
    else if (interface_freq >= 160000)
    {
        pll_n_in_div = 2;
    }
    else
        pll_n_in_div = 1;

    pll_input_freq = interface_freq/pll_n_in_div;

    pll_n_fb_div = vco_clk/pll_input_freq;

    /* Encode pll_n_in_div value for PLL_CONFIG.N_IN_DIV */
    pll_n_in_div = cy_arr_lvds_pll_n_in_div[pll_n_in_div];

    /* Encode pll_n_serial_div for PLL_CONFIG.N_SERIAL_DIV */
    pll_n_serial_div = cy_arr_lvds_pll_n_serial_div[pll_n_serial_div];

    /* Encode pll_n_frame_div value for PLL_CONFIG.N_FRAME_DIV */
    pll_n_frame_div = cy_arr_lvds_pll_n_frame_div[pll_n_frame_div];

    /* Encode pll_n_fb_div for PLL_CONFIG.N_FB_DIV */
    pll_n_fb_div = cy_arr_lvds_pll_n_fb_div[pll_n_fb_div];

    /* Trace Log */
    DBG_LVDS_TRACE("pll_n_fb_div:%d pll_n_frame_div:%d pll_n_serial_div:%d pll_n_in_div:%d \r\n", \
                        pll_n_fb_div,pll_n_frame_div,pll_n_serial_div,pll_n_in_div);
    DBG_LVDS_TRACE("pll_serial_clk:%d kHz, vco_clk:%d kHz, pll_input_freq:%d kHz, pll_frame_clk:%d kHz\r\n", \
                        pll_serial_clk,vco_clk,pll_input_freq,pll_frame_clk);
    DBG_LVDS_TRACE("cnt_10us24_val:%d \r\n",cnt_10us24_val);

    DBG_LVDS_INFO("interface_freq:%d kHz ,lane_data_bit_rate:%d kHz, gear_ratio:%d \r\n",interface_freq,lane_data_bit_rate,gear_ratio);

    base->AFE[portNo].PHY_GENERAL_CONFIG_2  |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_2_CNT_10US24_VAL, cnt_10us24_val);
    base->AFE[portNo].PHY_GENERAL_CONFIG    |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM, phy_deskew_algo);

    /* For PHY_DESKEW  SDR_FAST/DDR_FAST  */
    if ((phy_deskew_algo == CY_LVDS_PHY_DESKEW_DDR_FAST) || (phy_deskew_algo == CY_LVDS_PHY_DESKEW_SDR_FAST))
    {
        DBG_LVDS_INFO("PHY_DESKEW is SDR or DDR FAST Mode \r\n");
        /* DESKEW_ERR_THRESH2 value set set to 10 when the monitor is off and to 6 when the monitor is on */
        base->AFE[portNo].PHY_GENERAL_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1_Msk;
        base->AFE[portNo].PHY_GENERAL_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2_Msk;
        base->AFE[portNo].PHY_GENERAL_CONFIG    |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1, 0) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2, 5);

        /* Set value for PHY_GENERAL_CONFIG_2.MONITOR_ERR_THRESH */
        base->AFE[portNo].PHY_GENERAL_CONFIG_2  |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_2_MONITOR_ERR_THRESH, 10);

        /* Set value for PHY_GENERAL_CONFIG.MONITOR_TIME */
        base->AFE[portNo].PHY_GENERAL_CONFIG    |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_MONITOR_TIME, 2);
    }

    Cy_SysLib_DelayUs(10);

    /* VDDIO and V1P1 have to enabled */
    base->AFE[portNo].PHY_GENERAL_CONFIG    |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P1_VCCD, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_SCANON, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P25_VCCD, 1u);
    Cy_SysLib_DelayUs(10);

    /* 10uA current for PLL biasing needs to be generated. 10uA have to present at i_ibias_10u_lnk0_ai */
    /* Reference current generator has to be switched ON */
    base->AFE[portNo].LICIO_VSSIO_IREF      |=  _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN, 1u);
    Cy_SysLib_DelayUs(10);

    /* Switch on REG_1P25 Regulator for dll_m and use it (Not set it in bypass mode) */
    /* Switch on all needed LVDS RX with proper settings:
        1. In Bypass mode, invert the polarity of the serial clock that enters RX
           RX.INV_CLK_SER = 1
        2. In Bypass mode,
           RX.CLK_MUX_SEL: a. For Frame Clock channel, 1 for LVDS Slow and LVDS Bypass.
                           b. Data Channel 3, 1 for LVDS Slow, 2 for LVDS Bypass.
    */

    /* For PHY_DESKEW SDR_FAST/DDR_FAST  */
    if ((phy_deskew_algo == CY_LVDS_PHY_DESKEW_DDR_FAST) || (phy_deskew_algo == CY_LVDS_PHY_DESKEW_SDR_FAST))
    {
        DBG_LVDS_INFO("PHY_DESKEW is SDR or DDR FAST Mode \r\n");
        /* Switch on REG_1P25 Regulator */
        base->AFE[portNo].REG_1P25      |=   _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_ENABLE, 1u) |
                                             _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_USE_REG, 1u);
        /* Switch on all needed LVDS RX */
        for(i = 0; i < 10; i++)
        {
            base->AFE[portNo].RX[i]    |=   _BOOL2FLD(LVDSSS_LVDS_AFE_RX_EN_DESER, 1u)|
                                            _BOOL2FLD(LVDSSS_LVDS_AFE_RX_FIXTIME_FRAMECLK, 1u);
        }
    }
    else if (phy_deskew_algo == CY_LVDS_PHY_DESKEW_BYPASS) /* Deskew Algorithm Bypass */
    {

        DBG_LVDS_INFO("PHY_DESKEW is Bypass Mode \r\n");
        /* Switch on all needed LVDS RX */
        for(i = 0; i < 10; i++)
        {
            if(i != 3)
            {
                base->AFE[portNo].RX[i]     |=  _VAL2FLD(LVDSSS_LVDS_AFE_RX_CLK_MUX_SEL, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_RX_INV_CLK_SER, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_RX_EN_DESER, 1u)|
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_RX_FIXTIME_FRAMECLK, 1u);
            }
        }
        base->AFE[portNo].RX[3]         |=  _VAL2FLD(LVDSSS_LVDS_AFE_RX_CLK_MUX_SEL, 2u) |
                                            _BOOL2FLD(LVDSSS_LVDS_AFE_RX_INV_CLK_SER, 1u) |
                                            _BOOL2FLD(LVDSSS_LVDS_AFE_RX_EN_DESER, 1u)|
                                            _BOOL2FLD(LVDSSS_LVDS_AFE_RX_FIXTIME_FRAMECLK, 1u);
    }
    else /* Deskew Algorithm Slow */
    {
        DBG_LVDS_ERR("PHY_DESKEW is Slow not supported \r\n");
        return CY_LVDS_CONFIG_ERROR;
    }

    /* Set LICIO_CIO_CTRL_DS.DS = 1 */
    base->AFE[portNo].LICIO_CIO_CTRL_DS     |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_CTRL_DS_DS, 0x1Fu);


    /* Set LICIO_CIO_DATA_DS[x].DS = 1 (0 <= x <= 16) */
    for(i = 0; i < 17; i++)
    {
        base->AFE[portNo].LICIO_CIO_DATA_DS[i] |=  _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_DATA_DS_DS, 0x1Fu);
    }


    /* Inorder to save clock tree power of this path, set this bit field to 1 in case of LVDS mode */
    base->AFE[portNo].GENERAL_RX_LVCMOS     |=   _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_MUX_SEL, 1u);

    /* Enable LVCMOS sideband control signals: LVCMOS sideband control signals can be both TX or RX */
    /* Depending on control bus bit map, pins which are input needs to be configured - For widelink there can be 14 pins */

    if(config->wideLink == true)
    {
        DBG_LVDS_INFO("LVDS WL Mode\r\n");
        for(i = 0; i < 7; i++)
        {
            if(((config->ctrlBusBitMap >> (i+(7*portNo)))) & 0x0001)
                    base->AFE[portNo].RX_LVCMOS[i+16] |= _BOOL2FLD(LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS, 1u);
        }
    }
    else
    {
        DBG_LVDS_INFO("LVDS NL Mode\r\n");
        for(i = 0; i < 7; i++)
        {
            if((config->ctrlBusBitMap >> i) & 0x0001)
                base->AFE[portNo].RX_LVCMOS[i+16] |= _BOOL2FLD(LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS, 1u);
        }
    }

    /* Turn ON LVDS receivers */
    base->AFE[portNo].GENERAL_LICIO_LI      |=   _VAL2FLD(LVDSSS_LVDS_AFE_GENERAL_LICIO_LI_LVDS_CONFIG, 3u);

    /* Switch on licio_li for Data Lane[7:0] , control Lane (8) and Frame clock (9) */
    for(i = 0; i < 10; i++)
    {
        base->AFE[portNo].LICIO_LI[i]       |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_LI_LVDS_RX_EN, 1u);

    }

    /* Set the correct mode for DR_MODE as DDR = 0 or SDR = 1  and also set DLL_BYPASS_EN in case of Bypass Mode */
    /* In case of Bypass Mode DR_MODE =1 (as SDR) */
    /* Find the Bypass Mode */
    if (phy_deskew_algo == CY_LVDS_PHY_DESKEW_BYPASS)
    {
        DBG_LVDS_INFO("PHY_DESKEW is Bypass Mode \r\n");
        /* Take LVDS pads as data source */
        base->AFE[portNo].GENERAL_RX            |=   _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_DLL_BYPASS_EN, 1u);
        base->AFE[portNo].GENERAL_RX            |=   _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_DR_MODE, 1u);
    }
    else if (phy_deskew_algo == CY_LVDS_PHY_DESKEW_SDR_FAST)
    {
        DBG_LVDS_INFO("PHY_DESKEW is SDR FAST Mode \r\n");
        base->AFE[portNo].GENERAL_RX            |=   _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_DR_MODE, 1u);
    }
    else if (phy_deskew_algo == CY_LVDS_PHY_DESKEW_DDR_FAST)
    {
        DBG_LVDS_INFO("PHY_DESKEW is DDR FAST Mode \r\n");
        base->AFE[portNo].GENERAL_RX            |=   _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_DR_MODE, 0u);
    }
    else /* Deskew Algorithm Slow */
    {
        DBG_LVDS_ERR("PHY_DESKEW is Slow not supported \r\n");
        return CY_LVDS_CONFIG_ERROR;
    }
    Cy_SysLib_DelayUs(10);

    /* Enable Differential resistance for all LVDS lanes */
    base->AFE[portNo].GENERAL_LICIO_CIO = 0x000003FF;

    /* Wait 10us */
    Cy_SysLib_DelayUs(10);

    /* Set PLL values based on port configuration then wait 1us */
    /* Switch on the PLL only when the frequency of the input clock is different from the
       wanted frequency of the serial clock at the PLL output */

    base->AFE[portNo].PLL_CONFIG            |=  _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_N_SERIAL_DIV, pll_n_serial_div) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_N_IN_DIV, pll_n_in_div) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_N_FB_DIV, pll_n_fb_div) |
                                                    _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_N_FRAME_DIV, pll_n_frame_div);

    base->AFE[portNo].PLL_CONFIG_2          |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_MD, 0x0u) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_FB_SEL, 0x0u) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_RA_UP_TR, 0x2u) |
                                                    _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_VCO_GAIN, 7u);


    /* Switch on PLL and wait for PLL Lock */
    /* Take care of PLL BYPASS Case */
    if (pll_bypass == 1)
    {
        DBG_LVDS_INFO("PLL BYPASS\r\n");
        Cy_SysLib_DelayUs(1);
        base->AFE[portNo].PLL_CONFIG            |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_BYPASS, 0x1u);
        Cy_SysLib_DelayUs(10);
    }
    else  /* Acquire  PLL LOCK */
    {
        /* Clear PLL_LOCK_ACQUIRED bit in PHY_INTR */
        base->AFE[portNo].PHY_INTR &= ~LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_ACQUIRED_Msk;

        base->AFE[portNo].PLL_CONFIG            |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN, 0x1u);
        Cy_SysLib_DelayUs(10);

        base->AFE[portNo].PLL_CONFIG            |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN, 0x1u);

        DBG_LVDS_INFO("Wait till PLL Lock\r\n");
        timeoutCount = 0;
        while(!(base->AFE[portNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_ACQUIRED_Msk))
        {
            Cy_SysLib_Delay(1);
            timeoutCount++;
            if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
            {
                status = CY_LVDS_TIMEOUT_ERROR;
                DBG_LVDS_ERR("PLL lock timed out for Port: %d\r\n", portNo);
                return status;
            }
        }
        base->AFE[portNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_ACQUIRED_Msk;
        status = CY_LVDS_SUCCESS;
        DBG_LVDS_INFO("PLL lock done for Port: %d\r\n", portNo);
    }

    Cy_SysLib_DelayUs(1);

    /*  For SLOW/FAST deskew algorithm acquire DLL_LOCK  */
    /*  For BYPASS deskew algorithm not required DLL_LOCK */

    /* Find PHY_DESKEW SLOW or FAST DDR/SDR Deskew Algorithm Case */
    /* NOTE: FAST DDR/SDR Deskew Algorithm Case supported */
    if ((phy_deskew_algo == CY_LVDS_PHY_DESKEW_DDR_FAST) || (phy_deskew_algo == CY_LVDS_PHY_DESKEW_SDR_FAST))
    {

        /* Clear DLL_LOCK_ACQUIRED bit in PHY_INTR */
        base->AFE[portNo].PHY_INTR &= ~LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk;

        /* set DLL_M_CONFIG.EN = 1 */
        base->AFE[portNo].DLL_M_CONFIG         |=   _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_EN, 1u);
        Cy_SysLib_DelayUs(10);

        /* set DLL_S_CONFIG.SDLL_EN = 1 */
        for(i = 0; i < 12; i++)
        {
            base->AFE[portNo].DLL_S_CONFIG[i]         |=   _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_S_CONFIG_SDLL_EN, 1u);
        }

        /* Wait until DLL_M_Status.DLL_LOCK goes to 1 */
        DBG_LVDS_INFO("Wait till DLL Lock\r\n");
        timeoutCount = 0;
        while(!(base->AFE[portNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk))
        {
            Cy_SysLib_Delay(1);
            timeoutCount++;
            if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
            {
                status = CY_LVDS_TIMEOUT_ERROR;
                DBG_LVDS_ERR("LVDS DLL lock timedout for Port: %d\r\n", portNo);
                return status;
            }
        }

        base->AFE[portNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk;
        DBG_LVDS_INFO("LVDS DLL Lock done for Port: %d\r\n", portNo);
    }

    base->AFE[portNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_SEQ_Msk;
    base->AFE[portNo].PHY_TRAIN_CONFIG = _VAL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_SEQ, config->phyTrainingPattern) |
                                         _VAL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_CYCLES_NO, 127);

    return status;
}

/*******************************************************************************
* Function Name: Cy_LVDS_Phy_Lvds_Deinit
****************************************************************************//**
*
* De-initialise the Sensor Interface Port if previously configured in LVDS mode.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
*******************************************************************************/
void Cy_LVDS_Phy_Lvds_Deinit(LVDSSS_LVDS_Type *base, uint8_t portNo)
{
    uint8_t i;

    if ((base == NULL) || (portNo >= 2)) {
        DBG_LVDS_ERR("%s: Invalid parameter\r\n", __func__);
        return;
    }

    /* Clear LINK training status and Disable PHY training. */
    base->LINK_TRAINING_STS[portNo] = 0x01FF01FFUL;
    base->AFE[portNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN_Msk;
    base->TRAINING_BLK_CFG &= ~(1UL << portNo);

    /* Disable all Slave DLLs. */
    for (i = 0; i < 12; i++) {
        base->AFE[portNo].DLL_S_CONFIG[i] &= ~LVDSSS_LVDS_AFE_DLL_S_CONFIG_SDLL_EN_Msk;
    }

    /* Disable Master DLL. */
    base->AFE[portNo].DLL_M_CONFIG &= ~(
            LVDSSS_LVDS_AFE_DLL_M_CONFIG_EN_Msk | LVDSSS_LVDS_AFE_DLL_M_CONFIG_SPEED_MODE_Msk);

    /* Disable the PLL. */
    base->AFE[portNo].PLL_CONFIG &= ~(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_BYPASS_Msk);
    base->AFE[portNo].PLL_CONFIG   = 0;
    base->AFE[portNo].PLL_CONFIG_2 = (100 << LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_CTRL_LOCK_DELAY_Pos);

    /* Restore all IO configuration to default values. */
    base->AFE[portNo].GENERAL_LICIO_CIO = 0;
    base->AFE[portNo].GENERAL_RX = 0;
    for (i = 0; i < 10; i++) {
        base->AFE[portNo].LICIO_LI[i] &= ~LVDSSS_LVDS_AFE_LICIO_LI_LVDS_RX_EN_Msk;
    }
    base->AFE[portNo].GENERAL_LICIO_LI = 0;
    for (i = 0; i < 26; i++) {
        base->AFE[portNo].RX_LVCMOS[i] &= ~LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS_Msk;
    }
    base->AFE[portNo].GENERAL_RX_LVCMOS = 0;
    for (i = 0; i < 17; i++) {
        base->AFE[portNo].LICIO_CIO_DATA_DS[i] = 0;
    }
    base->AFE[portNo].LICIO_CIO_CTRL_DS = 0;
    for (i = 0; i < 10; i++) {
        base->AFE[portNo].RX[i] = 0;
    }

    /* Disable regulators. */
    base->AFE[portNo].REG_1P25 &= ~(LVDSSS_LVDS_AFE_REG_1P25_ENABLE_Msk | LVDSSS_LVDS_AFE_REG_1P25_USE_REG_Msk);
    base->AFE[portNo].LICIO_VSSIO_IREF &= ~LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN_Msk;
    base->AFE[portNo].PHY_GENERAL_CONFIG &= ~(
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P1_VCCD_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_SCANON_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P25_VCCD_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_MONITOR_TIME_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM_Msk);
    base->AFE[portNo].PHY_GENERAL_CONFIG = (
            (base->AFE[portNo].PHY_GENERAL_CONFIG & ~(
                                                      LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1_Msk |
                                                      LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2_Msk)) |
            (10UL << LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1_Pos) |
            (20UL << LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2_Pos));
    base->AFE[portNo].PHY_GENERAL_CONFIG_2 = 0;
    base->AFE[portNo].PHY_ADC_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N_Msk;
    base->AFE[portNo].LICIO_VCCD_V1P1 &= ~LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN_Msk;

    /* Clear all PHY interrupts. */
    base->AFE[portNo].PHY_INTR = 0xFFFFFFFFUL;
}

#endif /* (!LVCMOS_16BIT_SDR) */

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
* Pointer to SIP driver context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_PhyInit(LVDSSS_LVDS_Type * base, uint8_t portNo, const cy_stc_lvds_phy_config_t * config, cy_stc_lvds_context_t *lvdsContext)
{
    uint8_t i = 0;
#if (!LVCMOS_16BIT_SDR)
    uint32_t timeoutCount = 0;
    uint32_t cnt_10us24_val = 0; /* For LVCMOS DDR */
    uint32_t interface_freq;
#endif /* (!LVCMOS_16BIT_SDR) */
    bool sf2bit = false;
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;

    if((base == NULL) || (config == NULL) || (portNo >= CY_LVDS_MAX_PHY_INSTANCE) ||
        (lvdsContext == NULL))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

#if LVCMOS_16BIT_SDR
    if (config->modeSelect == CY_LVDS_PHY_MODE_LVDS) {
        DBG_LVDS_ERR("FX2G3 device does not support LVDS interface\r\n");
        return CY_LVDS_BAD_PARAMETER;
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    /* Verify that the active part supports the desired function. */
    if (
#if LVCMOS_16BIT_SDR
            ((config->modeSelect == CY_LVDS_PHY_MODE_LVDS) && ((SipBlkInitConfig & (1UL << portNo)) != 0)) ||
#endif /* LVCMOS_16BIT_SDR */
            ((config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) && ((SipBlkInitConfig & (4UL << portNo)) != 0))
       ) {
        DBG_LVDS_ERR("Current device does not support selected configuration\r\n");
        return CY_LVDS_CONFIG_ERROR;
    }

    /* Initialize LVDS context structure */
    lvdsContext->base = base;
    if(portNo == 0)
    {
        lvdsContext->phyConfigP0 = (cy_stc_lvds_phy_config_t *)config;
    }
#if (!LVCMOS_16BIT_SDR)
    else
    {
        lvdsContext->phyConfigP1 = (cy_stc_lvds_phy_config_t *)config;
    }

    interface_freq = lvdsContext->interfaceClock_kHz[portNo];
#endif /* (!LVCMOS_16BIT_SDR) */

    /* Calculate value to be set in LINK_CONFIG.TWO_BIT_SLV_FF field. */
    if (config->slaveFifoMode == CY_LVDS_SLAVE_FIFO_2BIT)
    {
        sf2bit = true;
    }

    /* Initialize and enable the LINK */
    if (Cy_SysLib_GetDeviceRevision() == 0x11)
    {
#if (!LVCMOS_16BIT_SDR)
        /* Active device revision is A0. */
        if (config->modeSelect == CY_LVDS_PHY_MODE_LVDS)
        {
            base->LINK_CONFIG[portNo] =
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LVDS_MODE,          config->modeSelect) |
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_GEARING_RATIO,      config->gearingRatio) |
                _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_WIDE_LINK,         config->wideLink) |
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_NUM_LANES,          FX_A0_LVDS_LANE_CNT_MAP[config->dataBusWidth]) |
                _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF_A0, sf2bit) |
                0x100UL;
        }
        else
#endif /* (!LVCMOS_16BIT_SDR) */
        {
            base->LINK_CONFIG[portNo] =
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LVDS_MODE,          config->modeSelect) |
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_GEARING_RATIO,      config->gearingRatio) |
                _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_WIDE_LINK,         config->wideLink) |
                _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_NUM_LANES,          config->dataBusWidth) |
                _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF_A0, sf2bit) |
                0x100UL;
        }
    }
    else
    {
        /* Active device revision is B0. */
        base->LINK_CONFIG[portNo] =
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LVDS_MODE,       config->modeSelect) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_GEARING_RATIO,   config->gearingRatio) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_WIDE_LINK,      config->wideLink) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_NUM_LANES,       config->dataBusWidth) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF, sf2bit) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_LINK_ENABLE,    1u);
#if (!LVCMOS_16BIT_SDR)
        /* In LVCMOS DDR mode with TX enabled, we need to set the BYPASS_DDR_PHY bit. */
        if ((config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) && (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_2_1) &&
            (config->dataBusDirection != CY_LVDS_PHY_AD_BUS_DIR_INPUT) && (config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_MASTER))
        {
            base->LINK_CONFIG[portNo] |= _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_BYPASS_DDR_PHY, 1u);
        }
#endif /* (!LVCMOS_16BIT_SDR) */
    }

    Cy_LVDS_GpifConfigClock(base, portNo, lvdsContext, config);

#if (!LVCMOS_16BIT_SDR)
    /* In LVDS mode, since the buffer wrap is done by HW without GPIF involvement, the
    below configuration is done. Each bit in the below register corresponds to 1 socket */
    if(config->modeSelect == CY_LVDS_PHY_MODE_LVDS)
    {
        base->LVDS_EOP_EOT = 0xFFFFFFFFu;
    }

    /* Configuration of LINK training registers, required for
       all LVDS modes and only for LVCMOS DDR Widelink mode */
    if((config->modeSelect == CY_LVDS_PHY_MODE_LVDS) ||
        ((config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) && (config->wideLink == true) && (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_2_1)))
    {
        if (config->loopbackModeEn == false)
        {
            base->TRAINING_BLK_CFG = 0x3;
            base->TRAINING_BLK     = config->linkTrainingPattern;
        }
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    /* LVDS IP and PHY should be enabled before AFE initialization */
    base->CTL    =  _BOOL2FLD(LVDSSS_LVDS_CTL_PHY_ENABLED,  1u) |
                    _BOOL2FLD(LVDSSS_LVDS_CTL_IP_ENABLED,   1u);

#if (!LVCMOS_16BIT_SDR)
    /* AFE Initialization sequence */
    if(config->loopbackModeEn == true)
    {
        if(config->isPutLoopbackMode)
        {
            base->AFE[portNo].LICIO_VCCD_V1P1     |= 0x00000001;
            base->AFE[portNo].PHY_GENERAL_CONFIG  |= 0x0000003C;
            base->AFE[portNo].LICIO_VSSIO_IREF    |= 0x00000001;
            Cy_SysLib_DelayUs(10);
            base->AFE[portNo].RX[9]               = 0x00000000;
            base->AFE[portNo].RX[9]               = 0x0000002B;
            base->AFE[portNo].GENERAL_RX          |= 0x00000004;
            Cy_SysLib_DelayUs(10);
            base->AFE[portNo].PLL_CONFIG          |= 0x00000404;
            Cy_SysLib_DelayUs(1);
            base->AFE[portNo].PHY_TRAIN_CONFIG    |= 0x00000001;
            Cy_LVDS_EnableLoopbackMode(base, portNo);
            DBG_LVDS_INFO("Loopback mode Init Done for Port: %d\r\n", portNo);
        }
    }
    else
#endif /* (!LVCMOS_16BIT_SDR) */
    {
        /*********** AFE Initialization sequence for LVCMOS SDR(Slave or Master Clk Mode RX/TX
                      and DDR (Master Clk Mode TX) ************/
        if (
                (
                 (config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) &&
                 (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_1_1)
                )
#if (!LVCMOS_16BIT_SDR)
                ||
                (
                 (config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) &&
                 (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_2_1) &&
                 (config->dataBusDirection != CY_LVDS_PHY_AD_BUS_DIR_INPUT) &&
                 (config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_MASTER)
                )
#endif /* (!LVCMOS_16BIT_SDR) */
           )
        {
            DBG_LVDS_INFO("LVCMOS SDR Mode\r\n");
            if ((config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_MASTER)) {
                DBG_LVDS_INFO("LVCMOS SDR RX/TX OR DDR TX MASTER Mode\r\n");
                base->AFE[portNo].LICIO_VCCD_V1P1       |= _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN, 1u);
            }

            base->AFE[portNo].PHY_ADC_CONFIG            |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N, 1u);

            base->AFE[portNo].PHY_GENERAL_CONFIG        |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO, 1u);

            if ((config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_MASTER))
                base->AFE[portNo].PHY_GENERAL_CONFIG    |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P1_VCCD, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_SCANON, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P25_VCCD, 1u);


            if ((config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_MASTER) && (config->lvcmosMasterClkSrc == CY_LVDS_MASTER_CLK_SRC_PLL))
            {
                base->AFE[portNo].PLL_CONFIG            |=  _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_REF_SEL, 2u) |  /* Reference clock taken as clock HF */
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_BYPASS, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN, 1u);

                base->AFE[portNo].LICIO_VSSIO_IREF      |=  _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN, 1u);
                Cy_SysLib_DelayUs(10);

                base->AFE[portNo].REG_1P25              |=  _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_ENABLE, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_USE_REG, 1u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_TRIM_VREG_1P25, 7u);
            }

            base->AFE[portNo].GENERAL_RX_LVCMOS         |=  _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_CLK_MODE, 0u) |
                                                            _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_MUX_SEL, 1u);

            base->AFE[portNo].LICIO_CIO_CTRL_DS         |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_CTRL_DS_DS, 0x1Fu);

            for(i = 0; i < 17; i++)
            {
                base->AFE[portNo].LICIO_CIO_DATA_DS[i]  |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_DATA_DS_DS, 0x1Fu);
            }

            /* SDR clock pads (P0CP and P1D1P) */
            /* P0CP at CTL_8 (PORT0 CLK) and P1D1P at DQ_25 (PORT1 CLK) */
            /* Wide Link Case */
            if (config->wideLink == true)
            {
                if (config->lvcmosClkMode != CY_LVDS_LVCMOS_CLK_MASTER)
                {

                    base->AFE[0].LICIO_CIO[24]  |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);
                    base->AFE[1].LICIO_CIO[9]   |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);
                }
                else
                {
                    base->AFE[0].LICIO_CIO[24]  |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_TX_EN, 0x1u);
                    base->AFE[1].LICIO_CIO[9]   |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_TX_EN, 0x1u);
                }
            }
            else /* Narrow Link case */
            {
                /* PORT1 */
                if (portNo)
                {
                    if (config->lvcmosClkMode != CY_LVDS_LVCMOS_CLK_MASTER)
                    {

                        base->AFE[portNo].LICIO_CIO[9]   |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);
                    }
                    else
                    {
                        base->AFE[portNo].LICIO_CIO[9]   |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_TX_EN, 0x1u);
                    }
                }
                else /* PORT0 */
                {
                    if (config->lvcmosClkMode != CY_LVDS_LVCMOS_CLK_MASTER)
                    {

                        base->AFE[portNo].LICIO_CIO[24]  |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);
                    }
                    else
                    {
                        base->AFE[portNo].LICIO_CIO[24]  |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_TX_EN, 0x1u);
                    }
                }
            }

            /* Note: Direction (input or output) of data and control lines can be programmed by GPIF or from AFE Register.
               Note: In LVCMOS SDR RX mode there is no need to activate rx_lvcmos as AFE is bypassed altogether. */
            /* Please make sure to select:
                1. LVCMOS CLK Mode (Master/Slave) using GPIF->GPIF_CONFIG.CLK_SOURCE
                2. LVCMOS ClK Src  (in Master mode) using GPIF->LVCMOS_CLK_OUT_CFG.CLK_SRC
                3. Using Correct Div for Clk Src (Master mode) using GPIF->LVCMOS_CLK_OUT_CFG.CLK_SRC_DIV_VAL */

#if (!LVCMOS_16BIT_SDR)
            DBG_LVDS_INFO("LVCMOS SDR Init Done for Port: %d\r\n", portNo);
#else
            DBG_LVDS_INFO("LVCMOS SDR or SDR/DDR Master Init Done for Port: %d\r\n", portNo);
#endif /* (!LVCMOS_16BIT_SDR) */
        }
#if (!LVCMOS_16BIT_SDR)
        /*********** AFE Initialization sequence for LVCMOS DDR RX SLAVE CLK MODE ************/
        /* This case is for DDR RX Slave */
        if((config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS) && (config->gearingRatio == CY_LVDS_PHY_GEAR_RATIO_2_1) &&
                (config->lvcmosClkMode == CY_LVDS_LVCMOS_CLK_SLAVE))
        {
            if (config->dataBusDirection != CY_LVDS_PHY_AD_BUS_DIR_INPUT)
            {
                DBG_LVDS_ERR("DDR RX Slave Clk Mode Invalid Data Bus Direction\r\n");
                return CY_LVDS_CONFIG_ERROR;
            }

            DBG_LVDS_INFO("DDR RX Init started for Port: %d \r\n", portNo);

            if ((interface_freq > LVCMOS_DDR_SLAVE_MAX_INTERFACE_FREQ_RX_KHZ) ||
                (interface_freq < LVCMOS_DDR_SLAVE_MIN_INTERFACE_FREQ_RX_KHZ))
            {
                DBG_LVDS_ERR("DDR Invalid Interface Freq: %d kHz \r\n", interface_freq);
                return CY_LVDS_CONFIG_ERROR;
            }

            DBG_LVDS_INFO("LVCMOS DDR Interface CLK:%d kHz \r\n", interface_freq);
            base->AFE[portNo].LICIO_VCCD_V1P1       |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN, 1u);
            base->AFE[portNo].PHY_ADC_CONFIG        |=   _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N, 1u);

            /* Counter to wait 10.24 us used in locking procedure of DLL */
            cnt_10us24_val = (interface_freq / 100) + 100;
            base->AFE[portNo].PHY_GENERAL_CONFIG_2  |=  _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_2_CNT_10US24_VAL, cnt_10us24_val);

            base->AFE[portNo].PHY_GENERAL_CONFIG    |=   _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P1_VCCD, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_SCANON, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P25_VCCD, 1u);
            base->AFE[portNo].PLL_CONFIG            |=   _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_REF_SEL, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_BYPASS, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_RUN_AWAY_DIS, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN, 1u);
            base->AFE[portNo].LICIO_VSSIO_IREF      |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN, 1u);
            Cy_SysLib_DelayUs(10);

            base->AFE[portNo].REG_1P25              |=   _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_ENABLE, 1u) |
                                                        _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_USE_REG, 1u);
            Cy_SysLib_DelayUs(10);

            /* Since only RX is supported in DDR mode, all data lines will always be input */
            for(i = 0; i < 16; i++)
            {
                base->AFE[portNo].RX_LVCMOS[i]      |= _BOOL2FLD(LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS, 1u);
            }

            /* Depending on control bus bit map, pins which are input needs to be configured - For widelink there can be 20 pins*/
            if(config->wideLink == true)
            {
                for(i = 0; i < 10; i++)
                {
                    if(((config->ctrlBusBitMap >> (i+(10*portNo)))) & 0x0001)
                        base->AFE[portNo].RX_LVCMOS[i+16] |= _BOOL2FLD(LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS, 1u);
                }
            }
            else
            {
                for(i = 0; i < 10; i++)
                {
                    if((config->ctrlBusBitMap >> i) & 0x0001)
                        base->AFE[portNo].RX_LVCMOS[i+16] |= _BOOL2FLD(LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS, 1u);
                }
            }

            /* Frame clock */
            base->AFE[portNo].LICIO_CIO[26]  |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);

            base->AFE[portNo].LICIO_CIO_CTRL_DS     |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_CTRL_DS_DS, 0x1Fu);
            for(i = 0; i < 17; i++)
            {
                base->AFE[portNo].LICIO_CIO_DATA_DS[i] |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_DATA_DS_DS, 0x1Fu);
            }

            base->AFE[portNo].GENERAL_RX_LVCMOS |= _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_EN_DDR, 1u);
            base->AFE[portNo].PLL_CONFIG_2      |= _VAL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_RA_UP_TR, 2u);
            base->AFE[portNo].DLL_M_CONFIG      |= _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_EN, 1u) |
                _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_SPEED_MODE, 1u) |
                _VAL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_MDLL_SEL_PH, glLvcmosClkPhaseSel[portNo]) |
                _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_MDLL_OW_PH, 1u);
            timeoutCount = 0;
            while(!(base->AFE[portNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk))
            {
                Cy_SysLib_Delay(1);
                timeoutCount++;
                if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
                {
                    status = CY_LVDS_TIMEOUT_ERROR;
                    DBG_LVDS_ERR("DDR DLL_M lock timed out for Port: %d\r\n", portNo);
                    return status;
                }
            }
            base->AFE[portNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk;
            status = CY_LVDS_SUCCESS;
            DBG_LVDS_INFO("DDR DLL_M lock done for Port: %d\r\n", portNo);

            base->AFE[portNo].PHY_TRAIN_CONFIG      |=   _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN, 1u);
            base->AFE[portNo].PHY_GENERAL_CONFIG    |=   _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM, CY_LVDS_PHY_DESKEW_SLOW);

            timeoutCount = 0;
            while(!(base->AFE[portNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_DESKEW_COMPLETED_Msk))
            {
                Cy_SysLib_Delay(1);
                timeoutCount++;
                if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
                {
                    status = CY_LVDS_TIMEOUT_ERROR;
                    DBG_LVDS_ERR("DDR DESKEW timed out for Port: %d\r\n", portNo);
                    return status;
                }
            }
            base->AFE[portNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_DESKEW_COMPLETED_Msk;
            status = CY_LVDS_SUCCESS;
            DBG_LVDS_INFO("DDR Deskew complete for Port: %d\r\n", portNo);
        }
#endif /* (!LVCMOS_16BIT_SDR) */

#if (!LVCMOS_16BIT_SDR)
        /*********** AFE Initialization sequence for LVDS Interface  ************/
        if (config->modeSelect == CY_LVDS_PHY_MODE_LVDS) {
            status = Cy_LVDS_Phy_Lvds_Init(base, lvdsContext, config, portNo);
        }
#endif /* (!LVCMOS_16BIT_SDR) */
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_LVCMOS_PhyDeInit
****************************************************************************//**
*
* De-initialise the Sensor Interface Port if previously configured in LVCMOS mode.
*
* \param base
* Pointer to the SIP register base address
*
* \param portNo
* Port number on the Sensor Interface port (SIP)
*
*******************************************************************************/
static void Cy_LVCMOS_PhyDeInit(LVDSSS_LVDS_Type *base, uint8_t portNo)
{
    uint8_t i;

    /* Disable PHY training parameters. */
    base->LINK_TRAINING_STS[portNo] = 0x01FF01FFUL;
    base->AFE[portNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN_Msk;
    base->TRAINING_BLK_CFG &= ~(1UL << portNo);
    base->AFE[portNo].PHY_GENERAL_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM_Msk;

    /* Disable clock resources. */
    base->AFE[portNo].DLL_M_CONFIG = 0;
    base->AFE[portNo].PLL_CONFIG_2 &= ~LVDSSS_LVDS_AFE_PLL_CONFIG_2_PLL_RA_UP_TR_Msk;
    base->AFE[portNo].GENERAL_RX_LVCMOS &= ~LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_EN_DDR_Msk;

    /* Undo IO configuration. */
    for (i = 0; i < 17; i++) {
        base->AFE[portNo].LICIO_CIO_DATA_DS[i] = 0;
    }
    base->AFE[portNo].LICIO_CIO_CTRL_DS = 0;
    for (i = 0; i < 26; i++) {
        base->AFE[portNo].LICIO_CIO[i] &= ~(
                LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN_Msk |
                LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_TX_EN_Msk);
    }
    for (i = 0; i < 10; i++) {
        base->AFE[portNo].RX_LVCMOS[i] &= ~LVDSSS_LVDS_AFE_RX_LVCMOS_EN_LVCMOS_Msk;
        base->AFE[portNo].RX[i] = LVDSSS_LVDS_AFE_RX_FIXTIME_FRAMECLK_Msk;
    }
    base->AFE[portNo].GENERAL_RX_LVCMOS = 0;

    /* Disable regulators. */
    base->AFE[portNo].REG_1P25 &= ~(
            LVDSSS_LVDS_AFE_REG_1P25_ENABLE_Msk |
            LVDSSS_LVDS_AFE_REG_1P25_USE_REG_Msk);
    base->AFE[portNo].LICIO_VSSIO_IREF &= ~LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN_Msk;
    base->AFE[portNo].PLL_CONFIG &= ~(
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_REF_SEL_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_BYPASS_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_RUN_AWAY_DIS_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN_Msk |
            LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN_Msk);
    base->AFE[portNo].PHY_GENERAL_CONFIG &= ~(
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P1_VCCD_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_SCANON_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_V1P25_VCCD_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_MONITOR_TIME_Msk |
            LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM_Msk);
    base->AFE[portNo].PHY_GENERAL_CONFIG = (
            (base->AFE[portNo].PHY_GENERAL_CONFIG & ~(
                                                      LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1_Msk |
                                                      LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2_Msk)) |
            (10UL << LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH1_Pos) |
            (20UL << LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ERR_THRESH2_Pos));
    base->AFE[portNo].PHY_GENERAL_CONFIG_2 = 0;
    base->AFE[portNo].PHY_ADC_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N_Msk;
    base->AFE[portNo].LICIO_VCCD_V1P1 &= ~LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN_Msk;

    /* Clear all PHY interrupts. */
    base->AFE[portNo].PHY_INTR = 0xFFFFFFFFUL;
}

/*******************************************************************************
* Function Name: Cy_LVDS_PhyDeinit
****************************************************************************//**
*
* De-initialises LVDS Sensor interface port and Analog front end.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
* \param lvdsContext
* Pointer to SIP driver context structure.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_PhyDeinit (LVDSSS_LVDS_Type *base, uint8_t portNo, cy_stc_lvds_context_t *lvdsContext)
{
    if ((base == NULL) || (portNo >= 2) || (lvdsContext == NULL)) {
        return CY_LVDS_BAD_PARAMETER;
    }

#if (LVCMOS_16BIT_SDR)
    if (portNo == 0) {
        Cy_LVCMOS_PhyDeInit(base, portNo);
        base->GPIF_CLK_SEL[portNo] = 0x00000614UL;
        lvdsContext->phyConfigP0 = NULL;
    }
#else
    /* Disable loopback configuration. */
    base->LOOPBACK_CFG = 0;

    if (portNo == 0) {
        if (lvdsContext->phyConfigP0->wideLink) {
            if (lvdsContext->phyConfigP0->modeSelect == CY_LVDS_PHY_MODE_LVDS) {
                Cy_LVDS_Phy_Lvds_Deinit(base, 1);
            } else {
                Cy_LVCMOS_PhyDeInit(base, 1);
            }

            base->GPIF_CLK_SEL[1] = 0x00000614UL;
            base->LINK_CONFIG[1]  = 0;
            lvdsContext->phyConfigP1 = NULL;
        }

        if (lvdsContext->phyConfigP0->modeSelect == CY_LVDS_PHY_MODE_LVDS) {
            Cy_LVDS_Phy_Lvds_Deinit(base, portNo);
        } else {
            Cy_LVCMOS_PhyDeInit(base, portNo);
        }

        lvdsContext->phyConfigP0 = NULL;
    } else {
        if (lvdsContext->phyConfigP1->modeSelect == CY_LVDS_PHY_MODE_LVDS) {
            Cy_LVDS_Phy_Lvds_Deinit(base, portNo);
        } else {
            Cy_LVCMOS_PhyDeInit(base, portNo);
        }

        lvdsContext->phyConfigP1 = NULL;
    }

    base->GPIF_CLK_SEL[portNo] = 0x00000614UL;
    base->LINK_CONFIG[portNo] = 0;
#endif /* (!LVCMOS_16BIT_SDR) */

    base->LVDS_EOP_EOT = 0x00000000u;
    return CY_LVDS_SUCCESS;
}

#if (!LVCMOS_16BIT_SDR)

static uint8_t  LvdsPhyTrainAttempts = 1;
static uint32_t LvdsSlaveDllPhase[18][LVDS_PHY_TRAIN_MAX_COUNT] = {{0}};

/*******************************************************************************
* Function Name: Cy_LVDS_SetPhyTrainingLoopCount
****************************************************************************//**
*
* Specify the number of times LVDS PHY training should be attempted.
*
* Running the PHY training multiple times and taking the most frequent result for
* each lane is recommended to ensure that the best sampling phases are selected
* for each of the LVDS lanes. The count can be set to a maximum value of up to 10
* attempts, and is left with a count of 1 by default. This function must be called
* before \ref Cy_LVDS_PhyTrainingStart is called.
*
* The LVDS transmitter used must ensure that the PHY training sequence is sent
* for a duration of (count * 50 us) so that the receiver has sufficient time to
* finish the training and identify the best values.
*
* \param base
* Pointer to the LVDS register base address
*
* \param lvdsContext
* Pointer to the LVDS driver context structure.
*
* \param count
* Number of PHY training attempts to be made per LVDS lane.
*
*******************************************************************************/
void Cy_LVDS_SetPhyTrainingLoopCount (LVDSSS_LVDS_Type *base, cy_stc_lvds_context_t *lvdsContext, uint8_t count)
{
    if ((base != NULL) && (lvdsContext != NULL)) {

        /* Limit the maximum number of attempts to LVDS_PHY_TRAIN_MAX_COUNT. */
        if (count > LVDS_PHY_TRAIN_MAX_COUNT) {
            count = LVDS_PHY_TRAIN_MAX_COUNT;
        }

        LvdsPhyTrainAttempts = count;
    }
}

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
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_PhyTrainingStart(LVDSSS_LVDS_Type* base, uint8_t portNo, const cy_stc_lvds_phy_config_t * config)
{
    uint32_t timeoutCount = 0;
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;
    uint32_t most_frequent_phase = 0;
    uint32_t max_count = 0;
    uint32_t count = 0;
    uint32_t intMask;
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t k = 0;

    /* PHY training is only required for LVDS modes */
    if (config->modeSelect == CY_LVDS_PHY_MODE_LVCMOS)
        return CY_LVDS_CONFIG_ERROR;

    if (config->wideLink) {
        portNo = 0;
    }

    intMask = Cy_SysLib_EnterCriticalSection();

    /* Make sure loop count is limited to avoid memory corruption. */
    if (LvdsPhyTrainAttempts > LVDS_PHY_TRAIN_MAX_COUNT) {
        LvdsPhyTrainAttempts = LVDS_PHY_TRAIN_MAX_COUNT;
    }

    for (j = 0; j < LvdsPhyTrainAttempts; j++) {
        /* Enable PHY training. */
        base->AFE[portNo].PHY_TRAIN_CONFIG  |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN, 1);
        if (config->wideLink) {
            base->AFE[1].PHY_TRAIN_CONFIG  |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN, 1);
        }
        Cy_SysLib_DelayUs(5);

        /* Wait until training is complete. */
        while (!(base->AFE[portNo].PHY_GENERAL_STATUS_2 & LVDSSS_LVDS_AFE_PHY_GENERAL_STATUS_2_DESKEW_COMPLETED_Msk)) {
            Cy_SysLib_DelayUs(10);
            timeoutCount++;
            if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT) {
                status = CY_LVDS_TIMEOUT_ERROR;
                DBG_LVDS_ERR("DESKEW timed out for Port: %d\r\n", portNo);
                Cy_SysLib_ExitCriticalSection(intMask);
                return status;
            }
        }

        if (config->wideLink) {
            timeoutCount = 0;
            while (!(base->AFE[1].PHY_GENERAL_STATUS_2 & LVDSSS_LVDS_AFE_PHY_GENERAL_STATUS_2_DESKEW_COMPLETED_Msk)) {
                Cy_SysLib_DelayUs(10);
                timeoutCount++;
                if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT) {
                    status = CY_LVDS_TIMEOUT_ERROR;
                    DBG_LVDS_ERR("DESKEW timed out for Port: 1\r\n");
                    Cy_SysLib_ExitCriticalSection(intMask);
                    return status;
                }
            }
        }

        /* Save the training results. */
        for (i = 0; i < 9 ; i++) {
            LvdsSlaveDllPhase[portNo * 9 + i][j] = base->AFE[portNo].DLL_S_STATUS[i] & 0x0F;
        }

        if (config->wideLink) {
            for (i = 9; i < 18 ; i++) {
                LvdsSlaveDllPhase[i][j] = base->AFE[1].DLL_S_STATUS[i - 9] & 0x0F;
            }
        }

        /* If we have more PHY training cycles to be performed, clear the TRAIN_EN bit. */
        if (j < (LvdsPhyTrainAttempts - 1)) {
            base->AFE[portNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN_Msk;
            if (config->wideLink) {
                base->AFE[1].PHY_TRAIN_CONFIG  &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN_Msk;
            }
            Cy_SysLib_DelayUs(5);
        }
    }

    if (LvdsPhyTrainAttempts > 1) {
        /* Find the most common phase discovered for each lane and force that value. */
        for (i = 0; i < 9; i++) {

            max_count = 0;
            for (j = 0; j < LvdsPhyTrainAttempts; j++) {
                count = 0;
                for (k = 0; k < LvdsPhyTrainAttempts; k++) {
                    if (LvdsSlaveDllPhase[portNo * 9 + i][j] == LvdsSlaveDllPhase[portNo * 9 + i][k]) {
                        count++;
                    }
                }

                if (count >= max_count) {
                    max_count = count;
                    most_frequent_phase = LvdsSlaveDllPhase[portNo * 9 + i][j];
                }
            }

            base->AFE[portNo].DLL_S_CONFIG[i] &= ~(0xF <<4);
            base->AFE[portNo].DLL_S_CONFIG[i] |= ((most_frequent_phase << 4) | 0x100);
        }

        if (config->wideLink) {
            for (i = 9; i < 18; i++) {

                max_count = 0;
                for (j = 0; j < LvdsPhyTrainAttempts; j++) {
                    count = 0;
                    for (k = 0; k < LvdsPhyTrainAttempts; k++) {
                        if (LvdsSlaveDllPhase[i][j] == LvdsSlaveDllPhase[i][k]) {
                            count++;
                        }
                    }

                    if (count >= max_count) {
                        max_count = count;
                        most_frequent_phase = LvdsSlaveDllPhase[i][j];
                    }
                }

                base->AFE[1].DLL_S_CONFIG[i - 9] &= ~(0xF <<4);
                base->AFE[1].DLL_S_CONFIG[i - 9] |= ((most_frequent_phase << 4) | 0x100);
            }
        }
    }

    Cy_SysLib_ExitCriticalSection(intMask);
    return status;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GetLinkTrainingStatus
****************************************************************************//**
*
* Checks whether link training has been completed for the specified Sensor Interface
* Ports. Detailed status of link training is returned through the pCompletedLanes
* and pFailedLanes parameters.
*
* \param base
* Pointer to the LVDS register base address
*
* \param portNo
* Port number on the LVDS Sensor Interface port (SIP)
*
* \param pCompletedLanes
* Returns address to store bit-map representing lanes on which link training has
* been completed. Bit 16 represents the control lane and bits 15 to 0 represent the
* data lanes corresponding to the port.
*
* \param pFailedLanes
* Returns address to store bit-map representing lanes on which link training has
* failed. Bit 16 represents the control lane and bits 15 to 0 represent the
* data lanes corresponding to the port.
*
* \return cy_en_lvds_status_t
* CY_LVDS_SUCCESS - If link training for the port has completed without error.
* CY_LVDS_CONFIG_ERROR - If the port specified is not enabled or is in LVCMOS mode.
* CY_LVDS_LINK_TRAINING_ERROR - If link training failed on one or more lanes.
* CY_LVDS_TIMEOUT_ERROR - If link training has not been completed and there is no sequence error.
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GetLinkTrainingStatus (LVDSSS_LVDS_Type* base, uint8_t portNo,
        uint32_t *pCompletedLanes, uint32_t *pFailedLanes)
{
    cy_en_lvds_status_t stat = CY_LVDS_SUCCESS;
    uint32_t linkEnMask = 0x8001UL;
    uint32_t lanesToCheck = 0, temp = 0;
    uint32_t lanesDone = 0;
    uint32_t lanesFailed = 0;

    if ((base == NULL) || (portNo > 2)) {
        return CY_LVDS_BAD_PARAMETER;
    }

    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
        /* A0 Silicon. */
        linkEnMask   = 0x0101UL;
        lanesToCheck = ((1UL << (1UL << ((base->LINK_CONFIG[portNo] & 0xF0UL) >> 4))) - 1UL);
    } else {
        /* B0 Silicon. */
        linkEnMask   = 0x8001UL;
        lanesToCheck = ((1UL << (1U + ((base->LINK_CONFIG[portNo] & 0xF0UL) >> 4U))) - 1UL);
    }

    if (
            ((base->CTL & LVDSSS_LVDS_CTL_LINK_ENABLED_Msk) == 0) ||
            ((base->LINK_CONFIG[portNo] & linkEnMask) != linkEnMask)
       ) {
        return CY_LVDS_CONFIG_ERROR;
    }

    /* If link training is not enabled, return success with no lane specific status. */
    if ((base->TRAINING_BLK_CFG & (1UL << portNo)) != 0) {
        if (portNo != 0) {
            lanesDone = (
                    (base->LINK_TRAINING_STS[1] & lanesToCheck) |
                    ((base->LINK_TRAINING_STS[1] & 0x0100UL) << 8U)
                    );
            lanesFailed = (
                    ((base->LINK_TRAINING_STS[1] >> 16U) & lanesToCheck) |
                    ((base->LINK_TRAINING_STS[1] & 0x01000000UL) >> 8U)
                    );
        } else {
            temp = lanesToCheck & 0xFFUL;
            lanesDone = (
                    (base->LINK_TRAINING_STS[0] & temp) |
                    ((base->LINK_TRAINING_STS[0] & 0x0100UL) << 8U)
                    );
            lanesFailed = (
                    ((base->LINK_TRAINING_STS[0] >> 16U) & temp) |
                    ((base->LINK_TRAINING_STS[0] & 0x01000000UL) >> 8U)
                    );

            /* In WideLink case, collect status of upper 8 data lanes from LINK_TRAINING_STS[1]. */
            if ((lanesToCheck & 0xFF00UL) != 0) {
                temp = (lanesToCheck >> 8U);
                lanesDone   |= ((base->LINK_TRAINING_STS[1] & temp) << 8U);
                lanesFailed |= (((base->LINK_TRAINING_STS[1] >> 16U) & temp) << 8U);
            }
        }

        /* Clear failed bit on any lanes where link training (eventually) succeeded. */
        lanesFailed &= ~lanesDone;
    }

    /*
     * Update return status value:
     * If failure status seen on at least one lane, return CY_LVDS_LINK_TRAINING_ERROR.
     * If no failure status seen, but not all lanes have completed training; return CY_LVDS_TIMEOUT_ERROR.
     */
    if (lanesFailed != 0) {
        stat = CY_LVDS_LINK_TRAINING_ERROR;
    } else {
        if (lanesDone != (0x10000UL | lanesToCheck)) {
            stat = CY_LVDS_TIMEOUT_ERROR;
        }
    }

    /* Store the completion status in return parameter locations. */
    if (pCompletedLanes != NULL) {
        *pCompletedLanes = lanesDone;
    }
    if (pFailedLanes != NULL) {
        *pFailedLanes = lanesFailed;
    }

    return stat;
}
#endif /* (!LVCMOS_16BIT_SDR) */

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
* \ref  cy_en_lvds_phy_gpio_index_t
*
* \param dir
* \ref cy_en_lvds_phy_gpio_dir_t
*
* \param intrMode
* \ref cy_en_lvds_phy_gpio_intr_mode_t
*
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_PhyGpioModeEnable (LVDSSS_LVDS_Type* base,
                                uint8_t portNo,
                                cy_en_lvds_phy_gpio_index_t pinNo,
                                cy_en_lvds_phy_gpio_dir_t dir,
                                cy_en_lvds_phy_gpio_intr_mode_t intrMode)
{
    if(dir == CY_LVDS_PHY_GPIO_INPUT)
    {
        base->AFE[portNo].PHY_GPIO[pinNo] |=    _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_INPUT_EN, 1u) |
                                                _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_GPIO_ENABLE, 1u);

        /* Configure interrupt corresponding to GPIO if required*/
        if(intrMode != CY_LVDS_PHY_GPIO_NO_INTERRUPT)
        {
            base->AFE[portNo].PHY_GPIO_INTR_CFG |= _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_INTR_CFG_GPIO_INTR_PIN_SEL, pinNo);
            base->AFE[portNo].PHY_GPIO_INTR_CFG |= _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_INTR_CFG_INTRMODE, intrMode);
        }
    }
    else
    {
        base->AFE[portNo].PHY_GPIO[pinNo] |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_OUTPUT_EN, 1u) |
                                           _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_GPIO_ENABLE, 1u);
    }
}

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
* \ref  cy_en_lvds_phy_gpio_index_t
*
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_PhyGpioModeDisable (LVDSSS_LVDS_Type* base, uint8_t portNo, cy_en_lvds_phy_gpio_index_t pinNo)
{
    base->AFE[portNo].PHY_GPIO[pinNo] = _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GPIO_GPIO_ENABLE, 0);
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifEnableComp
****************************************************************************//**
*
* Configures comparators in GPIF block.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param compType
* \ref  cy_en_lvds_gpif_comp_type_t
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
* \funcusage
* \snippet
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifEnableComp(LVDSSS_LVDS_Type* base, uint8_t smNo, cy_en_lvds_gpif_comp_type_t compType, void *compValue, void *compMask)
{
    uint32_t * value = (uint32_t *)compValue;
    uint32_t * mask = (uint32_t *)compMask;

    if(smNo >= CY_LVDS_MAX_GPIF_INSTANCE)
    {
        return CY_LVDS_BAD_PARAMETER;
    }
    switch (compType)
    {
        case CY_LVDS_GPIF_COMP_CTRL:
            base->GPIF[smNo].GPIF_CTRL_COMP_VALUE = value[0];
            base->GPIF[smNo].GPIF_CTRL_COMP_MASK  = mask[0];
            base->GPIF[smNo].GPIF_CONFIG  |= LVDSSS_LVDS_GPIF_GPIF_CONFIG_CTRL_COMP_ENABLE_Msk;
            break;

        case CY_LVDS_GPIF_COMP_ADDR:
            base->GPIF[smNo].GPIF_ADDR_COMP_VALUE = value[0];
            base->GPIF[smNo].GPIF_ADDR_COMP_MASK  = mask[0];
            base->GPIF[smNo].GPIF_CONFIG  |= LVDSSS_LVDS_GPIF_GPIF_CONFIG_ADDR_COMP_ENABLE_Msk;
            break;

        case CY_LVDS_GPIF_COMP_DATA:
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD0 = value[0];
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD0  = mask[0];
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD1 = value[1];
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD1  = mask[1];
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD2 = value[2];
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD2  = mask[2];
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD3 = value[3];
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD3  = mask[3];
            base->GPIF[smNo].GPIF_CONFIG  |= LVDSSS_LVDS_GPIF_GPIF_CONFIG_DATA_COMP_ENABLE_Msk;
            break;

        default:
            return CY_LVDS_BAD_PARAMETER;
    }

    return CY_LVDS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifDisableComp
****************************************************************************//**
*
* Resets the comparators in GPIF block.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \param compType
* \ref  cy_en_lvds_gpif_comp_type_t
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
*
* \funcusage
* \snippet
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifDisableComp(LVDSSS_LVDS_Type* base, uint8_t smNo, cy_en_lvds_gpif_comp_type_t compType)
{
    if(smNo >= CY_LVDS_MAX_GPIF_INSTANCE)
    {
        return CY_LVDS_BAD_PARAMETER;
    }
    switch (compType)
    {
        case CY_LVDS_GPIF_COMP_CTRL:
            base->GPIF[smNo].GPIF_CTRL_COMP_VALUE = 0u;
            base->GPIF[smNo].GPIF_CTRL_COMP_MASK  = 0u;
            base->GPIF[smNo].GPIF_CONFIG  &= ~LVDSSS_LVDS_GPIF_GPIF_CONFIG_CTRL_COMP_ENABLE_Msk;
            break;

        case CY_LVDS_GPIF_COMP_ADDR:
            base->GPIF[smNo].GPIF_ADDR_COMP_VALUE = 0u;
            base->GPIF[smNo].GPIF_ADDR_COMP_MASK  = 0u;
            base->GPIF[smNo].GPIF_CONFIG  &= ~LVDSSS_LVDS_GPIF_GPIF_CONFIG_ADDR_COMP_ENABLE_Msk;
            break;

        case CY_LVDS_GPIF_COMP_DATA:
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD0 = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD0  = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD1 = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD1  = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD2 = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD2  = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_VALUE_WORD3 = 0u;
            base->GPIF[smNo].GPIF_DATA_COMP_MASK_WORD3  = 0u;
            base->GPIF[smNo].GPIF_CONFIG  &= ~LVDSSS_LVDS_GPIF_GPIF_CONFIG_DATA_COMP_ENABLE_Msk;
            break;

        default:
            return CY_LVDS_BAD_PARAMETER;
    }

    return CY_LVDS_SUCCESS;
}

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
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_GpifInitAddrCounter(LVDSSS_LVDS_Type * base, uint8_t smNo, uint32_t initValue, uint32_t limit, bool reload, bool upCount, uint8_t increment)
{
    /* Make sure that the counter is stopped */
    base->GPIF[smNo].GPIF_ADDR_COUNT_CONFIG = 0u;

    /* Set the initial value and the limit */
    base->GPIF[smNo].GPIF_ADDR_COUNT_RESET = initValue;
    base->GPIF[smNo].GPIF_ADDR_COUNT_LIMIT = limit;

    /* Set the counter configuration as desired */
    base->GPIF[smNo].GPIF_ADDR_COUNT_CONFIG = _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_ADDR_COUNT_CONFIG_ENABLE, 1u)       |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_ADDR_COUNT_CONFIG_RELOAD, reload)   |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_ADDR_COUNT_CONFIG_DOWN_UP, upCount) |
                                         _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_ADDR_COUNT_CONFIG_INCREMENT, increment);
}


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
* Selects counter bit to be connected to CTRL[9] output.
*
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_GpifInitCtrlCounter(LVDSSS_LVDS_Type* base, uint8_t smNo, uint32_t initValue, uint32_t limit, bool reload, bool upCount, uint8_t outputBit)
{
    /* Make sure that the counter is stopped. */
    base->GPIF[smNo].GPIF_CTRL_COUNT_CONFIG = 0;

    /* Set the initial value and the limit. */
    base->GPIF[smNo].GPIF_CTRL_COUNT_RESET = initValue;
    base->GPIF[smNo].GPIF_CTRL_COUNT_LIMIT = limit;

    /* Set the counter configuration as desired */
    base->GPIF[smNo].GPIF_CTRL_COUNT_CONFIG = _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_CTRL_COUNT_CONFIG_ENABLE, 1u)       |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_CTRL_COUNT_CONFIG_RELOAD_CNTR, reload)   |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_CTRL_COUNT_CONFIG_DOWN_UP_CNTR, upCount) |
                                         _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_CTRL_COUNT_CONFIG_CONNECT, outputBit);
}


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
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_GpifInitDataCounter(LVDSSS_LVDS_Type* base, uint8_t smNo, uint32_t initValueMsb, uint32_t initValueLsb, uint32_t limitMsb, uint32_t limitLsb, bool reload, bool upCount, uint8_t increment)
{
    /* Make sure that the counter is stopped. */
    base->GPIF[smNo].GPIF_DATA_COUNT_CONFIG = 0u;

    /* Set the initial value and the limit. */
    base->GPIF[smNo].GPIF_DATA_COUNT_RESET_LSB = initValueLsb;
    base->GPIF[smNo].GPIF_DATA_COUNT_RESET_MSB = initValueMsb;
    base->GPIF[smNo].GPIF_DATA_COUNT_LIMIT_LSB = limitLsb;
    base->GPIF[smNo].GPIF_DATA_COUNT_LIMIT_MSB = limitMsb;

    /* Set the counter configuration as desired. */
    base->GPIF[smNo].GPIF_DATA_COUNT_CONFIG = _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_DATA_COUNT_CONFIG_ENABLE, 1u)       |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_DATA_COUNT_CONFIG_RELOAD, reload)   |
                                         _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_DATA_COUNT_CONFIG_DOWN_UP, upCount) |
                                         _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_DATA_COUNT_CONFIG_INCREMENT, increment);
}


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
* \funcusage
* \snippet
*
*******************************************************************************/
void Cy_LVDS_GpifInitStateCounter(LVDSSS_LVDS_Type* base, uint8_t smNo, uint16_t limit)
{
    /* Make sure that the counter is stopped. */
    base->GPIF[smNo].GPIF_STATE_COUNT_CONFIG = 0u;

    /* Set the initial value and the limit. */
    base->GPIF[smNo].GPIF_STATE_COUNT_LIMIT = limit;

    /* Enable the counter */
    base->GPIF[smNo].GPIF_STATE_COUNT_CONFIG = _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_STATE_COUNT_CONFIG_ENABLE, 1u);
}


cy_en_lvds_status_t Cy_LVDS_GpifWriteData(LVDSSS_LVDS_Type* base, uint8_t smNo, uint32_t threadIndex, bool selectThread, uint32_t numWords, uint32_t *buffer, uint32 waitOption)
{
   return CY_LVDS_BAD_PARAMETER;
}

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
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
* \funcusage
* \snippet
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifReadData(LVDSSS_LVDS_Type* base,
                                        uint8_t smNo,
                                        uint32_t threadIndex,
                                        bool selectThread,
                                        uint32_t numWords,
                                        uint32_t *buffer,
                                        uint32 waitOption)
{
    cy_en_lvds_status_t status = CY_LVDS_BAD_PARAMETER;
    uint32_t timeoutCount = 0;
    uint32_t interruptMaskVal;
    if ((threadIndex >= CY_LVDS_MAX_THREAD_COUNT) || (base == NULL) ||
        (smNo >= CY_LVDS_MAX_GPIF_INSTANCE))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* If the thread has to be activated, check if software based thread selection
     * is permitted; and do so.
     */
    if (selectThread)
    {
        if (((base->GPIF[smNo].GPIF_CONFIG & LVDSSS_LVDS_GPIF_GPIF_CONFIG_THREAD_IN_STATE_Msk) != 0) ||
                ((base->GPIF[smNo].GPIF_AD_CONFIG & LVDSSS_LVDS_GPIF_GPIF_AD_CONFIG_AIN_DATA_Msk) == 0))
        {
            return CY_LVDS_CONFIG_ERROR;
        }

        base->GPIF[smNo].GPIF_AD_CONFIG = (base->GPIF[smNo].GPIF_AD_CONFIG & ~LVDSSS_LVDS_GPIF_GPIF_AD_CONFIG_DATA_THREAD_Msk) |
            (threadIndex << LVDSSS_LVDS_GPIF_GPIF_AD_CONFIG_DATA_THREAD_Pos);
    }

    /* Clear interrupt mask bits, to ensure the handler is not invoked */
    interruptMaskVal = base->GPIF[smNo].GPIF_INTR_MASK;
    base->GPIF[smNo].GPIF_INTR_MASK &= ~ (1 << (LVDSSS_LVDS_GPIF_GPIF_STATUS_IN_DATA_VALID_Pos + threadIndex));

    /* If the register already has data, read the first word from it directly without waiting for an event. */
    if (base->GPIF[smNo].GPIF_INTR & (1 << (LVDSSS_LVDS_GPIF_GPIF_STATUS_IN_DATA_VALID_Pos + threadIndex)))
    {
        *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD0;
        *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD1;
        *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD2;
        *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD3;
        base->THREAD[threadIndex].GPIF_DATA_CTRL = (1 << threadIndex);
        numWords--;
    }

    if (numWords)
    {
        while (numWords--)
        {
            while(!(base->GPIF[smNo].GPIF_INTR & (1 << (LVDSSS_LVDS_GPIF_GPIF_STATUS_IN_DATA_VALID_Pos + threadIndex))))
            {
                Cy_SysLib_Delay(1);
                timeoutCount++;
                if (timeoutCount >= 2500U)
                {
                    status = CY_LVDS_TIMEOUT_ERROR;
                    break;
                }
            }

            if (status == CY_LVDS_TIMEOUT_ERROR)
            {
                base->GPIF[smNo].GPIF_INTR_MASK = interruptMaskVal;
                return status;
            }

            /* Read the data into the buffer and clear the data valid flag. */
            *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD0;
            *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD1;
            *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD2;
            *buffer++ = base->THREAD[threadIndex].GPIF_INGRESS_DATA_WORD3;
            base->THREAD[threadIndex].GPIF_DATA_CTRL = (1 << threadIndex);
        }
    }
    base->GPIF[smNo].GPIF_INTR_MASK = interruptMaskVal;
    return CY_LVDS_SUCCESS;
}


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
* \funcusage
* \snippet
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS_Type* base, uint8_t threadIndex, uint32_t socketNo, bool flagOnData,
                                            uint16_t watermark, uint8_t burst)
{
    uint8_t socketNoLsb;
    socketNoLsb = socketNo & 0x00000001u;
    socketNo = socketNo >> 1;
    /* Parameter checks. */
    if ((threadIndex >= CY_LVDS_MAX_THREAD_COUNT) || (base == NULL) || (socketNo >= CY_LVDS_MAX_SOCKET_COUNT))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Disable the thread before enabling it with the correct configuration. */
    base->THREAD[threadIndex].GPIF_THREAD_CONFIG = _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 0u);

    /* Compute and update the register value. */
    base->THREAD[threadIndex].GPIF_THREAD_CONFIG = _VAL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_THREAD_NUM, socketNoLsb) |
                                                   _VAL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_THREAD_SOCK, socketNo) |
                                                   _VAL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_WM_CFG, flagOnData)    |
                                                   _VAL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_BURST_SIZE, burst)     |
                                                   _VAL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_WATERMARK, watermark)  |
                                                   _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 1u);

    return CY_LVDS_SUCCESS;
}


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
* Bit-mask representing Alpha signals which should be asserted when the state machine
* starts from the toState.
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
cy_en_lvds_status_t Cy_LVDS_GpifSMSwitch(LVDSSS_LVDS_Type * base, uint8_t smNo,
                                uint16_t fromState, uint16_t toState, uint16_t endState,
                                uint8_t initialAlpha, uint32_t switchTimeout)
{
    uint32_t switchVal = 0;
    uint32_t numdss;
    uint32_t gpifstat;
    uint8_t  curState;

    if((base == NULL) || (smNo >= CY_LVDS_MAX_GPIF_INSTANCE) ||
                (toState >= CY_LVDS_GPIF_MAX_NUM_STATES))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Find the DSS level of the state machine. */
    numdss   = _FLD2VAL(LVDSSS_LVDS_GPIF_GPIF_BUS_CONFIG2_STATE_FROM_CTRL, base->GPIF[smNo].GPIF_BUS_CONFIG2);

    /* Update the initial Alpha values for the next state switch. */
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT = _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_ALPHA_INIT, initialAlpha) |
                                               _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID, 1u);

    switchVal = _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_DESTINATION_STATE, toState) | (base->GPIF[smNo].GPIF_WAVEFORM_SWITCH);

    if (fromState < CY_LVDS_GPIF_MAX_NUM_STATES)
    {
        /* Check if the state machine is already in the fromState or a mirror. */
        curState = (base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT & LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CURRENT_STATE_Msk) >> LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_CURRENT_STATE_Pos;
        if ((curState & GPIF_MAX_STATE(numdss)) == (fromState & GPIF_MAX_STATE(numdss)))
        {
            switchVal |= _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u);
        }
        else
        {
            /* The timeout setting is assumed to be valid if a fromState is specified. */
            switchVal |= (_VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_TERMINAL_STATE, fromState)) | (_VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_TIMEOUT_MODE, 1u));
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH_TIMEOUT = switchTimeout;
        }
    }
    else
    {
        /* Set the switch immediately flag. */
        switchVal |= _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u);
    }

    /* If a valid endState is specified, set this information and enable the DONE event. */
    if (endState < CY_LVDS_GPIF_MAX_NUM_STATES)
    {
        switchVal |= (_VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_DONE_STATE, endState) | _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_DONE_ENABLE, 1u));
        base->GPIF[smNo].GPIF_INTR_MASK |= _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_INTR_GPIF_DONE, 1u);
    }

    /* Write into the GPIF register to request the switch. */
    base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = switchVal;

    gpifstat = base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT;
    switch (numdss)
    {
        case 1:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF7FFFFF) |
                                                    ((gpifstat & 0x80000000u) >> 8));
            break;
        case 2:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF3FFFFF) |
                                                    ((gpifstat & 0xC0000000u) >> 8));
            break;
        case 3:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF1FFFFF) |
                                                    ((gpifstat & 0xE0000000u) >> 8));
            break;
        default:
            break;
    }

    return CY_LVDS_SUCCESS;
}

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
* Alpha signal to be asserted when the state machine starts from stateIndex.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
* \note
* This function should be called after initialization both the PHY and LINK
* layer.
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifSMStart(LVDSSS_LVDS_Type* base, uint8_t smNo, uint8_t stateIndex, uint8_t initialAlpha)
{
    uint32_t numdss;
    uint32_t gpifstat;
    uint32_t switchVal;

    if(smNo >= CY_LVDS_MAX_GPIF_INSTANCE)
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* State checks. */
    if (((base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT & LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID_Msk) == 1) ||
            ((base->GPIF[smNo].LEFT.WAVEFORM[0].WAVEFORM2 & 0x80000000ul) == 0))
    {
        return CY_LVDS_CONFIG_ERROR;
    }

    /* Mark the waveform as valid. */
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT = _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_ALPHA_INIT, initialAlpha) |
                                               _VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID, 1u);

    /* Switch to the desired state and start execution. */
    switchVal = base->GPIF[smNo].GPIF_WAVEFORM_SWITCH;
    switchVal = (switchVal & 0xFF00003A) | (_VAL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_DESTINATION_STATE, stateIndex));

    base->GPIF[smNo].GPIF_WAVEFORM_SWITCH  = switchVal;
    base->GPIF[smNo].GPIF_WAVEFORM_SWITCH |= _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH, 1u) |
                                             _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u);

    gpifstat = base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT;
    numdss   = _FLD2VAL(LVDSSS_LVDS_GPIF_GPIF_BUS_CONFIG2_STATE_FROM_CTRL, base->GPIF[smNo].GPIF_BUS_CONFIG2);
    switch (numdss)
    {
        case 1:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF7F003A) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH, 1u) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u) |
                                                    ((gpifstat & 0x80000000u) >> 8));
            break;

        case 2:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF3F003A) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH, 1u) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u) |
                                                    ((gpifstat & 0xC0000000u) >> 8));
            break;

        case 3:
            base->GPIF[smNo].GPIF_WAVEFORM_SWITCH = ((switchVal & 0xFF1F003A) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_WAVEFORM_SWITCH, 1u) |
                                                    _BOOL2FLD(LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_SWITCH_SWITCH_NOW, 1u) |
                                                    ((gpifstat & 0xE0000000u) >> 8));
            break;

        default:
            break;
    }

    return CY_LVDS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_LVDS_GpifSMStop
****************************************************************************//**
*
* Stop the GPIF state machine.
*
* \param base
* Pointer to the LVDS register base address
*
* \param smNo
* GPIF State Machine number.
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid
* CY_LVDS_SUCCESS - If the operation is successful
* CY_LVDS_CONFIG_ERROR - If the sequence in which registers are configured is incorrect
*
* \note
* This function will only disable/stop the state machine. The GPIF configurations
* will be retained.
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifSMStop(LVDSSS_LVDS_Type* base, uint8_t smNo)
{
    uint32_t intMask;
    if(smNo >= CY_LVDS_MAX_GPIF_INSTANCE)
    {
        return CY_LVDS_BAD_PARAMETER;
    }
    if (((base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT & LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID_Msk) == 0))
    {
        return CY_LVDS_CONFIG_ERROR;
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT |= LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_PAUSE_Msk;
    Cy_SysLib_DelayUs(10);
    base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT = 0;
    base->GPIF[smNo].GPIF_INTR = 0xFFFFFFFFUL;
    base->GPIF[smNo].GPIF_INTR_MASK = 0;

    Cy_SysLib_ExitCriticalSection(intMask);

    return CY_LVDS_SUCCESS;

}

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
cy_en_lvds_status_t Cy_LVDS_GpifSMControl(LVDSSS_LVDS_Type * base, uint8_t smNo, bool pause)
{
    if(smNo >= CY_LVDS_MAX_GPIF_INSTANCE)
    {
        return CY_LVDS_BAD_PARAMETER;
    }
    if (((base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT & LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID_Msk) == 0))
    {
        return CY_LVDS_CONFIG_ERROR;
    }

    if (pause) {
        base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT |= LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_PAUSE_Msk;
    } else {
        base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT &= ~LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_PAUSE_Msk;
    }

    return CY_LVDS_SUCCESS;
}

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
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifWaveformLoad(LVDSSS_LVDS_Type* base, uint8_t smNo, uint8 stateCnt, uint8_t firstState, uint8_t *stateDataMap, cy_stc_lvds_gpif_wavedata_t *transitionData)
{
    uint16_t index, entry;

    /* Parameter checks. */
    if ((transitionData == NULL) || ((firstState + stateCnt) > CY_LVDS_GPIF_MAX_NUM_STATES) ||
        (smNo >= CY_LVDS_MAX_GPIF_INSTANCE) || (base == NULL))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    if (((base->GPIF[smNo].GPIF_WAVEFORM_CTRL_STAT & LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID_Msk) == 0))
    {
        return CY_LVDS_CONFIG_ERROR;
    }

    /* Copy the data from the input arrays into the waveform memory. */
    for (index = 0; index < stateCnt; index++)
    {
        /* No look-up table means direct mapping. */
        entry = (stateDataMap != 0) ? stateDataMap[index] : index;

        base->GPIF[smNo].LEFT.WAVEFORM[index + firstState].WAVEFORM0  = (transitionData[entry].leftData[0]);
        base->GPIF[smNo].LEFT.WAVEFORM[index + firstState].WAVEFORM1  = (transitionData[entry].leftData[1]);
        base->GPIF[smNo].LEFT.WAVEFORM[index + firstState].WAVEFORM2  = (transitionData[entry].leftData[2]);
        base->GPIF[smNo].LEFT.WAVEFORM[index + firstState].WAVEFORM3  = (transitionData[entry].leftData[3]);


        base->GPIF[smNo].RIGHT.WAVEFORM[index + firstState].WAVEFORM0 = (transitionData[entry].rightData[0]);
        base->GPIF[smNo].RIGHT.WAVEFORM[index + firstState].WAVEFORM1 = (transitionData[entry].rightData[1]);
        base->GPIF[smNo].RIGHT.WAVEFORM[index + firstState].WAVEFORM2 = (transitionData[entry].rightData[2]);
        base->GPIF[smNo].RIGHT.WAVEFORM[index + firstState].WAVEFORM3 = (transitionData[entry].rightData[3]);
    }

    return CY_LVDS_SUCCESS;
}

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
* This function should be called after initialization both the PHY and LINK
* layer.
*
*******************************************************************************/
void Cy_LVDS_GpifInitTransFunction(LVDSSS_LVDS_Type * base, uint8_t smNo, uint16 *fnTable)
{
    uint32_t index;

    for (index = 0; index < CY_LVDS_GPIF_NUM_TRANS_FUNC; index++)
    {
        base->GPIF[smNo].GPIF_FUNCTION[index] = (uint32_t)fnTable[index];
    }
}



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
* \note
* The gpifConig sructure will be generated by GPIF Designer tool.
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_GpifInit(LVDSSS_LVDS_Type * base, uint8_t smNo, const cy_stc_lvds_gpif_config_t *gpifConfig, cy_stc_lvds_context_t *lvdsContext)
{
    uint32_t addr = (uint32_t)&(base->GPIF[smNo]);
    uint8_t entry = 0;
    uint32_t index = 0;

    DBG_LVDS_INFO("Cy_LVDS_GpifInit Start\r\n");
    if ((base == NULL) || (lvdsContext == NULL) || (smNo >= CY_LVDS_MAX_GPIF_INSTANCE))
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Initialize context structure */
    if(smNo == 0)
        lvdsContext->gpifConfigP0 = (cy_stc_lvds_gpif_config_t *)gpifConfig;
    else
        lvdsContext->gpifConfigP1 = (cy_stc_lvds_gpif_config_t *)gpifConfig;


    for (index = 0; index < gpifConfig->regCount; index++)
    {
        *(uint32_t *)(addr + (uint32_t)gpifConfig->regData[index].regAddr) = (uint32_t)gpifConfig->regData[index].regValue;
    }

    for (index = 0; index < gpifConfig->stateCount; index++)
    {
        entry = gpifConfig->statePosition[index];
        base->GPIF[smNo].LEFT.WAVEFORM[index].WAVEFORM0  = gpifConfig->stateData[entry].leftData[0];
        base->GPIF[smNo].LEFT.WAVEFORM[index].WAVEFORM1  = gpifConfig->stateData[entry].leftData[1];
        base->GPIF[smNo].LEFT.WAVEFORM[index].WAVEFORM2  = gpifConfig->stateData[entry].leftData[2];
        base->GPIF[smNo].LEFT.WAVEFORM[index].WAVEFORM3  = gpifConfig->stateData[entry].leftData[3];

        base->GPIF[smNo].RIGHT.WAVEFORM[index].WAVEFORM0 = gpifConfig->stateData[entry].rightData[0];
        base->GPIF[smNo].RIGHT.WAVEFORM[index].WAVEFORM1 = gpifConfig->stateData[entry].rightData[1];
        base->GPIF[smNo].RIGHT.WAVEFORM[index].WAVEFORM2 = gpifConfig->stateData[entry].rightData[2];
        base->GPIF[smNo].RIGHT.WAVEFORM[index].WAVEFORM3 = gpifConfig->stateData[entry].rightData[3];
    }

    for (index = 0; index < gpifConfig->functionCount; index++)
    {
        base->GPIF[smNo].GPIF_FUNCTION[index] = gpifConfig->functionData[index];
    }

    return CY_LVDS_SUCCESS;
}



static void ReportThreadError(cy_en_lvds_gpif_thread_error_t errorCode,
                                cy_en_lvds_gpif_thread_no_t threadNo,
                                cy_stc_lvds_context_t *lvdsContext)
{
    if(lvdsContext->intrCallback->gpif_thread_error)
    {
        switch(errorCode)
        {
            case CY_LVDS_GPIF_THREAD_DIR_ERROR:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_DIR_ERROR, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_WR_OVERFLOW:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_WR_OVERFLOW, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_RD_UNDERRUN:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_RD_UNDERRUN, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_SCK_ACTIVE:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_SCK_ACTIVE, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_READ_FORCE_END:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_READ_FORCE_END, lvdsContext->userContext);
                break;
            case CY_LVDS_GPIF_THREAD_READ_BURST_ERR:
                lvdsContext->intrCallback->gpif_thread_error(threadNo, CY_LVDS_GPIF_THREAD_READ_BURST_ERR, lvdsContext->userContext);
                break;
            default:
                break;
        }
    }
}

static void HandleThreadError(uint32_t threadError, uint32_t lvdsError, cy_stc_lvds_context_t *lvdsContext)
{
    cy_en_lvds_gpif_thread_error_t errorCode;

    /* Check if Thread 0 has any error codes reported */
    if(threadError & LVDSSS_LVDS_LVDS_INTR_SET_WD0_THREAD0_ERR_Msk)
    {
       errorCode = (cy_en_lvds_gpif_thread_error_t)((lvdsError &
                    LVDSSS_LVDS_LVDS_ERROR_THREAD0_ERR_CODE_Msk) >>
                    LVDSSS_LVDS_LVDS_ERROR_THREAD0_ERR_CODE_Pos);
        ReportThreadError(errorCode, CY_LVDS_GPIF_THREAD_0, lvdsContext);
    }
    /* Check if Thread 1 has any error codes reported */
    if(threadError & LVDSSS_LVDS_LVDS_INTR_SET_WD0_THREAD1_ERR_Msk)
    {
        errorCode = (cy_en_lvds_gpif_thread_error_t)((lvdsError &
                    LVDSSS_LVDS_LVDS_ERROR_THREAD1_ERR_CODE_Msk) >>
                    LVDSSS_LVDS_LVDS_ERROR_THREAD1_ERR_CODE_Pos);
        ReportThreadError(errorCode, CY_LVDS_GPIF_THREAD_1, lvdsContext);
    }
#if (!LVCMOS_16BIT_SDR)
    /* Check if Thread 2 has any error codes reported */
    if(threadError & LVDSSS_LVDS_LVDS_INTR_SET_WD0_THREAD2_ERR_Msk)
    {
        errorCode = (cy_en_lvds_gpif_thread_error_t)((lvdsError &
                    LVDSSS_LVDS_LVDS_ERROR_THREAD2_ERR_CODE_Msk) >>
                    LVDSSS_LVDS_LVDS_ERROR_THREAD2_ERR_CODE_Pos);
        ReportThreadError(errorCode, CY_LVDS_GPIF_THREAD_2, lvdsContext);
    }
    /* Check if Thread 3 has any error codes reported */
    if(threadError & LVDSSS_LVDS_LVDS_INTR_SET_WD0_THREAD3_ERR_Msk)
    {
        errorCode = (cy_en_lvds_gpif_thread_error_t)((lvdsError &
                    LVDSSS_LVDS_LVDS_ERROR_THREAD3_ERR_CODE_Msk) >>
                    LVDSSS_LVDS_LVDS_ERROR_THREAD3_ERR_CODE_Pos);
        ReportThreadError(errorCode, CY_LVDS_GPIF_THREAD_3, lvdsContext);
    }
#endif /* (!LVCMOS_16BIT_SDR) */
}

static void ReportGpifError(uint32_t smNo, cy_stc_lvds_context_t *lvdsContext)
{
    cy_en_lvds_gpif_error_t gpifError;
    gpifError = (cy_en_lvds_gpif_error_t)lvdsContext->base->GPIF[smNo].GPIF_ERROR;
    switch(gpifError)
    {
    case CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_DMA_ADDR_RD_ERROR:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_DMA_ADDR_RD_ERROR, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR, lvdsContext->userContext);
        break;
    case CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR:
        lvdsContext->intrCallback->gpif_error(smNo, CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR, lvdsContext->userContext);
        break;
    default:
        break;
    }
}

static void HandleGpifInterrupts(uint32_t gpifIntr, cy_stc_lvds_context_t *lvdsContext)
{
    uint8_t i;
    uint32_t activeGpifIntr;
    for(i = 0; i < CY_LVDS_MAX_GPIF_INSTANCE; i++)
    {
        activeGpifIntr = (lvdsContext->base->GPIF[i].GPIF_INTR &
                        lvdsContext->base->GPIF[i].GPIF_INTR_MASK);
        lvdsContext->base->GPIF[i].GPIF_INTR = activeGpifIntr;
        if(activeGpifIntr != 0)
        {
            if(lvdsContext->intrCallback->gpif_events != NULL)
            {
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_GPIF_DONE_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_END_STATE, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_GPIF_FSM_INTR_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_SM_INTERRUPT, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_SWITCH_TIMEOUT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_SWITCH_TIMEOUT, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_CRC_ERROR_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_CRC_ERROR, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_ADDR_COUNT_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_ADDR_COUNTER, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_DATA_COUNT_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_DATA_COUNTER, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_CTRL_COUNT_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_CTRL_COUNTER, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_ADDR_COMP_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_ADDR_COMP, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_DATA_COMP_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_DATA_COMP, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_CTRL_COMP_HIT_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_CTRL_COMP, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_WAVEFORM_BUSY_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_WAVEFORM_BUSY, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_EG_DATA_EMPTY_BIT0_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH0, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_EG_DATA_EMPTY_BIT1_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH1, lvdsContext->userContext);
                }
#if (!LVCMOS_16BIT_SDR)
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_EG_DATA_EMPTY_BIT2_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH2, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_EG_DATA_EMPTY_BIT3_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_EG_DATA_EMPTY_TH3, lvdsContext->userContext);
                }
#endif /* (!LVCMOS_16BIT_SDR) */
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_IN_DATA_VALID_BIT0_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH0, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_IN_DATA_VALID_BIT1_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH1, lvdsContext->userContext);
                }
#if (!LVCMOS_16BIT_SDR)
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_IN_DATA_VALID_BIT2_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH2, lvdsContext->userContext);
                }
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_IN_DATA_VALID_BIT3_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_IN_DATA_VALID_TH3, lvdsContext->userContext);
                }
#endif /* (!LVCMOS_16BIT_SDR) */
                if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_INVLD_CMD_DET_Msk)
                {
                    lvdsContext->intrCallback->gpif_events(i, CY_LVDS_GPIF_EVT_CONTROL_BYTE_INVALID, lvdsContext->userContext);
                }
            }
            if(activeGpifIntr & LVDSSS_LVDS_GPIF_GPIF_INTR_GPIF_ERR_Msk)
            {
                if(lvdsContext->intrCallback->gpif_error != NULL)
                {
                    ReportGpifError(i, lvdsContext);
                }
            }
        }
    }
}

static void HandlePhyInterrupts(uint32_t phyIntr, cy_stc_lvds_context_t *lvdsContext)
{
    uint8_t i;
    uint32_t activePhyIntr;
    for(i = 0; i < CY_LVDS_MAX_PHY_INSTANCE; i++)
    {
        activePhyIntr = (lvdsContext->base->AFE[i].PHY_INTR &
                            lvdsContext->base->AFE[i].PHY_INTR_MASK);
        lvdsContext->base->AFE[i].PHY_INTR = (lvdsContext->base->AFE[i].PHY_INTR &
                                                    activePhyIntr);
        if(activePhyIntr != 0)
        {
            if(lvdsContext->intrCallback->phy_events != NULL)
            {
                if(activePhyIntr & LVDSSS_LVDS_AFE_PHY_INTR_GPIO_INTR_Msk)
                {
                    lvdsContext->intrCallback->phy_events(i, CY_LVDS_PHY_GPIO_INTR_PORT, lvdsContext->userContext);
                }
                if(activePhyIntr & LVDSSS_LVDS_AFE_PHY_INTR_MONITOR_ADJUST_FAIL_Msk)
                {
                    lvdsContext->intrCallback->phy_events(i, CY_LVDS_PHY_MONITOR_FAIL, lvdsContext->userContext);
                }
                if(activePhyIntr & LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_LOST_Msk)
                {
                    lvdsContext->intrCallback->phy_events(i, CY_LVDS_PHY_PLL_LOCK_LOST, lvdsContext->userContext);
                }
                if(activePhyIntr & LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_LOST_Msk)
                {
                    lvdsContext->intrCallback->phy_events(i, CY_LVDS_PHY_MDLL_LOCK_LOST, lvdsContext->userContext);
                }
            }
        }
    }
}

static void HandleThreadEvents(cy_en_lvds_gpif_thread_no_t threadNo, uint32_t threadEvent,
                                bool setClear, cy_stc_lvds_context_t *lvdsContext)
{
    uint8_t flagNo;
    if(setClear)
    {
        switch(threadNo)
        {
            case CY_LVDS_GPIF_THREAD_0:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH0_HDR_FLGS_SET, threadEvent);
            break;
            case CY_LVDS_GPIF_THREAD_1:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH1_HDR_FLGS_SET, threadEvent);
            break;
#if (!LVCMOS_16BIT_SDR)
            case CY_LVDS_GPIF_THREAD_2:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH2_HDR_FLGS_SET, threadEvent);
            break;
            case CY_LVDS_GPIF_THREAD_3:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH3_HDR_FLGS_SET, threadEvent);
            break;
#endif /* (!LVCMOS_16BIT_SDR) */
        }
        for(flagNo = 0; flagNo < 4; flagNo++)
        {
            if((threadEvent >> flagNo) & (0x1))
            {
                switch(flagNo)
                {
                    case 0:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG0_SET, lvdsContext->userContext);
                    break;
                    case 1:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG1_SET, lvdsContext->userContext);
                    break;
#if (!LVCMOS_16BIT_SDR)
                    case 2:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG2_SET, lvdsContext->userContext);
                    break;
                    case 3:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG3_SET, lvdsContext->userContext);
                    break;
#endif /* (!LVCMOS_16BIT_SDR) */
                }
            }
        }
    }
    else
    {
        switch(threadNo)
        {
            case CY_LVDS_GPIF_THREAD_0:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH0_HDR_FLGS_CLR, threadEvent);
            break;
            case CY_LVDS_GPIF_THREAD_1:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH1_HDR_FLGS_CLR, threadEvent);
            break;
#if (!LVCMOS_16BIT_SDR)
            case CY_LVDS_GPIF_THREAD_2:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH2_HDR_FLGS_CLR, threadEvent);
            break;
            case CY_LVDS_GPIF_THREAD_3:
            threadEvent = _FLD2VAL(LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH3_HDR_FLGS_CLR, threadEvent);
            break;
#endif /* (!LVCMOS_16BIT_SDR) */
        }
        for(flagNo = 0; flagNo < 4; flagNo++)
        {
            if((threadEvent >> flagNo) & (0x1))
            {
                switch(flagNo)
                {
                    case 0:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG0_CLR, lvdsContext->userContext);
                    break;
                    case 1:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG1_CLR, lvdsContext->userContext);
                    break;
#if (!LVCMOS_16BIT_SDR)
                    case 2:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG2_CLR, lvdsContext->userContext);
                    break;
                    case 3:
                    lvdsContext->intrCallback->gpif_thread_event(threadNo, CY_LVDS_GPIF_HDR_FLG3_CLR, lvdsContext->userContext);
                    break;
#endif /* (!LVCMOS_16BIT_SDR) */
                }
            }
        }
    }
}

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

void Cy_LVDS_IrqHandler(LVDSSS_LVDS_Type *base, cy_stc_lvds_context_t *lvdsContext)
{
    uint32_t activeIntr;
    uint32_t lvdsError;

    if ((base->LVDS_INTR_MASKED_WD0) || (base->LVDS_INTR_MASKED_WD1))
    {
        /* Check for active interrupts for word 0 */
        activeIntr = base->LVDS_INTR_MASKED_WD0;

        if(activeIntr != 0)
        {
            /* Fetch LVDS_ERROR before clearing LVDS_INTR_WD0 */
            lvdsError = lvdsContext->base->LVDS_ERROR;

            /* Clear Active Interrupt */
            base->LVDS_INTR_WD0 = activeIntr;
            if(lvdsContext->intrCallback != NULL)
            {
                if((activeIntr & LVDSSS_LVDS_LVDS_INTR_MASK_WD0_GPIF0_INTERRUPT_Msk) ||
                    (activeIntr & LVDSSS_LVDS_LVDS_INTR_MASK_WD0_GPIF1_INTERRUPT_Msk))
                {
                    HandleGpifInterrupts(activeIntr, lvdsContext);
                }
                if((activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_PHY_LINK0_INTERRUPT_Msk) ||
                    (activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_PHY_LINK1_INTERRUPT_Msk))
                {
                    HandlePhyInterrupts(activeIntr, lvdsContext);
                }

                if((activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_THREAD0_ERR_Msk) ||
                    (activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_THREAD1_ERR_Msk) ||
                    (activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_THREAD2_ERR_Msk) ||
                    (activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_THREAD3_ERR_Msk))
                {
                    HandleThreadError(activeIntr, lvdsError, lvdsContext);
                }
                if(lvdsContext->intrCallback->phy_events != NULL)
                {
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_FF_OVER_FLOW_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_FF_OVERFLOW, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_FF_OVER_FLOW_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_FF_OVERFLOW, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_DONE_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_TRAINING_DONE, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_DONE_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_TRAINING_DONE, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DETECTED_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_LNK_TRAIN_BLK_DET, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_BLK_DETECTED_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_LNK_TRAIN_BLK_DET, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DET_FAILD_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_LNK_TRAIN_BLK_DET_FAIL, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_BLK_DET_FAILD_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_LNK_TRAIN_BLK_DET_FAIL, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_ENTRY_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_L1_ENTRY, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L1_ENTRY_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_L1_ENTRY, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_EXIT_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_L1_EXIT, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L1_EXIT_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_L1_EXIT, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L3_ENTRY_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(0, CY_LVDS_PHY_L3_ENTRY, lvdsContext->userContext);
                    }
                    if(activeIntr & LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L3_ENTRY_Msk)
                    {
                        lvdsContext->intrCallback->phy_events(1, CY_LVDS_PHY_L3_ENTRY, lvdsContext->userContext);
                    }
                }
            }
        }

        /* Check for active interrupts for word 1 */
        activeIntr = base->LVDS_INTR_MASKED_WD1;
        if(activeIntr != 0)
        {
            /* Clear Active Interrupt */
            base->LVDS_INTR_WD1 = activeIntr;
            if(lvdsContext->intrCallback->gpif_thread_event != NULL)
            {
                /* Check if any of the clear flag bit for Thread 0, 1, 2, 3 is asserted*/
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH0_HDR_FLGS_CLR_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_0, activeIntr, 0, lvdsContext);
                }
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH1_HDR_FLGS_CLR_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_1, activeIntr, 0, lvdsContext);
                }
#if (!LVCMOS_16BIT_SDR)
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH2_HDR_FLGS_CLR_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_2, activeIntr, 0, lvdsContext);
                }
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH3_HDR_FLGS_CLR_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_3, activeIntr, 0, lvdsContext);
                }
#endif /* (!LVCMOS_16BIT_SDR) */

                /* Check if any of the set flag bit for Thread 0, 1, 2, 3 is asserted*/
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH0_HDR_FLGS_SET_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_0, activeIntr, 1, lvdsContext);
                }
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH1_HDR_FLGS_SET_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_1, activeIntr, 1, lvdsContext);
                }
#if (!LVCMOS_16BIT_SDR)
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH2_HDR_FLGS_SET_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_2, activeIntr, 1, lvdsContext);
                }
                if(activeIntr & LVDSSS_LVDS_LVDS_INTR_MASKED_WD1_TH3_HDR_FLGS_SET_Msk)
                {
                    HandleThreadEvents(CY_LVDS_GPIF_THREAD_3, activeIntr, 1, lvdsContext);
                }
#endif /* (!LVCMOS_16BIT_SDR) */
            }
        }
    }
}


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
void Cy_LVDS_LowPowerIrqHandler(LVDSSS_LVDS_Type * base, cy_stc_lvds_context_t *lvdsContext)
{
    uint32_t activeIntr;
    if (base->LVDS_LOW_PWR_INTR.LVDS_WAKEUP_INTR_MASKED)
    {
        activeIntr = base->LVDS_LOW_PWR_INTR.LVDS_WAKEUP_INTR_MASKED;

        base->LVDS_LOW_PWR_INTR.LVDS_WAKEUP_INTR = activeIntr;
        if(lvdsContext->intrCallback->low_power_events != NULL)
        {
            if(activeIntr & LVDSSS_LVDS_LVDS_LOW_PWR_INTR_LVDS_WAKEUP_INTR_LNK0_L3_EXIT_Msk)
            {
                lvdsContext->intrCallback->low_power_events(CY_LVDS_LOW_POWER_LNK0_L3_EXIT, lvdsContext->userContext);
            }
            if(activeIntr & LVDSSS_LVDS_LVDS_LOW_PWR_INTR_LVDS_WAKEUP_INTR_LNK1_L3_EXIT_Msk)
            {
                lvdsContext->intrCallback->low_power_events(CY_LVDS_LOW_POWER_LNK1_L3_EXIT, lvdsContext->userContext);
            }
        }
    }
}

#if (!LVCMOS_16BIT_SDR)
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
void Cy_LVDS_SetLvcmosDDRClockPhase (uint8_t sipNo, uint8_t clkPhase)
{
    /* Store the desired clock phase value if it is in the valid range. */
    if ((sipNo < CY_LVDS_MAX_PHY_INSTANCE) && (clkPhase < 16)) {
        glLvcmosClkPhaseSel[sipNo] = clkPhase;
    }
}
#endif /* (!LVCMOS_16BIT_SDR) */

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
void Cy_LVDS_L3_Entry (LVDSSS_LVDS_Type *base,
                       cy_stc_lvds_context_t *lvdsContext,
                       uint8_t sipNo)
{
    /*
       DLL_M_CONFIG.EN =0
       PLL_CONFIG.PLL_EN =0
       PLL_CONFIG.PLL_SUPPLY_EN = 0
       REG_1P25.ENABLE = 0
       REG_1P25.USE_REG = 0
       LICIO_VSSIO_IREF.EN = 0
       LICIO_VCCD_V1P1.EN = 0
     */

    /*
       a) Disables all thread's by programming
          "GPIF_THREAD_CONFIG.ENABLE_THREAD_CTLR" to 0
       b) Suspend the sockets
       c) Disables DLL first, followed by PLL
       d) Asserts active reset
          Programming soft reset bits (LVDS_CTL.LINK_ENABLE, LVDS_CTL.PHY_ENABLE) to 0
     */

    if ((base == NULL) || (lvdsContext == NULL)) {
        DBG_LVDS_ERR("L3_Entry: Invalid parameters\r\n");
        return;
    }

    /*  Mark the GPIF state machine configuration invalid. */
    base->GPIF[sipNo].GPIF_WAVEFORM_CTRL_STAT &= ~LVDSSS_LVDS_GPIF_GPIF_WAVEFORM_CTRL_STAT_WAVEFORM_VALID_Msk;

    /* Disable all threads. */
    if (sipNo == 0)
    {
        base->THREAD[0].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
        base->THREAD[1].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
    }

#if (!LVCMOS_16BIT_SDR)
    if ((lvdsContext->phyConfigP0->wideLink == true) || (sipNo == 1))
    {
        base->THREAD[2].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
        base->THREAD[3].GPIF_THREAD_CONFIG &= ~LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR_Msk;
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    base->AFE[sipNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN_Msk;

    base->AFE[sipNo].DLL_M_CONFIG &=  ~LVDSSS_LVDS_AFE_DLL_M_CONFIG_EN_Msk;
    base->AFE[sipNo].PLL_CONFIG   &=  ~LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN_Msk;
    Cy_SysLib_DelayUs(2);
    base->AFE[sipNo].PLL_CONFIG   &=  ~LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN_Msk;

    base->AFE[sipNo].REG_1P25  &= ~(LVDSSS_LVDS_AFE_REG_1P25_ENABLE_Msk | LVDSSS_LVDS_AFE_REG_1P25_USE_REG_Msk);

    base->AFE[sipNo].LICIO_VSSIO_IREF      &= ~LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN_Msk;
    base->AFE[sipNo].LICIO_VCCD_V1P1       &= ~LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN_Msk;

    /* Disable the LINK and AFE blocks once L3 entry has been processed on all active ports. */
    if (((sipNo == 0) && (lvdsContext->phyConfigP0->wideLink != true)) || (sipNo == 1))
    {
        base->CTL &= ~(LVDSSS_LVDS_CTL_LINK_ENABLED_Msk | LVDSSS_LVDS_CTL_PHY_ENABLED_Msk);
    }

    DBG_LVDS_INFO("L3_Entry Done\r\n");
}

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
cy_en_lvds_status_t Cy_LVDS_L3_Exit (LVDSSS_LVDS_Type *base,
                                     cy_stc_lvds_context_t *lvdsContext,
                                     uint8_t sipNo)
{
    cy_stc_lvds_phy_config_t *pPhyConfig = NULL;

    /*
       LICIO_VCCD_V1P1.EN = 1
       LICIO_VSSIO_IREF.EN = 1
       REG_1P25.ENABLE = 1
       PLL_CONFIG.PLL_SUPPLY_EN = 1
       Wait for 8usec
       PLL_CONFIG.PLL_EN = 1
       Wait for PLL Lock
       Poll PLL_STATUS.PLL_LOCK for 1
       DLL_M_CONFIG.EN = 1
       Wait for DLL Lock
       Poll DLL_M_STATUS.DLL_LOCK for 1
       PHY_TRAIN_CONFIG.TRAIN_EN = 1
     */

    /*
       a) de-assert active reset
          Programming soft reset bits (LVDS_CTL.LINK_ENABLE, LVDS_CTL.PHY_ENABLE) to 1.
       b) Enable sockets
       c) Enable threads
       d) Enable clocks (enabling PLL and DLL)
       e) Re-initiates link training
       f) Enables the GPIF_WAVEFORM_SWITCH.SWITCH_NOW
     */
    uint32_t timeoutCount = 0;
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;

    if ((base == NULL) || (lvdsContext == NULL))
    {
        DBG_LVDS_ERR("L3_Exit: Invalid parameters\r\n");
        return CY_LVDS_BAD_PARAMETER;
    }

    if (sipNo == 0)
    {
        pPhyConfig = lvdsContext->phyConfigP0;
    }
    else
    {
        pPhyConfig = lvdsContext->phyConfigP1;
    }

    /* Enable IP block */
    base->CTL |= _BOOL2FLD(LVDSSS_LVDS_CTL_PHY_ENABLED,  1u) |
                    _BOOL2FLD(LVDSSS_LVDS_CTL_LINK_ENABLED, 1u);

    base->AFE[sipNo].LICIO_VCCD_V1P1  |= _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VCCD_V1P1_EN, 1u);
    base->AFE[sipNo].LICIO_VSSIO_IREF |= _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_VSSIO_IREF_EN, 1u);
    Cy_SysLib_DelayUs(2);

    base->AFE[sipNo].REG_1P25         |= _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_ENABLE, 1u) |
                                         _BOOL2FLD(LVDSSS_LVDS_AFE_REG_1P25_USE_REG, 1u);
    Cy_SysLib_DelayUs(2);

    base->AFE[sipNo].PLL_CONFIG       |= _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_SUPPLY_EN, 1u);
    Cy_SysLib_DelayUs(8);
    base->AFE[sipNo].PLL_CONFIG       |= _BOOL2FLD(LVDSSS_LVDS_AFE_PLL_CONFIG_PLL_EN, 1u);

#if (!LVCMOS_16BIT_SDR)
    if (pPhyConfig->modeSelect != CY_LVDS_PHY_MODE_LVCMOS)
    {
        timeoutCount = 0;

        while(!(base->AFE[sipNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_ACQUIRED_Msk))
        {
            Cy_SysLib_DelayUs(10);
            timeoutCount++;
            if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
            {
                status = CY_LVDS_TIMEOUT_ERROR;
                DBG_LVDS_ERR("L3_Exit: PLL lock timed out for Port: %d\r\n", sipNo);
                return status;
            }
        }

        base->AFE[sipNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_PLL_LOCK_ACQUIRED_Msk;
        status = CY_LVDS_SUCCESS;
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    base->AFE[sipNo].DLL_M_CONFIG |= _BOOL2FLD(LVDSSS_LVDS_AFE_DLL_M_CONFIG_EN, 1u);
    timeoutCount = 0;

    while(!(base->AFE[sipNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk))
    {
        Cy_SysLib_DelayUs(10);
        timeoutCount++;
        if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
        {
            status = CY_LVDS_TIMEOUT_ERROR;
            DBG_LVDS_ERR("L3_Exit: LVDS DLL lock timed out for Port: %d\r\n", sipNo);
            return status;
        }
    }

    base->AFE[sipNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_MDLL_LOCK_ACQUIRED_Msk;

    /* In Case of LVCMOS external phyTrainingPattern not Required */
    if (pPhyConfig->modeSelect == CY_LVDS_PHY_MODE_LVCMOS)
    {
        base->AFE[sipNo].PHY_TRAIN_CONFIG   |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_EN, 1u);
        base->AFE[sipNo].PHY_GENERAL_CONFIG |= _VAL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_DESKEW_ALGORITHM, CY_LVDS_PHY_DESKEW_SLOW);

        timeoutCount = 0;
        while (!(base->AFE[sipNo].PHY_INTR & LVDSSS_LVDS_AFE_PHY_INTR_DESKEW_COMPLETED_Msk))
        {
            Cy_SysLib_DelayUs(10);
            timeoutCount++;
            if (timeoutCount >= CY_LVDS_MAX_TIMEOUT_COUNT)
            {
                status = CY_LVDS_TIMEOUT_ERROR;
                DBG_LVDS_ERR("L3_Exit: DDR DESKEW timed out for Port: %d\r\n", sipNo);
                return status;
            }
        }

        base->AFE[sipNo].PHY_INTR = LVDSSS_LVDS_AFE_PHY_INTR_DESKEW_COMPLETED_Msk;
        status = CY_LVDS_SUCCESS;
    }
#if (!LVCMOS_16BIT_SDR)
    else  /* In case of LVDS external phyTrainingPattern required */
    {

        base->AFE[sipNo].PHY_TRAIN_CONFIG &= ~LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_SEQ_Msk;
        base->AFE[sipNo].PHY_TRAIN_CONFIG  = _VAL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_SEQ, pPhyConfig->phyTrainingPattern) |
                                             _VAL2FLD(LVDSSS_LVDS_AFE_PHY_TRAIN_CONFIG_TRAIN_CYCLES_NO, 127);
    }
#endif /* (!LVCMOS_16BIT_SDR) */

    /* Enable Thread in case of LVCMOS */
    if (pPhyConfig->modeSelect == CY_LVDS_PHY_MODE_LVCMOS)
    {
        if (sipNo == 0)
        {
            base->THREAD[0].GPIF_THREAD_CONFIG |= _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 1u);
            base->THREAD[1].GPIF_THREAD_CONFIG |= _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 1u);
        }
#if (!LVCMOS_16BIT_SDR)
        if ((lvdsContext->phyConfigP0->wideLink == true) || (sipNo == 1))
        {
            base->THREAD[2].GPIF_THREAD_CONFIG |= _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 1u);
            base->THREAD[3].GPIF_THREAD_CONFIG |= _BOOL2FLD(LVDSSS_LVDS_THREAD_GPIF_THREAD_CONFIG_ENABLE_THREAD_CTLR, 1u);
        }
#endif /* (!LVCMOS_16BIT_SDR) */
    }

    return status;
}

#if LVCMOS_16BIT_SDR
/*******************************************************************************
* Function Name: Cy_LVDS_ClkOutEnable
****************************************************************************//**
*
* Enables clock out. Cy_LVDS_Enable should be called after Cy_LVDS_ClkOutEnable.
*
* \param base
* Pointer to the LVDS register base address
*
* \param clkOutFreq
* Clock out frequency
*
* \return cy_en_lvds_status_t
* CY_LVDS_BAD_PARAMETER - If the arguments are incorrect/invalid or LVDS is already enabled
* CY_LVDS_SUCCESS - If the operation is successful
*
*******************************************************************************/
cy_en_lvds_status_t Cy_LVDS_ClkOutEnable(LVDSSS_LVDS_Type * base,  cy_en_lvds_phy_interface_clockOut_t clkOutFreq)
{
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;
    uint8_t clkDiv = 0;
    uint8_t clkoutEn = 1;
    uint32_t index = 0;

    if(base == NULL)
    {
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Check if LVDS AFE is already enabled */
    if(base->CTL & LVDSSS_LVDS_CTL_LINK_ENABLED_Msk)
    {
        DBG_LVDS_ERR(" AFE already enabled\n\r");
        return CY_LVDS_LINK_ALREADY_ENABLED;
    }


    switch(clkOutFreq)
    {
        case CY_LVDS_PHY_INTERFACE_CLK_OUT_48_MHZ:
            clkDiv = 10;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_OUT_24_MHZ:
            clkDiv = 20;
            break;
        case CY_LVDS_PHY_INTERFACE_CLK_OUT_12_MHZ:
            clkDiv = 40;
            break;
        default:
            DBG_LVDS_ERR(" Cy_LVDS_ClkOutEnable: Invalid frequency \n\r");
            return CY_LVDS_BAD_PARAMETER;
            break;
    }

    /* Initialize and enable the LINK */
    if (Cy_SysLib_GetDeviceRevision() == 0x11)
    {
        base->LINK_CONFIG[clkoutEn] =
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LVDS_MODE,          CY_LVDS_PHY_MODE_LVCMOS) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_GEARING_RATIO,      CY_LVDS_PHY_GEAR_RATIO_1_1) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_WIDE_LINK,         0) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_NUM_LANES,          CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF_A0, false) |
            0x100UL;
    }
    else
    {
        /* Active device revision is B0. */
        base->LINK_CONFIG[clkoutEn] =
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_LVDS_MODE,       CY_LVDS_PHY_MODE_LVCMOS) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_GEARING_RATIO,   CY_LVDS_PHY_GEAR_RATIO_1_1) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_WIDE_LINK,      0) |
            _VAL2FLD(LVDSSS_LVDS_LINK_CONFIG_NUM_LANES,       CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_TWO_BIT_SLV_FF, false) |
            _BOOL2FLD(LVDSSS_LVDS_LINK_CONFIG_LINK_ENABLE,    1u);
    }

    base->GPIF_CLK_SEL[clkoutEn] =  _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_GPIF_CLK_SRC,CY_LVDS_GPIF_CLOCK_LVCMOS_IF) |
        _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_USB_CLK_DIV_VAL, CY_LVDS_GPIF_CLOCK_DIV_4) |
        _BOOL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_GATE_WAVEFORM_MEM_CLK, 1) |
        _BOOL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_LVCMOS_IF_CLK_100MHZ, true) |
        _VAL2FLD(LVDSSS_LVDS_GPIF_CLK_SEL_WAVEFORM_RAM_CG_DURATION, 6);


    /* LVDS IP and PHY should be enabled before AFE initialization */
    base->CTL    =  _BOOL2FLD(LVDSSS_LVDS_CTL_PHY_ENABLED,  1u) |
        _BOOL2FLD(LVDSSS_LVDS_CTL_IP_ENABLED,   1u);

    /*********** AFE Initialization sequence for LVCMOS SDR    ************/
    base->AFE[clkoutEn].PHY_ADC_CONFIG            |= _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_ADC_CONFIG_ISO_N, 1u);

    base->AFE[clkoutEn].PHY_GENERAL_CONFIG        |=  _BOOL2FLD(LVDSSS_LVDS_AFE_PHY_GENERAL_CONFIG_ENABLE_VDDIO, 1u);


    base->AFE[clkoutEn].GENERAL_RX_LVCMOS         |=  _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_CLK_MODE, 0u) |
        _BOOL2FLD(LVDSSS_LVDS_AFE_GENERAL_RX_LVCMOS_MUX_SEL, 1u);

    base->AFE[clkoutEn].LICIO_CIO_CTRL_DS         |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_CTRL_DS_DS, 0x1Fu);

    for(index = 0; index < 17; index++)
    {
        base->AFE[clkoutEn].LICIO_CIO_DATA_DS[index]  |=   _VAL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_DATA_DS_DS, 0x1Fu);
    }

    /* SDR clock pad (P1D1P) */
    base->AFE[clkoutEn].LICIO_CIO[9]   |=   _BOOL2FLD(LVDSSS_LVDS_AFE_LICIO_CIO_LVCMOS_RX_EN, 0x1u);

    base->GPIF[clkoutEn].GPIF_CONFIG =  (uint32_t)(1u << LVDSSS_LVDS_GPIF_GPIF_CONFIG_CLK_SOURCE_Pos);
    base->GPIF[clkoutEn].LVCMOS_CLK_OUT_CFG = (uint32_t)(
            (clkDiv << LVDSSS_LVDS_GPIF_LVCMOS_CLK_OUT_CFG_CLK_SRC_DIV_VAL_Pos) &
            LVDSSS_LVDS_GPIF_LVCMOS_CLK_OUT_CFG_CLK_SRC_DIV_VAL_Msk);

    return status;
}
#endif /* LVCMOS_16BIT_SDR */

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS */

/* [] END OF FILE */
