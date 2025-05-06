/***************************************************************************//**
* \file cy_usbss_cal_drv.c
* \version 1.0
*
* Driver implementation for the USBSS (3.x) block in the FX device family.
* The driver functions are meant to be called by the USB device stack (USBD)
* and not expected to be called directly by user code.
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
#ifndef FX2G3_EN

#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_debug.h"
#include "cy_gpio.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Set this to one to enable the use of Pipe Override setting to delay valid TS1 transmission
 * during USB Gen2 U2 and U3 exit.
 */
#define ENABLE_TS1_DELAY_IN_UX_EXIT                   (0)

#define CY_USBSS_PROT_LMP_PORT_CAP_TIMER_VALUE        (0x00000908)  /* 18.5 us. */
#define CY_USBSS_PROT_LMP_PORT_CFG_TIMER_VALUE        (0x00000908)  /* 18.5 us. */
static volatile uint32_t gClockMultiplier = 75;

#define CY_USBSS_U1_EXIT_LFPS_DURATION                (120UL)       /* 120 us */
#define CY_USBSS_MAX_U1_WAKE_LFPS_DURATION            (100UL)       /* 100 us */

#define CY_USBSS_U2_EXIT_LFPS_DURATION                (800UL)       /* 800 us */
#define CY_USBSS_MAX_U2_WAKE_LFPS_DURATION            (850UL)       /* 850 us */

#define CY_USBSS_U3_EXIT_LFPS_DURATION                (2100UL)      /* 2100 us = 2.10 ms */
#define CY_USBSS_MAX_U3_WAKE_LFPS_DURATION            (1900UL)      /* 1900 us = 1.90 ms */

#define CY_USBSS_LFPS_PERIOD_TO_REG_G1(burst_us)        ((burst_us) * 125UL)
#define CY_USBSS_LFPS_PERIOD_TO_REG_G2(burst_us)        ((burst_us) * 312UL)

/* Calculate CDR_SW_CTRL register value given the L2R and L2D delay values in us. */
#define CY_FX10_USBSS_CDR_SW_CTRL_VAL(l2r_del, l2d_del_us)                                            \
    (((l2r_del) << USB32DEV_PHYSS_USB40PHY_TOP_CDR_SW_CTRL_L2R_START_DELAY_Pos) |                     \
     ((l2d_del_us * gClockMultiplier) << USB32DEV_PHYSS_USB40PHY_TOP_CDR_SW_CTRL_L2D_START_DELAY_Pos) \
     )

static cy_en_usb_cal_ret_code_t Cy_USBSS_Phy_PllInit
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint8_t laneId);

static cy_en_usb_cal_ret_code_t Cy_USBSS_Phy_RxInit
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint8_t laneId,
                                         bool isPowerOn);

static cy_en_usb_cal_ret_code_t Cy_USBSS_Phy_TxInit
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         uint8_t laneId);

static void Cy_USBSS_Cal_UpdateDmaClkFreq
                                        (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                         bool pclk_enabled);

static void SSPhy_Update_TxSettings (USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs,
                                     uint32_t *pTxSettings);

/* Compile out all TRACE statements. */
#undef DBG_SSCAL_TRACE
#define DBG_SSCAL_TRACE(str, ...)

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
Cy_USBSS_Cal_MeasureCCVoltage (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool cc2select)
{
    USB32DEV_PHYSS_USB40PHY_TOP_Type *phyTopRegs;
    uint8_t ccVoltageReading = 0;
    bool    phyEnabled = false;

    if (pCalCtxt == NULL) {
        DBG_SSCAL_WARN("MeasureCCVoltage: SSCalCtxt=NULL\r\n");
        return ccVoltageReading;
    }

    phyTopRegs = &(pCalCtxt->regBase->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP);

    /* Enable the PHY if it is not enabled. */
    if ((phyTopRegs->TOP_CTRL_0 & USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk) == 0) {
        phyTopRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;
    } else {
        phyEnabled = true;
    }

    /* Basic ADC configuration: Using VDDIO as reference and measuring active CC pin. */
    if (cc2select) {
        phyTopRegs->ADC  = (
                (phyTopRegs->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
                (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VREF_DAC_SEL_Pos) |
                (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos) |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk);
    } else {
        phyTopRegs->ADC  = (
                (phyTopRegs->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
                (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VREF_DAC_SEL_Pos) |
                (0x00UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos) |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk);
    }

    /* Enable ADC conversion. */
    phyTopRegs->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;

    /* Wait for conversion to complete and get the measured voltage. */
    Cy_SysLib_DelayUs(50);

    ccVoltageReading = (
            (phyTopRegs->ADC_STATUS & USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Msk) >>
            USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Pos);
    ccVoltageReading &= 0x7FUL;

    if (!phyEnabled) {
        /* Disable the PHY if we had enabled it for measurement. */
        phyTopRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;
    }

    DBG_SSCAL_TRACE("CC ADC reading is %x\r\n", ccVoltageReading);
    return (ccVoltageReading);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetBusOrientation
****************************************************************************//**
*
* Identifies the USB-C connection orientation for the present
* connection.
*
* \param pCalCtxt
* USBSS Controller context structure.
*
* \return
* Index of SSPHY to be used for connection.
*******************************************************************************/
uint8_t
Cy_USBSS_Cal_GetBusOrientation (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_Type     *phyRegs    = &(base->USB32DEV_PHYSS.USB40PHY[0]);
    USB32DEV_PHYSS_USB40PHY_TOP_Type *phyTopRegs = &(phyRegs->USB40PHY_TOP);
    uint32_t cc0Voltage, cc1Voltage;
    uint8_t phyIdx = 0;

    /* Enable the 0th PHY which is used for measurement. */
    phyTopRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;

    /* Basic ADC configuration: Using VDDIO as reference and measuring CC1 pin. */
    phyTopRegs->ADC  = (
            (phyTopRegs->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VREF_DAC_SEL_Pos) |
            (0x00UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk);

    /* Enable ADC conversion. */
    phyTopRegs->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;

    /* Wait for conversion to complete and save the measured voltage. */
    Cy_SysLib_DelayUs(50);
    DBG_SSCAL_TRACE("ADC=%x ADCSTAT=%x\r\n", phyTopRegs->ADC, phyTopRegs->ADC_STATUS);
    cc0Voltage = (
            (phyTopRegs->ADC_STATUS & USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Msk) >>
            USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Pos);
    cc0Voltage &= 0x7FUL;

    /* Basic ADC configuration: Using VDDIO as reference and measuring CC2 pin. */
    phyTopRegs->ADC  = (
            (phyTopRegs->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VREF_DAC_SEL_Pos) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk);

    /* Enable ADC conversion. */
    phyTopRegs->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;

    /* Wait for conversion to complete and save the measured voltage. */
    Cy_SysLib_DelayUs(50);
    DBG_SSCAL_TRACE("ADC=%x ADCSTAT=%x\r\n", phyTopRegs->ADC, phyTopRegs->ADC_STATUS);
    cc1Voltage = (
            (phyTopRegs->ADC_STATUS & USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Msk) >>
            USB32DEV_PHYSS_USB40PHY_TOP_ADC_STATUS_SAR_OUT_Pos);
    cc1Voltage &= 0x7FUL;

    DBG_SSCAL_INFO("CC0 voltage: %x, CC1 voltage: %x\r\n", cc0Voltage, cc1Voltage);

    /* Applying simplified logic to detect CC polarity.
     * If voltage on CC0 is less than ~0.13 V and voltage on CC1 is higher than ~0.13 V;
     * go ahead with CC1 polarity. Otherwise, continue with CC0 polarity.
     */
    if ((cc1Voltage >= 10) && (cc0Voltage < 10)) {
        DBG_SSCAL_INFO("Going ahead with PHY1\r\n");
        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PHY1_SELECT);
        phyIdx = 1;
    } else {
        DBG_SSCAL_INFO("Going ahead with PHY0\r\n");
        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PHY0_SELECT);
    }

    /* Disable the PHY used for measurement. */
    phyTopRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;

    return (phyIdx);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_InitSSPhy
****************************************************************************//**
*
* Initializes the specified USB40 PHY instance.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param numLanes
* Number of USB lanes to be initialized.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_InitSSPhy (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t numLanes)
{
    /* If numLanes = 1, initialize the active lane
     * If numLanes > 1, initialize the active lane first, then then the alternate lane.
     */
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_Type     *phyRegs;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *phyTopRegs;

    uint8_t startPhyIdx = pCalCtxt->activePhyIdx;
    uint8_t phyId = 0;
    uint8_t iter = 0;
    uint8_t i = 0;
    pCalCtxt->numPhyInitialized  = 0;

    for (iter = 0, phyId = startPhyIdx; iter < numLanes; iter++, phyId = !phyId)
    {
        DBG_SSCAL_INFO("PhyInit start:%d\r\n",phyId);
        phyRegs = &(base->USB32DEV_PHYSS.USB40PHY[phyId]);
        phyTopRegs = &(phyRegs->USB40PHY_TOP);

        for (i = 0; i < 16; i++) {
            pCalCtxt->phyEyeHeight[phyId][i] = 0;
            pCalCtxt->phyResult[phyId][i] = 0;
        }

        /* Enable PHYSS */
        phyTopRegs->TOP_CTRL_0 |= (
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk);

        /* Update the LFPS clock frequency to a target of 25 MHz (40 ns period). */
        if (Cy_SysClk_ClkFastGetFrequency() == 150000000UL) {
            phyTopRegs->TOP_CTRL_0 = (
                    (phyTopRegs->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_LFPS_CLK_DIV_Msk) |
                    (5UL << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_LFPS_CLK_DIV_Pos));
        } else {
            phyTopRegs->TOP_CTRL_0 = (
                    (phyTopRegs->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_LFPS_CLK_DIV_Msk) |
                    (3UL << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_LFPS_CLK_DIV_Pos));
        }

        /* Make sure that PHY0 is enabled as well. */
        if (numLanes != 2 && phyId != 0) {
            base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 = phyTopRegs->TOP_CTRL_0;
        }

        phyTopRegs->TX_AFE_RXDETECT_CNT = (
                (50UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_RXDETECT_CNT_RXDETECT_WAIT_CNT_Pos) |
                (63UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_RXDETECT_CNT_PULLUP_ASSERT_DLY_Pos));
    }


    for (iter = 0, phyId = startPhyIdx; iter < numLanes; iter++, phyId = !phyId)
    {
        /* Initialize the USB40PHY TX block */
        (void)Cy_USBSS_Phy_TxInit(pCalCtxt, phyId);
        DBG_SSCAL_TRACE("PhyTxInit done:%d\r\n",phyId);
    }


    /* PLL Init logic :
     * Configure PHY0 registers in all cases. Enable PHY1 also if active */
    if(numLanes == 1)
    {
        (void)Cy_USBSS_Phy_PllInit(pCalCtxt, startPhyIdx);
    }
    else
    {
        (void)Cy_USBSS_Phy_PllInit(pCalCtxt, 0);

        base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.INTR0 = 0xFFFFFFFFUL;
        base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
        base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 |= (
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk);
    }
    DBG_SSCAL_TRACE("PhyPllInit done\r\n");


    for (iter = 0, phyId = startPhyIdx; iter < numLanes; iter++, phyId = !phyId)
    {
        /* Initialize the USB40PHY RX block */
        (void)Cy_USBSS_Phy_RxInit(pCalCtxt, phyId, true);
        DBG_SSCAL_TRACE("PhyRX_Init done:%d\r\n",phyId);
    }

    for(iter = 0, phyId = startPhyIdx; iter < numLanes; iter++, phyId = !phyId)
    {
        phyRegs = &(base->USB32DEV_PHYSS.USB40PHY[phyId]);
        phyTopRegs = &(phyRegs->USB40PHY_TOP);

        /* Release PLL from reset, enable PCLK and set Power Present. */
        phyTopRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
        Cy_SysLib_DelayUs(1);
        phyTopRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
        Cy_SysLib_DelayUs(1);
        phyTopRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk;

        /* If PHY0 is not the active PHY in single-lane use-case, make appropriate writes to PHY0 regs also */
        if (numLanes != 2 && phyId != 0) {
            base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 |=
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
            Cy_SysLib_DelayUs(1);
            base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 |=
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
            Cy_SysLib_DelayUs(1);
            base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 |=
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk;
        }

        phyTopRegs->PCS_CTRL_0 |= (
                (7UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_CTRL_0_TX_SER_EN_DLY_Pos));

        /* Enable filtering of LFPS for 8 PERI clock cycles or 106.67 ns. */
        phyTopRegs->LOW_POWER_CTRL = (
                (8UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_Pos) |
                (15UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_HW_CTRL_DELAY_Pos) |
                USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_MASK_ON_TX_DATA_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_MODE_Msk);

        /* Setting L2D start delay corresponding to U1 exit case. */
        pCalCtxt->lpmExitLfpsDelay = CY_USBSS_U1_EXIT_LFPS_DURATION - 50U;
        phyTopRegs->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U1_WAKE_LFPS_DURATION);

        phyTopRegs->PIPE_RX_CTRL |= (
                (15UL << USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_HS_ERROR_DEBOUNCE_Pos) |
                (4UL << USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_HS_DATA_ALIGN_Pos));

        pCalCtxt->numPhyInitialized++;
    }
    return CY_USB_CAL_STATUS_SUCCESS;

}


/*******************************************************************************
 * Function Name: Cy_USBSS_Cal_PreConnect
 ****************************************************************************//**
 *
 * Pre-connection initialization for the USB 3.x controller block.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
 * allocated by the user.
 *
 *******************************************************************************/
static void
Cy_USBSS_Cal_PreConnect (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PROT_Type *USB32DEV_PROT = &base->USB32DEV_PROT;
    USB32DEV_LNK_Type  *USB32DEV_LNK  = &base->USB32DEV_LNK;

    USB32DEV_LNK->LNK_CONF = (USB32DEV_LNK_CONF_LCW_IGNORE_RSVD_Msk |
            USB32DEV_LNK_CONF_DEBUG_FEATURE_ENABLE_Msk |
            USB32DEV_LNK_CONF_FORCE_POWER_PRESENT_Msk |
            USB32DEV_LNK_CONF_LDN_DETECTION_Msk |
            (7UL << USB32DEV_LNK_CONF_EPM_FIRST_DELAY_Pos) |
            USB32DEV_LNK_CONF_RX_DPP_ERR_GO_RECOVERY_EN_Msk);

    /* Setting RXEQ_TRAINING_OVR=0, Elasticity Buffer to HALF-FULL configuration */
    USB32DEV_LNK->LNK_PHY_CONF = (
            (0x01UL << USB32DEV_LNK_PHY_CONF_PHY_MODE_Pos) |
            (0x00UL << USB32DEV_LNK_PHY_CONF_ELASTICIY_BUFFER_MODE_Pos)
            );

    /* Enable sequence number matching on Egress ISO transfers. */
    USB32DEV_PROT->PROT_ENHANCE |= (USB32DEV_PROT_ENHANCE_EN_EGRS_ISO_SEQ_MATCH_Msk);

    /* Flush and disable all EPs except EP0 */
    Cy_USBSS_Cal_FlushAllEndpSocket(pCalCtxt);
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_Init (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                   void *pUsbdCtxt,
                   cy_usb_cal_msg_callback_t cb)
{
    cy_en_usb_cal_ret_code_t retCode = CY_USB_CAL_STATUS_SUCCESS;
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PROT_Type *USB32DEV_PROT = &base->USB32DEV_PROT;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_LNK_Type  *USB32DEV_LNK  = &base->USB32DEV_LNK;

    pCalCtxt->pUsbdCtxt = pUsbdCtxt;
    pCalCtxt->forceLPMAccept = false;
    pCalCtxt->sofEvtEnable = false;
    pCalCtxt->msgCb = cb;
    pCalCtxt->pMainBase = USB32DEV_MAIN;
    pCalCtxt->pLinkBase = USB32DEV_LNK;
    pCalCtxt->pProtBase = USB32DEV_PROT;
    pCalCtxt->pEpmBase = &(base->USB32DEV_EPM);
    pCalCtxt->connectRcvd = false;
    pCalCtxt->deepSleepEntered = false;
    pCalCtxt->devLpmExitDisabled = false;
    pCalCtxt->lbadCounter = 0;
    pCalCtxt->stopClkOnEpResetEnable = false;
    pCalCtxt->usbConfigLane = CY_USB_CFG_LANE_AUTODETECT;

    pCalCtxt->desiredDmaFreq = CY_HBDMA_CLK_240_MHZ;
    pCalCtxt->currentDmaFreq = CY_HBDMA_CLK_150_MHZ;
    Cy_HBDma_SetClockFrequency(pCalCtxt->currentDmaFreq);

    /* Enable the USBSS IP */
    USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;

    /* Leave all block interrupts disabled. */
    USB32DEV_LNK->LNK_ERROR_CONF = 0xFFFFFFFFUL;
    USB32DEV_LNK->LNK_INTR       = 0xFFFFFFFFUL;
    USB32DEV_LNK->LNK_INTR_MASK  = 0x00000000UL;

    USB32DEV_PROT->PROT_EP_INTR_MASK = 0x00000000UL;
    USB32DEV_PROT->PROT_INTR         = 0xFFFFFFFFUL;
    USB32DEV_PROT->PROT_INTR_MASK    = 0x00000000UL;

    USB32DEV_MAIN->INTR_MASK = 0x00000000UL;

    Cy_USBSS_Cal_PreConnect(pCalCtxt);
    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_DRV_EN);

    return retCode;
}


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
* \return
* void
*******************************************************************************/
void
Cy_USBSS_Cal_SetGen2EBDepth (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint8_t   gen2_ebdepth)
{
    if ((pCalCtxt != NULL) && (gen2_ebdepth >= 4u) && (gen2_ebdepth <= 16u)) {
        DBG_SSCAL_INFO("Setting Gen2 elastic buffer half depth = %d\r\n",
                gen2_ebdepth);
        pCalCtxt->gen2_ebdepth = gen2_ebdepth;
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_InitEventLog
****************************************************************************//**
*
* Enables capture of USBSS CAL event and state changes into RAM buffer.
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
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_InitEventLog (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint32_t *pEvtLogBuf,
        uint16_t  evtLogSize)
{
    if (pCalCtxt == NULL) {
        return CY_USB_CAL_STATUS_CAL_CTXT_NULL;
    }

    if (pEvtLogBuf != NULL) {
        /* Enable interrupts on all link state changes. */
        pCalCtxt->pLinkBase->LNK_LTSSM_STATE_CHG_INTR_CONF = 0x05FFEFFFUL;
    } else {
        /* Enable only the required link state change interrupts. */
        pCalCtxt->pLinkBase->LNK_LTSSM_STATE_CHG_INTR_CONF = (
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_SS_DISABLE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_RX_DETECT_ACTIVE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_SS_INACTIVE_QUIET_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_SS_INACTIVE_DISCONNECT_DETECT_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_POLLING_LFPS_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_POLLING_LFPSPLUS_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_POLLING_RXEQ_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_POLLING_ACTIVE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_POLLING_CONFIGURATION_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_U0_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_U1_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_U2_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_U3_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_LOOPBACK_ACTIVE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_COMPLIANCE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_RECOVERY_ACTIVE_INTR_Msk |
            USB32DEV_LNK_LTSSM_STATE_CHG_INTR_CONF_ILLEGAL_STATE_INTR_Msk);
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

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
bool Cy_USBSS_Cal_IsEnabled (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type  *USB32DEV_MAIN = &base->USB32DEV_MAIN;

    return ((bool)((USB32DEV_MAIN->CTRL & USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk) != 0));
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_Connect (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      cy_en_usb_speed_t usbSpeed)
{
    USB32DEV_Type      *base = pCalCtxt->regBase;
    USB32DEV_PROT_Type *USB32DEV_PROT = &base->USB32DEV_PROT;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP_1;

    /* Pre-connect initialization of all endpoints. */
    Cy_USBSS_Cal_PreConnect(pCalCtxt);

    /* Force the start-up speed based on features supported by the device. */
    switch (usbSpeed) {
        case CY_USBD_USB_DEV_SS_GEN2X2:
            /* Confirm that device can actually support GEN2X2 operation. */
            if ((UsbBlkInitConfig & 0x00010000UL) == 0x00010000UL) {
                DBG_SSCAL_WARN("Device does not support GEN2X2 operation: Dropping to GEN1X1 speed\r\n");
                usbSpeed = CY_USBD_USB_DEV_SS_GEN1;
            } else {
                if ((UsbBlkInitConfig & 0x00020000UL) == 0x00020000UL) {
                    DBG_SSCAL_WARN("Device does not support GEN2X2 operation: Dropping to GEN1X2 speed\r\n");
                    usbSpeed = CY_USBD_USB_DEV_SS_GEN1X2;
                } else {
                    if ((UsbBlkInitConfig & 0x00040000UL) == 0x00040000UL) {
                        DBG_SSCAL_WARN("Device does not support GEN2X2 operation: Dropping to GEN2X1 speed\r\n");
                        usbSpeed = CY_USBD_USB_DEV_SS_GEN2;
                    }
                }
            }
            break;

        case CY_USBD_USB_DEV_SS_GEN2:
            /* Confirm that device can actually support GEN2X1 operation. */
            if ((UsbBlkInitConfig & 0x00010000UL) == 0x00010000UL) {
                DBG_SSCAL_WARN("Device does not support GEN2X1 operation: Dropping to GEN1X1 speed\r\n");
                usbSpeed = CY_USBD_USB_DEV_SS_GEN1;
            } else {
                if ((UsbBlkInitConfig & 0x00020000UL) == 0x00020000UL) {
                    DBG_SSCAL_WARN("Device does not support GEN2X1 operation: Dropping to GEN1X2 speed\r\n");
                    usbSpeed = CY_USBD_USB_DEV_SS_GEN1X2;
                }
            }
            break;

        case CY_USBD_USB_DEV_SS_GEN1X2:
            /* Confirm that device can actually support GEN1X2 operation. */
            if ((UsbBlkInitConfig & 0x00010000UL) == 0x00010000UL) {
                DBG_SSCAL_WARN("Device does not support GEN1X2 operation: Dropping to GEN1X1 speed\r\n");
                usbSpeed = CY_USBD_USB_DEV_SS_GEN1;
            }
            break;

        case CY_USBD_USB_DEV_SS_GEN1:
            /* Nothing to do here. */
            break;

        default:
            /* Invalid USB connection speed requested. */
            return CY_USB_CAL_STATUS_BAD_PARAM;
    }

    /* Make sure start-up USB speed is limited based on speed selected by user. */
    if ((usbSpeed == CY_USBD_USB_DEV_SS_GEN2) || (usbSpeed == CY_USBD_USB_DEV_SS_GEN1)) {
        if ((UsbBlkInitConfig & 0x00050000UL) == 0) {
            UsbBlkInitConfig |= 0x00040000UL;
        }
    } else {
        if ((UsbBlkInitConfig & 0x000F0000UL) == 0) {
            UsbBlkInitConfig |= 0x00080000UL;
        }
    }

    if ((usbSpeed == CY_USBD_USB_DEV_SS_GEN1X2) || (usbSpeed == CY_USBD_USB_DEV_SS_GEN2X2))
    {
        pCalCtxt->dualLaneEnabled = true;
    }
    else
    {
        pCalCtxt->dualLaneEnabled = false;
    }

    /* Identify the USB configuration lane. */
    switch (pCalCtxt->usbConfigLane) {
        case CY_USB_CFG_LANE_0:
            DBG_SSCAL_INFO("Selected USB PHY0 as config lane\r\n");
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PHY0_SELECT);
            pCalCtxt->activePhyIdx = 0;
            break;

        case CY_USB_CFG_LANE_1:
            DBG_SSCAL_INFO("Selected USB PHY1 as config lane\r\n");
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PHY1_SELECT);
            pCalCtxt->activePhyIdx = 1;
            break;

        case CY_USB_CFG_LANE_AUTODETECT:
        default:
            pCalCtxt->activePhyIdx = Cy_USBSS_Cal_GetBusOrientation(pCalCtxt);
            break;

    }

    pCalCtxt->connectRcvd = false;
    pCalCtxt->uxExitActive = false;
    pCalCtxt->u2TimeoutVal = 0;
    pCalCtxt->compExitDone = false;
    pCalCtxt->compExitTS   = 0;
    pCalCtxt->phyDisabled  = false;

    /* In a single lane configuration, make sure that we route signals from the active PHY to the DFT pins. */
    if (pCalCtxt->dualLaneEnabled == false) {
        if (pCalCtxt->activePhyIdx != 0) {
            if ((USB32DEV_MAIN->DDFT_MUX & 0x00FEUL) == 0) {
                USB32DEV_MAIN->DDFT_MUX |= 0x0002UL;
            }
            if ((USB32DEV_MAIN->DDFT_MUX & 0xFE00UL) == 0) {
                USB32DEV_MAIN->DDFT_MUX |= 0x0200UL;
            }
            if ((USB32DEV_MAIN->GPIO_DDFT_MUX & 0x00FEUL) == 0) {
                USB32DEV_MAIN->GPIO_DDFT_MUX |= 0x0002UL;
            }
            if ((USB32DEV_MAIN->GPIO_DDFT_MUX & 0xFE00UL) == 0) {
                USB32DEV_MAIN->GPIO_DDFT_MUX |= 0x0200UL;
            }
        } else {
            if ((USB32DEV_MAIN->DDFT_MUX & 0x00FEUL) == 0x0002UL) {
                USB32DEV_MAIN->DDFT_MUX &= ~0x0002UL;
            }
            if ((USB32DEV_MAIN->DDFT_MUX & 0xFE00UL) == 0x0200UL) {
                USB32DEV_MAIN->DDFT_MUX &= ~0x0200UL;
            }
            if ((USB32DEV_MAIN->GPIO_DDFT_MUX & 0x00FEUL) == 0x0002UL) {
                USB32DEV_MAIN->GPIO_DDFT_MUX &= ~0x0002UL;
            }
            if ((USB32DEV_MAIN->GPIO_DDFT_MUX & 0xFE00UL) == 0x0200UL) {
                USB32DEV_MAIN->GPIO_DDFT_MUX &= ~0x0200UL;
            }
        }
    }

    USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP;
    USB_PHY_TOP_1 = &base->USB32DEV_PHYSS.USB40PHY[!(pCalCtxt->activePhyIdx)].USB40PHY_TOP;

    USB32DEV_LNK->LNK_LFPS_OBSERVE = 0;

    /* Enable all USBSS interrupts at this stage. */
    USB32DEV_MAIN->INTR_MASK |= (USB32DEV_MAIN_INTR_LINK_Msk |
                                 USB32DEV_MAIN_INTR_PROT_Msk |
                                 USB32DEV_MAIN_INTR_PROT_EP_Msk |
                                 USB32DEV_MAIN_INTR_EPM_URUN_Msk |
                                 USB32DEV_MAIN_INTR_PHY0_Msk |
                                 USB32DEV_MAIN_INTR_PHY1_Msk);

    USB32DEV_LNK->LNK_INTR       = 0xFFFFFFFF;
    USB32DEV_LNK->LNK_INTR_MASK  = (USB32DEV_LNK_INTR_LGO_U3_Msk |
                                    USB32DEV_LNK_INTR_LTSSM_CONNECT_Msk |
                                    USB32DEV_LNK_INTR_LPMA_Msk |
                                    USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk |
                                    USB32DEV_LNK_INTR_LTSSM_RESET_Msk |
                                    USB32DEV_LNK_INTR_DATA_RATE_CHANGE_Msk |
                                    USB32DEV_LNK_INTR_LTSSM_U3_ENTRY_Msk |
                                    USB32DEV_LNK_INTR_LTSSM_STATE_CHG_Msk |
                                    USB32DEV_LNK_INTR_LBAD_Msk);

    USB32DEV_PROT->PROT_EP_INTR_MASK = 0UL;
    USB32DEV_PROT->PROT_INTR         = 0xFFFFFFFF;
    USB32DEV_PROT->PROT_INTR_MASK    = (USB32DEV_PROT_INTR_STATUS_STAGE_Msk |
                                        USB32DEV_PROT_INTR_SUTOK_EV_Msk |
                                        USB32DEV_PROT_INTR_TIMEOUT_PORT_CAP_EV_Msk |
                                        USB32DEV_PROT_INTR_TIMEOUT_PORT_CFG_EV_Msk |
                                        USB32DEV_PROT_INTR_LMP_RCV_EV_Msk |
                                        USB32DEV_PROT_INTR_LMP_PORT_CAP_EV_Msk |
                                        USB32DEV_PROT_INTR_LMP_PORT_CFG_EV_Msk |
                                        USB32DEV_PROT_INTR_SET_ADDR_Msk);
    /* Enable ITP Interrupt if configured to be enabled */
    if (pCalCtxt->sofEvtEnable == true)
    {
        USB32DEV_PROT->PROT_INTR_MASK |= USB32DEV_PROT_INTR_ITP_EV_Msk;
    }


    USB_PHY_TOP->INTR0 = 0xFFFFFFFFUL;
    USB_PHY_TOP->INTR0_MASK = USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_P0_CHANGE_Msk;

    gClockMultiplier = (Cy_SysClk_ClkPeriGetFrequency() / 1000000UL);

    /* Choose Gen2 mode where requested by caller. */
    if ((usbSpeed == CY_USBD_USB_DEV_SS_GEN2) || (usbSpeed == CY_USBD_USB_DEV_SS_GEN2X2)) {
        pCalCtxt->gen2Enabled = true;
        USB32DEV_LNK->LNK_DEBUG_RSVD &= ~USB32DEV_LNK_DEBUG_RSVD_FORCE_RATE_CONFIG_TO_GEN1_Msk;

        /* Set PHY data rate to Gen2. */
        USB_PHY_TOP->TOP_CTRL_0 = (
                (USB_PHY_TOP->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk) |
                (1UL << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos));
        /* Set PHY data rate to Gen2. */
        USB_PHY_TOP_1->TOP_CTRL_0 = (
                (USB_PHY_TOP_1->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk) |
                (1UL << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos));
    } else {
        pCalCtxt->gen2Enabled = false;

        USB32DEV_LNK->LNK_DEBUG_RSVD |= USB32DEV_LNK_DEBUG_RSVD_FORCE_RATE_CONFIG_TO_GEN1_Msk;

        /* Set PHY data rate to Gen1. */
        USB_PHY_TOP->TOP_CTRL_0 = (
                (USB_PHY_TOP->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk));
        USB_PHY_TOP_1->TOP_CTRL_0 = (
                (USB_PHY_TOP_1->TOP_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk));
    }

    /* Send fewer TSEQ ordered sets per lane so that DUT moves into Polling.Active state first. */
    USB32DEV_LNK->LNK_TSEQ_COUNT_GEN1 = 64000UL;
    USB32DEV_LNK->LNK_TSEQ_COUNT_GEN2 = 516000UL;

    /* Extend the valid tRepeat range of received Polling.LFPS bursts to [5.5us,15us] */
    USB32DEV_LNK->LNK_LFPS_RX_POLLING_REPEAT      = ((1875UL << 16UL) | 687UL);
    USB32DEV_LNK->LNK_LFPS_RX_POLLING_REPEAT_GEN2 = ((4688UL << 16UL) | 1718UL);

    /* Set the U2 exit timeout to 2 ms and U3 exit timeout to 10 ms based on USB spec. */
    USB32DEV_LNK->LNK_LTSSM_U2_EXIT_TIMEOUT = 250000UL;
    USB32DEV_LNK->LNK_LTSSM_U3_EXIT_TIMEOUT = 0;

    /* Set U1 exit LFPS duration as 120 us */
    USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT      = CY_USBSS_LFPS_PERIOD_TO_REG_G1(CY_USBSS_U1_EXIT_LFPS_DURATION);
    USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT_GEN2 = CY_USBSS_LFPS_PERIOD_TO_REG_G2(CY_USBSS_U1_EXIT_LFPS_DURATION);

    /* Increase U1 exit RX LFPS duration to 600 ns. */
    USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT           = 75;
    USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE      = 75;
    USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT_GEN2      = 187;
    USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE_GEN2 = 187;

    /* Restore default PendingHpTimer value. */
    USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT      = 1250UL;
    USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT_GEN2 = 3125UL;

    /* Increase U2 exit LFPS TX duration to 800 us from the default of 80 us. */
    USB32DEV_LNK->LNK_LFPS_TX_U2_EXIT      = CY_USBSS_LFPS_PERIOD_TO_REG_G1(CY_USBSS_U2_EXIT_LFPS_DURATION);
    USB32DEV_LNK->LNK_LFPS_TX_U2_EXIT_GEN2 = CY_USBSS_LFPS_PERIOD_TO_REG_G2(CY_USBSS_U2_EXIT_LFPS_DURATION);

    /* Increase U3 exit LFPS TX duration to 4 ms from the default of 80 us. */
    USB32DEV_LNK->LNK_LFPS_TX_U3_EXIT      = CY_USBSS_LFPS_PERIOD_TO_REG_G1(CY_USBSS_U3_EXIT_LFPS_DURATION);
    USB32DEV_LNK->LNK_LFPS_TX_U3_EXIT_GEN2 = CY_USBSS_LFPS_PERIOD_TO_REG_G2(CY_USBSS_U3_EXIT_LFPS_DURATION);

    /*
     * Increase U3 Exit LFPS duration to order of 1.8 ms.
     */
    USB32DEV_LNK->LNK_LFPS_RX_U3_EXIT      = CY_USBSS_LFPS_PERIOD_TO_REG_G1(1800UL);

    /* TD 7.01.5 fix: Reduce tPollingSCDLfpsTimeout value to 50 us from 60 us. */
    USB32DEV_LNK->LNK_LTSSM_SCD_LFPS_TIMEOUT = 6250UL;

    /* Inter-op workaround: Increase Polling.PortMatch and Polling.PortConfig timeout to 15 ms. */
    USB32DEV_LNK->LNK_LTSSM_LBPM_LFPS_TIMEOUT = 1875000UL;

    /* Set min IDLE count in recovery to 252. */
    USB32DEV_LNK->LNK_MISC_CONF = (
            (USB32DEV_LNK->LNK_MISC_CONF &
             ~(USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Msk | USB32DEV_LNK_MISC_CONF_TX_IDLE_CONF_Msk)) |
            (0x03UL << USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Pos) |
            (0x03UL << USB32DEV_LNK_MISC_CONF_TX_IDLE_CONF_Pos) |
            USB32DEV_LNK_MISC_CONF_INVALID_LFPS_NOT_RST_TIMER_EN_Msk);
    USB32DEV_LNK->LNK_MISC_CONF &= ~USB32DEV_LNK_MISC_CONF_TX_DL_SET_ENABLE_Msk;

    /*
     * It is possible that DUT is in U2 while host/hub is still in U1. So, allow U2 exit handshake
     * to be completed with a 640 us burst from upstream.
     */
    USB32DEV_LNK->LNK_LFPS_RX_U2_HANDSHAKE      = 80UL;
    USB32DEV_LNK->LNK_LFPS_RX_U2_HANDSHAKE_GEN2 = 200UL;

    /* Set Warm Reset LFPS detect duration to default. */
    USB32DEV_LNK->LNK_LFPS_RX_RESET_GEN2 = 25000000UL;

    /* Increase upper limit of RX Ping.LFPS burst to 240 ns to give margin during compliance testing. */
    USB32DEV_LNK->LNK_LFPS_RX_PING      = 0x001E0005UL;
    USB32DEV_LNK->LNK_LFPS_RX_PING_GEN2 = 0x004B000DUL;

    /*
     * Update Polling.LFPS burst duration to adjust for the ~120ns of LFPS filtering.
     * Also extending the max limit for GEN2 Polling.LFPS burst to get better margin.
     */
    USB32DEV_LNK->LNK_LFPS_RX_POLLING_BURST      = 0x00AF003CUL;
    USB32DEV_LNK->LNK_LFPS_RX_POLLING_BURST_GEN2 = 0x01BA0096UL;

    /* Initialize the active PHY instance for connection. */
    if (pCalCtxt->activePhyIdx != 0) {
        base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 = USB_PHY_TOP->TOP_CTRL_0;
    }

    if ((usbSpeed == CY_USBD_USB_DEV_SS_GEN1X2) || (usbSpeed == CY_USBD_USB_DEV_SS_GEN2X2)) {
        Cy_USBSS_Cal_InitSSPhy(pCalCtxt, 2);
    } else {
        Cy_USBSS_Cal_InitSSPhy(pCalCtxt, 1);
    }

    USB32DEV_LNK->LNK_CONF = (
            USB32DEV_LNK_CONF_DC_BAL_ALL_OS_Msk |
            USB32DEV_LNK_CONF_GEN1X1_LOOPBACK_MASTER_SKP_EN_Msk |
            USB32DEV_LNK_CONF_GEN1X1_CP_TXDETECT_RXLPBK_EN_Msk |
            USB32DEV_LNK_CONF_GEN1X1_INVALID_OS_DET_EN_Msk |
            USB32DEV_LNK_CONF_LDN_DETECTION_Msk |
            (7UL << USB32DEV_LNK_CONF_EPM_FIRST_DELAY_Pos) |
            USB32DEV_LNK_CONF_DEBUG_FEATURE_ENABLE_Msk |
            USB32DEV_LNK_CONF_LCW_IGNORE_RSVD_Msk);     /* 0x000F72C0UL */

    if (pCalCtxt->gen2Enabled) {
        USB32DEV_LNK->LNK_CONF |= USB32DEV_LNK_CONF_DC_BAL_EN_Msk;
    }

    /* Enable termination at LNK level and enable free running of LTSSM state machine. */
    USB32DEV_LNK->LNK_PHY_CONF |= USB32DEV_LNK_PHY_CONF_RX_TERMINATION_ENABLE_Msk;

    /* Configure the PHY lane ID with the LINK controller. */
    USB32DEV_MAIN->CTRL = ((USB32DEV_MAIN->CTRL & ~USB32DEV_MAIN_CTRL_CONFIG_LANE_Msk) |
        (pCalCtxt->activePhyIdx << USB32DEV_MAIN_CTRL_CONFIG_LANE_Pos));

    /* Enable the SuperSpeed Device function */
    USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;

    /* Configure the USB3.2 function to use PHY clock */
    USB32DEV_MAIN->CTRL = (USB32DEV_MAIN->CTRL & ~USB32DEV_MAIN_CTRL_PCLK_SRC_Msk) |
                          (1UL << USB32DEV_MAIN_CTRL_PCLK_SRC_Pos);
    USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_CONN_EN);

    /* Enable the control endpoint. */
    USB32DEV_PROT->PROT_EPI_CS1[0] = USB32DEV_PROT_EPI_CS1_VALID_Msk;
    USB32DEV_PROT->PROT_EPO_CS1[0] = USB32DEV_PROT_EPO_CS1_VALID_Msk;
    USB32DEV_PROT->PROT_EPI_CS2[0] = USB32DEV_PROT->PROT_EPO_CS2[0] = 0x03UL;

    /* Start the USB connection with LPM disabled. */
    pCalCtxt->linkSuspended = false;
    pCalCtxt->lpmConfig = CY_SSCAL_LPM_DISABLED;
    USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL =
        (USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U1_Msk |
         USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U2_Msk);

    /* Flush all endpoint memories when making a new connection. */
    Cy_USBSS_Cal_FlushEPM(pCalCtxt, false);

    USB32DEV_PROT->PROT_LMP_PORT_CAPABILITY_TIMER    = CY_USBSS_PROT_LMP_PORT_CAP_TIMER_VALUE;
    USB32DEV_PROT->PROT_LMP_PORT_CONFIGURATION_TIMER = CY_USBSS_PROT_LMP_PORT_CFG_TIMER_VALUE;

    /* Turn on AUTO response to LGO_U3 command from host. */
    USB32DEV_LNK->LNK_COMPLIANCE_PATTERN_8 |= USB32DEV_LNK_COMPLIANCE_PATTERN_8_LFPS_Msk;

    return CY_USB_CAL_STATUS_SUCCESS;
}

#define FX3G2_VPTX_MV           (1150U)  /* 1.15 V */
#define FX3G2_TXSWING_MV        (1150U)  /* 1.15 V */

const uint32_t PhyTxCursorPoints[] = {
    /* Different PHY TX settings for device characterisation only. */
#if 0
    125, 792,  83               /* C(1) = -0.125, C(0) = 0.792, C(-1) = -0.083 */
    230, 687,  83               /* C(1) = -0.230, C(0) = 0.687, C(-1) = -0.083 */
#endif
    100, 850,  50               /* C(1) = -0.10, C(0) = 0.85, C(-1) = -0.05 */
};

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_TxInit
****************************************************************************//**
*
* Powers-Up and enables the USB3 transmitter PHY AFE.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_TxInit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                     uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP;
    uint32_t i, j, slice;
    uint32_t nTotal, nVSwing;
    uint32_t nPre, nPost;
    uint32_t shiftData[8];

    /* Clear any stale interrupt status. */
    USB_PHY_TOP->INTR0 = 0xFFFFFFFFUL;

    /* If TX_AFE_CTRL_1 has not been initialized as part of trimming, set TX_ICAL_TRIM to default
     * value at midpoint of range.
     */
    if (USB_PHY_TOP->TX_AFE_CTRL_1 == 0) {
        USB_PHY_TOP->TX_AFE_CTRL_1 = 0x0000001FUL;
    }

    /* Enable the TX AFE driver and voltage regulators
     * Note: TX_DETECTRX_TRIM and TX_ELECIDLE_DRV fields in this register should not be changed.
     */
    USB_PHY_TOP->TX_AFE_CTRL_0 = (
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_DRV_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_CML2CMOS_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_SEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_SEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_SEL_Pos) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DCD_CTRL_Pos) |
            (0x02UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DETECTRX_TRIM_Pos) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Pos)
            );                                          /* 0x1846B4DFUL; */

    /* Wait for 10 us to settle voltage regulators */
    Cy_SysLib_DelayUs(10);

    /* Set impedance calibration comparator wait to maximum value and start impedance calibration. */
    USB_PHY_TOP->TX_AFE_ZTRIM  = (
            (USB_PHY_TOP->TX_AFE_ZTRIM & ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_ZTRIM_ZCAL_COMP_WAIT_Msk) |
            (3UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_ZTRIM_ZCAL_COMP_WAIT_Pos));
    USB_PHY_TOP->TX_AFE_ZTRIM |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_ZTRIM_ZCAL_EN_Msk;
    Cy_SysLib_DelayUs(1);

    /* Wait for the calibration to complete. */
    do {
        i = USB_PHY_TOP->INTR0;
    } while ((i & USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_TX_IMP_CAL_DONE_Msk) == 0);

    DBG_SSCAL_TRACE("TXAFE-%d: %x %x %x %x %x\r\n",
            laneId,
            USB_PHY_TOP->TX_AFE_CTRL_0, USB_PHY_TOP->TX_AFE_CTRL_1,
            USB_PHY_TOP->TX_AFE_ZTRIM,
            USB_PHY_TOP->INTR0,
            USB_PHY_TOP->TX_AFE_STATUS);

    /* Clear the calibration done interrupt. */
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;

    nTotal = 56UL +
        ((USB_PHY_TOP->TX_AFE_STATUS & USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_STATUS_ZCAL_CODE_Msk) >>
         USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_STATUS_ZCAL_CODE_Pos);

#if (FX3G2_VPTX_MV != FX3G2_TXSWING_MV)
    /* nVSwing = (1 - (vSwing / vTx)) * nTotal = (1 - (1.1 V / 1.15 V)) * nTotal. */
    nVSwing = (nTotal * (FX3G2_VPTX_MV - FX3G2_TXSWING_MV)) / FX3G2_VPTX_MV;
#else
    nVSwing = 0;
#endif /* (FX3G2_VPTX_MV != FX3G2_TXSWING_MV) */

    nPost = (PhyTxCursorPoints[0] * (nTotal - nVSwing)) / 1000;
    nPre  = (PhyTxCursorPoints[2] * (nTotal - nVSwing)) / 1000;

    /* Calculate the values to be programmed into the TX AFE shift register based on the above values. */
    shiftData[0] = 0x00UL;
    shiftData[1] = 0x00UL;
    shiftData[2] = 0x00UL;
    shiftData[3] = 0x00UL;
    shiftData[4] = 0x00UL;
    shiftData[5] = 0x00UL;
    shiftData[6] = 0x00UL;
    shiftData[7] = 0x00UL;

    slice = 0;
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 22; j += 2) {
            if (slice < nVSwing) {
                /* Set slice to 2 for VSwing control. */
                shiftData[i] |= (2UL << j);
            } else {
                if (slice < nTotal) {
                    if ((slice < (nVSwing + nPre)) || (slice > (nTotal - nPost))) {
                        /* Set slice to 1 for pre-shoot or post-cursor control. */
                        shiftData[i] |= (1UL << j);
                    } else {
                        /* Set slice to 3 to drive main cursor to pad. */
                        shiftData[i] |= (3UL << j);
                    }
                }
            }

            slice++;
        }
    }

    DBG_SSCAL_TRACE("TXCALDONE: %d %d %d %d\r\n", nTotal, nVSwing, nPre, nPost);

    /* Store the slice configuration in the AFE TX shift register. */
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_0 = pCalCtxt->txAfeSettings[laneId][0] = shiftData[0]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_1 = pCalCtxt->txAfeSettings[laneId][1] = shiftData[1]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_2 = pCalCtxt->txAfeSettings[laneId][2] = shiftData[2]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_3 = pCalCtxt->txAfeSettings[laneId][3] = shiftData[3]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_4 = pCalCtxt->txAfeSettings[laneId][4] = shiftData[4]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_5 = pCalCtxt->txAfeSettings[laneId][5] = shiftData[5]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_6 = pCalCtxt->txAfeSettings[laneId][6] = shiftData[6]; //0x003fffffUL;
    USB_PHY_TOP->TX_AFE_CFG_SFT_W_7 = pCalCtxt->txAfeSettings[laneId][7] = shiftData[7]; //0x003fffffUL;

    /* Program the write done bit to start shifting the data. */
    USB_PHY_TOP->TX_AFE_CFG = USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CFG_REG_TX_CFG_WRITE_DONE_Msk;
    Cy_SysLib_DelayUs(5);

    /* Wait until shift register update is complete. */
    do {
        i = USB_PHY_TOP->TX_AFE_CFG;
    } while ((i & USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CFG_REG_TX_CFG_WRITE_DONE_Msk) != 0);

    USB_PHY_TOP->PIPE_OVERRIDE_0 = 0;
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_TxDeinit
****************************************************************************//**
*
* Disable the transmitter portion of the USBSS PHY.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_TxDeinit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                     uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP;

    /* Disable the TX AFE driver and voltage regulators */
    USB_PHY_TOP->TX_AFE_ZTRIM &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_ZTRIM_ZCAL_EN_Msk;
    USB_PHY_TOP->TX_AFE_CTRL_0 &= ~(
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_DRV_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_CML2CMOS_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_EN_Msk
            );

    USB_PHY_TOP->PIPE_OVERRIDE_0 = 0;

    /* Clear any stale interrupt status. */
    USB_PHY_TOP->INTR0 = 0xFFFFFFFFUL;

    return CY_USB_CAL_STATUS_SUCCESS;
}

#define DFE_TAP_WAIT_US (150)
#define AFE_TAP_WAIT_US (50)
#define OSA_ROAM_MASK   (0x00FF)

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_RxOffsetCalibration
****************************************************************************//**
*
* Function that performs USB 3.x receiver offset calibration.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* void
*******************************************************************************/
void Cy_USBSS_Phy_RxOffsetCalibration (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                       uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_RX_Type *USB_PHY_RX = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_RX;

    USB_PHY_RX->RX_OSA_CFG5 = 0x0000UL;
    USB_PHY_RX->RX_OSA_CFG0 = 0x4040UL;
    USB_PHY_RX->RX_OSA_CFG1 = 0x4040UL;
    USB_PHY_RX->RX_OSA_CFG2 = 0x4040UL;
    USB_PHY_RX->RX_OSA_CFG3 = 0x4040UL;
    USB_PHY_RX->RX_OSA_CFG4 = 0x0040UL;

    /* Add some delay before offset calibration. */
    Cy_SysLib_Delay(1);

    /* Initiate DFE+AFE offset calibration. */
    USB_PHY_RX->RX_OSA_CFG5 = (
            (USB_PHY_RX->RX_OSA_CFG5 & 0xFFF8UL) | 0x05UL);
    Cy_SysLib_DelayUs(50UL);

    /* Verify that offset calibration is completed. */
    if ((USB_PHY_RX->RX_OSA_STAT5 & 0x07UL) != 0x07UL) {
        DBG_SSCAL_INFO("Phy:%d RX offset calibration timed out: %x\r\n", laneId, USB_PHY_RX->RX_OSA_STAT5);
    }
    Cy_SysLib_DelayUs(1UL);

    /* Initiate AFE offset calibration. */
    USB_PHY_RX->RX_OSA_CFG5 = (
            (USB_PHY_RX->RX_OSA_CFG5 & 0xFFF8UL) | 0x06UL);
    Cy_SysLib_DelayUs(50UL);

    /* Verify that offset calibration is completed. */
    if ((USB_PHY_RX->RX_OSA_STAT5 & 0x02UL) != 0x02UL) {
        DBG_SSCAL_INFO("Phy:%d RX AFE offset calibration timed out: %x\r\n",laneId, USB_PHY_RX->RX_OSA_STAT5);
    }
    Cy_SysLib_DelayUs(1UL);

    /* Print the results for analysis. */
    DBG_SSCAL_TRACE("Phy:%d RX_OSA_STAT=%x %x %x %x %x %x %x %x %x %x %x %x\r\n",
            laneId,
            USB_PHY_RX->RX_OSA_STAT0 & 0xFF, (USB_PHY_RX->RX_OSA_STAT0 >> 8),
            USB_PHY_RX->RX_OSA_STAT1 & 0xFF, (USB_PHY_RX->RX_OSA_STAT1 >> 8),
            USB_PHY_RX->RX_OSA_STAT2 & 0xFF, (USB_PHY_RX->RX_OSA_STAT2 >> 8),
            USB_PHY_RX->RX_OSA_STAT3 & 0xFF, (USB_PHY_RX->RX_OSA_STAT3 >> 8),
            USB_PHY_RX->RX_OSA_STAT4 & 0xFF, (USB_PHY_RX->RX_OSA_STAT4 >> 8),
            USB_PHY_RX->RX_OSA_STAT5 & 0xFF,
            USB_PHY_RX->RX_OSA_STAT8 & 0xFF);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_SetRxRegsToDefault
****************************************************************************//**
*
* Sets values control/status registers in the RX portion of the USBSS PHY
* to default values.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*******************************************************************************/
static void
Cy_USBSS_Phy_SetRxRegsToDefault (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_RX_Type *USB_PHY_RX   = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_RX;

    USB_PHY_RX->RX_DFE_CFG0 &= ~(
            USB32DEV_PHYSS_USB40PHY_RX_DFE_CFG0_OVRD_VGATAP_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_DFE_CFG0_OVRD_DFETAP1_EN_Msk);
    USB_PHY_RX->RX_DFE_CFG1 &= ~(
            USB32DEV_PHYSS_USB40PHY_RX_DFE_CFG1_OVRD_DFETAP2_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_DFE_CFG1_OVRD_DFETAP3_EN_Msk);
    USB_PHY_RX->RX_DFE_CFG2     = 0xB79EUL;
    USB_PHY_RX->RX_DFE_CFG3     = 0x0003UL;
    USB_PHY_RX->RX_CTLE_CFG0   &= ~USB32DEV_PHYSS_USB40PHY_RX_CTLE_CFG0_CTLE_CAP_GAIN_ADJ_Msk;
    USB_PHY_RX->RX_CTLE_CFG1    = 0x0808UL;
    USB_PHY_RX->RX_DFEA_CFG0    = 0x4444UL;
    USB_PHY_RX->RX_CP_CFG       = 0x0055UL;
    USB_PHY_RX->RX_BIASGEN_CFG0 = 0x0924UL;
}

/*******************************************************************************
 * Function Name: Cy_USBSS_Phy_TrimRxCtle
 ****************************************************************************//**
 *
 * Function to trim the RX CTLE common mode voltage.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
 * allocated by the user.
 *
 * \param laneId
 * USB3 data lines pair index.
 *
 *******************************************************************************/
void
Cy_USBSS_Phy_TrimRxCtle (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_RX_Type *USB_PHY_RX   = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP;
    uint32_t regVal;
    uint32_t i;
    uint32_t stg1 = 0;
    uint32_t stg2 = 0;
    uint32_t stg3 = 0;

    /* Make sure PHY0 is enabled and ADC is configured with LDO voltage as reference and ADFT output
     * connected to the input.
     */
    if (laneId) {
        if (pCalCtxt->dualLaneEnabled == false) {
            USB_PHY_TOP->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;
        }
        USB_PHY_TOP->ADC = (
                (USB_PHY_TOP->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
                (0x80UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_MID_VAL_Pos) |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk |
                0x04UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos);
    } else {
        USB_PHY_TOP->ADC = (
                (USB_PHY_TOP->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_TRIM_Msk) |
                (0x80UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_MID_VAL_Pos) |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_LDO_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_ADC_ISO_N_Msk |
                0x05UL << USB32DEV_PHYSS_USB40PHY_TOP_ADC_VSEL_Pos);
    }

    /* For each of the three CTLE stages, try out different bias currents until
     * we get the correct Common Mode voltage reading using ADC.
     */

    /* Stage 1:
     * Control using RX_BIASGEN_CFG0.AFEBIASSET0[2:0].
     * ADFT setting for monitoring is RX_ATEST_CFG.ADFT_CNTL = 10.
     */
    USB_PHY_RX->RX_ATEST_CFG = ((10UL << USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_CNTL_Pos) |
            USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_ENABLE_Msk);
    Cy_SysLib_DelayUs(20);

    regVal = USB_PHY_RX->RX_BIASGEN_CFG0;
    for (i = 8; i > 0; i--) {
        USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFFF8UL) | (i - 1));
        Cy_SysLib_DelayUs(20);

        USB_PHY_TOP->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;
        Cy_SysLib_DelayUs(5);
        while ((USB_PHY_TOP->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk) != 0);

        /* If the ADC value is greater than 0xC0 (0.8625 V), we can pick this setting and move on. */
        if (((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U) >= 0xC0UL) {
            stg1 = i - 1;
            break;
        }
    }

    DBG_SSCAL_TRACE("STG1: BIASSET=%d ADCVAL=%x\r\n", stg1, ((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U));
    USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFFF8UL) | (stg1));
    regVal = USB_PHY_RX->RX_BIASGEN_CFG0;

    /* Stage 2:
     * Control using RX_BIASGEN_CFG0.AFEBIASSET0[8:6].
     * ADFT setting for monitoring is RX_ATEST_CFG.ADFT_CNTL = 11.
     */
    USB_PHY_RX->RX_ATEST_CFG = ((11UL << USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_CNTL_Pos) |
            USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_ENABLE_Msk);
    Cy_SysLib_DelayUs(20);

    for (i = 8; i > 0; i--) {
        USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFE3FUL) | ((i - 1) << 6));
        Cy_SysLib_DelayUs(20);

        USB_PHY_TOP->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;
        Cy_SysLib_DelayUs(5);
        while ((USB_PHY_TOP->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk) != 0);

        /* If the ADC value is greater than 0xC0 (0.8625 V), we can pick this setting and move on. */
        if (((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U) >= 0xC0UL) {
            stg2 = i - 1;
            break;
        }
    }

    DBG_SSCAL_TRACE("STG2: BIASSET=%d ADCVAL=%x\r\n", stg2, ((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U));
    USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFE3FUL) | (stg2 << 6U));
    regVal = USB_PHY_RX->RX_BIASGEN_CFG0;

    /* Stage 3:
     * Control using RX_BIASGEN_CFG0.AFEBIASSET0[5:3].
     * ADFT setting for monitoring is RX_ATEST_CFG.ADFT_CNTL = 5.
     */
    USB_PHY_RX->RX_ATEST_CFG = ((5UL << USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_CNTL_Pos) |
            USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_ENABLE_Msk);
    Cy_SysLib_DelayUs(20);

    for (i = 8; i > 0; i--) {
        USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFFC7UL) | ((i - 1) << 3));
        Cy_SysLib_DelayUs(20);

        USB_PHY_TOP->ADC |= USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk;
        Cy_SysLib_DelayUs(5);
        while ((USB_PHY_TOP->ADC & USB32DEV_PHYSS_USB40PHY_TOP_ADC_SAR_EN_Msk) != 0);

        /* If the ADC value is greater than 0xC0 (0.8625 V), we can pick this setting and move on. */
        if (((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U) >= 0xC0UL) {
            stg3 = i - 1;
            break;
        }
    }

    DBG_SSCAL_TRACE("STG3: BIASSET=%d ADCVAL=%x\r\n", stg3, ((USB_PHY_TOP->ADC_STATUS & 0xFF00UL) >> 8U));

    /* Apply the calculated bias settings. */
    USB_PHY_RX->RX_BIASGEN_CFG0 = ((regVal & 0xFFFFFFC7UL) | (stg3 << 3U));
    DBG_SSCAL_INFO("Final BIASGEN value=%x\r\n", USB_PHY_RX->RX_BIASGEN_CFG0);

    /* Disable ADFT. */
    USB_PHY_RX->RX_ATEST_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_ENABLE_Msk;

    /* Disable PHY0 if we are connecting using PHY1. */
    if ((pCalCtxt->dualLaneEnabled == false) && (laneId)) {
        USB_PHY_TOP->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk;
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_RxInit
****************************************************************************//**
*
* Powers-Up and enables the USB3 receiver PHY AFE.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_RxInit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                     uint8_t laneId,
                     bool isPowerOn)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_RX_Type *USB_PHY_RX   = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP;
    uint32_t i;

    if (isPowerOn) {
        /* Make sure CSRs are holding default values at start. */
        Cy_USBSS_Phy_SetRxRegsToDefault(pCalCtxt, laneId);
    }

    /* Clear all stale interrupt status. */
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;

    /* Lock detect configuration */
    USB_PHY_RX->RX_LD_CFG = (
            (6UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LOCKIN_Pos));

    /* High Speed Divider configuration */
    USB_PHY_RX->RX_DIVH_CFG = 0;

    /* Signal detect / LFPS configuration. */
    USB_PHY_RX->RX_SD_CFG = (USB32DEV_PHYSS_USB40PHY_RX_SD_CFG_LFPS_VTH_Msk |
        USB32DEV_PHYSS_USB40PHY_RX_SD_CFG_LFPSDET_PDB_Msk);

    /* For Gen1. */
    USB_PHY_RX->RX_GNRL_CFG = (
            (USB_PHY_RX->RX_GNRL_CFG & USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_VCO_ADJ_Msk) |
            (2UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos));

    /* General controls configuration. */
    USB_PHY_RX->RX_BIASGEN_CFG1 |= USB32DEV_PHYSS_USB40PHY_RX_BIASGEN_CFG1_BIAS_PDB_Msk;

    /* Enable regulators. */
    USB_PHY_RX->RX_VREG_CFG1 |= (
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VCPREG_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXA_PDB_Msk);
    Cy_SysLib_DelayUs(1);

    DBG_SSCAL_TRACE("Wait for POWER_GOOD_RXA...");
    /* Wait for the RX_POWER_GOOD_RXA interrupt and clear it. */
    do {
        i = USB_PHY_TOP->INTR0;
    } while ((i & USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_RX_POWER_GOOD_RXA_Msk) == 0);
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;
    DBG_SSCAL_TRACE("Done\r\n");

    /* Enable VREGRXD and wait for corresponding interrupt. */
    USB_PHY_RX->RX_VREG_CFG1 |= USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXD_PDB_Msk;
    Cy_SysLib_DelayUs(1);

    DBG_SSCAL_TRACE("Wait for POWER_GOOD_RXD...");
    do {
        i = USB_PHY_TOP->INTR0;
    } while ((i & USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_RX_POWER_GOOD_RXD_Msk) == 0);
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;
    DBG_SSCAL_TRACE("Done\r\n");

    /* Enable VREGRXCK and wait for corresponding interrupt. */
    USB_PHY_RX->RX_VREG_CFG1 |= USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXCK_PDB_Msk;
    Cy_SysLib_DelayUs(1);

    DBG_SSCAL_TRACE("Wait for POWER_GOOD_RXCK...");
    do {
        i = USB_PHY_TOP->INTR0;
    } while ((i & USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_RX_POWER_GOOD_RXCK_Msk) == 0);
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;
    DBG_SSCAL_TRACE("Done\r\n");

    /* L2R offset calibration */
    USB_PHY_RX->RX_GNRL_CFG |= USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_CLKGEN_PDB_Msk;

    USB_PHY_RX->RX_LD_CFG = (
            (1UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LOCKIN_Pos) |
            (3UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FLOCKSEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LUDDISABLE_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LD_RESETB_Msk);
    USB_PHY_RX->RX_LD_CFG |= USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_RESETLOCKB_Msk;
    Cy_SysLib_DelayUs(60);

    /* Select 480 MHz reference clock. */
    USB_PHY_RX->RX_REFCKSEL_CFG = USB32DEV_PHYSS_USB40PHY_RX_REFCKSEL_CFG_LOCK2REF_SEL_Msk;

    /* Divide the reference clock down to 120 MHz. */
    USB_PHY_RX->SPARE_CFG |= (1UL << USB32DEV_PHYSS_USB40PHY_RX_SPARE_CFG_SPARE_OUT_Pos);

    /* Sigma-delta modulator configuration:
     * Reference clock frequency = 120 MHz
     * Target VCO frequency = 5000 MHz - (5000 MHz * 5000 ppm / 2) = 4987.5 MHz
     * Divider = (4987.5 / 120) - 3 = 38.5625
     *          SDM_CFG0 = INT(38.5625) = 38 = 0x26
     *          SDM_CFG1:SDM_CFG2 = FRAC(38.5625) * 2^18 = 0x24000 (0x9000, 0x2)
     */
    USB_PHY_RX->RX_SDM_CFG0 = (0x0026UL << USB32DEV_PHYSS_USB40PHY_RX_SDM_CFG0_DIVF_INTEGER_Pos);
    USB_PHY_RX->RX_SDM_CFG1 = (0x9000UL << USB32DEV_PHYSS_USB40PHY_RX_SDM_CFG1_DIVF_FRAC_MSB_Pos);
    USB_PHY_RX->RX_SDM_CFG2 = (0x0002UL << USB32DEV_PHYSS_USB40PHY_RX_SDM_CFG2_DIVF_FRAC_LSB_Pos);
    USB_PHY_RX->RX_SDM_CFG3 |= USB32DEV_PHYSS_USB40PHY_RX_SDM_CFG3_SDM_ENABLE_Msk;

    /* DFE analog configuration. */
    if (isPowerOn) {
        USB_PHY_RX->RX_DFEA_CFG0 = (
                (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFEMAINADJ_Pos) |
                USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk |
                (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1ADJ_Pos) |
                (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2ADJ_Pos) |
                (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3ADJ_Pos));

    } else {
        if (pCalCtxt->gen2Enabled) {
            USB_PHY_RX->RX_DFEA_CFG0 = (
                    (0x06UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFEMAINADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1ADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1_PDB_Msk |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2ADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2_PDB_Msk |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3ADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3_PDB_Msk);
        } else {
            USB_PHY_RX->RX_DFEA_CFG0 = (
                    (0x06UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFEMAINADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1ADJ_Pos) |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2ADJ_Pos) |
                    USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2_PDB_Msk |
                    (0x04UL << USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3ADJ_Pos));
        }
    }

    USB_PHY_RX->RX_GNRL_CFG |= (
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_SUMEVN_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_SUMODD_PDB_Msk);

    /* Configure for L2R mode. */
    USB_PHY_RX->RX_LD_CFG |= (
            (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Pos));

    USB_PHY_RX->RX_GNRL_CFG |= (
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DESERRESETB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_EDGE_DESER_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_ROM_PATH_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_ERR_PATH_EN_Msk);

    if (isPowerOn) {
        /* Enable phase interpolator */
        USB_PHY_RX->RX_EMA_CFG = (
                (4UL << USB32DEV_PHYSS_USB40PHY_RX_EMA_CFG_PICAPSEL_Pos) |
                USB32DEV_PHYSS_USB40PHY_RX_EMA_CFG_PI_PDB_Msk);
        Cy_SysLib_DelayUs(10);

        /* Trim the CTLE common mode voltages. */
        Cy_USBSS_Phy_TrimRxCtle(pCalCtxt, laneId);

        /* Eye monitor FSM configuration. */
        USB_PHY_RX->RX_EM_CFG1 = (
                (USB_PHY_RX->RX_EM_CFG1 & 0x00FF) |
                (4UL << USB32DEV_PHYSS_USB40PHY_RX_EM_CFG1_PIROAMPSEL_Pos) |
                USB32DEV_PHYSS_USB40PHY_RX_EM_CFG1_PIROAMPSEL_EN_Msk);

        /* Adjust common mode for offset calibration. */
        USB_PHY_RX->RX_AFE_CFG = (
                (USB_PHY_RX->RX_AFE_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_CMADJ_Msk) |
                (0x05UL << USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_CMADJ_Pos));

        Cy_USBSS_Phy_RxOffsetCalibration(pCalCtxt, laneId);

        /* Eye monitor FSM configuration. */
        USB_PHY_RX->RX_EM_CFG1 = (
                (USB_PHY_RX->RX_EM_CFG1 & 0x00FF) |
                (4UL << USB32DEV_PHYSS_USB40PHY_RX_EM_CFG1_PIROAMPSEL_Pos));

        /* Restore common mode for normal operation. */
        USB_PHY_RX->RX_AFE_CFG &= ~(USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_CMADJ_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_LPBK_EN_Msk);
        /* Enable RX termination. */
        USB_PHY_RX->RX_AFE_CFG |= USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_TERM_EN_Msk;
    }

    /* GEN1/GEN2 switch. */
    if (pCalCtxt->gen2Enabled) {
        /* For GEN2 */
        /* Set the correct VCO CP gain based on data rate. */
        USB_PHY_RX->RX_CP_CFG = (
                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));

        /* Set the data rate. */
        USB_PHY_RX->RX_GNRL_CFG = (
                (USB_PHY_RX->RX_GNRL_CFG & ~(
                                             USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk |
                                             USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk)) |
                (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos));
    } else {
        /* For GEN1 */
        /* Set the correct VCO CP gain based on data rate. */
        USB_PHY_RX->RX_CP_CFG = (
                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));

        /* Set the data rate. */
        USB_PHY_RX->RX_GNRL_CFG = (
                (USB_PHY_RX->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk);
    }

    if (isPowerOn) {
        /* Disable phase interpolator */
        USB_PHY_RX->RX_EMA_CFG = 0;
    } else {
        USB_PHY_RX->RX_ATEST_CFG &= ~(USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_VREGDIV_EN_Msk);
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_RxDeinit
****************************************************************************//**
*
* Disable the USBSS PHY receiver function.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_RxDeinit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                     uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_RX_Type *USB_PHY_RX   = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP;

    /* Disable termination. */
    USB_PHY_RX->RX_AFE_CFG &= ~(USB32DEV_PHYSS_USB40PHY_RX_AFE_CFG_TERM_EN_Msk);

    /* Disable lock detect. */
    USB_PHY_RX->RX_LD_CFG = (
            (1UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LOCKIN_Pos) |
            (3UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FLOCKSEL_Pos));

    /* Disable signal detect/LFPS. */
    USB_PHY_RX->RX_SD_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_SD_CFG_LFPSDET_PDB_Msk;

    /* Clear CDR hardware enable. */
    USB_PHY_RX->RX_HDWR_ENABLE &= ~USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;

    USB_PHY_RX->RX_GNRL_CFG = (
            (USB_PHY_RX->RX_GNRL_CFG & USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_VCO_ADJ_Msk) |
            (2UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos));

    /* General controls configuration. */
    USB_PHY_RX->RX_BIASGEN_CFG1 &= ~USB32DEV_PHYSS_USB40PHY_RX_BIASGEN_CFG1_BIAS_PDB_Msk;

    /* Disable regulators. */
    USB_PHY_RX->RX_VREG_CFG1 &= ~(
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VCPREG_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXA_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXD_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXCK_PDB_Msk);

    /* Disable sigma-delta modulator. */
    USB_PHY_RX->RX_SDM_CFG3 &= ~USB32DEV_PHYSS_USB40PHY_RX_SDM_CFG3_SDM_ENABLE_Msk;

    /* DFE analog configuration. */
    USB_PHY_RX->RX_DFEA_CFG0 &= ~USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk;

    /* Disable phase interpolator */
    USB_PHY_RX->RX_EMA_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_EMA_CFG_PI_PDB_Msk;

    USB_PHY_RX->RX_OSA_CFG5 &= ~(
            USB32DEV_PHYSS_USB40PHY_RX_OSA_CFG5_OSA_DFE_START_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_OSA_CFG5_OSA_AFE_START_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_OSA_CFG5_OSA_RESETB_Msk);

    /* Restore register values to default. */
    Cy_USBSS_Phy_SetRxRegsToDefault(pCalCtxt, laneId);

    /* Clear all stale interrupt status. */
    USB_PHY_TOP->INTR0 = USB_PHY_TOP->INTR0;

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_PllInit
****************************************************************************//**
*
* Powers-Up and enables the USB3 PHY PLL.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_PllInit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;

    /* PLL is only part of PHY0 instance. */
    USB32DEV_PHYSS_USB40PHY_TOP_Type     *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *USB_PHY_PLL = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;

    /* Clear all stale interrupt state. */
    USB_PHY_TOP->INTR0 = 0xFFFFFFFFUL;

    /* Code sequence adapted from the test bench used for Mixed-Signal Simulaton. */

    /* Enable regulators (address, mask, write_data, readback_data), allow 50 us for bring-up before timeout */
    /* <0> vcpreg_pdb = 1 */
    /* <1> vreglcpll_pdb = 1 */
    /* <2> vregref_pdb = 1 */
    /* <3> vregdig_pdb = 1 */
    USB_PHY_PLL->PLL_VREG_CFG2 = (USB_PHY_PLL->PLL_VREG_CFG2 & ~0x0000000FUL) | 0x0000000FUL;
    USB_PHY_PLL->PLL_VREG_CFG1 = 0xFFFFUL;

    /* Configure bias generator */
    /* <5:0> cfg_bias = 6'b100100 */
    /* <12:8> term_trim = 5'b10000 */
    USB_PHY_PLL->PLL_BIASGEN_CFG = (USB_PHY_PLL->PLL_BIASGEN_CFG & ~0x00001F3FUL) | 0x00001024UL;

    /* Now wait for 50us and confirm that we can read back power_good on all regulators */
    Cy_SysLib_DelayUs(50u);
    if ((USB_PHY_PLL->PLL_VREG_STAT & 0x07UL) != 0x07UL) {
        DBG_SSCAL_TRACE("[WARNING]: PLL Power good all not detected, proceeding anyways\r\n");
    } else {
        DBG_SSCAL_TRACE("PLL power good all detected\r\n");
    }

    /* Enable proper reference clock */
    /* <2:0> refclksel=3'b100 */
    USB_PHY_PLL->PLL_REFCKSEL_CFG = (USB_PHY_PLL->PLL_REFCKSEL_CFG & ~0x00000007UL) | 0x00000004UL;

    /* Update divider ratio */
    /* <3:0> lcdivr = 4'b0010 */
    USB_PHY_PLL->PLL_DIVR_CFG = (USB_PHY_PLL->PLL_DIVR_CFG & ~0x0000000FUL) | 0x00000002UL;

    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
        /* Configure PFD */
        /* <3:0> pfd_delay=4'b0001 */
        USB_PHY_PLL->PLL_PFD_CFG = (USB_PHY_PLL->PLL_PFD_CFG & ~0x0000000FUL) | 0x00000001UL;
    }

    /* Configure Charge Pump */
    /* <3:0> cfgrcp = 4'b0010 */
    /* <7:4> icpdac = 4'b0010 */
    /* <8> cp_pdb = 1'b1 */
    USB_PHY_PLL->PLL_CP_CFG = (USB_PHY_PLL->PLL_CP_CFG & ~0x000001FFUL) | 0x00000122UL;

    /* Configure Loop filter */
    /* <2:0> lf_rtune = 3'b001 */
    /* <4:3> lf_ctune = 2'b01 */
    USB_PHY_PLL->PLL_LF_CFG = (USB_PHY_PLL->PLL_LF_CFG & ~0x0000001FUL) | 0x00000009UL;

    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
        /* Configure VCO */
        /* <3:0> dcvarmode = 4'b0101 */
        /* <7:4> varcm1 = 4'b0110 */
        /* <11:8> varcm2 = 4'b0111 */
        USB_PHY_PLL->PLL_VCO_CFG = (USB_PHY_PLL->PLL_VCO_CFG & ~0x00000FFFUL) | 0x00000765UL;
    } else {
        /* Configure VCO */
        /* <3:0> dcvarmode = 4'b0110 */
        /* <7:4> varcm1 = 4'b0111 */
        /* <11:8> varcm2 = 4'b1000 */
        USB_PHY_PLL->PLL_VCO_CFG = (USB_PHY_PLL->PLL_VCO_CFG & ~0x00000FFFUL) | 0x00000876UL;
    }

    /* Enable PLL */
    /* <0> pll_pdb = 1'b1 */
    USB_PHY_PLL->PLL_GNRL_CFG = (USB_PHY_PLL->PLL_GNRL_CFG & ~0x00000001UL) | 0x00000001UL;

    /* Configure Sigma-Delta */
    /* <0> sdm_enable = 1'b1 */
    /* <1> dither_en = 1'b1 */
    /* <4:2> dither_gain = 3'b001 */
    USB_PHY_PLL->PLL_SDM_CFG = (USB_PHY_PLL->PLL_SDM_CFG & ~0x0000001FUL) | 0x00000007UL;

    /* Configure SSC DIVF = (5e9/240e6)-3 = 17.83333333d = 11.D55554h, note these are the default values */
    /* <7:0> divf_frac_msb = 8'hD5 */
    /* <15:8> divf_integer = 8'b00010001 */
    USB_PHY_PLL->PLL_SSM_CFG0 = (USB_PHY_PLL->PLL_SSM_CFG0 & ~0x0000FFFFUL) | 0x000011D5UL;
    /* <9:0> divf_frac_lsb = 10'h155 */
    USB_PHY_PLL->PLL_SSM_CFG1 = (USB_PHY_PLL->PLL_SSM_CFG1 & ~0x000003FFUL) | 0x00000155UL;
    /* <14> ssm_enable = 1'b0; (leave off for now) */
    USB_PHY_PLL->PLL_SSM_CFG2 = (USB_PHY_PLL->PLL_SSM_CFG2 & ~0x00004000UL) | 0x00000000UL;

    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
        /* Configure Lock-Detect */
        /* <2:0> lockos, default=000b (0.025% accuracy) */
        /* <3> Unused, set to zero */
        /* <5:4> dlfcntsel, default=11b (1024 cycles before flock) */
        /* <6> luddivsel, default=0 (128 cycles before declaring LUDO), set to 0b */
        /* <7> luddisable, default=0 (enable LUDO) */
        USB_PHY_PLL->PLL_LD_CFG = (USB_PHY_PLL->PLL_LD_CFG & ~0x000000FFUL) | 0x00000030UL;
    } else {
        /* Configure Lock-Detect */
        /* <2:0> lockos, default=100b (0.025% accuracy) */
        /* <3> Unused, set to zero */
        /* <5:4> dlfcntsel, default=11b (1024 cycles before flock) */
        /* <6> luddivsel, default=1 (128 cycles before declaring LUDO), set to 0b */
        /* <7> luddisable, default=1 (enable LUDO) */
        USB_PHY_PLL->PLL_LD_CFG = (USB_PHY_PLL->PLL_LD_CFG & ~0x000000FFUL) | 0x000000F4UL;
    }

    /* Enable level shifter on the USB2 clock reference to PLL. */
    USB_PHY_TOP->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;

    /* Reset Lock-Detect */
    /* <0> pll_pdb = 1'b1 */
    /* <1> ld_resetb = 1'b1 */
    /* <2> resetlockb = Toggle 0/1 */
    USB_PHY_PLL->PLL_GNRL_CFG = (USB_PHY_PLL->PLL_GNRL_CFG & ~0x00000006UL) | 0x00000003UL;
    USB_PHY_PLL->PLL_GNRL_CFG = (USB_PHY_PLL->PLL_GNRL_CFG & ~0x00000006UL) | 0x00000007UL;

    /* Configure PLL for Initial AFC Calibration */
    /* <2:0> afc_vctl_set = 3'b100 */
    /* <3> afc_vcmp_pdb = 1'b1 */
    /* <7:4> afc_vcmp_sel = 4'b1000 */
    USB_PHY_PLL->PLL_AFC_CFG0 = (USB_PHY_PLL->PLL_AFC_CFG0 & ~0x000000FFUL) | 0x0000008CUL;
    /* <4:0> afc_vctl_hi_cnt = 5'b11111 */
    /* <9:5> afc_vctl_lo_cnt = 5'b10000 */
    USB_PHY_PLL->PLL_AFC_CFG1 = (USB_PHY_PLL->PLL_AFC_CFG1 & ~0x000003FFUL) | 0x0000021FUL;
    /* <5:0> afc_clkdivsel = 6'111111 */
    USB_PHY_PLL->PLL_AFC_CFG2 = (USB_PHY_PLL->PLL_AFC_CFG2 & ~0x0000003FUL) | 0x0000003FUL;

    /* Force aacsel/itailsel=0 to maximize gain */
    /* <8:4> aac_ovrd_aacsel = 5'b00000 */
    /* <9> aac_ovrden = 1'b1 */
    USB_PHY_PLL->PLL_AAC_CFG0 = (USB_PHY_PLL->PLL_AAC_CFG0 & ~0x000003F0UL) | 0x00000200UL;

    /* Start AFC Calibration Loop */
    /* <6> afc_pdb = 1'b1 */
    USB_PHY_PLL->PLL_AFC_CFG2 = (USB_PHY_PLL->PLL_AFC_CFG2 & ~0x00000040UL) | 0x00000040UL;
    DBG_SSCAL_TRACE("Starting Initial AFC Calibration Loop\r\n");

    /* Run a long time..... */
    Cy_SysLib_DelayUs(1800UL);

    /* Remove force on aacsel */
    /* <8:4> aac_ovrd_aacsel = 5'b00000 */
    /* <9> aac_ovrden = 1'b0 */
    USB_PHY_PLL->PLL_AAC_CFG0 = (USB_PHY_PLL->PLL_AAC_CFG0 & ~0x000003F0UL) | 0x00000000UL;

    /* Configure AAC Calibration */
    /* <3:0> aacrefsel = 4'b0110 (default = 4'b0110) */
    USB_PHY_PLL->PLL_AAC_CFG0 = (USB_PHY_PLL->PLL_AAC_CFG0 & ~0x0000000FUL) | 0x00000006UL;
    /* <5:0> aac_clkdivsel = 6'b111000 (default = 6'b101000) */
    /* <6> aac_frzcnt = 1'b0 */
    /* <7> aac_norecal = 1'b0 (default = 1'b0, not sure what this does) */
    USB_PHY_PLL->PLL_AAC_CFG1 = (USB_PHY_PLL->PLL_AAC_CFG1 & ~0x000000FFUL) | 0x00000038UL;

    /* Start AAC Calibration */
    /* <8> aac_pdb = 1'b1 */
    USB_PHY_PLL->PLL_AAC_CFG1 = (USB_PHY_PLL->PLL_AAC_CFG1 & ~0x00000100UL) | 0x00000100UL;

    DBG_SSCAL_TRACE("Starting AAC Calibration Loop\r\n");
    /* Run a long time..... */
    Cy_SysLib_DelayUs(200UL);

    /* End AAC Calibration */
    /* <6> aac_frzcnt = 1'b1 */
    USB_PHY_PLL->PLL_AAC_CFG1 = (USB_PHY_PLL->PLL_AAC_CFG1 & ~0x00000040UL) | 0x00000040UL;
    Cy_SysLib_DelayUs(2);

    /* Repeat AFC Calibration Loop */
    /* <6> afc_pdb = 1'b0/1 */
    USB_PHY_PLL->PLL_AFC_CFG2 = (USB_PHY_PLL->PLL_AFC_CFG2 & ~0x00000040UL) | 0x00000000UL;
    Cy_SysLib_DelayUs(2UL);
    USB_PHY_PLL->PLL_AFC_CFG2 = (USB_PHY_PLL->PLL_AFC_CFG2 & ~0x00000040UL) | 0x00000040UL;
    DBG_SSCAL_TRACE("Starting Final AFC Calibration Loop\r\n");

    /* Wait for a long time to allow AFC calibration to complete. */
    Cy_SysLib_DelayUs(1800UL);

    USB_PHY_PLL->PLL_AFC_CFG0 = (USB_PHY_PLL->PLL_AFC_CFG0 & ~0x000000FFUL) | 0x00000084UL;

    /* GEN1/GEN2 switch. */
    if (pCalCtxt->gen2Enabled) {
        /* <3:0> lcdivp = 4'b0000 */
        USB_PHY_PLL->PLL_DIVP_CFG = (USB_PHY_PLL->PLL_DIVP_CFG & ~0x0000000FUL) | 0x00000000UL;
    } else {
        /* <3:0> lcdivp = 4'b0001 */
        USB_PHY_PLL->PLL_DIVP_CFG = (USB_PHY_PLL->PLL_DIVP_CFG & ~0x0000000FUL) | 0x00000001UL;
    }

    /* Enable output clock on lane 0 */
    /* <5> tx0_divp_pdb = 1'b1 */
    /* <6> tx1_divp_pdb = 1'b0 */
    USB_PHY_PLL->PLL_DIVP_CFG = (USB_PHY_PLL->PLL_DIVP_CFG & ~0x00000060UL) | 0x00000020UL;
    Cy_SysLib_DelayUs(2UL);

    /* Enable output clock on lane 1 */
    /* <5> tx0_divp_pdb = 1'b1 */
    /* <6> tx1_divp_pdb = 1'b1 */
    USB_PHY_PLL->PLL_DIVP_CFG = (USB_PHY_PLL->PLL_DIVP_CFG & ~0x00000060UL) | 0x00000060UL;
    Cy_SysLib_DelayUs(2UL);

    /* Enable Sync Reset */
    /* pll_sync_rstb = 1'b1 */
    USB_PHY_PLL->PLL_DIVP_CFG = (USB_PHY_PLL->PLL_DIVP_CFG & ~0x00000080UL) | 0x00000080UL;
    Cy_SysLib_DelayUs(2UL);

    /* Toggle pll_rstb */
    USB_PHY_TOP->TOP_CTRL_0 |= (
            USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk);
    if (laneId != 0) {
        base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP.TOP_CTRL_0 |= (
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk);
    }

    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
        /* Configure PLL for Initial AFC Calibration */
        /* <3> afc_vcmp_pdb = 1'b0 */
        USB_PHY_PLL->PLL_AFC_CFG0 = (USB_PHY_PLL->PLL_AFC_CFG0 & ~0x00000008UL) | 0x00000000UL;

        /* Run for a bit w/SSC turned off */
        Cy_SysLib_DelayUs(20UL);

        /* Enable SSC */
        /* <14> ssm_enable = 1'b1 */
        USB_PHY_PLL->PLL_SSM_CFG2 = (USB_PHY_PLL->PLL_SSM_CFG2 & ~0x00004000UL) | 0x00004000UL;
        Cy_SysLib_DelayUs(80UL);

        /* Updates for PLL 5 GHz operation. */

        /* <3:0> dcvarmode = 4'b0000 */
        /* <7:4> varcm1 = 4'b0000 */
        /* <11:8> varcm2 = 4'b0000 */
        USB_PHY_PLL->PLL_VCO_CFG = (USB_PHY_PLL->PLL_VCO_CFG & ~0x00000FFFUL) | 0x00000000UL;

        /* <8:4> aac_ovrd_aacsel = 5'b10000 */
        /* <9> aac_ovrden = 1'b1 */
        USB_PHY_PLL->PLL_AAC_CFG0 = (USB_PHY_PLL->PLL_AAC_CFG0 & ~0x000003F0UL) | 0x00000300UL;

        /* <14:8> afc_ovrd_capsel = 7'b0000000 */
        /* <15> afc_ovrriden = 1 */
        USB_PHY_PLL->PLL_AFC_CFG0 = (USB_PHY_PLL->PLL_AFC_CFG0 & ~0x0000FF00UL) | 0x00008000UL;
    } else {
        /* Enable SSC */
        /* <14> ssm_enable = 1'b1 */
        USB_PHY_PLL->PLL_SSM_CFG2 = (USB_PHY_PLL->PLL_SSM_CFG2 & ~0x00004000UL) | 0x00004000UL;
    }

    DBG_SSCAL_TRACE("PLL Init Finished\r\n");
    Cy_SysLib_DelayUs(100UL);
    DBG_SSCAL_INFO("PLL_LD_STAT=%x\r\n", USB_PHY_PLL->PLL_LD_STAT);

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Phy_PllDeinit
****************************************************************************//**
*
* Disable the PLL in the USBSS PHY block.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param laneId
* USB3 data lines pair index.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
static cy_en_usb_cal_ret_code_t
Cy_USBSS_Phy_PllDeinit (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      uint8_t laneId)
{
    USB32DEV_Type *base = pCalCtxt->regBase;

    /* PLL is only part of PHY0 instance. */
    USB32DEV_PHYSS_USB40PHY_TOP_Type     *USB_PHY_TOP = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *USB_PHY_PLL = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;

    /* Automatic Amplitude Control (AAC) configuration */
    USB_PHY_PLL->PLL_AAC_CFG0 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_AAC_CFG0_AAC_OVRRIDEN_Msk;

    /* Power down regulators. */
    USB_PHY_PLL->PLL_VREG_CFG2 &= ~(
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VCPREG_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGLCPLL_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGREF_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGDIG_PDB_Msk);

    /* Disable the level shifter. */
    USB_PHY_TOP->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
    if (laneId != 0) {
        base->USB32DEV_PHYSS.USB40PHY[laneId].USB40PHY_TOP.TOP_CTRL_0 &=
            ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
    }

    /* Disable automatic frequency control. */
    USB_PHY_PLL->PLL_AFC_CFG0 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_AFC_CFG0_AFC_VCMP_PDB_Msk;
    USB_PHY_PLL->PLL_AFC_CFG2 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_AFC_CFG2_AFC_PDB_Msk;

    /* Disable charge pump. */
    USB_PHY_PLL->PLL_CP_CFG &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_CP_CFG_CP_PDB_Msk;

    /* Disable the PLL. */
    USB_PHY_PLL->PLL_GNRL_CFG &= ~(
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_PLL_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_LKDT_RESETB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_RESETLOCKB_Msk);

    /* Configure and start AAC Calibration */
    USB_PHY_PLL->PLL_AAC_CFG1 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_AAC_CFG1_AAC_PDB_Msk;
    USB_PHY_PLL->PLL_AAC_CFG1 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_AAC_CFG1_AAC_FRZCNT_Msk;

    /* Disable the post divider. */
    USB_PHY_PLL->PLL_DIVP_CFG &= 0x1FUL;

    /* Clear all interrupt state. */
    USB_PHY_TOP->INTR0 = 0xFFFFFFFFUL;

    return CY_USB_CAL_STATUS_SUCCESS;
}

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
bool
Cy_USBSS_Cal_SendMsg (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      void *pMsg)
{
    /*
     * Here call back function provided by USBD layer will be called.
     * OS dependent message function should be implemented as part of
     * callback function.
     */
    return (pCalCtxt->msgCb(pCalCtxt->pUsbdCtxt, (void *)pMsg));
}

/* Imported from code used in MSV simulation. */

#define N_EYE_LEVELS    13      /* Number of slicer levels used during eye height sweep */
#define DELTA_EYE_LEVEL 2       /* Eye height sweep step size: */
                                /* N_EYE_LEVELS*DELTA_EYE_LEVEL must be <(64-range for offset) */
#define EYE_LEVEL_START 1       /* Starting point for eye height measurement (1  = ~45mV0-p,diff) */
                                /* Maximum Eye Level = EYE_LEVEL_START+DELTA_EYE_LEVEL*N_EYE_LEVELS = 38 */
#define FAIL_LIMIT      100     /* Number of slicer errors above which cause the loop to skip the */
                                /* remaining dfe_slicepoint steps */
#define FULL_SWEEP      0       /* When =1, perform a complete sweep of eye height,  */
                                /* when =0 skip remaining points when error count > FAIL_LIMIT */
#define LUT_WIDTH       16      /* Number of CTLE settings in the Look-Up Table. */
#define SLICEPOINT_DEC_G1  4    /* Gen1: Decrement to be applied on DFE slicepoint. */
#define SLICEPOINT_DEC_G2  2    /* Gen2: Decrement to be applied on DFE slicepoint. */

#define LOCKIN_SETTING  (5u)    /* Optimal RX_LD_CFG.lockin setting needs to be determined. Relaxed to 5. */
#define MIN_SLICEPOINT  20      /* Minimum DFE Slice Point */
#define MAX_SLICEPOINT  44      /* Maximum DFE Slice Point */
#define MIN_VGA         1       /* Minimum VGA value */
#define MAX_VGA         14      /* Maximum VGA value */

/* PHY_RX register offsets. */
#define MXS40USB40PHY_RX_SDM_CFG0_OFFS          (0x00u >> 2u)
#define MXS40USB40PHY_RX_SDM_CFG1_OFFS          (0x04u >> 2u)
#define MXS40USB40PHY_RX_SDM_CFG2_OFFS          (0x08u >> 2u)
#define MXS40USB40PHY_RX_SDM_CFG3_OFFS          (0x0cu >> 2u)
#define MXS40USB40PHY_RX_DFE_CFG0_OFFS          (0x10u >> 2u)
#define MXS40USB40PHY_RX_DFE_CFG1_OFFS          (0x14u >> 2u)
#define MXS40USB40PHY_RX_DFE_CFG2_OFFS          (0x18u >> 2u)
#define MXS40USB40PHY_RX_DFE_CFG3_OFFS          (0x1cu >> 2u)
#define MXS40USB40PHY_RX_DFE_STAT0_OFFS         (0x20u >> 2u)
#define MXS40USB40PHY_RX_DFE_STAT1_OFFS         (0x24u >> 2u)
#define MXS40USB40PHY_RX_DFE_STAT2_OFFS         (0x28u >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG0_OFFS          (0x2cu >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG1_OFFS          (0x30u >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG2_OFFS          (0x34u >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG3_OFFS          (0x38u >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG4_OFFS          (0x3cu >> 2u)
#define MXS40USB40PHY_RX_OSA_CFG5_OFFS          (0x40u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT0_OFFS         (0x44u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT1_OFFS         (0x48u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT2_OFFS         (0x4cu >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT3_OFFS         (0x50u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT4_OFFS         (0x54u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT5_OFFS         (0x58u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT6_OFFS         (0x5cu >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT7_OFFS         (0x60u >> 2u)
#define MXS40USB40PHY_RX_OSA_STAT8_OFFS         (0x64u >> 2u)
#define MXS40USB40PHY_RX_CTLE_CFG0_OFFS         (0x68u >> 2u)
#define MXS40USB40PHY_RX_CTLE_CFG1_OFFS         (0x6cu >> 2u)
#define MXS40USB40PHY_RX_CTLE_STAT0_OFFS        (0x70u >> 2u)
#define MXS40USB40PHY_RX_CTLE_STAT1_OFFS        (0x74u >> 2u)
#define MXS40USB40PHY_RX_EM_CFG0_OFFS           (0x78u >> 2u)
#define MXS40USB40PHY_RX_EM_CFG1_OFFS           (0x7cu >> 2u)
#define MXS40USB40PHY_RX_EM_STAT0_OFFS          (0x80u >> 2u)
#define MXS40USB40PHY_RX_EM_STAT1_OFFS          (0x84u >> 2u)
#define MXS40USB40PHY_RX_DFEA_CFG0_OFFS         (0x88u >> 2u)
#define MXS40USB40PHY_RX_OSAA_CFG0_OFFS         (0x8cu >> 2u)
#define MXS40USB40PHY_RX_OSAA_CFG1_OFFS         (0x90u >> 2u)
#define MXS40USB40PHY_RX_AFE_CFG_OFFS           (0x94u >> 2u)
#define MXS40USB40PHY_RX_EMA_CFG_OFFS           (0x98u >> 2u)
#define MXS40USB40PHY_RX_REFCKSEL_CFG_OFFS      (0x9cu >> 2u)
#define MXS40USB40PHY_RX_DIVH_CFG_OFFS          (0xa0u >> 2u)
#define MXS40USB40PHY_RX_PFD_CFG_OFFS           (0xa4u >> 2u)
#define MXS40USB40PHY_RX_CP_CFG_OFFS            (0xa8u >> 2u)
#define MXS40USB40PHY_RX_LF_CFG_OFFS            (0xacu >> 2u)
#define MXS40USB40PHY_RX_BIASGEN_CFG0_OFFS      (0xb0u >> 2u)
#define MXS40USB40PHY_RX_BIASGEN_CFG1_OFFS      (0xb4u >> 2u)
#define MXS40USB40PHY_RX_GNRL_CFG_OFFS          (0xb8u >> 2u)
#define MXS40USB40PHY_RX_VREG_CFG0_OFFS         (0xbcu >> 2u)
#define MXS40USB40PHY_RX_VREG_CFG1_OFFS         (0xc0u >> 2u)
#define MXS40USB40PHY_RX_VREG_STAT_OFFS         (0xc4u >> 2u)
#define MXS40USB40PHY_RX_SD_CFG_OFFS            (0xc8u >> 2u)
#define MXS40USB40PHY_RX_SD_STAT_OFFS           (0xccu >> 2u)
#define MXS40USB40PHY_RX_LD_CFG_OFFS            (0xd0u >> 2u)
#define MXS40USB40PHY_RX_LD_STAT_OFFS           (0xd4u >> 2u)
#define MXS40USB40PHY_RX_DFE_CFG4_OFFS          (0xd8u >> 2u)
#define MXS40USB40PHY_RX_HDWR_ENABLE_OFFS       (0xdcu >> 2u)
#define MXS40USB40PHY_RX_ATEST_CFG_OFFS         (0xe0u >> 2u)
#define MXS40USB40PHY_RX_DTEST_CFG_OFFS         (0xe4u >> 2u)
#define MXS40USB40PHY_SPARE_CFG_OFFS            (0xe8u >> 2u)
#define MXS40USB40PHY_SPARE_STAT_OFFS           (0xecu >> 2u)
#define MXS40USB40PHY_SPARE_HV_CFG_OFFS         (0xf0u >> 2u)

#define DATARATE_GEN1           (1u)
#define DATARATE_GEN2           (0u)

/* Map DPI API calls to firmware functions. */

#define DPI_wait_100ns()                                                                \
{                                                                                       \
    __NOP(); __NOP(); __NOP(); __NOP();                                                 \
    __NOP(); __NOP(); __NOP(); __NOP();                                                 \
    __NOP(); __NOP(); __NOP(); __NOP();                                                 \
}

#define DPI_csr_rd(addr, pRdData, dummy)                                                \
{                                                                                       \
    *pRdData = phyRxRegs_p[addr];                                                       \
}

#define DPI_csr_rd_mod_wr(addr, mask, wrData, pRdData, dummy)                           \
{                                                                                       \
    *pRdData = phyRxRegs_p[addr];                                                       \
    phyRxRegs_p[addr] = (((wrData) & (mask)) | (*(pRdData) & ~(mask)));                 \
}

int32_t ftn_max_dfe_tap(int32_t tap1, int32_t tap2, int32_t tap3)
{
    int32_t abs_tap1;
    int32_t abs_tap2;
    int32_t abs_tap3;
    int32_t cur_max;

    /* Note that this is a 7 bit unsigned value, we need to shift it around zero */
    abs_tap1 = tap1 - 64;
    abs_tap2 = tap2 - 64;
    abs_tap3 = tap3 - 64;

    /* Now get absolute value */
    if (abs_tap1 < 0) abs_tap1 = -abs_tap1;
    if (abs_tap2 < 0) abs_tap2 = -abs_tap2;
    if (abs_tap3 < 0) abs_tap3 = -abs_tap3;

    /* And find the biggest one */
    cur_max = abs_tap1;
    if( abs_tap2 > cur_max) cur_max = abs_tap2;
    if( abs_tap3 > cur_max) cur_max = abs_tap3;

    return cur_max;
}

const uint32_t P_LUT_5G[] =  {
    0x0000C3C3UL, 0x00008484UL, 0x0000B4B4UL, 0x00005454UL,
    0x0000D4D4UL, 0x00008686UL, 0x00006262UL, 0x0000C6C6UL,
    0x00006666UL, 0x00000808UL, 0x00004848UL, 0x00000A0AUL,
    0x0000E6E6UL, 0x00000404UL, 0x00002222UL, 0x0000C2C2UL
};

const uint32_t P_LUT_10G[] = {
    0x0000A2A2UL, 0x0000C2C2UL, 0x00003030UL, 0x0000C4C4UL,
    0x00002626UL, 0x0000C6C6UL, 0x0000C0C0UL, 0x00009898UL,
    0x0000C8C8UL, 0x00008A8AUL, 0x0000DADAUL, 0x0000DCDCUL,
    0x00000000UL, 0x00000404UL, 0x00000808UL, 0x00000202UL
};

#define AFE_CAL_RESULT_EYE_COMPLETE     (0x1A)
#define AFE_CAL_RESULT_LOCKDATA_FAILED  (0x1B)
#define AFE_CAL_RESULT_PLL_LOCK_LOST    (0x1C)
#define AFE_CAL_RESULT_SIGNAL_LOW       (0x1D)
#define AFE_CAL_RESULT_SIGNAL_HIGH      (0x1E)
#define AFE_CAL_RESULT_EYE_DITHERING    (0x1F)

void rx_offset_calibration (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                            volatile uint32_t *phyRxRegs_p,
                            bool onlyLock)
{
    uint32_t csr_rd_rtn_data;
    uint32_t ctle_config;
    uint32_t dfe_stat0, dfe_stat1;
    uint32_t afeosdac1, afeosdac2;

    DPI_csr_rd( MXS40USB40PHY_RX_OSA_STAT4_OFFS, &csr_rd_rtn_data, 1);
    afeosdac1 = csr_rd_rtn_data;

    if (!onlyLock) {
        /* Read and save the current CTLE and DFE TAP configuration. */
        DPI_csr_rd( MXS40USB40PHY_RX_CTLE_CFG1_OFFS, &csr_rd_rtn_data, 1);
        ctle_config = csr_rd_rtn_data;
        DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT0_OFFS, &csr_rd_rtn_data, 1);
        dfe_stat0 = csr_rd_rtn_data & 0x00007F00;
        DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT1_OFFS, &csr_rd_rtn_data, 1);
        dfe_stat1 = csr_rd_rtn_data & 0x00007F7F;

        DBG_SSCAL_TRACE("Repeating AFE offset calibration\r\n");
        /* Configure CTLE for offset calibration, note that we do not change the resistor values */
        /* <6:4> stg1_cap = 0 */
        /* <7> stg1_boost = 0 */
        /* <14:12> stg2_cap = 0 */
        /* <15> stg2_boost = 0 */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG1_OFFS, 0x0000F0F0, 0x00000000, &csr_rd_rtn_data, 1);

        /* Override DFE taps to a value of 64. */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG0_OFFS, 0x0000FF00, 0x0000C000, &csr_rd_rtn_data, 1);
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG1_OFFS, 0x0000FFFF, 0x0000C0C0, &csr_rd_rtn_data, 1);

        /* Adjust input common-mode for offset cancellation */
        /* <2:0> cm_adj=5 */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_AFE_CFG_OFFS, 0x00000007, 0x00000005, &csr_rd_rtn_data, 1);

        /* Launch only AFE offset calibration. */
        /* osa_afe_start=0, osa_resetb=1 */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_OSA_CFG5_OFFS, 0x00000006, 0x00000004, &csr_rd_rtn_data, 1);
        /* osa_afe_start=1, osa_resetb=1 */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_OSA_CFG5_OFFS, 0x00000006, 0x00000006, &csr_rd_rtn_data, 1);

        /* Wait for 100us for calibration to complete */
        Cy_SysLib_DelayUs(100UL);

        /* Confirm that calibration is completed. */
        DPI_csr_rd( MXS40USB40PHY_RX_OSA_STAT5_OFFS, &csr_rd_rtn_data, 1);
        if ((csr_rd_rtn_data & 0x00000002) == 0x00000002) {
            DBG_SSCAL_TRACE("[INFO]: Offset Routine complete detected\r\n");
        } else {
            DBG_SSCAL_INFO("[WARNING]: Offset complete not detected, proceeding anyways\r\n");
        }

        /* Restore input common-mode for normal operation */
        /* <2:0> cm_adj=0 */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_AFE_CFG_OFFS, 0x00000007, 0x00000000, &csr_rd_rtn_data, 1);

        /* Restore CTLE configuration */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG1_OFFS, 0x0000FFFF, ctle_config, &csr_rd_rtn_data, 1);

        /* Override the DFE taps to the previously determined values. */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG1_OFFS, 0x0000FFFF, 0x00008080 | dfe_stat1, &csr_rd_rtn_data, 1);
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG0_OFFS, 0x0000FF00, 0x00008000 | dfe_stat0, &csr_rd_rtn_data, 1);

        DPI_csr_rd( MXS40USB40PHY_RX_OSA_STAT4_OFFS, &csr_rd_rtn_data, 1);
        afeosdac2 = csr_rd_rtn_data;

        /* Force L2R then L2D and wait for lock. */
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000101, &csr_rd_rtn_data, 1);
        DPI_wait_100ns();
        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000501, &csr_rd_rtn_data, 1);
        do {
            DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
        } while ((csr_rd_rtn_data & 0x01) == 0);
        DPI_wait_100ns();
    }

    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000F07, 0x00000E00 | LOCKIN_SETTING,
            &csr_rd_rtn_data, 1);
    DPI_wait_100ns();
    do {
        DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
    } while ((csr_rd_rtn_data & 0x01) == 0);

    if ((!onlyLock) && (!pCalCtxt->deepSleepEntered)) {
        DBG_SSCAL_INFO("AFE cal result: Before=%x After=%x\r\n", afeosdac1, afeosdac2);
    }
}

/*
 * Algorithm implementation #1:
 *      1. Running for 7 LUT settings: C2C2, D2D2, D4D4, 6262, 6666, 0808 and 4848.
 *      2. lock_achieved being reset on each entry into state 2.
 *
 * Algorithm implementation #2:
 *      1. Updated State 401 implementation to try reducing DFE slicepoint when lock is lost.
 *
 * Algorithm implementation #3:
 *      1. Increased TSEQ transmit count from DUT side by 4X.
 *
 * Algorithm implementation #4:
 *      1. Set RX_DFE_CFG4.init_vgatap_value = 12 (from 8)
 *      2. Increased number of data words examined while measuring eye height to 2048 (from 128)
 *      3. Updated state 3022 to just reset the VGA/DFE engine rather than test for loss-of-lock
 *
 * Algorithm implementation #5:
 *      1. Reverted number of data words examined while measuring eye height to 128.
 *      2. Increased RAM log buffer size to 96 KB from 32 KB.
 *      3. Added offset calibration step at the end of CTLE adaptation.
 *      4. Updated code to use a TCPWM based timer (running at 1 MHz) to take timestamps.
 *
 * Algorithm implementation #6:
 *      1. Clear VGA and DFE toggle counts after updating DFE slicepoint.
 *      2. Increased RAM log buffer size to 96 KB from 32 KB.
 *      3. Updated code to use a TCPWM based timer (running at 1 MHz) to take timestamps.
 *
 * Algorithm implementation #8:
 *      1. Added offset calibration step at the end of CTLE adaptation.
 *
 * Algorithm implementation #9:
 *      1. Added option to pick a fixed CTLE setting when operating in GEN2 mode.
 *
 * Algorithm implementation #10:
 *      1. Updated LUT mask scheme.
 */
#define CTLE_ALGORITHM_VERSION  0x000Au
#define LUT_MASK_GEN1X1 (0x0FFF) /* LUT index 0-11 */
#define LUT_MASK_GEN2X1 (0xFFFF) /* LUT index 0-15 */
#define LUT_MASK_GEN1X2 (0x3B09) /* LUT index 0,3,8,9,11,12,13 */
#define MIN_LUT_GEN1X2  (0x1B00) /* LUT index 8,9,11,12 */
#define LUT_MASK_GEN2X2 (0x03FF) /* LUT index 0-9 */

/* Using a global variable to avoid adding a large array to stack. */
static uint32_t RxEyeErrCnt[LUT_WIDTH][N_EYE_LEVELS] = {{0}};

int32_t z_dfe_ctle_adaptation (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t data_rate, uint8_t phyIndex)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    volatile uint32_t *phyRxRegs_p =
        (volatile uint32_t *)(&(base->USB32DEV_PHYSS.USB40PHY[phyIndex].USB40PHY_RX));
    DBG_SSCAL_TRACE("CTLE:PHY:%d,Rate:%d\r\n",phyIndex,data_rate);

    uint32_t lut_index;                                 /* LUT Index */
    uint32_t cur_lut;                                   /* Current LUT entry */
    uint32_t cur_cap_gain = 0x1000;                     /* Current cap_gain_adj entry */
    uint32_t final_cap_gain[LUT_WIDTH];                 /* Actual cap gain value used for a given LUT entry */

    uint32_t eye_height [] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    };                                                  /* Measured eye height */
    int32_t final_vga[LUT_WIDTH];                       /* Actual VGA used for a given LUT entry */
    int32_t final_dfe1[LUT_WIDTH];                      /* Actual DFE Tap 1 used for a given LUT entry */
    int32_t final_dfe2[LUT_WIDTH];                      /* Actual DFE Tap 2 used for a given LUT entry */
    int32_t final_dfe3[LUT_WIDTH];                      /* Actual DFE Tap 3 used for a given LUT entry */
    int32_t eye_sweep_index = 0;                        /* Eye Sweep Index; */
    int32_t clk_cnt;                                    /* Number of timer clocks while waiting for timeout */
    int32_t tmp_index;                                  /* Temporary loop index */
    int32_t x_index;                                    /* Temporary loop index */
    uint32_t tmp_eye;                                   /* Temporary eye height value */
    uint32_t bcs_eye = 0;                               /* Best eye opening */
    uint32_t bcs_index = 0;                             /* Best LUT Index */
    int32_t cur_vga = 8;                                /* Current VGA value */
    int32_t cur_dfe1 = 64;                              /* Current Tap 1 DFE value */
    int32_t cur_dfe2 = 64;                              /* Current Tap 2 DFE value */
    int32_t cur_dfe3 = 64;                              /* Current Tap 3 DFE value */
    int32_t prior_vga = 8;                              /* Previous VGA value */
    int32_t prior_dfe1 = 64;                            /* Previous Tap 1 DFE value */
    int32_t prior_dfe2 = 64;                            /* Previous Tap 2 DFE value */
    int32_t prior_dfe3 = 64;                            /* Previous Tap 3 DFE value */
    int32_t pp_vga = 8;                                 /* Previous VGA value */
    int32_t pp_dfe1 = 64;                               /* Previous Tap 1 DFE value */
    int32_t pp_dfe2 = 64;                               /* Previous Tap 2 DFE value */
    int32_t pp_dfe3 = 64;                               /* Previous Tap 3 DFE value */
    int32_t vga_toggle_cnt = 0;                         /* Count of VGA toggles */
    int32_t dfe1_toggle_cnt = 0;                        /* Count of DFE Tap 1 toggles */
    int32_t dfe2_toggle_cnt = 0;                        /* Count of DFE Tap 2 toggles */
    int32_t dfe3_toggle_cnt = 0;                        /* Count of DFE Tap 3 toggles */
    int32_t eye_height_thres = EYE_LEVEL_START;         /* Eye Height threshold, swept from 0-64 */
    int32_t cur_err_cnt;                                /* Reading of eye height error count after loop completes */
    int32_t lcl_adapt_done;                             /* Local adapt_done signal */
    int32_t sm_state;                                   /* Current State Machine state */
    int32_t cur_dfe_slicepoint = MAX_SLICEPOINT;        /* Current DFE Slice Point */
    int32_t lock_achieved;                              /* Lock State */
    int32_t min_abs_dfe_tap;                            /* Minimum abs Sum of all DFE taps */
    int32_t new_abs_dfe_tap;                            /* Minimum abs Sum of all DFE taps */
    uint32_t csr_rd_rtn_data;                           /* Temp variable to hold register value */
    uint32_t lut_check_mask = 0;

    if (pCalCtxt->activeLutMask != 0) {
        lut_check_mask = pCalCtxt->activeLutMask;
    } else {
        if (pCalCtxt->dualLaneEnabled) {
            lut_check_mask = (data_rate == DATARATE_GEN1) ? LUT_MASK_GEN1X2 : LUT_MASK_GEN2X2;
        } else {
            lut_check_mask = (data_rate == DATARATE_GEN1) ? LUT_MASK_GEN1X1 : LUT_MASK_GEN2X1;
        }
    }

    (void)final_vga;
    (void)lock_achieved;

    /* Initialize testbench variables */
    lut_index      = 0;
    clk_cnt        = 0;
    lcl_adapt_done = 0;
    sm_state       = 1;
    lcl_adapt_done = 0;

    /* Initialize the eye error count array. */
    for (x_index = 0; x_index < LUT_WIDTH; x_index++) {
        for (tmp_index = 0; tmp_index < N_EYE_LEVELS; tmp_index++) {
            RxEyeErrCnt[x_index][tmp_index] = FAIL_LIMIT * 10;
        }
    }

    while ((sm_state != 0) && (lcl_adapt_done == 0))
    {
        switch (sm_state)
        {
            case 1:
                DBG_SSCAL_TRACE("---------------------------------------\r\n");
                DBG_SSCAL_TRACE("Initialize CTLE/DFE Calibration Routine (version %x)\r\n", CTLE_ALGORITHM_VERSION);
                DBG_SSCAL_TRACE("PLL_LD_STAT=%x\r\n",
                        base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS.PLL_LD_STAT);
                lut_index = 0;                                  /* Start at begining of LUT */
                eye_height_thres = EYE_LEVEL_START;             /* Start eye sweep with an offset */
                lock_achieved = 0;                              /* Assume we're unlocked at the start */
                clk_cnt = 0;                                    /* Reset delay counter */

                /* Reduce tap_update_threshold */
                /* <7:5> tap_update_thrshld=3'b001 */

                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG4_OFFS, 0x000000E0, 0x00000020, &csr_rd_rtn_data, 1);

                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x00000F00, 0x00000300, &csr_rd_rtn_data, 1);//we added for vga mu
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000F000, 0x00007000, &csr_rd_rtn_data, 1);//we added for dfe mu

                DPI_csr_rd_mod_wr(MXS40USB40PHY_RX_DFEA_CFG0_OFFS,0x0000FFFF, 0x0000CCCE,&csr_rd_rtn_data, 1); //dfea_cfg

                if (data_rate == DATARATE_GEN1) {
                    /* Disable DFE odd path summer in Gen1 case. */
                    /* <10> sumodd_pdb = 0 */
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_GNRL_CFG_OFFS, 0x00000400, 0x00000000, &csr_rd_rtn_data, 1);
                }

                /* Adjust DFE SM Settings */
                /* <2:0> dfe_bal_avrg = 3'b100 (Number of data word checked for each VGA/DFE decision) */
                /* <9> vga_adapt_en = 1 */
                /* <10> dfe_enable = 1 */
                /* <11> vga_freeze = 0 */
                /* <12> dfe_freeez = 0 */
                /* <14> dfe_resetb = 1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E07, 0x00004604, &csr_rd_rtn_data, 1);
                /* Start with cap_gain = 1 (high gain) */
                cur_cap_gain = 0x00001000;
                sm_state = 2;        /* Next step */

                /* Set cmadj = 0 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_AFE_CFG_OFFS, 0x00000007, 0x00000000, &csr_rd_rtn_data, 1);
                break;

            case 2:
                while(lut_index < (LUT_WIDTH - 1))
                {
                    if((1 << lut_index) & lut_check_mask)
                    {
                        break;
                    }
                    else
                    {
                        lut_index++;
                    }
                }

                /* State - 2 - Set up for next LUT entry */
                DBG_SSCAL_TRACE("Next LUT entry: %d CapGain:%d\r\n", lut_index, cur_cap_gain);

                /* Update from LUT */
                if( data_rate == DATARATE_GEN2 ) {
                    cur_lut = P_LUT_10G[ lut_index];
                } else {
                    cur_lut = P_LUT_5G[ lut_index];
                }

                /* Load CTLE settings */
                /* <3:0> stg1_reg */
                /* <6:4> stg1_cap */
                /* <7> stg1_boost */
                /* <11:8> stg2_reg */
                /* <14:12> stg2_cap */
                /* <15> stg2_boost */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG1_OFFS, 0x0000FFFF, cur_lut, &csr_rd_rtn_data, 1);
                /* <0> ctle_eyeht_en = 0 */
                /* <1> eyeht_start_cnt = 0 */
                /* <12> ctle_cap_gain_adj */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00001003, cur_cap_gain, &csr_rd_rtn_data, 1);

                /* Toggle VGA/DFE Enable */
                /* <9> vga_adapt_en = 0 */
                /* <10> dfe_enable = 0 */
                /* <11> vga_freeze = 0 */
                /* <12> dfe_freeez = 0 */
                /* <14> dfe_resetb = 0 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00000000, &csr_rd_rtn_data, 1);
                DPI_wait_100ns();

                /* Set dfe_slicepoint */
                /* <5:0> dfe_slicepoint */
                cur_dfe_slicepoint = MAX_SLICEPOINT;
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, cur_dfe_slicepoint, &csr_rd_rtn_data, 1);

                /* Enable DFE Hardware */
                /* <3> dfe_pdb=1 */
                /* <7> dfetap1_pdb=!data_rate */
                /* <11> dfetap2_pdb=1 */
                /* <15> dfetap3_pdb=!data_rate */
                if( data_rate == DATARATE_GEN1 ) {
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFEA_CFG0_OFFS, 0x00008888, 0x00000808, &csr_rd_rtn_data, 0);
                } else {
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFEA_CFG0_OFFS, 0x00008888, 0x00008888, &csr_rd_rtn_data, 0);
                }

                /* Enable VGA/DFE */
                /* <9> vga_adapt_en = 1 */
                /* <10> dfe_enable = 1 */
                /* <11> vga_freeze = 0 */
                /* <12> dfe_freeez = 0 */
                /* <14> dfe_resetb = 1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00004600, &csr_rd_rtn_data, 1);

                clk_cnt = 0;
                prior_vga = 8;    /* Initial Previous VGA value  */
                pp_vga = 8;       /* Initial Previous VGA value  */
                prior_dfe1 = 64;  /* Initial Previous Tap 1 DFE value */
                pp_dfe1 = 64;     /* Initial Previous Tap 1 DFE value */
                prior_dfe2 = 64;  /* Initial Previous Tap 2 DFE value */
                pp_dfe2 = 64;     /* Initial Previous Tap 2 DFE value */
                prior_dfe3 = 64;  /* Initial Previous Tap 3 DFE value */
                pp_dfe3 = 64;     /* Initial Previous Tap 3 DFE value */

                lock_achieved = 0;
                sm_state = 3020;        /* Run L2R step. */
                break;

            case 3020:
                /* Force L2R mode and toggle Lock Detect */
                DBG_SSCAL_TRACE("Init PLL L2R lock\r\n");
                /* Force L2R Mode, Toggle Reset in lock detector */
                /* <2:0> lockin, default=001b (0.05% accuracy) */
                /* <9:8> fd_ovrd, default=00b, set to 01b (L2R Mode) */
                /* <10> resetlockb, default=0, toggle to reset lock detect */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000101, &csr_rd_rtn_data, 1);
                DPI_wait_100ns();
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000501, &csr_rd_rtn_data, 1);
                sm_state = 3021;
                clk_cnt = 0;
                break;

            case 3021:
                /* Wait for PLL Lock */

                /* Read lock status */
                DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
                /* If we're locked, move on, otherwise wait */
                if( (csr_rd_rtn_data & 0x01) == 1) {
                    DBG_SSCAL_TRACE("L2R lock achieved\r\n");
                    sm_state = 3;
                    lock_achieved = 1;
                }

                /* If watchdog timer times out, abort */
                if( clk_cnt > 1000) {
                    DBG_SSCAL_TRACE("PLL L2R lock failed\r\n");
                    sm_state = 9999;
                }

                clk_cnt = clk_cnt+1;
                break;

            case 3:
                /* Force L2D and relax lock detect sensitivity */
                DBG_SSCAL_TRACE("Init PLL L2D lock\r\n");

                /* Force L2D Mode, but do not toggle Reset in lock detector */
                /* <2:0> lockin, default=001b (0.05% accuracy), relax to 0.4%, lockin=100b */
                /* <9:8> fd_ovrd, default=00b, set to 10b (L2D Mode) */
                /* <10> resetlockb, default=0, leave on = 1 */
                /* <11> ld_resetb=1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000F07, 0x00000E00 | LOCKIN_SETTING,
                        &csr_rd_rtn_data, 1);
                vga_toggle_cnt = 0;
                dfe1_toggle_cnt = 0;
                dfe2_toggle_cnt = 0;
                dfe3_toggle_cnt = 0;
                clk_cnt = 0;
                sm_state = 3022;
                break;

            case 3022:
                /* Wait for PLL Lock */
                /* Initialize local variables */
                cur_vga = 8;
                cur_dfe1 = 64;
                cur_dfe2 = 64;
                cur_dfe3 = 64;

                /* Read lock status */
                DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);

                /* If we're locked, move on, otherwise wait */
                if( (csr_rd_rtn_data & 0x01) == 1) {
                    DBG_SSCAL_TRACE("L2D lock achieved\r\n");
                    /* Reset VGA/DFE State Machine */
                    /* <9> vga_adapt_en = 0/1 */
                    /* <10> dfe_enable = 0/1 */
                    /* <11> vga_freeze = 0 */
                    /* <12> dfe_freeez = 0 */
                    /* <14> dfe_resetb = 0/1 */
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00000000, &csr_rd_rtn_data, 1);
                    DPI_wait_100ns();
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00004600, &csr_rd_rtn_data, 1);
                    sm_state = 4;
                    clk_cnt  = 0;
                    lock_achieved = 1;
                }

                /* If watchdog timer times out, abort */
                if( clk_cnt > 1000) {
                    DBG_SSCAL_TRACE("PLL L2D lock failed\r\n");
                    for( tmp_index=0; tmp_index <= N_EYE_LEVELS-1; tmp_index=tmp_index+1) {
                        RxEyeErrCnt[lut_index][tmp_index]=FAIL_LIMIT*10;
                    }
                    lock_achieved = 0;
                    final_cap_gain[ lut_index] = cur_cap_gain;
                    final_vga[ lut_index] = cur_vga;
                    final_dfe1[ lut_index] = cur_dfe1;
                    final_dfe2[ lut_index] = cur_dfe2;
                    final_dfe3[ lut_index] = cur_dfe3;
                    pCalCtxt->phyResult[phyIndex][lut_index] = AFE_CAL_RESULT_LOCKDATA_FAILED;
                    sm_state = 20; /* Skip eye height measurement */
                }

                clk_cnt = clk_cnt+1;
                break;

            case 4:
                /* 500 cycles of 20 ns ==> 10 us delay ==> Increased to 20 us. */
                Cy_SysLib_DelayUs(20);
                sm_state = 401;
                DBG_SSCAL_TRACE("S4: DFE slicept = %d\r\n", cur_dfe_slicepoint);
                break;

            case 401:
                pp_vga = prior_vga;
                prior_vga = cur_vga;
                pp_dfe1 = prior_dfe1;
                prior_dfe1 = cur_dfe1;
                pp_dfe2 = prior_dfe2;
                prior_dfe2 = cur_dfe2;
                pp_dfe3 = prior_dfe3;
                prior_dfe3 = cur_dfe3;
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT0_OFFS, &csr_rd_rtn_data, 1);
                cur_vga = csr_rd_rtn_data & 0x1F; /* Current VGA value */
                cur_dfe1 = (csr_rd_rtn_data >> 8) & 0x7F; /* Current DFE Tap 1 value */
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT1_OFFS, &csr_rd_rtn_data, 1);
                cur_dfe2 = csr_rd_rtn_data & 0x7F; /* Current DFE Tap 2 value */
                cur_dfe3 = (csr_rd_rtn_data >> 8) & 0x7F; /* Current DFE Tap 3 value */

                DBG_SSCAL_TRACE("S401: VGACalDone: Cur_DFE2=%x Prev_DFE2=%x Cur_VGA=%x Prev_VGA=%x\r\n",
                        cur_dfe2, prior_dfe2, cur_vga, prior_vga);

                /* Check for toggling */
                if( ( (pp_vga < prior_vga)&&(cur_vga < prior_vga))||( (pp_vga > prior_vga)&&(cur_vga > prior_vga)) ) {
                    vga_toggle_cnt = vga_toggle_cnt+1;
                }
                if( ( (pp_dfe1 < prior_dfe1)&&(cur_dfe1 < prior_dfe1))||( (pp_dfe1 > prior_dfe1)&&(cur_dfe1 > prior_dfe1)) ) {
                    dfe1_toggle_cnt = dfe1_toggle_cnt+1;
                }
                if( ( (pp_dfe2 < prior_dfe2)&&(cur_dfe2 < prior_dfe2))||( (pp_dfe2 > prior_dfe2)&&(cur_dfe2 > prior_dfe2)) ) {
                    dfe2_toggle_cnt = dfe2_toggle_cnt+1;
                }
                if( ( (pp_dfe3 < prior_dfe3)&&(cur_dfe3 < prior_dfe3))||( (pp_dfe3 > prior_dfe3)&&(cur_dfe3 > prior_dfe3)) ) {
                    dfe3_toggle_cnt = dfe3_toggle_cnt+1;
                }

                /* Read lock status in case we lose lock */
                DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
                /* If we're locked, move on, otherwise wait */
                if( (csr_rd_rtn_data & 0x01) == 0) {
                    /* If we haven't looked at the cap_gain_adj = 0, try that, otherwise skip measurement */
                    if( cur_cap_gain == 0x00001000) {
                        if (cur_dfe_slicepoint > MIN_SLICEPOINT) {
                            if ( data_rate == DATARATE_GEN1 )
                                cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G1;
                            else
                                cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G2;
                            DBG_SSCAL_TRACE("S401: PLL lock lost: set cur_dfe_spicepoint = %x\r\n", cur_dfe_slicepoint);

                            /* Set dfe_slicepoint */
                            /* <5:0> dfe_slicepoint */
                            DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, cur_dfe_slicepoint, &csr_rd_rtn_data, 1);
                            lock_achieved = 0;
                            sm_state = 3020;
                        } else {
                            DBG_SSCAL_TRACE("S401: PLL lock lost: set cap_gain=0\r\n");
                            cur_dfe_slicepoint = MAX_SLICEPOINT;
                            cur_cap_gain = 0x00000000;
                            lock_achieved = 0;
                            sm_state = 2;
                        }
                    } else {
                        DBG_SSCAL_TRACE("S401: PLL lock lost: record zero eye\r\n");
                        for( tmp_index=0; tmp_index <= N_EYE_LEVELS-1; tmp_index=tmp_index+1) {
                            RxEyeErrCnt[lut_index][tmp_index]=FAIL_LIMIT*10;
                        }
                        lock_achieved = 0;
                        final_cap_gain[ lut_index] = cur_cap_gain;
                        final_vga[ lut_index] = cur_vga;
                        final_dfe1[ lut_index] = cur_dfe1;
                        final_dfe2[ lut_index] = cur_dfe2;
                        final_dfe3[ lut_index] = cur_dfe3;
                        pCalCtxt->phyResult[phyIndex][lut_index] = AFE_CAL_RESULT_PLL_LOCK_LOST;
                        sm_state = 20; /* Skip eye height measurement */
                    }
                    /* If the VGA setting has changed, reset the timer and try again */
                } else if( (vga_toggle_cnt > 3)|| (dfe1_toggle_cnt > 5)|| (dfe2_toggle_cnt > 5) || (dfe3_toggle_cnt > 5) ) {
                    /* The VGA/DFE code is dithering, jump out */
                    DBG_SSCAL_TRACE("S401: Opt DFE slicept=%x VGA value=%x\r\n", cur_dfe_slicepoint, cur_vga);
                    sm_state = 5;
                    /* Store cap gain */
                    final_cap_gain[ lut_index] = cur_cap_gain;
                    final_vga[ lut_index] = cur_vga;
                    final_dfe1[ lut_index] = cur_dfe1;
                    final_dfe2[ lut_index] = cur_dfe2;
                    final_dfe3[ lut_index] = cur_dfe3;
                    pCalCtxt->phyResult[phyIndex][lut_index] = (cur_cap_gain != 0) ?
                        (0x80 | AFE_CAL_RESULT_EYE_DITHERING) : AFE_CAL_RESULT_EYE_DITHERING;
                } else if( (prior_vga != cur_vga)||( ((prior_dfe1 != cur_dfe1)||(prior_dfe2 != cur_dfe2)||(prior_dfe3 != cur_dfe3))&&(cur_vga != 0)&&(cur_vga != 15) ) ) {
                    /* Someone is changing, stay put unless we've hit the minimum or maximum VGA setting */
                    clk_cnt = 0;
                    sm_state = 4;
                } else {
                    /* We should only reach this point once everything is stable */
                    if( (cur_vga <= MIN_VGA) && (cur_cap_gain == 0x00001000) ) {
                        /* If VGA setting is too low (input signal is too big) and cap_gain=1, try cap_gain=0 */
                        DBG_SSCAL_TRACE("S401: Reducing cap_gain\r\n");
                        cur_dfe_slicepoint = MAX_SLICEPOINT;
                        /* <12> ctle_cap_gain_adj */
                        cur_cap_gain = 0x00000000;
                        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00001000, cur_cap_gain, &csr_rd_rtn_data, 1);
                        sm_state = 2;
                    } else if( (cur_vga >= MAX_VGA) && (cur_dfe_slicepoint > MIN_SLICEPOINT) ) {
                        /* If VGA setting is too high (input signal is too small) reduce slice point */
                        if ( data_rate == DATARATE_GEN1 )
                            cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G1;
                        else
                            cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G2;
                        /* Set dfe_slicepoint */
                        /* <5:0> dfe_slicepoint */
                        DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, cur_dfe_slicepoint, &csr_rd_rtn_data, 1);
                        DBG_SSCAL_TRACE("S401: Signal too small, reducing cur_dfe_slicepoint = %x\r\n", cur_dfe_slicepoint);
                        /* Reset toggle counts. */
                        vga_toggle_cnt  = 0;
                        dfe1_toggle_cnt = 0;
                        dfe2_toggle_cnt = 0;
                        dfe3_toggle_cnt = 0;

                        clk_cnt = 0;
                        sm_state = 4;
                    } else if( (cur_vga >= MAX_VGA) && (cur_dfe_slicepoint == MIN_SLICEPOINT) ) {
                        /* If VGA setting is too high (input signal is too small) there is nothing more we can do, abort measurement */
                        DBG_SSCAL_TRACE("S401: Signal too small, record zero eye\r\n");
                        for( tmp_index=0; tmp_index <= N_EYE_LEVELS-1; tmp_index=tmp_index+1) {
                            RxEyeErrCnt[lut_index][tmp_index]=FAIL_LIMIT*10;
                        }
                        final_cap_gain[ lut_index] = cur_cap_gain;
                        final_vga[ lut_index] = cur_vga;
                        final_dfe1[ lut_index] = cur_dfe1;
                        final_dfe2[ lut_index] = cur_dfe2;
                        final_dfe3[ lut_index] = cur_dfe3;
                        pCalCtxt->phyResult[phyIndex][lut_index] = AFE_CAL_RESULT_SIGNAL_LOW;
                        sm_state = 20; /* Skip eye height measurement */
                    } else if( (cur_vga <= MIN_VGA) && (cur_cap_gain == 0x00000000) ) {
                        /* If VGA setting is too low (input signal is too big) and we can't reduce cap_gain, abort measurement */
                        DBG_SSCAL_TRACE("S401: Signal too big, record zero eye\r\n");
                        for( tmp_index=0; tmp_index <= N_EYE_LEVELS-1; tmp_index=tmp_index+1) {
                            RxEyeErrCnt[lut_index][tmp_index]=FAIL_LIMIT*10;
                        }
                        final_cap_gain[ lut_index] = cur_cap_gain;
                        final_vga[ lut_index] = cur_vga;
                        final_dfe1[ lut_index] = cur_dfe1;
                        final_dfe2[ lut_index] = cur_dfe2;
                        final_dfe3[ lut_index] = cur_dfe3;
                        pCalCtxt->phyResult[phyIndex][lut_index] = AFE_CAL_RESULT_SIGNAL_HIGH;
                        sm_state = 20; /* Skip eye height measurement */
                    } else {
                        /* We're good */
                        DBG_SSCAL_TRACE("S401: Opt DFE slicept=%x VGA value=%x Cap_gain=%x DFE Coeff=%x %x %x\r\n", cur_dfe_slicepoint, cur_vga, cur_cap_gain, cur_dfe1, cur_dfe2, cur_dfe3);
                        sm_state = 5;
                        /* Store cap gain */
                        final_cap_gain[ lut_index] = cur_cap_gain;
                        final_vga[ lut_index] = cur_vga;
                        final_dfe1[ lut_index] = cur_dfe1;
                        final_dfe2[ lut_index] = cur_dfe2;
                        final_dfe3[ lut_index] = cur_dfe3;
                        pCalCtxt->phyResult[phyIndex][lut_index] = (cur_cap_gain != 0) ?
                            (0x80 | AFE_CAL_RESULT_EYE_COMPLETE) : AFE_CAL_RESULT_EYE_COMPLETE;
                    }
                }
                break;

            case 5:
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT2_OFFS, &csr_rd_rtn_data, 1);
                DBG_SSCAL_TRACE("S5: RX_DFE_STAT2   = %x\r\n", csr_rd_rtn_data);

                /* Freeze VGA, double check that we're in lock (in L2D mode) */
                /* Freeze VGA Loop */
                /* <9> vga_adapt_en = 1 */
                /* <10> dfe_enable = 1 */
                /* <11> vga_freeze = 1 */
                /* <12> dfe_freeez = 1 */
                /* <14> dfe_resetb = 1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00005E00, &csr_rd_rtn_data, 1);
                /* Initialize stuff, next step is the actual sweeping loop */
                /* Enable Eye Height Engine */
                /* <0> ctle_eyeht_en = 1 (note, leave eyeht_start_cnt zero for now) */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00000001, 0x00000001, &csr_rd_rtn_data, 1);
                /* Initialize eye center */
                eye_height_thres = EYE_LEVEL_START;
                /* Initial eye sweep index */
                eye_sweep_index = 0;
                sm_state = 7; /* Move on to next state */
                break;

            case 7:
                /* Start eye height sweep */
                DBG_SSCAL_TRACE("S7: Measuring eye height\r\n");
                /* Set Error Slicer Tresholds */
                /* dfe_slicepoint = eye_height_thres */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, eye_height_thres, &csr_rd_rtn_data, 1);
                /* Reset counter */
                /* <1> eyeht_start_cnt = 0 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00000002, 0x00000000, &csr_rd_rtn_data, 1);
                sm_state = 8; /* Next state */
                break;

            case 8:
                /* 50 loops of 20ns ==> 1 us delay. */
                Cy_SysLib_DelayUs(1);
                sm_state = 9; /* Next State */
                break;

            case 9:
                /* Start count */
                /* <1> eyeht_start_cnt = 1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00000002, 0x00000002, &csr_rd_rtn_data, 1);

                /* Needs a delay to prevent reading the previous done signal */
                Cy_SysLib_DelayUs(1);
                sm_state = 10; /* Next state */
                break;

            case 10:
                /* Wait for eye height measurement to complete then record result */
                DPI_csr_rd( MXS40USB40PHY_RX_CTLE_STAT1_OFFS, &csr_rd_rtn_data, 1);
                if( (csr_rd_rtn_data & 0x00000001) > 0x00000000) {
                    /* Count is complete */
                    DPI_csr_rd( MXS40USB40PHY_RX_CTLE_STAT0_OFFS, &csr_rd_rtn_data, 1);
                    cur_err_cnt = 0x0000FFFF & csr_rd_rtn_data;
                    RxEyeErrCnt[lut_index][eye_sweep_index] = cur_err_cnt;
                    DBG_SSCAL_TRACE("S10: Eye error count=%x at index=%x\r\n", cur_err_cnt, eye_sweep_index);
                    if( eye_sweep_index < N_EYE_LEVELS-1) {
                        /* If the number of errors is > FAIL_LIMIT, fill out the rest of the err_cnt slice and skip to the next entry */
                        if( (cur_err_cnt > FAIL_LIMIT)&&(FULL_SWEEP == 0) ) {
                            for( x_index = eye_sweep_index+1; x_index <= N_EYE_LEVELS-1; x_index++) {
                                RxEyeErrCnt[lut_index][x_index] = cur_err_cnt;
                            }
                            sm_state = 11; /* Next State */
                        } else {
                            /* Increment eye sweep and re-run */
                            eye_sweep_index = eye_sweep_index + 1;
                            eye_height_thres = eye_height_thres + DELTA_EYE_LEVEL;
                            sm_state = 7; /* Next State */
                        }
                    } else {
                        /* We're done, time for clean-up */
                        sm_state = 11; /* Next State */
                    }
                } else {
                    /* Check for loss of lock since that can cause some issues with the Eye Height measurement completion */
                    DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
                    /* If we're unlocked, clear status, fill in Eye height count and jump to next state */
                    if( (csr_rd_rtn_data & 0x01) == 0)  {
                        lock_achieved = 0;
                        DBG_SSCAL_TRACE("S10: Lock lost, record zero eye\r\n");
                        for( x_index = eye_sweep_index+1; x_index <= N_EYE_LEVELS-1; x_index++) {
                            RxEyeErrCnt[lut_index][x_index] = 5*FAIL_LIMIT;
                        }
                        sm_state = 11; /* Next State */
                    }
                }
                break;

            case 11:
                DBG_SSCAL_TRACE("S11: Eye sweep complete\r\n");
                /* Figure out eye height */
                tmp_index = 0;
                tmp_eye = N_EYE_LEVELS-1;
                /* We're looking for the lower, non-zero index in the err_cnt array */
                for( tmp_index=N_EYE_LEVELS-1; tmp_index >= 0; tmp_index=tmp_index-1) {
                    if( RxEyeErrCnt[lut_index][tmp_index] > 0) {
                        /* This entry is non-zero */
                        tmp_eye = tmp_index;
                    }
                }
                eye_height[ lut_index] = tmp_eye;

                sm_state = 20; /* Next State */
                break;

            case 20:
                lut_index = lut_index + 1;
                if (lut_index > (LUT_WIDTH - 1)) {
                    DBG_SSCAL_TRACE("S20: CTLE sweep complete\r\n");
                    sm_state = 21; /* All Done, go to next State */
                } else {
                    /* If current LUT entry is enabled, go to the next one. */
                    if (((1 << lut_index) & lut_check_mask) == 0) {
                        sm_state = 20;
                        break;
                    }

                    /* Check lock status */
                    DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
                    /* If we're unlocked, clear status */
                    if( (csr_rd_rtn_data & 0x01) == 0) lock_achieved = 0;
                    /* Start with maximum cap gain */
                    cur_cap_gain = 0x00001000;
                    sm_state = 2; /* Go back and run another one */
                }
                break;

            case 21:
                /* Find best LUT index */
                /* In the event of a tie, select one with minimum largest DFE coefficient */
                for ( tmp_index = LUT_WIDTH - 1; tmp_index >= 0; tmp_index = tmp_index - 1 ) {
                    if (((1U << tmp_index) & lut_check_mask) != 0) {
                        bcs_index = tmp_index;
                        break;
                    }
                }

                bcs_eye = 0;
                min_abs_dfe_tap = 64;
                for( tmp_index = bcs_index; tmp_index >= 0; tmp_index = tmp_index - 1 ) {
                    /* Don't consider masked out settings in the table. */
                    if (((1U << tmp_index) & lut_check_mask) == 0) {
                        continue;
                    }

                    if( eye_height[ tmp_index] > bcs_eye) {
                        /* This eye is better than all previous ones */
                        bcs_eye = eye_height[ tmp_index];
                        bcs_index = tmp_index;
                        new_abs_dfe_tap = ftn_max_dfe_tap( final_dfe1[ tmp_index], final_dfe2[ tmp_index], final_dfe3[ tmp_index]);
                    } else if((eye_height[ tmp_index] == bcs_eye) && (bcs_eye != 0)) {
                        /* We're tied, see if this one has a lower DFE tap(s) */
                        new_abs_dfe_tap = ftn_max_dfe_tap( final_dfe1[ tmp_index], final_dfe2[ tmp_index], final_dfe3[ tmp_index]);
                        if( new_abs_dfe_tap < min_abs_dfe_tap) {
                            /* Yes, this one has less DFE EQ, use it */
                            bcs_eye = eye_height[ tmp_index];
                            bcs_index = tmp_index;
                            min_abs_dfe_tap = new_abs_dfe_tap;
                        }
                    } else {
                        /* Temporary */
                        new_abs_dfe_tap = ftn_max_dfe_tap( final_dfe1[ tmp_index], final_dfe2[ tmp_index], final_dfe3[ tmp_index]);
                    }
                }

                if ( data_rate == DATARATE_GEN1 ) {
                    /* If eye height is zero, always select LUT-8 for Gen1 operation. */
                    if ( bcs_eye == 0 ) {
                        bcs_index = 8;
                    }
                    DBG_SSCAL_INFO("PHY:%d Gen1-Adap: Idx=%x Height=%x\r\n", phyIndex, bcs_index, bcs_eye);
                } else {
                    DBG_SSCAL_INFO("Phy:%d Gen2-Adap: Idx=%x Height=%x\r\n", phyIndex, bcs_index, bcs_eye);
                }

                DBG_SSCAL_TRACE("Best eye opening = %x code at index %x\r\n", bcs_eye, bcs_index);

                /* Update CTLE Registers  */
                cur_cap_gain = final_cap_gain[ bcs_index];
                if( data_rate == DATARATE_GEN2 ) {
                    cur_lut = P_LUT_10G[ bcs_index];
                } else {
                    cur_lut = P_LUT_5G[ bcs_index];
                }

                /* Load CTLE settings */
                /* <3:0> stg1_reg */
                /* <6:4> stg1_cap */
                /* <7> stg1_boost */
                /* <11:8> stg2_reg */
                /* <14:12> stg2_cap */
                /* <15> stg2_boost */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG1_OFFS, 0x0000FFFF, cur_lut, &csr_rd_rtn_data, 0);
                /* Turn off eye height engine and set ctle_cap_gain */
                /* <0> ctle_eyeht_en = 0 */
                /* <1> eyeht_start_cnt = 0 */
                /* <12> ctle_cap_gain_adj */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_CTLE_CFG0_OFFS, 0x00001003, cur_cap_gain, &csr_rd_rtn_data, 0);
                /* Reset VGA/DFE State Machine */
                /* <9> vga_adapt_en = 0 */
                /* <10> dfe_enable = 0 */
                /* <11> vga_freeze = 0 */
                /* <12> dfe_freeez = 0 */
                /* <14> dfe_resetb = 0 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00000000, &csr_rd_rtn_data, 1);

                /* Print Best Case results */
                DBG_SSCAL_TRACE("Best regval: RX_CTLE_CFG0=%x RX_CTLE_CFG1=%x\r\n", cur_cap_gain, cur_lut);

                /* We're done, Run VGA/DFE_SLICEPOINT Adaptation loop one last time with best LUT value */
                clk_cnt = 0;
                sm_state = 2201;
                break;

            case 2201:
                /* Force L2R Mode, Toggle Reset in lock detector */
                /* <2:0> lockin, default=001b (0.05% accuracy) */
                /* <9:8> fd_ovrd, default=00b, set to 01b (L2R Mode) */
                /* <10> resetlockb, default=0, toggle to reset lock detect */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000101, &csr_rd_rtn_data, 1);
                DPI_wait_100ns();
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000501, &csr_rd_rtn_data, 1);
                sm_state = 2202;
                clk_cnt = 0;
                break;

            case 2202:
                /* Wait for PLL Lock */
                /* Read lock status */
                DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);
                /* If we're locked, move on, otherwise wait */
                if( (csr_rd_rtn_data & 0x01) == 1) {
                    DBG_SSCAL_TRACE("PLL L2R lock achieved\r\n");
                    sm_state = 2203;
                }
                /* If watchdog timer times out, abort */
                if( clk_cnt > 1000) {
                    DBG_SSCAL_TRACE("PLL L2R lock failed, aborting\r\n");
                    sm_state = 9999;
                }
                clk_cnt = clk_cnt+1;
                break;

            case 2203:
                /* Move to L2D and wait for lock */

                /* Force L2D Mode, Toggle Reset in lock detector */
                /* <2:0> lockin, default=001b (0.05% accuracy), relax to 0.4%, lockin=100b */
                /* <9:8> fd_ovrd, default=00b, set to 10b (L2D Mode) */
                /* <10> resetlockb, default=0, toggle to reset lock detect */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000200 | LOCKIN_SETTING,
                        &csr_rd_rtn_data, 1);
                DPI_wait_100ns();
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_LD_CFG_OFFS, 0x00000707, 0x00000600 | LOCKIN_SETTING,
                        &csr_rd_rtn_data, 1);
                sm_state = 2204;
                clk_cnt = 0;
                break;

            case 2204:
                /* Wait for PLL Lock */

                /* Read lock status */
                DPI_csr_rd( MXS40USB40PHY_RX_LD_STAT_OFFS, &csr_rd_rtn_data, 1);

                /* If we're locked, move on, otherwise wait */
                if( (csr_rd_rtn_data & 0x01) == 1) {
                    DBG_SSCAL_TRACE("L2D lock achieved\r\n");
                    /* Reset DFE Slice Point */
                    cur_dfe_slicepoint = MAX_SLICEPOINT;
                    /* dfe_slicepoint = cur_dfe_slicepoint */
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, cur_dfe_slicepoint, &csr_rd_rtn_data, 1);
                    /* Enable VGA/DFE Loops */
                    /* <9> vga_adapt_en = 1 */
                    /* <10> dfe_enable = 1 */
                    /* <11> vga_freeze = 0 */
                    /* <12> dfe_freeez = 0 */
                    /* <14> dfe_resetb = 1 */
                    DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00004600, &csr_rd_rtn_data, 1);
                    /* Set prior_vga  */
                    prior_vga = 8;    /* Initial Previous VGA value */
                    pp_vga = 8;       /* Initial Previous VGA value */
                    prior_dfe1 = 64;  /* Initial Previous Tap 1 DFE value */
                    pp_dfe1 = 64;     /* Initial Previous Tap 1 DFE value */
                    prior_dfe2 = 64;  /* Initial Previous Tap 2 DFE value */
                    pp_dfe2 = 64;     /* Initial Previous Tap 2 DFE value */
                    prior_dfe3 = 64;  /* Initial Previous Tap 3 DFE value */
                    pp_dfe3 = 64;     /* Initial Previous Tap 3 DFE value */

                    /* Move on to next state */
                    sm_state = 2205;
                }

                /* If watchdog timer times out, abort */
                if( clk_cnt > 1000) {
                    /* Timeout, declare failure and skip ahead */
                    DBG_SSCAL_TRACE("L2D lock failed, aborting\r\n");
                    sm_state = 9999; /* Abort */
                }
                clk_cnt = clk_cnt+1;
                break;

            case 2205:
                /* Configure and Start VGA Loop */

                /* Set DFE Slice Point */
                /* <5:0> dfe_slicepoint */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG2_OFFS, 0x0000003F, cur_dfe_slicepoint, &csr_rd_rtn_data, 1);

                /* Reset Clock Counter */
                clk_cnt = 0;
                vga_toggle_cnt = 0;
                dfe1_toggle_cnt = 0;
                dfe2_toggle_cnt = 0;
                dfe3_toggle_cnt = 0;
                /* Move on to next state */
                sm_state = 2206;
                break;

            case 2206:
                /* 2500 cycles * 20ns ==> 50 us */
                Cy_SysLib_DelayUs(50);
                sm_state = 2207;
                break;

            case 2207:
                /* Adapt dfe_slicepoint */

                pp_vga = prior_vga;
                prior_vga = cur_vga;
                pp_dfe1 = prior_dfe1;
                prior_dfe1 = cur_dfe1;
                pp_dfe2 = prior_dfe2;
                prior_dfe2 = cur_dfe2;
                pp_dfe3 = prior_dfe3;
                prior_dfe3 = cur_dfe3;
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT0_OFFS, &csr_rd_rtn_data, 1);
                cur_vga = csr_rd_rtn_data & 0x1F; /* Current VGA value */
                cur_dfe1 = (csr_rd_rtn_data >> 8) & 0x7F; /* Current DFE Tap 1 value */
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT1_OFFS, &csr_rd_rtn_data, 1);
                cur_dfe2 = csr_rd_rtn_data & 0x7F; /* Current DFE Tap 2 value */
                cur_dfe3 = (csr_rd_rtn_data >> 8) & 0x7F; /* Current DFE Tap 3 value */

                /* Check for toggling */
                if( ( (pp_vga < prior_vga)&&(cur_vga < prior_vga))||( (pp_vga > prior_vga)&&(cur_vga > prior_vga)) ) {
                    vga_toggle_cnt = vga_toggle_cnt+1;
                }
                if( ( (pp_dfe1 < prior_dfe1)&&(cur_dfe1 < prior_dfe1))||( (pp_dfe1 > prior_dfe1)&&(cur_dfe1 > prior_dfe1)) ) {
                    dfe1_toggle_cnt = dfe1_toggle_cnt+1;
                }
                if( ( (pp_dfe2 < prior_dfe2)&&(cur_dfe2 < prior_dfe2))||( (pp_dfe2 > prior_dfe2)&&(cur_dfe2 > prior_dfe2)) ) {
                    dfe2_toggle_cnt = dfe2_toggle_cnt+1;
                }
                if( ( (pp_dfe3 < prior_dfe3)&&(cur_dfe3 < prior_dfe3))||( (pp_dfe3 > prior_dfe3)&&(cur_dfe3 > prior_dfe3)) ) {
                    dfe3_toggle_cnt = dfe3_toggle_cnt+1;
                }

                /* If the VGA/DFE setting has changed, reset the timer */
                if( (vga_toggle_cnt > 3)|| (dfe1_toggle_cnt > 5)|| (dfe2_toggle_cnt > 5) || (dfe3_toggle_cnt > 5) ) {
                    /* The VGA/DFE code is dithering, jump out */
                    DBG_SSCAL_TRACE("Opt DFE_slicept=%x vga_val=%x\r\n", cur_dfe_slicepoint, cur_vga);
                    sm_state = 2208;
                } else if( (prior_vga != cur_vga)||( ((prior_dfe1 != cur_dfe1)||(prior_dfe2 != cur_dfe2)||(prior_dfe3 != cur_dfe3))&&(cur_vga != 0)&&(cur_vga != 15) ) ) {
                    /* Someone is changing, stay put unless we've hit the minimum or maximum VGA setting */
                    clk_cnt = 0;
                    sm_state = 2206;
                } else {
                    /* We should only reach here once the VGA/DFE are stable */
                    if( (cur_vga >= MAX_VGA) && (cur_dfe_slicepoint > MIN_SLICEPOINT) ) {
                        /* If VGA setting is too high (input signal is too small) reduce slice point */
                        if (data_rate == DATARATE_GEN1)
                            cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G1;
                        else
                            cur_dfe_slicepoint = cur_dfe_slicepoint - SLICEPOINT_DEC_G2;
                        sm_state = 2205;
                    } else if( (cur_vga >= MAX_VGA) && (cur_dfe_slicepoint <= MIN_SLICEPOINT) ) {
                        /* If VGA setting is too high (input signal is too small) there is nothing more we can do, abort measurement */
                        DBG_SSCAL_TRACE("Failed VGA, signal too small\r\n");
                        sm_state = 9999; /* Abort  */
                    } else if( (cur_vga <= MIN_VGA) && (cur_dfe_slicepoint >= MAX_SLICEPOINT) ) {
                        /* If VGA setting is too low (input signal is too big) and we can't reduce cap_gain, abort measurement */
                        DBG_SSCAL_TRACE("Failed VGA, signal too big\r\n");
                        sm_state = 9999; /* Abort  */
                    } else {
                        /* We're good */
                        DBG_SSCAL_TRACE("Opt DFE_slicept=%x vga_val=%x\r\n", cur_dfe_slicepoint, cur_vga);
                        sm_state = 2208;
                    }
                }
                break;

            case 2208:
                /* Done with VGA/DFE loop, read result and freeze */
                /* <9> vga_adapt_en = 1 */
                /* <10> dfe_enable = 1 */
                /* <11> vga_freeze = 1 */
                /* <12> dfe_freeez = 1 */
                /* <14> dfe_resetb = 1 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_DFE_CFG3_OFFS, 0x00005E00, 0x00005E00, &csr_rd_rtn_data, 1);
                DPI_csr_rd( MXS40USB40PHY_RX_DFE_STAT0_OFFS, &csr_rd_rtn_data, 1);
                cur_vga = csr_rd_rtn_data & 0x1F;
                DBG_SSCAL_TRACE("Final VGA=%x DFE_slicept=%x gain=%x dfe1=%x dfe2=%x dfe3=%x\r\n",
                        cur_vga, cur_dfe_slicepoint, cur_cap_gain, cur_dfe1, cur_dfe2, cur_dfe3);

                lcl_adapt_done = 1;
                sm_state = 0;

                break;

            case 9999:
                /* Update RX settings for power saving. */
                /* <6> err_path_en = 0 */
                /* <7> rom_path_en = 0 */
                /* <8> edge_deser_en = 0 */
                DPI_csr_rd_mod_wr( MXS40USB40PHY_RX_GNRL_CFG_OFFS, 0x000001C0, 0x00000000, &csr_rd_rtn_data, 1);
                lcl_adapt_done = 1;
                sm_state = 0;
                break;

            default:
                break;
        }
    }

    /* Store the recorded eye height number for each LUT index. */
    pCalCtxt->rxAdaptationDone[phyIndex] = true;
    pCalCtxt->phyLutIndex[phyIndex]      = bcs_index;
    for (lut_index = 0; lut_index < LUT_WIDTH; lut_index++) {
        pCalCtxt->phyEyeHeight[phyIndex][lut_index] = eye_height[lut_index];
    }

    /* Run through offset calibration again in Gen2 connections. */
    if (pCalCtxt->gen2Enabled == true) {
        rx_offset_calibration(pCalCtxt, phyRxRegs_p, false);
    }

    if ((pCalCtxt->dualLaneEnabled) && (pCalCtxt->activeLutMask == 0)) {
        tmp_index = 0;
        if (bcs_eye <= 1) {
            /* Enable only the best case index for exploration on the second PHY. */
            pCalCtxt->activeLutMask = (1U << bcs_index);
            tmp_index = 1;
        } else {
            /* Pick the entries with the highest two eye height values. */
            for (lut_index = 0; lut_index < LUT_WIDTH - 1; lut_index++) {
                if (eye_height[lut_index] >= (bcs_eye - 1)) {
                    tmp_index++;
                    pCalCtxt->activeLutMask |= (1U << lut_index);
                }
            }
        }

        /* In Gen1x2 case, restrict the number of LUTs considered for the second PHY to reduce run time. */
        if ((!pCalCtxt->gen2Enabled) && (tmp_index > 4)) {
            pCalCtxt->activeLutMask = (pCalCtxt->activeLutMask & MIN_LUT_GEN1X2) | (1U << bcs_index);
        }
    }

    return 0;
}

/* Print the results of the CTLE equalization algorithm run. */
/* Debug function used during Silicon bring-up: To be removed. */
void Cy_USBSS_Cal_PrintCtleResults (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t phyIndex)
{
    if ((pCalCtxt != NULL) && (pCalCtxt->rxAdaptationDone[phyIndex])) {
        pCalCtxt->rxAdaptationDone[phyIndex] = false;

        DBG_SSCAL_INFO("PHY%d Rx result codes: %x %x %x %x\r\n",phyIndex, pCalCtxt->phyResult[phyIndex][0],
                pCalCtxt->phyResult[phyIndex][1], pCalCtxt->phyResult[phyIndex][2], pCalCtxt->phyResult[phyIndex][3]);
        DBG_SSCAL_INFO("                    %x %x %x %x\r\n", pCalCtxt->phyResult[phyIndex][4],
                pCalCtxt->phyResult[phyIndex][5], pCalCtxt->phyResult[phyIndex][6], pCalCtxt->phyResult[phyIndex][7]);
        DBG_SSCAL_INFO("                    %x %x %x %x\r\n", pCalCtxt->phyResult[phyIndex][8],
                pCalCtxt->phyResult[phyIndex][9], pCalCtxt->phyResult[phyIndex][10], pCalCtxt->phyResult[phyIndex][11]);
        DBG_SSCAL_INFO("                    %x %x %x %x\r\n", pCalCtxt->phyResult[phyIndex][12],
                pCalCtxt->phyResult[phyIndex][13], pCalCtxt->phyResult[phyIndex][14], pCalCtxt->phyResult[phyIndex][15]);

        DBG_SSCAL_INFO("PHY%d Rx Eye heights: %x %x %x %x\r\n", phyIndex, pCalCtxt->phyEyeHeight[phyIndex][0],
                pCalCtxt->phyEyeHeight[phyIndex][1], pCalCtxt->phyEyeHeight[phyIndex][2], pCalCtxt->phyEyeHeight[phyIndex][3]);
        DBG_SSCAL_INFO("                   %x %x %x %x\r\n", pCalCtxt->phyEyeHeight[phyIndex][4],
                pCalCtxt->phyEyeHeight[phyIndex][5], pCalCtxt->phyEyeHeight[phyIndex][6], pCalCtxt->phyEyeHeight[phyIndex][7]);
        DBG_SSCAL_INFO("                   %x %x %x %x\r\n", pCalCtxt->phyEyeHeight[phyIndex][8],
                pCalCtxt->phyEyeHeight[phyIndex][9], pCalCtxt->phyEyeHeight[phyIndex][10], pCalCtxt->phyEyeHeight[phyIndex][11]);
        DBG_SSCAL_INFO("                   %x %x %x %x\r\n", pCalCtxt->phyEyeHeight[phyIndex][12],
                pCalCtxt->phyEyeHeight[phyIndex][13], pCalCtxt->phyEyeHeight[phyIndex][14], pCalCtxt->phyEyeHeight[phyIndex][15]);
        DBG_SSCAL_INFO("PHY%d Best case LUT index=%d\r\n", phyIndex, pCalCtxt->phyLutIndex[phyIndex]);
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DelayLpmEnable
****************************************************************************//**
*
* Function used to delay LPM transitions once a low power state has been entered
* or when a control request is received. LPM transitions will be re-enabled through
* a scheduled USBD task after the link returns into U0.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return None.
*
*******************************************************************************/
static void
Cy_USBSS_Cal_DelayLpmEnable (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    cy_stc_usb_cal_msg_t msg = {CY_USB_CAL_MSG_INVALID, {0, 0}};

    if (pCalCtxt->forceLPMAccept == false) {
        USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL =
            USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U1_Msk |
            USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U2_Msk;

        /* If LPM re-enable task is already scheduled, stop it. */
        if (pCalCtxt->lpmConfig == CY_SSCAL_LPM_DELAYED) {
            msg.type = CY_USBSS_CAL_MSG_ABORT_UX_ENABLE;
            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
        }

        /* Set state to indicate that LPM re-enable has been delayed. */
        if (pCalCtxt->lpmConfig == CY_SSCAL_LPM_ENABLED) {
            pCalCtxt->lpmConfig = CY_SSCAL_LPM_DELAYED;
        }
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_LpmSettingsUpdate
****************************************************************************//**
*
* Update controller and PHY settings for proper LPM transition support in
* single-lane operation.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
*******************************************************************************/
static void
Cy_USBSS_Cal_LpmSettingsUpdate (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base;
    USB32DEV_LNK_Type *USB32DEV_LNK;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *USB_PHY_PLL;

    if (pCalCtxt != NULL) {
        base         = pCalCtxt->regBase;
        USB32DEV_LNK = &base->USB32DEV_LNK;
        USB_PHY_PLL  = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;

        if (pCalCtxt->dualLaneEnabled == false) {
            /* x1 operation: Enable SSC and limit TS2 send during Polling/Recovery to order of 16 ordered sets. */
            USB_PHY_PLL->PLL_SSM_CFG2 = (USB_PHY_PLL->PLL_SSM_CFG2 | 0x00004000UL);
            USB32DEV_LNK->LNK_MISC_CONF = (
                    (USB32DEV_LNK->LNK_MISC_CONF & ~USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Msk) |
                    (0x03UL << USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Pos)
                    );
        } else {
            /* Keep SSC enabled. */
            USB_PHY_PLL->PLL_SSM_CFG2 = (USB_PHY_PLL->PLL_SSM_CFG2 | 0x00004000UL);
            /* x2 operation: Send maximum number of TS2 supported by the device. */
            USB32DEV_LNK->LNK_MISC_CONF = (
                    (USB32DEV_LNK->LNK_MISC_CONF & ~USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Msk) |
                    (0x03UL << USB32DEV_LNK_MISC_CONF_TX_TS2_CONF_Pos)
                    );
        }
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_WaitWhileInState
****************************************************************************//**
*
* Function which polls the LTSSM until link has exited the specified state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param state
* LTSSM state which needs to be exited.
* \param timeout_us
* Timeout period in microseconds.
*
* \return
* true if state transition happened, false in case of timeout.
*******************************************************************************/
static bool Cy_USBSS_Cal_WaitWhileInState (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint32_t                 state,
        uint32_t                 timeout_us)
{
    USB32DEV_LNK_Type *USB32DEV_LNK = pCalCtxt->pLinkBase;
    uint32_t tmp;
    bool waitDone = false;

    while (timeout_us) {
        tmp = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;
        if (tmp != state) {
            waitDone = true;
            break;
        }

        Cy_SysLib_DelayUs(1);
        timeout_us--;
    }

    return waitDone;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_WaitUntilState
****************************************************************************//**
*
* Function which polls the LTSSM until link has reached the specified state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param state
* LTSSM state which needs to be entered.
* \param timeout_us
* Timeout period in microseconds.
*
* \return
* true if state transition happened, false in case of timeout.
*******************************************************************************/
bool Cy_USBSS_Cal_WaitUntilState (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint32_t                 state,
        uint32_t                 timeout_us)
{
    USB32DEV_LNK_Type *USB32DEV_LNK = pCalCtxt->pLinkBase;
    uint32_t tmp;
    bool waitDone = false;

    while (timeout_us) {
        tmp = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;
        if (tmp == state) {
            waitDone = true;
            break;
        }

        Cy_SysLib_DelayUs(1);
        timeout_us--;
    }

    return waitDone;
}

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
* \param pMsg
* Message structure pointer.
*
* \return None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_IntrHandler (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    uint32_t state, phy_intr, phy_intr_1;
    uint32_t lnkState, ltssmState;
    uint8_t lbpmVal = 0;
    cy_stc_usb_cal_msg_t msg = {CY_USB_CAL_MSG_INVALID, {0, 0}};

    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PROT_Type *USB32DEV_PROT = &base->USB32DEV_PROT;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs = &(base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP);
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegsAlt = &(base->USB32DEV_PHYSS.USB40PHY[!(pCalCtxt->activePhyIdx)].USB40PHY_TOP);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegs = &(base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_RX);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegsAlt = &(base->USB32DEV_PHYSS.USB40PHY[!(pCalCtxt->activePhyIdx)].USB40PHY_RX);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegs0 = &(base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_RX);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegs1 = &(base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_RX);

    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *pPllRegs = &(base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS);

    /* Call the wakeup interrupt handler first. */
    Cy_USBSS_Cal_WakeupIntrHandler(pCalCtxt);

    state = (USB32DEV_MAIN->INTR & USB32DEV_MAIN->INTR_MASK);

    if ((state & USB32DEV_MAIN_INTR_PHY1_Msk) || (state & USB32DEV_MAIN_INTR_PHY0_Msk)) {

        phy_intr = pPhyRegs->INTR0_MASKED;
        pPhyRegs->INTR0 = phy_intr;

        phy_intr_1 = pPhyRegsAlt->INTR0_MASKED;
        pPhyRegsAlt->INTR0 = phy_intr_1;

        /* Work-around for Jira 1685: Set the EXIT_LP just before transitioning into recovery from U1/U2/U3. */
        if (((phy_intr | phy_intr_1) & USB32DEV_PHYSS_USB40PHY_TOP_INTR0_REG_INT_P0_CHANGE_Msk) != 0) {

            if (pCalCtxt->uxExitActive) {
                pCalCtxt->uxExitActive = false;
                if (
                        (Cy_SysLib_GetDeviceRevision() == 0x11) &&
                        (pCalCtxt->devLpmExitDisabled == false) &&
                        (((USB32DEV_PROT->PROT_CS & 0x7FUL) != 0) || (pCalCtxt->linkSuspended)) &&
                        ((USB32DEV_LNK->LNK_LTSSM_STATE & 0x3FUL) < 0x18UL)
                   ) {
                    USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL |= USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk;
                }

                if (Cy_SysLib_GetDeviceRevision() != 0x11) {

                    /* LPM exit work-around for non-A0 silicon only:
                     * In case of device initiated exit, wait until EXIT_LP bit has been cleared.
                     * In case of host initiated exit, wait until we are close to the end of programmed
                     * LFPS duration.
                     * We will then force link into U0, let it time out into Recovery and then train properly.
                     */
                    if ((USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL & USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk) != 0) {
                        while ((USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL & USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk) != 0);
                    } else {
                        Cy_SysLib_DelayUs(pCalCtxt->lpmExitLfpsDelay);
                    }


                    if (pCalCtxt->linkSuspended) {
                        if (pCalCtxt->deepSleepEntered) {
                            /* In Gen2 case, repeat receiver offset calibration.
                             * In Gen1 case, directly move the receiver into L2D mode.
                             */
                            if (pCalCtxt->dualLaneEnabled) {
                                rx_offset_calibration(pCalCtxt, (volatile uint32_t *)pRxRegs0, !pCalCtxt->gen2Enabled);
                                rx_offset_calibration(pCalCtxt, (volatile uint32_t *)pRxRegs1, !pCalCtxt->gen2Enabled);
                            } else {
                                rx_offset_calibration(pCalCtxt, (volatile uint32_t *)pRxRegs, !pCalCtxt->gen2Enabled);
                            }
                            Cy_SysLib_DelayUs(10);

                            /* Restore default U3 exit LFPS duration. */
                            USB32DEV_LNK->LNK_LFPS_RX_U3_EXIT = CY_USBSS_LFPS_PERIOD_TO_REG_G1(1800UL);

                            /* Force link into U0 after receiver has been updated. */
                            CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_U0);
                        } else {
                            /* Force link into U0 as the first step. */
                            CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_U0);

                            /*
                             * If we had updated receiver settings preparatory to initiating U3 exit, restore the
                             * settings and reset the CDR data aligner.
                             */
                            if ((pRxRegs->RX_HDWR_ENABLE & USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk) == 0)
                            {
                                /* Delay to allow LFPS handshake from host to be completed. */
                                Cy_SysLib_DelayUs(150);

                                /* Configure RX for L2D mode. */
                                pRxRegs->RX_LD_CFG = (
                                        (pRxRegs->RX_LD_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Msk) |
                                        (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Pos));

                                pPhyRegs->PIPE_RX_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_FORCE_REALIGN_Msk;
                                Cy_SysLib_DelayUs(10);
                                pPhyRegs->PIPE_RX_CTRL &= ~USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_FORCE_REALIGN_Msk;

                                if (pCalCtxt->dualLaneEnabled) {
                                    /* Configure RX for L2D mode. */
                                    pRxRegsAlt->RX_LD_CFG = (
                                            (pRxRegsAlt->RX_LD_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Msk) |
                                            (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Pos));

                                    pPhyRegsAlt->PIPE_RX_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_FORCE_REALIGN_Msk;
                                    Cy_SysLib_DelayUs(10);
                                    pPhyRegsAlt->PIPE_RX_CTRL &= ~USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_FORCE_REALIGN_Msk;
                                }
                            }
                        }
                    } else {
                        /* Force link into U0. */
                        CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_U0);
                    }
                }
            }
        }
    }

    if (state & USB32DEV_MAIN_INTR_LINK_Msk) {
        lnkState = USB32DEV_LNK->LNK_INTR & USB32DEV_LNK->LNK_INTR_MASK;

        /* Clear the high level LINK interrupt */
        USB32DEV_MAIN->INTR = USB32DEV_MAIN_INTR_LINK_Msk;

        if (lnkState & USB32DEV_LNK_INTR_DATA_RATE_CHANGE_Msk)
        {
            /* Switch from Gen1 to Gen2 and vice versa. */
            uint32_t regVal;
            uint32_t regValAlt;

            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_DATA_RATE_CHANGE_Msk;

            DBG_SSCAL_TRACE("RateChg: %x\r\n", USB32DEV_LNK->LNK_DATARATE_CHG_OBSERVE);
            if ((pPhyRegs->PCS_STATUS & USB32DEV_PHYSS_USB40PHY_TOP_PCS_STATUS_REG_PIPE_RATE_Msk) == 0) {
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_RATE_GEN1);
                if (pCalCtxt->gen2Enabled) {
                    pCalCtxt->gen2Enabled = false;

                    /* Reconnect in Gen 1 if RX Active polling timed out */
                    if ((pCalCtxt->lastLinkState == CY_USBSS_LNK_STATE_POLLING_ACT) ||
                            (pCalCtxt->lastLinkState == CY_USBSS_LNK_STATE_POLLING_CFG))
                    {
                        DBG_SSCAL_TRACE("TO: %x\r\n", pCalCtxt->lastLinkState);

                        /* Notify USBD that Gen2 link training failed. */
                        msg.type = CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE;
                        msg.data[0] = CY_USBD_USB_DEV_SS_GEN2;
                        msg.data[1] = 0;
                        Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                    }

                    /* Assert Serializer reset */
                    pPhyRegs->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    /* Assert PLL Reset. */
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                        pPhyRegsAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    }

                    /* Rate change in PLL. */
                    pPllRegs->PLL_DIVP_CFG = 0x00000061UL;
                    Cy_SysLib_DelayUs(1);
                    pPllRegs->PLL_DIVP_CFG = 0x000000E1UL;

                    /* Set the correct VCO CP gain based on data rate. */
                    pRxRegs->RX_CP_CFG = (
                            (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                            (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));

                    /* Rate change in receiver. */
                    pRxRegs->RX_GNRL_CFG = (
                            (pRxRegs->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                            (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk);

                    /* Update Rate in top control register */
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk;

                    /* De-assert PLL reset. */
                    pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

                    /* De-assert Serializer reset */
                    pPhyRegs->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_CP_CFG = (
                                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                        pRxRegsAlt->RX_GNRL_CFG = (
                                (pRxRegsAlt->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                                (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk);
                        pPhyRegsAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk;
                        pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                        pPhyRegsAlt->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    }

                    /* Disable DC balance in Gen1 case. */
                    USB32DEV_LNK->LNK_CONF &= ~USB32DEV_LNK_CONF_DC_BAL_EN_Msk;

                    /* Notify USBD. */
                    msg.type = CY_USBSS_CAL_MSG_USB3_RATE_CHG;
                    if ((USB32DEV_LNK->LNK_LTSSM_OBSERVE >> USB32DEV_LNK_LTSSM_OBSERVE_DATA_RATE_CONFIG_Pos) == 1) {
                        msg.data[0] = CY_USBD_USB_DEV_SS_GEN1X2;
                    } else {
                        msg.data[0] = CY_USBD_USB_DEV_SS_GEN1;
                    }
                    Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                }
            } else {
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_RATE_GEN2);
                if (!pCalCtxt->gen2Enabled) {
                    pCalCtxt->gen2Enabled = true;

                    /* Assert Serializer reset */
                    pPhyRegs->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    /* Assert PLL Reset. */
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                        pPhyRegsAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    }

                    /* Rate change in PLL. */
                    pPllRegs->PLL_DIVP_CFG = 0x00000060UL;
                    Cy_SysLib_DelayUs(1);
                    pPllRegs->PLL_DIVP_CFG = 0x000000E0UL;

                    /* Set the correct VCO CP gain based on data rate. */
                    pRxRegs->RX_CP_CFG = (
                            (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                            (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));

                    /* Rate change in receiver. */
                    pRxRegs->RX_GNRL_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk;
                    pRxRegs->RX_GNRL_CFG = (
                            (pRxRegs->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                            (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_SUMODD_PDB_Msk);

                    /* Update Rate in top control register */
                    pPhyRegs->TOP_CTRL_0 |= (0x01U << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos);

                    /* De-assert PLL reset. */
                    pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

                    /* De-assert Serializer reset */
                    pPhyRegs->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_CP_CFG = (
                                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                        pRxRegsAlt->RX_GNRL_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk;
                        pRxRegsAlt->RX_GNRL_CFG = (
                                (pRxRegsAlt->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                                (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_SUMODD_PDB_Msk);

                        pPhyRegsAlt->TOP_CTRL_0 |= (0x01U << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos);
                        pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                        pPhyRegsAlt->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    }

                    /* Enable DC balance in Gen2 case. */
                    USB32DEV_LNK->LNK_CONF |= USB32DEV_LNK_CONF_DC_BAL_EN_Msk;

                    /* Notify USBD. */
                    msg.type = CY_USBSS_CAL_MSG_USB3_RATE_CHG;
                    if ((USB32DEV_LNK->LNK_LTSSM_OBSERVE >> USB32DEV_LNK_LTSSM_OBSERVE_DATA_RATE_CONFIG_Pos) == 3) {
                        msg.data[0] = CY_USBD_USB_DEV_SS_GEN2X2;
                    } else {
                        msg.data[0] = CY_USBD_USB_DEV_SS_GEN2;
                    }
                    Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                }
            }

            regVal = pPhyRegs->TOP_CTRL_0 | USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;
            pPhyRegs->TOP_CTRL_0 = regVal;

            regValAlt = pPhyRegsAlt->TOP_CTRL_0 | USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;
            pPhyRegsAlt->TOP_CTRL_0 = regValAlt;

            pPhyRegs->INTR0 = pPhyRegs->INTR0;
            pPhyRegsAlt->INTR0 = pPhyRegsAlt->INTR0;
            Cy_SysLib_DelayUs(10);

            DBG_SSCAL_TRACE("PhyCtrl: %x %x %x\r\n", regVal, pPhyRegs->TOP_CTRL_0, USB32DEV_LNK->LNK_LTSSM_OBSERVE);
            DBG_SSCAL_TRACE("PhyCtrlAlt: %x %x\r\n", regValAlt, pPhyRegsAlt->TOP_CTRL_0);
        }

        if (lnkState & USB32DEV_LNK_INTR_LBAD_Msk)
        {
            /* Nothing to do here but to clear the interrupt. */
            USB32DEV_LNK->LNK_INTR |= USB32DEV_LNK_INTR_LBAD_Msk;
            pCalCtxt->lbadCounter++;
        }

        if (lnkState & USB32DEV_LNK_INTR_LPMA_Msk)
        {
            /* Nothing to do. Just a placeholder in case logic needs to be added. */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LPMA_Msk;

            /* Disable further LPM entry after recovery is completed if force Ux accept is not set. */
            Cy_USBSS_Cal_DelayLpmEnable(pCalCtxt);
        }

        if (lnkState & USB32DEV_LNK_INTR_LTSSM_U3_ENTRY_Msk)
        {
            /* Nothing to do here but to clear the interrupt. */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LTSSM_U3_ENTRY_Msk;
        }

        if (lnkState & USB32DEV_LNK_INTR_LGO_U3_Msk)
        {
            /* Nothing to do here but to clear the interrupt.
             * The device sends LAU handshake automatically.
             */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LGO_U3_Msk;
        }

        if (lnkState & USB32DEV_LNK_INTR_LTSSM_STATE_CHG_Msk)
        {
            /* Clear the interrupt. */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LTSSM_STATE_CHG_Msk;

            ltssmState = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;

            /* Added to prevent logging of continuous state changes between SS.Inactive.Quiet and
             * SS.Inactive.Disconnect.Detect.
             */
            if (ltssmState == CY_USBSS_LNK_STATE_SSINACT_DET) {
                ltssmState = CY_USBSS_LNK_STATE_SSINACT_QUT;
            }

            if (ltssmState != pCalCtxt->lastLinkState) {
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_LNK_SS_DISABLE + ltssmState);
                pCalCtxt->prevLinkState = pCalCtxt->lastLinkState;
                pCalCtxt->lastLinkState = ltssmState;
            }

            switch (ltssmState) {
                case CY_USBSS_LNK_STATE_SSDISABLED:
                    {
                        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_LNK_EVENT_SS_DISABLED_ENTRY);

                        /* Notify USBD about SsDisable event. */
                        msg.type = CY_USBSS_CAL_MSG_LNK_SS_DISABLE;
                        Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);

                        /*
                         * x2 interop workaround: If we have not raised a disconnect interrupt after
                         * reach SS.Disabled state, raise it here.
                         */
                        if ((pCalCtxt->connectRcvd) && ((lnkState & USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk) == 0)) {
                            USB32DEV_LNK->LNK_INTR_SET = USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk;
                            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_LNK_EVENT_TRIG_DISCON_EVENT);
                        }
                    }
                    break;

                case CY_USBSS_LNK_STATE_SSINACT_QUT:
                case CY_USBSS_LNK_STATE_SSINACT_DET:
                    DBG_SSCAL_TRACE("LNK-SS.Inactive\r\n");
                    break;

                case CY_USBSS_LNK_STATE_RXDETECT_ACT:
                    DBG_SSCAL_TRACE("LNK-SS.RxDetectAct\r\n");
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_RX_EQ_TRAINING_FLAG_Msk;
                    pPhyRegsAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_RX_EQ_TRAINING_FLAG_Msk;
                    pCalCtxt->pollingLFPSDone = false;

                    /* Special handling for intermittent Reset-Device test failure. */
                    if (!Cy_USBSS_Cal_WaitWhileInState(pCalCtxt, CY_USBSS_LNK_STATE_RXDETECT_ACT, 1000)) {
                        /*
                         * If we are stuck in Rx.Detect.Active state continuously for more than 1 ms, force link into
                         * SS.Disabled state.
                         */
                        CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_SSDISABLED);
                    }
                    break;

                case CY_USBSS_LNK_STATE_RXDETECT_QUT:
                    /* Nothing to do at present. */
                    break;

                case CY_USBSS_LNK_STATE_POLLING_LFPS:
                    /* Need to enable PortCAP and PortCFG LMP exchange when going through Polling. */
                    USB32DEV_PROT->PROT_LMP_PORT_CAPABILITY_TIMER    = CY_USBSS_PROT_LMP_PORT_CAP_TIMER_VALUE;
                    USB32DEV_PROT->PROT_LMP_PORT_CONFIGURATION_TIMER = CY_USBSS_PROT_LMP_PORT_CFG_TIMER_VALUE;
                    pCalCtxt->pollingCfgDone = false;
                    pCalCtxt->pollingLFPSDone = true;
                    pCalCtxt->lgoodTimeoutCount = 0;

                    /* If it took less than 35 ms or more than 50 ms from compliance exit to Polling.LFPS entry,
                     * clear the compliance exit done flag.
                     */
                    if ((pCalCtxt->compExitDone) && (pCalCtxt->compExitTS != 0)) {
                        if (
                                (Cy_USBD_GetTimerTick() < (pCalCtxt->compExitTS + 35U)) ||
                                (Cy_USBD_GetTimerTick() > (pCalCtxt->compExitTS + 50U))
                           ) {
                            pCalCtxt->compExitDone = false;
                            pCalCtxt->compExitTS   = 0;
                        }
                    }
                    break;

                case CY_USBSS_LNK_STATE_POLLING_LFPSPLUS:
                    pCalCtxt->pollingLFPSDone = true;
                case CY_USBSS_LNK_STATE_POLLING_PORTMATCH:
                    lbpmVal = USB32DEV_LNK->LNK_RCVD_LBPM_OBSERVE;
                    if(!(lbpmVal & CY_USBSS_LBPM_PHY_CAP_MASK)) {
                        /* JIRA-2069: Check if the DUT needs to fallback to 1x1 when DUT is in 1x2 and
                         * Host/Hub is reporting 2x1 (to handle cases where hub does not support fallback) */
                        if ((pCalCtxt->dualLaneEnabled) && (!(pCalCtxt->gen2Enabled)) &&
                                (lbpmVal == CY_USBSS_LBPM_PHY_CAP_GEN2X1_VAL)
                           ) {
                            pCalCtxt->gen1x1FallbackEn = true;
                        }
                    }
                case CY_USBSS_LNK_STATE_POLLING_PORTCONF:
                    /* Disable the LFPS filter here. */
                    pPhyRegs->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    }

                    /* JIRA-2069: Check if the DUT needs to fallback to 1x1 when DUT is in 1x2 and
                     * Host/Hub is reporting 2x1 (to handle cases where hub does not support fallback) */
                    if ((pCalCtxt->dualLaneEnabled) && (!(pCalCtxt->gen2Enabled)) &&
                            (
                             ((USB32DEV_LNK->LNK_CAP_LBPM_OBSERVE & 0x0007UL) > 0x0001) &&
                             ((USB32DEV_LNK->LNK_CAP_LBPM_OBSERVE & 0x7000UL) < 0x2000)
                            )
                       ) {
                        pCalCtxt->gen1x1FallbackEn = true;
                    }
                    break;

                case CY_USBSS_LNK_STATE_POLLING_RxEQ:
                    /* Disable the LFPS filter here. */
                    pPhyRegs->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    }

                    /* Added to detect possible lane change conditions */
                    msg.type = CY_USBSS_CAL_MSG_USB3_RATE_CHG;
                    switch (USB32DEV_LNK->LNK_LTSSM_OBSERVE >> USB32DEV_LNK_LTSSM_OBSERVE_DATA_RATE_CONFIG_Pos)
                    {
                        default:
                        case 0:
                            msg.data[0] = CY_USBD_USB_DEV_SS_GEN1;
                            pCalCtxt->gen2Enabled = false;
                            pCalCtxt->dualLaneEnabled = false;

                            /* Disable Sublink notification. */
                            USB32DEV_PROT->PROT_SUBLINK_DEV_NNOTIFICATION = 0x0000000CUL;
                            USB32DEV_PROT->PROT_SUBLINK_LSM               = 0x00050005UL;
                            break;

                        case 1:
                            msg.data[0] = CY_USBD_USB_DEV_SS_GEN1X2;
                            pCalCtxt->gen2Enabled = false;
                            pCalCtxt->dualLaneEnabled = true;

                            /* Set Sublink notification parameters: Lane = 2, Lane Speed Matissa - 5 */
                            USB32DEV_PROT->PROT_SUBLINK_DEV_NNOTIFICATION = 0x0000004FUL;
                            USB32DEV_PROT->PROT_SUBLINK_LSM               = 0x00050005UL;
                            break;

                        case 2:
                            msg.data[0] = CY_USBD_USB_DEV_SS_GEN2;
                            pCalCtxt->gen2Enabled = true;
                            pCalCtxt->dualLaneEnabled = false;

                            /* Set Sublink notification parameters: Lane = 1, Lane Speed Matissa - 10 */
                            USB32DEV_PROT->PROT_SUBLINK_DEV_NNOTIFICATION = 0x0000040FUL;
                            USB32DEV_PROT->PROT_SUBLINK_LSM               = 0x000A000AUL;
                            break;
                        case 3:
                            msg.data[0] = CY_USBD_USB_DEV_SS_GEN2X2;
                            pCalCtxt->gen2Enabled = true;
                            pCalCtxt->dualLaneEnabled = true;

                            /* Set Sublink notification parameters: Lane = 2, Lane Speed Matissa - 10 */
                            USB32DEV_PROT->PROT_SUBLINK_DEV_NNOTIFICATION = 0x0000044FUL;
                            USB32DEV_PROT->PROT_SUBLINK_LSM               = 0x000A000AUL;
                            break;
                    }

                    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_USBD_DATA_RATE_UNCONNECTED + msg.data[0]);
                    Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);

                    pCalCtxt->activeLutMask = 0;                /* Start with no LUT customization. */
                    /* GEN1/GEN2 switch */
                    z_dfe_ctle_adaptation(pCalCtxt, (pCalCtxt->gen2Enabled)? DATARATE_GEN2 : DATARATE_GEN1, pCalCtxt->activePhyIdx);
                    if (pCalCtxt->dualLaneEnabled) {
                        z_dfe_ctle_adaptation(pCalCtxt, (pCalCtxt->gen2Enabled)? DATARATE_GEN2 : DATARATE_GEN1, !(pCalCtxt->activePhyIdx));
                    }

                    Cy_USBSS_Cal_LpmSettingsUpdate(pCalCtxt);


                    /* CDR hardware enable. */
                    pRxRegs->RX_HDWR_ENABLE |= USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_HDWR_ENABLE |= USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;
                    }

                    pCalCtxt->connectRcvd = true;
                    if (pCalCtxt->dualLaneEnabled)
                    {
                        /*  b4:Config Lane LFPS/Powermode are used for L2D change */
                        /*  b3:Elastic Buffer writes start on External input from controller (till both PHYs are aligned)*/
                        pPhyRegs->PCS_SPARE |=
                            (0x18UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                        pPhyRegsAlt->PCS_SPARE |=
                            (0x18UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    }
                    else
                    {
                        /* b4: This lane LFPS/Powermode are used for L2D change */
                        /* b3: Elastic Buffer writes start on RX alignment (of the selected PHY) */
                        pPhyRegs->PCS_SPARE &=
                            ~(0x18UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                        pPhyRegsAlt->PCS_SPARE &=
                            ~(0x18UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    }
                    break;

                case CY_USBSS_LNK_STATE_POLLING_ACT:
                    if ((pCalCtxt->gen2Enabled) && !(pCalCtxt->dualLaneEnabled) && (pCalCtxt->activePhyIdx == 1)) {
                        /* Enable polarity inversion if link is stuck in polling.active for 4ms */
                        /* TODO: Check whether we can use Cy_USBSS_Cal_WaitWhileInState() function instead. */
                        uint32_t pollActCounter = 20000;
                        while (pollActCounter) {
                            /* LTSSM no longer stuck in Polling.Active, exit out of loop */
                            if ((USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk) !=
                                    CY_USBSS_LNK_STATE_POLLING_ACT) {
                                break;
                            }
                            pollActCounter--;
                        }

                        if (!pollActCounter) {
                            pPhyRegs->PIPE_OVERRIDE_0 = 0x6000;
                            CY_LOG_EVENT(CY_LNK_EVENT_LANE_POLARITY_INV);
                        }
                    }

                    if (pCalCtxt->dualLaneEnabled) {
                        /* Keep the link forced in Polling.Active for some time and then release. */
                        CY_USBSS_FORCE_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_POLLING_ACT);
                        if (pCalCtxt->gen2Enabled) {
                            Cy_SysLib_DelayUs(400);
                        } else {
                            Cy_SysLib_DelayUs(700);
                        }
                        CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_POLLING_ACT);
                    }

                    if (
                            (pCalCtxt->dualLaneEnabled) &&
                            (pCalCtxt->compExitDone) && (pCalCtxt->compExitTS != 0)
                       ) {
                        /* Work-around for intermittent loopback entry failure.
                         * If we appear to be timing out here, force link into loopback state.
                         */
                        if (!Cy_USBSS_Cal_WaitWhileInState(pCalCtxt, CY_USBSS_LNK_STATE_POLLING_ACT, 15000)) {
                            pCalCtxt->compExitDone = false;
                            pCalCtxt->compExitTS   = 0;

                            CY_USBSS_FORCE_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_LPBK_ACTV);
                            msg.type = CY_USBSS_CAL_MSG_LPBK_FORCED;
                            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                        }
                    }
                    break;

                case CY_USBSS_LNK_STATE_POLLING_CFG:
                    pCalCtxt->pollingCfgDone = true;

                    if (
                            (pCalCtxt->dualLaneEnabled) &&
                            (pCalCtxt->compExitDone) && (pCalCtxt->compExitTS != 0)
                       ) {
                        /* Work-around for intermittent loopback entry failure.
                         * If we appear to be timing out here, force link into loopback state.
                         */
                        pCalCtxt->compExitDone = false;
                        pCalCtxt->compExitTS   = 0;

                        if (!Cy_USBSS_Cal_WaitWhileInState(pCalCtxt, CY_USBSS_LNK_STATE_POLLING_CFG, 15000)) {
                            CY_USBSS_FORCE_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_LPBK_ACTV);
                            msg.type = CY_USBSS_CAL_MSG_LPBK_FORCED;
                            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                        }
                    }
                    break;

                case CY_USBSS_LNK_STATE_POLLING_IDLE:
                case CY_USBSS_LNK_STATE_LPBK_ACTV:
                    pCalCtxt->pollingCfgDone = true;
                    pCalCtxt->compExitDone = false;
                    pCalCtxt->compExitTS   = 0;
                    break;

                case CY_USBSS_LNK_STATE_U0:
                    /* Restore TS1 holdoff delay to 0. */
                    USB32DEV_LNK->LNK_TX_TS1_HOLDOFF_TIMEOUT = 0;

                    pPhyRegs->PIPE_OVERRIDE_0 &= ~(0x1800);
                    pPhyRegsAlt->PIPE_OVERRIDE_0 &= ~(0x1800);
                    DBG_SSCAL_TRACE("LNK-U0\r\n");
                    pRxRegs->RX_HDWR_ENABLE |= USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;

                    /* Restore L2D start delay to U1 exit value. */
                    pCalCtxt->lpmExitLfpsDelay = CY_USBSS_U1_EXIT_LFPS_DURATION - 50U;
                    pPhyRegs->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U1_WAKE_LFPS_DURATION);
                    pPhyRegs->PCS_SPARE |= (0x04UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_HDWR_ENABLE |= USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;
                        pPhyRegsAlt->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U1_WAKE_LFPS_DURATION);
                        pPhyRegsAlt->PCS_SPARE |= (0x04UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    }

                    /* If we have seen two PendingHpTimer timeout errors, relax the PendingHpTimeout period
                     * to 20 us (2X the default value).
                     *
                     * Note: We will keep updating the timeout every time the counter rolls over from 1 to 2.
                     * Leaving the code as is as writing the same value to the timeout register will not
                     * cause any issues.
                     */
                    if ((USB32DEV_LNK->LNK_ERROR_STATUS & USB32DEV_LNK_ERROR_STATUS_HP_TIMEOUT_EV_Msk) != 0) {
                        USB32DEV_LNK->LNK_ERROR_STATUS = USB32DEV_LNK_ERROR_STATUS_HP_TIMEOUT_EV_Msk;
                        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_HP_TIMEOUT);

                        pCalCtxt->lgoodTimeoutCount++;
                        if (pCalCtxt->lgoodTimeoutCount == 2) {
                            /* Relax PendingHpTimer timeout to 20 us. */
                            USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT      = 2500UL;
                            USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT_GEN2 = 6250UL;
                        }
                    }

                    if (pCalCtxt->lpmConfig == CY_SSCAL_LPM_DELAYED) {
                        msg.type = CY_USBSS_CAL_MSG_UX_REENABLE;
                        Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                    }

                    pCalCtxt->pollingLFPSDone  = false;
                    pCalCtxt->pollingCfgDone   = false;
                    pCalCtxt->linkSuspended    = false;
                    pCalCtxt->gen1x1FallbackEn = false;
                    pCalCtxt->compExitDone     = false;
                    pCalCtxt->compExitTS       = 0;

                    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
                        /*
                         * Work-around for A0 Silicon bug: Device loses device address when deep sleep is entered
                         * and will not respond to host requests once link is resumed to U0. Force link into
                         * SS.Inactive state when this happens so that there will be a Warm Reset from the host
                         * and new device address gets assigned.
                         */
                        if (pCalCtxt->deepSleepEntered == true )
                        {
                            if ((USB32DEV_LNK->LNK_LTSSM_STATE & 0x3F) == 0x10)
                            {
                                USB32DEV_LNK->LNK_LTSSM_STATE = 0x2100;
                            }
                        }
                    }

                    if (pCalCtxt->deepSleepEntered) {
                        pPhyRegs->TX_AFE_CTRL_0 = (
                                (pPhyRegs->TX_AFE_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Msk) |
                                (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Pos));
                        if (pCalCtxt->dualLaneEnabled) {
                            pPhyRegsAlt->TX_AFE_CTRL_0 = (
                                    (pPhyRegsAlt->TX_AFE_CTRL_0 & ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Msk) |
                                    (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Pos));
                        }
                    }

                    /* Clear DeepSleep Flag */
                    pCalCtxt->deepSleepEntered = false;
                    break;

                case CY_USBSS_LNK_STATE_U2:

                    /* Setting L2D start delay to CY_USBSS_MAX_U2_WAKE_LFPS_DURATION to account for maximum
                     * U2 exit LFPS duration from the host/hub side.
                     */
                    pCalCtxt->lpmExitLfpsDelay = CY_USBSS_U2_EXIT_LFPS_DURATION - 50U;
                    pPhyRegs->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U2_WAKE_LFPS_DURATION);
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U2_WAKE_LFPS_DURATION);
                    }

                    /* Fall-through. */
                case CY_USBSS_LNK_STATE_U1:
                    /*
                     * If we are stuck in U1RXEXIT state for more than CY_USBSS_U1_EXIT_LFPS_DURATION duration,
                     * force the link into U2 state.
                     */
                    if ((USB32DEV_LNK->LNK_LTSSM_SUBSTATE_OBSERVE & 0xFC000000UL) == 0x74000000UL) {
                        Cy_SysLib_DelayUs(CY_USBSS_U1_EXIT_LFPS_DURATION + 5);
                        if ((USB32DEV_LNK->LNK_LTSSM_SUBSTATE_OBSERVE & 0xFC000000UL) == 0x74000000UL) {
                            CY_USBSS_JUMP_TO_LINK_STATE(USB32DEV_LNK, CY_USBSS_LNK_STATE_U2);
                            DBG_SSCAL_INFO("LTSSM force into U2\r\n");
                        }
                    }
                    pCalCtxt->uxExitActive = true;
                    break;

                case CY_USBSS_LNK_STATE_U3:
                    /* Setting L2D start delay to CY_USBSS_MAX_U3_WAKE_LFPS_DURATION to account for maximum
                     * U3 exit LFPS duration from the host/hub side.
                     */
                    pCalCtxt->lpmExitLfpsDelay = CY_USBSS_U3_EXIT_LFPS_DURATION - 50U;
                    pPhyRegs->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U3_WAKE_LFPS_DURATION);
                    pPhyRegs->PCS_SPARE &= ~(0x04UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->CDR_SW_CTRL = CY_FX10_USBSS_CDR_SW_CTRL_VAL(200UL, CY_USBSS_MAX_U3_WAKE_LFPS_DURATION);
                        pPhyRegsAlt->PCS_SPARE &= ~(0x04UL << USB32DEV_PHYSS_USB40PHY_TOP_PCS_SPARE_REG_DFT_Pos);
                    }

                    /* Set flag to indicate USB link has been suspended. */
                    pCalCtxt->linkSuspended = true;
                    pCalCtxt->uxExitActive = true;

                    /* Clear and enable the LFPS detect interrupt in the active PHY. */
                    pPhyRegs->INTR1      = 0xFFFFFFFFUL;
                    pPhyRegs->INTR1_MASK = USB32DEV_PHYSS_USB40PHY_TOP_INTR1_REG_INT_RX_LFPSDET_OUT_Msk;

                    /* Before disabling PHY PCLK, make sure HBWSS is not using the clock. */
                    Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, false);

                    /* Disable the clock at the IP and PHY level. */
                    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
                    pPhyRegsAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
                    break;

                case CY_USBSS_LNK_STATE_COMP:
                    DBG_SSCAL_INFO("LNK-Compliance\r\n");

                    /* Disable the LFPS filter here. */
                    pPhyRegs->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->LOW_POWER_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_DISABLE_LFPS_FILTER_Msk;
                    }

                    /* Notify USBD about compliance state entry. */
                    msg.type = CY_USBSS_CAL_MSG_LNK_COMPLIANCE;
                    Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                    break;

                    /* The same handling applies for Recovery.Active and Recovery.Config states. */
                case CY_USBSS_LNK_STATE_RECOV_ACT:
                case CY_USBSS_LNK_STATE_RECOV_CNFG:
                    {
                        if (pCalCtxt->linkSuspended) {
                            msg.type = CY_USB_CAL_MSG_RESUME_END;
                            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                        }
                    }
                    break;

                case CY_USBSS_LNK_STATE_ILLEGAL:
                    DBG_SSCAL_INFO("State:1F SubState:%x\r\n", USB32DEV_LNK->LNK_LTSSM_SUBSTATE_OBSERVE);
                    break;

                default:
                    /* Nothing to be done. */
                    break;
            }
        }

        if (lnkState & USB32DEV_LNK_INTR_LTSSM_RESET_Msk)
        {
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_RESET);
            pPhyRegs->INTR1_MASK = 0;
            pPhyRegs->PIPE_OVERRIDE_0 = 0;
            if(pCalCtxt->dualLaneEnabled)
            {
                pPhyRegsAlt->PIPE_OVERRIDE_0 = 0;
            }
            pCalCtxt->pollingLFPSDone = false;
            pCalCtxt->gen1x1FallbackEn = false;

            /* If we are in compliance state, it has to be exited. */
            if (pCalCtxt->inCompState) {
                Cy_USBSS_Cal_ExitLinkCompliance(pCalCtxt);
            }

            /* Re-enable LMP if LTSSM state is Rx.Detect. */
            if ((USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk) < 4UL) {
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_WARM_RESET);

                /* Make sure PCLK is not being used by HBWSS. */
                Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, false);

                /* Disable hardware control of CDR until Polling.RxEq is complete. */
                pRxRegs->RX_HDWR_ENABLE &= ~USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;
                if(pCalCtxt->dualLaneEnabled)
                {
                    pRxRegsAlt->RX_HDWR_ENABLE &= ~USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;
                }

                USB32DEV_PROT->PROT_LMP_PORT_CAPABILITY_TIMER    = CY_USBSS_PROT_LMP_PORT_CAP_TIMER_VALUE;
                USB32DEV_PROT->PROT_LMP_PORT_CONFIGURATION_TIMER = CY_USBSS_PROT_LMP_PORT_CFG_TIMER_VALUE;

                /* Re-enable filtering of LFPS for 8 PERI clock cycles or 106.67 ns. */
                pPhyRegs->LOW_POWER_CTRL = (
                        (8UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_Pos) |
                        (15UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_HW_CTRL_DELAY_Pos) |
                        USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_MASK_ON_TX_DATA_Msk |
                        USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_MODE_Msk);

                if (pCalCtxt->dualLaneEnabled)
                {
                    pPhyRegs->LOW_POWER_CTRL = (
                            (8UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_Pos) |
                            (15UL << USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_HW_CTRL_DELAY_Pos) |
                            USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_MASK_ON_TX_DATA_Msk |
                            USB32DEV_PHYSS_USB40PHY_TOP_LOW_POWER_CTRL_LFPS_FILTER_MODE_Msk);
                }

                /* Notify USBD about reset event only if connect has been received before. */
                if (pCalCtxt->connectRcvd) {
                    pCalCtxt->connectRcvd = false;
                    msg.type = CY_USBSS_CAL_MSG_LNK_RESET;
                    Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
                }
            } else {
                /* Always notify USBD about hot reset. */
                msg.type = CY_USBSS_CAL_MSG_LNK_RESET;
                Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            USB32DEV_PROT->PROT_EPO_CS1[0] &= ~USB32DEV_PROT_EPO_CS1_STALL_Msk;
            USB32DEV_PROT->PROT_EPI_CS1[0] &= ~USB32DEV_PROT_EPI_CS1_STALL_Msk;
            pCalCtxt->ptmStatus = (uint32_t)CY_USBSS_PTM_STAT_LDM_ENABLE_Msk;
            pCalCtxt->uxExitActive   = false;
            pCalCtxt->lpmWarningDone = false;
            pCalCtxt->u2TimeoutVal   = 0;

            /* Set Gen2 LFPS durations to default values. */
            USB32DEV_LNK->LNK_LFPS_RX_RESET_GEN2 = 25000000UL;
            USB32DEV_LNK->LNK_LFPS_RX_PING_GEN2  = 0x004B000DUL;

            /* Restore default PendingHpTimer value. */
            USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT      = 1250UL;
            USB32DEV_LNK->LNK_PENDING_HP_TIMEOUT_GEN2 = 3125UL;

            /* Clear and re-enable the interrupt. */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LTSSM_RESET_Msk;
        }

        if (lnkState & USB32DEV_LNK_INTR_LTSSM_CONNECT_Msk)
        {
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_CONNECT);

            USB32DEV_PROT->PROT_EPO_CS1[0] &= ~USB32DEV_PROT_EPO_CS1_STALL_Msk;
            USB32DEV_PROT->PROT_EPI_CS1[0] &= ~USB32DEV_PROT_EPI_CS1_STALL_Msk;
            pCalCtxt->ptmStatus = (uint32_t)CY_USBSS_PTM_STAT_LDM_ENABLE_Msk;
            pCalCtxt->connectRcvd    = true;
            pCalCtxt->uxExitActive   = false;
            pCalCtxt->lpmWarningDone = false;

            msg.type = CY_USBSS_CAL_MSG_LNK_CONNECT;
            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LTSSM_CONNECT_Msk;
        }

        if (lnkState & USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk)
        {
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_DISCONNECT);

            pCalCtxt->connectRcvd = false;
            pCalCtxt->linkSuspended = false;
            if ((pCalCtxt->gen1x1FallbackEn) && (pCalCtxt->prevLinkState == CY_USBSS_LNK_STATE_POLLING_PORTMATCH)) {
                pCalCtxt->gen1x1FallbackEn = false;
                msg.type    = CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE;
                msg.data[0] = CY_USBD_USB_DEV_SS_GEN1;
                msg.data[1] = 1;
            } else {
                msg.type = CY_USBSS_CAL_MSG_LNK_DISCONNECT;
            }
            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);

            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk;
        }

        if (lnkState & USB32DEV_LNK_INTR_RX_PING_LFPS_Msk)
        {
            /* Clear the interrupt. */
            USB32DEV_LNK->LNK_INTR = USB32DEV_LNK_INTR_RX_PING_LFPS_Msk;

            /* Advance to the next compliance pattern. */
            Cy_USBSS_Cal_NextCompliancePattern(pCalCtxt);
        }
    }

    if (state & USB32DEV_MAIN_INTR_PROT_Msk) {

        uint32_t prot_intr;
        bool yield_reqd = false;

        USB32DEV_MAIN->INTR = (USB32DEV_MAIN_INTR_PROT_Msk);
        Cy_SysLib_DelayUs(1);

        /* Read the PROT_INTR register after clearing the PROT bit in the INTR register. */
        prot_intr = USB32DEV_PROT->PROT_INTR_MASKED;

        /*
         * As off now following interrupts are ignored. If require the coded.
         * 1. EP0_STALL_EV.
         * 2. All LDM interrupt.
         * 3. ITP_EV (Sof): Not required to handle SOF.
         */
        if (prot_intr != 0x00) {

            if (prot_intr & USB32DEV_PROT_INTR_ITP_EV_Msk) {
                /* Clear the interrupt. */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_ITP_EV_Msk;

                /* Once the bus interval counter reaches max value, trigger link delay measurement. */
                if (((pCalCtxt->ptmStatus & CY_USBSS_PTM_STAT_LDM_ENABLE_Msk) != 0) && (pCalCtxt->gen2Enabled)) {
                    if ((USB32DEV_PROT->PROT_PTM_CONFIG & USB32DEV_PROT_PTM_CONFIG_WAIT_ITP_Msk) != 0) {
                        USB32DEV_PROT->PROT_PTM_CONFIG |= USB32DEV_PROT_PTM_CONFIG_LMP_SEND_Msk;
                    } else {
                        /* Trigger LDM measurement only at the time when received BIC value is going to 0. */
                        if ((USB32DEV_PROT->PROT_FRAMECNT & 0x3FFF) == 0x3FFF) {
                            uint32_t ptmConf;

                            /* Clear and enable LDM RX interrupt */
                            USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_LDM_RX_Msk;
                            USB32DEV_PROT->PROT_INTR_MASK |= (USB32DEV_PROT_INTR_LDM_RX_Msk);

                            /* Enable PTM engine to wait for next ITP. */
                            ptmConf = USB32DEV_PROT_PTM_CONFIG_LDM_ENABLE_Msk;
                            USB32DEV_PROT->PROT_PTM_CONFIG = ptmConf;
                            ptmConf |= USB32DEV_PROT_PTM_CONFIG_WAIT_ITP_Msk | USB32DEV_PROT_PTM_CONFIG_COUNTER_CFG_Msk;
                            USB32DEV_PROT->PROT_PTM_CONFIG = ptmConf;
                            ptmConf |= USB32DEV_PROT_PTM_CONFIG_START_Msk;
                            USB32DEV_PROT->PROT_PTM_CONFIG = ptmConf;
                        }
                    }
                }
            }

            if (prot_intr & USB32DEV_PROT_INTR_LDM_RX_Msk) {

                /* Disable ITP interrupt itself after receiving LDM LMP Response */
                USB32DEV_PROT->PROT_INTR_MASK &= ~(USB32DEV_PROT_INTR_ITP_EV_Msk);

                /****************************** Link  delay calculations **************************/
                volatile uint32_t t4Delay=0, t1Delay=0, tRespDelay=0, tLinkDelay=0;
                /* save t4 & t1 delay from 26:0 bits in PROT_PTM_T4 & PROT_PTM_T1 */
                t4Delay = (USB32DEV_PROT->PROT_PTM_T4) & (USB32DEV_PROT_PTM_T4_TIME_Msk);
                t1Delay = (USB32DEV_PROT->PROT_PTM_T1) & (USB32DEV_PROT_PTM_T1_TIME_Msk);

                /* Get the Response delay from LDM response from host */
                tRespDelay = ((USB32DEV_PROT->PROT_LMP_LDM_RESPONSE & USB32DEV_PROT_LMP_LDM_RESPONSE_LDM_RESP_DELAY_Msk) >>
                            USB32DEV_PROT_LMP_LDM_RESPONSE_LDM_RESP_DELAY_Pos);

                /* Calculate Link Delay */
                tLinkDelay = (((t4Delay - t1Delay) - tRespDelay) / 0x2U);

                /* Set the read-bit in HW_PTM_COUNTER */
                USB32DEV_PROT->PROT_HW_PTM_COUNTER = USB32DEV_PROT_HW_PTM_COUNTER_READ_Msk;

                /* Wait until the bit is cleared by HW */
                while ((USB32DEV_PROT->PROT_HW_PTM_COUNTER & USB32DEV_PROT_HW_PTM_COUNTER_READ_Msk) != 0);

                /* Read the PROT_HW_PTM_COUNTER BIC & DC values*/
                volatile uint32_t hw_bic=0, hw_dc=0;

                hw_bic = ((USB32DEV_PROT->PROT_HW_PTM_COUNTER & USB32DEV_PROT_HW_PTM_COUNTER_BIC_Msk) >>
                        USB32DEV_PROT_HW_PTM_COUNTER_BIC_Pos);

                hw_dc = ((USB32DEV_PROT->PROT_HW_PTM_COUNTER & USB32DEV_PROT_HW_PTM_COUNTER_DC_Msk) >>
                        USB32DEV_PROT_HW_PTM_COUNTER_DC_Pos);

                /****************************** BIC & DC calculations **************************/
                volatile uint32_t isochDelay=pCalCtxt->isochDelay, correction=0, calc_dc=0, calc_bic=0;

                correction = (USB32DEV_PROT->PROT_ITP_CORRECTION & USB32DEV_PROT_ITP_CORRECTION_CORRECTION_Msk);
                calc_dc = (uint32_t)(((isochDelay+tLinkDelay+hw_dc) - correction + 60) % 7500);
                calc_bic = hw_bic + (uint32_t)((tLinkDelay + hw_dc)/7500);

                /* Load PTM BIC and DC values in FW counter */
                USB32DEV_PROT->PROT_FW_PTM_COUNTER = ((calc_bic << USB32DEV_PROT_HW_PTM_COUNTER_BIC_Pos) |
                                                      (calc_dc << USB32DEV_PROT_HW_PTM_COUNTER_DC_Pos));

                /* Set the Load bit in PROT_FW_COUNTER */
                USB32DEV_PROT->PROT_FW_PTM_COUNTER |= USB32DEV_PROT_FW_PTM_COUNTER_LOAD_Msk;
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_LDM_RX_Msk;
                USB32DEV_PROT->PROT_PTM_CONFIG &= ~USB32DEV_PROT_PTM_CONFIG_WAIT_ITP_Msk;

                /* Set the LDM Link Delay Value and LDM Valid bit in the PTM STATUS repsonse field */
                pCalCtxt->ptmStatus = ((uint16_t)(tLinkDelay & 0xFFFFUL) << CY_USBSS_PTM_STAT_LDM_VALUE_Pos)
                                        | CY_USBSS_PTM_STAT_LDM_ENABLE_Msk
                                        | CY_USBSS_PTM_STAT_LDM_VALID_Msk;
            }

            if (prot_intr & USB32DEV_PROT_INTR_LMP_RCV_EV_Msk) {
                /* Clear the interrupt. */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_LMP_RCV_EV_Msk;

                /* Set/Clear Force LPM accept setting based on LMP received. */
                if ((USB32DEV_PROT->PROT_LMP_RECEIVED & USB32DEV_PROT_LMP_RECEIVED_FORCE_LINKPM_ACCEPT_Msk) != 0) {
                    Cy_USBSS_Cal_ForceLPMAccept(pCalCtxt, true);
                    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_LNK_EVENT_SET_FORCE_ACCEPT_LPM);
                } else {
                    Cy_USBSS_Cal_ForceLPMAccept(pCalCtxt, false);
                    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_LNK_EVENT_CLR_FORCE_ACCEPT_LPM);
                }

                /*
                   If host has defined a U2 inactivity timeout value, override a reduced value.
                 */
                if ((USB32DEV_PROT->PROT_LMP_RECEIVED & USB32DEV_PROT_LMP_RECEIVED_U2_INACTIVITY_TIMEOUT_Msk) !=
                        pCalCtxt->u2TimeoutVal) {
                    pCalCtxt->u2TimeoutVal = USB32DEV_PROT->PROT_LMP_RECEIVED &
                        USB32DEV_PROT_LMP_RECEIVED_U2_INACTIVITY_TIMEOUT_Msk;
                    if ((pCalCtxt->u2TimeoutVal > 1) && (pCalCtxt->u2TimeoutVal < 0xFFU)) {
                        USB32DEV_PROT->PROT_LMP_OVERRIDE = (
                                (pCalCtxt->u2TimeoutVal - 1) |
                                USB32DEV_PROT_LMP_OVERRIDE_INACITIVITY_TIMEOUT_OVR_Msk);
                    } else {
                        USB32DEV_PROT->PROT_LMP_OVERRIDE = 0;
                    }
                }
            }

            if (prot_intr & USB32DEV_PROT_INTR_LMP_PORT_CAP_EV_Msk) {
                /* Disable TX/RX of Port CAP LMP till next entry into Polling state. */
                USB32DEV_PROT->PROT_LMP_PORT_CAPABILITY_TIMER = 0x80008000UL;
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_LMP_PORT_CAP_EV_Msk;

                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PORT_CAP_HNDSHK);
                DBG_SSCAL_TRACE("PORT_CAP\r\n");
            }

            if (prot_intr & USB32DEV_PROT_INTR_LMP_PORT_CFG_EV_Msk) {
                /* Disable TX/RX of Port CFG LMP till next entry into Polling state. */
                USB32DEV_PROT->PROT_LMP_PORT_CONFIGURATION_TIMER = 0x80008000UL;
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_LMP_PORT_CFG_EV_Msk;

                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_PORT_CFG_HNDSHK);

                /* Link is stable in U0: switch to user selected DMA clock frequency. */
                Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, true);

                msg.type = CY_USBSS_CAL_MSG_PORT_CONFIGURED;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Handle SetAddress completion. */
            if (prot_intr & USB32DEV_PROT_INTR_SET_ADDR_Msk) {
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_SETADDR_INTR);
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_SET_ADDR_Msk;
                msg.type = CY_USB_CAL_MSG_SETADDR;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Handle STATUS STAGE completion */
            if (prot_intr & USB32DEV_PROT_INTR_STATUS_STAGE_Msk) {
#if DEBUG_ENABLE
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_EP0STAT_INTR);
#endif /* DEBUG_ENABLE */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_STATUS_STAGE_Msk;

                /* If LPM entry had been delayed, enable it now. */
                if (pCalCtxt->lpmConfig == CY_SSCAL_LPM_DELAYED) {
                    Cy_USBSS_Cal_LPMEnable(pCalCtxt, true);
                }

                msg.type = CY_USB_CAL_MSG_STATUS_STAGE;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Handle SUTOK_EV comes for setup Packet */
            if (prot_intr & USB32DEV_PROT_INTR_SUTOK_EV_Msk) {
#if DEBUG_ENABLE
                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_SUTOK_INTR);
#endif /* DEBUG_ENABLE */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_SUTOK_EV_Msk;

                /* Disable LPM entry while request is pending, if force Ux accept is not set. */
                Cy_USBSS_Cal_DelayLpmEnable(pCalCtxt);

                /* Get setupData, prepare msg and send msg to upper layer. */
                msg.type = CY_USB_CAL_MSG_PROT_SUTOK;
                msg.data[0] = USB32DEV_PROT->PROT_SETUPDAT0;
                msg.data[1] = USB32DEV_PROT->PROT_SETUPDAT1;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Handle SetAddress 0.  Device should go to default state. */
            if (prot_intr & USB32DEV_PROT_INTR_SET_ADDR0_EV_Msk) {
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_SET_ADDR0_EV_Msk;
                msg.type = CY_USB_CAL_MSG_PROT_SETADDR_0;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            if (prot_intr & USB32DEV_PROT_INTR_HOST_ERR_EV_Msk) {
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_HOST_ERR_EV_Msk;
                msg.type = CY_USB_CAL_MSG_PROT_HOST_ERR;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }

            if (prot_intr & USB32DEV_PROT_INTR_TIMEOUT_PORT_CAP_EV_Msk) {
                /* Clear the interrupt and notify USBD about the timeout. */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_TIMEOUT_PORT_CAP_EV_Msk;
                Cy_USBSS_Cal_DisConnect(pCalCtxt);
                msg.type = CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);

                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_LMP_TIMEOUT);
            }

            if (prot_intr & USB32DEV_PROT_INTR_TIMEOUT_PORT_CFG_EV_Msk) {
                /* Clear the interrupt and notify USBD about the timeout. */
                USB32DEV_PROT->PROT_INTR = USB32DEV_PROT_INTR_TIMEOUT_PORT_CFG_EV_Msk;
                Cy_USBSS_Cal_DisConnect(pCalCtxt);
                msg.type = CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);

                Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_LMP_TIMEOUT);
            }

            if (prot_intr & USB32DEV_PROT_INTR_TIMEOUT_PING_EV_Msk) {
                USB32DEV_PROT->PROT_INTR =
                                        USB32DEV_PROT_INTR_TIMEOUT_PING_EV_Msk;
                msg.type = CY_USB_CAL_MSG_PROT_TIMEOUT_PING;
                yield_reqd |= Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
            }
        }
    }

    if ((state & USB32DEV_MAIN_INTR_PROT_EP_Msk) != 0) {
        uint32_t tmp1, tmp2;
        uint32_t epIdx;
        uint8_t  seqnum;

        /* Clear interrupt by writing 1 */
        USB32DEV_MAIN->INTR = (USB32DEV_MAIN_INTR_PROT_EP_Msk);

        /* Read the PROT_EP_INTR register after clearing the PROT_EP bit in the INTR register. */
        tmp1 = USB32DEV_PROT->PROT_EP_INTR_MASKED;

        for (epIdx = 0; epIdx < 16; epIdx++) {
            if ((tmp1 & (0x00001UL << epIdx)) != 0) {
                /* Interrupt pending on epIdx-IN. */
                tmp2 = USB32DEV_PROT->PROT_EPI_INTR_MASKED[epIdx];
                USB32DEV_PROT->PROT_EPI_INTR[epIdx] = tmp2;

                if ((tmp2 & USB32DEV_PROT_EPI_INTR_OOSERR_Msk) != 0) {
                    Cy_USBSS_Cal_GetSeqNum(pCalCtxt, epIdx, CY_USB_ENDP_DIR_IN, &seqnum);
                    DBG_SSCAL_INFO("Sequence number error on EP %d-in with expected sequence %d\r\n", epIdx, seqnum);
                }
            }

            if ((tmp1 & (0x10000UL << epIdx)) != 0) {
                /* Interrupt pending on epIdx-OUT. */
                tmp2 = USB32DEV_PROT->PROT_EPO_INTR_MASKED[epIdx];
                USB32DEV_PROT->PROT_EPO_INTR[epIdx] = tmp2;
            }
        }
    }

    if (state & (USB32DEV_MAIN_INTR_EPM_URUN_Msk)) {
       USB32DEV_MAIN->INTR = USB32DEV_MAIN_INTR_EPM_URUN_Msk;

       /* Callback to the USB middleware for setting the event */
       msg.type = CY_USBSS_CAL_MSG_EPM_UNDERRUN;
       Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_WakeupIntrHandler
****************************************************************************//**
*
* Wakeup Interrupt Handler function for USB3 IP.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*******************************************************************************/
void
Cy_USBSS_Cal_WakeupIntrHandler (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    uint32_t phy_intr;

    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs = &(base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP);
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegsAlt = &(base->USB32DEV_PHYSS.USB40PHY[!(pCalCtxt->activePhyIdx)].USB40PHY_TOP);

    phy_intr = pPhyRegs->INTR1_MASKED;
    if (phy_intr != 0) {
        pPhyRegs->INTR1 = phy_intr;

        if (pCalCtxt->linkSuspended) {
            DBG_SSCAL_INFO("WakeIntr:0x%x 0x%x\r\n", pPhyRegs->TOP_CTRL_0, pPhyRegsAlt->TOP_CTRL_0);

            /* Run through steps required to wake the PHY if required. */
            Cy_USBSS_Cal_DeepSleepExit(pCalCtxt);

            pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
            if (pCalCtxt->dualLaneEnabled) {
                pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
            }
            Cy_SysLib_DelayUs(1);
            USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;

            /* Clear the wake interrupt. Interrupt will be disabled when link gets to Recovery
             * or there is a Warm Reset. */
            pPhyRegs->INTR1 = pPhyRegs->INTR1;

            /* Switch to the desired HBWSS clock frequency. */
            Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, true);
        }
    }
}

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
uint32_t
Cy_USBSS_Cal_Get_PtmStatus (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return pCalCtxt->ptmStatus;
}


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_LPMEnable (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool isResume)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;

    if ((pCalCtxt->lpmConfig != CY_SSCAL_LPM_DELAYED) || (isResume)) {
        pCalCtxt->lpmConfig = CY_SSCAL_LPM_ENABLED;

        /*
         * A0 Silicon Work-around:
         * CDT_005497-1711: If EXIT_LP bit is 1, DUT will attempt U1 exit as soon as U1 is entered. So, disallow
         *                  LPM acceptance if EXIT_LP bit is 1.
         */
        if (
                ((USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL & USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk) != 0)
           ) {
            if (!pCalCtxt->lpmWarningDone) {
                DBG_SSCAL_INFO("LPMReEnable fail: %x\r\n", USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL);
                pCalCtxt->lpmWarningDone = true;
            }
        } else {
            pCalCtxt->lpmWarningDone = false;
            USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL =
                USB32DEV_LNK_DEVICE_POWER_CONTROL_AUTO_U1_Msk |
                USB32DEV_LNK_DEVICE_POWER_CONTROL_AUTO_U2_Msk;
        }
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ForceLPMAccept (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool enable)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    USB32DEV_PROT_Type *USB32DEV_PROT = &base->USB32DEV_PROT;

    if (enable) {
        pCalCtxt->forceLPMAccept = true;

        /* Allow U1/U2 entry to to be accepted always. */
        USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL = USB32DEV_LNK_DEVICE_POWER_CONTROL_YES_U1_Msk |
                                                 USB32DEV_LNK_DEVICE_POWER_CONTROL_YES_U2_Msk;

    } else {
        pCalCtxt->forceLPMAccept = false;

        /* Revert to previous setting of U1/U2 enable. */
        if (pCalCtxt->lpmConfig != CY_SSCAL_LPM_ENABLED) {
            USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL = USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U1_Msk |
                USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U2_Msk;
        } else {
            USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL = USB32DEV_LNK_DEVICE_POWER_CONTROL_AUTO_U1_Msk |
                USB32DEV_LNK_DEVICE_POWER_CONTROL_AUTO_U2_Msk;
        }
    }

    if ((pCalCtxt->forceLPMAccept) && ((USB32DEV_PROT->PROT_CS & 0x7FUL) == 0)) {
        /* When force LPM accept is set and device is not addressed, use low duration for U1 exit LFPS. */
        USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT      = 130UL;         /* 1.04 us */
        USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT_GEN2 = 325UL;         /* 1.04 us */

        /* Set U1 exit RX LFPS duration to default value of 300 ns. */
        USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT           = 38;
        USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE      = 38;
        USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT_GEN2      = 94;
        USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE_GEN2 = 94;
    } else {
        /* Increase U1 exit LFPS duration to 120 us */
        USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT      = CY_USBSS_LFPS_PERIOD_TO_REG_G1(CY_USBSS_U1_EXIT_LFPS_DURATION);
        USB32DEV_LNK->LNK_LFPS_TX_U1_EXIT_GEN2 = CY_USBSS_LFPS_PERIOD_TO_REG_G2(CY_USBSS_U1_EXIT_LFPS_DURATION);

        /* Increase U1 exit RX LFPS duration to 600 ns. */
        USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT           = 75;
        USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE      = 75;
        USB32DEV_LNK->LNK_LFPS_RX_U1_EXIT_GEN2      = 187;
        USB32DEV_LNK->LNK_LFPS_RX_U1_HANDSHAKE_GEN2 = 187;
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_LPMDisable (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;

    pCalCtxt->lpmConfig = CY_SSCAL_LPM_DISABLED;
    USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL = USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U1_Msk |
                                             USB32DEV_LNK_DEVICE_POWER_CONTROL_NO_U2_Msk;

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_U3WakePrepare
****************************************************************************//**
*
* Prepares the receive part of the USB40 PHY for an extended U3 exit LFPS
* sequence so that the CDR does not try to lock to data prematurely.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* void
*******************************************************************************/
static void Cy_USBSS_Cal_U3WakePrepare (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                        uint8_t phyIdx)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs = &(base->USB32DEV_PHYSS.USB40PHY[phyIdx].USB40PHY_TOP);
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegsAlt = &(base->USB32DEV_PHYSS.USB40PHY[!phyIdx].USB40PHY_TOP);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegs = &(base->USB32DEV_PHYSS.USB40PHY[phyIdx].USB40PHY_RX);
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegsAlt = &(base->USB32DEV_PHYSS.USB40PHY[!phyIdx].USB40PHY_RX);

    /* Disable hardware control of CDR. */
    pRxRegs->RX_HDWR_ENABLE    &= ~USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;

    /* Configure RX for L2R mode. */
    pRxRegs->RX_LD_CFG = (
            (pRxRegs->RX_LD_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Msk) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Pos));

    if (pCalCtxt->dualLaneEnabled) {
        pRxRegsAlt->RX_HDWR_ENABLE &= ~USB32DEV_PHYSS_USB40PHY_RX_HDWR_ENABLE_CDR_HDWR_ENABLE_Msk;

        pRxRegsAlt->RX_LD_CFG = (
                (pRxRegsAlt->RX_LD_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Msk) |
                (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_FD_OVRD_Pos));

        pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
    }

    /* Enable PHY and IP Clock. */
    pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
    Cy_SysLib_DelayUs(1);
    base->USB32DEV_MAIN.CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    /* Switch to the desired HBWSS clock frequency. */
    Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, true);
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetLinkPowerState (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                cy_en_usbss_lnk_power_mode_t lnkMode)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    cy_en_usb_cal_ret_code_t ret = CY_USB_CAL_STATUS_FAILURE;
    uint8_t state;

    (void)USB32DEV_MAIN;

    state = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;

    switch (lnkMode)
    {
    case CY_USBSS_LPM_U0:
        /* Do nothing if the EXIT_LP bit is already set, or if the link state is not {U1,U2,U3}. */
        if ((USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL & USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk) == 0) {
            state = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;
            if (
                    (state >= CY_USBSS_LNK_STATE_U1) &&
                    (state <= CY_USBSS_LNK_STATE_U3)
               ) {
                /* If device initiated LPM exit is disabled, return error. */
                if (pCalCtxt->devLpmExitDisabled) {
                    return CY_USB_CAL_STATUS_FAILURE;
                }

                /* Special handling required for U3 exit. */
                if (pCalCtxt->linkSuspended) {
                    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_DEV_U3_EXIT);
                    Cy_USBSS_Cal_U3WakePrepare(pCalCtxt, pCalCtxt->activePhyIdx);
                }

                /* Delay sending of TS1 in Recovery.Active state by 10 us so that we have sufficient
                 * time to apply work-arounds.
                 */
                if (pCalCtxt->gen2Enabled) {
                    USB32DEV_LNK->LNK_TX_TS1_HOLDOFF_TIMEOUT = 3125UL;
                } else {
                    USB32DEV_LNK->LNK_TX_TS1_HOLDOFF_TIMEOUT = 1250UL;
                }

                /*
                 * Work-around for FX10 RevA0 limitation: PCLK needs to be paused before we request device to
                 * initiate U1 exit and then resumed. This sequence is not needed for U2/U3 exit.
                 */
                if ((state == CY_USBSS_LNK_STATE_U1) && (Cy_SysLib_GetDeviceRevision() == 0x11)) {
                    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;
                    pCalCtxt->uxExitActive = false;
                    USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL |= USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk;
                    USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;
                } else {
                    if (Cy_SysLib_GetDeviceRevision() == 0x11) {
                        pCalCtxt->uxExitActive = false;
                    }
                    USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL |= USB32DEV_LNK_DEVICE_POWER_CONTROL_EXIT_LP_Msk;
                }
            }
        }
        ret = CY_USB_CAL_STATUS_SUCCESS;
        break;

    case CY_USBSS_LPM_U1:
        if (state == CY_USBSS_LNK_STATE_U0) {
            USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL |= USB32DEV_LNK_DEVICE_POWER_CONTROL_TX_U1_Msk;
            ret = CY_USB_CAL_STATUS_SUCCESS;
        }
        break;

    case CY_USBSS_LPM_U2:
        if (state == CY_USBSS_LNK_STATE_U0) {
            USB32DEV_LNK->LNK_DEVICE_POWER_CONTROL |= USB32DEV_LNK_DEVICE_POWER_CONTROL_TX_U2_Msk;
            ret = CY_USB_CAL_STATUS_SUCCESS;
        }
        break;

    default:
        ret = CY_USB_CAL_STATUS_BAD_PARAM;
        break;
    }

    return ret;
}

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
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
void Cy_USBSS_Cal_GetUsbLinkActive (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    USB32DEV_PROT_Type *USB3DEV_PROT = &base->USB32DEV_PROT;
    uint32_t state;

    /* Make sure that any EP stall is cleared up at this stage. */
    USB3DEV_PROT->PROT_EPI_CS1[0] &= ~CY_USBSS_PROT_EPI_CS1_STALL;
    USB3DEV_PROT->PROT_EPO_CS1[0] &= ~CY_USBSS_PROT_EPO_CS1_STALL;

    /* If the Link is in U1/U2, try and get it back to U0 before moving on. */
    state = USB32DEV_LNK->LNK_LTSSM_STATE &  USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;
    if ((state >= CY_USBSS_LNK_STATE_U1) && (state <= CY_USBSS_LNK_STATE_U3)) {
        Cy_USBSS_Cal_SetLinkPowerState (pCalCtxt, CY_USBSS_LPM_U0);
    }
}

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetLinkPowerState (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                cy_en_usbss_lnk_power_mode_t *pMode)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type  *USB32DEV_LNK = &base->USB32DEV_LNK;
    uint8_t state;

    if (pMode == 0)
        return CY_USB_CAL_STATUS_BAD_PARAM;
/* TODO: Need to add this parameter
    if (glUibDeviceInfo.ssCompliance)
    {
        *pMode = CY_USBSS_LPM_COMP;
        return CY_USB_CAL_STATUS_SUCCESS;
    }
*/
    state = USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk;
    switch (state)
    {
    case CY_USBSS_LNK_STATE_U0:
        *pMode = CY_USBSS_LPM_U0;
        break;
    case CY_USBSS_LNK_STATE_U1:
        *pMode = CY_USBSS_LPM_U1;
        break;
    case CY_USBSS_LNK_STATE_U2:
        *pMode = CY_USBSS_LPM_U2;
        break;
    case CY_USBSS_LNK_STATE_U3:
        *pMode = CY_USBSS_LPM_U3;
        break;
    case CY_USBSS_LNK_STATE_COMP:
        *pMode = CY_USBSS_LPM_COMP;
        break;
    default:
        *pMode = CY_USBSS_LPM_UNKNOWN;
        break;
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

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
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_ProtSendTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t *pTpData)
{
    USB32DEV_PROT_Type *pProtBase = pCalCtxt->pProtBase;

    /*
     * Update the device address in b31:b25 of the first word. This is common
     * for all Transaction Packets.
     */
    pTpData[0] = ((pTpData[0] & 0x01FFFFFFUL) | (
                ((pProtBase->PROT_CS & USB32DEV_PROT_CS_DEVICEADDR_Msk) >> USB32DEV_PROT_CS_DEVICEADDR_Pos)
                << CY_USB32DEV_PROT_TP_DEVADDR_POS));

    /* prot layer fills first three word and last word filled by LinkLayer */
    pProtBase->PROT_LMP_PACKET_TX[0] =  pTpData[0];
    pProtBase->PROT_LMP_PACKET_TX[1] =  pTpData[1];
    pProtBase->PROT_LMP_PACKET_TX[2] =  pTpData[2];
    pProtBase->PROT_LMP_OVERRIDE |= CY_USB32DEV_PROT_LMP_OVERRIDE_LMP_SEND;
    while ((pProtBase->PROT_LMP_OVERRIDE & CY_USB32DEV_PROT_LMP_OVERRIDE_LMP_SEND) != 0);
    return;
}   /* end of function  */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendAckTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir, uint8_t numP,
                            uint16_t bulkStream)
{
    USB32DEV_PROT_Type *pProtBase;
    uint32_t tpPkt[3] = {0,0,0};
    uint8_t seqNum = 0x00;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /* Get the next sequence number for this endpoint. */
    Cy_USBSS_Cal_GetSeqNum(pCalCtxt, endpNum, endpDir, &seqNum);

    /* Now prepare ACK TP packet. */
    tpPkt[0] = CY_USBSS_PKT_TYPE_TP;
    tpPkt[1] = ((seqNum << CY_USB32DEV_PROT_TP_SEQ_NUM_POS) |
                (numP << CY_USB32DEV_PROT_TP_NUMP_POS) |
                (CY_USBSS_TP_SUBTYPE_ACK << CY_USB32DEV_PROT_TP_SUBTYPE_POS) |
                (endpDir << CY_USB32DEV_PROT_TP_DIR_POS) |
                (endpNum << CY_USB32DEV_PROT_TP_ENDP_NUM_POS));

    /* If Stream is enabled then update stream info also in TP */
    if (endpDir) {
        if (pProtBase->PROT_EPI_CS1[endpNum] & USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
         }
    } else {
        if (pProtBase->PROT_EPO_CS1[endpNum] & USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
        }
    }

    Cy_USBSS_Cal_ProtSendTp(pCalCtxt, tpPkt);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendErdyTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                             cy_en_usb_endp_dir_t endpDir, uint8_t numP,
                             uint16_t bulkStream)
{
    USB32DEV_PROT_Type *pProtBase;
    uint32_t tpPkt[3] = {0,0,0};

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /* Now prepare ERDY TP packet. */
    tpPkt[0]  = CY_USBSS_PKT_TYPE_TP;
    tpPkt[1] |= ((numP << CY_USB32DEV_PROT_TP_NUMP_POS) |
                 (CY_USBSS_TP_SUBTYPE_ERDY << CY_USB32DEV_PROT_TP_SUBTYPE_POS) |
                  (endpDir << CY_USB32DEV_PROT_TP_DIR_POS) |
                  (endpNum << CY_USB32DEV_PROT_TP_ENDP_NUM_POS));

    /* If Stream is enabled then update stream info also in TP */
    if (endpDir) {
        if (pProtBase->PROT_EPI_CS1[endpNum] & USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
         }
    } else {
        if (pProtBase->PROT_EPO_CS1[endpNum] & USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
        }
    }
    Cy_USBSS_Cal_ProtSendTp(pCalCtxt, tpPkt);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendZlpTp
****************************************************************************//**
*
* This function prepares and send ZLP transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
/* TBD: Need to add endp number as paramter so zlp can be sent for any endp.*/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendZlpTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

    USB32DEV_PROT_Type *pProtBase;
    uint32_t tpPkt[3] = {0,0,0};

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /* Now prepare the Data Packet Header. */
    tpPkt[0]  = CY_USBSS_PKT_TYPE_DPH;
    tpPkt[1] |= 0xC0;
    tpPkt[2]  = 0;

    Cy_USBSS_Cal_ProtSendTp(pCalCtxt, tpPkt);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendNrdyTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                             cy_en_usb_endp_dir_t endpDir, uint16_t bulkStream)
{
    USB32DEV_PROT_Type *pProtBase;
    uint32_t tpPkt[3] = {0,0,0};

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /* Now prepare NRDY TP packet. */
    tpPkt[0]  = CY_USBSS_PKT_TYPE_TP;
    tpPkt[1] |=   ((CY_USBSS_TP_SUBTYPE_NRDY << CY_USB32DEV_PROT_TP_SUBTYPE_POS) |
                  (endpDir << CY_USB32DEV_PROT_TP_DIR_POS) |
                  (endpNum << CY_USB32DEV_PROT_TP_ENDP_NUM_POS));

    /* If Stream is enabled then update stream info also in TP */
    if (endpDir) {
        if (pProtBase->PROT_EPI_CS1[endpNum] & USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
         }
    } else {
        if (pProtBase->PROT_EPO_CS1[endpNum] & USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk) {
            tpPkt[2] = bulkStream;
        }
    }
    Cy_USBSS_Cal_ProtSendTp(pCalCtxt, tpPkt);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetClearStall
****************************************************************************//**
*
* This function enable or disable STALL condition in hw.
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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetClearStall (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                uint32_t endpNum,
                                cy_en_usb_endp_dir_t endpDir,
                                bool setClear)
{
    USB32DEV_PROT_Type *pProtBase;
    USB32DEV_EPM_Type *pEpmBase;
    cy_en_usb_cal_ret_code_t retCode;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;
    pEpmBase = pCalCtxt->pEpmBase;

    if ((NULL == pProtBase) || (NULL == pEpmBase)) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    retCode = CY_USB_CAL_STATUS_SUCCESS;

    /* Endpoint 0 is special case */
    switch (endpNum) {
        case 0:
            if (setClear) {
                DBG_SSCAL_INFO("EP0Stall\r\n");
                pProtBase->PROT_EPI_CS1[0] |= CY_USBSS_PROT_EPI_CS1_STALL;
                pProtBase->PROT_EPO_CS1[0] |= CY_USBSS_PROT_EPO_CS1_STALL;
                Cy_SysLib_DelayUs(1);
                pProtBase->PROT_CS |= (CY_USBSS_PROT_CS_SETUP_CLR_BUSY);
            } else {
                /* Clear EP0 stall bits. No need to clear sequence number explicitly. */
                pProtBase->PROT_EPI_CS1[0] &= ~CY_USBSS_PROT_EPI_CS1_STALL;
                pProtBase->PROT_EPO_CS1[0] &= ~CY_USBSS_PROT_EPO_CS1_STALL;
                return CY_USB_CAL_STATUS_SUCCESS;
            }
            break;

        default:
            if (CY_USB_ENDP_DIR_OUT == endpDir) {
                if (setClear) {
                    DBG_SSCAL_INFO("EPO-%x stall\r\n", endpNum);
                    pProtBase->PROT_EPO_CS1[endpNum] |= CY_USBSS_PROT_EPO_CS1_STALL;
                    if ((pEpmBase->IEPM_ENDPOINT[endpNum] & USB32DEV_EPM_IEPM_ENDPOINT_EP_READY_Msk) == 0) {
                        Cy_USBSS_Cal_ProtSendErdyTp(pCalCtxt, endpNum, CY_USB_ENDP_DIR_OUT, 1U, 0);
                    } else {
                        DBG_SSCAL_INFO("Skip ERDY as EP is ready\r\n");
                    }
                } else {
                    /* Clear the STALL bit. Reset is to be done separately. */
                    pProtBase->PROT_EPO_CS1[endpNum] &= ~CY_USBSS_PROT_EPO_CS1_STALL;
                }
            } else {
                /* Handle IN endpoint. */
                if (setClear) {
                    DBG_SSCAL_INFO("EPI-%x stall\r\n", endpNum);
                    pProtBase->PROT_EPI_CS1[endpNum] |= CY_USBSS_PROT_EPI_CS1_STALL;
                    if ((pEpmBase->EEPM_ENDPOINT[endpNum] & USB32DEV_EPM_IEPM_ENDPOINT_EP_READY_Msk) == 0) {
                        Cy_USBSS_Cal_ProtSendErdyTp(pCalCtxt, endpNum, CY_USB_ENDP_DIR_IN, 1U, 0);
                    } else {
                        DBG_SSCAL_INFO("Skip ERDY as EP is ready\r\n");
                    }
                } else {
                    /* Clear the STALL bit. Reset is to be done separately. */
                    pProtBase->PROT_EPI_CS1[endpNum] &= ~CY_USBSS_PROT_EPI_CS1_STALL;
                }
            }
            break;
    }

    return(retCode);
}   /* end of function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpReset
****************************************************************************//**
*
* Function to flush and reset stale state and data on an endpoint. This function
* will cause any data already present to be dropped and the sequence number on
* the endpoint to be cleared.
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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpReset (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir)
{
    USB32DEV_PROT_Type *pProtBase;
    USB32DEV_EPM_Type  *pEpmBase;
    USB32DEV_MAIN_Type *pMainBase;
    uint32_t intState;
    bool stopClkEn = false;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pProtBase = pCalCtxt->pProtBase;
    pMainBase = pCalCtxt->pMainBase;
    pEpmBase  = pCalCtxt->pEpmBase;
    if ((NULL == pMainBase) || (NULL == pProtBase) || (NULL == pEpmBase)) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    stopClkEn = ((pCalCtxt->stopClkOnEpResetEnable) && (endpNum != 0));

    intState = Cy_SysLib_EnterCriticalSection();

    if ((pProtBase->PROT_CS & USB32DEV_PROT_CS_DEVICEADDR_Msk) != 0) {
        /* Set the NRDYALL bit to block all data transfers and wait for one service interval. */
        pProtBase->PROT_CS = (
                (pProtBase->PROT_CS & ~(USB32DEV_PROT_CS_SETUP_CLR_BUSY_Msk | USB32DEV_PROT_CS_STATUS_CLR_BUSY_Msk)) |
                USB32DEV_PROT_CS_NRDY_ALL_Msk);
        Cy_SysLib_DelayUs(125U);

        if (stopClkEn) {
            pMainBase->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;
        }
    }

    if ((CY_USB_ENDP_DIR_OUT == endpDir) || (endpNum == 0)) {
        pEpmBase->IEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pProtBase->PROT_EPO_CS1[endpNum] |= (CY_USBSS_PROT_EPO_CS1_RESET | CY_USBSS_PROT_EPO_CS1_VALID);
    }

    if ((CY_USB_ENDP_DIR_IN == endpDir) || (endpNum == 0)) {
        pEpmBase->EEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pProtBase->PROT_EPI_CS1[endpNum] |= (CY_USBSS_PROT_EPI_CS1_RESET | CY_USBSS_PROT_EPI_CS1_VALID);
    }

    if (stopClkEn) {
        pMainBase->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;
        __NOP();__NOP();__NOP();__NOP();
        pMainBase->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;
    } else {
        __NOP();__NOP();__NOP();__NOP();
    }

    if ((CY_USB_ENDP_DIR_OUT == endpDir) || (endpNum == 0)) {
        pEpmBase->IEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pProtBase->PROT_EPO_CS1[endpNum] &= ~CY_USBSS_PROT_EPO_CS1_RESET;
    }

    if ((CY_USB_ENDP_DIR_IN == endpDir) || (endpNum == 0)) {
        pEpmBase->EEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pProtBase->PROT_EPI_CS1[endpNum] &= ~CY_USBSS_PROT_EPI_CS1_RESET;
    }

    /* Clear stream error detected status. */
    pProtBase->PROT_STREAM_ERROR_STATUS |= CY_USBSS_PROT_STREAM_ERROR_STATUS_ERROR_DETECTED;

    if (stopClkEn) {
        /* Make sure PCLK is enabled. */
        pMainBase->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;

        /* In case of Gen1x2 operation, force link through recovery manually after starting the clock. */
        if ((pCalCtxt->dualLaneEnabled) && (!pCalCtxt->gen2Enabled)) {
            if ((pCalCtxt->pLinkBase->LNK_LTSSM_STATE & 0x3F) == 0x10) {
                CY_USBSS_JUMP_TO_LINK_STATE(pCalCtxt->pLinkBase, CY_USBSS_LNK_STATE_RECOV_ACT);
            }
        }
    }

    /* Clear the NRDY_ALL bit to allow data transfers to resume. */
    pProtBase->PROT_CS = (pProtBase->PROT_CS & ~(
                USB32DEV_PROT_CS_SETUP_CLR_BUSY_Msk |
                USB32DEV_PROT_CS_STATUS_CLR_BUSY_Msk |
                USB32DEV_PROT_CS_NRDY_ALL_Msk));

    Cy_SysLib_ExitCriticalSection(intState);

    /*
     * Make sure that the endpoint sequence number is cleared at this point.
     * This happens implicitly for IN endpoints. Explicit clear is only needed for OUT endpoints.
     */
    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        Cy_USBSS_Cal_SetSeqNum(pCalCtxt, endpNum, endpDir, 0);
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of Function */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetSeqNum (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir,  uint8_t *pSeqNum)
{
    USB32DEV_PROT_Type *pProtBase;
    uint32_t protSeqNumber;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if ((endpNum > 15) || (pSeqNum == NULL)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /* Set the endpoint number and read the sequence number after a delay. */
    protSeqNumber = pProtBase->PROT_SEQ_NUM;
    if (CY_USB_ENDP_DIR_IN == endpDir) {
        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_EGRESS_EP_NUM_Msk;
        protSeqNumber |= (endpNum << USB32DEV_PROT_SEQ_NUM_EGRESS_EP_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);
        protSeqNumber = pProtBase->PROT_SEQ_NUM;
        *pSeqNum = ((protSeqNumber & USB32DEV_PROT_SEQ_NUM_EGRESS_READ_SEQ_NUM_Msk) >>
                USB32DEV_PROT_SEQ_NUM_EGRESS_READ_SEQ_NUM_Pos);
    } else {
        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_INGRESS_EP_NUM_Msk;
        protSeqNumber |= (endpNum << USB32DEV_PROT_SEQ_NUM_INGRESS_EP_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);
        protSeqNumber = pProtBase->PROT_SEQ_NUM;
        *pSeqNum = ((protSeqNumber & USB32DEV_PROT_SEQ_NUM_INGRESS_READ_SEQ_NUM_Msk) >>
                USB32DEV_PROT_SEQ_NUM_INGRESS_READ_SEQ_NUM_Pos);
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


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
* \param pSeqNum
* sequence number need to be written in HW register.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetSeqNum (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir,  uint8_t seqNum)
{
    USB32DEV_PROT_Type *pProtBase;
    uint32_t protSeqNumber;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;
    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if (((endpNum == 0) || (endpNum > 15)) || (seqNum >= 32)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /* TBD: Need lock here  */

    /*
     * Select the endpoint first and update the sequence number after a delay.
     * The IP updates sequence number when the WRITE_SEQ_NUM field is changed.
     * If the sequence number was already correct (matching the desired value),
     * this sequence will be benign and a check is not required.
     */
    protSeqNumber = pProtBase->PROT_SEQ_NUM;
    if (CY_USB_ENDP_DIR_IN == endpDir) {
        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_EGRESS_EP_NUM_Msk;
        protSeqNumber |= (endpNum << USB32DEV_PROT_SEQ_NUM_EGRESS_EP_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);

        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_EGRESS_WRITE_SEQ_NUM_Msk;
        protSeqNumber |= (((seqNum + 1UL) & 0x1FUL) << USB32DEV_PROT_SEQ_NUM_EGRESS_WRITE_SEQ_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);

        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_EGRESS_WRITE_SEQ_NUM_Msk;
        protSeqNumber |= (seqNum << USB32DEV_PROT_SEQ_NUM_EGRESS_WRITE_SEQ_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
    } else {
        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_INGRESS_EP_NUM_Msk;
        protSeqNumber |= (endpNum << USB32DEV_PROT_SEQ_NUM_INGRESS_EP_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);

        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_INGRESS_WRITE_SEQ_NUM_Msk;
        protSeqNumber |= (((seqNum + 1UL) & 0x1FUL) << USB32DEV_PROT_SEQ_NUM_INGRESS_WRITE_SEQ_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
        Cy_SysLib_DelayUs(1);

        protSeqNumber &= ~USB32DEV_PROT_SEQ_NUM_INGRESS_WRITE_SEQ_NUM_Msk;
        protSeqNumber |= (seqNum << USB32DEV_PROT_SEQ_NUM_INGRESS_WRITE_SEQ_NUM_Pos);
        pProtBase->PROT_SEQ_NUM = protSeqNumber;
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
} /* End of function */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetDevAddress (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t *pDevAddr)
{

    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if (pDevAddr == NULL) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    *pDevAddr = (uint8_t)((pProtBase->PROT_CS & USB32DEV_PROT_CS_DEVICEADDR_Msk) >>
           (USB32DEV_PROT_CS_DEVICEADDR_Pos));
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function Cy_USBSS_Cal_GetDevAddress() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetClearNrdy (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNum,
                               cy_en_usb_endp_dir_t endpDir,
                               bool setClear)
{
    USB32DEV_PROT_Type *pProtBase;
    cy_en_usb_cal_ret_code_t retCode;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    retCode = CY_USB_CAL_STATUS_SUCCESS;

    /* Endpoint 0 is special case */
    switch (endpNum) {
        case 0:
            if (setClear) {
                pProtBase->PROT_EPI_CS1[0] |= CY_USBSS_PROT_EPI_CS1_NRDY;
                pProtBase->PROT_EPO_CS1[0] |= CY_USBSS_PROT_EPO_CS1_NRDY;
            } else {
                pProtBase->PROT_EPI_CS1[0] &= (~CY_USBSS_PROT_EPI_CS1_NRDY);
                pProtBase->PROT_EPO_CS1[0] &= (~CY_USBSS_PROT_EPO_CS1_NRDY);
            }
            break;

        /* case fall through 1 to 16 */
        case 1:   case 2:   case 3:
        case 4:   case 5:   case 6:
        case 7:   case 8:   case 9:
        case 10:  case 11:  case 12:
        case 13:  case 14:  case 15:
            /* 1..15 is not working with ARM */
            if (CY_USB_ENDP_DIR_OUT == endpDir) {
                /* Handle OUT endpoint. */
                if (setClear) {
                    pProtBase->PROT_EPO_CS1[endpNum] |=
                                                CY_USBSS_PROT_EPO_CS1_NRDY;
                } else {
                    pProtBase->PROT_EPO_CS1[endpNum] &=
                                              (~CY_USBSS_PROT_EPO_CS1_NRDY);
                }
            } else {
                /* Handle IN endpoint. */
                if (setClear) {
                    pProtBase->PROT_EPI_CS1[endpNum] |=
                                                CY_USBSS_PROT_EPI_CS1_NRDY;
                } else {
                    pProtBase->PROT_EPI_CS1[endpNum] &=
                                              (~CY_USBSS_PROT_EPI_CS1_NRDY);
                }
            }
            break;

        default:
            retCode = CY_USB_CAL_STATUS_BAD_PARAM;
            break;
        }

    return(retCode);
}   /* end of function  */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetClearNrdyAll (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USB32DEV_PROT_Type *pProtBase = NULL;

    if (pCalCtxt != NULL) {
        pProtBase = pCalCtxt->pProtBase;
        if (NULL == pProtBase) {
            return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
        }
    } else {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    if (setClear) {
        /* Clear SETUP_CLR_BUSY bit and set NRDYALL bit */
        pProtBase->PROT_CS &=
             (((~CY_USBSS_PROT_CS_SETUP_CLR_BUSY)) | (CY_USBSS_PROT_CS_NRDY_ALL));
    } else {
        /* clear  SETUP_CLR_BUSY and NRDY  both the bits */
        pProtBase->PROT_CS &=
                (~(CY_USBSS_PROT_CS_SETUP_CLR_BUSY | CY_USBSS_PROT_CS_NRDY_ALL));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */

const uint8_t UsbSSEpTypeMap[] = {
    3,  /* CTRL */
    0,  /* ISO */
    2,  /* BULK */
    1   /* INTR */
};

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpConfig (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                            cy_stc_usb_endp_config_t configParam)
{
    USB32DEV_PROT_Type *pProtBase;
    USB32DEV_EPM_Type *pEpmBase;
    cy_en_usb_endp_dir_t endpDir = configParam.endpDirection;
    cy_en_usb_endp_type_t endpType = configParam.endpType;
    uint32_t endpNum = configParam.endpNumber;
    uint32_t maxPktSize = configParam.maxPktSize;
    uint32_t maxBurst   = configParam.burstSize;

    /* based on endp direction below variable will be mapped to EPI or EPO. */
    uint32_t prot_ep_cs1;
    uint32_t prot_ep_cs2;
    uint32_t epm_ept;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;
    pEpmBase  = pCalCtxt->pEpmBase;

    if ((NULL == pProtBase) || (NULL == pEpmBase)) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /* check all possibility of wrong input parameter. */
    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpType >= CY_USB_ENDP_TYPE_INVALID) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {

        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if ((maxBurst > 16) ||
        ((maxBurst > 0x01) && (maxPktSize < 1024))) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /*
     * The burst size to be set in the register should be 1 less than
     * the actual burst size. burstSize 0 means 1 Pkt and 15 means 16Pkts.
     */
    if (maxBurst != 0) {
        maxBurst--;
    }

    /*
     * While applying any config to endpoint, these steps should be
     * followed:
     * 1. Flush endpint.
     * 2. RESET endpoint.
     * 3. Put all endpoint fields in default state.
     * 4. Configure the endpoint and make it valid based on config param.
     */
    if ((CY_USB_ENDP_DIR_OUT == endpDir) || (0 == endpNum)) {
        /* Flush and reset the endpoint if it was already enabled. */
        if ((pProtBase->PROT_EPO_CS1[endpNum] & USB32DEV_PROT_EPO_CS1_VALID_Msk) != 0) {
            Cy_USBSS_Cal_EndpReset(pCalCtxt, endpNum, CY_USB_ENDP_DIR_OUT);

            /* Restore PROT_EPO_CS2 to default value. */
            pProtBase->PROT_EPO_CS2[endpNum] = (
                    (pProtBase->PROT_EPO_CS2[endpNum] & CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPO_CS2) |
                    CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPO_CS2);
        }

        /* For OUT endpoints, make sure that PROT_EPO_CS1 is set to the expected value. */
        pProtBase->PROT_EPO_CS1[endpNum] = (
                (pProtBase->PROT_EPO_CS1[endpNum] & CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPO_CS1) |
                CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPO_CS1);

        /* Now start endpoint configuration. */
        prot_ep_cs1 = pProtBase->PROT_EPO_CS1[endpNum];
        prot_ep_cs2 = pProtBase->PROT_EPO_CS2[endpNum];

        /* clear require bits and then set with required value. */
        prot_ep_cs2 &= ~(USB32DEV_PROT_EPO_CS2_EPO_TYPE_Msk);
        prot_ep_cs2 |= (UsbSSEpTypeMap[endpType] & 0x03) << USB32DEV_PROT_EPO_CS2_EPO_TYPE_Pos;

        prot_ep_cs2 &= ~(USB32DEV_PROT_EPO_CS2_MAXBURST_Msk);
        prot_ep_cs2 |= (maxBurst << USB32DEV_PROT_EPO_CS2_MAXBURST_Pos);

        if ((CY_USB_ENDP_TYPE_BULK == endpType) && (0 < configParam.streamID)) {
            /* Map Stream #1 to the socket corresponding to the endpoint. */
            Cy_USBSS_Cal_EndpMapStream(pCalCtxt, endpNum, CY_USB_ENDP_DIR_OUT,
                    0x01, endpNum);

            prot_ep_cs1 |= USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk;
        } else {
            prot_ep_cs1 &= ~USB32DEV_PROT_EPO_CS1_STREAM_EN_Msk;
        }

        prot_ep_cs2 &= ~(USB32DEV_PROT_EPO_CS2_ISOINPKS_Msk);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            prot_ep_cs2 |= (configParam.isoPkts << USB32DEV_PROT_EPO_CS2_ISOINPKS_Pos);
        }

        if (configParam.valid) {
            /*
             * Update maxPkt size. Enable interrupt for the endpoint.
             * enable endpoint also.
             */
            epm_ept = (configParam.maxPktSize & USB32DEV_EPM_IEPM_ENDPOINT_PACKET_SIZE_Msk);

            /* Clear and enable relevant EPO interrupts. */
            pProtBase->PROT_EPO_INTR[endpNum] = 0x3FFFFUL;
            pProtBase->PROT_EPO_INTR_MASK[endpNum] = (
                    USB32DEV_PROT_EPO_INTR_RETRY_Msk |
                    USB32DEV_PROT_EPO_INTR_OOSERR_Msk
                    );
            pProtBase->PROT_EP_INTR_MASK |= (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));

            /* Make the endpoint valid. */
            prot_ep_cs1 |= USB32DEV_PROT_EPO_CS1_VALID_Msk;
        } else {
            /* Make endpoint invalid. */
            prot_ep_cs1 &= ~USB32DEV_PROT_EPO_CS1_VALID_Msk;
            epm_ept = 0x0400UL;

            /* Disable and clear all EPO interrupts. */
            pProtBase->PROT_EP_INTR_MASK &= (~(0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER)));
            pProtBase->PROT_EPO_INTR_MASK[endpNum] = 0;
            pProtBase->PROT_EPO_INTR[endpNum] = 0x3FFFFUL;
        }

        /* Update the endpoint configuration registers. */
        pProtBase->PROT_EPO_CS2[endpNum] = prot_ep_cs2;
        pEpmBase->IEPM_ENDPOINT[endpNum] = epm_ept;
        pProtBase->PROT_EPO_CS1[endpNum] = prot_ep_cs1;

        if (configParam.valid) {
            /* Make sure sequence number is cleared for the endpoint. */
            Cy_USBSS_Cal_SetSeqNum(pCalCtxt, endpNum, endpDir, 0);
        }

    }   /* configuration related to OUT endpoint ends here. */

    if ((CY_USB_ENDP_DIR_IN == endpDir) || (0 == endpNum)) {
        /* Flush and reset the endpoint if it was already enabled. */
        if ((pProtBase->PROT_EPI_CS1[endpNum] & USB32DEV_PROT_EPI_CS1_VALID_Msk) != 0) {
            Cy_USBSS_Cal_EndpReset(pCalCtxt, endpNum, CY_USB_ENDP_DIR_IN);

            /*  Make sure CS1 and CS2 at default state. */
            pProtBase->PROT_EPI_CS2[endpNum] = (
                    (pProtBase->PROT_EPI_CS2[endpNum] & CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPI_CS2) |
                    CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPI_CS2);
            pProtBase->PROT_EPI_CS1[endpNum] = (
                    (pProtBase->PROT_EPI_CS1[endpNum] & CY_USBSS_ENDP_DEFAULT_STATE_ZERO_EPI_CS1) |
                    CY_USBSS_ENDP_DEFAULT_STATE_NONZERO_EPI_CS1);
        }

        prot_ep_cs1 = pProtBase->PROT_EPI_CS1[endpNum];
        prot_ep_cs2 = pProtBase->PROT_EPI_CS2[endpNum];

        /* clear require bits and then set with required value. */
        prot_ep_cs2 &= ~(USB32DEV_PROT_EPI_CS2_EPI_TYPE_Msk);
        prot_ep_cs2 |= (UsbSSEpTypeMap[endpType] & 0x03) << USB32DEV_PROT_EPI_CS2_EPI_TYPE_Pos;

        /* burstSize 0 means 1 Pkt and 15 means 16Pkts. */
        prot_ep_cs2 &= ~(USB32DEV_PROT_EPI_CS2_MAXBURST_Msk);
        prot_ep_cs2 |= (maxBurst << USB32DEV_PROT_EPI_CS2_MAXBURST_Pos);

        if ((CY_USB_ENDP_TYPE_BULK == endpType) && (0 < configParam.streamID)) {
            /* Map Stream #1 to the socket corresponding to the endpoint. */
            Cy_USBSS_Cal_EndpMapStream(pCalCtxt, endpNum, CY_USB_ENDP_DIR_IN,
                    0x01, endpNum);

            prot_ep_cs1 |= USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk;
        } else {
            prot_ep_cs1 &= ~USB32DEV_PROT_EPI_CS1_STREAM_EN_Msk;
        }

        prot_ep_cs2 &= ~(USB32DEV_PROT_EPI_CS2_ISOINPKS_Msk);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            prot_ep_cs2 |= (configParam.isoPkts << USB32DEV_PROT_EPI_CS2_ISOINPKS_Pos);

            /* Set sequence number update rule for ISOCHRONOUS egress endpoints. */
            prot_ep_cs2 &= ~(USB32DEV_PROT_EPI_CS2_BINTERVAL_Msk);
            switch(configParam.interval) {
                case 0:
                case 1:
                    /* No change required. BINTERVAL should be set to 0. */
                    break;
                case 2:
                    /* Sequence number should be cleared when two ITPs have been received. */
                    prot_ep_cs2 |= (1UL << USB32DEV_PROT_EPI_CS2_BINTERVAL_Pos);
                    break;
                case 3:
                    /* Sequence number should be cleared when four ITPs have been received. */
                    prot_ep_cs2 |= (2UL << USB32DEV_PROT_EPI_CS2_BINTERVAL_Pos);
                    break;
                default:
                    /* Sequence number should be cleared when eight ITPs have been received. */
                    prot_ep_cs2 |= (3UL << USB32DEV_PROT_EPI_CS2_BINTERVAL_Pos);
                    break;
            }
        }

        if (configParam.valid) {
            epm_ept = (configParam.maxPktSize & USB32DEV_EPM_EEPM_ENDPOINT_PACKET_SIZE_Msk);

            /* Clear and enable relevant EPI interrupts. */
            pProtBase->PROT_EPI_INTR[endpNum] = 0x3FFFFUL;
            pProtBase->PROT_EPI_INTR_MASK[endpNum] = (
                    USB32DEV_PROT_EPI_INTR_RETRY_Msk |
                    USB32DEV_PROT_EPI_INTR_OOSERR_Msk);
            pProtBase->PROT_EP_INTR_MASK |= (0x01 << endpNum);

            /* Make the endpoint valid. */
            prot_ep_cs1 |= USB32DEV_PROT_EPI_CS1_VALID_Msk;
        } else {
            prot_ep_cs1 &= ~USB32DEV_PROT_EPI_CS1_VALID_Msk;
            epm_ept = 0x0400UL;

            /* Disable and clear all EP specific interrupts. */
            pProtBase->PROT_EP_INTR_MASK &= (~(0x01 << endpNum));
            pProtBase->PROT_EPI_INTR_MASK[endpNum] = 0;
            pProtBase->PROT_EPI_INTR[endpNum] = 0x3FFFFUL;
        }

        /* Update the endpoint configuration registers. */
        pProtBase->PROT_EPI_CS2[endpNum] = prot_ep_cs2;
        pEpmBase->EEPM_ENDPOINT[endpNum] = epm_ept;
        pProtBase->PROT_EPI_CS1[endpNum] = prot_ep_cs1;

        if (configParam.valid) {
            /* Make sure sequence number is cleared for the endpoint. */
            Cy_USBSS_Cal_SetSeqNum(pCalCtxt, endpNum, endpDir, 0);
        }
    }

    if (endpNum == 0) {
        pEpmBase->EEPM_ENDPOINT[0] = 512;
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


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
* \param endpNum
* Endpoint number.
*
* \param pktsPerBuffer
* packats per buffer.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetPktsPerBuffer (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                   uint32_t endpNum,
                                   uint8_t pktsPerBuffer)
{
    USB32DEV_EPM_Type *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pEpmBase = pCalCtxt->pEpmBase;
    if (NULL == pEpmBase) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    /*
     * Operation only permitted for non-EP0 OUT endpoints and number of
     * packets should be <= 31.
     */
    if ((endpNum == 0) || (endpNum >= 16UL) || (pktsPerBuffer > 31U)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pEpmBase->IEPM_ENDPOINT[endpNum] &= ~(USB32DEV_EPM_IEPM_ENDPOINT_ODD_MAX_NUM_PKTS_Msk |
            USB32DEV_EPM_IEPM_ENDPOINT_ODD_MAX_PKT_SIZE_EN_Msk);
    if (pktsPerBuffer != 0) {
        pEpmBase->IEPM_ENDPOINT[endpNum] |= (USB32DEV_EPM_IEPM_ENDPOINT_ODD_MAX_PKT_SIZE_EN_Msk |
            (pktsPerBuffer << USB32DEV_EPM_IEPM_ENDPOINT_ODD_MAX_NUM_PKTS_Pos));
    }

    return (CY_USB_CAL_STATUS_SUCCESS);
}

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
* \param endpNum
* Endpoint number.
*
* \param endpDir
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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpMapStream (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir, uint16_t streamId,
                            uint32_t socketNum)
{
    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pProtBase = pCalCtxt->pProtBase;
    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if (
            (0 == endpNum) || (CY_USB_MAX_ENDP_NUMBER <= endpNum) ||
            (CY_USB_ENDP_DIR_INVALID <= endpDir) || (0 == streamId) ||
            (0 == socketNum) || (CY_USB_MAX_ENDP_NUMBER <= socketNum)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if (CY_USB_ENDP_DIR_IN == endpDir) {
        /* If socket is already mapped to a stream, unmap it first. */
        if ((pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPI_MAPPED_STREAM_ENABLE_Msk) != 0) {
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] &= ~(USB32DEV_PROT_EPI_MAPPED_STREAM_ENABLE_Msk);
            while (!(pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAPPED_Msk));
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] &= ~USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum]  = 0;
        }

        /* Set the new endpoint and stream number and enable. */
        pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] = (
                (endpNum << USB32DEV_PROT_EPI_MAPPED_STREAM_EP_NUMBER_Pos) |
                (streamId << USB32DEV_PROT_EPI_MAPPED_STREAM_STREAM_ID_Pos));
        pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPI_MAPPED_STREAM_ENABLE_Msk;
    } else {
        /* If socket is already mapped to a stream, unmap it first. */
        if ((pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPO_MAPPED_STREAM_ENABLE_Msk) != 0) {
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] &= ~(USB32DEV_PROT_EPO_MAPPED_STREAM_ENABLE_Msk);
            while (!(pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAPPED_Msk));
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] &= ~USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum]  = 0;
        }

        /* Set the new endpoint and stream number and enable. */
        pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] = (
                (endpNum << USB32DEV_PROT_EPO_MAPPED_STREAM_EP_NUMBER_Pos) |
                (streamId << USB32DEV_PROT_EPO_MAPPED_STREAM_STREAM_ID_Pos));
        pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPO_MAPPED_STREAM_ENABLE_Msk;
    }

    return (CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */

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
Cy_USBSS_Cal_EndpUnmapStream (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir,
                            uint32_t socketNum)
{
    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pProtBase = pCalCtxt->pProtBase;
    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if (
            (0 == endpNum) || (CY_USB_MAX_ENDP_NUMBER <= endpNum) ||
            (CY_USB_ENDP_DIR_INVALID <= endpDir) ||
            (0 == socketNum) || (CY_USB_MAX_ENDP_NUMBER <= socketNum)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if (CY_USB_ENDP_DIR_IN == endpDir) {
        /* If socket is already mapped to a stream, unmap it first. */
        if ((pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPI_MAPPED_STREAM_ENABLE_Msk) != 0) {
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] &= ~(USB32DEV_PROT_EPI_MAPPED_STREAM_ENABLE_Msk);
            while (!(pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAPPED_Msk));
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum] &= ~USB32DEV_PROT_EPI_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPI_MAPPED_STREAM[socketNum]  = 0;
        }
    } else {
        /* If socket is already mapped to a stream, unmap it first. */
        if ((pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPO_MAPPED_STREAM_ENABLE_Msk) != 0) {
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] |= USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] &= ~(USB32DEV_PROT_EPO_MAPPED_STREAM_ENABLE_Msk);
            while (!(pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] & USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAPPED_Msk));
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum] &= ~USB32DEV_PROT_EPO_MAPPED_STREAM_UNMAP_Msk;
            pProtBase->PROT_EPO_MAPPED_STREAM[socketNum]  = 0;
        }
    }

    return (CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


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
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableEndp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                         cy_en_usb_endp_dir_t endpDir,  bool enable)
{
    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    /*
     * Enable: set valid bit and enable interrupt for this endpoint.
     * Disable: clear valid bit and disable interrupt for this endpoint.
     */
    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        if (enable) {
            pProtBase->PROT_EPO_CS1[endpNum] |= CY_USBSS_PROT_EPO_CS1_VALID;
            pProtBase->PROT_EP_INTR_MASK |= (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
        } else {
            pProtBase->PROT_EPO_CS1[endpNum] &= (~(CY_USBSS_PROT_EPO_CS1_VALID));
            pProtBase->PROT_EP_INTR_MASK &= (~(0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER)));
        }
    } else {
        if (enable) {
            pProtBase->PROT_EPI_CS1[endpNum] |= CY_USBSS_PROT_EPI_CS1_VALID;
            pProtBase->PROT_EP_INTR_MASK |= (0x01 << (endpNum));
        } else {
            pProtBase->PROT_EPI_CS1[endpNum] &= (~(CY_USBSS_PROT_EPI_CS1_VALID));
            pProtBase->PROT_EP_INTR_MASK &= (~(0x01 << (endpNum)));
        }
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBSS_Cal_EnableEndp () */


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
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param enable
* true for enable burst mode and false for disable burst mode.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetEpBurstMode (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                             bool enable)
{
    USB32DEV_EPM_Type *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pEpmBase = pCalCtxt->pEpmBase;
    if (NULL == pEpmBase) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) || (endpNum == CY_USB_ENDP_0)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /*
     * For OUT endpoints other than EP0, we need to set the MULT_EN bit in the IEPM_MULT register.
     * Nothing to do for IN endpoints.
     * Feature is not supported on EP0.
     */
    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        if (enable) {
            pEpmBase->IEPM_MULT |= (1UL << (endpNum - 1));
        } else {
            pEpmBase->IEPM_MULT &= ~(1UL << (endpNum - 1));
        }
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBSS_Cal_SetEpBurstMode () */


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
* \param endpNum
* Endpoint number.
*
* \param retryBufOffset
* Retry buffer offset.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetEndpRetryOffset (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 uint32_t endpNum,
                                 uint16_t retryBufOffset)
{
    USB32DEV_EPM_Type *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pEpmBase = pCalCtxt->pEpmBase;
    if (NULL == pEpmBase) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) || (retryBufOffset >= 16384U)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pEpmBase->EEPM_RETRY_OFFSET[endpNum] = (uint32_t)retryBufOffset;
    return(CY_USB_CAL_STATUS_SUCCESS);
} /* End of function Cy_USBSS_Cal_SetEndpRetryOffset */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_UpdateXferCount (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum,
                              cy_en_usb_endp_dir_t endpDir,
                              uint32_t xferCount)
{
    USB32DEV_PROT_Type *pProtBase;
    USB32DEV_EPM_Type  *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }
    pEpmBase =  pCalCtxt->pEpmBase;
    (void)pEpmBase;

    /* TBD: Update required registers. */
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableStatusCtrl (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool enable)
{

    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }
    if (enable) {
        pProtBase->PROT_CS |= (USB32DEV_PROT_CS_EN_STATUS_CONTROL_Msk);
    } else {
        pProtBase->PROT_CS &= (~USB32DEV_PROT_CS_EN_STATUS_CONTROL_Msk);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ClearStatusClrBusy (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

    USB32DEV_PROT_Type *pProtBase;
    volatile uint32_t protCs;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }
    /* There are certain bits which is RW1C take of those bits. */
    protCs = pProtBase->PROT_CS;
    protCs &= (~CY_USBSS_PROT_CS_SETUP_CLR_BUSY);
    protCs |= (USB32DEV_PROT_CS_STATUS_CLR_BUSY_Msk);
    pProtBase->PROT_CS = protCs;
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SendAckSetupDataStatusStage (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

    USB32DEV_PROT_Type *pProtBase;
    volatile uint32_t protCs;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }
    protCs = pProtBase->PROT_CS;
    /*
     * While clearing SETUP_CLR_BUSY, don't clear SETUP_CLR_BUSY bit.
     * Writting '0' in STATUS_CLR_BUSY bit will not have any impact.
     */
    protCs &= (~USB32DEV_PROT_CS_STATUS_CLR_BUSY_Msk);
    protCs |= CY_USBSS_PROT_CS_SETUP_CLR_BUSY;
    pProtBase->PROT_CS = protCs;

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushEPM (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool force)
{

    USB32DEV_EPM_Type  *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pEpmBase = pCalCtxt->pEpmBase;

    if (NULL == pEpmBase) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    /* Flush EPM only if pointers are non-zero or if force parameter is set. */
    if ((force) || (pEpmBase->IEPM_CS != 0)) {
        do {
            pEpmBase->IEPM_CS |= USB32DEV_EPM_IEPM_CS_EPM_FLUSH_Msk;
            Cy_SysLib_DelayUs(1);
            pEpmBase->IEPM_CS &= ~USB32DEV_EPM_IEPM_CS_EPM_FLUSH_Msk;
            DBG_SSCAL_INFO("EPM flush\r\n");
        } while (pEpmBase->IEPM_CS != 0);
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushEndpSocket (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    USB32DEV_EPM_Type *pEpmBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pEpmBase = pCalCtxt->pEpmBase;
    if (NULL == pEpmBase) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /* Function is only supported for EP0. For other endpoints, Cy_USBSS_Cal_EndpReset should be used. */
    if (endpNum == 0x00) {
        pEpmBase->IEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pEpmBase->EEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        Cy_SysLib_DelayUs(1);
        pEpmBase->IEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pEpmBase->EEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
    } else {
        return (CY_USB_CAL_STATUS_FAILURE);
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushAllEndpSocket (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_EPM_Type  *pEpmBase;
    USB32DEV_MAIN_Type *pMainBase;
    uint32_t endpNum;
    uint32_t intState;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pEpmBase  = pCalCtxt->pEpmBase;
    pMainBase = pCalCtxt->pMainBase;
    if ((NULL == pEpmBase) || (NULL == pMainBase)) {
        return(CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL);
    }

    intState = Cy_SysLib_EnterCriticalSection();

    /*
     * Not including endp0 from this. Endpoint 0 socket should be flushed
     * with Cy_USBSS_Cal_FlushEndpSocket. This arrangement to avoid any
     * timing problem with endpoint0.
     */
    for (endpNum = 0x01; endpNum < CY_USB_MAX_ENDP_NUMBER; endpNum++) {
        pEpmBase->IEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pEpmBase->EEPM_ENDPOINT[endpNum] |= USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
    }

    Cy_SysLib_DelayUs(1);

    for (endpNum = 0x01; endpNum < CY_USB_MAX_ENDP_NUMBER; endpNum++) {
        pEpmBase->IEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_IEPM_ENDPOINT_SOCKET_FLUSH_Msk;
        pEpmBase->EEPM_ENDPOINT[endpNum] &= ~USB32DEV_EPM_EEPM_ENDPOINT_SOCKET_FLUSH_Msk;
    }

    Cy_SysLib_ExitCriticalSection(intState);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableMainIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    USB32DEV_MAIN_Type *pMainBase;
    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pMainBase = pCalCtxt->pMainBase;
    if (NULL == pMainBase) {
        return(CY_USB_CAL_STATUS_CAL_MAIN_BASE_NULL);
    }

    if (EnableDisable) {
        /* Before enable interrupts, make sure all interrupts are cleared. */
        pMainBase->INTR = 0xFFFFFFFF;
        /* Enable required main interrupts  */
        pMainBase->INTR_MASK |= (USB32DEV_MAIN_INTR_LINK_Msk |
                                     USB32DEV_MAIN_INTR_PROT_Msk |
                                     USB32DEV_MAIN_INTR_PROT_EP_Msk |
                                     USB32DEV_MAIN_INTR_EPM_URUN_Msk |
                                     USB32DEV_MAIN_INTR_PHY0_Msk |
                                     USB32DEV_MAIN_INTR_PHY1_Msk);
    } else {
        /* Disable main interrupts  */
        pMainBase->INTR_MASK = 0x00;
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableLinkIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    USB32DEV_LNK_Type  *pLinkBase;
    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pLinkBase = pCalCtxt->pLinkBase;
    if (NULL == pLinkBase) {
        return(CY_USB_CAL_STATUS_CAL_LNK_BASE_NULL);
    }

    if (EnableDisable) {
        /* Before enable interrupts, make sure all interrupts are cleared. */
        pLinkBase->LNK_INTR       = 0xFFFFFFFF;
        pLinkBase->LNK_INTR_MASK  |= (USB32DEV_LNK_INTR_LGO_U3_Msk |
                                      USB32DEV_LNK_INTR_LTSSM_CONNECT_Msk |
                                      USB32DEV_LNK_INTR_LPMA_Msk |
                                      USB32DEV_LNK_INTR_LTSSM_DISCONNECT_Msk |
                                      USB32DEV_LNK_INTR_LTSSM_RESET_Msk |
                                      USB32DEV_LNK_INTR_DATA_RATE_CHANGE_Msk |
                                      USB32DEV_LNK_INTR_LTSSM_U3_ENTRY_Msk |
                                      USB32DEV_LNK_INTR_LTSSM_STATE_CHG_Msk);
    } else {
        /* Disable all link interrupt */
        pLinkBase->LNK_INTR_MASK  = 0x00;
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */

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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableProtIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    USB32DEV_PROT_Type *pProtBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }

    pProtBase = pCalCtxt->pProtBase;
    if (NULL == pProtBase) {
        return(CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL);
    }

    if (EnableDisable) {
        /* Before enable interrupts, make sure all interrupts are cleared. */
        pProtBase->PROT_INTR      = 0xFFFFFFFF;
        pProtBase->PROT_INTR_MASK =
                                  (USB32DEV_PROT_INTR_STATUS_STAGE_Msk |
                                   USB32DEV_PROT_INTR_SUTOK_EV_Msk |
                                   USB32DEV_PROT_INTR_TIMEOUT_PORT_CAP_EV_Msk |
                                   USB32DEV_PROT_INTR_TIMEOUT_PORT_CFG_EV_Msk |
                                   USB32DEV_PROT_INTR_LMP_RCV_EV_Msk |
                                   USB32DEV_PROT_INTR_LMP_PORT_CAP_EV_Msk |
                                   USB32DEV_PROT_INTR_LMP_PORT_CFG_EV_Msk |
                                   USB32DEV_PROT_INTR_SET_ADDR_Msk);

    } else {
        /* Disable all prot interrupt and clear. */
        pProtBase->PROT_INTR_MASK  = 0x00;
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
bool
Cy_USBSS_Cal_EndpIsNakNrdySet (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    USB32DEV_PROT_Type *pProtBase;
    bool endpNrdyed = false;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpNrdyed);
    }

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pProtBase)) {
        return (endpNrdyed);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        if ((pProtBase->PROT_EPO_CS1[endpNum] & CY_USBSS_PROT_EPO_CS1_NRDY) ||
            (pProtBase->PROT_CS & CY_USBSS_PROT_CS_NRDY_ALL)) {
            endpNrdyed = true;
        }    /* end of if NRDY-ALL */
    } else {
        if ((pProtBase->PROT_EPI_CS1[endpNum] & CY_USBSS_PROT_EPI_CS1_NRDY) ||
                (pProtBase->PROT_CS & CY_USBSS_PROT_CS_NRDY_ALL)) {
                endpNrdyed = true;
        }   /* endof if NRDY-ALL */
    }   /* end of direction */
    return endpNrdyed;
}   /* End of function() */


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
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* Whether endpoint is currently STALLed. True means STALLed state.
*
*******************************************************************************/
bool
Cy_USBSS_Cal_EndpIsStallSet (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    USB32DEV_PROT_Type *pProtBase;
    bool endpStalled = false;

   if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpStalled);
    }

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pProtBase)) {
        return (endpStalled);
    }
    pProtBase = pCalCtxt->pProtBase;

    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        if (pProtBase->PROT_EPO_CS1[endpNum] & USB32DEV_PROT_EPO_CS1_STALL_Msk) {
                endpStalled = true;
        }
    } else {
        if (pProtBase->PROT_EPI_CS1[endpNum] & USB32DEV_PROT_EPI_CS1_STALL_Msk) {
                endpStalled = true;
        }
    }
    return endpStalled;
}   /* End of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableSsDevice (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool enable)
{
    USB32DEV_MAIN_Type *pMainBase;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pMainBase = pCalCtxt->pMainBase;
    if (NULL == pMainBase) {
        return(CY_USB_CAL_STATUS_CAL_MAIN_BASE_NULL);
    }
    pMainBase = pCalCtxt->pMainBase;
    if (enable) {
        pMainBase->CTRL |= USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;
    } else {
        pMainBase->CTRL &= (~USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


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
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_DisConnect (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type  *USB32DEV_MAIN = &base->USB32DEV_MAIN;
    USB32DEV_LNK_Type  *pLinkBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs, *pPhyRegsAlt;
    uint32_t intstate;
    uint8_t  phyCount;

    if (NULL == pCalCtxt) {
        return(CY_USB_CAL_STATUS_CAL_CTXT_NULL);
    }
    pLinkBase = pCalCtxt->pLinkBase;
    if (NULL == pLinkBase) {
        return(CY_USB_CAL_STATUS_CAL_LNK_BASE_NULL);
    }

    DBG_SSCAL_TRACE("CalDisconnect: susp_flag=%d IPCTRL=%x\r\n", pCalCtxt->linkSuspended, USB32DEV_MAIN->CTRL);

    /* Make sure clocks to the USB block are enabled. */
    if (pCalCtxt->linkSuspended) {
        pPhyRegs = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP;
        pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;

        if (pCalCtxt->dualLaneEnabled) {
            pPhyRegsAlt = &base->USB32DEV_PHYSS.USB40PHY[!pCalCtxt->activePhyIdx].USB40PHY_TOP;
            pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
        }

        Cy_SysLib_DelayUs(1);
        USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;
        pCalCtxt->linkSuspended = false;
        pCalCtxt->uxExitActive = false;
    }

    phyCount = pCalCtxt->numPhyInitialized;
    DBG_SSCAL_TRACE("SSCal_Disconnect: numPhyInitialized = %d\r\n", pCalCtxt->numPhyInitialized);

    intstate = Cy_SysLib_EnterCriticalSection();

    /* Force LTSSM into SS.Disabled state. */
    pLinkBase->LNK_LTSSM_STATE =
            (CY_USBSS_LNK_STATE_SSDISABLED << USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_VALUE_Pos) |
             USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk;

    /* Disable the SSPHY. */
    Cy_USBSS_Phy_RxDeinit(pCalCtxt, pCalCtxt->activePhyIdx);
    Cy_USBSS_Phy_PllDeinit(pCalCtxt,pCalCtxt->activePhyIdx);
    Cy_USBSS_Phy_TxDeinit(pCalCtxt, pCalCtxt->activePhyIdx);

    pCalCtxt->numPhyInitialized--;
    if (pCalCtxt->numPhyInitialized != 0)
    {
        Cy_USBSS_Phy_RxDeinit(pCalCtxt, !(pCalCtxt->activePhyIdx));

        /* Do the PHY1 writes getting skipped in PllDeinit if activePhyidx is 0. */
        if (pCalCtxt->activePhyIdx == 0) {
            base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
                ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
        }
        Cy_USBSS_Phy_TxDeinit(pCalCtxt, !(pCalCtxt->activePhyIdx));
        pCalCtxt->numPhyInitialized = 0;
    }

    /* Disable USBSS RX terminations */
    pLinkBase->LNK_PHY_CONF &= ~USB32DEV_LNK_PHY_CONF_RX_TERMINATION_ENABLE_Msk;

    /* Before disabling PHY PCLK, make sure HBWSS is not using the clock. */
    Cy_USBSS_Cal_UpdateDmaClkFreq(pCalCtxt, false);

    /* Disable PHYSS */
    base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);
    base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    /* Free the link state machine. */
    pLinkBase->LNK_LTSSM_STATE &= ~USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk;

    /* Disable the clock for USB3.2 function */
    USB32DEV_MAIN->CTRL &= ~(USB32DEV_MAIN_CTRL_CLK_EN_Msk |
            USB32DEV_MAIN_CTRL_CONFIG_LANE_Msk |
            USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk);

    DBG_SSCAL_INFO("SSCal_Disconnect: Disabled %d PHYs\r\n", phyCount);
    Cy_SysLib_ExitCriticalSection(intstate);

    /* Clear link suspended (in U3) flag. */
    pCalCtxt->linkSuspended = false;
    pCalCtxt->activePhyIdx  = 0;
    pCalCtxt->connectRcvd   = false;
    pCalCtxt->compExitDone  = false;
    pCalCtxt->compExitTS    = 0;
    pCalCtxt->stopClkOnEpResetEnable = false;

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function () */


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
/*
 * ptmControl must be set to 1 for receiving LDM RX interrupt and further
 * proceeding with PTM engine logic.
 */
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_PTMConfig (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool ptmControl)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_PROT_Type  *USB32DEV_PROT = &base->USB32DEV_PROT;

    if (ptmControl == true) {
        /* Set the LDM Enable bit in the PTM STATUS response */
        pCalCtxt->ptmStatus = (uint32_t)CY_USBSS_PTM_STAT_LDM_ENABLE_Msk;

        /* Make sure PTM state machine starts from disabled state. */
        USB32DEV_PROT->PROT_PTM_CONFIG = 0;

        /* Enable interrupt on ITP event. */
        USB32DEV_PROT->PROT_INTR_MASK |= USB32DEV_PROT_INTR_ITP_EV_Msk;
    } else {
        /* Clear the LDM Enable bit in the PTM STATUS response */
        pCalCtxt->ptmStatus = (uint32_t)(0x00UL);

        /* Reset enter PTM logic. */
        USB32DEV_PROT->PROT_PTM_CONFIG = 0;

        /* Disable LDM_RX and ITP interrupts. */
        USB32DEV_PROT->PROT_INTR_MASK &= ~(USB32DEV_PROT_INTR_ITP_EV_Msk |
                USB32DEV_PROT_INTR_LDM_RX_Msk);
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */

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
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_EnterLinkCompliance (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_LNK_Type *pLinkBase;

    if ((pCalCtxt != 0) && (pCalCtxt->pLinkBase != 0)) {
        DBG_SSCAL_INFO("ComplianceEntry\r\n");
        pLinkBase = pCalCtxt->pLinkBase;

        /* Update variables to indicate compliance state. */
        pCalCtxt->inCompState     = true;
        pCalCtxt->currCompPattern = 0;
        pCalCtxt->gen1SpeedForced = false;

        /* Clear and enable interrupt for Ping.LFPS. */
        pLinkBase->LNK_INTR       = USB32DEV_LNK_INTR_RX_PING_LFPS_Msk;
        pLinkBase->LNK_INTR_MASK |= USB32DEV_LNK_INTR_RX_PING_LFPS_Msk;
    }
}

static void SSPhy_Update_TxSettings (USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs,
                                     uint32_t *pTxSettings)
{
    uint32_t tmp;

    pPhyRegs->TX_AFE_CFG_SFT_W_0 = pTxSettings[0];
    pPhyRegs->TX_AFE_CFG_SFT_W_1 = pTxSettings[1];
    pPhyRegs->TX_AFE_CFG_SFT_W_2 = pTxSettings[2];
    pPhyRegs->TX_AFE_CFG_SFT_W_3 = pTxSettings[3];
    pPhyRegs->TX_AFE_CFG_SFT_W_4 = pTxSettings[4];
    pPhyRegs->TX_AFE_CFG_SFT_W_5 = pTxSettings[5];
    pPhyRegs->TX_AFE_CFG_SFT_W_6 = pTxSettings[6];
    pPhyRegs->TX_AFE_CFG_SFT_W_7 = pTxSettings[7];

    /* Program the write done bit to start shifting the data. */
    pPhyRegs->TX_AFE_CFG = USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CFG_REG_TX_CFG_WRITE_DONE_Msk;
    Cy_SysLib_DelayUs(5);

    /* Wait until shift register update is complete. */
    do {
        tmp = pPhyRegs->TX_AFE_CFG;
    } while ((tmp & USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CFG_REG_TX_CFG_WRITE_DONE_Msk) != 0);
}

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
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_NextCompliancePattern (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_LNK_Type *pLinkBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegs = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyRegsAlt =
        &base->USB32DEV_PHYSS.USB40PHY[!pCalCtxt->activePhyIdx].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegs = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_RX_Type *pRxRegsAlt = &base->USB32DEV_PHYSS.USB40PHY[!pCalCtxt->activePhyIdx].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *pPllRegs = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;

    uint32_t txSettings[8];

    if ((pCalCtxt != 0) && (pCalCtxt->pLinkBase != 0) && (pCalCtxt->inCompState)) {
        pLinkBase = pCalCtxt->pLinkBase;
        DBG_SSCAL_INFO("PHY TOP_CTRL:%x gen2En:%d\r\n", pPhyRegs->TOP_CTRL_0, pCalCtxt->gen2Enabled);

        /* Pick the next compliance pattern value and apply it. */
        pCalCtxt->currCompPattern++;
        if (pCalCtxt->gen2Enabled) {
            if (pCalCtxt->currCompPattern > 16) {
                pCalCtxt->currCompPattern = 0;

                /* When switching from CP16 (Gen2) to CP0 (Gen1), force rate change without relying on interrupt. */
                if (pLinkBase->LNK_COMPLIANCE_PATTERN_OBSERVE == 0x13) {
                    pCalCtxt->gen2Enabled = false;

                    /* Assert serializer and PLL reset. */
                    pPhyRegs->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    pPhyRegs->TOP_CTRL_0    &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                        pPhyRegsAlt->TOP_CTRL_0    &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    }

                    /* Rate change in PLL. */
                    pPllRegs->PLL_DIVP_CFG = 0x00000061UL;
                    Cy_SysLib_DelayUs(1);
                    pPllRegs->PLL_DIVP_CFG = 0x000000E1UL;

                    /* Update VCO CP gain and rate change in receivers. */
                    pRxRegs->RX_CP_CFG = (
                            (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                            (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                    pRxRegs->RX_GNRL_CFG = (
                            (pRxRegs->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                            (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk);

                    /* Change rate to Gen1 in TOP_CTRL_0 register. */
                    pPhyRegs->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk;

                    /* De-assert PLL and serializer reset. */
                    pPhyRegs->TOP_CTRL_0    |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    pPhyRegs->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_CP_CFG = (
                                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                                (6UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                        pRxRegsAlt->RX_GNRL_CFG = (
                                (pRxRegsAlt->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                                (0x01UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk);
                        pPhyRegsAlt->TOP_CTRL_0    &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Msk;
                        pPhyRegsAlt->TOP_CTRL_0    |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                        pPhyRegsAlt->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    }

                    /* Assert RATE_CHANGE_DONE bit. */
                    Cy_SysLib_DelayUs(10);
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;
                    }
                    pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;

                    /*
                     * Need to modify PING and RESET LFPS detect durations as link may not realise
                     * that DUT has switched back to Gen1 operation.
                     */
                    pLinkBase->LNK_LFPS_RX_RESET_GEN2 = pLinkBase->LNK_LFPS_RX_RESET;
                    pLinkBase->LNK_LFPS_RX_PING_GEN2  = pLinkBase->LNK_LFPS_RX_PING;

                    pCalCtxt->gen1SpeedForced = true;
                }
            }
        } else {
            if (pCalCtxt->currCompPattern > 8) {
                pCalCtxt->currCompPattern = 0;

                /* When switching from CP8 (Gen1) to CP9 (Gen2), force rate change without relying on interrupt. */
                if (pLinkBase->LNK_COMPLIANCE_PATTERN_OBSERVE == 0x12) {
                    pCalCtxt->gen2Enabled = true;

                    /* Assert serializer and PLL reset. */
                    pPhyRegs->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    pPhyRegs->TOP_CTRL_0    &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TX_AFE_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                        pPhyRegsAlt->TOP_CTRL_0    &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    }

                    /* Rate change in PLL and receiver. */
                    pPllRegs->PLL_DIVP_CFG = 0x00000060UL;
                    Cy_SysLib_DelayUs(1);
                    pPllRegs->PLL_DIVP_CFG = 0x000000E0UL;

                    /* Update VCP CP gain and rate settings in the receiver. */
                    pRxRegs->RX_CP_CFG = (
                            (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                            (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                    pRxRegs->RX_GNRL_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk;
                    pRxRegs->RX_GNRL_CFG = (
                            (pRxRegs->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                            (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos));

                    /* Set rate as Gen2 in TOP_CTRL_0 register. */
                    pPhyRegs->TOP_CTRL_0 |= (0x01U << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos);

                    /* De-assert PLL and serializer reset. */
                    pPhyRegs->TOP_CTRL_0    |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                    pPhyRegs->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;

                    if (pCalCtxt->dualLaneEnabled) {
                        pRxRegsAlt->RX_CP_CFG = (
                                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPUP_ADJ_Pos) |
                                (2UL << USB32DEV_PHYSS_USB40PHY_RX_CP_CFG_IPDN_ADJ_Pos));
                        pRxRegsAlt->RX_GNRL_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DATA_RATE_Msk;
                        pRxRegsAlt->RX_GNRL_CFG = (
                                (pRxRegsAlt->RX_GNRL_CFG & ~USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Msk) |
                                (0x02UL << USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_BUS_BIT_MODE_Pos) |
                                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_SUMODD_PDB_Msk);

                        pPhyRegsAlt->TOP_CTRL_0    |= (0x01U << USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_Pos);
                        pPhyRegsAlt->TOP_CTRL_0    |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
                        pPhyRegsAlt->TX_AFE_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk;
                    }

                    /* Assert RATE_CHANGE_DONE bit. */
                    Cy_SysLib_DelayUs(10);
                    if (pCalCtxt->dualLaneEnabled) {
                        pPhyRegsAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;
                    }
                    pPhyRegs->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_RATE_CHANGE_DONE_Msk;

                    /* Set Gen2 LFPS durations to default values. */
                    pLinkBase->LNK_LFPS_RX_RESET_GEN2 = 25000000UL;
                    pLinkBase->LNK_LFPS_RX_PING_GEN2  = 0x004B000DUL;

                    pCalCtxt->currCompPattern = 9;
                }
            }
        }

        switch (pCalCtxt->currCompPattern) {
            case 5:
            case 7:
                /* Gen1 with de-emphasis enabled. */
                txSettings[0] = 0x003FF5FFUL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFD55UL;
                txSettings[6] = 0x00155500UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 6:
            case 8:
                /* Gen1 with de-emphasis disabled. */
                txSettings[0] = 0x003FFFFFUL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFFFFUL;
                txSettings[6] = 0x003FFF00UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 13:
                /* Gen2 with pre-shoot enabled. */
                txSettings[0] = 0x003FFF55UL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFFFFUL;
                txSettings[6] = 0x003FFF00UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 14:
                /* Gen2 with de-emphasis enabled. */
                txSettings[0] = 0x003FFFFFUL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFFFFUL;
                txSettings[6] = 0x00155500UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 15:
                /* Gen2 with pre-shoot and de-emphasis enabled. */
                txSettings[0] = 0x003FF555UL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFFF5UL;
                txSettings[6] = 0x00155500UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 16:
                /* Gen2 with pre-shoot and de-emphasis disabled. */
                txSettings[0] = 0x003FFFFFUL;
                txSettings[1] = 0x003FFFFFUL;
                txSettings[2] = 0x003FFFFFUL;
                txSettings[3] = 0x003FFFFFUL;
                txSettings[4] = 0x003FFFFFUL;
                txSettings[5] = 0x003FFFFFUL;
                txSettings[6] = 0x003FFF00UL;
                txSettings[7] = 0x00000000UL;
                SSPhy_Update_TxSettings(pPhyRegs, txSettings);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, txSettings);
                }
                break;

            case 9:
            case 0:
                /* Return to the default TX settings. */
                SSPhy_Update_TxSettings(pPhyRegs, pCalCtxt->txAfeSettings[pCalCtxt->activePhyIdx]);
                if (pCalCtxt->dualLaneEnabled) {
                    SSPhy_Update_TxSettings(pPhyRegsAlt, pCalCtxt->txAfeSettings[!pCalCtxt->activePhyIdx]);
                }
                break;

            default:
                /* No change. Settings are already correct. */
                break;
        }

        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_NEXT_CP_SEL);
        DBG_SSCAL_INFO("CP: %x %x\r\n", pCalCtxt->currCompPattern, pLinkBase->LNK_COMPLIANCE_PATTERN_OBSERVE);
    }
}


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
*******************************************************************************/
void
Cy_USBSS_Cal_ExitLinkCompliance (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_LNK_Type *pLinkBase;
    cy_stc_usb_cal_msg_t msg = {CY_USB_CAL_MSG_INVALID, {0, 0}};

    if ((pCalCtxt != 0) && (pCalCtxt->pLinkBase != 0) && (pCalCtxt->inCompState)) {
        pLinkBase = pCalCtxt->pLinkBase;

        /* Disable and clear the Ping.LFPS interrupt. */
        pLinkBase->LNK_INTR_MASK &= ~(USB32DEV_LNK_INTR_RX_PING_LFPS_Msk);
        pLinkBase->LNK_INTR       = USB32DEV_LNK_INTR_RX_PING_LFPS_Msk;

        /* Move LTSSM to Rx.Detect and release it. */
        pLinkBase->LNK_LTSSM_STATE = ((CY_USBSS_LNK_STATE_RXDETECT_ACT << USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_VALUE_Pos) |
            USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk);
        Cy_SysLib_DelayUs(100);
        pLinkBase->LNK_LTSSM_STATE &= ~USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk;

        /* Update variables to indicate link not in compliance. */
        pCalCtxt->inCompState     = false;
        pCalCtxt->currCompPattern = 0;
        pCalCtxt->compExitDone    = true;
        pCalCtxt->compExitTS      = Cy_USBD_GetTimerTick();

        if (pCalCtxt->gen1SpeedForced) {
            pCalCtxt->gen1SpeedForced = false;

            /* Force link reconnection by sending message to USBD. */
            msg.type    = CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE;
            msg.data[0] = (pCalCtxt->dualLaneEnabled) ? CY_USBD_USB_DEV_SS_GEN2X2 : CY_USBD_USB_DEV_SS_GEN2;
            msg.data[1] = 1;
            Cy_USBSS_Cal_SendMsg(pCalCtxt, &msg);
        }
    }
}   /* end of function */

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
bool
Cy_USBSS_Cal_DeepSleepPrep (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type *pMainBase = pCalCtxt->pMainBase;
    USB32DEV_LNK_Type *pLinkBase = pCalCtxt->pLinkBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyTop = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_RX_Type  *pPhyRx  = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_RX;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *pPhyPll = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyTopAlt = &base->USB32DEV_PHYSS.USB40PHY[!pCalCtxt->activePhyIdx].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_RX_Type  *pPhyRxAlt = &base->USB32DEV_PHYSS.USB40PHY[!pCalCtxt->activePhyIdx].USB40PHY_RX;
    uint32_t intState;

    intState = Cy_SysLib_EnterCriticalSection();

    /* Bail out if link is not suspended or if INTR1 has been received already. */
    if ((pCalCtxt->linkSuspended == false) || (pPhyTop->INTR1 != 0)) {
        Cy_SysLib_ExitCriticalSection(intState);
        return false;
    }

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_U3_PHY_DISABLED);
    DBG_SSCAL_INFO("DPSLPPREP1: %x\r\n", pPhyTop->INTR1_MASKED);

    /* 1. Disable clock to controller (already done). */
    pMainBase->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    if (pCalCtxt->dualLaneEnabled) {
        pPhyRxAlt->RX_ATEST_CFG = USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_VREGDIV_EN_Msk;
        /* 2a. Reset PLL. */
        pPhyTopAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

        /* 3a. Disable PCLK. */
        pPhyTopAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;

        /* 4a. Tx PowerDown. */
        pPhyTopAlt->TX_AFE_CTRL_0 = 0;

        /* 5a. Rx PowerDown. */
        pPhyRxAlt->RX_GNRL_CFG &= ~(USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DESERRESETB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_CLKGEN_PDB_Msk);
        pPhyRxAlt->RX_BIASGEN_CFG1 &= ~USB32DEV_PHYSS_USB40PHY_RX_BIASGEN_CFG1_BIAS_PDB_Msk;
        pPhyRxAlt->RX_DFEA_CFG0 &= ~(USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3_PDB_Msk);
        pPhyRxAlt->RX_LD_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LD_RESETB_Msk;
        pPhyRxAlt->RX_VREG_CFG1 &= ~(USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VCPREG_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXA_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXD_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXCK_PDB_Msk);

        /* 6a. PLL PowerDown. */
        pPhyTopAlt->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
    }
    pPhyRx->RX_ATEST_CFG = USB32DEV_PHYSS_USB40PHY_RX_ATEST_CFG_ADFT_VREGDIV_EN_Msk;

    /* 2. Reset PLL. */
    pPhyTop->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;

    /* 3. Disable PCLK. */
    pPhyTop->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;

    /* 4. Tx PowerDown. */
    pPhyTop->TX_AFE_CTRL_0 = 0;

    /* 5. Rx PowerDown. */
    pPhyRx->RX_GNRL_CFG &= ~(USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_DESERRESETB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_GNRL_CFG_CLKGEN_PDB_Msk);
    pPhyRx->RX_BIASGEN_CFG1 &= ~USB32DEV_PHYSS_USB40PHY_RX_BIASGEN_CFG1_BIAS_PDB_Msk;
    pPhyRx->RX_DFEA_CFG0 &= ~(USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFE_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP1_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP2_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_DFEA_CFG0_DFETAP3_PDB_Msk);
    pPhyRx->RX_LD_CFG &= ~USB32DEV_PHYSS_USB40PHY_RX_LD_CFG_LD_RESETB_Msk;
    pPhyRx->RX_VREG_CFG1 &= ~(USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VCPREG_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXA_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXD_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_RX_VREG_CFG1_VREGRXCK_PDB_Msk);

    /* 6. PLL PowerDown */
    pPhyPll->PLL_SSM_CFG2 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_SSM_CFG2_SSM_ENABLE_Msk;
    pPhyPll->PLL_GNRL_CFG &= ~(
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_PLL_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_LKDT_RESETB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_RESETLOCKB_Msk);
    pPhyPll->PLL_DIVP_CFG &= ~(
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_DIVP_CFG_TX0_DIVP_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_DIVP_CFG_TX1_DIVP_PDB_Msk);
    pPhyTop->TOP_CTRL_0 &= ~USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
    pPhyPll->PLL_VREG_CFG2 |= USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VCPREG_BPB_Msk;
    Cy_SysLib_DelayUs(1);
    pPhyPll->PLL_VREG_CFG2 &= ~(
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VCPREG_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGLCPLL_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGREF_PDB_Msk |
            USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGDIG_PDB_Msk);

    /* 7. Change supply used for LFPS detection. */
    pPhyTop->PIPE_RX_CTRL |= USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_VCC_SEL_LFPS_DETECT_Msk;

    /* 8. Set flag indicating SSPHY has been disabled and needs to be re-enabled. */
    pCalCtxt->phyDisabled = true;

    /* 9. Reduce U3 exit LFPS duration when we are entering deep sleep. */
    pLinkBase->LNK_LFPS_RX_U3_EXIT = CY_USBSS_LFPS_PERIOD_TO_REG_G1(250UL);

    /* Set DeepSleep Flag */
    pCalCtxt->deepSleepEntered = true;
    DBG_SSCAL_INFO("U3: SSPhy disabled (%x)\r\n", pPhyTop->INTR1_MASKED);
    Cy_SysLib_ExitCriticalSection(intState);

    return true;
}

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
void
Cy_USBSS_Cal_DeepSleepExit (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type *pMainBase = pCalCtxt->pMainBase;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyTop = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_TOP_Type *pPhyTopAlt = &base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP;
    USB32DEV_PHYSS_USB40PHY_PLL_SYS_Type *pPhyPll = &base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_PLL_SYS;
    uint32_t intState;

    if (pCalCtxt->dualLaneEnabled == false) {
        pPhyTop = &base->USB32DEV_PHYSS.USB40PHY[pCalCtxt->activePhyIdx].USB40PHY_TOP;
    }

    intState = Cy_SysLib_EnterCriticalSection();
    if (pCalCtxt->phyDisabled) {
        DBG_SSCAL_TRACE("U3Exit: SSPhy enabled\r\n");

        /* 8u. Clear PHY disabled flag. */
        pCalCtxt->phyDisabled = false;

        /* 7u. Restore supply used for LFPS detection. */
        pPhyTop->PIPE_RX_CTRL &= ~USB32DEV_PHYSS_USB40PHY_TOP_PIPE_RX_CTRL_VCC_SEL_LFPS_DETECT_Msk;

        /* 4u. Restore TX state */
        pPhyTop->TX_AFE_CTRL_0 = (
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_DRV_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_CML2CMOS_EN_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_SEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_SEL_Pos) |
            USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_EN_Msk |
            (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_SEL_Pos) |
            (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DCD_CTRL_Pos) |
            (0x02UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DETECTRX_TRIM_Pos) |
            (0x03UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Pos)
            );

        if (pCalCtxt->dualLaneEnabled) {
            /* 4ua. Restore TX state */
            pPhyTopAlt->TX_AFE_CTRL_0 = (
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_EN_Msk |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_DRV_EN_Msk |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_CML2CMOS_EN_Msk |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_TX_SERIALIZER_RST_Msk |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_EN_Msk |
                    (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VCPREG_SEL_Pos) |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_EN_Msk |
                    (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_CLK_SEL_Pos) |
                    USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_EN_Msk |
                    (0x06UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_VREG_DRV_SEL_Pos) |
                    (0x01UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DCD_CTRL_Pos) |
                    (0x02UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_DETECTRX_TRIM_Pos) |
                    (0x03UL << USB32DEV_PHYSS_USB40PHY_TOP_TX_AFE_CTRL_0_REG_TX_AFE_TX_ELECIDLE_DRV_Pos)
                    );
        }

        /* 6u. Restore PLL state. */
        pPhyPll->PLL_VREG_CFG2 |= (
                USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VCPREG_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGLCPLL_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGREF_PDB_Msk |
                USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VREGDIG_PDB_Msk);
        Cy_SysLib_DelayUs(1);
        pPhyPll->PLL_VREG_CFG2 &= ~USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_VREG_CFG2_VCPREG_BPB_Msk;

        pPhyTop->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
        if (pCalCtxt->dualLaneEnabled) {
            pPhyTopAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PLL_VHS_LEVELSHIFT_EN_Msk;
        }
        pPhyPll->PLL_GNRL_CFG |= USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_PLL_PDB_Msk;
        Cy_SysLib_DelayUs(1);
        pPhyPll->PLL_GNRL_CFG |= USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_LKDT_RESETB_Msk;
        pPhyPll->PLL_GNRL_CFG |= USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_GNRL_CFG_RESETLOCKB_Msk;

        if (pCalCtxt->gen2Enabled) {
            pPhyPll->PLL_DIVP_CFG = 0x00000020UL;
            Cy_SysLib_DelayUs(2);
            pPhyPll->PLL_DIVP_CFG = 0x00000060UL;
            Cy_SysLib_DelayUs(2);
            pPhyPll->PLL_DIVP_CFG = 0x000000E0UL;
        } else {
            pPhyPll->PLL_DIVP_CFG = 0x00000021UL;
            Cy_SysLib_DelayUs(2);
            pPhyPll->PLL_DIVP_CFG = 0x00000061UL;
            Cy_SysLib_DelayUs(2);
            pPhyPll->PLL_DIVP_CFG = 0x000000E1UL;
        }

        pPhyPll->PLL_SSM_CFG2 |= USB32DEV_PHYSS_USB40PHY_PLL_SYS_PLL_SSM_CFG2_SSM_ENABLE_Msk;

        /* 5u. Restore RX state. */
        Cy_USBSS_Phy_RxInit(pCalCtxt, pCalCtxt->activePhyIdx, false);
        if (pCalCtxt->dualLaneEnabled) {
            Cy_USBSS_Phy_RxInit(pCalCtxt, !(pCalCtxt->activePhyIdx), false);
        }

        /* 2u. Release PLL from reset. */
        pPhyTop->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
        if(pCalCtxt->dualLaneEnabled) {
            /* 2ua. Release PLL from reset. */
            pPhyTopAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PLL_RSTB_Msk;
        }

        /* 3u. Enable PCLK */
        pPhyTop->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;

        if (pCalCtxt->dualLaneEnabled) {
            /* 3ua. Enable PCLK */
            pPhyTopAlt->TOP_CTRL_0 |= USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk;
        }

        /* 1u. Enable clock to controller */
        pMainBase->CTRL |= USB32DEV_MAIN_CTRL_CLK_EN_Msk;

        if (pCalCtxt->dualLaneEnabled) {
            SSPhy_Update_TxSettings(pPhyTop, pCalCtxt->txAfeSettings[0]);
            SSPhy_Update_TxSettings(pPhyTopAlt, pCalCtxt->txAfeSettings[1]);
        } else {
            SSPhy_Update_TxSettings(pPhyTop, pCalCtxt->txAfeSettings[pCalCtxt->activePhyIdx]);
        }

        if (pCalCtxt->gen2Enabled) {
            if (pCalCtxt->dualLaneEnabled) {
                pCalCtxt->lpmExitLfpsDelay -= 250U;
            } else {
                pCalCtxt->lpmExitLfpsDelay -= 150U;
            }
        } else {
            pCalCtxt->lpmExitLfpsDelay -= 50U;
        }

        Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_SSCAL_EVT_U3_WAKE_PHY_ENABLE);
    }

    Cy_SysLib_ExitCriticalSection(intState);
}

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
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetDmaClkFreq (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                     cy_en_hbdma_clk_freq_t dmaFreq)
{
    cy_en_usb_cal_ret_code_t status = CY_USB_CAL_STATUS_CAL_CTXT_NULL;

    if (pCalCtxt != NULL) {
        if (pCalCtxt->connectRcvd) {
            /* Desired frequency has to be set before USB connection is active. */
            status = CY_USB_CAL_STATUS_FAILURE;
        } else {
            status = CY_USB_CAL_STATUS_SUCCESS;
            pCalCtxt->desiredDmaFreq = dmaFreq;
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_USBSS_Cal_UpdateDmaClkFreq
 ****************************************************************************//**
 *
 * Function which updates the DMA clock frequency at runtime based on USB
 * controller and PHY status.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
 * allocated by the user.
 *
 * \param pclk_enabled
 * Indicates whether PCLK at the device level is enabled.
 *******************************************************************************/
static void Cy_USBSS_Cal_UpdateDmaClkFreq (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool pclk_enabled)
{
    USB32DEV_MAIN_Type *USB32DEV_MAIN;

    if (pCalCtxt != NULL) {
        USB32DEV_MAIN = &(pCalCtxt->regBase->USB32DEV_MAIN);

        if (pclk_enabled) {
            /* Switch to the desired frequency: Doing this even if frequency is already correct so that
             * half data rate bit gets set correctly.
             */
            if (Cy_HBDma_SetClockFrequency(pCalCtxt->desiredDmaFreq) == CY_HBDMA_SUCCESS) {
                pCalCtxt->currentDmaFreq = pCalCtxt->desiredDmaFreq;

                if ((pCalCtxt->gen2Enabled) && (pCalCtxt->currentDmaFreq != CY_HBDMA_CLK_240_MHZ) &&
                        (pCalCtxt->currentDmaFreq != CY_HBDMA_CLK_SSPHY_CLK)) {
                    /* Set half data rate bit when operating in Gen2 with DMA clock slower than 240 MHz. */
                    USB32DEV_MAIN->CTRL |= USB32DEV_MAIN_CTRL_EN_EGRS_RQ_HALF_DATA_RATE_Msk;
                } else {
                    /* Clear half data rate bit in other cases. */
                    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_EN_EGRS_RQ_HALF_DATA_RATE_Msk;
                }
            } else {
                DBG_SSCAL_ERR("Failed to set DMA clock frequency\r\n");
            }
        } else {
            /* If PCLK was being used for DMA, revert to 150 MHz clock. */
            if (pCalCtxt->currentDmaFreq == CY_HBDMA_CLK_SSPHY_CLK) {
                if (Cy_HBDma_SetClockFrequency(CY_HBDMA_CLK_150_MHZ) == CY_HBDMA_SUCCESS) {
                    pCalCtxt->currentDmaFreq = CY_HBDMA_CLK_150_MHZ;
                } else {
                    DBG_SSCAL_ERR("Failed to set DMA clock frequency\r\n");
                }
            }

            /* Clear half data rate bit whenever PCLK is disabled. */
            USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_EN_EGRS_RQ_HALF_DATA_RATE_Msk;
        }
    }
}

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
Cy_USBSS_Cal_DisableLPMDeviceExit (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool devExitDisable)
{
    if (pCalCtxt != NULL) {
        pCalCtxt->devLpmExitDisabled = devExitDisable;
    }
}

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
void
Cy_USBSS_Cal_PostEpEnable (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_MAIN_Type *USB32DEV_MAIN;

    /* This configuration is not applicable on Rev-A0 Silicon. */
    if ((pCalCtxt != NULL) && (Cy_SysLib_GetDeviceRevision() != 0x11)) {
        USB32DEV_MAIN = pCalCtxt->pMainBase;

        /* Initialisation work-around. */
        USB32DEV_MAIN->BUG_FIX |= USB32DEV_MAIN_BUG_FIX_RESET_SKT_HAS_EOB_FUNC_Msk;
        Cy_SysLib_DelayUs(1);
        USB32DEV_MAIN->BUG_FIX &= ~USB32DEV_MAIN_BUG_FIX_RESET_SKT_HAS_EOB_FUNC_Msk;
    }
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ReleaseLTSSM
****************************************************************************//**
*
* Function to release the LTSSM from a forced state.
*
* \param pCalCtxt
* USBSS Controller context structure.
*******************************************************************************/
void
Cy_USBSS_Cal_ReleaseLTSSM (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_LNK_Type *USB32DEV_LNK;

    if (pCalCtxt != NULL) {
        USB32DEV_LNK = pCalCtxt->pLinkBase;

        /* If the LTSSM force bit is set, clear it. Link layer will manage further transitions. */
        if ((USB32DEV_LNK->LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk) != 0) {
            USB32DEV_LNK->LNK_LTSSM_STATE &= ~USB32DEV_LNK_LTSSM_STATE_LTSSM_OVERRIDE_EN_Msk;
        }
    }
}

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
void Cy_USBSS_Cal_ClkStopOnEpRstEnable (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool clkStopEn)
{
    if (pCalCtxt != NULL) {
        pCalCtxt->stopClkOnEpResetEnable = clkStopEn;
    }
}

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
void Cy_USBSS_Cal_SelectConfigLane (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                    cy_en_usb_config_lane_t laneSel)
{
    if ((pCalCtxt != NULL) && (laneSel <= CY_USB_CFG_LANE_1)) {
        pCalCtxt->usbConfigLane = laneSel;
    }
}

#if defined(__cplusplus)
}
#endif

#endif /* FX2G3_EN */

/* End of file */

