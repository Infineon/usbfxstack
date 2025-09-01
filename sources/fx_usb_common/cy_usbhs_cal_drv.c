/***************************************************************************//**
* \file cy_usbhs_cal_drv.c
* \version 1.0
*
* Driver implementation for the USBHS (2.x) block in the FX10 device family.
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

#include "cy_device_headers.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_USBHS_ConfigExtClkPin
 ****************************************************************************//**
 *
 * Function used to enable selection of external 24 MHz clock input provided
 * on external clock pin as reference for the PLL in the USB block.
 *
 * \param init - Set TRUE to initialize pin for external clock input
 *               Set FALSE to remove external clock input capability for the IO.
 * \param pinSel Set TRUE to try the alternate clock input pin.
 *
 * \return Pass/fail status of configuration update.
 *******************************************************************************/
static cy_en_gpio_status_t Cy_USBHS_ConfigExtClkPin (bool init, bool pinSel)
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;

    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    /* First, change IO mode for the alternate pin to GPIO to avoid interference. */
    pinCfg.driveMode = CY_GPIO_DM_ANALOG;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    (void)Cy_GPIO_Pin_Init(GPIO_PRT5, (pinSel) ? CY_EXTERNAL_CLK_PIN : CY_EXTERNAL_CLK_PIN_ALT, &pinCfg);

    if (init) {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        pinCfg.hsiom     = P5_0_SRSS_EXT_CLK;
    }

    status = Cy_GPIO_Pin_Init(GPIO_PRT5, (pinSel) ? CY_EXTERNAL_CLK_PIN_ALT : CY_EXTERNAL_CLK_PIN, &pinCfg);
    return status;
}

/*******************************************************************************
 * Function name:Cy_USBHS_WaitForPllLock
 ****************************************************************************//**
 *  Wait till timeout for USB HS PLL to get locked.
 *
 * \param timeoutMs - Timeout value in milliseconds.
 *                    Set timeout as 0 for infinite wait.
 *
 * \return True - PLL Lock, False - PLL not locked (timeout)
 *******************************************************************************/
static bool Cy_USBHS_WaitForPllLock (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t timeoutMs)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    uint32_t pllLockWait = 0;

    /* wait for PLL_LOCK interrupt and then clear the interrupt*/
    while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK)))
    {
        if ((timeoutMs) && (pllLockWait++ > timeoutMs))
        {
            DBG_HSCAL_INFO("PLL Lock timedout\r\n");
            return false;
        }
        Cy_SysLib_Delay(1);
    }

    pPhyBase->INTR0 = USBHSPHY_INTR0_PLL_LOCK;
    return true;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_Init
****************************************************************************//**
*
* Initializes the HS controller ie CAL layer and PHY. It also stores callback
* function and USBD layer context.
*
* \param pCalCtxt
* CAL layer library context pointer.
*
* \param pUsbdCtxt
* USBd layer context pointer.
*
* \param callBackFunc
* Callback function provied by Upper layer ie USBD layer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_Init (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                   void *pUsbdCtxt,
                   cy_usb_cal_msg_callback_t callBackFunc)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    DBG_HSCAL_TRACE("Cy_USBHS_Cal_Init  >>\r\n");

    /*
     * This function assumes that pCalBase and pPhyBase allready
     * populated by initial caller.
     */
    pCalCtxt->pUsbdCtxt = pUsbdCtxt;
    pCalCtxt->msgCb = callBackFunc;
    pCalCtxt->enableSOFInterrupt = false;

    pCalBase->POWER |= USBHSDEV_POWER_RESETN;
    Cy_SysLib_DelayUs(1);

    pCalBase->POWER |= USBHSDEV_POWER_VBUS_VALID;
    pCalBase->POWER &= ~(USBHSDEV_POWER_EPM_DCG_ENABLE);

    /*
     * Clear the CONT_TO_DATA bit by default. This has to be set only when we
     * are handling the EP0-OUT transfers.
     */
    pCalBase->DEV_CS &= (~(USBHSDEV_DEV_CS_CONT_TO_DATA_Msk));

    /* Enable setting to reduce round-trip latency by passing flop in EPM. */
    pCalBase->LEGACY_FEATURE_ENABLE = USBHSDEV_LEGFEAT_BYPASS_FLOP_EN;

    /*
     * Initialize FS and HS PHY together and then disable HS clock
     * if it is not required.
     */
    DBG_HSCAL_TRACE("Calling PhyCommonInit\r\n");
    Cy_USBHS_Cal_PhyCommonInit(pCalCtxt);
    Cy_USBHS_Cal_FsHsModePhyInit(pCalCtxt);
    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_CAL_INIT_DONE);
    DBG_HSCAL_TRACE("Cy_USBHS_Cal_Init  <<\r\n");
    return;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_SOFIntrUpdate
****************************************************************************//**
*
* Function to enable or disable the SOF interrupt based on user requirement.
*
* \param pCalCtxt
* CAL layer library context pointer.
*
* \param sofIntrEnable
* Whether the SOF interrupt should be enabled.
*
*******************************************************************************/
void Cy_USBHS_Cal_SOFIntrUpdate (
        cy_stc_usb_cal_ctxt_t *pCalCtxt,
        bool sofIntrEnable)
{
    USBHSDEV_Type *pCalBase;

    if ((pCalCtxt != NULL) && (pCalCtxt->pCalBase != NULL)) {
        pCalCtxt->enableSOFInterrupt = sofIntrEnable;
        pCalBase = pCalCtxt->pCalBase;

        if (sofIntrEnable) {
            /* Enable the interrupt only if other USB 2.x interrupts are enabled. */
            if (pCalBase->DEV_CTL_INTR_MASK != 0) {
                pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_SOF;
            }
        } else {
            /* Disable the interrupt. */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_SOF;
        }
    }
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_UpdateFx2g3PllSettings
****************************************************************************//**
*
* The FX2G3 devices require a different PLL configuration to ensure jitter-free
* transmitter operation. This function applies these settings before the PLL is
* enabled.
*
* \param pPhyBase
* Pointer to USBHS PHY registers.
*******************************************************************************/
static void Cy_USBHS_Cal_UpdateFx2g3PllSettings (USBHSPHY_Type  *pPhyBase)
{
#if FX2G3_EN
    if (pPhyBase != NULL) {
        /*
           Update PLL settings to avoid high jitter.
           1) pll_vco_gain[3:0] = 1011
           2) pll_p[1:0] = 01
           3) pll_q[1:0] = 01
           4) pll_cp_cur_trim = 01
           */
        /* setting vco gain, pdiv & qdiv values */
        pPhyBase->PLL_CONTROL_1 &= ~(
                USBHSPHY_PLL_CONTROL_1_P_DIV_Msk |
                USBHSPHY_PLL_CONTROL_1_Q_DIV_Msk |
                USBHSPHY_PLL_CONTROL_1_VCO_GAIN_Msk);
        pPhyBase->PLL_CONTROL_1 |= (
                (0x0BUL << USBHSPHY_PLL_CONTROL_1_VCO_GAIN_Pos) |
                (0x01UL << USBHSPHY_PLL_CONTROL_1_P_DIV_Pos) |
                (0x01UL << USBHSPHY_PLL_CONTROL_1_Q_DIV_Pos));


        /* setting cp cur trim value */
        pPhyBase->PLL_TRIMS &= ~USBHSPHY_PLL_TRIMS_CP_CUR_Msk;
        pPhyBase->PLL_TRIMS |= (0x01UL << USBHSPHY_PLL_TRIMS_CP_CUR_Pos);
    }
#else
    (void)pPhyBase;
#endif /* FX2G3_EN */
}

/*******************************************************************************
 * Function name:Cy_USBHS_Cal_TryPllLock
 ****************************************************************************//**
 * Enable the PLL and check whether lock is obtained within expected time.
 *
 * \param pCalCtxt
 * Pointer to CAL context structure.
 *
 * \param isEco
 * true to check for PLL lock with ECO reference, false to check with clock input.
 *
 * \return
 * true if PLL lock was obtained, false otherwise.
 *******************************************************************************/
static bool Cy_USBHS_Cal_TryPllLock (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool isEco)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    bool pllLocked = false;

    /* Disable the PLL and clear interrupt status first. */
    pPhyBase->PLL_CONTROL_1 &= ~(USBHSPHY_PLL_CONTROL_1_SUPPLY_EN | USBHSPHY_PLL_CONTROL_1_PLL_EN);
    pPhyBase->INTR0 = USBHSPHY_INTR0_PLL_LOCK;

    if (isEco) {
        pCalBase->POWER &= ~(USBHSDEV_POWER_REFCLK_SEL_Msk);
    } else {
        pCalBase->POWER |= (USBHSDEV_POWER_REFCLK_SEL_Msk);
    }

    /* Enable PLL supply, wait for at least 8 us and then enable PLL. */
    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
    Cy_SysLib_DelayUs(10);
    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;

    if (isEco) {
        DBG_HSCAL_INFO("ECO: Waiting for USBHS PLL Lock\r\n");
    } else {
        DBG_HSCAL_INFO("EXTCLK: Waiting for USBHS PLL Lock\r\n");
    }

    pllLocked = Cy_USBHS_WaitForPllLock(pCalCtxt, CY_USBHS_PLL_LOCK_TIMEOUT_MS);

    /* Disable PLL again if lock was not achieved. */
    if (!pllLocked) {
        pPhyBase->PLL_CONTROL_1 &= ~(USBHSPHY_PLL_CONTROL_1_SUPPLY_EN | USBHSPHY_PLL_CONTROL_1_PLL_EN);
        if (isEco) {
            Cy_SysClk_EcoDisable();
        }
    }

    return pllLocked;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_FsHsModePhyInit
****************************************************************************//**
*
* Initializes high speed PHY.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_FsHsModePhyInit (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    bool pllLocked = false;

    /* At start, we don't know whether the reference clock is from ECO or externally provided. */
    pCalCtxt->clkSrcType = USB2REF_CLK_SRC_UNKNOWN;

    DBG_HSCAL_INFO("Cy_USBHS_Cal_FsHsModePhyInit >>\r\n");

    pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_ENABLE_LV;

    /* Wait for VCCD interrupt and then clear the interrupt. */
    DBG_HSCAL_TRACE("before ENABLE_HS_VCCD loop\r\n");
    while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_HS_VCCD)) {
        Cy_SysLib_DelayUs(1);
        DBG_HSCAL_TRACE("Inside ENABLE_HS_VCCD loop. \r\n");
    }
    pPhyBase->INTR0 = USBHSPHY_INTR0_ENABLE_HS_VCCD;

    Cy_USBHS_Cal_UpdateFx2g3PllSettings(pPhyBase);

    /* If ECO has already been enabled and is showing OK status, go ahead with existing config. */
    if (Cy_SysClk_EcoGetStatus() == CY_SYSCLK_ECOSTAT_STABLE) {
        pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, true);
        if (pllLocked) {
            pCalCtxt->clkSrcType = USB2REF_CLK_SRC_ECO;
        }
    }

    if (!pllLocked) {
        /*
         * If either the XTALIN or XTALOUT pins are configured in external clock mode, try to obtain PLL lock with
         * the clock input.
         */
        if (
                (Cy_GPIO_GetHSIOM(GPIO_PRT5, CY_EXTERNAL_CLK_PIN) == P5_0_SRSS_EXT_CLK) ||
                (Cy_GPIO_GetHSIOM(GPIO_PRT5, CY_EXTERNAL_CLK_PIN_ALT) == P5_0_SRSS_EXT_CLK)
           ) {
            pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, false);
            if (pllLocked) {
                pCalCtxt->clkSrcType = USB2REF_CLK_SRC_EXT_CLK;
            }
        }

        /* Try whether we can obtain PLL lock using external clock input on either XTALOUT or XTALIN. */
        if (!pllLocked) {
            /* Enable XTALOUT pin as external clock input. */
            Cy_USBHS_ConfigExtClkPin(true, false);

            pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, false);
            if (pllLocked) {
                pCalCtxt->clkSrcType = USB2REF_CLK_SRC_EXT_CLK;
            } else {
                /* Enable XTALIN pin as external clock input. */
                Cy_USBHS_ConfigExtClkPin(true, true);

                pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, false);
                if (pllLocked) {
                    pCalCtxt->clkSrcType = USB2REF_CLK_SRC_EXT_CLK;
                } else {
                    /* Leave XTALIN and XTALOUT pins in default state. */
                    Cy_USBHS_ConfigExtClkPin(false, true);
                }
            }
        }
    }

        /* USBHS PLL is not locked, check if clock is coming from the ECO */
    if (!pllLocked) {
        DBG_HSCAL_INFO("Enabling ECO\r\n");

        /*
         * Apply typical ECO trim settings where required:
         * CLK_TRIM_ECO_CTL: WDTRIM=0, ATRIM=15, FTRIM=3, RTRIM=1, GTRIM=2
         */
        if ((SRSS->CLK_TRIM_ECO_CTL & SRSS_CLK_TRIM_ECO_CTL_WDTRIM_Msk) > 1UL) {
            SRSS->CLK_TRIM_ECO_CTL = (SRSS->CLK_TRIM_ECO_CTL & ~(0xFFFFUL)) | 0x27F0UL;
        }

        /* Enable the ECO and wait for ECO stability for a maximum of 10 ms. */
        if (Cy_SysClk_EcoEnable(10000UL) != CY_SYSCLK_SUCCESS) {
            DBG_HSCAL_INFO("ECO stability timeout\r\n");
            return;
        }

        pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, true);
        if (pllLocked) {
            pCalCtxt->clkSrcType = USB2REF_CLK_SRC_ECO;
        } else {
            DBG_HSCAL_ERR("Failed to get PLL lock. Please ensure crystal or clock input is present.\r\n");
            return;
        }
    }

    /* wait for 1 us */
    Cy_SysLib_DelayUs(1);
    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD;
    pPhyBase->CDR_CONTROL |=  USBHSPHY_CDR_CONTROL_CDR_ENABLE;

#if FX2G3_EN
    /* Update PHY TX settings for initial connection.
     * AFE_CONTROL_1.HS_AMP_SEL = 6
     * AFE_CONTROL_1.HS_PREE_SEL = 0
     */
    pPhyBase->AFE_CONTROL_1 = (
            (pPhyBase->AFE_CONTROL_1 & ~(USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Msk | USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk)) |
            (0x06UL << USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Pos));
#else
    /* Update PHY TX settings for initial connection.
     * AFE_CONTROL_1.HS_AMP_SEL = 5
     * AFE_CONTROL_1.HS_PREE_SEL = 0
     */
    pPhyBase->AFE_CONTROL_1 = (
            (pPhyBase->AFE_CONTROL_1 & ~(USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Msk | USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk)) |
            (0x05UL << USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Pos));
#endif /* FX2G3_EN */

    /* PHY RX update for compliance. */
    pPhyBase->VREFGEN_CONTROL = (
            (pPhyBase->VREFGEN_CONTROL & ~USBHSPHY_VREFGEN_CONTROL_TED_SEL_0_Msk) |
            (0x07UL << USBHSPHY_VREFGEN_CONTROL_TED_SEL_0_Pos));

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_PHY_FSHSMODE_INIT);

    DBG_HSCAL_INFO("Cy_USBHS_Cal_FsHsModePhyInit <<\r\n");
    return;
}    /* end of function */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_PreemphasisControl
****************************************************************************//**
*
* Function to enable/disable Pre-emphasis based on the active USB test mode.
*
* \param pCalCtxt
* CAL layer context pointer.
* \param enable
* Whether pre-emphasis is to be enabled or disabled.
*
*******************************************************************************/
void
Cy_USBHS_Cal_PreemphasisControl (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                 bool enable)
{
    USBHSPHY_Type *pPhyBase;

    if ((pCalCtxt != NULL) && (pCalCtxt->pPhyBase != NULL)) {
        pPhyBase = pCalCtxt->pPhyBase;

        if (enable) {
#if FX2G3_EN
            /* Optimal PHY settings for far and near end eye testing.
             * AFE_CONTROL_1.HS_AMP_SEL = 6
             * AFE_CONTROL_1.HS_PREE_SEL = 5
             * DIGITAL_CONTROL.DLAUNCH_SEL = 2
             */
            pPhyBase->AFE_CONTROL_1 = (
                    (pPhyBase->AFE_CONTROL_1 & ~(USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Msk | USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk)) |
                    (0x06UL << USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Pos) |
                    (0x05UL << USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Pos));
            Cy_SysLib_Delay(1);
            pPhyBase->DIGITAL_CONTROL = (
                    (pPhyBase->DIGITAL_CONTROL & ~USBHSPHY_DIGITAL_CONTROL_DLAUNCH_SEL_Msk) |
                    (0x02UL << USBHSPHY_DIGITAL_CONTROL_DLAUNCH_SEL_Pos));
#else
            /* Optimal PHY settings for far and near end eye testing.
             * AFE_CONTROL_1.HS_AMP_SEL = 5
             * AFE_CONTROL_1.HS_PREE_SEL = 5
             * DIGITAL_CONTROL.DLAUNCH_SEL = 2
             */
            pPhyBase->AFE_CONTROL_1 = (
                    (pPhyBase->AFE_CONTROL_1 & ~(USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Msk | USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk)) |
                    (0x05UL << USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Pos) |
                    (0x05UL << USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Pos));
            Cy_SysLib_Delay(1);
            pPhyBase->DIGITAL_CONTROL = (
                    (pPhyBase->DIGITAL_CONTROL & ~USBHSPHY_DIGITAL_CONTROL_DLAUNCH_SEL_Msk) |
                    (0x02UL << USBHSPHY_DIGITAL_CONTROL_DLAUNCH_SEL_Pos));
#endif /* FX2G3_EN */
        } else {
            /* Disable pre-emphasis where required.
             * AFE_CONTROL_1.HS_PREE_SEL = 0
             * DIGITAL_CONTROL.DLAUNCH_SEL = 0
             */
            pPhyBase->AFE_CONTROL_1 &= ~USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk;
            pPhyBase->DIGITAL_CONTROL &= ~USBHSPHY_DIGITAL_CONTROL_DLAUNCH_SEL_Msk;
        }
    }
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_DeinitPLL
****************************************************************************//**
*
* Function to de-initialize the PLL in the USBHS block.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS in case of success; error code otherwise.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DeinitPLL (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type *pPhyBase;

    if ((pCalCtxt == NULL) || (pCalCtxt->pPhyBase == NULL)) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pPhyBase = pCalCtxt->pPhyBase;

    /* Disable CDR. */
    pPhyBase->CDR_CONTROL &= ~USBHSPHY_CDR_CONTROL_CDR_ENABLE;

    /* Disable the HS supply. */
    pPhyBase->AFE_CONTROL_1 &= ~USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD_Msk;

    /* Make sure PLL is disabled. */
    pPhyBase->PLL_CONTROL_1 &= ~(USBHSPHY_PLL_CONTROL_1_SUPPLY_EN_Msk | USBHSPHY_PLL_CONTROL_1_PLL_EN);

    /* Disable the 1P1 regulator. */
    pPhyBase->REG_1P1_CONTROL &= ~(USBHSPHY_REG_1P1_CONTROL_ENABLE_LV_Msk | USBHSPHY_REG_1P1_CONTROL_SWITCH_EN_Msk);

    if (pCalCtxt->clkSrcType == USB2REF_CLK_SRC_ECO) {
        /* Disable the ECO. */
        Cy_SysClk_EcoDisable();
    }

    /* regulator bypass to use 3.3v */
    pPhyBase->REG_2P5_CONTROL |= USBHSPHY_REG_2P5_CONTROL_BYPASS_MODE;
    pPhyBase->REG_2P5_CONTROL &= (~USBHSPHY_REG_2P5_CONTROL_ENABLE_LV);

    /* Disable VREFGEN, IREFGEN and clear all interrupts. */
    pPhyBase->VREFGEN_CONTROL &= (~USBHSPHY_VREFGEN_CONTROL_ENABLE_LV);
    pPhyBase->IREFGEN_CONTROL &= ~USBHSPHY_IREFGEN_CONTROL_ENABLE_LV_Msk;
    pPhyBase->INTR0 = pPhyBase->INTR0;
    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_PLL_DEINIT);

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_InitPLL
****************************************************************************//**
*
* Function to initialize the PLL in the USBHS block to generate the 480 MHz
* clock required by the High BandWidth SubSystem for its operation.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS in case of success; error code otherwise.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_InitPLL (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type *pPhyBase;
    uint32_t regVal, loopCnt;
    bool pllLocked = false;

    if ((pCalCtxt == NULL) || (pCalCtxt->pPhyBase == NULL)) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pPhyBase = pCalCtxt->pPhyBase;

    /* Make sure any earlier interrupt status is cleared. */
    pPhyBase->INTR0 = pPhyBase->INTR0;

    /* Enable the current and voltage reference generators and wait 20 us for its output to be stable. */
    pPhyBase->IREFGEN_CONTROL |= USBHSPHY_IREFGEN_CONTROL_ENABLE_LV_Msk;
    pPhyBase->VREFGEN_CONTROL |= USBHSPHY_VREFGEN_CONTROL_ENABLE_LV;
    Cy_SysLib_DelayUs(20);

    /* Enable 2.5 V supply and remove bypass connection to 3.3V supply. */
    pPhyBase->REG_2P5_CONTROL |= (USBHSPHY_REG_2P5_CONTROL_ENABLE_LV);
    pPhyBase->REG_2P5_CONTROL &= (~USBHSPHY_REG_2P5_CONTROL_BYPASS_MODE);

    /* Enable the 1P1 regulator. */
    pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_ENABLE_LV_Msk | USBHSPHY_REG_1P1_CONTROL_SWITCH_EN_Msk;

    /* Wait for 1P1 regulator output to be stable. */
    loopCnt = 100;
    do {
        regVal = pPhyBase->INTR0;
        Cy_SysLib_DelayUs(1);
    } while (((regVal & USBHSPHY_INTR0_ENABLE_HS_VCCD_Msk) == 0) && (loopCnt--));

    if ((pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_HS_VCCD_Msk) == 0) {
        DBG_HSCAL_INFO("1P1 regulator timeout\r\n");
        Cy_USBHS_Cal_DeinitPLL(pCalCtxt);
        return CY_USB_CAL_STATUS_TIMEOUT;
    }
    pPhyBase->INTR0 = USBHSPHY_INTR0_ENABLE_HS_VCCD_Msk;

    Cy_USBHS_Cal_UpdateFx2g3PllSettings(pPhyBase);

    if (pCalCtxt->clkSrcType != USB2REF_CLK_SRC_ECO) {
        /* Make sure ECO is disabled first. */
        Cy_SysClk_EcoDisable();

        /* If clock source is unknown, try enabling clock input on XTALOUT pin. */
        if (pCalCtxt->clkSrcType == USB2REF_CLK_SRC_UNKNOWN) {
            Cy_USBHS_ConfigExtClkPin(true, false);
        }

        pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, false);
        if (pllLocked) {
            pCalCtxt->clkSrcType = USB2REF_CLK_SRC_EXT_CLK;
        } else {
            Cy_USBHS_ConfigExtClkPin(false, false);
        }
    }

    /* USBHS PLL is not locked, check if clock is coming from the ECO */
    if (!pllLocked) {
        DBG_HSCAL_TRACE("Enabling ECO\r\n");

        /*
         * Apply typical ECO trim settings where required:
         * CLK_TRIM_ECO_CTL: WDTRIM=0, ATRIM=15, FTRIM=3, RTRIM=1, GTRIM=2
         */
        if ((SRSS->CLK_TRIM_ECO_CTL & SRSS_CLK_TRIM_ECO_CTL_WDTRIM_Msk) > 1UL) {
            SRSS->CLK_TRIM_ECO_CTL = (SRSS->CLK_TRIM_ECO_CTL & ~(0xFFFFUL)) | 0x27F0UL;
        }

        /* Enable the ECO and wait for ECO stability for a maximum of 10 ms. */
        if (Cy_SysClk_EcoEnable(10000UL) != CY_SYSCLK_SUCCESS) {
            DBG_HSCAL_INFO("ECO stability timeout\r\n");
            Cy_USBHS_Cal_DeinitPLL(pCalCtxt);
            return CY_USB_CAL_STATUS_TIMEOUT;
        }

        pllLocked = Cy_USBHS_Cal_TryPllLock(pCalCtxt, true);
        if (pllLocked) {
            pCalCtxt->clkSrcType = USB2REF_CLK_SRC_ECO;
        } else {
            DBG_HSCAL_ERR("Failed to obtain USBHS PLL lock: Check crystal/clock connection\r\n");
            pCalCtxt->clkSrcType = USB2REF_CLK_SRC_UNKNOWN;
            return CY_USB_CAL_STATUS_TIMEOUT;
        }
    }

    pPhyBase->INTR0 = USBHSPHY_INTR0_PLL_LOCK_Msk;

    /* Enable the HS supply and ungate the clock. */
    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD_Msk;
    pPhyBase->CDR_CONTROL |= USBHSPHY_CDR_CONTROL_CDR_ENABLE;

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_PLL_INIT);

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_PhyCommonInit
****************************************************************************//**
*
* Initializes registers common to full and high speed PHY configuration.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_PhyCommonInit (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    DBG_HSCAL_INFO("Cy_USBHS_Cal_PhyCommonInit >>\r\n");

    /* Make sure any earlier interrupt status is cleared. */
    pPhyBase->INTR0 = pPhyBase->INTR0;

    /* Current reference initialization */
    pPhyBase->IREFGEN_CONTROL |= USBHSPHY_IREFGEN_CONTROL_ENABLE_LV;
    /* Voltage reference initialization */
    pPhyBase->VREFGEN_CONTROL |= USBHSPHY_VREFGEN_CONTROL_ENABLE_LV;
    /* wait for 20us  */
    Cy_SysLib_DelayUs(20);

    /*
     * During this time interrupts are not enabled so keep checking
     * required bit in interrupt register as polling mechanism.
     */
    pPhyBase->REG_2P5_CONTROL |= USBHSPHY_REG_2P5_CONTROL_ENABLE_LV;

    DBG_HSCAL_TRACE("before USBHSPHY_INTR0_ENABLE_VCCD loop\r\n");
    /* check for enable vccd interrupt here only */
    while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_VCCD)) {
        Cy_SysLib_DelayUs(1);
        DBG_HSCAL_TRACE("Inside ENABLE_VCCD loop. \r\n");
    }
    /* Clear interrupt bit by writing "1" */
    pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_VCCD);

    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_VCCD;
    /* wait for 40us  */
    Cy_SysLib_DelayUs(40);
    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_PHY_COMM_INIT);

    DBG_HSCAL_INFO("Cy_USBHS_Cal_PhyCommonInit <<\r\n");
    return;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_HandleReset
****************************************************************************//**
*
* Some of controller register will be brought to default state.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_HandleReset (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intState;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intState = Cy_SysLib_EnterCriticalSection();
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);
    pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_URESET;
    Cy_SysLib_ExitCriticalSection(intState);

    return(CY_USB_CAL_STATUS_SUCCESS);
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_ConnUsbPins
****************************************************************************//**
*
* Update USB device controller register to make USB device visible on Bus. Once
* device is visible, host will start reset and Enumeration.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ConnUsbPins (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* Disable pre-emphasis before making USB connection. */
    Cy_USBHS_Cal_PreemphasisControl(pCalCtxt, false);

    pCalBase->DEV_TIM_T_UCH           = 36000UL;                /* Set device chirp duration to 1.2 ms. */
    pCalBase->DEV_TIM_T_WTFS          = 36000UL;                /* Set tWTFS delay to 1.2 ms. */
    pCalBase->DEV_TIM_T_WTREV_WTRSTFS = 42300UL;                /* Time till end of chirp is 1.41 ms. */

    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DISCON);

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_CONN_EN);

    return(CY_USB_CAL_STATUS_SUCCESS);
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_DisconUsbPins
****************************************************************************//**
*
* Update Controller register to make USB device invisible on Bus.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DisconUsbPins (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* If link is in suspend at time of connection, we need to re-enable PHY first. */
    if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_DEV_SUSPEND) != 0) {
        DBG_HSCAL_INFO("Re-activating USBHS IP before disconnection\r\n");
        Cy_USBHS_Cal_HsHandleL2Resume(pCalCtxt);
    } else {
        if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0) {
            /* We need to re-enable the PHY in L1 state as well. */
            DBG_HSCAL_INFO("Restore PHY state before disconnection\r\n");
            Cy_USBHS_Cal_HsHandleL1WakeupCommon(pCalCtxt);
        }
    }

    pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_DISCON;

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_CONN_DIS);
    return(CY_USB_CAL_STATUS_SUCCESS);

}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_HsHandleL1Sleep
****************************************************************************//**
*
* L1-SLEEP request during device in high speed mode will be handled here.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_HsHandleL1Sleep (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    DBG_HSCAL_TRACE("Handle L1_SLEEP >>\r\n");
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    pPhyBase->CDR_CONTROL &= (~USBHSPHY_CDR_CONTROL_CDR_ENABLE);
    pPhyBase->PLL_CONTROL_1 &= (~USBHSPHY_PLL_CONTROL_1_PLL_EN);
    pPhyBase->PLL_CONTROL_1 &= (~USBHSPHY_PLL_CONTROL_1_SUPPLY_EN);
    pPhyBase->PLL_CONTROL_2 &= (~USBHSPHY_PLL_CONTROL_2_SOURCE_OF_PLL_LOCK_Msk);
    DBG_HSCAL_TRACE("Handle L1_SLEEP <<\r\n");

    return;
}   /* end of function () */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_HsHandleL1WakeupCommon
****************************************************************************//**
*
* L1-Wakeup request during device in high speed mode will be handled here.
* This is common function which will be used irrespective of host initiated
* or device initiated L1 wakeup.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_HsHandleL1WakeupCommon (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    uint32_t intState = Cy_SysLib_EnterCriticalSection();

    if (pPhyBase->CDR_CONTROL & USBHSPHY_CDR_CONTROL_CDR_ENABLE) {
        Cy_SysLib_ExitCriticalSection(intState);
        DBG_HSCAL_TRACE("CDR_ENABLE already set\r\n");
        return;
    }

    /* Make sure any earlier interrupt status is cleared. */
    pPhyBase->INTR0 = pPhyBase->INTR0;

    Cy_USBHS_Cal_UpdateFx2g3PllSettings(pPhyBase);

    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
    /* wait for 8us  */
    Cy_SysLib_DelayUs(8);
    pPhyBase->PLL_CONTROL_2 |= USBHSPHY_PLL_CONTROL_2_SOURCE_OF_PLL_LOCK_Msk;

    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;
    while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK))) {
        Cy_SysLib_DelayUs(1);
        DBG_HSCAL_TRACE("L1 WAKE: Inside PLL_LOCK wait loop. \r\n");
    }

    /* Clear the PLL_LOCK interrupt. */
    pPhyBase->INTR0  |= USBHSPHY_INTR0_PLL_LOCK;
    pPhyBase->CDR_CONTROL |=  USBHSPHY_CDR_CONTROL_CDR_ENABLE;
    Cy_SysLib_ExitCriticalSection(intState);

    return;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_DevInitiatedL1Exit
****************************************************************************//**
*
* Function helps device to initiate resume from L1 Sleep Device state.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param duration
* LPM timing parameter as per Specification. Default value 1500.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_DevInitiatedL1Exit (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                        uint32_t duration)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    /*
     * First update PHY for common resume related functionality then
     * send signal to host for resume.
     */
    Cy_USBHS_Cal_HsHandleL1WakeupCommon(pCalCtxt);

    pCalBase->DEV_LPM_TIM_1 =  duration;
    pCalBase->DEV_PWR_CS |=  USBHSDEV_DEV_PWR_CS_SIGRSUME;
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_DevInitiatedResumeL2Sleep
****************************************************************************//**
*
* Function helps device to initiate resume from L2 SleepDevice.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param duration
* LPM timing parameter as per Specification.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_DevInitiatedResumeL2Sleep (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                         uint32_t duration)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    pCalBase->DEV_LPM_TIM_1 =  duration;
    pCalBase->DEV_PWR_CS |=  USBHSDEV_DEV_PWR_CS_SIGRSUME;
    return;
}   /* end of function () */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_HsHandleL2SuspendEntry
****************************************************************************//**
*
* This function Handles Suspend at CAL layer in High speed mode.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param keepPllOn
* Parameter which indicates whether PLL should be left enabled.
*
* \return
* True if suspend entry is initiated
* False if suspend entry is skipped
*
*******************************************************************************/
bool
Cy_USBHS_Cal_HsHandleL2SuspendEntry (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                     bool keepPllOn)
{
    bool entryStatus = false;
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    /* Enter suspend only if device is still in L2 suspend state. */
    if (pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L2_SUSPEND_Msk)
    {
        DBG_HSCAL_TRACE("Cy_USBHS_Cal_HsHandleL2SuspendEntry >>\r\n");

        /* Need to write 0x2 in 23:22 bit of SPARE register. */
        pPhyBase->SPARE = (pPhyBase->SPARE & (~(0x00C00000UL))) | 0x00800000UL;

        pCalBase->DEV_PWR_CS |= (USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);

        pPhyBase->CDR_CONTROL &= (~USBHSPHY_CDR_CONTROL_CDR_ENABLE);

        /* Turn off PLL and supplies if they are not required. */
        if (!keepPllOn) {
            pPhyBase->AFE_CONTROL_1 &=
                (~USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD);

            pPhyBase->PLL_CONTROL_1 &= (~USBHSPHY_PLL_CONTROL_1_PLL_EN);
            pPhyBase->PLL_CONTROL_1 &= (~USBHSPHY_PLL_CONTROL_1_SUPPLY_EN);

            pPhyBase->REG_1P1_CONTROL &= (~USBHSPHY_REG_1P1_CONTROL_ENABLE_LV);

            /* regulator bypass to use 3.3v */
            pPhyBase->REG_2P5_CONTROL |= USBHSPHY_REG_2P5_CONTROL_BYPASS_MODE;
            pPhyBase->REG_2P5_CONTROL &= (~USBHSPHY_REG_2P5_CONTROL_ENABLE_LV);

            pPhyBase->VREFGEN_CONTROL &= (~USBHSPHY_VREFGEN_CONTROL_ENABLE_LV);
            pPhyBase->IREFGEN_CONTROL &= (~USBHSPHY_IREFGEN_CONTROL_ENABLE_LV);
        }

        entryStatus = true;
        DBG_HSCAL_TRACE("Cy_USBHS_Cal_HsHandleL2SuspendEntry <<\r\n");
    } else {
        DBG_HSCAL_INFO("Device already out of suspend. Skipping.\r\n");
    }
    return entryStatus;
}   /* end of function() */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_FsHandleSuspend
****************************************************************************//**
*
* L1-Wakeup request during device in full speed mode will be handled here.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_FsHandleSuspend (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    (void)pPhyBase;
    (void)pCalBase;
    return;
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_HsHandleL2Resume
****************************************************************************//**
*
* This function Handles resume at CAL layer in high speed mode.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_HsHandleL2Resume (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    USBHSPHY_Type  *pPhyBase;

    pCalBase = pCalCtxt->pCalBase;
    pPhyBase = pCalCtxt->pPhyBase;

    DBG_HSCAL_TRACE("Cy_USBHS_Cal_HsHandleL2Resume >>\r\n");

    if ((pPhyBase->AFE_CONTROL_1 & USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD) != 0) {
        DBG_HSCAL_INFO("HandleL2Resume: HS_VCCD enabled, skip to CDR enable\r\n");
    } else {
        /* Clear any stale interrupts. */
        pPhyBase->INTR0 = pPhyBase->INTR0;

        /* Current reference initialization */
        pPhyBase->IREFGEN_CONTROL |= USBHSPHY_IREFGEN_CONTROL_ENABLE_LV;
        /* Voltage reference initialization */
        pPhyBase->VREFGEN_CONTROL |= USBHSPHY_VREFGEN_CONTROL_ENABLE_LV;
        /* wait for 20us  */
        Cy_SysLib_DelayUs(20);

        /* Enable 2.5 V supply and remove bypass connection to 3.3V supply. */
        pPhyBase->REG_2P5_CONTROL |= (USBHSPHY_REG_2P5_CONTROL_ENABLE_LV);
        pPhyBase->REG_2P5_CONTROL &= (~USBHSPHY_REG_2P5_CONTROL_BYPASS_MODE);

        /* wait for VCCD interrupt and then clear the interrupt*/
        while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_VCCD)) {
            Cy_SysLib_DelayUs(1);
            DBG_HSCAL_TRACE("Inside ENABLE_VCCD loop. \r\n");
        }
        pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_VCCD);

        pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_ENABLE_LV;
        Cy_SysLib_DelayUs(10);
        pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_VCCD;
        Cy_SysLib_DelayUs(40);

        /* wait for VCCD interrupt and then clear the interrupt*/
        while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_HS_VCCD)) {
            Cy_SysLib_DelayUs(1);
            DBG_HSCAL_TRACE("Inside ENABLE_HS_VCCD loop. \r\n");
        }
        pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_HS_VCCD);

        Cy_USBHS_Cal_UpdateFx2g3PllSettings(pPhyBase);

        pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
        Cy_SysLib_DelayUs(10);
        pPhyBase->PLL_CONTROL_2 |= USBHSPHY_PLL_CONTROL_2_SOURCE_OF_PLL_LOCK_Msk;
        pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;

        /* wait for PLL_LOCK interrupt and then clear the interrupt*/
        while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK))) {
            Cy_SysLib_DelayUs(1);
            DBG_HSCAL_TRACE("Inside PLL_LOCK loop. \r\n");
        }
        pPhyBase->INTR0  |= USBHSPHY_INTR0_PLL_LOCK;

        /* Enable the HS_VCCD supply. */
        pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD;
    }

    Cy_SysLib_DelayUs(1);

    /* Enable CDR functionality. */
    pPhyBase->CDR_CONTROL |=  USBHSPHY_CDR_CONTROL_CDR_ENABLE;

    /* Handle L2 PHY sequence for HS and then disable suspend bit. */
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);

    /* Need to write 0x0 in 23:22 bit of SPARE register. */
    pPhyBase->SPARE &= 0xFF3FFFFFUL;
    DBG_HSCAL_TRACE("Cy_USBHS_Cal_HsHandleL2Resume <<\r\n");
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_HostInitiateL2ResumeWithdDeepSleep
****************************************************************************//**
*
* This function Handles Host initiated resume when device in deep-sleep.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_HostInitiateL2ResumeWithdDeepSleep (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    Cy_USBHS_Cal_HsHandleL2Resume(pCalCtxt);
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_FsHandleResume
****************************************************************************//**
*
* This function Handles resume at CAL layer in full speed mode.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_FsHandleResume (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;
    (void)pCalBase;

    return;
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_IntrHandler
****************************************************************************//**
*
* This function Handles All interrupt related to controller. Based on interrupt
* required function will be invoked.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* true if yield is pending when the ISR returns, false otherwise.
*
*******************************************************************************/
bool
Cy_USBHS_Cal_IntrHandler (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    cy_stc_usb_cal_msg_t msg = {CY_USB_CAL_MSG_INVALID, {0, 0}};
    uint32_t activeIntr;
    bool yield_reqd = false;
    volatile uint32_t count;

    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    /* get active interrupt list and address them one by one */
    activeIntr = ((pCalBase->DEV_CTL_INTR) & (pCalBase->DEV_CTL_INTR_MASK));

    if (activeIntr != 0) {
        /* clear interrupt and handle */

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SUSP) {
            /* When Host Suspend the Bus. Handle suspend mode */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SUSP;
            msg.type = CY_USB_CAL_MSG_SUSP;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_SUSP);
            DBG_HSCAL_INFO("SUSPEND INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_URESET) {
            /* When Host initiates USB reset. Handle start of reset */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_URESET;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_URESET;

            /* Disable LPM support by default. */
            pCalBase->POWER &= ~USBHSDEV_POWER_LPM_ENABLE_Msk;

            msg.type = CY_USB_CAL_MSG_RESET;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_URESET);
            DBG_HSCAL_INFO("RESET INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_HSGRANT) {
            /* When Host Grant high speed communication. Handle HS mode */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_HSGRANT;

            /* Enable LPM in case of High-Speed operation. */
            pCalBase->POWER |= USBHSDEV_POWER_LPM_ENABLE_Msk;

            msg.type = CY_USB_CAL_MSG_HSGRANT;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_HSGRANT);
            DBG_HSCAL_INFO("HSGNT INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SUDAV) {
            /* Handle setup token with data */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SUDAV;
            msg.type = CY_USB_CAL_MSG_SUDAV;
            msg.data[0] = pCalBase->DEV_SETUPDAT_0;
            msg.data[1] = pCalBase->DEV_SETUPDAT_1;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_ERRLIMIT) {
            /* Handle Error Limit Interrupt */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_ERRLIMIT;
            /* fetch count value  and clear it by writting 0x00. */
            count = ((pCalBase->DEV_CS) & (USBHSDEV_DEV_CS_COUNT_Msk));
            count = (count >> (USBHSDEV_DEV_CS_COUNT_Pos));
            msg.data[0] = count;
            msg.type = CY_USB_CAL_MSG_ERRLIMIT;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CS &= (~(USBHSDEV_DEV_CS_COUNT_Msk));
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_ERRLIMIT);
            DBG_HSCAL_INFO("ERRLIMIT INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_URESUME) {
            /*
             * When host initiated USB UResume. K state more than 2.5us.
             * Handle resume signaling
             */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_URESUME;
            msg.type = CY_USB_CAL_MSG_RESUME_END;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_URESUME);
            DBG_HSCAL_INFO("URESUME INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_STATUS_STAGE) {
            /* Handle completion of status stage */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_STATUS_STAGE;
            msg.type = CY_USB_CAL_MSG_STATUS_STAGE;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_L1_SLEEP_REQ) {
            /* Handle L1 sleep request */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_L1_SLEEP_REQ;

            /* Send msg to USBD for device state related housekeeping */
            msg.type = CY_USB_CAL_MSG_L1_SLEEP;
            msg.data[0] = pCalBase->DEV_LPM_ATTR;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_L1_SLEEP_REQ);
            DBG_HSCAL_TRACE("L1_SLEEP INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_L1_URESUME) {
            /* Handle L1 resume */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_L1_URESUME;
            Cy_USBHS_Cal_HsHandleL1WakeupCommon(pCalCtxt);
            /* Send msg to USBD for device state related housekeeping */
            msg.type = CY_USB_CAL_MSG_L1_URESUME;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_L1_URESUME);
            DBG_HSCAL_TRACE("L1_RESUME INTR\r\n");
        }

        /* Handle end of reset done */
        if (activeIntr & USBHSDEV_DEV_CTL_INTR_RESETDONE) {
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_RESETDONE;

            /* Once reset is complete, enable TX pre-emphasis. */
            Cy_USBHS_Cal_PreemphasisControl(pCalCtxt, true);

            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_RESET_DONE;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_RESET_DONE);
            DBG_HSCAL_INFO("RESET DONE INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SETADDR) {
            /* Handle completion of set address  */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SETADDR;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_SETADDR;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_SET_ADDR);
            DBG_HSCAL_INFO("SET ADDR INTR\r\n");
        }

        /* Host initiated resume signal for 3usec. */
        if (activeIntr & USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED) {
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED;
            pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);
            msg.type = CY_USB_CAL_MSG_RESUME_START;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_HOST_URESUME_ARR);
            DBG_HSCAL_TRACE("HOST_URSUME_ARRIVED INTR\r\n");
        }

        /* Wakeup request due to K-State during host initiated Reesume. */
        if (activeIntr & USBHSDEV_DEV_CTL_INTR_DPSLP_Msk) {
            /* Need to wait for HOST_URESUME_INTR so just clear INTR */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_DPSLP_Msk;
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_DPSLP);
            DBG_HSCAL_INFO("DPSLP INTR\r\n");
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SOF) {
            /* Clear the interrupt. */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SOF;

            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type    = CY_USB_CAL_MSG_SOF_ITP;
            msg.data[0] = pCalBase->DEV_FRAMECNT;
            msg.data[1] = 0;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_PID_MISMATCH_ON_NAK) {
            /* Clear the interrupt. */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_PID_MISMATCH_ON_NAK;
            Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_PID_MISMATCH);
            DBG_HSCAL_INFO("PID Mismatch\r\n");
        }
    }

    /* Handling Endpoint interrupts  */
    volatile uint32_t actDevEpIntr;
    volatile uint32_t epIngrActIntr;
    volatile uint32_t dev_ep_cs;
    uint32_t endpNum;

    /* Check for EP_INGRS interrupts. */
    epIngrActIntr = ((pCalBase->DEV_EP_INGRS_INTR) & (pCalBase->DEV_EP_INGRS_INTR_MASK));
    endpNum = 0;
    while (epIngrActIntr != 0) {
        /* Check if ZLP interrupt is asserted for any OUT endpoint. */
        if ((epIngrActIntr & (1 << endpNum)) != 0) {
            /*
             * Disable the interrupt instead of clearing it.
             * Interrupt shall be cleared and re-enabled by the APP.
             * next set of data comes to controller only when zlp is
             * cleared so zlp should be cleared after it is handled
             * in app.
             */
            pCalBase->DEV_EP_INGRS_INTR_MASK &= ~(0x00000001UL << endpNum);
            msg.type = CY_USB_CAL_MSG_OUT_ZLP;
            msg.data[0] = endpNum;
            msg.data[1] = 0x00;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        /* Check if SLP interrupt is asserted for any OUT endpoint. */
        if ((epIngrActIntr & (1 << (endpNum + USBHSDEV_DEV_EP_INGRS_INTR_EP_INGRS_SLP_RCVD_Pos))) != 0) {

            /* Notify USBD for handling. */
            /*
             *  0-9 bit in the register gives size of slp.
             *  value 0x00 means 1 byte and 0x01 means 2 bytes so
             *  while sending message add 1.
             */
            msg.type    = CY_USB_CAL_MSG_OUT_SLP;
            msg.data[0] = endpNum;
            msg.data[1] = ((pCalBase->IEPM_ENDPOINT[endpNum]) & USBHSDEV_IEPM_ENDPOINT_INGRS_SLP_BYTE_COUNT_Msk) + 1;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);

            /* Clear the interrupt. Re-enable it for endpoints other than 0. */
            pCalBase->DEV_EP_INGRS_INTR = (0x00010000UL << endpNum);
        }

        epIngrActIntr &= ~(0x10001UL << endpNum);
        endpNum++;
    }

    /* Check for other EP interrupts. */
    actDevEpIntr = (pCalBase->DEV_EP_INTR & pCalBase->DEV_EP_INTR_MASK);

    /* Clear the interrupts we have read. */
    pCalBase->DEV_EP_INTR = actDevEpIntr;

    endpNum = 0;
    while (actDevEpIntr != 0) {
        /* Check for interrupt on IN endpoint. */
        if ((actDevEpIntr & (1UL << endpNum)) != 0) {
            dev_ep_cs =  pCalBase->DEV_EPI_CS[endpNum];

            /* Currently, we only need to handle the ZERO interrupt. */
            if ((dev_ep_cs & USBHSDEV_DEV_EPI_CS_ZERO) != 0) {
                msg.type = CY_USB_CAL_MSG_IN_ZLP;
                msg.data[0] = (CY_USBD_ENDP_DIR_MASK | endpNum);
                msg.data[1] = 0x00;
                yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Clear the interrupts that are active by writing 1 into the status bits. */
            pCalBase->DEV_EPI_CS[endpNum] = dev_ep_cs;

            actDevEpIntr &= ~(1UL << endpNum);
        }

        /* Check for interrupt on OUT endpoint. */
        if ((actDevEpIntr & (0x10000UL << endpNum)) != 0) {
            dev_ep_cs = pCalBase->DEV_EPO_CS[endpNum];

            /* Currently, we only need to handle the DONE interrupt. */
            if ((dev_ep_cs & USBHSDEV_DEV_EPO_CS_DONE_Msk) != 0) {
                /* Leave the DONE interrupt disabled until the message is handled by USBD. */
                dev_ep_cs &= ~USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk;

                msg.type = CY_USB_CAL_MSG_OUT_DONE;
                msg.data[0] = endpNum;
                msg.data[1] = 0x00;
                yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Clear the interrupts that are active by writing 1 into the status bits. */
            pCalBase->DEV_EPO_CS[endpNum] = dev_ep_cs;

            actDevEpIntr &= ~(0x10000UL << endpNum);
        }

        endpNum++;
    }

    return yield_reqd;
}   /* end of function cy_usbhs_intr_handler() */



/*******************************************************************************
* Function name: Cy_USBHS_Cal_EnableReqDevCtrlIntr
****************************************************************************//**
*
* This function will enable required interrupt under device ctrl reister.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableReqDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("HsHandleL2Resume:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CTL_INTR_MASK |=
        (USBHSDEV_DEV_CTL_INTR_MASK_SETADDR |
         USBHSDEV_DEV_CTL_INTR_MASK_SUSP |
         USBHSDEV_DEV_CTL_INTR_MASK_URESET |
         USBHSDEV_DEV_CTL_INTR_MASK_HSGRANT |
         USBHSDEV_DEV_CTL_INTR_MASK_SUDAV |
         USBHSDEV_DEV_CTL_INTR_MASK_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_SLEEP_REQ |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE |
         USBHSDEV_DEV_CTL_INTR_DPSLP_Msk |
         USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED |
         USBHSDEV_DEV_CTL_INTR_MASK_PID_MISMATCH_ON_NAK);

    if (pCalCtxt->enableSOFInterrupt) {
        pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_SOF;
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function()  */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_DisableAllDevCtrlIntr
****************************************************************************//**
*
* This function will disable all possible interrupt generated by USBHS block
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DisableAllDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("DisableAllDevCtrlIntr:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* All interrupts under dev_ctl_reg is disabled. */
    pCalBase->DEV_CTL_INTR_MASK &= ~(
            USBHSDEV_DEV_CTL_INTR_MASK_SETADDR |
            USBHSDEV_DEV_CTL_INTR_MASK_SOF |
            USBHSDEV_DEV_CTL_INTR_MASK_SUSP |
            USBHSDEV_DEV_CTL_INTR_MASK_URESET |
            USBHSDEV_DEV_CTL_INTR_MASK_HSGRANT |
            USBHSDEV_DEV_CTL_INTR_MASK_SUTOK |
            USBHSDEV_DEV_CTL_INTR_MASK_SUDAV |
            USBHSDEV_DEV_CTL_INTR_MASK_ERRLIMIT |
            USBHSDEV_DEV_CTL_INTR_MASK_URESUME |
            USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE |
            USBHSDEV_DEV_CTL_INTR_MASK_L1_SLEEP_REQ |
            USBHSDEV_DEV_CTL_INTR_MASK_L1_URESUME |
            USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE |
            USBHSDEV_DEV_CTL_INTR_DPSLP_Msk |
            USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED |
            USBHSDEV_DEV_CTL_INTR_MASK_PID_MISMATCH_ON_NAK);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_ClearAllDevCtrlIntr
****************************************************************************//**
*
* This function will clear all interrupt bit in device ctrl register.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ClearAllDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("ClearAllDevCtrlIntr:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CTL_INTR = 0xFFFFFFFF;
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_UpdateEpIntrMask
****************************************************************************//**
*
* This function updates the interrupt mask for an endpoint with the desired values.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param epIntrMask
* Endpoint interrupt mask.
*
* \param setClear
* Interrupt Mask bits to be set or cleared.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_UpdateEpIntrMask (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNum,
                               cy_en_usb_endp_dir_t endpDir,
                               uint32_t epIntrMask,
                               bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;
    uint32_t epCs;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("UpdateEpIntrMask:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        epCs = pCalCtxt->pCalBase->DEV_EPO_CS[endpNum];
        if (setClear) {
            pCalCtxt->pCalBase->DEV_EPO_CS[endpNum] = ((epCs & 0xFF01FFFFUL) | epIntrMask);
        } else {
            pCalCtxt->pCalBase->DEV_EPO_CS[endpNum] = (epCs & (0xFF01FFFFUL & ~epIntrMask));
        }
    } else {
        epCs = pCalCtxt->pCalBase->DEV_EPI_CS[endpNum];
        if (setClear) {
            pCalCtxt->pCalBase->DEV_EPI_CS[endpNum] = ((epCs & 0xFF01FFFFUL) | epIntrMask);
        } else {
            pCalCtxt->pCalBase->DEV_EPI_CS[endpNum] = (epCs & (0xFF01FFFFUL & ~epIntrMask));
        }
    }
    Cy_SysLib_ExitCriticalSection(intMask);

    return(CY_USB_CAL_STATUS_SUCCESS);
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask
****************************************************************************//**
*
* This function clears endpoint interrupt for ingress and egress intrrupt and
* enables respective mask register. Writing 1 in intr register will clear the
* interrupt. writting 1 in mask register will enable that register.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param zlpSlp
* 1 for ZLP and 0 for SLP.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNum,
                                        cy_en_usb_endp_dir_t endpDir,
                                        bool zlpSlp)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("ClearZlpSlpIntrEnableMask:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();

    /* zlp: 0-15 bits and slp: 16-31 bits for ingr and egrs register. */
    if (CY_USB_ENDP_DIR_IN == endpDir) {
        if (zlpSlp) {
            pCalBase->DEV_EP_EGRS_INTR = (0x01 << (endpNum));
            pCalBase->DEV_EP_EGRS_INTR_MASK |= (0x01 << (endpNum));
        } else {
            pCalBase->DEV_EP_EGRS_INTR =
                               (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
            pCalBase->DEV_EP_EGRS_INTR_MASK |=
                               (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
        }
    } else {
        if (zlpSlp) {
            pCalBase->DEV_EP_INGRS_INTR = 0x01 << (endpNum);
            pCalBase->DEV_EP_INGRS_INTR_MASK |= 0x01 << (endpNum);
        } else {
            pCalBase->DEV_EP_INGRS_INTR =
                                (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
            pCalBase->DEV_EP_INGRS_INTR_MASK |=
                                (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
        }
    }

    Cy_SysLib_ExitCriticalSection(intMask);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EnableCtrlSlpIntr
****************************************************************************//**
*
* This function enables the SLP_RCVD interrupt for EP0-OUT.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableCtrlSlpIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("EnableCtrlSlpIntr:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    pCalBase->DEV_EP_INGRS_INTR_MASK |= (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos);
    Cy_SysLib_ExitCriticalSection(intMask);

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_DisableCtrlSlpIntr
****************************************************************************//**
*
*  This function disable the SLP_RCVD interrupt for EP0-OUT.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DisableCtrlSlpIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("DisableCtrlSlpIntr:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    pCalBase->DEV_EP_INGRS_INTR_MASK &=
              (~(1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos));
    Cy_SysLib_ExitCriticalSection(intMask);

    return CY_USB_CAL_STATUS_SUCCESS;
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EnableEndp
****************************************************************************//**
*
* This function enables/disable endpoint and set/reset respective interrupt.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param enable
* 1 for enable and 0 for disable.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                         cy_en_usb_endp_dir_t endpDir,  bool enable)
{
    USBHSDEV_Type  *pCalBase;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("EnableEndp:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /*
     * Enable: set valid bit and enable interrupt for this endpoint.
     * Disable: clear valid bit and disable interrupt for this endpoint.
     */
    if ((CY_USB_ENDP_DIR_OUT == endpDir) || (0U == endpNum)) {
        if (enable) {
            pCalBase->DEV_EPO_CS[endpNum] |= USBHSDEV_DEV_EPO_CS_VALID;
            pCalBase->DEV_EP_INTR_MASK |=
                               (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
        } else {
            pCalBase->DEV_EPO_CS[endpNum] &= (~(USBHSDEV_DEV_EPO_CS_VALID));
            pCalBase->DEV_EP_INTR_MASK &=
                            (~(0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER)));
        }
    }

    if ((CY_USB_ENDP_DIR_IN == endpDir) || (0U == endpNum)) {
        if (enable) {
            pCalBase->DEV_EPI_CS[endpNum] |= USBHSDEV_DEV_EPI_CS_VALID;
        } else {
            pCalBase->DEV_EPI_CS[endpNum] &= (~(USBHSDEV_DEV_EPI_CS_VALID));
        }
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBHS_Cal_EnableEndp () */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EndpConfig
****************************************************************************//**
*
* This function handles configuration of endpoint.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param configParam
* Contains all config parameters.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpConfig (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                         cy_stc_usb_endp_config_t configParam)
{
    cy_en_usb_endp_dir_t endpDir = configParam.endpDirection;
    cy_en_usb_endp_type_t endpType = configParam.endpType;
    uint32_t endpNum = configParam.endpNumber;
    uint32_t maxPktSize = configParam.maxPktSize;
    uint32_t dev_ep_cs = 0x00;
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("EndpConfig:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* check all possibility of wrong input parameter. */
    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpType >= CY_USB_ENDP_TYPE_INVALID) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {

        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if ((CY_USB_ENDP_DIR_OUT == endpDir) || (0U == endpNum)) {
        /* Start by disabling the endpoint. */
        pCalBase->DEV_EPO_CS[endpNum] &= ~USBHSDEV_DEV_EPO_CS_VALID;

        dev_ep_cs = pCalBase->DEV_EPO_CS[endpNum];
        /* first clear payload related bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk);
        dev_ep_cs |= ((maxPktSize & USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk) <<
                                        (USBHSDEV_DEV_EPI_CS_PAYLOAD_Pos));
        /* First clear type bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_TYPE_Msk);
        dev_ep_cs |= ((endpType & 0x03) << USBHSDEV_DEV_EPO_CS_TYPE_Pos);

        /* For non ISO, two bits are 0 so don't need else condition. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_ISOINPKS_Msk);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            /* Need to set value in case of ISO */
            dev_ep_cs |= ((configParam.isoPkts & 0x03) <<
                               (USBHSDEV_DEV_EPO_CS_ISOINPKS_Pos));
        }
        pCalBase->DEV_EPO_CS[endpNum] = dev_ep_cs;

        if (configParam.allowNakTillDmaRdy) {
            pCalBase->IEPM_ENDPOINT[endpNum] |=
                         USBHSDEV_IEPM_ENDPOINT_ALLOW_NAK_TILL_DMA_RDY_Msk;
        } else {
            pCalBase->IEPM_ENDPOINT[endpNum] &=
                         (~USBHSDEV_IEPM_ENDPOINT_ALLOW_NAK_TILL_DMA_RDY_Msk);
        }

        if (configParam.valid) {
            if (endpNum != 0x00) {
                /* These configurations are not for endpoint 0 */
                pCalBase->DEV_EP_INTR_MASK |=
                               (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));
                /* 0-15 for ZLP and 16-31 for SLP interrupt. */
                pCalBase->DEV_EP_INGRS_INTR_MASK |= (0x01 << (endpNum));
                pCalBase->DEV_EP_INGRS_INTR_MASK |=
                               (0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER));

                /* Enable the DONE interrupt for the endpoint if the allowNakTillDmaRdy setting is enabled. */
                if (configParam.allowNakTillDmaRdy) {
                    pCalBase->DEV_EPO_CS[endpNum] |= USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk;
                }
            }

            /* Enable the endpoint after other configurations have been set. */
            pCalBase->DEV_EPO_CS[endpNum] |= USBHSDEV_DEV_EPO_CS_VALID;
        } else {
            /* Disable the endpoint first and then clear interrupt settings. */
            pCalBase->DEV_EPO_CS[endpNum] &= (~(USBHSDEV_DEV_EPO_CS_VALID));

            if (endpNum != 0x00) {
                /* These configurations are not for endpoint 0 */
                pCalBase->DEV_EPO_CS[endpNum] &= ~(USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk);

                pCalBase->DEV_EP_INTR_MASK &=
                            (~(0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER)));
                /* 0-15 for ZLP and 16-31 for SLP interrupt. */
                pCalBase->DEV_EP_INGRS_INTR_MASK &= (~(0x01 << (endpNum)));
                pCalBase->DEV_EP_INGRS_INTR_MASK &=
                            (~(0x01 << (endpNum + CY_USB_MAX_ENDP_NUMBER)));
            }
        }
    }

    if ((CY_USB_ENDP_DIR_IN == endpDir) || (0U == endpNum)) {
        /* Start by disabling the endpoint. */
        pCalBase->DEV_EPI_CS[endpNum] &= ~USBHSDEV_DEV_EPI_CS_VALID;

        /* Direction is IN */
        dev_ep_cs = pCalBase->DEV_EPI_CS[endpNum];
        /* first clear payload related bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk);
        dev_ep_cs |= ((maxPktSize & USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk) <<
                                        (USBHSDEV_DEV_EPI_CS_PAYLOAD_Pos));
        /* First clear type bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_TYPE_Msk);
        dev_ep_cs |= ((endpType & 0x03) << USBHSDEV_DEV_EPO_CS_TYPE_Pos);

        /* For non ISO, two bits are 0 so don't need else condition. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_ISOINPKS_Msk);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            /* Need to set value in case of ISO */
            dev_ep_cs |= ((configParam.isoPkts & 0x03) <<
                               (USBHSDEV_DEV_EPO_CS_ISOINPKS_Pos));
        }
        pCalBase->DEV_EPI_CS[endpNum] = dev_ep_cs;

        /* if endp need to enable then interrupt also need to be enabled. */
        if (configParam.valid) {
            pCalBase->DEV_EPI_CS[endpNum] |= USBHSDEV_DEV_EPI_CS_VALID;
            if (endpNum != 0x00) {
                /* These configurations are not for endpoint 0 */
                pCalBase->DEV_EP_INTR_MASK |= (0x01 << (endpNum));
                pCalBase->DEV_EPI_CS[endpNum] |= USBHSDEV_DEV_EPI_CS_ZERO_MASK;
            }
        } else {
            pCalBase->DEV_EPI_CS[endpNum] &= (~(USBHSDEV_DEV_EPI_CS_VALID));
            if (endpNum != 0x00) {
                /* These configurations are not for endpoint 0 */
                pCalBase->DEV_EP_INTR_MASK &= (~(0x01 << (endpNum)));
                pCalBase->DEV_EPI_CS[endpNum] &=
                                            (~(USBHSDEV_DEV_EPI_CS_ZERO_MASK));
            }
        }
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_UpdateXferCount
****************************************************************************//**
*
* This function updates register with transfer count.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param xferCount
* transfer count.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_UpdateXferCount (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum,
                              cy_en_usb_endp_dir_t endpDir,
                              uint32_t xferCount)
{
    USBHSDEV_Type  *pCalBase;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("UpdateXferCount:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        pCalBase->DEV_EPO_XFER_CNT[endpNum] = xferCount;
    } else {
        pCalBase->DEV_EPI_XFER_CNT[endpNum] = xferCount;
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_GetXferCount
****************************************************************************//**
*
* Retrieve the remaining transfer count on the specified endpoint.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param pCntPending
* Return parameter through which pending count is returned.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_GetXferCount (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                           uint32_t endpNum,
                           cy_en_usb_endp_dir_t endpDir,
                           uint32_t *pCntPending)
{
    USBHSDEV_Type  *pCalBase;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID) ||
        (pCntPending == NULL)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("UpdateXferCount:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        *pCntPending = pCalBase->DEV_EPO_XFER_CNT[endpNum];
    } else {
        *pCntPending = pCalBase->DEV_EPI_XFER_CNT[endpNum];
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EndpSetClearNak
****************************************************************************//**
*
* This function enable or disable NAK condition in hw. By setting NAK bit,
* endpoint will keep sending NAK till NAK bit is cleared.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* 1 for set and 0 for clear.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpSetClearNak (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum,
                              cy_en_usb_endp_dir_t endpDir,
                              bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("EndpSetClearNak:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
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
                pCalBase->DEV_EPI_CS[0] |= USBHSDEV_DEV_EPI_CS_NAK;
                pCalBase->DEV_EPO_CS[0] |= USBHSDEV_DEV_EPO_CS_NAK;
            } else {
                pCalBase->DEV_EPI_CS[0] &= (~USBHSDEV_DEV_EPI_CS_NAK);
                pCalBase->DEV_EPO_CS[0] &= (~USBHSDEV_DEV_EPO_CS_NAK);
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
                    pCalBase->DEV_EPO_CS[endpNum] |=
                                                USBHSDEV_DEV_EPO_CS_NAK;
                } else {
                    pCalBase->DEV_EPO_CS[endpNum] &=
                                              (~USBHSDEV_DEV_EPO_CS_NAK);
                }
            } else {
                /* Handle IN endpoint. */
                if (setClear) {
                    pCalBase->DEV_EPI_CS[endpNum] |=
                                                USBHSDEV_DEV_EPI_CS_NAK;
                } else {
                    pCalBase->DEV_EPI_CS[endpNum] &=
                                              (~USBHSDEV_DEV_EPI_CS_NAK);
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
* Function name: Cy_USBHS_Cal_SetClearNakAll
****************************************************************************//**
*
* This function either set or clear NAK for all endpoint.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param setClear
* 1 for set and 0 for clear.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetClearNakAll (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("SetClearNakAll:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (setClear) {
        /* Set CLR bit and set NAKALL bit */
        pCalBase->DEV_CS =
             ((pCalBase->DEV_CS & (~USBHSDEV_DEV_CS_SETUP_CLR_BUSY)) |
              (USBHSDEV_DEV_CS_NAKALL));
    } else {
        pCalBase->DEV_CS &=
                (~(USBHSDEV_DEV_CS_SETUP_CLR_BUSY | USBHSDEV_DEV_CS_NAKALL));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EndpIsNakNrdySet
****************************************************************************//**
*
* This function checks endpoint's status related to NAK bit is set or reset.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* 1 if NAK bit is set.
* 0 if NAK bit is reset.
*
*******************************************************************************/
bool Cy_USBHS_Cal_EndpIsNakNrdySet (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                    uint32_t endpNum,
                                    cy_en_usb_endp_dir_t endpDir)
{
    USBHSDEV_Type *pCalBase;
    bool endpNaked = false;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpNaked);
    }

    if ((pCalCtxt != NULL) && (pCalCtxt->pCalBase != NULL)) {
        pCalBase = pCalCtxt->pCalBase;

        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            if ((pCalBase->DEV_EPO_CS[endpNum] & USBHSDEV_DEV_EPO_CS_NAK) ||
                (pCalBase->DEV_CS & USBHSDEV_DEV_CS_NAKALL)) {
                endpNaked = true;
            }
        } else {
            if ((pCalBase->DEV_EPI_CS[endpNum] & USBHSDEV_DEV_EPI_CS_NAK) ||
                (pCalBase->DEV_CS & USBHSDEV_DEV_CS_NAKALL)) {
                endpNaked = true;
            }
        }
    }
    return endpNaked;
}   /* End of function() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EndpSetClearStall
****************************************************************************//**
*
* This function enable or disable STALL condition for the specified endpoint
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* 1 for set and 0 for clear.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpSetClearStall (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                uint32_t endpNum,
                                cy_en_usb_endp_dir_t endpDir,
                                bool setClear)
{
    uint32_t devToggle;
    uint32_t tempCounter = 0x00;
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("EndpSetClearStall:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
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
                /*
                 * To send STALL for endp0, CS_STALL bit and SetupClearBusy
                 * bit need to set.
                 * Setting SetupClearBusy will leads to stopping NAK and send
                 * required response to Host.
                 */
                pCalBase->DEV_EPI_CS[0] |= USBHSDEV_DEV_EPI_CS_STALL;
                pCalBase->DEV_EPO_CS[0] |= USBHSDEV_DEV_EPO_CS_STALL;
                pCalBase->DEV_CS |= (USBHSDEV_DEV_CS_SETUP_CLR_BUSY);

            } else {
                pCalBase->DEV_EPI_CS[0] &= (~USBHSDEV_DEV_EPI_CS_STALL);
                pCalBase->DEV_EPO_CS[0] &= (~USBHSDEV_DEV_EPO_CS_STALL);
            }
            break;

        /* case fall through 1 to 16 */
        case 1:   case 2:   case 3:
        case 4:   case 5:   case 6:
        case 7:   case 8:   case 9:
        case 10:  case 11:  case 12:
        case 13:  case 14:  case 15:
            if (CY_USB_ENDP_DIR_OUT == endpDir) {
                /* Handle OUT endpoint. */
                if (setClear) {
                    pCalBase->DEV_EPO_CS[endpNum] |=
                                                USBHSDEV_DEV_EPO_CS_STALL;
                } else {
                    pCalBase->DEV_EPO_CS[endpNum] &=
                                              (~USBHSDEV_DEV_EPO_CS_STALL);
                }
            } else {
                /* Handle IN endpoint. */
                if (setClear) {
                    pCalBase->DEV_EPI_CS[endpNum] |=
                                                USBHSDEV_DEV_EPI_CS_STALL;
                } else {
                    pCalBase->DEV_EPI_CS[endpNum] &=
                                              (~USBHSDEV_DEV_EPI_CS_STALL);
                }
            }
            break;

        default:
            retCode = CY_USB_CAL_STATUS_BAD_PARAM;
            break;
    }

    /*
     * During STALL need take care toggle big and To change the value of the
     * toggle bit, s/w must first write ENDPOINT IO, then poll for VALID=1,
     * then write to R/S, then poll again for VALID=1.
     */
    if ((!setClear) && (endpNum != 0) &&
        (CY_USB_CAL_STATUS_SUCCESS == retCode)) {

        /*
         * clearing stall so take care of toggle bit here as per info
         * given in  DEV_TOGGLE.
         */
        devToggle = (endpNum & USBHSDEV_DEV_TOGGLE_ENDPOINT_Msk);

        if (CY_USB_ENDP_DIR_OUT == endpDir) {
            devToggle &= (~USBHSDEV_DEV_TOGGLE_IO);
        } else {
            devToggle |= (USBHSDEV_DEV_TOGGLE_IO);
        }
        pCalBase->DEV_TOGGLE = devToggle;
        /* now poll for valid bit. */
        do {
            /*
             * wait till while VALID = 1. if time limit is reached then
             * return error.
             */
            tempCounter++;
            if (tempCounter == 0x0000FFFF) {
                break;
            }
        } while (!(pCalBase->DEV_TOGGLE & USBHSDEV_DEV_TOGGLE_TOGGLE_VALID));

        if (tempCounter == 0x0000FFFF) {
            return(CY_USB_CAL_STATUS_TOGGLE_FAILED);
        }

        /*
         * control comes here means valid is 1 so Write R bit "1"
         * to reset Data toggle.
         */
        pCalBase->DEV_TOGGLE = ((pCalBase->DEV_TOGGLE & ~USBHSDEV_DEV_TOGGLE_TOGGLE_VALID) | USBHSDEV_DEV_TOGGLE_R);
        tempCounter = 0x00;
        do {
            /*
             * wait till while VALID = 1. if time limit is reached then
             * return error.
             */
            tempCounter++;
            if (tempCounter == 0x0000FFFF) {
                break;
            }
        } while (!(pCalBase->DEV_TOGGLE & USBHSDEV_DEV_TOGGLE_TOGGLE_VALID));

        if (tempCounter == 0x0000FFFF) {
            return(CY_USB_CAL_STATUS_TOGGLE_FAILED);
        }
    }
    return(retCode);
}   /* end of function     */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_EndpIsStallSet
****************************************************************************//**
*
* This function checks whether the specified endpoint is currently in the STALLed
* state.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* 1 if STALL bit is set.
* 0 if STALL bit is reset.
*
*******************************************************************************/
bool Cy_USBHS_Cal_EndpIsStallSet (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                  uint32_t endpNum,
                                  cy_en_usb_endp_dir_t endpDir)
{
    USBHSDEV_Type *pCalBase;
    bool endpStalled = false;

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpStalled);
    }

    if ((pCalCtxt != NULL) && (pCalCtxt->pCalBase != NULL)) {
        pCalBase = pCalCtxt->pCalBase;

        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            endpStalled =
            (bool)((pCalBase->DEV_EPO_CS[endpNum] & USBHSDEV_DEV_EPO_CS_STALL) != 0);
        } else {
            endpStalled =
            (bool)((pCalBase->DEV_EPI_CS[endpNum] & USBHSDEV_DEV_EPI_CS_STALL) != 0);
        }
    } else {
        DBG_HSCAL_ERR("EndpIsStallSet:pCalBase OR PcalCtxt NULL\r\n");
    }

    return endpStalled;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_FlushEndp
****************************************************************************//**
*
* This function will flush data available in perticular endpoint FIFO. High speed
* USB IP supports flush functionality only for IN endpoint.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_FlushEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                       uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("FlushEndp:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if ((endpNum >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDir >= CY_USB_ENDP_DIR_INVALID)) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDir) {
        /*
         * Ingress endp ie OUT endp flushing is not allowed so nothing
         * required here.
         */
    } else {
        pCalBase->EEPM_ENDPOINT[endpNum] |=
                                        (USBHSDEV_EEPM_ENDPOINT_EGRS_FLUSH_EP);
        /* wait for ms as per hw recommentation. */
        pCalBase->EEPM_ENDPOINT[endpNum] &=
                                     (~(USBHSDEV_EEPM_ENDPOINT_EGRS_FLUSH_EP));
        pCalBase->DEV_EP_INTR_MASK |= (0x01 << (endpNum));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_FlushAllEndp
****************************************************************************//**
*
* This function will flush data available in all Ingress and egress endpoint FIFO.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_FlushAllEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("FlushAllEndp:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->EPM_CS |= (USBHSDEV_EPM_CS_EGRS_FORCE_FLUSH_ALL);
    pCalBase->EPM_CS |= (USBHSDEV_EPM_CS_IGRS_FORCE_FLUSH_ALL);
    Cy_SysLib_DelayUs(1);
    pCalBase->EPM_CS &= (~(USBHSDEV_EPM_CS_EGRS_FORCE_FLUSH_ALL));
    pCalBase->EPM_CS &= (~(USBHSDEV_EPM_CS_IGRS_FORCE_FLUSH_ALL));

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SendEgressZLP
****************************************************************************//**
*
* This function triggers sending of a ZLP on an Egress endpoint.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SendEgressZLP (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                            uint32_t endpNum)
{
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode = CY_USB_CAL_STATUS_SUCCESS;

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pCalBase)) {
        DBG_HSCAL_ERR("SendEgressZlp:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (endpNum >= CY_USB_MAX_ENDP_NUMBER) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    /* We can set the ZLP request bit immediately. EPM takes care of sending ZLP only after any data that is
     * in the buffer is gone. Application will need to ensure that it waits for ZLP completion before next
     * packet is queued.
     */
    pCalBase->DEV_EP_EGRS_REQ = (1UL << endpNum);

    return retCode;
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_HandleCtrlOutSlp
****************************************************************************//**
*
* This function clears SLP ingress interrupt.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param endpNum
* Endpoint number.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_HandleCtrlOutSlp (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("HandleCtrlOutSlp:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (0 != ((pCalBase->DEV_EP_INGRS_INTR) & (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos))) {
        pCalBase->DEV_EP_INGRS_INTR = (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos);
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_CtrlEndp0DataOutAck
****************************************************************************//**
*
* Data stage ACK for endpoint0 is controlled by this function. In USBHS,
* when "CONT_TO_DATA" bit is set then controller wont send ACK till data is
* validated.
*
* \param pCalCtxt
* CAL layer context pointer.
*
*  \param setClear
* 1 for set and 0 for clear.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_CtrlEndp0DataOutAck (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    volatile uint32_t devCs;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("CtrlEndp0DataOutAck:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /*
     * Writting '0' does not have any affect on  SETUP_CLR_BUST
     * bit but writting 1 will clear the bit.
     */
    devCs = pCalBase->DEV_CS;
    if (setClear) {
        devCs = devCs & (~USBHSDEV_DEV_CS_SETUP_CLR_BUSY);
        devCs = devCs | (USBHSDEV_DEV_CS_CONT_TO_DATA_Msk);
    } else {
        devCs = devCs & (~USBHSDEV_DEV_CS_SETUP_CLR_BUSY);
        devCs = devCs & (~USBHSDEV_DEV_CS_CONT_TO_DATA_Msk);
    }

    pCalBase->DEV_CS = devCs;
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SendAckSetupDataStatusStage
****************************************************************************//**
*
* This function update register so that device will send ACK to complete control
* transfer.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SendAckSetupDataStatusStage (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("SendAck:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CS |= (USBHSDEV_DEV_CS_SETUP_CLR_BUSY);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_IsNewCtrlRqtReceived
****************************************************************************//**
*
* This function uses the SETUP_CLR_BUSY to check whether a new control request
* has been received while a previous send/recv data call is pending.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* true if a new request has been received, false otherwise.
*
*******************************************************************************/
bool
Cy_USBHS_Cal_IsNewCtrlRqtReceived (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    if (pCalCtxt != NULL) {
        pCalBase = pCalCtxt->pCalBase;
        if (pCalBase != NULL) {
            if ((pCalBase->DEV_CS & USBHSDEV_DEV_CS_SETUP_CLR_BUSY) != 0) {
                return true;
            }
        }
    }

    return false;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_GetRemoteWakeupStatus
****************************************************************************//**
*
* This function returns status of L2 remote wakeup in HW.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* true if L2 remote wakeup is enabled.
* false if L2 remote wakeup is disabled.
*
*******************************************************************************/
bool
Cy_USBHS_Cal_GetRemoteWakeupStatus (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("GetRemoteWakeupStatus:pCalBase NULL\r\n");
        return(FALSE);
    }

    /* Check whether remote wakeup is allowed. */
    if ((pCalBase->DEV_LPM_ATTR & USBHSDEV_DEV_LPM_ATTR_L2_SUSP_RMT_WAKEUP_EN) != 0) {
        return(TRUE);
    } else {
        return(FALSE);
    }
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_GetL1RemoteWakeupStatus
****************************************************************************//**
*
* This function returns status of L1 remote wakeup in HW.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* true if L1 remote wakeup is enabled.
* false if L1 remote wakeup is disabled.
*
*******************************************************************************/
bool
Cy_USBHS_Cal_GetL1RemoteWakeupStatus (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("GetRemoteWakeupStatusL1:pCalBase NULL\r\n");
        return(FALSE);
    }

    /* Check whether remote wakeup is allowed. */
    if ((pCalBase->DEV_LPM_ATTR & USBHSDEV_DEV_LPM_ATTR_RMT_WAKEUP_ENABLE) != 0) {
        return(TRUE);
    } else {
        return(FALSE);
    }
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SignalRemotWakup
****************************************************************************//**
*
* This function will update register so that device will initiate remote wakeup
* signaling, ie. it will try to comeout from L2_SUSPEND.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param startEndSignal
* Input to tell either start or end signaling.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/

/*
 * It is callers responsibility to maintain 15ms time between Signal
 * start and signal end.
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SignalRemotWakup (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                               bool startEndSignal)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("SignalRmoteWakeup:pCalbase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* Device initiated L2 SUSPEND exit sequence. */
    if (startEndSignal) {
        pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND);
        pCalBase->DEV_LPM_TIM_1 =  0x33450;
        pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_SIGRSUME;
    } else {
        pCalBase->DEV_PWR_CS &= ~USBHSDEV_DEV_PWR_CS_SIGRSUME;
    }

    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_START_REMOTE_WAKE_SGNAL);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SetTestMode
****************************************************************************//**
*
* This function will update register to setup required test mode.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param testMode
* perticular test mode.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetTestMode (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              cy_en_usbhs_cal_test_mode_t testMode)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("SetTestMode:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if ((testMode == CY_USBHS_CAL_TEST_MODE_TEST_J) || (testMode == CY_USBHS_CAL_TEST_MODE_TEST_K)) {
        /* Disable pre-emphasis in TEST-J and TEST-K modes. */
        Cy_USBHS_Cal_PreemphasisControl(pCalCtxt, false);
    } else {
        /* Enable pre-emphasis in other modes. */
        Cy_USBHS_Cal_PreemphasisControl(pCalCtxt, true);
    }

    pCalBase->DEV_CS = ((pCalBase->DEV_CS & ~USBHSDEV_DEV_CS_TEST_MODE_Msk) |
        (testMode << USBHSDEV_DEV_CS_TEST_MODE_Pos));
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_LpmSetClearNYET
****************************************************************************//**
*
* This function either set or clear NYET bit for LPM response.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param setClear
* 1 for set and 0 for clear.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_LpmSetClearNYET (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("LpmSetClearNYET:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (setClear) {
        /* Set NYET bit to block LPM-L1 entry. */
        pCalBase->DEV_LPM_ATTR |= (USBHSDEV_DEV_LPM_ATTR_NYET);
    } else {
        /* Clear NYET bit to enable LPM-L1 entry. */
        pCalBase->DEV_LPM_ATTR &= (~USBHSDEV_DEV_LPM_ATTR_NYET);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SetControllerSpeed
****************************************************************************//**
*
* This function set controller speed to FS or HS. If explicitly FS is not
* mentioned then function will set speed to HS.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param speed
* iether HS or FS.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetControllerSpeed (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                 cy_en_usb_speed_t speed)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("SetControllerSpeed:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (CY_USBD_USB_DEV_FS == speed) {
        pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_FORCE_FS;
    } else {
        pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_FORCE_FS);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function. */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_GetDevAddress
****************************************************************************//**
*
* This function will get device address assigned by Host.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param pDevAddr
* Address of device will be stored here.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_BAD_PARAM if direction and endpoint number invalid.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_GetDevAddress (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint8_t *pDevAddr)
{

    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        DBG_HSCAL_ERR("GetDevAddress:pCalBase NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (pDevAddr == NULL) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    *pDevAddr = (uint8_t)((pCalBase->DEV_CS & USBHSDEV_DEV_CS_DEVICEADDR_Msk)
                          >> (USBHSDEV_DEV_CS_DEVICEADDR_Pos));
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function Cy_USBHS_Cal_GetDevAddress() */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_GetLinkActive
****************************************************************************//**
*
* This function makes sure that the USB 2.x link gets into the L0 state if it
* is in L1.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_GetLinkActive (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    DBG_HSCAL_TRACE("Cy_USBHS_Cal_GetLinkActive >>\r\n");
    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pCalBase)) {
        DBG_HSCAL_ERR("GetLinkActive:pCalBase OR pCalCtxt NULL\r\n");
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase = pCalCtxt->pCalBase;

    /* If link is in L1 state, initiate a remote wakeup. */
    if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L2_SUSPEND) != 0) {
        DBG_HSCAL_INFO("GetLinkActive:L2Wake\r\n");
        Cy_USBHS_Cal_DevInitiatedResumeL2Sleep(pCalCtxt, USBHSDEV_LPM_L2_WAKE_DURATION);
    } else {
        if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0) {
            /* TBD: For now, ensure minimum residency in L1 state of ~100 us. Quicker remote wake
             * seems to cause issues with some host controllers.
             */
            Cy_SysLib_DelayUs(100);
            if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0) {
                DBG_HSCAL_TRACE("GetLinkActive:L1Wake\r\n");
                Cy_USBHS_Cal_DevInitiatedL1Exit(pCalCtxt, USBHSDEV_LPM_L1_WAKE_DURATION);
            }
        }
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}


/*******************************************************************************
* Function name: Cy_USBHS_Cal_SendMsg
****************************************************************************//**
*
* This function send message to upper layer ie USBD layer.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \param pMsg
* message will be given to upper layer.
*
* \return
* true if context switch is required at the end of ISR, false otherwise.
*
*******************************************************************************/
bool
Cy_USBHS_Cal_SendMsg (cy_stc_usb_cal_ctxt_t *pCalCtxt, void *pMsg)
{
    return (pCalCtxt->msgCb(pCalCtxt->pUsbdCtxt, (void *)pMsg));
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_IsLinkActive
****************************************************************************//**
*
* Check whether the USB 2.x link is in active (L0) state.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* true if link is in L0, false if link is in L1 or L2.
*******************************************************************************/
bool
Cy_USBHS_Cal_IsLinkActive (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type *pCalBase;
    bool linkActive = true;

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pCalBase)) {
        DBG_HSCAL_ERR("IsLinkActive:pCalBase OR pCalCtxt NULL\r\n");
        return (false);
    }

    pCalBase = pCalCtxt->pCalBase;
    if ((pCalBase->DEV_PWR_CS & (USBHSDEV_DEV_PWR_CS_L1_SLEEP | USBHSDEV_DEV_PWR_CS_L2_SUSPEND)) != 0) {
        linkActive = false;
    }

    return (linkActive);
}

#if defined(__cplusplus)
}
#endif

/*[]*/

