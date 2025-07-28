/***************************************************************************//**
* \file cy_fx_common.c
* \version 1.0
*
* Implements a set of common utility functions for use with USBFX devices.
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

#include <string.h>
#include <stdarg.h>
#include "cy_pdl.h"
#include "cy_fx_common.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_UsbFx_OnResetInit
 ****************************************************************************//**
 * This function performs initialization that is required to enable scatter
 * loading of data into the High BandWidth RAM during device boot-up. The FX10/FX20
 * device comes up with the High BandWidth RAM disabled and hence any attempt
 * to read/write the RAM will cause the processor to hang. The RAM needs to
 * be enabled with default clock settings to allow scatter loading to work.
 * This function needs to be called from Cy_OnResetUser.
 *
 *******************************************************************************/
void
Cy_UsbFx_OnResetInit (
        void)
{
    /* Enable clk_hf4 with IMO as input. */
    SRSS->CLK_ROOT_SELECT[4] = SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    /* Enable LVDS2USB32SS IP and select clk_hf[4] as clock input. */
    MAIN_REG->CTRL = (
            MAIN_REG_CTRL_IP_ENABLED_Msk |
            (1UL << MAIN_REG_CTRL_NUM_FAST_AHB_STALL_CYCLES_Pos) |
            (1UL << MAIN_REG_CTRL_NUM_SLOW_AHB_STALL_CYCLES_Pos) |
            (3UL << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
}

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
bool Cy_UsbFx_SelectDFTFunctions (
        uint32_t dft0_func,
        uint32_t dft1_func,
        uint32_t usbdft0_func,
        uint32_t usbdft1_func,
        uint32_t sipdft0_func,
        uint32_t sipdft1_func)
{
#if FX2G3_EN
    /* This function cannot be supported on FX2G3 device. */
    return false;
#else
    cy_stc_gpio_pin_config_t pinCfg;
    uint32_t mainSel = 0;
    uint32_t usbSel  = 0, usbGpioSel = 0;
    uint32_t phySel  = 0, phyRxSel = 0;
    uint32_t lvdsSel = 0, lvdsGpioSel = 0;

    /* Clear the pinCfg structure to start with. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    if ((dft0_func != CY_FX_DBG_FUNC_NONE) || (dft1_func != CY_FX_DBG_FUNC_NONE)) {
        /* Choose chip level DDFT function for P11.0 and P11.1 pins
         * and enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_0_SRSS_DDFT_PIN_IN0;
        Cy_GPIO_Pin_Init(P11_0_PORT, P11_0_PIN, &pinCfg);
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_1_SRSS_DDFT_PIN_IN1;
        Cy_GPIO_Pin_Init(P11_1_PORT, P11_1_PIN, &pinCfg);

        /* Route the DDFT outputs from the High BandWidth SubSystem to
         * the chip level pins.
         */
        SRSS_TST_DDFT_FAST_CTL_REG = 0x00000C0BUL;
        SRSS_TST_DDFT_SLOW_CTL_REG = 0xC0008080UL;
    }

    if (usbdft0_func != CY_FX_DBG_FUNC_NONE) {
        /* Choose HBWSS GPIO DFT function for P9.2 and P9.3 pins and
         * enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_2_LVDS2USB32SS_USB32_GPIO_DDFT_O0;
        Cy_GPIO_Pin_Init(P9_2_PORT, P9_2_PIN, &pinCfg);
    }

    if (usbdft1_func != CY_FX_DBG_FUNC_NONE) {
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_3_LVDS2USB32SS_USB32_GPIO_DDFT_O1;
        Cy_GPIO_Pin_Init(P9_3_PORT, P9_3_PIN, &pinCfg);
    }

    if (dft0_func != CY_FX_DBG_FUNC_NONE) {
        if (dft0_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE) {
            mainSel |= 0x0002UL;
            lvdsSel = ((dft0_func - 0x1000UL) & 0x0FFFUL);
        } else {
            if (dft0_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
                usbSel |= ((dft0_func - 0x0200UL) & 0x00FFUL);
            } else {
                if (dft0_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (dft0_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((dft0_func - 0x0180UL) & 0x000FUL));
                    } else {
                        phySel |= ((dft0_func - 0x0100UL) & 0x3FUL);
                    }
                } else {
                    if (dft0_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((dft0_func - 0x0080UL) & 0x000FUL));
                    } else {
                        phySel |= (dft0_func & 0x3FUL);
                    }
                }
            }
        }
    }

    if (dft1_func != CY_FX_DBG_FUNC_NONE) {
        if (dft1_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE) {
            mainSel |= 0x0300UL;
            lvdsSel |= (((dft1_func - 0x1000UL) & 0x0FFFUL) << 15U);
        } else {
            mainSel |= 0x0100UL;

            if (dft1_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
                usbSel |= (((dft1_func - 0x0200UL) & 0x00FFUL) << 8U);
            } else {
                usbSel |= 0x0100UL;
                if (dft1_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (dft1_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((dft1_func - 0x0180UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= (((dft1_func - 0x0100UL) & 0x3FUL) << 8U);
                    }
                } else {
                    if (dft1_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((dft1_func - 0x0080UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= ((dft1_func & 0x3FUL) << 8U);
                    }
                }
            }
        }
    }

    if ((usbdft0_func != CY_FX_DBG_FUNC_NONE) && (usbdft0_func < CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        if (usbdft0_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
            usbGpioSel |= ((usbdft0_func - 0x0200UL) & 0x00FFUL);
        } else {
            if ((phySel & 0x00FFUL) != 0) {
                /* Too many USB PHY signals requested. Cannot route. */
                return false;
            } else {
                if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((usbdft0_func - 0x0180UL) & 0x000FUL));
                    } else {
                        phySel |= ((usbdft0_func - 0x0100UL) & 0x3FUL);
                    }
                } else {
                    if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((usbdft0_func - 0x0080UL) & 0x000FUL));
                    } else {
                        phySel |= (usbdft0_func & 0x3FUL);
                    }
                }
            }
        }
    }

    if ((usbdft1_func != CY_FX_DBG_FUNC_NONE) && (usbdft1_func < CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        if (usbdft1_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
            usbGpioSel |= (((usbdft1_func - 0x0200UL) & 0x00FFUL) << 8U);
        } else {
            if ((phySel & 0xFF00UL) != 0) {
                /* Too many USB PHY signals requested. Cannot route. */
                return false;
            } else {
                usbGpioSel |= 0x0100UL;
                if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((usbdft1_func - 0x0180UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= (((usbdft1_func - 0x0100UL) & 0x3FUL) << 8U);
                    }
                } else {
                    if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((usbdft1_func - 0x0080UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= ((usbdft1_func & 0x3FUL) << 8U);
                    }
                }
            }
        }
    }

    if ((sipdft0_func != CY_FX_DBG_FUNC_NONE) && (sipdft0_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        /* Configure P9.4 pin for DDFT operation. */
        pinCfg.hsiom     = P9_4_LVDS2USB32SS_LVDS_GPIO_DDFT_O0;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        Cy_GPIO_Pin_Init(P9_4_PORT, P9_4_PIN, &pinCfg);

        lvdsGpioSel = ((sipdft0_func - 0x1000UL) & 0x0FFFUL);
    }

    if ((sipdft1_func != CY_FX_DBG_FUNC_NONE) && (sipdft1_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        /* Configure P9.5 pin for DDFT operation. */
        pinCfg.hsiom     = P9_5_LVDS2USB32SS_LVDS_GPIO_DDFT_O1;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        Cy_GPIO_Pin_Init(P9_5_PORT, P9_5_PIN, &pinCfg);

        lvdsGpioSel |= (((sipdft1_func - 0x1000UL) & 0x0FFFUL) << 15U);
    }

    /* Apply the DDFT settings. */
    MAIN_REG->DDFT_MUX                          = mainSel;
    USB32DEV->USB32DEV_MAIN.DDFT_MUX            = usbSel;
    USB32DEV->USB32DEV_MAIN.GPIO_DDFT_MUX       = usbGpioSel;
    LVDSSS_LVDS->DDFT_MUX_SEL                   = lvdsSel;
    LVDSSS_LVDS->GPIO_DDFT_MUX_SEL              = lvdsGpioSel;

    USB32DEV->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.PHYSS_DDFT_MUX_SEL = phySel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.PHYSS_DDFT_MUX_SEL = phySel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_RX.RX_DTEST_CFG        = phyRxSel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_RX.RX_DTEST_CFG        = phyRxSel;

    return true;
#endif /* FX2G3_EN */
}

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
Cy_UsbFx_MemSetDword (
        uint32_t *pBuffer,
        uint32_t value,
        uint32_t byteCount)
{
    uint32_t index;

    if ((pBuffer != NULL) && (byteCount > 0)) {
        for (index = 0; index < (byteCount / 4); index++) {
            pBuffer[index] = value;
        }

        switch (byteCount & 0x03) {
            case 3:
                pBuffer[index] = (pBuffer[index] & 0xFF000000UL) | (value & 0x00FFFFFFUL);
                break;

            case 2:
                pBuffer[index] = (pBuffer[index] & 0xFFFF0000UL) | (value & 0x0000FFFFUL);
                break;

            case 1:
                pBuffer[index] = (pBuffer[index] & 0xFFFFFF00UL) | (value & 0x000000FFUL);
                break;

            default:
                break;
        }
    }
}

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
Cy_UsbFx_MemCpyDword (
        uint32_t *pDst,
        const uint32_t *pSrc,
        uint32_t byteCount)
{
    uint32_t index;

    if ((pDst != NULL) && (pSrc != NULL) && (byteCount > 0)) {
        for (index = 0; index < (byteCount / 4); index++) {
            pDst[index] = pSrc[index];
        }

        switch (byteCount & 0x03) {
            case 3:
                pDst[index] = (pDst[index] & 0xFF000000UL) | (pSrc[index] & 0x00FFFFFFUL);
                break;

            case 2:
                pDst[index] = (pDst[index] & 0xFFFF0000UL) | (pSrc[index] & 0x0000FFFFUL);
                break;

            case 1:
                pDst[index] = (pDst[index] & 0xFFFFFF00UL) | (pSrc[index] & 0x000000FFUL);
                break;

            default:
                break;
        }
    }
}

#if defined(__cplusplus)
}
#endif

/*[]*/

