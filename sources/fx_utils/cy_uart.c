/***************************************************************************//**
* \file cy_uart.c
* \version 1.0
*
* Provides functions to configure SCB-UART block on the FX device family for debug
* logging.
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
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*****************************************************************************
 * Function Name: ConfigureSCBClock
 *****************************************************************************
 * Summary
 *  Configure the clock applied to the specified SCB.
 *
 * Parameters:
 * \param scbIndex
 * Index of SCB for which the clock is to be configured.
 *
 * Return:
 *  void
 ****************************************************************************/
void ConfigureSCBClock (uint8_t scbIndex)
{
    /* Get the PERI clock frequency for the platform. */
    uint32_t hfClkFreq = Cy_SysClk_ClkPeriGetFrequency();

    /* Configure PERI 16 bit clock divider for 8.33 MHz operation and enable it. */
    switch (hfClkFreq)
    {
        case 50000000UL:
            /* Divide 50 MHz by 6 to get 8.33 MHz. */
            Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0, 5);
            break;

        case 60000000UL:
            /* Divide 60 MHz by 5 to get 12 MHz. */
            Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0, 4);
            break;

        case 75000000UL:
            /* Divide 75 MHz by 9 to get 8.33 MHz. */
            Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0, 8);
            break;

        case 100000000UL:
            /* Divide 100 MHz by 12 to get 8.33 MHz. */
            Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0, 11);
            break;

        default:
            break;
    }

    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 0);
    Cy_SysLib_DelayUs(10);

    /* Connect the PERI clock to the SCB input. */
    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)(PCLK_SCB0_CLOCK + scbIndex), CY_SYSCLK_DIV_16_BIT, 0);
}

/* SCB-UART context structure. */
static cy_stc_scb_uart_context_t gUartCtxt;

/*****************************************************************************
 * Function Name: InitUart
 *****************************************************************************
 * Summary
 *  Initialize SCB as UART for printing logs.
 *
 * Parameters:
 * \param scbIndex
 * Index of SCB to be used for UART output. Only 0 and 1 supported at
 * present.
 *
 * Return:
 *  void
 ****************************************************************************/
void InitUart (uint8_t scbIndex)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_scb_uart_config_t uartCfg;
    CySCB_Type *pSCB = NULL;

    /* Only SCB0, SCB1 and SCB4 are supported at present.*/
    CY_ASSERT_L1((scbIndex <= 1) || (scbIndex==4));

    ConfigureSCBClock (scbIndex);

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure the UART pins:
     * UART_TX as Strong Drive output.
     * UART_RX as input pin with output driver tri-stated.
     */
    switch (scbIndex) {
    case 0:
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P10_1_SCB0_UART_TX;
        Cy_GPIO_Pin_Init(P10_1_PORT, P10_1_PIN, &pinCfg);
        pinCfg.hsiom     = P10_0_SCB0_UART_RX;
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        Cy_GPIO_Pin_Init(P10_0_PORT, P10_0_PIN, &pinCfg);
        pSCB = SCB0;
        break;

    case 1:
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P8_1_SCB1_UART_TX;
        Cy_GPIO_Pin_Init(P8_1_PORT, P8_1_PIN, &pinCfg);
        pinCfg.hsiom     = P8_0_SCB1_UART_RX;
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        Cy_GPIO_Pin_Init(P8_0_PORT, P8_0_PIN, &pinCfg);
        pSCB = SCB1;
        break;

    case 4:
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_0_SCB4_UART_TX;
        Cy_GPIO_Pin_Init(P11_0_PORT, P11_0_PIN, &pinCfg);
        pinCfg.hsiom     = P11_1_SCB4_UART_RX;
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        Cy_GPIO_Pin_Init(P11_1_PORT, P11_1_PIN, &pinCfg);
        pSCB = SCB4;
        break;

    default:
        break;
    }

    memset((void *)&uartCfg, 0, sizeof(uartCfg));

    /* Divide 8.33 MHz clock by 9 to get baud rate of ~921600. */
    uartCfg.oversample = 9;
    uartCfg.uartMode   = CY_SCB_UART_STANDARD;
    uartCfg.dataWidth  = 8;
    uartCfg.enableMsbFirst = false;
    uartCfg.stopBits   = CY_SCB_UART_STOP_BITS_1;
    uartCfg.parity     = CY_SCB_UART_PARITY_NONE;
    uartCfg.breakWidth = 12;
    uartCfg.rxFifoTriggerLevel = 4;
    uartCfg.txFifoTriggerLevel = 4;

    if (Cy_SysClk_ClkPeriGetFrequency() == 60000000UL) {
        /* Divide 12 MHz clock by 13 to get baud rate of ~921600. */
        uartCfg.oversample = 13;
    }

    /* Configure and enable the SCB block as UART. */
    Cy_SCB_UART_Init(pSCB, &uartCfg, &gUartCtxt);
    Cy_SCB_UART_Enable(pSCB);
}

#if defined(__cplusplus)
}
#endif

/* End of file */

