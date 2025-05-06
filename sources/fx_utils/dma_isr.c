/***************************************************************************//**
* \file dma_isr.c
* \version 1.0
*
* Provides wrapper ISRs that can be used to manage DMA completion interrupts
* raised by DataWire channels transferring data from/to the USBHS endpoint
* memories.
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

#include "cy_pdl.h"
#include <string.h>
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"

#if defined(__cplusplus)
extern "C" {
#endif

__WEAK void OutEpDma_ISR (uint8_t endpNumber)
{
    (void)endpNumber;
}

__WEAK void InEpDma_ISR (uint8_t endpNumber)
{
    (void)endpNumber;
}

void Ep0OutDma_ISR(void)
{
    OutEpDma_ISR(0x00U);
}

void Ep1OutDma_ISR(void)
{
    OutEpDma_ISR(0x01U);
}

void Ep2OutDma_ISR(void)
{
    OutEpDma_ISR(0x02U);
}

void Ep3OutDma_ISR(void)
{
    OutEpDma_ISR(0x03U);
}

void Ep4OutDma_ISR(void)
{
    OutEpDma_ISR(0x04U);
}

void Ep5OutDma_ISR(void)
{
    OutEpDma_ISR(0x05U);
}

void Ep6OutDma_ISR(void)
{
    OutEpDma_ISR(0x06U);
}

void Ep7OutDma_ISR(void)
{
    OutEpDma_ISR(0x07U);
}

void Ep8OutDma_ISR(void)
{
    OutEpDma_ISR(0x08U);
}

void Ep9OutDma_ISR(void)
{
    OutEpDma_ISR(0x09U);
}

void Ep10OutDma_ISR(void)
{
    OutEpDma_ISR(0x0AU);
}

void Ep11OutDma_ISR(void)
{
    OutEpDma_ISR(0x0BU);
}

void Ep12OutDma_ISR(void)
{
    OutEpDma_ISR(0x0CU);
}

void Ep13OutDma_ISR(void)
{
    OutEpDma_ISR(0x0DU);
}

void Ep14OutDma_ISR(void)
{
    OutEpDma_ISR(0x0EU);
}

void Ep15OutDma_ISR(void)
{
    OutEpDma_ISR(0x0FU);
}

void Ep1InDma_ISR(void)
{
    InEpDma_ISR(0x01U);
}

void Ep2InDma_ISR(void)
{
    InEpDma_ISR(0x02U);
}

void Ep3InDma_ISR(void)
{
    InEpDma_ISR(0x03U);
}

void Ep4InDma_ISR(void)
{
    InEpDma_ISR(0x04U);
}

void Ep5InDma_ISR(void)
{
    InEpDma_ISR(0x05U);
}

void Ep6InDma_ISR(void)
{
    InEpDma_ISR(0x06U);
}

void Ep7InDma_ISR(void)
{
    InEpDma_ISR(0x07U);
}

void Ep8InDma_ISR(void)
{
    InEpDma_ISR(0x08U);
}

void Ep9InDma_ISR(void)
{
    InEpDma_ISR(0x09U);
}

void Ep10InDma_ISR(void)
{
    InEpDma_ISR(0x0AU);
}

void Ep11InDma_ISR(void)
{
    InEpDma_ISR(0x0BU);
}

void Ep12InDma_ISR(void)
{
    InEpDma_ISR(0x0CU);
}

void Ep13InDma_ISR(void)
{
    InEpDma_ISR(0x0DU);
}

void Ep14InDma_ISR(void)
{
    InEpDma_ISR(0x0EU);
}

void Ep15InDma_ISR(void)
{
    InEpDma_ISR(0x0FU);
}

#if CY_CPU_CORTEX_M4
static cy_israddress OutEpDmaIsrList[] = {
    Ep0OutDma_ISR,
    Ep1OutDma_ISR,
    Ep2OutDma_ISR,
    Ep3OutDma_ISR,
    Ep4OutDma_ISR,
    Ep5OutDma_ISR,
    Ep6OutDma_ISR,
    Ep7OutDma_ISR,
    Ep8OutDma_ISR,
    Ep9OutDma_ISR,
    Ep10OutDma_ISR,
    Ep11OutDma_ISR,
    Ep12OutDma_ISR,
    Ep13OutDma_ISR,
    Ep14OutDma_ISR,
    Ep15OutDma_ISR
};

static cy_israddress InEpDmaIsrList[] = {
    NULL,
    Ep1InDma_ISR,
    Ep2InDma_ISR,
    Ep3InDma_ISR,
    Ep4InDma_ISR,
    Ep5InDma_ISR,
    Ep6InDma_ISR,
    Ep7InDma_ISR,
    Ep8InDma_ISR,
    Ep9InDma_ISR,
    Ep10InDma_ISR,
    Ep11InDma_ISR,
    Ep12InDma_ISR,
    Ep13InDma_ISR,
    Ep14InDma_ISR,
    Ep15InDma_ISR
};
#else
void CombinedInEpISR(void)
{
    uint32_t chnId;

    /* For each DW1 channel from 1 to 15, if the interrupt is pending; call the ISR. */
    for (chnId = 1; chnId < 16; chnId++) {
        if (Cy_DMA_Channel_GetInterruptStatus(DW1, chnId) != 0) {
            InEpDma_ISR((uint8_t)chnId);
        }
    }

    /* For DMAC channels 3 and 5, if the interrupt is pending; call the ISR. */
    if (Cy_DMAC_Channel_GetInterruptStatus(DMAC, 3) != 0) {
        InEpDma_ISR(1);
    }
    if (Cy_DMAC_Channel_GetInterruptStatus(DMAC, 5) != 0) {
        InEpDma_ISR(2);
    }
}

void CombinedOutEpISR(void)
{
    uint32_t chnId;

    /* For each DW0 channel from 0 to 15, if the interrupt is pending; call the ISR. */
    for (chnId = 0; chnId < 16; chnId++) {
        if (Cy_DMA_Channel_GetInterruptStatus(DW0, chnId) != 0) {
            OutEpDma_ISR((uint8_t)chnId);
        }
    }

    /* For DMAC channels 2 and 4, if the interrupt is pending; call the ISR. */
    if (Cy_DMAC_Channel_GetInterruptStatus(DMAC, 2) != 0) {
        OutEpDma_ISR(1);
    }
    if (Cy_DMAC_Channel_GetInterruptStatus(DMAC, 4) != 0) {
        OutEpDma_ISR(2);
    }
}
#endif /* (CY_CPU_CORTEX_M4) */

cy_israddress GetEPOutDmaIsr(uint8_t epNum)
{
#if CY_CPU_CORTEX_M4
    if (epNum < 16u)
        return OutEpDmaIsrList[epNum];
    return NULL;
#else
    if ((epNum > 0) && (epNum < 16u))
        return CombinedOutEpISR;
    return NULL;
#endif /* CY_CPU_CORTEX_M4 */
}

cy_israddress GetEPInDmaIsr(uint8_t epNum)
{
#if CY_CPU_CORTEX_M4
    if (epNum < 16u)
        return InEpDmaIsrList[epNum];
    return NULL;
#else
    if ((epNum > 0) && (epNum < 16u))
        return CombinedInEpISR;
    return NULL;
#endif /* CY_CPU_CORTEX_M4 */
}

#if defined(__cplusplus)
}
#endif

/* End of File */

