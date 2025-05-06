/***************************************************************************//**
* \file cy_hbdma_version.h
* \version 1.00
*
* Header file providing version information for the High BandWidth DMA
* Manager Library.
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
 * \addtogroup group_usbfxstack_hb_dma
 * \{
 */

/**
 * \addtogroup group_usbfxstack_hb_dma_macros
 * \{
 */

#if !defined(CY_HBDMA_VERSION_H)

/** Indicates the use of HBDMA Version */
#define CY_HBDMA_VERSION_H

/** HBDma manager middleware major version */
#define HBDMA_VERSION_MAJOR      (1u)

/** HBDma manager middleware minor version */
#define HBDMA_VERSION_MINOR      (1u)

/** HBDma manager middleware patch version */
#define HBDMA_VERSION_PATCH      (0u)

/** HBDma manager middleware build number */
#define HBDMA_VERSION_BUILD      (31u)

/** HBDma manager middleware version number */
#define HBDMA_VERSION_NUM        ((HBDMA_VERSION_MAJOR << 28u) | (HBDMA_VERSION_MINOR << 24u) | \
        (HBDMA_VERSION_PATCH << 16u) | (HBDMA_VERSION_BUILD))

#endif /* CY_HBDMA_VERSION_H */

/** \} group_usbfxstack_hb_dma_macros */

/** \} group_usbfxstack_hb_dma */

/* [] END OF FILE */

