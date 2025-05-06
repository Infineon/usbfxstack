/***************************************************************************//**
* \file cy_efuse_access.h
* \version 1.0
*
* Header file providing details of eFUSE read/write APIs for USB-FX device.
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

#ifndef _CY_EFUSE_ACCESS_H_

/** Indicates the use of cy_efuse_access */
#define _CY_EFUSE_ACCESS_H_

#include "cy_pdl.h"
#include "cy_device.h"

#if defined(__cplusplus)
extern "C" {
#endif

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/*****************************************************************************
 * Function name: Cy_Efuse_IsBitBlown
 *************************************************************************//**
 *
 * Check whether the specified CustomerData eFuse bit is blown. The fuse read
 * is done through system call made to USB-FX device ROM code.
 *
 * \param bitPos
 * Bit position of the fuse to be checked. Valid values are in the range of 0 to 7
 *
 * \param offset
 * Byte offset of the fuse to be checked.  Valid values are in the range of 0x52 to 0x7F
 *
 * \return
 * true if the fuse bit is already blown, false otherwise.
 ****************************************************************************/
bool Cy_Efuse_IsBitBlown(uint32_t bitPos, uint32_t offset);

/*****************************************************************************
 * Function name: Cy_Efuse_WriteBit
 *************************************************************************//**
 *
 * Blow one fuse bit in the CustomerData region. The fuse programming is
 * done through system call made to the USB-FX device ROM code.
 *
 * \param bitPos
 * Bit position of the fuse to be written. Valid values are in the range of 0 to 7
 *
 * \param offset
 * Byte offset of the fuse to be written.  Valid values are in the range of 0x52 to 0x7F
 *
 ****************************************************************************/
void Cy_Efuse_WriteBit(uint32_t bitPos, uint32_t offset);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_EFUSE_ACCESS_H_ */

/** \} group_usbfxstack_fx_utils */

/*[]*/


