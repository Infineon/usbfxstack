/***************************************************************************//**
* \file cy_fault_handlers.h
* \version 1.0
*
* Defines the fault handling function interfaces for FX device family
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

#ifndef _CY_FAULT_HANDLERS_H_

/** Indicates the use of cy_fault_handlers */
#define _CY_FAULT_HANDLERS_H_

/** \} group_usbfxstack_fx_utils_macros */

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/* Initialize the WatchDog and set the timeout period for reset. */
void InitializeWDT (uint32_t timeout_ms);

/* Reset pending WatchDog interrupts. */
void KickWDT(void);

/** \} group_usbfxstack_fx_utils_functions */

#if defined(__cplusplus)
}
#endif

#endif /* _CY_FAULT_HANDLERS_H_ */

/** \} group_usbfxstack_fx_utils */

/* End of file */

