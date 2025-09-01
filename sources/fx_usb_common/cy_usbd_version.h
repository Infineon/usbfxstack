/***************************************************************************//**
* \file cy_usbd_version.h
* \version 1.0
*
* Provides version information for the USB Device Stack.
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
 * \addtogroup group_usbfxstack_usb_common
 * \{
 */

/**
 * \addtogroup group_usbfxstack_usb_common_macros
 * \{
 */

#ifndef _CY_USBD_VERSION_H_

/** Indicates the use of cy_usbd_version */
#define _CY_USBD_VERSION_H_

/** USBD middleware major version */
#define USBD_VERSION_MAJOR      (1u)

/** USBD middleware minor version */
#define USBD_VERSION_MINOR      (3u)

/** USBD middleware patch version */
#define USBD_VERSION_PATCH      (1u)

/** USBD middleware build number */
#define USBD_VERSION_BUILD      (96u)

/** USBD middleware version number. */
#define USBD_VERSION_NUM        ((USBD_VERSION_MAJOR << 28u) | (USBD_VERSION_MINOR << 24u) | \
        (USBD_VERSION_PATCH << 16u) | (USBD_VERSION_BUILD))

#endif /* _CY_USBD_VERSION_H_ */

/** \} group_usbfxstack_usb_common_macros */

/** \} group_usbfxstack_usb_common */

/* [] EOF */
