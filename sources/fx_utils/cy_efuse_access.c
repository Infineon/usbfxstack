/***************************************************************************//**
* \file cy_efuse_access.c
* \version 1.0
*
* Implements functions to read/write CustomerData eFUSE locations.
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
#include "cy_device.h"
#include "cy_debug.h"
#include "cy_efuse_access.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if CY_CPU_CORTEX_M4
    /* Cortex-M4 shall use IPC struct 1 to make system calls. */
    #define USB_FX_SYSCALL_IPC        IPC_STRUCT1
#else
    /* Cortex-M0+ shall use IPC struct 0 to make system calls. */
    #define USB_FX_SYSCALL_IPC        IPC_STRUCT0
#endif /* CY_CPU_CORTEX_M4 */

/*****************************************************************************
 * Function Name: Cy_Efuse_IsBitBlown
 *****************************************************************************
 *
 * Check whether the specified CustomerData eFuse bit is blown. The fuse read
 * is done through system call made to USB-FX device ROM code.
 *
 * \param bitPos
 * Bit position of the fuse to be checked. Valid values are in the range of 0 to 7.
 *
 * \param offset
 * Byte offset of the fuse to be checked.  Valid values are in the range of 0x52 to 0x7F.
 *
 * \return
 * true if the fuse bit is already blown, false otherwise.
 ****************************************************************************/
bool Cy_Efuse_IsBitBlown (uint32_t bitPos, uint32_t offset)
{
    uint32_t i, j;
    bool ret = false;

    /* Reading is only allowed for CUSTOMER_DATA fuses. */
    if ((offset >= 0x52) && (bitPos < 8)) {
        /* First, try to acquire the IPC structure and proceed only in case of success. */
        i = USB_FX_SYSCALL_IPC->ACQUIRE;
        j = USB_FX_SYSCALL_IPC->LOCK_STATUS;

        if ((j & IPC_STRUCT_LOCK_STATUS_ACQUIRED_Msk) != 0) {
            /* Setup the parameters for fuse read and make the system call. */
            USB_FX_SYSCALL_IPC->DATA0  = 0x03000001UL | (offset << 8U);
            USB_FX_SYSCALL_IPC->NOTIFY = 0x00000001UL;

            /* Wait until IPC structure is released. */
            do {
                j = USB_FX_SYSCALL_IPC->LOCK_STATUS;
            } while ((j & IPC_STRUCT_LOCK_STATUS_ACQUIRED_Msk) != 0);

            i = USB_FX_SYSCALL_IPC->DATA0;
            if ((i & 0xF0000000) == 0xA0000000UL) {
                if ((i & (1UL << bitPos)) != 0) {
                    DBG_APP_INFO("Fuse bit %d at offset %d is blown\r\n", bitPos, offset);
                    ret = true;
                } else {
                    DBG_APP_INFO("Fuse bit %d at offset %d is not blown\r\n", bitPos, offset);
                }
            } else {
                DBG_APP_WARN("Fuse read status: %x\r\n", i);
            }
        } else {
            DBG_APP_ERR("Failed to acquire IPC structure\r\n");
        }
    } else {
        DBG_APP_ERR("Invalid fuse location\r\n");
    }

    return ret;
}

/*****************************************************************************
 * Function Name: Cy_Efuse_WriteBit
 *****************************************************************************
 *
 * Blow one fuse bit in the CustomerData region. The fuse programming is
 * done through system call made to the USB-FX device ROM code.
 *
 * \param bitPos
 * Bit position of the fuse to be written. Valid values are in the range of 0 to 7.
 *
 * \param offset
 * Byte offset of the fuse to be written.  Valid values are in the range of 0x52 to 0x7F.
 *
 ****************************************************************************/
void Cy_Efuse_WriteBit (uint32_t bitPos, uint32_t offset)
{
    uint32_t i, j;

    /* Programming is only allowed for CUSTOMER_DATA fuses. */
    if ((offset >= 0x52) && (bitPos < 8)) {
        /* First, try to acquire the IPC structure and proceed only in case of success. */
        i = USB_FX_SYSCALL_IPC->ACQUIRE;
        j = USB_FX_SYSCALL_IPC->LOCK_STATUS;

        if ((j & IPC_STRUCT_LOCK_STATUS_ACQUIRED_Msk) != 0) {
            /* Setup the parameters for fuse programming and make the system call. */
            USB_FX_SYSCALL_IPC->DATA0  = 0x01000001UL | ((offset & 0x1FUL) << 16U) |
                ((offset >> 5U) << 12U) | (bitPos << 8U);
            USB_FX_SYSCALL_IPC->NOTIFY = 0x00000001UL;

            /* Wait until IPC structure is released. */
            do {
                j = USB_FX_SYSCALL_IPC->LOCK_STATUS;
            } while ((j & IPC_STRUCT_LOCK_STATUS_ACQUIRED_Msk) != 0);

            i = USB_FX_SYSCALL_IPC->DATA0;
            if (i == 0xA0000000UL) {
                DBG_APP_INFO("Fuse programming of bit %d at offset %d successful\r\n", bitPos, offset);
            } else {
                DBG_APP_WARN("Fuse programming status: %x\r\n", i);
            }
        } else {
            DBG_APP_ERR("Failed to acquire IPC structure\r\n");
        }
    } else {
        DBG_APP_ERR("Invalid fuse location\r\n");
    }
}

#if defined(__cplusplus)
}
#endif

/*[]*/


