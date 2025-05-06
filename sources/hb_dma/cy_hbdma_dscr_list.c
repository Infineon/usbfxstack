/***************************************************************************//**
* \file cy_hbdma_dscr_list.c
* \version 1.0
*
* Implements the HBWSS DMA descriptor management functions.
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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 * Function name: Cy_HBDma_DscrList_Create
 ****************************************************************************//**
 *
 * Initialize the allocator used to manage High BandWidth DMA descriptors. It is
 * assumed that all descriptors are free for use after this API has been called.
 *
 * \param list_p
 * Pointer to the descriptor list structure to be initialized.
 *
 * \param maxDscrCount
 * Maximum number of descriptors to be allowed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the allocator initialization is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DscrList_Create (
        cy_stc_hbdma_dscr_list_t *list_p,
        uint16_t maxDscrCount)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    if ((list_p == NULL) || (maxDscrCount > CY_HBDMA_MAX_DSCR_CNT))
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        /* Round maxDscrCount down to a multiple of 32. */
        maxDscrCount &= 0xFFE0U;

        /* Zero out the structure to indicate that all descriptors are free (unused). */
        memset ((void *)list_p, 0, sizeof(cy_stc_hbdma_dscr_list_t));

        /* Update the total and current available descriptor counts. */
        list_p->totalCount = maxDscrCount;
        list_p->availCount = maxDscrCount;
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_DscrList_Destroy
 ****************************************************************************//**
 *
 * De-initialize the allocator used to manage High BandWidth DMA descriptors.
 *
 * \param list_p
 * Pointer to the descriptor list structure to be de-initialized.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the allocator de-init is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameter is invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DscrList_Destroy (
        cy_stc_hbdma_dscr_list_t *list_p)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    if (list_p == NULL)
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        memset((void *)list_p, 0, sizeof(cy_stc_hbdma_dscr_list_t));
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_DscrList_Get
 ****************************************************************************//**
 *
 * Obtain a free HBW DMA descriptor from the free list.
 *
 * \param list_p
 * Pointer to the descriptor list structure.
 *
 * \param dscrIndex_p
 * Return parameter through which the descriptor index is returned.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if a free descriptor is available and is being returned.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_MEMORY_ERROR if no descriptors are available.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DscrList_Get (
        cy_stc_hbdma_dscr_list_t *list_p,
        uint16_t *dscrIndex_p)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint32_t wIndex, bIndex;
    uint32_t temp;

    if ((list_p == NULL) || (dscrIndex_p == NULL))
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        if (list_p->availCount > 0)
        {
            for (wIndex = 0; wIndex < (list_p->totalCount / 32U); wIndex++)
            {
                temp = list_p->dscrStatus[wIndex];
                if (temp != 0xFFFFFFFFUL)
                {
                    for (bIndex = 0; bIndex < 32U; bIndex++)
                    {
                        if ((temp & (1UL << bIndex)) == 0)
                        {
                            list_p->dscrStatus[wIndex] |= (1UL << bIndex);
                            list_p->availCount--;
                            *dscrIndex_p = (wIndex * 32U + bIndex);

                            return status;
                        }
                    }
                }
            }
        }

        /* Failed to find a free descriptor. */
        status = CY_HBDMA_MGR_MEMORY_ERROR;
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_DscrList_Put
 ****************************************************************************//**
 *
 * Release a HBW DMA descriptor back to the free list.
 *
 * @note The function will succeed even if the descriptor being freed was already
 * free.
 *
 * \param list_p
 * Pointer to the descriptor list structure.
 *
 * \param dscrIndex
 * Index of descriptor to be freed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the descriptor free operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DscrList_Put (
        cy_stc_hbdma_dscr_list_t *list_p,
        uint16_t dscrIndex)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    if ((list_p == NULL) || (dscrIndex >= list_p->totalCount))
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        if ((list_p->dscrStatus[dscrIndex / 32U] & (1UL << (dscrIndex & 0x1FU))) != 0)
        {
            list_p->dscrStatus[dscrIndex / 32U] &= ~(1UL << (dscrIndex & 0x1FU));
            list_p->availCount++;
        }
    }

    return status;
}

#if defined(__cplusplus)
}
#endif

/* End of file */

