/***************************************************************************//**
* \file cy_hbdma_buf_alloc.c
* \version 1.0
*
* Implements the DMA buffer management functions.
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

/* Keep error detection feature enabled. */
#define CY_HBDMA_ERRORDETECTION         (1u)

#define CY_DMA_MEM_START_SIG            (0x4658334D)
#define CY_DMA_MEM_END_SIG              (0x454E444D)

/* Round a given value up to a multiple of n (assuming n is a power of 2). */
#define ROUND_UP(s, n)                  (((s) + (n) - 1) & (~(n - 1)))

/* Convert size from BYTE to DWORD. */
#define BYTE_TO_DWORD(s)                ((s) >> 2)

/* Granularity of DMA buffers allocated and managed by the HBDMA buffer allocator. */
#define HBDMA_BUFFER_SIZE_UNIT          (64U)

/* Number of bits to shift the page number to get the byte address. */
#define HBDMA_PAGE_TO_ADDR_SHIFT        (6U)

/* Memory size covered by one DWORD of usedStatus. */
#define HBDMA_STATUS_SZ_MULT            (HBDMA_BUFFER_SIZE_UNIT * 32U)

#if CY_HBDMA_ERRORDETECTION

/**
 * @brief Structure added at the beginning of each allocated DMA buffer to enable
 * checks for memory leaks as well as memory corruption due to out of bound accesses.
 */
typedef struct HBDmaMemBlockInfo {
    uint32_t                  alloc_id;         /** Allocation id. */
    uint32_t                  alloc_size;       /** Actual memory size requested. */
    struct HBDmaMemBlockInfo *prev_blk;         /** Previous memory block. */
    struct HBDmaMemBlockInfo *next_blk;         /** Next memory block. */
    uint32_t                  start_sig;        /** Block start signature. */
    uint32_t                  pad[3];           /** Padding used to extend the header to 32 bytes. */
} HBDmaMemBlockInfo;

/**
 * @brief Type of callback function used by the buffer manager to notify the
 * application layer about any memory corruption.
 *
 * \param mem_p
 * Pointer to the memory buffer where corruption was detected.
 *
 * \return
 * void
 */
typedef void (*HBDmaMemCorruptCallback_t) (
        void *mem_p
        );

/*
   Debug variables used for doing memory leak and corruption checks around buffers allocated through
   the Cy_HBDma_BufMgr_Alloc function.
 */
static bool                       glBufMgrEnableChecks = false; /* Whether checks are enabled. */
static uint32_t                   glBufAllocCnt        = 0;     /* Number of alloc operations performed. */
static uint32_t                   glBufFreeCnt         = 0;     /* Number of free operations performed. */
static HBDmaMemBlockInfo         *glBufInUseList       = 0;     /* List of all memory blocks in use. */
static HBDmaMemCorruptCallback_t  glBufBadCb    = 0;            /* Callback for notification of corrupted memory. */

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_EnableChecks
 ****************************************************************************//**
 *
 * Enable memory leak and corruption checks in the DMA buffer allocator. Enabling
 * checks increases the memory required for each allocated block to increase by 64
 * bytes and the allocation operation takes additional time.
 *
 * \param mgr_p
 * Pointer to the buffer manager context structure.
 *
 * \param enable
 * Whether the memory leak and corruption checks are to be enabled.
 *
 * \param cb
 * Callback function to be called when memory corruption is detected.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the request to enable memory checks is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the callback pointer is invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the buffer manager has already been initialized.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_BufMgr_EnableChecks (
        cy_stc_hbdma_buf_mgr_t   *mgr_p,
        bool                      enable,
        HBDmaMemCorruptCallback_t cb)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    if ((mgr_p == NULL) || ((enable) && (cb == NULL)))
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        if ((enable) && (mgr_p->usedStatus != 0))
        {
            status = CY_HBDMA_MGR_SEQUENCE_ERROR;
        }
        else
        {
            glBufMgrEnableChecks = enable;
            glBufBadCb           = cb;
        }
    }

    return status;
}

#endif /* CY_HBDMA_ERRORDETECTION */

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_Create
 ****************************************************************************//**
 *
 * This function initializes the custom heap used for DMA buffer allocation.
 * These functions use a home-grown allocator in order to ensure that all
 * DMA buffers allocated are cache line aligned (multiple of 64 bytes).
 *
 * \param mgr_p
 * Pointer to the buffer manager context structure.
 *
 * \param baseAddr
 * Base address of the memory region reserved for DMA buffers.
 *
 * \param regionSize
 * Size of the memory region reserved for DMA buffers.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the custom heap init is successful.
 * CY_HBDMA_MGR_BAD_PARAM in case of invalid parameters.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_BufMgr_Create (
        cy_stc_hbdma_buf_mgr_t *mgr_p,
        uint32_t *baseAddr,
        uint32_t regionSize)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint32_t startAddr, lastAddr;
    uint32_t statusSize;
    uint32_t i;

    if ((mgr_p == NULL) || ((uint32_t)baseAddr < CY_HBW_SRAM_BASE_ADDR) || (regionSize == 0))
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        /* Store the first and last valid buffer address. */
        startAddr = (uint32_t)baseAddr;
        lastAddr  = (uint32_t)baseAddr + regionSize;

        /* Calculate the size of the usedStatus data structure needed. */
        statusSize = ROUND_UP(regionSize, HBDMA_STATUS_SZ_MULT) / HBDMA_STATUS_SZ_MULT;
        startAddr += statusSize * sizeof(uint32_t);

        /* Round baseAddr up to a multiple of 64 bytes. */
        startAddr  = ROUND_UP(startAddr, HBDMA_BUFFER_SIZE_UNIT);
        regionSize = lastAddr - startAddr;

        /* Round regionSize down to a multiple of 64 bytes. */
        regionSize = regionSize & ~(HBDMA_BUFFER_SIZE_UNIT - 1);

        /* We shall reserve statusSize DWORDs at the base of the memory region for the usedStatus
         * structure and reduce the available memory by the equivalent amount.
         */
        mgr_p->usedStatus = baseAddr;
        mgr_p->startAddr  = (uint8_t *)startAddr;
        mgr_p->regionSize = regionSize;
        mgr_p->statusSize = statusSize;
        mgr_p->searchPos  = 0;

        /* Clear the memory used status array. */
        memset ((void *)mgr_p->usedStatus, 0, statusSize * sizeof(uint32_t));

        /* Mark any status bits beyond the available space as used. */
        for (i = (regionSize / HBDMA_BUFFER_SIZE_UNIT); i < (statusSize * 32U); i++)
        {
            mgr_p->usedStatus[i / 32U] |= (1UL << (i & 31U));
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_Destroy
 ****************************************************************************//**
 *
 * This function frees up the custom heap used for DMA buffer allocation.
 *
 * \param mgr_p
 * Pointer to the buffer manager context structure.
 *
 * \param freeAll
 * Whether all memory buffers should be freed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the heap de-init is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_BufMgr_Destroy (
        cy_stc_hbdma_buf_mgr_t *mgr_p,
        bool                    freeAll)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;

    (void)freeAll;      /* To be implemented. */

    if (mgr_p == NULL)
    {
        status = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
        memset ((void *)mgr_p, 0, sizeof(cy_stc_hbdma_buf_mgr_t));

#if CY_HBDMA_ERRORDETECTION
        /* Clear status tracking variables. */
        glBufAllocCnt  = 0;
        glBufFreeCnt   = 0;
        glBufInUseList = 0;
#endif /* CY_HBDMA_ERRORDETECTION */
    }

    return status;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_SetStatus
 ****************************************************************************//**
 *
 * Helper function for the DMA buffer manager. Used to set or clear
 * a set of status bits from the alloc and free functions.
 *
 * \param mgr_p
 * Pointer to the DMA buffer manager context.
 *
 * \param startPos
 * Starting bit position.
 *
 * \param numBits
 * Number of bits to be updated.
 *
 * \param value
 * Value to set the bits to.
 *
 * \return
 * void
 *******************************************************************************/
static void
Cy_HBDma_BufMgr_SetStatus (
        cy_stc_hbdma_buf_mgr_t *mgr_p,
        uint32_t startPos,
        uint32_t numBits,
        bool value)
{
    uint32_t wordnum  = (startPos >> 5U);
    uint32_t startbit, endbit, mask;

    startbit = (startPos & 31UL);
    endbit   = (startbit + numBits);
    if (endbit > 32UL)
    {
        endbit = 32UL;
    }

    /* Compute a mask that has a 1 at all bit positions to be altered. */
    mask  = (endbit == 32UL) ? 0xFFFFFFFFUL : ((uint32_t)(1UL << endbit) - 1UL);
    mask -= ((1UL << startbit) - 1UL);

    /* Repeatedly go through the array and update each 32 bit word as required. */
    while (numBits)
    {
        if (value)
        {
            mgr_p->usedStatus[wordnum] |= mask;
        }
        else
        {
            mgr_p->usedStatus[wordnum] &= ~mask;
        }

        wordnum++;
        numBits -= (endbit - startbit);
        if (numBits >= 32UL)
        {
            startbit = 0;
            endbit   = 32UL;
            mask     = 0xFFFFFFFFUL;
        }
        else
        {
            startbit = 0;
            endbit   = numBits;
            mask     = ((uint32_t)(1UL << numBits) - 1UL);
        }
    }
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_Alloc
 ****************************************************************************//**
 *
 * This function allocates memory required for DMA buffers required by the
 * firmware application. The size parameter will be rounded up to the next
 * multiple of 64 as the allocator only provides memory blocks in multiples
 * of 64 bytes.
 *
 * \param mgr_p
 * Pointer to DMA buffer manager.
 *
 * \param bufferSize
 * Size of the DMA buffer to be allocated (in bytes).
 *
 * \return
 * Pointer to the DMA buffer.
 *******************************************************************************/
void *
Cy_HBDma_BufMgr_Alloc (
        cy_stc_hbdma_buf_mgr_t *mgr_p,
        uint32_t bufferSize)
{
#if CY_HBDMA_ERRORDETECTION
    HBDmaMemBlockInfo *block_p;
#endif /* CY_HBDMA_ERRORDETECTION */

    uint32_t tmp;
    uint32_t wordnum, bitnum;
    uint32_t count, start = 0;
    uint32_t blk_size = bufferSize;
    void *ptr = 0;

    /* Make sure the buffer manager has been initialized. */
    if ((mgr_p->startAddr == 0) || (mgr_p->regionSize == 0) || (mgr_p->usedStatus == 0))
    {
        return ptr;
    }

    /* Make sure the buffer size requested is valid. */
    if ((bufferSize == 0) || (bufferSize > CY_HBDMA_MAX_BUFFER_SIZE(true)))
    {
        return ptr;
    }

#if CY_HBDMA_ERRORDETECTION
    if (glBufMgrEnableChecks)
    {
        /* Allow for addition of header on top of a maximum sized allocation. */
        blk_size  = ROUND_UP(blk_size, 4);
        blk_size += sizeof (HBDmaMemBlockInfo) + sizeof (uint32_t);
    }
#endif /* CY_HBDMA_ERRORDETECTION */

    /* Find the number of cache lines required. The minimum size that can be handled is 2 cache lines. */
    bufferSize = (blk_size <= HBDMA_BUFFER_SIZE_UNIT) ? 2U :
        ((blk_size + HBDMA_BUFFER_SIZE_UNIT - 1U) / HBDMA_BUFFER_SIZE_UNIT);

    /* Search through the status array to find the first block that fits the need. */
    wordnum = mgr_p->searchPos;
    bitnum  = 0;
    count   = 0;
    tmp     = 0;

    /* Stop searching once we have checked all of the words. */
    while (tmp < mgr_p->statusSize)
    {
        if ((mgr_p->usedStatus[wordnum] & (1UL << bitnum)) == 0)
        {
            if (count == 0)
            {
                start = (wordnum << 5UL) + bitnum + 1UL;
            }

            count++;
            if (count == (uint16_t)(bufferSize + 1U))
            {
                /* The last bit corresponding to the allocated memory is left as zero.
                   This allows us to identify the end of the allocated block while freeing
                   the memory. We need to search for one additional zero while allocating
                   to account for this hack. */
                mgr_p->searchPos = wordnum;
                break;
            }
        }
        else
        {
            count = 0;
        }

        bitnum++;
        if (bitnum == 32UL)
        {
            bitnum = 0;
            wordnum++;
            tmp++;
            if (wordnum == mgr_p->statusSize)
            {
                /* Wrap back to the top of the array. */
                wordnum = 0;
                count   = 0;
            }
        }
    }

    if (count == (uint16_t)(bufferSize + 1))
    {
        /* Mark the memory region identified as occupied and return the pointer. */
        Cy_HBDma_BufMgr_SetStatus (mgr_p, start, bufferSize - 1U, true);
        ptr = (void *)(mgr_p->startAddr + (start << HBDMA_PAGE_TO_ADDR_SHIFT));

#if CY_HBDMA_ERRORDETECTION
        if (glBufMgrEnableChecks)
        {
            /* Store the header information used for leak and corruption checks. */
            block_p = (HBDmaMemBlockInfo *)ptr;
            block_p->alloc_id        = glBufAllocCnt++;
            block_p->alloc_size      = blk_size;
            block_p->prev_blk        = glBufInUseList;
            block_p->next_blk        = 0;
            block_p->start_sig       = CY_DMA_MEM_START_SIG;
            if (glBufInUseList != 0)
            {
                glBufInUseList->next_blk = block_p;
            }
            glBufInUseList           = block_p;

            /* Add the end block signature as a footer. */
            ((uint32_t *)block_p)[BYTE_TO_DWORD (blk_size) - 1] = CY_DMA_MEM_END_SIG;

            /* Update the return pointer to skip the header created. */
            ptr = (void *)((uint8_t *)block_p + sizeof (HBDmaMemBlockInfo));
        }
#endif /* CY_HBDMA_ERRORDETECTION */
    }

    return (ptr);
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_Free
 ****************************************************************************//**
 *
 * This function frees memory which was previously allocated using Cy_HBDma_BufMgr_Alloc.
 *
 * \param mgr_p
 * Pointer to DMA buffer manager.
 *
 * \param buffer_p
 * Pointer to the buffer to be freed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the free operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_BufMgr_Free (
        cy_stc_hbdma_buf_mgr_t *mgr_p,
        void *buffer_p)
{
#if CY_HBDMA_ERRORDETECTION
    HBDmaMemBlockInfo *block_p;
    uint32_t     *sig_p;
#endif /* CY_HBDMA_ERRORDETECTION */

    cy_en_hbdma_mgr_status_t retVal = CY_HBDMA_MGR_SUCCESS;
    uint32_t start, count;
    uint32_t wordnum, bitnum;

    if ((mgr_p == NULL) || (buffer_p == NULL) || ((uint8_t *)buffer_p < mgr_p->startAddr))
    {
        retVal = CY_HBDMA_MGR_BAD_PARAM;
    }
    else
    {
#if CY_HBDMA_ERRORDETECTION
        /* Update the structures used for leak checking. */
        if (glBufMgrEnableChecks)
        {
            block_p = (HBDmaMemBlockInfo *)((uint8_t *)buffer_p - sizeof (HBDmaMemBlockInfo));
            sig_p   = (uint32_t *)((uint8_t *)block_p + block_p->alloc_size - sizeof (uint32_t));
            if ((block_p->start_sig != CY_DMA_MEM_START_SIG) || (*sig_p != CY_DMA_MEM_END_SIG))
            {
                /* Notify the user that memory has been corrupted. */
                if (glBufBadCb != 0)
                {
                    glBufBadCb (buffer_p);
                }
            }

            glBufFreeCnt++;

            /* Update the in-use linked list to drop the freed-up block. */
            if (block_p->next_blk != 0)
                block_p->next_blk->prev_blk = block_p->prev_blk;
            if (block_p->prev_blk != 0)
                block_p->prev_blk->next_blk = block_p->next_blk;
            if (glBufInUseList == block_p)
            {
                glBufInUseList = block_p->prev_blk;
            }

            buffer_p = (void *)block_p;
        }
#endif /* CY_HBDMA_ERRORDETECTION */

        /* If the buffer address is within the range specified, count the number of consecutive ones and
           clear them. */
        start = (uint32_t)buffer_p;
        if ((start > (uint32_t)mgr_p->startAddr) && (start < ((uint32_t)mgr_p->startAddr + mgr_p->regionSize)))
        {
            start = ((start - (uint32_t)mgr_p->startAddr) >> HBDMA_PAGE_TO_ADDR_SHIFT);

            wordnum = (start >> 5U);
            bitnum  = (start & 0x1FU);
            count   = 0;

            while ((wordnum < mgr_p->statusSize) && ((mgr_p->usedStatus[wordnum] & (1 << bitnum)) != 0))
            {
                count++;
                bitnum++;
                if (bitnum == 32UL)
                {
                    bitnum = 0;
                    wordnum++;
                }
            }

            Cy_HBDma_BufMgr_SetStatus (mgr_p, start, count, false);

            /* Start the next buffer search at the top of the heap. This can help reduce fragmentation in cases where
               most of the heap is allocated and then freed as a whole. */
            mgr_p->searchPos = 0;
        }
    }

    return retVal;
}

#if CY_HBDMA_ERRORDETECTION

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_GetCounts
 ****************************************************************************//**
 *
 * Obtain allocation and free statistics from the DMA buffer manager.
 *
 * \param allocCnt_p
 * Parameter to be filled with number of Cy_HBDma_BufMgr_Alloc calls.
 *
 * \param freeCnt_p
 * Parameter to be filled with number of Cy_HBDma_BufMgr_Free calls.
 *
 * \return None
 *******************************************************************************/
void
Cy_HBDma_BufMgr_GetCounts (
        uint32_t *allocCnt_p,
        uint32_t *freeCnt_p)
{
    if (allocCnt_p != 0)
        *allocCnt_p = glBufAllocCnt;
    if (freeCnt_p != 0)
        *freeCnt_p = glBufFreeCnt;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_GetActiveList
 ****************************************************************************//**
 *
 * Get list of current in-use memory blocks. This can be used to check for memory
 * leaks leading to allocation failure at runtime.
 *
 * \return
 * Pointer to memory blocks allocated.
 *******************************************************************************/
HBDmaMemBlockInfo *
Cy_HBDma_BufMgr_GetActiveList (
        void)
{
    return glBufInUseList;
}

/*******************************************************************************
 * Function Name: Cy_HBDma_BufMgr_CorruptionCheck
 ****************************************************************************//**
 *
 * Check all in-use memory blocks for memory corruption. The
 * in-use memory list is traversed and each block is checked
 * for a valid start and end signature. The registered bad memory
 * callback function is called if any corruption is detected.
 *
 * \param mgr_p
 * Pointer to buffer manager context structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if no memory corruption is found.
 * CY_HBDMA_MGR_MEM_CORRUPTION if any corrupted memory is found.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_BufMgr_CorruptionCheck (
        cy_stc_hbdma_buf_mgr_t *mgr_p)
{
    HBDmaMemBlockInfo *block_p;
    uint32_t     *mem_p;

    if (mgr_p == NULL)
    {
        return CY_HBDMA_MGR_BAD_PARAM;
    }

    /* Run through all in-use memory blocks and send a callback for any blocks that do
       not match the start and end signatures.
     */
    block_p = glBufInUseList;
    while (block_p != 0)
    {
        if (((uint8_t *)block_p < mgr_p->startAddr) || ((uint8_t *)block_p >= (mgr_p->startAddr + mgr_p->regionSize)))
            return CY_HBDMA_MGR_MEM_CORRUPTION;

        mem_p = (uint32_t *)((uint8_t *)block_p + block_p->alloc_size - sizeof (uint32_t));
        if ((block_p->start_sig != CY_DMA_MEM_START_SIG) || (*mem_p != CY_DMA_MEM_END_SIG))
        {
            if (glBufBadCb != 0)
            {
                glBufBadCb ((void *)((uint8_t *)block_p + sizeof (HBDmaMemBlockInfo)));
            }

            /* Once we find any corruption, we cannot rely on the list pointers any more. */
            return CY_HBDMA_MGR_MEM_CORRUPTION;
        }

        /* Verify that the next block pointer is valid. */
        block_p = block_p->prev_blk;
    }

    return CY_HBDMA_MGR_SUCCESS;
}

#endif /* CY_HBDMA_ERRORDETECTION */

#if defined(__cplusplus)
}
#endif

/*[]*/

