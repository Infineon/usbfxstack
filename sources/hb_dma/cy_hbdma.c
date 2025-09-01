/***************************************************************************//**
* \file cy_hbdma.c
* \version 1.00
*
* Source code file for the High BandWidth DMA driver.
*
********************************************************************************
* \copyright
* Copyright (2025) Cypress Semiconductor Corporation
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

#include "cy_device.h"

#if defined (CY_IP_MXS40LVDS2USB32SS)

#include <string.h>
#include "cy_pdl.h"
#include "cy_hbdma.h"
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define CY_HBDMA_SCK_TR_OUT_DELAY_VAL   (0x27UL)

/* Debug option: Set to 1 to disable support for egress transfer from buffers not aligned to 16 bytes. */
#define UNALIGNED_READ_DISABLE          (0u)

/* Debug option: Set to 1 to disable descriptor prefetching by DMA adapters. */
#define DISABLE_DESCRIPTOR_PREFETCH     (0u)

/*******************************************************************************
 * Function name: Cy_HBDma_Init
 ****************************************************************************//**
 *
 * Initializes the High BandWidth DMA driver. Pointers to the USB32DEV and
 * LVDS control register blocks need to be passed as parameters while initializing
 * the driver.
 *
 * \param lvds_base
 * Pointer to the LVDS IP control registers.
 *
 * \param usbss_base
 * Pointer to the USB32DEV IP control registers.
 *
 * \param pContext
 * Pointer to the HBDma driver context structure.
 *
 * \param usb_egrs_fq_depth
 * Fetch queue depth setting to be set for the USB32DEV egress DMA adapter.
 *
 * \param usb_egrs_rq_ctrl
 * Read queue empty threshold to be set for the USB32DEV egress DMA adapter.
 *
 * \return
 * CY_HBDMA_SUCCESS if driver initialization is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_Init (
        LVDSSS_LVDS_Type *lvds_base,
        USB32DEV_Type *usbss_base,
        cy_stc_hbdma_context_t *pContext,
        uint32_t usb_egrs_fq_depth,
        uint32_t usb_egrs_rq_ctrl)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    uint32_t sckIndex;

    (void)usb_egrs_rq_ctrl;

    if (pContext == NULL) {
        status = CY_HBDMA_BAD_PARAM;
    } else {
        if (lvds_base != NULL) {
            pContext->lvdsEnabled = true;

            pContext->LVDSAD0_SCK     = (HBDMA_SCK_Type *)lvds_base->ADAPTER_DMA[0].SCK;
            pContext->LVDSAD0_SCK_GBL = (HBDMA_SCK_GBL_Type *)&(lvds_base->ADAPTER_DMA[0].SCK_GBL);
            pContext->LVDSAD1_SCK     = (HBDMA_SCK_Type *)lvds_base->ADAPTER_DMA[1].SCK;
            pContext->LVDSAD1_SCK_GBL = (HBDMA_SCK_GBL_Type *)&(lvds_base->ADAPTER_DMA[1].SCK_GBL);

            /* Enable the DMA adapters. */
            pContext->LVDSAD0_SCK_GBL->ADAPTER_CTRL = (LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_ADAPTER_EN_Msk |
#if UNALIGNED_READ_DISABLE
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk |
#endif /* UNALIGNED_READ_DISABLE */
                    (CY_HBDMA_SCK_TR_OUT_DELAY_VAL << LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_TR_OUT_DELAY_Pos));
            pContext->LVDSAD1_SCK_GBL->ADAPTER_CTRL = (LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_ADAPTER_EN_Msk |
#if UNALIGNED_READ_DISABLE
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk |
#endif /* UNALIGNED_READ_DISABLE */
                    (CY_HBDMA_SCK_TR_OUT_DELAY_VAL << LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_TR_OUT_DELAY_Pos));

#if DISABLE_DESCRIPTOR_PREFETCH
            /* Disable descriptor prefetch on the LVDS DMA adapter. */
            pContext->LVDSAD0_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;
            pContext->LVDSAD1_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;
#endif /* DISABLE_DESCRIPTOR_PREFETCH */

            /* Run through all the sockets in the LVDS block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->LVDSAD0_SCK[sckIndex].SCK_STATUS &= ~LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
                pContext->LVDSAD1_SCK[sckIndex].SCK_STATUS &= ~LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }
        }

        if (usbss_base != NULL) {
            pContext->usb3Enabled = true;

            pContext->USBIN_SCK = (HBDMA_SCK_Type *)usbss_base->USB32DEV_ADAPTER[0].SCK;
            pContext->USBIN_SCK_GBL = (HBDMA_SCK_GBL_Type *)&(usbss_base->USB32DEV_ADAPTER[0].SCK_GBL);

            /* Enable the DMA adapter. As this is INGRESS only, we can disable unaligned data read support. */
            pContext->USBIN_SCK_GBL->ADAPTER_CTRL = (USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_ADAPTER_EN_Msk |
                    USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk |
                    (CY_HBDMA_SCK_TR_OUT_DELAY_VAL << USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_TR_OUT_DELAY_Pos));

#if DISABLE_DESCRIPTOR_PREFETCH
            /* Disable descriptor prefetch on the USB ingress DMA adapter. */
            pContext->USBIN_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;
#endif /* DISABLE_DESCRIPTOR_PREFETCH */

            /* Run through all the sockets in the USB-Ingress block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->USBIN_SCK[sckIndex].SCK_STATUS &= ~USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }

            pContext->USBEG_SCK = (HBDMA_SCK_Type *)usbss_base->USB32DEV_ADAPTER[1].SCK;
            pContext->USBEG_SCK_GBL = (HBDMA_SCK_GBL_Type *)&(usbss_base->USB32DEV_ADAPTER[1].SCK_GBL);

            /* Enable the DMA adapter. */
            pContext->USBEG_SCK_GBL->ADAPTER_CTRL = (USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_ADAPTER_EN_Msk |
#if UNALIGNED_READ_DISABLE
                    USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk |
#endif /* UNALIGNED_READ_DISABLE */
                    (CY_HBDMA_SCK_TR_OUT_DELAY_VAL << USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_TR_OUT_DELAY_Pos));
            if (usb_egrs_fq_depth != 0) {
                uint32_t adpCtrl = (USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_ADAPTER_EN_Msk |
#if UNALIGNED_READ_DISABLE
                        USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk |
#endif /* UNALIGNED_READ_DISABLE */
                        (CY_HBDMA_SCK_TR_OUT_DELAY_VAL << USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_TR_OUT_DELAY_Pos) |
                        (usb_egrs_fq_depth << USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_FQ_DEPTH_CTRL_Pos));
                Cy_SysLib_DelayUs(10);
                pContext->USBEG_SCK_GBL->ADAPTER_CTRL = adpCtrl;
                DBG_HBDMA_INFO("Setting USB_EGR_ADAP_CTRL=%x\r\n", adpCtrl);
            }

#if DISABLE_DESCRIPTOR_PREFETCH
            /* Disable descriptor prefetch on the USB egress DMA adapter. */
            pContext->USBEG_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;
#endif /* DISABLE_DESCRIPTOR_PREFETCH */

            /* Run through all the sockets in the USB-Egress block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->USBEG_SCK[sckIndex].SCK_STATUS &= ~USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }
        }

        /* Enable 64 KB DMA buffer support if this is not A0 silicon. */
        if (Cy_SysLib_GetDeviceRevision() == 0x11) {
            MAIN_REG->CTRL &= ~MAIN_REG_CTRL_BUFFSIZE_64KB_EN_Msk;
        } else {
            MAIN_REG->CTRL |= MAIN_REG_CTRL_BUFFSIZE_64KB_EN_Msk;
        }
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_DeInit
 ****************************************************************************//**
 *
 * De-initialize the High BandWidth DMA adapter. This function also disables
 * the DMA adapters associated with the LVDS and USB32DEV IP blocks.
 *
 * \param pContext
 * Pointer to the HBDma driver context structure.
 *
 * \return
 * void
 *******************************************************************************/
void
Cy_HBDma_DeInit (
        cy_stc_hbdma_context_t *pContext)
{
    uint32_t sckIndex;

    if (pContext != NULL)
    {
        /* Walk through each adapter, disable all sockets and then disable the adapter. */
        if (pContext->usb3Enabled)
        {
            /* Run through all the sockets in the USB-Ingress block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->USBIN_SCK[sckIndex].SCK_STATUS &= ~USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }

            /* Disable the DMA adapter. */
            pContext->USBIN_SCK_GBL->ADAPTER_CTRL = 0;

            /* Run through all the sockets in the USB-Egress block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->USBEG_SCK[sckIndex].SCK_STATUS &= ~USB32DEV_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }

            /* Disable the DMA adapter. */
            pContext->USBEG_SCK_GBL->ADAPTER_CTRL = 0;
        }

        if (pContext->lvdsEnabled)
        {
            /* Run through all the sockets in the LVDS block and disable them. */
            for (sckIndex = 0; sckIndex < CY_HBDMA_SOCK_PER_ADAPTER; sckIndex++)
            {
                pContext->LVDSAD0_SCK[sckIndex].SCK_STATUS &= ~LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
                pContext->LVDSAD1_SCK[sckIndex].SCK_STATUS &= ~LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
            }

            /* Disable the DMA adapters. */
            pContext->LVDSAD0_SCK_GBL->ADAPTER_CTRL = 0;
            pContext->LVDSAD1_SCK_GBL->ADAPTER_CTRL = 0;
        }

        /* Clear the state fields in the context structure. */
        pContext->usb3Enabled = false;
        pContext->lvdsEnabled = false;
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_GetDescriptor
 ****************************************************************************//**
 *
 * Reads and returns the contents of the High BandWidth DMA descriptor with the
 * specified index.
 *
 * \param dscrIndex
 * Index of the DMA descriptor to be fetched.
 *
 * \param dscr_p
 * Pointer to structure to be filled with the descriptor information.
 *
 * \return
 * CY_HBDMA_SUCCESS if descriptor fetch is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_GetDescriptor (
        uint16_t dscrIndex,
        cy_stc_hbdma_desc_t *pDscr)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    cy_stc_hbdma_desc_t *pSrc;

    if ((pDscr == NULL) || (dscrIndex >= CY_HBDMA_MAX_DSCR_CNT))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
#if CY_CPU_CORTEX_M4
        pSrc = (cy_stc_hbdma_desc_t *)(CY_HBDMA_GET_DESC_ADDR(dscrIndex));

        /* Make sure that the AHB read cache is flushed so that we do not get stale data. */
        Cy_HBDma_EvictReadCache(true);

        /* Upper bits of buffer address might have been masked in the descriptor and need to be restored. */
        pDscr->pBuffer = (uint8_t *)((uint32_t)(pSrc->pBuffer) | 0x1C000000UL);
        pDscr->sync    = pSrc->sync;
        pDscr->chain   = pSrc->chain;
        pDscr->size    = pSrc->size;
#else
        /* We use DW1.channel0 set to the same priority as other DW1 channels to read the descriptor.
         * This ensures that the read does not interrupt another ongoing DW1 read operation.
         * We also make sure that a complete 32-byte region is read from the DMA RAM and then
         * extract the required fields.
         */
        uint32_t dw_descr[6];
        uint32_t dscrtmp[8];
        uint32_t intMask;

        intMask = Cy_SysLib_EnterCriticalSection();
        pSrc = (cy_stc_hbdma_desc_t *)(CY_HBDMA_GET_DESC_ADDR(dscrIndex & 0xFFFE));

        dw_descr[0] = 0x61000068UL;
        dw_descr[1] = (uint32_t)pSrc;
        dw_descr[2] = (uint32_t)dscrtmp;
        dw_descr[3] = 0x07001001UL;
        dw_descr[4] = 0x00000000UL;
        dw_descr[5] = 0x00000000UL;

        /* Make sure the DW1 IP is enabled. */
        DW1->CTL |= 0x80000000UL;

        /* Clear stale channel interrupt. */
        DW1_CH_STRUCT0->INTR        = 0x00000001UL;
        DW1_CH_STRUCT0->INTR_MASK   = 0;

        /* Enable channel with above descriptor and set lowest priority. */
        DW1_CH_STRUCT0->CH_CURR_PTR = (uint32_t)dw_descr;
        DW1_CH_STRUCT0->CH_IDX      = 0;
        DW1_CH_STRUCT0->CH_CTL      = 0x80000300UL;

        /* Trigger the DataWire channel. */
        Cy_TrigMux_SwTrigger(TRIG_OUT_MUX_1_PDMA1_TR_IN0, CY_TRIGGER_TWO_CYCLES);

        /* Wait until the transfer is complete and make sure interrupt is cleared. */
        while (DW1_CH_STRUCT0->INTR == 0);
        DW1_CH_STRUCT0->INTR = 0x00000001UL;

        if ((dscrIndex & 0x01) != 0) {
            /* Odd descriptor: Take values from second half of the buffer. */
            pDscr->pBuffer = (uint8_t *)(dscrtmp[4] | 0x1C000000UL);
            pDscr->sync    = dscrtmp[5];
            pDscr->chain   = dscrtmp[6];
            pDscr->size    = dscrtmp[7];
        } else {
            /* Even descriptor: Take values from first half of the buffer. */
            pDscr->pBuffer = (uint8_t *)(dscrtmp[0] | 0x1C000000UL);
            pDscr->sync    = dscrtmp[1];
            pDscr->chain   = dscrtmp[2];
            pDscr->size    = dscrtmp[3];
        }

        Cy_SysLib_ExitCriticalSection(intMask);
#endif /* CY_CPU_CORTEX_M4 */
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetDescriptor
 ****************************************************************************//**
 *
 * Updates the High BandWidth DMA descriptor at a specified index with the
 * desired values.
 *
 * \param dscrIndex
 * DMA descriptor index.
 *
 * \param dscr_p
 * Structure containing the values to be updated in the DMA descriptor.
 *
 * \return
 * CY_HBDMA_SUCCESS if descriptor update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetDescriptor (
        uint16_t dscrIndex,
        cy_stc_hbdma_desc_t *pDscr)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    cy_stc_hbdma_desc_t *pTgt;

    if ((pDscr == NULL) || (dscrIndex >= CY_HBDMA_MAX_DSCR_CNT))
    {
        DBG_HBDMA_ERR("SetDesc Error\r\n");
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        pTgt = (cy_stc_hbdma_desc_t *)(CY_HBDMA_GET_DESC_ADDR(dscrIndex));
        /* Clear the upper bits of the buffer address to work-around RTL bug. */
        pTgt->pBuffer = (uint8_t *)((uint32_t)(pDscr->pBuffer) & 0x80FFFFFFUL);
        pTgt->sync    = pDscr->sync;
        pTgt->chain   = pDscr->chain;
        pTgt->size    = pDscr->size;
        __DMB();
        __DSB();
    }

    return status;
}

/*******************************************************************************
 * Function Name: GetSocketPtrByIndex
 ****************************************************************************//**
 *
 * Retrieve pointer to the control registers associated with a socket.
 *
 * \param pContext
 * Pointer to High BandWidth driver context structure.
 *
 * \param sock_id
 * ID of socket to be retrieved.
 *
 * \return
 * Pointer to the socket control registers if valid, NULL otherwise.
 *******************************************************************************/
static HBDMA_SCK_Type *
GetSocketPtrByIndex (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id)
{
    HBDMA_SCK_Type *socket = NULL;

    if (sock_id <= CY_HBDMA_LVDS_SOCKET_15)
    {
        socket = &(pContext->LVDSAD0_SCK[sock_id - CY_HBDMA_LVDS_SOCKET_00]);
    }
    if ((sock_id >= CY_HBDMA_LVDS_SOCKET_16) && (sock_id <= CY_HBDMA_LVDS_SOCKET_31))
    {
        socket = &(pContext->LVDSAD1_SCK[sock_id - CY_HBDMA_LVDS_SOCKET_16]);
    }
    if ((sock_id >= CY_HBDMA_USBIN_SOCKET_00) && (sock_id <= CY_HBDMA_USBIN_SOCKET_15))
    {
        socket = &(pContext->USBIN_SCK[sock_id - CY_HBDMA_USBIN_SOCKET_00]);
    }
    if ((sock_id >= CY_HBDMA_USBEG_SOCKET_00) && (sock_id <= CY_HBDMA_USBEG_SOCKET_15))
    {
        socket = &(pContext->USBEG_SCK[sock_id - CY_HBDMA_USBEG_SOCKET_00]);
    }

    return socket;
}

/*******************************************************************************
 * Function name: Cy_HBDma_GetSocketStatus
 ****************************************************************************//**
 *
 * Read and return the contents of all control and status registers associated
 * with a High BandWidth DMA socket. This function can be used to check the
 * current socket status, and also as the first step during a read-modify-write
 * update of the socket configuration.
 *
 * \param pContext
 * Pointer to the driver context structure.
 *
 * \param sock_id
 * ID of the socket whose status is to be retrieved.
 *
 * \param sckConf_p
 * Pointer to structure to be filled with the socket status.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket status read is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_GetSocketStatus (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        cy_stc_hbdma_sock_t *pSckConf)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;

    if ((pContext == NULL) || (pSckConf == NULL) ||
            (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        pSckConf->actDscrIndex  = socket->SCK_DSCR;
        pSckConf->reqXferSize   = socket->SCK_SIZE;
        pSckConf->compXferCount = socket->SCK_COUNT;
        pSckConf->status        = socket->SCK_STATUS;
        pSckConf->intrStatus    = socket->SCK_INTR;
        pSckConf->intrMask      = socket->SCK_INTR_MASK;

        pSckConf->actDscr.pBuffer = (uint8_t *)socket->DSCR_BUFFER;
        pSckConf->actDscr.sync    = socket->DSCR_SYNC;
        pSckConf->actDscr.chain   = socket->DSCR_CHAIN;
        pSckConf->actDscr.size    = socket->DSCR_SIZE;
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetSocketConfig
 ****************************************************************************//**
 *
 * Updates the configuration of a HBDma socket with the desired fields from the
 * config structure passed in as parameter. This API can be used to update the
 * current descriptor, the interrupt masks and the various config bits associated
 * with the socket functionality.
 *
 * \param pContext
 * Pointer to the driver context structure.
 *
 * \param sock_id
 * ID of the socket whose configuration is to be updated.
 *
 * \param conf
 * Pointer to the desired socket configuration.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetSocketConfig (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        cy_stc_hbdma_sockconfig_t *conf)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;

    if ((pContext == NULL) || (conf == NULL) ||
            (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        socket->SCK_DSCR      = conf->actDscrIndex;
        socket->SCK_SIZE      = conf->reqXferSize;
        socket->SCK_COUNT     = 0UL;
        socket->SCK_INTR      = 0xFFFFFFFFUL;
        socket->SCK_INTR_MASK = conf->intrMask;
        socket->SCK_STATUS    = conf->status & CY_HBDMA_SOCK_STATUS_WR_MASK;
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_UpdateSockIntrMask
 ****************************************************************************//**
 *
 * Special function provided to update only the interrupt mask associated
 * with a HBDma socket. This is a subset of the functionality provided by the
 * Cy_HBDma_SetSocketConfig API.
 *
 * \param pContext
 * Pointer to the HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be updated.
 *
 * \param intrMap
 * Bit map specifying the interrupt mask bits to be updated.
 *
 * \param enable
 * Whether the interrupts specified in intrMap are to be enabled or disabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_UpdateSockIntrMask (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        uint32_t intrMap,
        bool enable)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;

    if ((pContext == NULL) || (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        if (enable)
        {
            socket->SCK_INTR_MASK |= intrMap;
        }
        else
        {
            socket->SCK_INTR_MASK &= ~intrMap;
        }
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SocketEnable
 ****************************************************************************//**
 *
 * Function to enable a High BandWidth DMA socket after it has been configured
 * using the Cy_HBDma_SetSocketConfig API. If the socket has not been configured
 * properly before it is enabled, it can result in unexpected transfers or
 * errors.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be enabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketEnable (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;
    uint32_t pollCnt = 1000;

    if ((pContext == NULL) || (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        /* Enable the socket and wait until it becomes enabled. */
        socket->SCK_STATUS |= LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk;
        while (((socket->SCK_STATUS & LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_ENABLED_Msk) == 0) && (pollCnt--));
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SocketDisable
 ****************************************************************************//**
 *
 * Function to disable a High BandWidth DMA socket. This function causes any
 * ongoing DMA transfer on the socket to be aborted and blocks until the socket
 * has moved into the disabled state.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be disabled.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketDisable (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;
    uint32_t pollCnt = 1000;

    if ((pContext == NULL) || (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        /* Clear all socket interrupts first. */
        socket->SCK_INTR = 0xFFFFFFFFUL;

        /* Disable the socket and wait until it becomes disabled. */
        socket->SCK_STATUS &= ~(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_GO_ENABLE_Msk |
                                LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_WRAPUP_Msk);
        while (((socket->SCK_STATUS & LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_ENABLED_Msk) != 0) && (pollCnt--));
    }
    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SocketSetWrapUp
 ****************************************************************************//**
 *
 * This function sets the bit that forces a socket with a partially filled buffer
 * to wrap up the buffer. The function does not wait for the socket to wrap up.
 * This API should be called after ensuring that the socket in question is not
 * actively receiving data. Otherwise, this can result in data loss.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to be wraped up.
 *
 * \return
 * CY_HBDMA_SUCCESS if socket update is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SocketSetWrapUp (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    HBDMA_SCK_Type *socket = NULL;
    
    if ((pContext == NULL) || (!CY_HBDMA_IS_SOCKET_VALID(sock_id)))
    {
        status = CY_HBDMA_BAD_PARAM;
    }
    else
    {
        socket = GetSocketPtrByIndex(pContext, sock_id);
        CY_ASSERT_L1(socket != NULL);

        /* Set the socket wrap up bit. */
        socket->SCK_STATUS |= (LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_WRAPUP_Msk);
    }
    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetInterruptCallback
 ****************************************************************************//**
 *
 * Function that registers a callback which can be called the HBDma driver to
 * provide notification of socket interrupts. The callback will be registered by
 * the HBDma manager layer and will be called when the socket interrupts are
 * serviced through the Cy_HBDma_HandleInterrupts function.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param cb_p
 * Callback function pointer.
 *
 * \param cbContext
 * Opaque user data to be passed to the callback function.
 *
 * \return
 * CY_HBDMA_SUCCESS if callback registration is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SetInterruptCallback (
        cy_stc_hbdma_context_t *pContext,
        cy_cb_hbdma_intr_callback_t cb_p,
        void *cbContext)
{
    cy_en_hbdma_status_t status = CY_HBDMA_BAD_PARAM;

    if (pContext != NULL)
    {
        pContext->intrCallback = cb_p;
        pContext->cbContext    = cbContext;

        status = CY_HBDMA_SUCCESS;
    }

    return status;
}

/*******************************************************************************
 * Function Name: GetHBDmaAdapterPtr
 ****************************************************************************//**
 *
 * Retrieve pointer to the control registers associated with a DMA adapter.
 *
 * \param pContext
 * Pointer to High BandWidth driver context structure.
 *
 * \param adapter
 * ID of the DMA adapter to be retrieved.
 *
 * \return
 * Pointer to the adapter control registers if valid, NULL otherwise.
 *******************************************************************************/
static HBDMA_SCK_GBL_Type *
GetHBDmaAdapterPtr(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_adapter_id_t adapter)
{
    HBDMA_SCK_GBL_Type *pAdapter = NULL;

    if (pContext != NULL)
    {
        switch (adapter)
        {
            case CY_HBDMA_ADAP_LVDS_0:
                pAdapter = pContext->LVDSAD0_SCK_GBL;
                break;

            case CY_HBDMA_ADAP_LVDS_1:
                pAdapter = pContext->LVDSAD1_SCK_GBL;
                break;

            case CY_HBDMA_ADAP_USB_IN:
                pAdapter = pContext->USBIN_SCK_GBL;
                break;

            case CY_HBDMA_ADAP_USB_EG:
                pAdapter = pContext->USBEG_SCK_GBL;
                break;

            default:
                break;
        }
    }

    return pAdapter;
}

/*******************************************************************************
 * Function Name: HbDma_Get_TrigMux_By_Socket
 ****************************************************************************//**
 *
 * Retrieve the trigger MUX index corresponding to the specified socket.
 *
 * \param sock_id
 * ID of the socket to be queried.
 *
 * \param adapter
 * Return parameter to be filled with the MUX index.
 *
 * \return
 * CY_HBDMA_SUCCESS if the query function is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed are invalid.
 *******************************************************************************/
static cy_en_hbdma_status_t
HbDma_Get_TrigMux_By_Socket(
        cy_hbdma_socket_id_t sock_id,
        uint8_t *mux_id_p)
{
    uint8_t mux_id = 0u;

    if ((!CY_HBDMA_IS_SOCKET_VALID(sock_id)) || (sock_id >= CY_HBDMA_VIRT_SOCKET_RD) || (mux_id_p == 0))
    {
        return CY_HBDMA_BAD_PARAM;
    }

    /* Muxes 0 to 15 are used for USB Ingress Sockets. */
    if ((sock_id >= CY_HBDMA_USBIN_SOCKET_00) && (sock_id <= CY_HBDMA_USBIN_SOCKET_15))
    {
        mux_id = (sock_id - CY_HBDMA_USBIN_SOCKET_00);
    }

    /* Muxes 16 to 31 are used for USB Egress Sockets. */
    if ((sock_id >= CY_HBDMA_USBEG_SOCKET_00) && (sock_id <= CY_HBDMA_USBEG_SOCKET_15))
    {
        mux_id = 16U + (sock_id - CY_HBDMA_USBEG_SOCKET_00);
    }

    /* Muxes 32 to 63 are used for LVDS Sockets. */
    if ((sock_id >= CY_HBDMA_LVDS_SOCKET_00) && (sock_id <= CY_HBDMA_LVDS_SOCKET_31))
    {
        mux_id = 32U + (sock_id - CY_HBDMA_LVDS_SOCKET_00);
    }

    *mux_id_p = mux_id;
    return CY_HBDMA_SUCCESS;
}

/*******************************************************************************
 * Function Name: HbDma_Get_MuxInput_By_Socket
 ****************************************************************************//**
 *
 * Retrieve the trigger MUX input index corresponding to the specified socket.
 *
 * \param sock_id
 * ID of the socket to be queried.
 *
 * \param adapter
 * Return parameter to be filled with the trigger MUX input index.
 *
 * \return
 * CY_HBDMA_SUCCESS if the query function is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed are invalid.
 *******************************************************************************/
static cy_en_hbdma_status_t
HbDma_Get_MuxInput_By_Socket (
        cy_hbdma_socket_id_t sock_id,
        uint8_t *mux_inp_p)
{
    uint8_t mux_inp = 0u;

    if ((!CY_HBDMA_IS_SOCKET_VALID(sock_id)) || (sock_id >= CY_HBDMA_VIRT_SOCKET_RD) || (mux_inp_p == 0))
    {
        return CY_HBDMA_BAD_PARAM;
    }

    /* Inputs 32 to 47 come from USB Ingress Sockets. */
    if ((sock_id >= CY_HBDMA_USBIN_SOCKET_00) && (sock_id <= CY_HBDMA_USBIN_SOCKET_15))
    {
        mux_inp = 32U + (sock_id - CY_HBDMA_USBIN_SOCKET_00);
    }

    /* Inputs 48 to 63 come from USB Egress Sockets. */
    if ((sock_id >= CY_HBDMA_USBEG_SOCKET_00) && (sock_id <= CY_HBDMA_USBEG_SOCKET_15))
    {
        mux_inp = 48U + (sock_id - CY_HBDMA_USBEG_SOCKET_00);
    }

    /* Inputs 64 to 95 come from LVDS sockets. */
    if ((sock_id >= CY_HBDMA_LVDS_SOCKET_00) && (sock_id <= CY_HBDMA_LVDS_SOCKET_31))
    {
        mux_inp = 64U + (sock_id - CY_HBDMA_LVDS_SOCKET_00);
    }

    *mux_inp_p = mux_inp;
    return CY_HBDMA_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_HBDma_SendSocketEvent
 ****************************************************************************//**
 *
 * Function to send an event notification to the specified socket. Either a
 * produce event or a consume event can be sent based on the use case and type
 * of the socket.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param sock_id
 * ID of the socket to which the event is to be sent.
 *
 * \param isProduceEvent
 * Set to true for sending a produce event, false for consume event.
 *
 * \return
 * CY_HBDMA_SUCCESS if sending the event is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_SendSocketEvent (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id,
        bool isProduceEvent)
{
    cy_en_hbdma_status_t status = CY_HBDMA_SUCCESS;
    uint32_t intMask;
    uint8_t  mux_id, mux_inp;

    HbDma_Get_TrigMux_By_Socket(sock_id, &mux_id);
    HbDma_Get_MuxInput_By_Socket(sock_id, &mux_inp);

    /*
     * Connect the trigger input of the target socket to its own trigger output.
     * Then assert the trigger in edge triggered mode.
     */
    intMask = Cy_SysLib_EnterCriticalSection();
    MAIN_REG->TR_GR[0].TR_CTL[mux_id] = mux_inp;
    MAIN_REG->TR_CMD = (0xA0000000UL | mux_inp);
    Cy_SysLib_ExitCriticalSection(intMask);

    return status;
}

/*******************************************************************************
 * Function name: Cy_HbDma_ConnectEventTrigger
 ****************************************************************************//**
 *
 * Function to connect the event trigger output from one socket to the input of
 * another socket. The same mechanism is used for sending produce events from an
 * ingress socket to an egress socket as well as for sending consume events from
 * an egress socket to an ingress socket.
 *
 * If the DMA configuration used is a 1:2 or a 2:1 channel, trigger outputs from
 * multiple sockets can be connected to the input of one socket. The trigNum
 * parameter is used to specify the index of the event trigger input on the
 * destination side.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param src_sock
 * ID of the socket which generates the produce or consume event trigger.
 *
 * \param dst_sock
 * ID of the socket which receives the event trigger.
 *
 * \param trigNum
 * Index of the trigger input on the destination socket. Can be 0 or 1.
 *
 * \return
 * CY_HBDMA_SUCCESS if the trigger connection is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HbDma_ConnectEventTrigger (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t src_sock,
        cy_hbdma_socket_id_t dst_sock,
        uint8_t trigNum)
{
    cy_en_hbdma_status_t status = CY_HBDMA_BAD_PARAM;
    uint8_t mux_id, mux_inp;

    if (pContext != NULL)
    {
        if (
                (HbDma_Get_TrigMux_By_Socket(dst_sock, &mux_id) == CY_HBDMA_SUCCESS) &&
                (HbDma_Get_MuxInput_By_Socket(src_sock, &mux_inp) == CY_HBDMA_SUCCESS)
           )
        {
            if (trigNum == 0)
            {
                MAIN_REG->TR_GR[0].TR_CTL[mux_id] = mux_inp;
            }
            else
            {
                MAIN_REG->TR_GR[0].TR_CTL[mux_id] |= (mux_inp << 7);
            }
            status = CY_HBDMA_SUCCESS;
        }
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HbDma_DisconnectEventTriggers
 ****************************************************************************//**
 *
 * Function to break all event trigger connections coming to a socket.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param dst_sock
 * ID of the socket which receives the event trigger.
 *
 * \return
 * CY_HBDMA_SUCCESS if the operation is successful.
 * CY_HBDMA_BAD_PARAM if the parameters passed in are invalid.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HbDma_DisconnectEventTriggers (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t dst_sock)
{
    cy_en_hbdma_status_t status = CY_HBDMA_BAD_PARAM;
    uint8_t mux_id;

    if (pContext != NULL)
    {
        if (HbDma_Get_TrigMux_By_Socket(dst_sock, &mux_id) == CY_HBDMA_SUCCESS)
        {
            MAIN_REG->TR_GR[0].TR_CTL[mux_id] = 0;
            status = CY_HBDMA_SUCCESS;
        }
    }

    return status;
}

/*******************************************************************************
 * Function Name: HandleSocketInterrupt
 ****************************************************************************//**
 *
 * Handle pending interrupts on a specific High BandWidth DMA socket.
 *
 * \param pContext
 * Pointer to High BandWidth driver context structure.
 *
 * \param sock_ids
 * Socket index on which interrupts are to be handled.
 *
 * \return
 * true if further interrupt processing is to be stopped.
 *******************************************************************************/
static bool
HandleSocketInterrupt(
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_socket_id_t sock_id)
{
    uint32_t actIntr;
    uint32_t curDscr;
    HBDMA_SCK_Type *pSock;
    bool cbReturn = false;

    /* Ensure that the parameters passed in are valid. */
    CY_ASSERT_L1((pContext != NULL) && (CY_HBDMA_IS_SOCKET_VALID(sock_id)));

    pSock  = GetSocketPtrByIndex(pContext, sock_id);
    actIntr = pSock->SCK_INTR & pSock->SCK_INTR_MASK;

    if (actIntr != 0)
    {
        /* Clear the active interrupt first. Some interrupts like PRODUCE_EVENT may fire repeatedly. */
        pSock->SCK_INTR = actIntr;

        /* Current descriptor for the socket is to be read after interrupt is cleared. */
        curDscr = pSock->SCK_DSCR & USB32DEV_ADAPTER_DMA_SCK_DSCR_DSCR_NUMBER_Msk;

        /* Send callback notifications for interrupts of interest. */
        if (pContext->intrCallback != NULL)
        {
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_PRODUCE_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_CONSUME_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_STALL_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_STALL_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_SUSPEND_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_SUSPEND_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_ERROR_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_ERROR_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_TRANS_DONE_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_XFERDONE_EVT, curDscr, pContext->cbContext);
            }
            if ((actIntr & LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_EVENT_RCVD_Msk) != 0)
            {
                cbReturn = pContext->intrCallback(sock_id, CY_HBDMA_SOCK_EVT_RCVD, curDscr, pContext->cbContext);
            }
        }
    }

    return cbReturn;
}

const uint8_t _AdapterToDmaPIDMap[] = {
    0xFF,               /* 0: Invalid */
    0x01,               /* CY_HBDMA_ADAP_LVDS_0 */
    0x02,               /* CY_HBDMA_ADAP_LVDS_1 */
    0x04,               /* CY_HBDMA_ADAP_USB_IN */
    0x03                /* CY_HBDMA_ADAP_USB_EG */
};

/*******************************************************************************
 * Function name: Cy_HBDma_HandleInterrupts
 ****************************************************************************//**
 *
 * Function which handles the interrupts generated by any of the sockets associated
 * with a High BandWidth DMA adapter. This function is expected to be called from
 * the ISR for the corresponding interrupt vector.
 *
 * The interrupt handling in the driver is only a top-half implementation which
 * notifies the HBDma manager layer and then clears the interrupt.
 *
 * \param pContext
 * Pointer to HBDMA driver context structure.
 *
 * \param adapter
 * Identifies the adapter for which the interrupts are to be serviced.
 *
 * \return
 * CY_HBDMA_SUCCESS if the adapter specified is valid.
 * CY_HBDMA_BAD_PARAM if invalid parameters are passed in.
 *******************************************************************************/
cy_en_hbdma_status_t
Cy_HBDma_HandleInterrupts (
        cy_stc_hbdma_context_t *pContext,
        cy_hbdma_adapter_id_t adapter)
{
    cy_en_hbdma_status_t status = CY_HBDMA_BAD_PARAM;
    HBDMA_SCK_GBL_Type *adap_p;
    cy_hbdma_socket_id_t sockId;
    uint32_t intrStat;
    uint32_t sockNum;

    if ((pContext != NULL) && (adapter <= CY_HBDMA_ADAP_USB_EG))
    {
        adap_p = GetHBDmaAdapterPtr(pContext, adapter);
        if (adap_p != NULL)
        {
            /* Check whether the adapter indicates pending interrupts on any of the sockets. */
            intrStat = adap_p->SCK_INTR;
            if (intrStat != 0)
            {
                for (sockNum = 0; sockNum < CY_HBDMA_SOCK_PER_ADAPTER; sockNum++)
                {
                    if ((intrStat & (1U << sockNum)) != 0)
                    {
                        sockId = (cy_hbdma_socket_id_t)((_AdapterToDmaPIDMap[adapter] << 4U) + sockNum);
                        if (HandleSocketInterrupt(pContext, sockId))
                            break;
                    }
                }
            }

            status = CY_HBDMA_SUCCESS;
        }
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_EvictReadCache
 ****************************************************************************//**
 *
 * Evicts the read cache used while masters are using either the Fast AHB
 * bus (Cortex-M4) or the Slow AHB bus (Cortex-M0+, DataWire, DMAC or Crypto)
 * to read content from the High BandWidth RAM area. This operation is required
 * whenever a master is reading from the RAM while there is a possibility of
 * the corresponding memory having been modified by DMA operations.
 *
 * \param isCm4Access
 * Whether the read cache used by Cortex-M4 is to be cleared. Set to false
 * for all other masters.
 *
 *******************************************************************************/
void Cy_HBDma_EvictReadCache (
        bool isCm4Access)
{
    /* Request cache eviction and wait till it is completed. */
    if (isCm4Access) {
        MAIN_REG->CTRL |= MAIN_REG_CTRL_EVICT_FAST_AHB_RD_CACHE_Msk;
        while ((MAIN_REG->CTRL & MAIN_REG_CTRL_EVICT_FAST_AHB_RD_CACHE_Msk) != 0);
    } else {
        MAIN_REG->CTRL |= MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk;
        while ((MAIN_REG->CTRL & MAIN_REG_CTRL_EVICT_SLOW_AHB_RD_CACHE_Msk) != 0);
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetClockFrequency
 ****************************************************************************//**
 *
 * Sets the high bandwidth DMA clock frequency to the desired value. Please note
 * that parameters in the USB block need to be set based on the operating USB
 * speed and selected clock frequency. Hence, this function is expected to be
 * called from the USB stack and not expected to be called directly.
 *
 * \param dmaFreq
 * Desired DMA domain clock frequency.
 *
 * \return
 * CY_HBDMA_SUCCESS if clock setup is successful
 * CY_HBDMA_BAD_PARAM if the parameter passed in is invalid.
 *******************************************************************************/
cy_en_hbdma_status_t Cy_HBDma_SetClockFrequency (cy_en_hbdma_clk_freq_t dmaFreq)
{
    cy_en_hbdma_status_t status = CY_HBDMA_NOT_READY;

    if (MAIN_REG->CTRL & MAIN_REG_CTRL_IP_ENABLED_Msk) {
        switch (dmaFreq) {
            case CY_HBDMA_CLK_SSPHY_CLK:
                /* It is the responsibility of the caller to ensure that USB3 PHY is in the appropriate
                 * state to generate the desired clock frequency. */
                MAIN_REG->CTRL = (
                        (MAIN_REG->CTRL & ~(MAIN_REG_CTRL_DMA_SRC_SEL_Msk | MAIN_REG_CTRL_DMA_DIV_SEL_Msk)) |
                        (2U << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
                status = CY_HBDMA_SUCCESS;
                break;

            case CY_HBDMA_CLK_240_MHZ:
                MAIN_REG->CTRL = (
                        (MAIN_REG->CTRL & ~(MAIN_REG_CTRL_DMA_SRC_SEL_Msk | MAIN_REG_CTRL_DMA_DIV_SEL_Msk)) |
                        (1U << MAIN_REG_CTRL_DMA_DIV_SEL_Pos));
                status = CY_HBDMA_SUCCESS;
                break;

            case CY_HBDMA_CLK_160_MHZ:
                MAIN_REG->CTRL = (
                        (MAIN_REG->CTRL & ~(MAIN_REG_CTRL_DMA_SRC_SEL_Msk | MAIN_REG_CTRL_DMA_DIV_SEL_Msk)) |
                        (2U << MAIN_REG_CTRL_DMA_DIV_SEL_Pos));
                status = CY_HBDMA_SUCCESS;
                break;

            case CY_HBDMA_CLK_150_MHZ:
                MAIN_REG->CTRL = (
                        (MAIN_REG->CTRL & ~(MAIN_REG_CTRL_DMA_SRC_SEL_Msk | MAIN_REG_CTRL_DMA_DIV_SEL_Msk)) |
                        (3U << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
                status = CY_HBDMA_SUCCESS;
                break;

            default:
                status = CY_HBDMA_BAD_PARAM;
                break;
        }
    }

    return status;
}

/*******************************************************************************
 * Function name: Cy_HBDma_Is64KBufferEnabled
 ****************************************************************************//**
 *
 * Check whether 64KB DMA buffer support is enabled on the device. This feature
 * is not supported on A0 silicon revision of FX10.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \return
 * true if 64KB DMA buffers are supported.
 * false if 64KB DMA buffers are not supported.
 *******************************************************************************/
bool Cy_HBDma_Is64KBufferEnabled (cy_stc_hbdma_context_t *pDrvContext)
{
    (void)pDrvContext;
    return ((MAIN_REG->CTRL & MAIN_REG_CTRL_BUFFSIZE_64KB_EN_Msk) != 0);
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetUsbEgressAdapterDelay
 ****************************************************************************//**
 *
 * Update the number of cycles of delay to be applied between consecutive AXI
 * data fetches made by the USB egress DMA adapter. The function is meant to
 * be used by the USB stack based on current USB speed.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \param gblDelayCycles
 * Number of delay cycles to be applied in the range of 0 to 15.
 *
 *******************************************************************************/
void Cy_HBDma_SetUsbEgressAdapterDelay (
        cy_stc_hbdma_context_t *pDrvContext,
        uint8_t gblDelayCycles)
{
    /* Apply the value to the ADAPTER_CONF register if parameters are valid. */
    if ((pDrvContext != NULL) && (pDrvContext->USBEG_SCK_GBL != NULL) && (gblDelayCycles <= 16)) {
        pDrvContext->USBEG_SCK_GBL->ADAPTER_CONF = (
                (pDrvContext->USBEG_SCK_GBL->ADAPTER_CONF & ~USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_GBL_CYCLES_Msk) |
                (gblDelayCycles << USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_GBL_CYCLES_Pos)
                );
    }
}

/*******************************************************************************
 * Function name: Cy_HBDma_SetLvdsAdapterIngressMode
 ****************************************************************************//**
 *
 * This function enables or disables the support for egress data transfers from
 * RAM buffers which are not 16-byte aligned based on whether the specified
 * LVDS DMA adapters are working in Ingress only mode or not. For any ingress-only
 * adapter, this support can be disabled to gain better DMA performance.
 *
 * \param pDrvContext
 * Pointer to DMA driver context structure.
 *
 * \param isAdap0Ingress
 * Whether adapter 0 (sockets 0 to 15) is being used only in ingress direction.
 *
 * \param isAdap1Ingress
 * Whether adapter 1 (sockets 16 to 31) is being used only in ingress direction.
 *
 *******************************************************************************/
void Cy_HBDma_SetLvdsAdapterIngressMode (
        cy_stc_hbdma_context_t *pDrvContext,
        bool isAdap0Ingress,
        bool isAdap1Ingress)
{
    if (pDrvContext != NULL) {
        if (pDrvContext->LVDSAD0_SCK_GBL != NULL) {
            if (isAdap0Ingress) {
                pDrvContext->LVDSAD0_SCK_GBL->ADAPTER_CTRL |=
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk;
            } else {
#if (!UNALIGNED_READ_DISABLE)
                pDrvContext->LVDSAD0_SCK_GBL->ADAPTER_CTRL &=
                    ~LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk;
#endif /* (!UNALIGNED_READ_DISABLE) */
            }
        }

        if (pDrvContext->LVDSAD1_SCK_GBL != NULL) {
            if (isAdap1Ingress) {
                pDrvContext->LVDSAD1_SCK_GBL->ADAPTER_CTRL |=
                    LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk;
            } else {
#if (!UNALIGNED_READ_DISABLE)
                pDrvContext->LVDSAD1_SCK_GBL->ADAPTER_CTRL &=
                    ~LVDSSS_LVDS_ADAPTER_DMA_SCK_GBL_ADAPTER_CTRL_EN_UNALIGNED_READ_N_Msk;
#endif /* (!UNALIGNED_READ_DISABLE) */
            }
        }
    }
}

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS */

/* [] END OF FILE */

