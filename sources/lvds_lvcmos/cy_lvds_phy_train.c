/***************************************************************************//**
* \file cy_lvds_phy_train.c
* \version 1.00
*
* Driver file implementing the custom LVDS PHY training algorithm.
*
*******************************************************************************
* \copyright
* (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_device.h"

#if defined(CY_IP_MXS40LVDS2USB32SS)
#if (!LVCMOS_16BIT_SDR)

#include <string.h>
#include "cy_pdl.h"
#include "cy_lvds.h"
#include "cy_debug.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_lvds_phy_train_cfg.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define LVDS_DLL_PHASE_CNT              (12u)           /**< LVDS slave DLL supports 12 phases. */
#define LVDS_MAX_CHN_CNT                (18u)           /**< 16 data channels plus 2 control channels. */
#define LVDS_TRAIN_PAT_COUNT            (8u)            /**< Number of valid rotated versions of training pattern. */

/* patCount array is of type uint8_t patCount[LVDS_DLL_PHASE_CNT][LVDS_MAX_CHN_CNT][LVDS_TRAIN_PAT_COUNT] */
#define LVDS_PATCOUNT_INDEX(ph,ln,i)    (((ph) * 144u) + ((ln) * 8u) + i)

/**
 * @brief Container for all status information related to custom PHY training implementation.
 */
typedef struct cy_stc_lvds_train_info_ {
    cy_stc_hbdma_mgr_context_t *pDmaMgr;        /**< DMA manager handle. */
    cy_stc_hbdma_channel_t      chn;            /**< DMA channel used to receive incoming data. */
    LVDSSS_LVDS_Type           *pIp;            /**< Base pointer for the IP register block. */
    cy_stc_lvds_context_t      *pCtxt;          /**< LVDS driver context. */
    uint8_t                    *patCount;       /**< Number of times each shifted pattern is received. */
    uint8_t                     patterns[LVDS_TRAIN_PAT_COUNT];     /**< Shifted versions of the training pattern. */
    uint8_t                     optimalPhase[LVDS_MAX_CHN_CNT];     /**< Optimal sampling phase for the lane. */
    bool                        lnk0Enable;     /**< Whether custom link training is enabled for Link-0 */
    bool                        lnk1Enable;     /**< Whether custom link training is enabled for Link-1 */
    bool                        lnk0Pending;    /**< Whether custom link training on Link-0 is pending. */
    bool                        lnk1Pending;    /**< Whether custom link training on Link-1 is pending. */
    bool                        lnk0TrainDone;  /**< Whether Link-0 training is complete. */
    bool                        lnk1TrainDone;  /**< Whether Link-1 training is complete. */
    uint8_t                     lnk0LaneCnt;    /**< Data lane count for link-0 */
    uint8_t                     lnk1LaneCnt;    /**< Data lane count for link-1 */
    bool                        isWideLink;     /**< Whether WideLink mode is enabled. */
    uint8_t                     currentLink;    /**< Index of link being trained. */
    uint8_t                     ctrlTraining;   /**< Whether control lane training is ongoing. */
    uint8_t                     nextPhaseVal;   /**< Next slave DLL phase to be applied. */
} cy_stc_lvds_train_info_t;

static cy_stc_lvds_train_info_t gLvdsTrain;     /**< LVDS training data structure. */

/*******************************************************************************
 * Function name: Cy_LVDS_CheckTrainingData
 ***************************************************************************//**
 *
 * Function to check the received data for PHY training pattern and
 * rotated versions.
 *
 * \param pData
 * Pointer to data buffer
 * \param size
 * Size of data in bytes
 *
 ******************************************************************************/
static void
Cy_LVDS_CheckTrainingData (
        uint8_t  *pData,
        uint32_t  size)
{
    uint32_t lane, idx;
    uint32_t phaseIdx  = gLvdsTrain.nextPhaseVal - 1;
    uint32_t startlane = (gLvdsTrain.currentLink != 0) ? 8u : 0;
    uint32_t laneCnt   = (gLvdsTrain.currentLink != 0) ? gLvdsTrain.lnk1LaneCnt : gLvdsTrain.lnk0LaneCnt;
    uint32_t patIndex;
    uint8_t  counts[16][LVDS_TRAIN_PAT_COUNT];
    uint8_t  goodPat = 0xFFu;
    uint8_t  tempData;

    if (gLvdsTrain.ctrlTraining) {
        /*
         * In the second round of custom training, we only take data for the Control Lane which
         * will be found in the Lane-0 position.
         */
        lane = (gLvdsTrain.currentLink != 0) ? 17u : 16u;

        /* Clear the local counts array for all patterns. */
        for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
            counts[0][patIndex] = 0;
        }

        /*
         * Once we find any of the expected data patterns, we can expect to keep finding the same data.
         * So, there is no need to scan for all data patterns repeatedly.
         * Similarly, we can stop scanning once we see any data byte which does not match any of the expected
         * patterns.
         */
        goodPat = 0xFFu;
        for (idx = 0; idx < size; idx += laneCnt) {
            tempData = pData[idx];
            if (tempData == goodPat) {
                counts[0][patIndex]++;
            } else {
                for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
                    if (tempData == gLvdsTrain.patterns[patIndex]) {
                        counts[0][patIndex]++;
                        goodPat = gLvdsTrain.patterns[patIndex];
                        break;
                    }
                }

                /* At least one byte did not match any of the known patterns. Skip counting these values. */
                if (patIndex >= LVDS_TRAIN_PAT_COUNT) {
                    break;
                }
            }
        }

        /* Store the calculated counts in the global table. */
        for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phaseIdx,lane,patIndex)] = counts[0][patIndex];
        }
    } else {
        /*
         * In the first round of custom training, we will be collecting data for all data lanes.
         */
        for (lane = 0; lane < laneCnt; lane++) {
            /* Zero out all counts for the lane being checked. */
            for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
                counts[lane][patIndex] = 0;
            }

            /*
             * Scan through the entire data buffer and count the number of patterns found on the current lane.
             *
             * Once we find any of the expected data patterns, we can expect to keep finding the same data.
             * So, there is no need to scan for all data patterns repeatedly.
             * Similarly, we can stop scanning once we see any data byte which does not match any of the expected
             * patterns.
             */
            goodPat = 0xFFu;
            for (idx = lane; idx < size; idx += laneCnt) {
                tempData = pData[idx];
                if (tempData == goodPat) {
                    counts[lane][patIndex]++;
                } else {
                    for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
                        if (tempData == gLvdsTrain.patterns[patIndex]) {
                            counts[lane][patIndex]++;
                            goodPat = gLvdsTrain.patterns[patIndex];
                            break;
                        }
                    }

                    /* At least one byte did not match any of the known patterns. Skip counting these values. */
                    if (patIndex >= LVDS_TRAIN_PAT_COUNT) {
                        break;
                    }
                }
            }
        }

        /* Store the calculated counts in the global table. */
        for (lane = 0; lane < laneCnt; lane++) {
            for (patIndex = 0; patIndex < LVDS_TRAIN_PAT_COUNT; patIndex++) {
                gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phaseIdx,startlane+lane,patIndex)] = counts[lane][patIndex];
            }
        }
    }
}

/*******************************************************************************
 * Function name: Cy_LVDS_ForcePhaseSelection
 ***************************************************************************//**
 *
 * Override Slave DLL instances associated with data and control lanes of a port
 * with the desired phase setting.
 *
 * \param isOptimal
 * If true, apply the optimal phase identified. If false, apply the
 * gLvdsTrain.nextPhaseVal phase.
 * \param portNo
 * When isOptimal is true, identifies the port on which phases are to be applied.
 *
 ******************************************************************************/
static void
Cy_LVDS_ForcePhaseSelection (
        bool    isOptimal,
        uint8_t portNo)
{
    LVDSSS_LVDS_Type *pRegs = gLvdsTrain.pIp;
    uint32_t lane;

    if (isOptimal) {
        if (portNo == 0) {
            /* Data lanes of link-0 */
            for (lane = 0; ((lane < gLvdsTrain.lnk0LaneCnt) && (lane < 8)); lane++) {
                pRegs->AFE[0].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                pRegs->AFE[0].DLL_S_CONFIG[lane] |= ((gLvdsTrain.optimalPhase[lane] << 4u) | 0x100UL);
            }

            /* CTRL lane of link-0 */
            pRegs->AFE[0].DLL_S_CONFIG[8] &= ~(0x1F0UL);
            pRegs->AFE[0].DLL_S_CONFIG[8] |= ((gLvdsTrain.optimalPhase[16u] << 4u) | 0x100UL);

            if (gLvdsTrain.isWideLink) {
                /* Data lanes of link-1 */
                for (lane = 0; lane < (gLvdsTrain.lnk0LaneCnt - 8); lane++) {
                    pRegs->AFE[1].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                    pRegs->AFE[1].DLL_S_CONFIG[lane] |= ((gLvdsTrain.optimalPhase[lane + 8] << 4u) | 0x100UL);
                }
            }
        }

        if (portNo != 0) {
            /* Data lanes of link-1 */
            for (lane = 0; lane < gLvdsTrain.lnk1LaneCnt; lane++) {
                pRegs->AFE[1].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                pRegs->AFE[1].DLL_S_CONFIG[lane] |= ((gLvdsTrain.optimalPhase[lane + 8] << 4u) | 0x100UL);
            }

            /* CTRL lane of link-1 */
            pRegs->AFE[1].DLL_S_CONFIG[8] &= ~(0x1F0UL);
            pRegs->AFE[1].DLL_S_CONFIG[8] |= ((gLvdsTrain.optimalPhase[17u] << 4u) | 0x100UL);
        }
    } else {
        if (gLvdsTrain.currentLink == 0) {
            /* Data lanes of link-0 */
            for (lane = 0; ((lane < gLvdsTrain.lnk0LaneCnt) && (lane < 8)); lane++) {
                pRegs->AFE[0].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                pRegs->AFE[0].DLL_S_CONFIG[lane] |= ((gLvdsTrain.nextPhaseVal << 4u) | 0x100UL);
            }

            /* CTRL lane of link-0 */
            pRegs->AFE[0].DLL_S_CONFIG[8] &= ~(0x1F0UL);
            pRegs->AFE[0].DLL_S_CONFIG[8] |= ((gLvdsTrain.nextPhaseVal << 4u) | 0x100UL);

            if (gLvdsTrain.isWideLink) {
                /* Data lanes of link-1 */
                for (lane = 0; lane < (gLvdsTrain.lnk0LaneCnt - 8); lane++) {
                    pRegs->AFE[1].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                    pRegs->AFE[1].DLL_S_CONFIG[lane] |= ((gLvdsTrain.nextPhaseVal << 4u) | 0x100UL);
                }
            }
        }

        if (gLvdsTrain.currentLink != 0) {
            /* Data lanes of link-1 */
            for (lane = 0; lane < gLvdsTrain.lnk1LaneCnt; lane++) {
                pRegs->AFE[1].DLL_S_CONFIG[lane] &= ~(0x1F0UL);
                pRegs->AFE[1].DLL_S_CONFIG[lane] |= ((gLvdsTrain.nextPhaseVal << 4u) | 0x100UL);
            }

            /* CTRL lane of link-1 */
            pRegs->AFE[1].DLL_S_CONFIG[8] &= ~(0x1F0UL);
            pRegs->AFE[1].DLL_S_CONFIG[8] |= ((gLvdsTrain.nextPhaseVal << 4u) | 0x100UL);
        }
    }
}

/*******************************************************************************
 * Function name: Cy_LVDS_IdentifyOptimalPhase
 ***************************************************************************//**
 *
 * Go through the training data collected and identify the best phase for each
 * data and control lane.
 *
 * \return
 * true if custom training state machine needs to be run for the other port,
 * false otherwise.
 *
 ******************************************************************************/
static bool
Cy_LVDS_IdentifyOptimalPhase (
        void)
{
    uint32_t numDataLanes, ctrlLaneIdx, firstDataLane;
    uint32_t i, lane;
    uint8_t  first;
    uint8_t  a1, a2, b1, b2, a, b, c, d;
    uint8_t  sel_ph, variant;
    uint8_t  eyeVector[LVDS_DLL_PHASE_CNT + 1];
    cy_en_lvds_status_t stat;
    bool     toContinue = false;

    firstDataLane = (gLvdsTrain.currentLink != 0) ? 8 : 0;
    numDataLanes  = (gLvdsTrain.currentLink != 0) ? gLvdsTrain.lnk1LaneCnt : gLvdsTrain.lnk0LaneCnt;
    ctrlLaneIdx   = (gLvdsTrain.currentLink != 0) ? 17 : 16;

    for (lane = 0; lane < LVDS_MAX_CHN_CNT; lane++) {
        if (((lane < firstDataLane) || (lane >= (firstDataLane + numDataLanes))) && (lane != ctrlLaneIdx)) {
            /* Not a valid lane. Continue to the next one. */
            continue;
        }

        /* Zero out the eye vector. */
        memset(eyeVector, 0, (LVDS_DLL_PHASE_CNT + 1));

        /* Identify what variants of the expected data are received for each phase selection. */
        for (i = 0; i < LVDS_DLL_PHASE_CNT; i++) {
            for (variant = 0; variant < LVDS_TRAIN_PAT_COUNT; variant++) {
                if (gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(i,lane,variant)] == 128U) {
                    eyeVector[i + 1] = variant + 1;
                    break;
                }
            }
        }

        /*
         * At maximum, we expect two different variants of the training patterns to be captured while scanning
         * across all the phases. The specific variants do not matter, we just need to identify the phases
         * where one pattern or the other is being consistently captured.
         * So, we can reduce the eye vector to values of 0 (no pattern), 1 (first valid pattern) and
         * 2 (second valid pattern).
         */
        for (i = 0, first = 0; i < LVDS_DLL_PHASE_CNT; i++) {
            if (eyeVector[i + 1] != 0) {
                if (first == 0) {
                    first = eyeVector[i + 1];
                    eyeVector[i + 1] = 1;
                } else {
                    if (eyeVector[i + 1] == first) {
                        eyeVector[i + 1] = 1;
                    } else {
                        eyeVector[i + 1] = 2;
                    }
                }
            }
        }

        /* Find the longest sequence of phases across which a specific pattern is being consistently detected. */
        a1 = 1 * (eyeVector[1] == 1);
        b1 = LVDS_DLL_PHASE_CNT *(eyeVector[LVDS_DLL_PHASE_CNT] == 1);
        a2 = 1 * (eyeVector[1] == 2);
        b2 = LVDS_DLL_PHASE_CNT * (eyeVector[LVDS_DLL_PHASE_CNT] == 2);

        for (i = 2; i <= LVDS_DLL_PHASE_CNT; i++)
        {
            if ((eyeVector[i - 1] == 1) && (eyeVector[i] == 0))     /* 1 -> 0 transition */
            {
                b1 = i - 1;
            }

            if ((eyeVector[i - 1] == 0) && (eyeVector[i] == 1))     /* 0 -> 1 transition */
            {
                a1 = i;
            }

            if ((eyeVector[i - 1] == 2) && (eyeVector[i] == 0))     /* 2 -> 0 transition */
            {
                b2 = i - 1;
            }

            if ((eyeVector[i - 1] == 0) && (eyeVector[i] == 2))     /* 0 -> 2 transition */
            {
                a2 = i;
            }

            if ((eyeVector[i - 1] == 1) && (eyeVector[i] == 2))     /* 1 -> 2 transition */
            {
                a2 = i;
                b1 = i - 1;
            }

            if ((eyeVector[i - 1] == 2) && (eyeVector[i] == 1))     /* 2 -> 1 transition */
            {
                a1 = i;
                b2 = i - 1;
            }
        }

        if (a1 > b2) {
            a = a2;
            b = b2;
            c = a1;
            d = b1;
        } else {
            a = a1;
            b = b1;
            c = a2;
            d = b2;
        }

        if ((a == 1) && (d == LVDS_DLL_PHASE_CNT)) {
            sel_ph = c + (((d - c) + (b - a)) / 2) + 1;
        } else {
            if ((b - a) > (d - c)) {
                if (b < a) {
                    sel_ph = ((LVDS_DLL_PHASE_CNT + b - a) / 2) + 1 + a;
                } else {
                    sel_ph = ((b - a) / 2) + a;
                }
            } else {
                if (d < c) {
                    sel_ph = ((LVDS_DLL_PHASE_CNT + d - c) / 2) + 1 + c;
                } else {
                    sel_ph = ((d - c) / 2) + c;
                }
            }
        }

        if (sel_ph > LVDS_DLL_PHASE_CNT) {
            sel_ph -= LVDS_DLL_PHASE_CNT;
        }

        gLvdsTrain.optimalPhase[lane] = sel_ph;
        DBG_LVDS_TRACE("Lane-%d: Optimal phase=%d\r\n", lane, sel_ph);
    }

    /* Destroy the DMA channel used for training. */
    Cy_HBDma_Channel_Disable(&(gLvdsTrain.chn));
    Cy_HBDma_Channel_Destroy(&(gLvdsTrain.chn));

    /* Free the buffer allocated to hold pattern counts. */
    if (gLvdsTrain.patCount != NULL) {
        Cy_HBDma_BufMgr_Free(gLvdsTrain.pDmaMgr->pBufMgr, (void *)gLvdsTrain.patCount);
    }

    if (gLvdsTrain.currentLink == 0) {
        /* Mark training done. */
        gLvdsTrain.lnk0TrainDone = true;

        /* Disable the thread. */
        Cy_LVDS_ThreadDisable(gLvdsTrain.pIp, 0);

        /* Stop the GPIF state machine. */
        Cy_LVDS_GpifSMStop(gLvdsTrain.pIp, 0);

        /* If training is pending on the other link, proceed with it. */
        if (gLvdsTrain.lnk1Pending) {
            stat = Cy_LVDS_CustomTraining_Enable(gLvdsTrain.pIp, gLvdsTrain.pCtxt, 1, false, gLvdsTrain.lnk1LaneCnt,
                    ((gLvdsTrain.pIp->AFE[1].PHY_TRAIN_CONFIG & 0x1FEUL) >> 1U),
                    gLvdsTrain.pDmaMgr);
            if (stat != CY_LVDS_SUCCESS) {
                DBG_LVDS_INFO("Configuration of LVDS custom training for Port-1 failed with status %x\r\n", stat);
            } else {
                toContinue = true;
            }
        }
    } else {
        /* Mark training done. */
        gLvdsTrain.lnk1TrainDone = true;

        /* Disable the thread. */
        Cy_LVDS_ThreadDisable(gLvdsTrain.pIp, 2);

        /* Stop the GPIF state machine. */
        Cy_LVDS_GpifSMStop(gLvdsTrain.pIp, 1);

        /* If training is pending on the other link, proceed with it. */
        if (gLvdsTrain.lnk0Pending) {
            stat = Cy_LVDS_CustomTraining_Enable(gLvdsTrain.pIp, gLvdsTrain.pCtxt, 0, false, gLvdsTrain.lnk0LaneCnt,
                    ((gLvdsTrain.pIp->AFE[0].PHY_TRAIN_CONFIG & 0x1FEUL) >> 1U),
                    gLvdsTrain.pDmaMgr);
            if (stat != CY_LVDS_SUCCESS) {
                DBG_LVDS_INFO("Configuration of LVDS custom training for Port-0 failed with status %x\r\n", stat);
            } else {
                toContinue = true;
            }
        }
    }

    return (toContinue);
}

/*******************************************************************************
 * Function name: Cy_LVDS_Training_DmaCb
 ***************************************************************************//**
 *
 * DMA callback function for the custom PHY training implementation.
 *
 * \param pChn
 * DMA channel handle
 * \param type
 * Type of DMA event
 * \param pbufStat
 * Pointer to DMA buffer status information
 * \param userCtx
 * Opaque caller context pointer
 *
 ******************************************************************************/
static void
Cy_LVDS_Training_DmaCb (
        struct cy_stc_hbdma_channel     *pChn,
        cy_en_hbdma_cb_type_t            type,
        struct cy_stc_hbdma_buff_status *pbufStat,
        void                            *userCtx)
{
    if (type == CY_HBDMA_CB_PROD_EVENT) {
        DBG_LVDS_TRACE("+++LvdsDmaCb: next phase=%d count=%d\r\n", gLvdsTrain.nextPhaseVal, pbufStat->count);

        /* Check the received data and update the pattern counts. */
        Cy_LVDS_CheckTrainingData(pbufStat->pBuffer, pbufStat->count);

        /* Move to the next slave DLL phase setting. */
        gLvdsTrain.nextPhaseVal++;
        if ((gLvdsTrain.nextPhaseVal <= LVDS_DLL_PHASE_CNT) || (!gLvdsTrain.ctrlTraining)) {
            if (gLvdsTrain.nextPhaseVal > LVDS_DLL_PHASE_CNT) {
                /* We have completed collecting data for all data lanes. Now, repeat for the CTRL lane. */
                gLvdsTrain.nextPhaseVal = 1;
                gLvdsTrain.ctrlTraining = true;

                /* Modify CHAR configuration to get CTRL lane data into DATA0 position. */
                gLvdsTrain.pIp->LVDS_CHAR_CFG[gLvdsTrain.currentLink] = (
                        (gLvdsTrain.pIp->LVDS_CHAR_CFG[gLvdsTrain.currentLink] & 0x800000FEUL) | 0x0100UL);
            }

            /* Discard the data buffer so that we can get the next data. */
            Cy_HBDma_Channel_DiscardBuffer(pChn, pbufStat);

            /* Apply the newly selected phase. */
            Cy_LVDS_ForcePhaseSelection(false, 0);

            /* De-assert and assert FW_TRIG to get the next data buffer in. */
            Cy_LVDS_GpifClearFwTrig(gLvdsTrain.pIp, gLvdsTrain.currentLink);
            Cy_SysLib_DelayUs(10);
            Cy_LVDS_GpifSetFwTrig(gLvdsTrain.pIp, gLvdsTrain.currentLink);
        }
    }
}

/*******************************************************************************
 * Function name: Cy_LVDS_CustomTraining_Init
 ***************************************************************************//**
 *
 * Perform default initialization of the LVDS custom training related data
 * structures.
 *
 ******************************************************************************/
void
Cy_LVDS_CustomTraining_Init (
        void)
{
    /* Reset the custom training related data structures. */
    memset(&gLvdsTrain, 0, sizeof(gLvdsTrain));
}

/*******************************************************************************
 * Function name: Cy_LVDS_CustomTraining_Enable
 ***************************************************************************//**
 *
 * Function to specify that custom link training is to be enabled for an LVDS
 * port.
 *
 * \param base
 * Base pointer to LVDS register set.
 * \param pDrvCtxt
 * Pointer to LVDS driver context structure.
 * \param portNo
 * LVDS port number
 * \param isWL
 * Whether training is to be done for a WideLink port.
 * \param nLanes
 * Number of data lanes associated with the link.
 * \param phyTrainingPattern
 * The PHY training pattern which is being used.
 * \param pDmaMgr
 * Pointer to the DMA manager. Used to create and operate a DMA channel.
 *
 * \return
 * Success if custom training state machine could be started.
 ******************************************************************************/
cy_en_lvds_status_t
Cy_LVDS_CustomTraining_Enable (
        LVDSSS_LVDS_Type           *base,
        cy_stc_lvds_context_t      *pDrvCtxt,
        uint8_t                     portNo,
        bool                        isWL,
        uint8_t                     nLanes,
        uint8_t                     phyTrainingPattern,
        cy_stc_hbdma_mgr_context_t *pDmaMgr)
{
    uint32_t lane;
    uint32_t firstLane, ctrlLane;
    uint32_t phase;
    cy_stc_hbdma_chn_config_t dmaConf;
    cy_en_hbdma_mgr_status_t  dmaStat;

    /* Perform parameter checks. */
    if ((base == NULL) || (pDrvCtxt == NULL) || (pDmaMgr == NULL) || (portNo >= 2) || ((portNo == 1) && (isWL))) {
        DBG_LVDS_ERR("Cy_LVDS_CustomTraining_Enable: BadParam\r\n");
        return CY_LVDS_BAD_PARAMETER;
    }

    /* Make sure custom link training is not already enabled for the port. */
    if (((portNo == 0) && (gLvdsTrain.lnk0Enable)) || ((portNo == 1) && (gLvdsTrain.lnk1Enable))) {
        DBG_LVDS_ERR("Cy_LVDS_CustomTraining_Enable: BadSequence\r\n");
        return CY_LVDS_LINK_ALREADY_ENABLED;
    }

    /* Allocate a RAM buffer to hold the pattern counts during training. */
    gLvdsTrain.patCount = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pDmaMgr->pBufMgr,
            LVDS_DLL_PHASE_CNT * LVDS_MAX_CHN_CNT * LVDS_TRAIN_PAT_COUNT);
    if (gLvdsTrain.patCount == NULL) {
        return (CY_LVDS_CONFIG_ERROR);
    }

    /* If training has been enabled on the other port, mark the operation pending and return. */
    if ((portNo == 0) && (gLvdsTrain.lnk1Enable) && (!gLvdsTrain.lnk1TrainDone)) {
        gLvdsTrain.lnk0Pending = true;
        gLvdsTrain.lnk0LaneCnt = nLanes;
        return CY_LVDS_SUCCESS;
    } else {
        gLvdsTrain.lnk0Pending = false;
    }

    if ((portNo == 1) && (gLvdsTrain.lnk0Enable) && (!gLvdsTrain.lnk0TrainDone)) {
        gLvdsTrain.lnk1Pending = true;
        gLvdsTrain.lnk1LaneCnt = nLanes;
        return CY_LVDS_SUCCESS;
    } else {
        gLvdsTrain.lnk1Pending = false;
    }

    /* Identify the lanes to be trained. */
    firstLane = (portNo != 0) ? 8u : 0;
    ctrlLane  = (portNo != 0) ? 17u : 16u;

    gLvdsTrain.pIp        = base;
    gLvdsTrain.pCtxt      = pDrvCtxt;
    gLvdsTrain.pDmaMgr    = pDmaMgr;
    gLvdsTrain.isWideLink = isWL;

    if (portNo == 0) {
        gLvdsTrain.lnk0Enable    = true;
        gLvdsTrain.lnk0TrainDone = false;
        gLvdsTrain.currentLink   = 0;
        gLvdsTrain.lnk0LaneCnt   = nLanes;
    } else {
        gLvdsTrain.lnk1Enable    = true;
        gLvdsTrain.lnk1TrainDone = false;
        gLvdsTrain.currentLink   = 1;
        gLvdsTrain.lnk1LaneCnt   = nLanes;
    }

    /* Save all bit-wise rotated versions of the training pattern. */
    gLvdsTrain.patterns[0] = phyTrainingPattern;
    gLvdsTrain.patterns[1] = (phyTrainingPattern >> 1u) | ((phyTrainingPattern & 0x01U) << 7u);
    gLvdsTrain.patterns[2] = (phyTrainingPattern >> 2u) | ((phyTrainingPattern & 0x03U) << 6u);
    gLvdsTrain.patterns[3] = (phyTrainingPattern >> 3u) | ((phyTrainingPattern & 0x07U) << 5u);
    gLvdsTrain.patterns[4] = (phyTrainingPattern >> 4u) | ((phyTrainingPattern & 0x0FU) << 4u);
    gLvdsTrain.patterns[5] = (phyTrainingPattern >> 5u) | ((phyTrainingPattern & 0x1FU) << 3u);
    gLvdsTrain.patterns[6] = (phyTrainingPattern >> 6u) | ((phyTrainingPattern & 0x3FU) << 2u);
    gLvdsTrain.patterns[7] = (phyTrainingPattern >> 7u) | ((phyTrainingPattern & 0x7FU) << 1u);

    /* Clear the training counts and optimal phase selected. */
    for (lane = 0; lane < LVDS_MAX_CHN_CNT; lane++) {
        if (((lane < firstLane) || (lane >= (firstLane + nLanes))) && (lane != ctrlLane)) {
            /* Skip lanes which are not part of current link. */
            continue;
        }

        gLvdsTrain.optimalPhase[lane] = 0;
        for (phase = 0; phase < LVDS_DLL_PHASE_CNT; phase++) {
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,0)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,1)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,2)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,3)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,4)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,5)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,6)] = 0;
            gLvdsTrain.patCount[LVDS_PATCOUNT_INDEX(phase,lane,7)] = 0;
        }
    }

    /* Config parameters for the training DMA channel. */
    dmaConf.size                = 128UL * nLanes;                       /* Buffer should hold 128 bytes per lane. */
    dmaConf.count               = 1;                                    /* Only one buffer required. */
    dmaConf.prodHdrSize         = 0;                                    /* No headers. */
    dmaConf.prodBufSize         = dmaConf.size;                         /* Producer buffer size matches actual size. */
    dmaConf.chType              = CY_HBDMA_TYPE_IP_TO_MEM;              /* IP to memory channel type. */
    dmaConf.bufferMode          = true;
    dmaConf.eventEnable         = false;
    dmaConf.prodSckCount        = 1;
    dmaConf.prodSck[0]          = (portNo != 0) ? CY_HBDMA_LVDS_SOCKET_16 : CY_HBDMA_LVDS_SOCKET_00;
    dmaConf.prodSck[1]          = CY_HBDMA_VIRT_SOCKET_WR;
    dmaConf.consSckCount        = 1;
    dmaConf.consSck[0]          = CY_HBDMA_VIRT_SOCKET_RD;
    dmaConf.consSck[1]          = CY_HBDMA_VIRT_SOCKET_RD;
    dmaConf.endpAddr            = 0;
    dmaConf.usbMaxPktSize       = dmaConf.size;
    dmaConf.intrEnable          = 0x01UL;                               /* Enable interrupt for produce event. */
    dmaConf.cb                  = Cy_LVDS_Training_DmaCb;
    dmaConf.userCtx             = (void *)&gLvdsTrain;

    gLvdsTrain.ctrlTraining = 0;
    gLvdsTrain.nextPhaseVal = 0;

    dmaStat = Cy_HBDma_Channel_Create(pDmaMgr, &(gLvdsTrain.chn), &dmaConf);
    if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
        dmaStat = Cy_HBDma_Channel_Enable(&(gLvdsTrain.chn), 0);
        if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
            DBG_LVDS_TRACE("Enabled DMA channel for custom PHY training\r\n");
        } else {
            DBG_LVDS_ERR("Failed to enable DMA channel for custom PHY training\r\n");
            if (portNo == 0) {
                gLvdsTrain.lnk0Enable = false;
            } else {
                gLvdsTrain.lnk1Enable = false;
            }
            Cy_HBDma_Channel_Destroy(&(gLvdsTrain.chn));
            return CY_LVDS_LINK_TRAINING_ERROR;
        }
    } else {
        DBG_LVDS_ERR("Failed to create DMA channel to collect training data\r\n");
        if (portNo == 0) {
            gLvdsTrain.lnk0Enable = false;
        } else {
            gLvdsTrain.lnk1Enable = false;
        }
        return CY_LVDS_LINK_TRAINING_ERROR;
    }

    /* Override GPIF configuration with custom state machine and start it. */
    Cy_LVDS_GpifSMStop(base, portNo);
    Cy_LVDS_GpifInit(base, portNo, &_CyGpifLvdsTrainConfig, pDrvCtxt);
    if (portNo == 0) {
        Cy_LVDS_GpifSMStart(base, portNo, CY_LVDS_GPIF_P0_TRAINSTART, CY_LVDS_GPIF_ALPHA_P0_TRAINSTART);
    } else {
        Cy_LVDS_GpifSMStart(base, portNo, CY_LVDS_GPIF_P1_TRAINSTART, CY_LVDS_GPIF_ALPHA_P1_TRAINSTART);
    }

    /* Enable the thread to receive data. */
    Cy_LVDS_GpifThreadConfig(base, (portNo == 0) ? 0 : 2, 0, false, 1, 4);

    return CY_LVDS_SUCCESS;
}

/*******************************************************************************
 * Function name: Cy_LVDS_CustomTraining_ApplyResults
 ***************************************************************************//**
 *
 * Function to apply the results of the LVDS custom training where available.
 *
 * \param pDrvCtxt
 * Pointer to LVDS driver context structure.
 * \param portNo
 * LVDS port number
 *
 * \return
 * true if the custom training results have been applied, false otherwise.
 ******************************************************************************/
bool Cy_LVDS_CustomTraining_ApplyResults (
        cy_stc_lvds_context_t *pDrvCtxt,
        uint8_t                portNo)
{
    bool retval = false;

    /* If custom training was enabled for the port and has been completed, apply the results. */
    if ((pDrvCtxt != NULL) && (gLvdsTrain.pCtxt == pDrvCtxt)) {
        if ((portNo == 0) && (gLvdsTrain.lnk0Enable) && (gLvdsTrain.lnk0TrainDone)) {
            Cy_LVDS_ForcePhaseSelection(true, 0);
            retval = true;
        }

        if ((portNo == 1) && (gLvdsTrain.lnk1Enable) && (gLvdsTrain.lnk1TrainDone)) {
            Cy_LVDS_ForcePhaseSelection(true, 1);
            retval = true;
        }
    }

    return (retval);
}

/*******************************************************************************
 * Function name: Cy_LVDS_CustomTraining_Task
 ***************************************************************************//**
 *
 * Function which updates states related LVDS custom training sequence. Needs
 * to be called until training is complete.
 *
 * \param pDrvCtxt
 * Pointer to LVDS driver context structure.
 *
 * \return
 * true if the training task is yet to be completed, false if task is complete.
 *
 ******************************************************************************/
bool
Cy_LVDS_CustomTraining_Task (
        cy_stc_lvds_context_t *pDrvCtxt)
{
    uint8_t donePort = 0;

    /* Check whether ongoing training sequence has been completed. */
    if (
            (
             ((gLvdsTrain.lnk0Enable) && (!gLvdsTrain.lnk0TrainDone)) ||
             ((gLvdsTrain.lnk1Enable) && (!gLvdsTrain.lnk1TrainDone))
            )
       ) {
        /* If process has not been started, start it off. */
        if (gLvdsTrain.nextPhaseVal == 0) {
            /* Select phase-1 on all active lanes. */
            gLvdsTrain.nextPhaseVal = 1;
            Cy_LVDS_ForcePhaseSelection(false, gLvdsTrain.currentLink);

            /* Trigger the GPIF state machine to capture the first set of data. */
            Cy_LVDS_GpifSetFwTrig(gLvdsTrain.pIp, gLvdsTrain.currentLink);
        }

        /* Check whether training is complete. */
        if (
                (gLvdsTrain.nextPhaseVal > LVDS_DLL_PHASE_CNT) &&
                (gLvdsTrain.ctrlTraining)
           ) {
            donePort = gLvdsTrain.currentLink;

            /* Data collection is complete. Analyze the data and select the optimal phase for each lane. */
            if (!Cy_LVDS_IdentifyOptimalPhase()) {
                /* Send PHY training complete callback for the port from here. */
                gLvdsTrain.pCtxt->intrCallback->phy_events(donePort, CY_LVDS_PHY_TRAINING_DONE,
                        gLvdsTrain.pCtxt->userContext);
            }
        }
    }

    /* Return true if training is yet to be completed on at least one of the ports. */
    return (
            ((gLvdsTrain.lnk0Enable) && (!gLvdsTrain.lnk0TrainDone)) ||
            ((gLvdsTrain.lnk1Enable) && (!gLvdsTrain.lnk1TrainDone))
           );
}

#if defined(__cplusplus)
}
#endif

#endif /* (!LVCMOS_16BIT_SDR) */
#endif /* defined(CY_IP_MXS40LVDS2USB32SS) */

/* [] END OF FILE */

