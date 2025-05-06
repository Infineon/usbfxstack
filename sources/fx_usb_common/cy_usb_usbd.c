/***************************************************************************//**
* \file cy_usb_usbd.c
* \version 1.0
*
* Source file that implements the USB Device programming interface used in
* the FX device family.
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

#include "cy_device_headers.h"
#include "cy_device.h"
#include "cy_pdl.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#endif /* FREERTOS_ENABLE */
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usbd_version.h"
#include <string.h>
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

static const uint32_t cy_usbd_version_num = USBD_VERSION_NUM;

/* Variable that tracks the number of completed timer ticks. */
static volatile uint32_t gCurrentTimerTick = 0;

static void
Cy_USBD_DisableConnection(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          bool isApiCall);

static void
Cy_USBD_EnableConnection(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_en_usb_speed_t usbSpeed,
                         bool isApiCall);

/*******************************************************************************
* Function name: Cy_USBD_GetVersion
****************************************************************************//**
*
* This API returns the USBD stack version number.
*
* \return
* 32-bit USBD stack version number.
*******************************************************************************/
uint32_t
Cy_USBD_GetVersion (void)
{
    return cy_usbd_version_num;
}

/*******************************************************************************
* Function name: Cy_USBD_Ep0DmaChn_Cb
****************************************************************************//**
*
* High BandWidth DMA channel callback for EP0 OUT and IN data transfers.
*
* \param pChannel
* DMA channel pointer.
*
* \param type
* Type of DMA event.
*
* \param pbufStat
* Event specific data.
*
* \param pUserCtxt
* Context information for the callback: points to the USBD stack context structure.
*
*******************************************************************************/
static void
Cy_USBD_Ep0DmaChn_Cb (
        cy_stc_hbdma_channel_t *pChannel,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *pUserCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pUserCtxt;
    cy_en_hbdma_mgr_status_t dmaStatus;
    uint32_t actualSize = 0;
    cy_stc_usb_cal_msg_t xMsg;

    /* If the callback indicates receive data completion on EP0-OUT. */
    if ((pUsbdCtxt != NULL) && (pUsbdCtxt->ep0RecvCb != NULL) && (pChannel == &(pUsbdCtxt->outEp0DmaUsb3Ch))) {
        xMsg.type    = CY_USB_CAL_MSG_EP0_RCV_DONE;
        xMsg.data[0] = (uint32_t)pUsbdCtxt->pEp0ReceiveBuffer;

        dmaStatus = Cy_HBDma_Channel_WaitForReceiveCplt(pChannel, 0, &actualSize);
        if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
            if (actualSize != pUsbdCtxt->ep0ExpRcvSize) {
                DBG_USBD_WARN("EP0Recv size error: exp=%d act=%d\r\n", pUsbdCtxt->ep0ExpRcvSize, actualSize);
            }

            xMsg.data[1] = actualSize;
        } else {
            DBG_USBD_ERR("EP0Recv DMA error: %x\r\n", dmaStatus);
        }

        pUsbdCtxt->ep0RecvCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, &xMsg);
        pUsbdCtxt->pEp0ReceiveBuffer = NULL;
        pUsbdCtxt->ep0ExpRcvSize = 0;
    }
}

/*******************************************************************************
* Function name: Cy_USB_Create_Ep0HbDmaChannels
****************************************************************************//**
*
* Create high bandwidth DMA channel for endpoint 0.
*
*  \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if high bandwidth DMA manager pointer NULL.
*
*******************************************************************************/
static cy_en_usbd_ret_code_t
Cy_USB_Create_Ep0HbDmaChannels (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t  mgrStat;

    if (pUsbdCtxt->pHBDmaMgr == NULL) {
        DBG_USBD_WARN("HBDMA Mgr is NULL\r\n");
        return CY_USBD_STATUS_FAILURE;
    }

    /* Allocate a buffer large enough to hold the maximum EP0 transfer. */
    pUsbdCtxt->inEp0ScratchBuffer = Cy_HBDma_BufMgr_Alloc(pUsbdCtxt->pHBDmaMgr->pBufMgr, 4096UL);

    dmaConfig.size           = 512;                        /* DMA Buffer size in bytes */
    dmaConfig.count          = 1;                          /* DMA Buffer Count */
    dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_MEM;    /* DMA Channel type: from USB3 IP to HB-SRAM */
    dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
    dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
    dmaConfig.prodSck[0]     = CY_HBDMA_USBIN_SOCKET_00;   /* Producer Socket ID: USB INGRESS */
    dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
    dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
    dmaConfig.consSck[0]     = CY_HBDMA_VIRT_SOCKET_RD;    /* Consumer Socket ID: HB-SRAM */
    dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
    dmaConfig.eventEnable    = 0;                          /* Event Notifications enable bit map*/
    dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk;
    dmaConfig.cb             = Cy_USBD_Ep0DmaChn_Cb;       /* Callback for transfer completion. */
    dmaConfig.userCtx        = (void *)pUsbdCtxt;          /* Pass USB stack context as user context. */

    mgrStat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr,
                                      &(pUsbdCtxt->outEp0DmaUsb3Ch),
                                      &dmaConfig);
    if (mgrStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_USBD_ERR("HB-DMA-OUT Ch failed 0x%x\r\n", mgrStat);
        return CY_USBD_STATUS_FAILURE;
    }

    dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;    /* DMA Channel type: from HB-SRAM to USB3 IP */
    dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;    /* Producer Socket ID: HB-SRAM */
    dmaConfig.consSck[0]     = CY_HBDMA_USBEG_SOCKET_00;   /* Consumer Socket ID: USB EGRESS */

    mgrStat = Cy_HBDma_Channel_Create(pUsbdCtxt->pHBDmaMgr,
                                      &(pUsbdCtxt->inEp0DmaUsb3Ch),
                                      &dmaConfig);
    if (mgrStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_USBD_ERR("HB-DMA-IN Ch failed 0x%x\r\n", mgrStat);
        return CY_USBD_STATUS_FAILURE;
    }
    DBG_USBD_INFO("HB-DMA EP0 init done\r\n");
    return CY_USBD_STATUS_SUCCESS;
}


#if FREERTOS_ENABLE
/*******************************************************************************
* Function name: Cy_USBD_TimerCallback
****************************************************************************//**
*
* Timer Callback used for USBD tasks like power management.
*
* \param xTimer
* Timer Handle.
*
* \return
* None
*
*******************************************************************************/
static void
Cy_USBD_TimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pvTimerGetTimerID(xTimer);

    /* If LPM (L1, U1 or U2) enable is pending and there is no ongoing control request, enable it. */
    if (pUsbdCtxt->lpmEnablePending) {
        pUsbdCtxt->lpmEnablePending = false;

        if (!pUsbdCtxt->setupReqActive) {
            if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
                Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, true);
            } else {
                Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, false);
            }
        }
    }

    if (pUsbdCtxt->lpbkStateTimerStarted) {
        pUsbdCtxt->lpbkStateTimerStarted = false;
        Cy_USBSS_Cal_ReleaseLTSSM(pUsbdCtxt->pSsCalCtxt);
    }
}

/*******************************************************************************
* Function name: Cy_USBD_Usb3ConnTimerCb
****************************************************************************//**
*
* Timer Callback used to enable USB 3.x connection after USB 2.x Bus Reset
*
* \param xTimer
* Timer Handle.
*
* \return
* None
*
*******************************************************************************/
static void
Cy_USBD_Usb3ConnTimerCb (TimerHandle_t xTimer)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pvTimerGetTimerID(xTimer);

    if ((!(Cy_USBSS_Cal_IsEnabled(pUsbdCtxt->pSsCalCtxt))) && (pUsbdCtxt->usb3ConnectTimerStarted)) {
        /* Enable USB 3.x connection where required. */
        pUsbdCtxt->usb3ConnectTimerStarted = false;
        if (!pUsbdCtxt->ssOnBusResetDisable) {
            Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_SS_REEN);
            Cy_USBD_ConnectSsDevice(pUsbdCtxt);
        }
    }
}
#else
/*******************************************************************************
* Function name: Cy_USBD_LpmEnableTask
****************************************************************************//**
*
* Delayed USBD task used to re-enable Low Power Mode transitions.
*
* \param pUsbdCtxt
* USBD stack context.
*
* \return
* None
*******************************************************************************/
static void
Cy_USBD_LpmEnableTask (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    /* If LPM (L1, U1 or U2) enable is pending and there is no ongoing control request, enable it. */
    if (pUsbdCtxt->lpmEnablePending) {
        pUsbdCtxt->lpmEnablePending = false;

        if (!pUsbdCtxt->setupReqActive) {
            if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
                Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, true);
            } else {
                Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, false);
            }
        }
    }
}
#endif /* FREERTOS_ENABLE */

/*******************************************************************************
* Function name: Cy_USB_USBD_Init
****************************************************************************//**
*
* This function initializes USBD layer and activates CAL layer initialization
* function for SS and HS controller.
*
* \param pAppCtxt
* application layer context pointer(void).
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCpuDmacBase
* Base address for CPU DMA.
*
* \param pCalCtxt
* CAL layer context pointer for HS controller.
*
* \param pSsCalCtxt
* CAL layer context pointer for SS controller.
*
* \param pHbDmaMgrCtxt
* High Bandwidth DMA MgrCtxt.
*
* \return
* CY_USBD_STATUS_SUCCESS if the operation is successful.
* CY_USBD_STATUS_FAILURE if the opration is failed.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_Init (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                  DMAC_Type *pCpuDmacBase, cy_stc_usb_cal_ctxt_t *pCalCtxt,
                  cy_stc_usbss_cal_ctxt_t *pSsCalCtxt,
                  cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    cy_en_usbd_ret_code_t  retCode = CY_USBD_STATUS_SUCCESS;
    uint32_t intfNum;

#if FREERTOS_ENABLE
    BaseType_t status = pdFALSE;
#endif /* FREERTOS_ENABLE */

    DBG_USBD_TRACE("Cy_USB_USBD_Init  >>\r\n");
    pUsbdCtxt->pCalCtxt = pCalCtxt;
    pUsbdCtxt->pSsCalCtxt = pSsCalCtxt;
    pUsbdCtxt->pAppCtxt = pAppCtxt;
    pUsbdCtxt->pCpuDmacBase = pCpuDmacBase;
    pUsbdCtxt->channel0 = 0x00;
    pUsbdCtxt->channel1 = 0x01;
    pUsbdCtxt->pHBDmaMgr = pHbDmaMgrCtxt;

    /* Make HS device Invisible on BUS */
    Cy_USB_USBD_DisableHsDevice(pUsbdCtxt);

    /* Initialize dscr pointers so that application can set required dscr */
    Cy_USBD_InitUsbDscrPtrs(&(pUsbdCtxt->dscrs));

    /* Reset all callbacks */
    pUsbdCtxt->busResetCb = NULL;
    pUsbdCtxt->busResetDoneCb = NULL;
    pUsbdCtxt->busSpeedCb = NULL;
    pUsbdCtxt->setupCb = NULL;
    pUsbdCtxt->suspendCb = NULL;
    pUsbdCtxt->resumeCb = NULL;
    pUsbdCtxt->setConfigCb = NULL;
    pUsbdCtxt->setIntfCb = NULL;
    pUsbdCtxt->statusStageComplCb = NULL;
    pUsbdCtxt->ep0RecvCb = NULL;
    pUsbdCtxt->debugSlpCb = NULL;

    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
        pUsbdCtxt->numOfAltSettings[intfNum] = 0x00;
    }

    /* initialize USBD data structure.
     * Call Controller initialization function.
     *  - Call FS PHY Initialization function.
     *  - Call HS PHY initialization function.
     *  - HS Clock will be disable in reset_done if required.
     */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->startHsDevSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->startSsDevSpeed = CY_USBD_USB_DEV_SS_GEN1;
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->selfPowered = 0x00;
    pUsbdCtxt->remoteWakeupAbility = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;
    pUsbdCtxt->usbDeviceStat = 0x00;
    pUsbdCtxt->termDetectCount = 0x00;
    pUsbdCtxt->disableHsOnComplianceEntry = true;
    pUsbdCtxt->intfRemoteWakeEnabled = 0x00;
    pUsbdCtxt->numOfIntfInActiveCfg = 0x00;
    pUsbdCtxt->ssOnBusResetDisable = false;

    /* Initialize the USBHS and USBSS IP */
    Cy_USBHS_Cal_Init(pUsbdCtxt->pCalCtxt, pUsbdCtxt, Cy_USBD_ProcessMsg);

    if (pHbDmaMgrCtxt != NULL) {
        Cy_USBSS_Cal_Init(pUsbdCtxt->pSsCalCtxt, pUsbdCtxt, Cy_USBD_ProcessMsg);
        Cy_USBSS_Cal_InitEventLog(pUsbdCtxt->pSsCalCtxt, pUsbdCtxt->pUsbEvtBuf, pUsbdCtxt->usbEvtLogSize);
    } else {
        DBG_USBD_ERR("SS_Cal_Init not called:pHbDmaMgrCtxt NULL\r\n");
    }

    /* This function takes care endp initialization for HS and SS */
    Cy_USB_USBD_EndpInit(pUsbdCtxt);

    /* USBD uses CPUDMA for HS and this setup DMA portion for USBD. */
    retCode = Cy_USB_USBD_cpuDmaInit(pUsbdCtxt);

    if (pHbDmaMgrCtxt != NULL) {

        /* Set the default elastic buffer half depth after first time Cal_Init. */
        Cy_USBSS_Cal_SetGen2EBDepth(pUsbdCtxt->pSsCalCtxt, FX3G2_DFLT_GEN2_EB_DEPTH);

        /* Initialize the HB-DMA channel for EP0 IN and OUT */
        Cy_USB_Create_Ep0HbDmaChannels(pUsbdCtxt);
    }

#if FREERTOS_ENABLE
    /* create queue and register it to kernel. */
    pUsbdCtxt->usbdMsgQueue = xQueueCreate(CY_USB_USBD_MSG_QUEUE_SIZE,
                                                    CY_USB_USBD_MSG_SIZE);
    DBG_USBD_INFO("createUsbdQueue\r\n");
    vQueueAddToRegistry(pUsbdCtxt->usbdMsgQueue, "USBDMsgQueue");

    /* Create task and check status to confirm task created properly. */
    status = xTaskCreate(Cy_USBD_TaskHandler, "UsbdTask", CY_USBD_TASK_STACK_DEPTH,
                        (void *)pUsbdCtxt, CY_USBD_TASK_PRIORITY, &(pUsbdCtxt->usbdTaskHandle));

    if (status != pdPASS) {
        DBG_USBD_ERR("UsbdTaskcreateFail\r\n");
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /* Set timer period to 50 ticks or 50 ms. */
    pUsbdCtxt->usbdTimerHandle = xTimerCreate(NULL, 50, pdFALSE, (void *)pUsbdCtxt, Cy_USBD_TimerCallback);
    if (pUsbdCtxt->usbdTimerHandle == NULL) {
        DBG_USBD_ERR("UsbdTimerCreateFail\r\n");
        retCode = CY_USBD_STATUS_FAILURE;
    }

    pUsbdCtxt->usb3ConnectTimer = xTimerCreate(NULL, 400, pdFALSE, (void *)pUsbdCtxt, Cy_USBD_Usb3ConnTimerCb);
    if (pUsbdCtxt->usb3ConnectTimer == NULL) {
        DBG_USBD_ERR("UsbdTimerCreateFail\r\n");
        retCode = CY_USBD_STATUS_FAILURE;
    }
#else
    pUsbdCtxt->nextTaskTick = 0;
#endif /* FREERTOS_ENABLE */

    /* Make State Enable, enable interrupt, and then make device visible. */
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ENABLE;
    pUsbdCtxt->strDscrAvailable = FALSE;
    pUsbdCtxt->EnumerationDone = false;
    Cy_USBD_AddEvtToLog(pCalCtxt->pUsbdCtxt, CY_HSCAL_EVT_USBD_INIT);
    DBG_USBD_TRACE("Cy_USB_USBD_Init  <<\r\n");
    return(retCode);
}   /* end of function Cy_USB_USBD_Init() */

/*******************************************************************************
* Function name: Cy_USBD_DebugRegisterCallback
****************************************************************************//**
*
*  This API will be used by CDC (Debug) interface to register required callback
*
* \param pUsbdCtxt
* USBD Context
*
* \param debugOutEp
* CDC (Debug) OUT endpoint number.
*
* \param callBackType
* USBD callback type
*
* \param callBackFunc
* Registered callback function
*
* \return
* CY_USBD_STATUS_SUCCESS if the operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if ednpoint number is out of range
* CY_USBD_STATUS_INVALID_CALLBACK_TYPE if callback type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_DebugRegisterCallback (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint8_t debugOutEp,
                            cy_en_usb_usbd_cb_t callBackType,
                            cy_usb_usbd_callback_t callBackFunc)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if ((debugOutEp == 0 ) || (debugOutEp >= CY_USB_MAX_ENDP_NUMBER)) {
        return(CY_USBD_STATUS_BAD_PARAM);
    }

    switch (callBackType) {
        case CY_USB_USBD_CB_SLP:
            pUsbdCtxt->debugSlpCb = callBackFunc;
            pUsbdCtxt->debugOutEp = debugOutEp;
            break;
        default:
            retCode = CY_USBD_STATUS_INVALID_CALLBACK_TYPE;
            break;
    }
    return(retCode);
}   /* end of function Cy_USBD_DebugRegisterCallback() */


/*******************************************************************************
* Function name: Cy_USBD_RegisterCallback
****************************************************************************//**
*
* This API will be used by application to register required callback.
*
* \param pAppCtxt
* application layer context pointer(void).
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param callBackType
* Base address for CPU DMA.
*
* \param callBackFunc
* Base address for DATAWIRE0.
*
* \return
* CY_USBD_STATUS_SUCCESS if the operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_INVALID_CALLBACK_TYPE if callback type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_RegisterCallback (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_en_usb_usbd_cb_t callBackType,
                          cy_usb_usbd_callback_t callBackFunc)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("RegCal:usbdCtxt NULL\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    switch (callBackType) {
        case CY_USB_USBD_CB_RESET:
            pUsbdCtxt->busResetCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_RESET_DONE:
            pUsbdCtxt->busResetDoneCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_BUS_SPEED:
            pUsbdCtxt->busSpeedCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SETUP:
            pUsbdCtxt->setupCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SUSPEND:
            pUsbdCtxt->suspendCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_RESUME:
            pUsbdCtxt->resumeCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SET_CONFIG:
            pUsbdCtxt->setConfigCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SET_INTF:
            pUsbdCtxt->setIntfCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_STATUS_STAGE_COMP:
            pUsbdCtxt->statusStageComplCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_L1_SLEEP:
            pUsbdCtxt->l1SleepCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_L1_RESUME:
            pUsbdCtxt->l1ResumeCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_ZLP:
            pUsbdCtxt->zlpCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SLP:
            pUsbdCtxt->slpCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_DONE:
            pUsbdCtxt->doneCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_DISCONNECT:
            pUsbdCtxt->DisconnectCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SETADDR:
            pUsbdCtxt->setAddrCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_EP0_RCV_DONE:
            pUsbdCtxt->ep0RecvCb  = callBackFunc;
            break;

        default:
            DBG_USBD_ERR("RegCal:default\r\n");
            retCode = CY_USBD_STATUS_INVALID_CALLBACK_TYPE;
            break;
    }
    return(retCode);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_SetDscr
****************************************************************************//**
*
* This API will be used by application to provide various standard descriptors
* to USBD layer and USBD layer will use this descriptor in "fast enumeration".
* It is callers responsibility to provide proper descriptor with right index.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param dscrType
* Type of descriptor.
*
* \param dscrIndex
* Descriptor Index.
*
* \param pDscr
* Pointer to descriptor data based on type.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to descriptor is NULL.
* CY_USBD_STATUS_INVALID_DSCR_TYPE if descriptor type not supported.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_SetDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                 cy_en_usb_set_dscr_type_t dscrType,
                 uint8_t dscrIndex,
                 uint8_t *pDscr)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;
    uint8_t numIntf;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SetDscr:usbdCtxt NULL\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (NULL == pDscr) {
        DBG_USBD_ERR("SetDscr:pDscr NULL\r\n");
        return(CY_USBD_STATUS_BAD_PARAM);
    }

    switch (dscrType) {

        case CY_USB_SET_HS_DEVICE_DSCR:
            pUsbdCtxt->dscrs.pUsbDevDscr = pDscr;
            break;

        case CY_USB_SET_SS_DEVICE_DSCR:
            pUsbdCtxt->dscrs.pUsbSsDevDscr = pDscr;
            break;

        case CY_USB_SET_DEVICE_QUAL_DSCR:
            pUsbdCtxt->dscrs.pUsbDevQualDscr = pDscr;
            break;

        case CY_USB_SET_FS_CONFIG_DSCR:
            numIntf = *(pDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
            if (numIntf <= CY_USB_MAX_INTF) {
                pUsbdCtxt->dscrs.pUsbFsCfgDscr = pDscr;
            } else {
                pUsbdCtxt->dscrs.pUsbFsCfgDscr = NULL;
                retCode = CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED;
            }
            break;

        case CY_USB_SET_HS_CONFIG_DSCR:
            numIntf = *(pDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
            if (numIntf <= CY_USB_MAX_INTF) {
                pUsbdCtxt->dscrs.pUsbHsCfgDscr = pDscr;
            } else {
                pUsbdCtxt->dscrs.pUsbHsCfgDscr = NULL;
                retCode = CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED;
            }
            break;

        case CY_USB_SET_SS_CONFIG_DSCR:
            numIntf = *(pDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
            if (numIntf <= CY_USB_MAX_INTF) {
                pUsbdCtxt->dscrs.pUsbSsCfgDscr = pDscr;
            } else {
                pUsbdCtxt->dscrs.pUsbSsCfgDscr = NULL;
                retCode = CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED;
            }
            break;

        case CY_USB_SET_STRING_DSCR:
            /* TBD: Need to have boundry check when string dscr is updated. */
            if (dscrIndex > CY_USBD_MAX_STR_DSCR_INDX) {
                return(CY_USBD_STATUS_INVALID_INDEX);
            }

            pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex] = pDscr;
            /*
             * If string descriptor is not register (other than index 0)
             * then device should return stall and strDscrAvailable will
             * be used for the same.
             */
            if ((dscrIndex != 0x00) &&
                (pUsbdCtxt->strDscrAvailable == FALSE)) {
                pUsbdCtxt->strDscrAvailable = TRUE;
            }
            break;

        case CY_USB_SET_HS_BOS_DSCR:
            pUsbdCtxt->dscrs.pUsbHsBosDscr = pDscr;
            break;

        case CY_USB_SET_SS_BOS_DSCR:
            pUsbdCtxt->dscrs.pUsbSsBosDscr = pDscr;
            break;

        default:
            DBG_USBD_ERR("SetDscr:default\r\n");
            retCode = CY_USBD_STATUS_INVALID_DSCR_TYPE;
            break;
    }
    return(retCode);
}   /* end of function   */


/*******************************************************************************
* Function name: Cy_USBD_GetDeviceSpeed
****************************************************************************//**
*
* This API returns device's operating speed.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_USB_DEV_NOT_CONNECTED if device is not connected.
* cy_en_usb_speed_t speed of device.
*
*******************************************************************************/
cy_en_usb_speed_t
Cy_USBD_GetDeviceSpeed (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        /*
         * Since we do not have a valid context structure, return not
         * connected status.
         */
        DBG_USBD_ERR("GetDevSpe:usbdCtxt NULL\r\n");
        return(CY_USBD_USB_DEV_NOT_CONNECTED);
    }
    return(pUsbdCtxt->devSpeed);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_SetDeviceSpeed
****************************************************************************//**
*
* This API force operating speed of dvice.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param speed
* speed of device.
*
* \return
* None
*
*******************************************************************************/
void
Cy_USBD_SetDeviceSpeed (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                        cy_en_usb_speed_t speed)
{
    if (NULL == pUsbdCtxt) {
        /* can not change speed */
        DBG_USBD_ERR("SetDevSpe:usbdCtxt NULL\r\n");
        return;
    }
    /* Take care FS/HS case here. Once SS case comes then add the code. */
    if ((CY_USBD_USB_DEV_FS == speed) || (CY_USBD_USB_DEV_HS == speed)) {

        Cy_USBHS_Cal_SetControllerSpeed(pUsbdCtxt->pCalCtxt, speed);
        pUsbdCtxt->devSpeed = speed;
    }
    return;
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_SetDeviceSpeedAtUSBDOnly
****************************************************************************//**
*
* This API is used to set device speed only at USBD layer.
* This API should be used when application wants to fetch different speed
* dependent data even before enumaration is done. During enumeration speed
* field will be overwritten at USBD layer.
* It is application's responsibility to pass right speed.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param speed
* speed of device.
*
* \return
* None
*
*******************************************************************************/
void
Cy_USBD_SetDeviceSpeedAtUSBDOnly (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_en_usb_speed_t speed)
{
    if (NULL == pUsbdCtxt) {
        /* can not change speed */
        DBG_USBD_ERR("SetDevSpe:usbdCtxt NULL\r\n");
        return;
    }
    pUsbdCtxt->devSpeed = speed;
    return;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetDeviceState
****************************************************************************//**
*
* This API returns present state of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USB_DEVICE_STATE_INVALID if usbd context NULL.
* cy_en_usb_device_state_t present state of device.
*
*******************************************************************************/
cy_en_usb_device_state_t
Cy_USBD_GetDeviceState (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        /*
         * Since we do not have a valid context structure,
         * return invalid state. */
        DBG_USBD_ERR("GetDevSta:usbdCtxt NULL\r\n");
        return(CY_USB_DEVICE_STATE_INVALID);
    }
    return(pUsbdCtxt->devState);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_FindEndp0MaxPktSize
****************************************************************************//**
*
* This API finds endpoint 0 max packet size from device descriptor. device
* descriptors are different at different speed.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param devSpeed
* speed of device for which endpoint 0 size is required.
*
* \param pMaxPktSize
* Max packet size of endpoint 0 will be sotored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM in all other case.
* cy_en_usb_device_state_t present state of device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_FindEndp0MaxPktSize (uint8_t *pDevDscr, cy_en_usb_speed_t devSpeed,
                             uint32_t *pMaxPktSize)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_BAD_PARAM;

    if ((devSpeed == CY_USBD_USB_DEV_FS) ||
            (devSpeed == CY_USBD_USB_DEV_HS)) {
        if (NULL != pDevDscr) {
            *pMaxPktSize =
            (*(uint8_t *)(pDevDscr + CY_USB_DEVICE_DSCR_OFFSET_MAX_PKT_SIZE));
            retCode = CY_USBD_STATUS_SUCCESS;
        } else {
            *pMaxPktSize = 0x00;
        }
    } else {
        if (NULL != pDevDscr) {
            /*
             * As per the spec, endpoint0 size should be 512. Irrespective
             * of what written in device descriptor.
             */
            *pMaxPktSize = 512;
            retCode = CY_USBD_STATUS_SUCCESS;
        }  else {
            *pMaxPktSize = 0x00;
        }
    }
    return(retCode);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetLpmBosUSBExt
****************************************************************************//**
*
*  Get information about LPM support from USB ext descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pLpmSupp
* Information about LPM support is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetLpmBosUSBExt (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pLpmSupp)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetLpmBosUSBExt:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    if (pLpmSupp == NULL) {
        DBG_USBD_ERR("GetLpmBosUSBExt:pLpmSupp NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetLpmBosUSBExt:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {
        if ((*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
             (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
            (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
             (uint8_t)CY_USB_DEVICE_CAP_TYPE_USB2_EXT)) {
            /* First bit gives info about LPM supported or not*/
            *pLpmSupp =
            (*(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE) & 0x02);
            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }

    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_GetAttributeBosUSBExt
****************************************************************************//**
*
*  Get attribute from USB ext descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pAttribute
* required information related to attribute will be store here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetAttributeBosUSBExt (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t *pAttribute)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetAttributeBosUSBExt:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    if (pAttribute == NULL) {
        DBG_USBD_ERR("GetAttributeBosUSBExt:pAttribute NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetAttributeBosUSBExt:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_USB2_EXT)) {

            *pAttribute =
              *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE)) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 1)) << 8) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 2)) << 16) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 3)) << 24);

            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_GetAttributeBosSS
****************************************************************************//**
*
*  Get attribute from Super speed BoS descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pLpmSupp
* Max packet size of endpoint 0 will be sotored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetAttributeBosSS (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pAttribute)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetAttributeBosSS:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    if (pAttribute == NULL) {
        DBG_USBD_ERR("GetAttributeBosSS:pAttribute NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetAttributeBosSS:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_SS_USB)) {

            *pAttribute =
            *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_ATTRIBUTE));
            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USBD_GetSpeedSuppBosSS
****************************************************************************//**
*
*  Get Speed supported from Super speed descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pSpeedSupp
* speed information is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetSpeedSuppBosSS (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t *pSpeedSupp)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetSpeedSuppBosSS:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    if (pSpeedSupp == NULL) {
        DBG_USBD_ERR("GetSpeedSuppBosSS:pSpeedSupp NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetSpeedSuppBosSS:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_SS_USB)) {

            *pSpeedSupp =
              *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_SPEED_SUPPORT)) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_SPEED_SUPPORT + 1)) << 8);

            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_GetU1ExitLatBosSS
****************************************************************************//**
*
*   Get U1 exit latency  from Super speed BoS descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pU1ExitLat
* U1 exit latency will be stored here.
*
* \param pU2ExitLat
* U2 exit latency will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered OR pAttribute is NULL.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetU1U2ExitLatBosSS (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pU1ExitLat, uint16_t *pU2ExitLat)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetU1U2ExitLatBosSS:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    if (pU1ExitLat == NULL || pU2ExitLat == NULL) {
        DBG_USBD_ERR("GetU1U2ExitLatBosSS:pU1ExitLat OR pU2ExitLat NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr)  {
        DBG_USBD_ERR("GetU1U2ExitLatBosSS:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_SS_USB)) {

            *pU1ExitLat =
            *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_U1_EXIT_LAT));

            *pU2ExitLat =
              *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_U2_EXIT_LAT)) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_U2_EXIT_LAT + 1)) << 8);
            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */

/*
 * Function: Cy_USBD_GetAttributeBosSSP()
 * Description: Get attribute from Super speed plus descriptor.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pAttribute.
 * return: CY_USBD_STATUS_CTXT_NULL, CY_USBD_STATUS_PTR_NULL,
 *         or CY_USBD_STATUS_SUCCESS.
 */
cy_en_usbd_ret_code_t
Cy_USBD_GetAttributeBosSSP (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t *pAttribute)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetAttributeBosSSP:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;
    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetAttributeBosSSP:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_SS_PLUS)) {

            *pAttribute =
              *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE)) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 1)) << 8) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 2)) << 16) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE + 3)) << 24);

            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USBD_GetFunctSupportBosSSP
****************************************************************************//**
*
*  Get functionality supported from Super speed plus descriptor
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pFuncSupp
* Information about functionality supported is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_PTR_NULL if BOS descriptor is not registered.
* CY_USBD_STATUS_FAILURE in all other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetFunctSupportBosSSP (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t *pFuncSupp)
{
    uint8_t *pBosDscr;
    uint8_t  bNumDevCap;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetFunctSupportBosSSP:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    pBosDscr = pUsbdCtxt->dscrs.pUsbSsBosDscr;

    if (NULL == pBosDscr) {
        DBG_USBD_ERR("GetFunctSupportBosSSP:pBosDscr NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    bNumDevCap = *(pBosDscr + CY_USB_BOS_DSCR_OFFSET_DEVICE_CAP);
    /* with this reached to various capability descriptor. */
    pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));

    while (bNumDevCap != 0x00) {

        if ( (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_TYPE) ==
              (uint8_t)CY_USB_DSCR_TYPE_DEVICE_CAP) &&
             (*(pBosDscr + CY_USB_CAP_DSCR_OFFSET_CAP_TYPE) ==
              (uint8_t)CY_USB_DEVICE_CAP_TYPE_SS_PLUS)) {

            *pFuncSupp =
              *((uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_FUNC_SUPPORT)) +
              ((*(uint8_t *)(pBosDscr + CY_USB_CAP_DSCR_SSUSB_OFFSET_FUNC_SUPPORT + 1)) << 8);

            break;
        }
        /* if dscr is not USB2.0 EXT then move to next capability dscr */
        pBosDscr = (pBosDscr + (uint8_t)*(pBosDscr + CY_USB_BOS_DSCR_OFFSET_LEN));
        bNumDevCap--;
    }
    if (bNumDevCap != 0x00) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_isCfgValid
****************************************************************************//**
*
*  This function checks validity of given configuration.
*
* \param cfgNum
* Configuration Number.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if config is valid.
* FALSE if config is not valid.
*
*******************************************************************************/
bool
Cy_USBD_isCfgValid (uint8_t cfgNum, const uint8_t *pCfgDscr)
{
    uint8_t cfgValue;

    if (NULL == pCfgDscr) {
        DBG_USBD_ERR("IsCfgVal:pCfgDscr NULL\r\n");
        return (FALSE);
    }

    cfgValue = *(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_CFG_VALUE);
    if (cfgValue == cfgNum) {
        return(TRUE);
    }
    return(FALSE);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveCfgNum
****************************************************************************//**
*
* This API called by application to know active configuration number.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCfgNum
* config number will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_GetActiveCfgNum (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pCfgNum)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetActCfgNum:usbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    *pCfgNum = pUsbdCtxt->activeCfgNum;
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveCfgDscr
****************************************************************************//**
*
* This API called by application to get active configuration descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* Pointer to active configuration descriptor.
*
*******************************************************************************/
uint8_t  *
Cy_USB_USBD_GetActiveCfgDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetActCfgDsc:usbdCtxt NULL\r\n");
        return(NULL);
    }
    return(pUsbdCtxt->pActiveCfgDscr);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_FindNumOfIntf
****************************************************************************//**
*
*  This function finds number of interface supported in given configation.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* 0x00 if Configuration descriptor is NULL.
* Number of interface supported by configuration.
*
*******************************************************************************/
uint8_t
Cy_USBD_FindNumOfIntf (const uint8_t *pCfgDscr)
{
    uint8_t numIntf = 0x00;

    if (NULL != pCfgDscr) {
        numIntf = *(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
    } else {
        DBG_USBD_ERR("FindNumOfIntf:pCfgDscr NULL\r\n");
    }
    return(numIntf);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_FindSelfPower
****************************************************************************//**
*
*  This function finds device supports Self power or BUS power.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if device supports self power.
* FALSE if device supports only BUS power.
*
*******************************************************************************/
bool
Cy_USBD_FindSelfPower (const uint8_t *pCfgDscr)
{
    uint8_t bmAttributes;

    if (NULL != pCfgDscr) {
        bmAttributes = *(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_ATTRIBUTE);
        if (bmAttributes & CY_USB_CFG_DSCR_SELF_POWER_MASK) {
            return(TRUE);
        }
    } else {
        DBG_USBD_ERR("FindSelfPower:pCfgDscr NULL\r\n");
    }
    return(FALSE);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_FindRemoteWakeupAbility
****************************************************************************//**
*
*  This function finds device supports remote wakeup or not.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if device supports remote wakeup.
* FALSE if device does not support remote wakeup.
*
*******************************************************************************/
bool
Cy_USBD_FindRemoteWakeupAbility (uint8_t *pCfgDscr)
{
    uint8_t bmAttributes;

    if (NULL != pCfgDscr) {
        bmAttributes = *(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_ATTRIBUTE);
        if (bmAttributes & CY_USB_CFG_DSCR_REMOTE_WAKEUP_MASK) {
            return(TRUE);
        }
    } else {
        DBG_USBD_ERR("FindRemoteWakeupAbility:pCfgDscr NULL\r\n");
    }
    return(FALSE);
}   /* End of function. */


/*******************************************************************************
* Function name: Cy_USBD_GetRemoteWakeupStatus
****************************************************************************//**
*
*  This function checks L2 remote wakeup status and return information.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* TRUE if remote wakeup enabled in HW register.
* FALSE if remote wakeup disabled in HW register.
*
*******************************************************************************/
bool
Cy_USBD_GetRemoteWakeupStatus (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    bool retVal = FALSE;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetRemWakeSta:usbdCtxt Null\r\n");
        return(retVal);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        if ((pUsbdCtxt->remoteWakeupEnable) &&
             (Cy_USBHS_Cal_GetRemoteWakeupStatus(pUsbdCtxt->pCalCtxt))) {
             retVal = TRUE;
        } else {
             retVal = FALSE;
        }
    } else {
        /* TBD: CALL SS related CAL function. */

    }
    return (retVal);
}   /* End of function. */


/*******************************************************************************
* Function name: Cy_USBD_SignalRemoteWakeup
****************************************************************************//**
*
*  This function triggers HW to start/end remotewakeup signaling.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param startEnd
* True to start signaling and false to end signaling.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_SignalRemoteWakeup (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool startEnd)
{
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        if (pUsbdCtxt->remoteWakeupEnable) {
            if (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) {
                /* Make sure PHY out of suspend before start signaling */
                Cy_USBHS_Cal_HsHandleL2Resume(pUsbdCtxt->pCalCtxt);
            }
            Cy_USBHS_Cal_SignalRemotWakup(pUsbdCtxt->pCalCtxt, startEnd);
        } else {
            DBG_USBD_ERR("remoteWakeupEnable False. Dont send Signal\r\n");
        }

    } else {
        /* TBD: CALL SS related CAL function. */

    }
    return;
}   /* End of function. */


/*******************************************************************************
* Function name: Cy_USBD_GetIntfDscr
****************************************************************************//**
*
*  This function called by application to get interface descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* Interface number.
*
* \param altSetting
* Alternate setting number.
*
* \return
* NULL if USBD context pointer is NULL.
* pointer to Interface descriptor.
*
*******************************************************************************/
uint8_t *
Cy_USBD_GetIntfDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t intfNum,
                     uint8_t altSetting)
{
    uint8_t  *pSearch;
    int16_t totalLength;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetIntDsc:usbdCtxt Null\r\n");
        return(NULL);
    }

    if (pUsbdCtxt->EnumerationDone) {
        pSearch = pUsbdCtxt->pActiveCfgDscr;
    } else {
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
            pSearch = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
        } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
            pSearch = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
        } else {
            pSearch = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
        }
    }

    if (pSearch == NULL) {
        DBG_USBD_ERR("GetIntDsc:activeCfg Null\r\n");
        return(NULL);
    }

    totalLength = *((uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN)) +
        ((*(uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN + 1)) << 8);

    while ((totalLength > 0x00) && (pSearch != NULL)) {
       if ((*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_INTR_DSCR) &&
             (intfNum == *(pSearch + CY_USB_INTF_DSCR_OFFSET_NUMBER)) &&
             (altSetting == *(pSearch + CY_USB_INTF_DSCR_OFFSET_ALT_SETTING))) {
           return(pSearch);
       } else {
           /* If it is not endpoint descriptor then move by dscr length. */
           totalLength -= (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
           pSearch =
               pSearch + (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
       }
    }
    /* Interface not found then return NULL. */
    return(NULL);
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USBD_isIntfValid
****************************************************************************//**
*
*  This function checks given interface is part of configuration descriptor or not.
*
* \param intfNum
* Interface number.
*
* \param pCfgDscr
* pointer to configuration descriptor.
*
* \return
* TRUE if given interfae number is part of configuration descriptor.
* FALSEif given interface is not part of configuration descriptor.
*
*******************************************************************************/
bool
Cy_USBD_isIntfValid (uint16_t intfNum, uint8_t *pCfgDscr)
{
    /*
     * Exaustive implementation also possible with going through all interface.
     */
    uint8_t numOfIntf;

    if (NULL == pCfgDscr) {
        DBG_USBD_ERR("IsIntVal:pCfgDscr Null\r\n");
        return (FALSE);
    }
    numOfIntf = *(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
    if (intfNum < numOfIntf) {
        return(TRUE);
    }

    return(FALSE);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_FindNumOfEndp
****************************************************************************//**
*
*  This function  gives number of Endpoints supported by given interface.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* 0x00 if interface descriptor is NULL.
* Number of endpoints.
*
*******************************************************************************/
uint8_t
Cy_USBD_FindNumOfEndp (uint8_t *pIntfDscr)
{
    uint8_t numOfEndp = 0x00;

    if (NULL != pIntfDscr) {
        numOfEndp = *(pIntfDscr + CY_USB_INTF_DSCR_OFFSET_NUM_ENDP);
    }
    return(numOfEndp);
}

/*******************************************************************************
* Function name: Cy_USBD_GetEndpDscr
****************************************************************************//**
*
*  This function to fetch endpoint descriptor for a given interface descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to endpoint descriptors.
*
*******************************************************************************/
uint8_t *
Cy_USBD_GetEndpDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pIntfDscr)
{
    uint8_t *pSearch;
    int16_t totalLength;
    uint16_t startIndex = 0x00;
    uint8_t intfNum, altSetting, numOfEndp;
    bool intfFound = FALSE;

    /*
     * 1. Check all pointer against null.
     * 2. find cfg dscr based on enumeration done or not.
     *     - If enumeration done then take from activeCfgDscr
     *     - otherwise take as per speed config.
     * 3. Find required intf dscr and fetch endp dscr from there.
     * 4. use wLength field to make sure search should not go beyond
     *    cfg dscr limit.
     */

    if ((NULL == pUsbdCtxt) || (NULL == pIntfDscr)) {
        DBG_USBD_ERR("GetEndDscr:usbdCtxt Null\r\n");
        return(NULL);
    }

    if (*(pIntfDscr + CY_USB_DSCR_OFFSET_TYPE) !=
        (uint8_t)CY_USB_DSCR_TYPE_INTF) {
        DBG_USBD_ERR("GetEndDscr:NotIntDscrType\r\n");
        return(NULL);
    }

    /* Based on enumeration  and speed of device referece point changes. */
    if (pUsbdCtxt->EnumerationDone) {
        pSearch = pUsbdCtxt->pActiveCfgDscr;
    } else {
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
            pSearch = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
        } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
            pSearch = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
        } else {
            pSearch = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
        }
    }

    if (pSearch == NULL) {
        DBG_USBD_ERR("GetEndDscr:pActiveCfg Null\r\n");
        return(NULL);
    }

    numOfEndp = *(pIntfDscr + CY_USB_INTF_DSCR_OFFSET_NUM_ENDP);
    if (numOfEndp == 0x00 ) {
        DBG_USBD_ERR("GetEndDscr:numEndp0\r\n");
        return(NULL);
    }

    intfNum = *(pIntfDscr + CY_USB_INTF_DSCR_OFFSET_NUMBER);
    altSetting = *(pIntfDscr + CY_USB_INTF_DSCR_OFFSET_ALT_SETTING);

    totalLength = *((uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN)) +
        ((*(uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN + 1)) << 8);

    while ((totalLength > 0x00) && (pSearch != NULL)) {
       if ((*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_INTR_DSCR) &&
             (intfNum == *(pSearch + CY_USB_INTF_DSCR_OFFSET_NUMBER)) &&
             (altSetting == *(pSearch + CY_USB_INTF_DSCR_OFFSET_ALT_SETTING))) {
           intfFound = TRUE;
           break;
       } else {
           startIndex += (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
           totalLength -= (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
           pSearch =
               pSearch + (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
       }
    }

    if (intfFound == FALSE) {
        /* Given interface is not available so return NULL */
        DBG_USBD_ERR("GetEndDscr:IntfNotFound\r\n");
        return(NULL);
    }

    pSearch +=  *(pSearch + CY_USB_DSCR_OFFSET_LEN);
    /*
     * Now Index is available as well as remaining length. Find EndpDscr
     * from remaining length. While searching for endpoint descriptor
     * if interface descriptor is seen which means means given intf
     * dscr does not have endp dscr.
     */
    while ((totalLength > 0x00) && (pSearch != NULL)) {
        if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_ENDP_DSCR) {
            return(pSearch);
        } else {
            if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_DSCR_TYPE_INTF) {
                /* reached to another interface so break from here. */
               break;
            }
            totalLength -= (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
            pSearch =
               pSearch + (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
       }
    }
    return(NULL);
}   /* End of Function */

/*******************************************************************************
* Function name: Cy_USBD_GetSsEndpCompDscr
****************************************************************************//**
*
*  This function to fetch super speed endpoint companion descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *
Cy_USBD_GetSsEndpCompDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           uint8_t *pEndpDscr)
{
    uint8_t *pSearch;
    uint8_t dscrLen;
    if ((NULL == pUsbdCtxt) || (NULL == pEndpDscr)) {
        DBG_USBD_ERR("GetSsEndpCompDscr:usbdCtxt/pEndpDscr Null\r\n");
        return(NULL);
    }
    dscrLen =  *(pEndpDscr + CY_USB_DSCR_OFFSET_LEN);
    pSearch = pEndpDscr + dscrLen;

    if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_DSCR_TYPE_SS_ENDP_COMP) {
        return(pSearch);
    }

    /* SS endp companion descriptor is not found so return NULL */
    return(NULL);
}   /* end of function*/


/*******************************************************************************
* Function name: Cy_USBD_GetSspIsoCompDscr
****************************************************************************//**
*
*  This function to fetch super speed plus Iso endpoint companion descriptor from
*  given companion descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pCompDscr
* pointer to endpoint companion descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to Iso endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *
Cy_USBD_GetSspIsoCompDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           uint8_t *pCompDscr)
{
    uint8_t *pSearch;
    uint8_t dscrLen;
    if ((NULL == pUsbdCtxt) || (NULL == pCompDscr)) {
        DBG_USBD_ERR("Cy_USBD_GetSspIsoCompDscr:usbdCtxt/pEndpDscr Null\r\n");
        return(NULL);
    }
    dscrLen =  *(pCompDscr + CY_USB_DSCR_OFFSET_LEN);
    pSearch = pCompDscr + dscrLen;

    if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_DSCR_TYPE_SSP_ISO_ENDP_COMP) {
        return(pSearch);
    }

    /* SSP ISO endp companion descriptor is not found so return NULL */
    return(NULL);
}   /* end of function*/


/*******************************************************************************
* Function name: Cy_USBD_GetSspIsoEndpCompDscr
****************************************************************************//**
*
*  This function to fetch super speed plus Iso endpoint companion descriptor from
*  given endpoint descriptor.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* NULL if any of the parameter is NULL.
* Pointer to super speed plus Iso endpoint companion descriptors.
*
*******************************************************************************/
uint8_t *
Cy_USBD_GetSspIsoEndpCompDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               uint8_t *pEndpDscr)
{
    uint8_t *pSearch;
    uint8_t dscrLen;
    uint8_t bmAttribute;

    if ((NULL == pUsbdCtxt) || (NULL == pEndpDscr)) {
        DBG_USBD_ERR("GetSsEndpCompDscr:usbdCtxt/pEndpDscr Null\r\n");
        return(NULL);
    }
    bmAttribute = *(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE);
    /* Get endpoint type */
    bmAttribute = (bmAttribute & 0x03);

    if (bmAttribute != (uint8_t)CY_USB_ENDP_TYPE_ISO) {
        return(NULL);
    }

    /* Now check if it has companion descriptor or not */
    dscrLen =  *(pEndpDscr + CY_USB_DSCR_OFFSET_LEN);
    pSearch = pEndpDscr + dscrLen;
    if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_DSCR_TYPE_SS_ENDP_COMP) {
        /* Update pEndpDscr to point to endpoint companion descriptor.*/
        pEndpDscr = pSearch;
        /* Now check if it has SSP ISO endpoint companion descriptor or not */
        dscrLen =  *(pEndpDscr + CY_USB_DSCR_OFFSET_LEN);
        pSearch = pEndpDscr + dscrLen;
        if (*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_DSCR_TYPE_SSP_ISO_ENDP_COMP) {
            return(pSearch);
        } else {
            return(NULL);
        }
    }
    return(NULL);
}


/*******************************************************************************
* Function name: Cy_USB_USBD_GetActiveAltSetting
****************************************************************************//**
*
*  This function called by application to know active altSetting for a given
*  interface.
*
* \param pUsbdCtxt
* USBD layer context pointer.

* \param intfNum
* interface number.
*
* \param pAltSetting
* required alt setting pointer.
*
* \return
* CY_USBD_STATUS_CTXT_NULL when context is null.
* CY_USBD_STATUS_FAILURE when active config descriptor is null.
* CY_USBD_STATUS_SUCCESS whem able to fetch altsetting number.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_GetActiveAltSetting (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                 uint8_t intfNum,
                                 uint8_t *pAltSetting)
{
    uint8_t *pCfgDscr;
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetActAltSet:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    /* Based on enumeration  and speed of device referece point changes. */
    if (pUsbdCtxt->EnumerationDone) {
        pCfgDscr = pUsbdCtxt->pActiveCfgDscr;
    } else {
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
            pCfgDscr = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
        } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
            pCfgDscr = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
        } else {
            pCfgDscr = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
        }
    }
    if ((pCfgDscr == NULL) ||
        (intfNum >= (*(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF)))) {
        return(CY_USBD_STATUS_FAILURE);
    }
    *pAltSetting =  (uint8_t)(pUsbdCtxt->altSettings[intfNum]);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_FindAltSetting
****************************************************************************//**
*
* This API gives information about alt setting supported by given interface.
*
* \param pIntfDscr
* pointer to interface descriptor.
*
* \return
* number of alt setting supported by an interface.
*
*******************************************************************************/
uint8_t
Cy_USBD_FindAltSetting (uint8_t *pIntfDscr)
{
    uint8_t altSetting = 0x00;

    if (NULL != pIntfDscr) {
        altSetting = *(pIntfDscr + CY_USB_INTF_DSCR_OFFSET_ALT_SETTING);
    }
    return(altSetting);
}


/*******************************************************************************
* Function name: Cy_USBD_GetNumOfAltSetting
****************************************************************************//**
*
* This API gets number of alt setting in an interface from USBD context data structure.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* interface number.
*
* \return
* number of alt setting supported by interface.
*
*******************************************************************************/
uint8_t
Cy_USBD_GetNumOfAltSetting (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t intfNum)
{
    return(pUsbdCtxt->numOfAltSettings[intfNum]);
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USBD_UpdateNumOfAltSetting
****************************************************************************//**
*
* This API gets number of alt setting in an interface.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param intfNum
* interface number.
*
* \return
* number of alt setting supported by interface.
*
*******************************************************************************/

uint8_t
Cy_USBD_UpdateNumOfAltSetting (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t intfNum)
{
    uint8_t  *pSearch;
    int16_t totalLength;
    uint8_t numOfAltsetting = 0x00;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("GetIntDsc:usbdCtxt Null\r\n");
        return(0);
    }

    if (pUsbdCtxt->EnumerationDone) {
        pSearch = pUsbdCtxt->pActiveCfgDscr;
    } else {
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
            pSearch = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
        } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
            pSearch = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
        } else {
            pSearch = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
        }
    }

    if (pSearch == NULL) {
        DBG_USBD_ERR("GetIntDsc:activeCfg Null\r\n");
        return(0);
    }

    totalLength = *((uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN)) +
        ((*(uint8_t *)(pSearch + CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN + 1)) << 8);

    while ((totalLength > 0x00) && (pSearch != NULL)) {
       if ((*(pSearch + CY_USB_DSCR_OFFSET_TYPE) == (uint8_t)CY_USB_INTR_DSCR) &&
             (intfNum == *(pSearch + CY_USB_INTF_DSCR_OFFSET_NUMBER))) {
        numOfAltsetting++;
        totalLength -= (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
           pSearch =
               pSearch + (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
       } else {
           /* If it is interface descriptor then move by dscr length. */
           totalLength -= (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
           pSearch =
               pSearch + (uint8_t)(*(pSearch + CY_USB_DSCR_OFFSET_LEN));
       }
    }
    /* Interface not found then return NULL. */
    return(numOfAltsetting);
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_EndpDscrValid
****************************************************************************//**
*
*  This API confirms given descriptor is endpoint descriptor or not.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* TRUE if given descriptor is endpoint descriptor.
* FALSE if given descriptor is not endpoint descriptor.
*
*******************************************************************************/
bool
Cy_USBD_EndpDscrValid (uint8_t *pEndpDscr)
{
    if ((NULL == pEndpDscr) ||
       (*(pEndpDscr + CY_USB_DSCR_OFFSET_TYPE) != ((uint8_t)CY_USB_ENDP_DSCR))) {
        return(FALSE);
    }
    return(TRUE);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpNumMaxPktDir
****************************************************************************//**
*
*  This function fetches endpoint number, maxPktSize and direction from endpoint
*  descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pEndpNum
* endpoint number will be stored here.
*
* \param pMaxPktSize
* max packet size of endpoint will be stored here.
*
* \param pDir
* Direction of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_PTR_NULL if any of the parameter is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpNumMaxPktDir (uint8_t *pEndpDscr, uint32_t *pEndpNum,
                             uint16_t *pMaxPktSize, uint32_t *pDir)
{
    if (NULL == pEndpDscr) {
        DBG_USBD_ERR("GetEndpNumMaxPktDir:pEndpDscr Null\r\n");
        *pMaxPktSize = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    if ((pEndpNum == NULL) || (pMaxPktSize == NULL) || (pMaxPktSize == NULL)) {
        DBG_USBD_ERR("GetEndpNumMaxPktDir: Null\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    *pEndpNum = ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

    *pMaxPktSize = (*(uint8_t *)(pEndpDscr +CY_USB_ENDP_DSCR_OFFSET_MAX_PKT)) +
        ((*(uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) << 8);
    *pMaxPktSize &= CY_USB_ENDP_MAX_PKT_SIZE_MASK;

    *pDir = (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80);

    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpMaxPktSize
****************************************************************************//**
*
*  This function fetches maxPktSize from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pMaxPktSize
* max packet size of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpMaxPktSize (uint8_t *pEndpDscr, uint16_t *pMaxPktSize)
{

    if (NULL == pEndpDscr) {
        DBG_USBD_ERR("GetEndpMaxPktSize:pEndpDscr Null\r\n");
        *pMaxPktSize = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    *pMaxPktSize = (*(uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT)) +
        ((*(uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) << 8);
    *pMaxPktSize &= CY_USB_ENDP_MAX_PKT_SIZE_MASK;

    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_GetIsoBytesPerIntvl
****************************************************************************//**
*
*  This function fetches max bytes per interval for Iso endpoint from
*  iso companion endpoint descriptor.
*
* \param pIsoCompDscr
* pointer to Iso endpoint companion descriptor.
*
* \param pMaxPktSize
* max packet size of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if ompanion descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetIsoBytesPerIntvl (uint8_t *pIsoCompDscr, uint32_t *pBytesPerIntvl)
{

    if (NULL == pIsoCompDscr) {
        DBG_USBD_ERR("GetIsoBytesPerIntvl:pIsoCompDscr Null\r\n");
        *pBytesPerIntvl = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    *pBytesPerIntvl = (*(uint32_t *)(pIsoCompDscr + CY_USB_ENDP_DSCR_OFFSET_BYTES_PER_INTVL));
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpAttribute
****************************************************************************//**
*
*  This function fetches endpoint attribute from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pAttribute
* atrribute of endpoint will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpAttribute (uint8_t *pEndpDscr, uint8_t *pAttribute)
{

    if (NULL == pEndpDscr) {
        DBG_USBD_ERR("GetEndpAttribute:pEndpDscr Null\r\n");
        *pAttribute = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    *pAttribute = *(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpType
****************************************************************************//**
*
*  This function fetches endpoint type from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pEndpType
*  endpoint type is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to endpoint type is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpType (uint8_t *pEndpDscr, uint32_t *pEndpType)
{

    if (NULL == pEndpDscr) {
        DBG_USBD_ERR("GetEndpType:pEndpDscr Null\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    if (NULL == pEndpType) {
        DBG_USBD_ERR("GetEndpType:pEndpType Null\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    *pEndpType =
    (uint32_t)((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE)) & 0x03);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpInterval
****************************************************************************//**
*
*  This function fetches bInterval from endpoint descriptor.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \param pEndpType
*  endpoint type is stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pointer to interval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpInterval (uint8_t *pEndpDscr, uint8_t *pInterval)
{

    if (NULL == pEndpDscr) {
        DBG_USBD_ERR("GetEndpInterval:pEndpDscr Null\r\n");
        *pInterval = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    if (NULL == pInterval) {
        DBG_USBD_ERR("GetEndpInterval:pInterval Null\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    *pInterval = *(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_INTERVAL);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnMaxburst
****************************************************************************//**
*
*  This function fetches bMaxBurst from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pMaxBust
*  max burst information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pMaxBurst is NULL.
*
*******************************************************************************/
/* Note: Caller should call this function for non control endpoint. */
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpCompnMaxburst (uint8_t *pEndpCompnDscr, uint8_t *pMaxBust)
{
    if (NULL == pEndpCompnDscr) {
        DBG_USBD_ERR("GetEndpCompMaxBurst:pEndpCompnDscr NULL\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    if (NULL == pMaxBust) {
        DBG_USBD_ERR("GetEndpCompMaxBurst:pMaxBurst NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    /* As per spec, MAX_BURST 0x00 means 1 Pkt at time so return MAX_BURST+1 */
    *pMaxBust = (*(pEndpCompnDscr
                        + CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMAX_BURST) + 1);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnMaxStream
****************************************************************************//**
*
*  This function fetches bMaxStream from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pMaxStream
*  max stream information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pMaxStream is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpCompnMaxStream (uint8_t *pEndpCompnDscr, uint8_t *pMaxStream)
{
    if (NULL == pEndpCompnDscr) {
        DBG_USBD_ERR("GetEndpCompnMaxStream:pEndpCompnDscr NULL\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    if (NULL == pMaxStream) {
        DBG_USBD_ERR("GetEndpCompnMaxStream:pMaxStream NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }

    *pMaxStream =
    (*(pEndpCompnDscr + CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMATTRIBUTE));
    /* first 5 bit belongs to maxStream. */
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnAttribute
****************************************************************************//**
*
*  This function fetches attribute from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pAttribute
*  attribute information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pAttribute is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpCompnAttribute (uint8_t *pEndpCompnDscr, uint8_t *pAttribute)
{
    if (NULL == pEndpCompnDscr) {
        DBG_USBD_ERR("GetEndpCompnAttribute:pEndpCompnDscr NULL\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    if (NULL == pAttribute) {
        DBG_USBD_ERR("GetEndpCompnAttribute:pAttribute NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    *pAttribute = *(pEndpCompnDscr +
                            CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMATTRIBUTE);
    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpCompnBytePerInterval
****************************************************************************//**
*
*  This function fetches bytePerInterval from endpoint companion descriptor.
*
* \param pEndpCompnDscr
* pointer to endpoint companion descriptor.
*
* \param pBytePerInterval
*  Byte per interval information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBytePerInterval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpCompnBytePerInterval (uint8_t *pEndpCompnDscr,
                                     uint16_t *pBytePerInterval)
{
    if (NULL == pEndpCompnDscr) {
        DBG_USBD_ERR("GetEndpCompnBytePerInterval:pEndpCompnDscr NULL\r\n");
        *pBytePerInterval = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    if (NULL == pBytePerInterval) {
        DBG_USBD_ERR("GetEndpCompnBytePerInterval:pBytePerInterval NULL\r\n");
        *pBytePerInterval = 0x00;
        return (CY_USBD_STATUS_PTR_NULL);
    }
    *pBytePerInterval =
     (*(uint8_t *)(pEndpCompnDscr + CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL)) +
        ((*(uint8_t *)(pEndpCompnDscr + CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL + 1)) << 8);

    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_GetEndpSspIsoCompnBytePerInterval
****************************************************************************//**
*
*  This function fetches bytePerInterval from endpoint SS Iso companion descriptor.
*
* \param pSspIsoEndpCompnDscr
* pointer to super speed Iso endpoint companion descriptor.
*
* \param pBytePerInterval
*  Bytes per interval information will be stored here.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_BAD_PARAM if endpoint companion descriptor is NULL.
* CY_USBD_STATUS_PTR_NULL if pBytePerInterval is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpSspIsoCompnBytePerInterval (uint8_t *pSspIsoEndpCompnDscr,
                                           uint32_t *pBytePerInterval)
{
    if (NULL == pSspIsoEndpCompnDscr) {
        DBG_USBD_ERR("GetEndpSspIsoCompnBytePerInterval:pSspIsoEndpCompnDscr NULL\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    if (NULL == pBytePerInterval) {
        DBG_USBD_ERR("GetEndpSspIsoCompnBytePerInterval:pBytePerInterval NULL\r\n");
        return (CY_USBD_STATUS_PTR_NULL);
    }
    *pBytePerInterval =
     (*(uint8_t *)(pSspIsoEndpCompnDscr + CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL)) +
     ((*(uint8_t *)(pSspIsoEndpCompnDscr + CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL + 1)) << 8) +
     ((*(uint8_t *)(pSspIsoEndpCompnDscr + CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL + 2)) << 16) +
     ((*(uint8_t *)(pSspIsoEndpCompnDscr + CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL + 3)) << 24);

    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */



/*******************************************************************************
* Function name: Cy_USBD_EnableEndp
****************************************************************************//**
*
*  This function enable/disable endpoint. Based on speed, this function will
*  call appropriate HW function to enable/disable endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param enable
* true for enabling endpoint and false for disabling endpoint.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EnableEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                    cy_en_usb_endp_dir_t endpDir,  bool enable)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EnableEndp:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetCode = Cy_USBHS_Cal_EnableEndp(pUsbdCtxt->pCalCtxt, endpNum,
                                             endpDir, enable);
    } else {
        calRetCode = Cy_USBSS_Cal_EnableEndp(pUsbdCtxt->pSsCalCtxt, endpNum,
                                             endpDir, enable);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_SetEpBurstMode
****************************************************************************//**
*
*  This function enables/disables the MULT (allow burst across multiple DMA buffers)
*  feature for the specified USB endpoints. Enabling the feature can improve the data
*  transfer throughput on the respective endpoints. The settings need to be updated
*  in the USB IP as well as at the DMA socket level.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param enable
* true for enabling burst mode and false for disabling burst mode.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_SetEpBurstMode (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir,  bool enable)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_FAILURE;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SetEpBurstMode:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /* Operation is only relevant under USB3.2 */
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        calRetCode = Cy_USBSS_Cal_SetEpBurstMode(pUsbdCtxt->pSsCalCtxt, endpNum, endpDir, enable);

        /* If the CAL level update was successful, make sure the socket configuration is updated. */
        if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
            Cy_HBDma_Mgr_UpdateMultEn(pUsbdCtxt->pHBDmaMgr, endpNum,
                    (endpDir == CY_USB_ENDP_DIR_IN), enable);
            retCode = CY_USBD_STATUS_SUCCESS;
        }
    }

    return(retCode);
}   /* end of function */


/*******************************************************************************
* Function name: GetRetryBufferRegion
****************************************************************************//**
*
* This function allocates the required amount of space in the USB32DEV
* IP retry buffer for a CTRL/BULK endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param maxBurst
* Burst size of the endpoint.
*
* \return
* retry buffer offset used for endpoint.
*
*******************************************************************************/
static uint16_t
GetRetryBufferRegion (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t maxBurst)
{
    uint32_t index, byteindex, bitindex;
    uint32_t start = 0;
    uint32_t found = 0;
    uint32_t intrState;
    uint16_t retryBufOffset = 0xFFFFU;

    if ((pUsbdCtxt == NULL) || (maxBurst > 16UL)) {
        return retryBufOffset;
    }

    /* Work-around for ease of use. A caller may pass burst of 0 instead of 1. */
    if (maxBurst == 0) {
        maxBurst = 1UL;
    }

    intrState = Cy_SysLib_EnterCriticalSection();

    /* Check whether we have the required amount of contiguous retry buffer space available. */
    for (index = 0; index < 64U; index++) {
        byteindex = (index >> 3U);
        bitindex  = (index & 0x07U);

        if ((pUsbdCtxt->ssRetryBufStatus[byteindex] & (1U << bitindex)) == 0) {
            if (found == 0) {
                start = index;
            }

            found++;
            if (found >= maxBurst) {
                retryBufOffset = start;
                break;
            }
        } else {
            found = 0;
        }
    }

    /* Mark the allocated region of the retry buffer in use. */
    if (retryBufOffset < 64U) {
        for (index = start; index < (start + maxBurst); index++) {
            byteindex = (index >> 3U);
            bitindex  = (index & 0x07U);
            pUsbdCtxt->ssRetryBufStatus[byteindex] |= (1U << bitindex);
        }
    }

    Cy_SysLib_ExitCriticalSection(intrState);

    /* Offset returned should be in 4-byte units. */
    return (retryBufOffset << 8U);
}

/*******************************************************************************
* Function name: FreeRetryBufferRegion
****************************************************************************//**
*
* This function frees up the previously allocated retry buffer region for a
* CTRL/BULK interrupt.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param startAddr
* start address of retry region allocated for the endpoint.
*
* \param maxBurst
* Burst size of the endpoint.
*
* \return
* None.
*
*******************************************************************************/
static void
FreeRetryBufferRegion (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       uint16_t startAddr, uint32_t maxBurst)
{
    uint32_t index, byteindex, bitindex;
    uint32_t intrState;

    if ((pUsbdCtxt == NULL) || (startAddr >= 16384U)) {
        DBG_USBD_WARN("FreeRetryBufferRegion:InvalidArgs\r\n");
        return;
    }

    if (maxBurst == 0) {
        maxBurst = 1UL;
    }

    /* Reduce start address to 1K units. */
    startAddr >>= 8U;

    intrState = Cy_SysLib_EnterCriticalSection();

    /* Loop through and mark the corresponding region of retry buffer unused. */
    for (index = startAddr; index < (startAddr + maxBurst); index++) {
        byteindex = index >> 3U;
        bitindex  = index & 0x07U;
        pUsbdCtxt->ssRetryBufStatus[byteindex] &= ~(1U << bitindex);
    }

    Cy_SysLib_ExitCriticalSection(intrState);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_USBD_EndpConfig
****************************************************************************//**
*
*  This function configures endpoint based on user requirement.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpConfig
* configuration parameters passed by user.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM if any of the configuration parameter invalid.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpConfig (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                        cy_stc_usb_endp_config_t endpConfig)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDir;
    cy_stc_usb_endp_info_t *pEndpInfo;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndCon:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /*
     * If endpoint number > 15 then error.
     * Endpoint 0 should be configured before reset Done.
     * Other endpoint should be configured only after device in
     * configured state.
     */
    if ((endpConfig.endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        ((endpConfig.endpNumber == CY_USB_ENDP_0) &&
        (pUsbdCtxt->devState >= CY_USB_DEVICE_STATE_DEFAULT)) ||
        ((endpConfig.endpNumber > CY_USB_ENDP_0) &&
        (pUsbdCtxt->devState < CY_USB_DEVICE_STATE_CONFIGURED))) {
        return(CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM);
    }

    endpNumber = endpConfig.endpNumber;
    endpDir = endpConfig.endpDirection;

    if (endpDir == CY_USB_ENDP_DIR_OUT) {
        pEndpInfo = &(pUsbdCtxt->endpInfoOut[endpNumber]);
    } else {
        pEndpInfo = &(pUsbdCtxt->endpInfoIn[endpNumber]);

        /*
         * In USB3.x case, if we have previously allocated retry
         * buffer region for CTRL/BULK egress endpoints; it needs to
         * be freed up first.
         */
        if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
            if (pEndpInfo->retryBufOffset != 0xFFFFU) {
                Cy_USBSS_Cal_SetEndpRetryOffset(pUsbdCtxt->pSsCalCtxt,
                        endpNumber, 0);
                FreeRetryBufferRegion(pUsbdCtxt, pEndpInfo->retryBufOffset,
                        pEndpInfo->burstSize);
                pEndpInfo->retryBufOffset = 0xFFFFU;
            }
        }
    }

    pEndpInfo->maxPktSize = endpConfig.maxPktSize;
    pEndpInfo->endpType = endpConfig.endpType;
    pEndpInfo->valid = endpConfig.valid;
    pEndpInfo->burstSize = endpConfig.burstSize;
    pEndpInfo->streamID = endpConfig.streamID;
    pEndpInfo->valid = endpConfig.valid;
    pEndpInfo->allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;

    /* for endpoint 0  IN/OUT taken care in single call. */
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            /*
             * if allowNakTillDmaRdy set then explicitly set NAK bit for that
             * endpoint. With this fist packet will be responded with NAK.
             */
            if (endpConfig.allowNakTillDmaRdy) {
                Cy_USBHS_Cal_EndpSetClearNak(pUsbdCtxt->pCalCtxt,endpNumber,
                                             CY_USB_ENDP_DIR_OUT, CY_USB_SET);
            }
        }
        calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
    } else {
        calRetCode = Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);

        /*
         * When any CTRL/BULK IN endpoint is enabled on a USB-SS connection,
         * we need to allocate the retry buffer region for that endpoint
         * based on its maximum burst length setting.
         */
        if (
                (endpConfig.valid) &&
                (calRetCode == CY_USB_CAL_STATUS_SUCCESS) &&
                (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) &&
                (endpDir == CY_USB_ENDP_DIR_IN) &&
                (
                    (endpConfig.endpType == CY_USB_ENDP_TYPE_CTRL) ||
                    (endpConfig.endpType == CY_USB_ENDP_TYPE_BULK)
                )
           ) {
            pEndpInfo->retryBufOffset = GetRetryBufferRegion(pUsbdCtxt,
                    pEndpInfo->burstSize);
            if (pEndpInfo->retryBufOffset >= 16384U) {
#if 0
                /*
                 * Failed to allocate retry buffer.
                 * Disable the EP and return error.
                 */
                endpConfig.valid = false;
                Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
                pEndpInfo->valid = false;
                calRetCode = CY_USB_CAL_STATUS_FAILURE;
#else
                /* TODO: For now, don't treat failure to allocate retry buffer as an error. */
                Cy_USBSS_Cal_SetEndpRetryOffset(pUsbdCtxt->pSsCalCtxt,
                        endpNumber, 0);
#endif
            } else {
                /* Update the retry buffer offset in the register. */
                Cy_USBSS_Cal_SetEndpRetryOffset(pUsbdCtxt->pSsCalCtxt,
                        endpNumber, pEndpInfo->retryBufOffset);
            }
        }
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearStall
****************************************************************************//**
*
*  This function enable or disable STALL condition for the endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param setClear
* true for set and false for clear stall condition.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
/*
 * It is caller's responsibility to call this function for various endpoints
 * based on required device state.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearStall (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               uint32_t endpNum,
                               cy_en_usb_endp_dir_t endpDir,
                               bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndSetCleSta:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
            (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetStatus = Cy_USBHS_Cal_EndpSetClearStall(pUsbdCtxt->pCalCtxt,
                                                      endpNum, endpDir,
                                                      setClear);
    } else {
        calRetStatus = Cy_USBSS_Cal_EndpSetClearStall(pUsbdCtxt->pSsCalCtxt,
                                                      endpNum, endpDir,
                                                      setClear);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        /* for non endpoint 0, update halt Variable */
        if (endpNum != CY_USB_ENDP_0) {
            if (setClear) {
                if (CY_USB_ENDP_DIR_IN == endpDir) {
                    pUsbdCtxt->endpInfoIn[endpNum].halt = TRUE;
                } else {
                    pUsbdCtxt->endpInfoOut[endpNum].halt = TRUE;
                }
            } else {
                if (CY_USB_ENDP_DIR_IN == endpDir) {
                    pUsbdCtxt->endpInfoIn[endpNum].halt = FALSE;
                } else {
                    pUsbdCtxt->endpInfoOut[endpNum].halt = FALSE;
                }
            }
        } else {
            pUsbdCtxt->setupReqActive = false;
        }
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearNakNrdy
****************************************************************************//**
*
*  This function enable or disable NAK/NRDY condition for the endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param setClear
* true for set and false for clear NAK condition.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearNakNrdy (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                                bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndSetCleNakNrd:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((endpNum > 0x00) &&
        (pUsbdCtxt->devState < CY_USB_DEVICE_STATE_CONFIGURED)) {
        return(CY_USBD_STATUS_FAILURE);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetStatus = Cy_USBHS_Cal_EndpSetClearNak(pUsbdCtxt->pCalCtxt,
                                                    endpNum, endpDir,
                                                    setClear);
    } else {
        calRetStatus = Cy_USBSS_Cal_EndpSetClearNrdy(pUsbdCtxt->pSsCalCtxt,
                                                     endpNum, endpDir,
                                                     setClear);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* end of function. */


/*******************************************************************************
* Function name: Cy_USBD_ResetEndp
****************************************************************************//**
*
*  This function resets endpoint related paramters by calling appropriate function
*  at HW level. This clears sticky bits e.g retry bit, flowcontrol bit.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param preserveSeqNo
* true to preserve sequence number and false to reset sequence number.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_ResetEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                   cy_en_usb_endp_dir_t endpDir, bool preserveSeqNo)
{

    uint8_t epSeqNo = 0;
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        /* HS does not have endpoint reset implementation. */

    } else {
        /* SS implementation of endp reset */
        if (endpNum ==  0x00) {
            /* Sequence number retention is not supported on EP0. */
            Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, endpNum, CY_USB_ENDP_DIR_IN);
        } else {
            if (preserveSeqNo) {
                Cy_USBSS_Cal_GetSeqNum(pUsbdCtxt->pSsCalCtxt, endpNum,
                                       endpDir, &epSeqNo);
            }

            Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, endpNum, endpDir);
            if (preserveSeqNo) {
                Cy_USBSS_Cal_SetSeqNum(pUsbdCtxt->pSsCalCtxt, endpNum,
                                        endpDir, epSeqNo);
            }
        }
    }
}   /* end of function() */


/*******************************************************************************
* Function name: Cy_USBD_FlushEndp
****************************************************************************//**
*
*  This function flush endpoint data present in EPM.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \return
* None
*
*******************************************************************************/
void
Cy_USBD_FlushEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                        uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("FluResEnd:usbdCtxt Null\r\n");
        return;
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        if (endpNum ==  0x00) {
            /*
             * HS has different implementation of EPM so need to call function
             * two times with different direction for endp 0.
             */
            Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNum,
                                   CY_USB_ENDP_DIR_IN);
            Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNum,
                                   CY_USB_ENDP_DIR_OUT);
        } else {
            Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNum,
                                   endpDir);
        }
    } else {
        /*
         * Stand-alone flush function is supported only for EP0. For other
         * endpoints Cy_USBD_ResetEndp has to be called.
         */
        if (endpNum ==  0x00) {
            /*
             * For endpoint 0x00, No need to call function with both direction.
             * CAL layer will take care updating required registers in both
             * direction.
             */
            Cy_USBSS_Cal_FlushEndpSocket(pUsbdCtxt->pSsCalCtxt, endpNum, CY_USB_ENDP_DIR_IN);
        }
    }
    return;
}   /* end of function. */


/*******************************************************************************
* Function name: Cy_USBD_FlushEndpAll
****************************************************************************//**
*
*  This function flush all endpoints data endpoint data present in EPM.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpDir
* Parameter is unused. Both IN and OUT endpoints will be flushed.
*******************************************************************************/
void
Cy_USBD_FlushEndpAll (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                      cy_en_usb_endp_dir_t endpDir)
{
    /*
     * There is implementation difference between HS and SS. Please check
     * respective CAL layer function.
     */
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("FluResEndAll:usbdCtxt Null\r\n");
        return;
    }

    /* For FlushAll, direction is ignored while calling CAL layer function. */
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt);
    } else {
        Cy_USBSS_Cal_FlushAllEndpSocket(pUsbdCtxt->pSsCalCtxt);
        Cy_USBSS_Cal_FlushEPM(pUsbdCtxt->pSsCalCtxt, true);
    }
    return;
}   /* end of function. */


/*******************************************************************************
* Function name: Cy_USBD_FlushResetEndpAll
****************************************************************************//**
*
*  This function flush/reset all endpoint with perticular direction based on
*  speed of device controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpDir
* endpoint direction.
*
* \return
* None
*
*******************************************************************************/
void
Cy_USBD_FlushResetEndpAll (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_en_usb_endp_dir_t endpDir)
{
    uint32_t index;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("FlushResetAllEP:usbdCtxt Null\r\n");
        return;
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt);
    } else {
        for (index = 0x00; index < CY_USB_MAX_ENDP_NUMBER; index++) {
            Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, index, endpDir);
        }
    }
    return;
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearNakNrdyAll
****************************************************************************//**
*
*  This function enable or disable NAK/NRDY condition on all endpoints based on
*  speed of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* set ie true to enable NAK/NRDY and clear ie false to disable NAK/NRDY.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearNakNrdyAll (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndSetCleNakNrdAll:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        calRetStatus = Cy_USBHS_Cal_SetClearNakAll(pUsbdCtxt->pCalCtxt,
                                                   setClear);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBD_EndpIsNakNrdySet
****************************************************************************//**
*
*  This function checks whether the specified endpoint currently has the NAK/NRDY bit set.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \return
* TRUE if NAK/NRDY bit is set
* FALSE if NAK/NRDY bit is reset
*
*******************************************************************************/
bool
Cy_USBD_EndpIsNakNrdySet (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                          cy_en_usb_endp_dir_t endpDir)
{
    bool epNaked = false;

    if ((pUsbdCtxt != NULL) && (endpNum < CY_USB_MAX_ENDP_NUMBER)) {
        if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            epNaked = Cy_USBHS_Cal_EndpIsNakNrdySet(pUsbdCtxt->pCalCtxt,
                                                    endpNum, endpDir);
        } else {
            epNaked = Cy_USBSS_Cal_EndpIsNakNrdySet(pUsbdCtxt->pSsCalCtxt,
                                                    endpNum, endpDir);
        }
    }
    return epNaked;
}


/*******************************************************************************
* Function name: Cy_USBD_EndpIsStallSet
****************************************************************************//**
*
*  This function checks whether the specified endpoint currently has the STALL bit set.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \return
* TRUE if STALL bit is set
* FALSE if STALL bit is reset
*
*******************************************************************************/
bool
Cy_USBD_EndpIsStallSet (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir)
{
    bool epStalled = false;

    if ((pUsbdCtxt != NULL) && (endpNum < CY_USB_MAX_ENDP_NUMBER)) {
        if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            epStalled = Cy_USBHS_Cal_EndpIsStallSet(pUsbdCtxt->pCalCtxt,
                                                    endpNum, endpDir);
        } else {
            epStalled = Cy_USBSS_Cal_EndpIsStallSet(pUsbdCtxt->pSsCalCtxt,
                                                    endpNum, endpDir);
        }
    }
    return epStalled;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_EndpMapStream
****************************************************************************//**
*
* This function maps an unused USB Ingress/Egress socket to the specified stream
* associated with a bulk endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param streamId
* stream number.
*
* \param socketNum
* socket number to be mapped.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EndpMapStream (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                       cy_en_usb_endp_dir_t endpDir, uint16_t streamId,
                       uint32_t socketNum)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndpMapStream:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        /* Operation is invalid under USB 2.x. */
        DBG_USBD_ERR("EndpMapStream:USB2Mode\r\n");
        return (CY_USBD_STATUS_FAILURE);
    } else {
        calRetStatus = Cy_USBSS_Cal_EndpMapStream(pUsbdCtxt->pSsCalCtxt,
                                                  endpNum, endpDir,
                                                  streamId, socketNum);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_BAD_PARAM);
    }
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_EndpUnmapStream
****************************************************************************//**
*
* This function unmaps the USB ingress/egress socket to bulk stream mapping
* for a bulk endpoint and socket.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param socketNum
* socket number to be mapped.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EndpUnmapStream (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                       cy_en_usb_endp_dir_t endpDir, uint32_t socketNum)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndpUnmapStream:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        /* Operation is invalid under USB 2.x. */
        DBG_USBD_ERR("EndpUnmapStream:USB2Mode\r\n");
        return (CY_USBD_STATUS_FAILURE);
    } else {
        calRetStatus = Cy_USBSS_Cal_EndpUnmapStream(pUsbdCtxt->pSsCalCtxt,
                                                  endpNum, endpDir,
                                                  socketNum);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_BAD_PARAM);
    }
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_EndpSetPktsPerBuffer
****************************************************************************//**
*
*  This function is used to update the USB block with the number of max. sized
* packets received on a SuperSpeed OUT endpoint that can fit into one DMA buffer.
* Setting this value is required for proper operation in cases where the maximum
* packet size of the endpoint is not a power of 2.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param pktsPerBuffer
* Number of packets that can fit in one DMA buffer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if HW call fails due to any parameter.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EndpSetPktsPerBuffer (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              uint32_t endpNum, uint8_t pktsPerBuffer)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndpSetPktsPerBuffer:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        /* Operation is invalid under USB 2.x. */
        DBG_USBD_ERR("EndpSetPktsPerBuf:USB2Mode\r\n");
        return (CY_USBD_STATUS_FAILURE);
    } else {
        calRetStatus = Cy_USBSS_Cal_EndpSetPktsPerBuffer(pUsbdCtxt->pSsCalCtxt,
                                                         endpNum,
                                                         pktsPerBuffer);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_BAD_PARAM);
    }
}   /* end of function */

/*******************************************************************************
* Function name: CopyToEndp0EgressFifo
****************************************************************************//**
*
*  Function to copy data from pBuffer into the Egress SRAM for EP0.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* None
*
*******************************************************************************/
void
CopyToEndp0EgressFifo (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pBuffer,
                       uint32_t bufferSize)
{
    uint32_t words, bytes;
    uint32_t i;
    volatile uint32_t *dst32_p, *src32_p;
    volatile uint8_t *dst8_p, *src8_p;

    (void) pUsbdCtxt;

    words = (bufferSize / 4U);
    bytes = (bufferSize & 3U);

    dst32_p = (volatile uint32_t *)0x30000000UL;
    src32_p = (uint32_t *)pBuffer;

    for (i = 0; i < words; i++)
    {
        dst32_p[i] = src32_p[i];
    }

    dst8_p = (volatile uint8_t *)(0x30000000UL + words * 4U);
    src8_p = pBuffer + words * 4U;

    for (i = 0; i < (bytes); i++)
    {
        dst8_p[i] = src8_p[i];
    }
}

/*******************************************************************************
* Function name: Cy_USBD_UpdateXferCount
****************************************************************************//**
*
* This function will update xfer count based on speed of device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param bufferSize
* buffer size.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_UpdateXferCount (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                         cy_en_usb_endp_dir_t endpDir, uint32_t bufferSize)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("UpdXfrCnt:usbdCtxt Null\r\n");
        return;
    }
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        Cy_USBHS_Cal_UpdateXferCount(pUsbdCtxt->pCalCtxt, endpNum,
                                               endpDir, bufferSize);
    } else {
        Cy_USBSS_Cal_UpdateXferCount(pUsbdCtxt->pSsCalCtxt, endpNum,
                                               endpDir, bufferSize);
    }
    return;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_CtrlEndp0DataOutAck
****************************************************************************//**
*
* Data stage ACK for endpoint0 is controlled by this function. In USBHS,
* when "CONT_TO_DATA" bit is set then controller won't send ACK till data
* is validated.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* true for set and false for clear.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_CtrlEndp0DataOutAck (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool setClear)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("CtrlEndp0DataOutAck:usbdCtxt Null\r\n");
        return;
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        if (setClear) {
            Cy_USBHS_Cal_CtrlEndp0DataOutAck(pUsbdCtxt->pCalCtxt, true);
        } else {
            Cy_USBHS_Cal_CtrlEndp0DataOutAck(pUsbdCtxt->pCalCtxt, false);
        }
    } else {
        /* TBD: For CAL layer SS function need to be coded. */
    }

    return;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_EnableStatusCtrl
****************************************************************************//**
*
* This function enable/disable status control feature which is part of SS controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param enable
* true for enable and false for disable.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_EnableStatusCtrl (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool enable)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SndAckSet:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)) {
        Cy_USBSS_Cal_EnableStatusCtrl(pUsbdCtxt->pSsCalCtxt,enable);
        return(CY_USBD_STATUS_SUCCESS);
    }
    return(CY_USBD_STATUS_FAILURE);
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBD_ClearStatusClrBusy
****************************************************************************//**
*
* This function write "1" to STATUS_CLR_BUSY which clears the bit to initiate
* STATUS response.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_ClearStatusClrBusy (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SndAckSet:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)) {
        Cy_USBSS_Cal_ClearStatusClrBusy(pUsbdCtxt->pSsCalCtxt);
        return(CY_USBD_STATUS_SUCCESS);
    }
    return(CY_USBD_STATUS_FAILURE);
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBD_SendAckSetupDataStatusStage
****************************************************************************//**
*
* Allow chipset to send ACK to complete control transfer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_SendAckSetupDataStatusStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SndAckSet:usbdCtxt Null\r\n");
        return;
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        if (!pUsbdCtxt->lpmDisabled) {
            /*
             * We need to make sure that the Link is in L0 and does not
             * go back into L1 until the request handling is complete.
             */
            Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, CY_USB_SET);

            /* Call USBD LinkActive func so that dev states are taken care */
            Cy_USBD_GetUSBLinkActive(pUsbdCtxt);
        }

        Cy_USBHS_Cal_SendAckSetupDataStatusStage(pUsbdCtxt->pCalCtxt);

    } else {
        if (Cy_USBSS_Cal_SendAckSetupDataStatusStage(pUsbdCtxt->pSsCalCtxt) != CY_USB_CAL_STATUS_SUCCESS) {
            DBG_USBD_ERR("AckSetupFailed\r\n");
        }
    }
    return;
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0Data
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0. Based on speed of device,
*  SS or HS function will be invoked.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_SendEndp0Data (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;

    /*
     * There is possibility of optimize this code. it will be done
     * after all usecases.
     */
    Cy_USBD_UpdateXferCount(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, bufferSize);
    if(pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        retStatus = Cy_USB_USBD_SendEndp0DataSs(pUsbdCtxt, pBuffer,
                                                bufferSize);
    } else {
        retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, pBuffer,
                                                bufferSize);
    }
    return retStatus;
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0DataSs
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0 of SS device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_SendEndp0DataSs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;
    cy_en_hbdma_mgr_status_t dmaStatus;
    uint32_t *pSrc, *pDst;
    uint32_t idx;
    bool zlpReqd = false;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SndEnd0DataSs:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
        DBG_USBD_ERR("SndEnd0DataSs:bufferOrSize Null\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    /* Make sure we are in U0 state before proceeding. */
    Cy_USBSS_Cal_GetUsbLinkActive(pUsbdCtxt->pSsCalCtxt);

    if (bufferSize != 0) {
        /* If the data to be sent is not present in HBW SRAM, make a copy and send from there. */
        if (((uint32_t)pBuffer < CY_HBW_SRAM_BASE_ADDR) || ((uint32_t)pBuffer >= CY_HBW_SRAM_LAST_ADDR)) {
            if (((uint32_t)pBuffer & 0x03) != 0) {
                DBG_USBD_ERR("SndEnd0DataSs:unaligned buffer\r\n");
                return (CY_USBD_STATUS_BAD_PARAM);
            }

            pSrc = (uint32_t *)pBuffer;
            pDst = pUsbdCtxt->inEp0ScratchBuffer;
            for (idx = 0; idx < ((bufferSize + 3) >> 2); idx++)
            {
                pDst[idx] = pSrc[idx];
            }
        } else {
            pDst = (uint32_t *)pBuffer;
        }

        if ((pUsbdCtxt->setupReq.wLength > bufferSize) && ((bufferSize & 0x1FFU) == 0)) {
            zlpReqd = true;
        }

        /* Send the data to EP0 using HB DMA */
        dmaStatus = Cy_HBDma_Channel_SendData(&(pUsbdCtxt->inEp0DmaUsb3Ch),
                                              0,
                                              (uint8_t *)pDst,
                                              bufferSize);
        if(dmaStatus == CY_HBDMA_MGR_SUCCESS) {

            /* Clear the busy bit to complete the transfer */
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

            dmaStatus = Cy_HBDma_Channel_WaitForSendCplt(&(pUsbdCtxt->inEp0DmaUsb3Ch), 0);
            if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
                /* Complete the transfer with a ZLP where needed. */
                if (zlpReqd) {
                    dmaStatus = Cy_HBDma_Channel_SendData(&(pUsbdCtxt->inEp0DmaUsb3Ch),
                                                          0,
                                                          (uint8_t *)pDst,
                                                          0);
                    if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
                        dmaStatus = Cy_HBDma_Channel_WaitForSendCplt(&(pUsbdCtxt->inEp0DmaUsb3Ch), 0);
                    }
                }
            }

            if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
                pUsbdCtxt->ep0SendDone = true;
                DBG_USBD_TRACE("EP0Send Done\r\n");
                retStatus = CY_USBD_STATUS_SUCCESS;
            } else {
                DBG_USBD_ERR("HBDMA error: 0x%x\r\n", dmaStatus);
            }
        }
        else
        {
            DBG_USBD_ERR("HBDMA error: 0x%x\r\n", dmaStatus);
        }
    } else {
        /* Send data with length of 0. */
        DBG_USBD_INFO("SendData: wlen=0\r\n");
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        retStatus = CY_USBD_STATUS_SUCCESS;
    }

    return retStatus;
}

/*******************************************************************************
* Function name: Cy_USB_USBD_SendEndp0DataHs
****************************************************************************//**
*
*  This function sends data overthe DMA for endpoint 0 of HS device.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_SendEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_stc_usbd_dma_descr_t *pDmaCh0InXferDscr;
    cy_stc_usbd_dma_descr_conf_t dscr_config;
    cy_en_usbd_ret_code_t retval = CY_USBD_STATUS_SUCCESS;

    uint32_t dmaIntr;
    uint32_t loopCnt = 0;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SndEnd0DataHs:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
        DBG_USBD_ERR("SndEnd0DataHs:bufferOrSize Null\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    /*
     * If the amount of data to send is more than 64 bytes (max. pkt. size for
     * EP0), we need to use two DMA descriptors. The first one will transfer
     * all the full data packets, waiting for trigger from EPM between packets.
     * The second one will transfer the remaining portion at the end.
     */
    if (bufferSize >= 64U) {
        /* Need to set src, dst addr and update size of transfer ie xCount. */
        pDmaCh0InXferDscr = &(pUsbdCtxt->dmaCh0InXferDscr0);

        Cy_USBD_DMADesc_SetSrcAddress(pDmaCh0InXferDscr, pBuffer);
        Cy_USBD_DMADesc_SetDstAddress(pDmaCh0InXferDscr,
                                      (uint8_t *)0x30000000UL);
        Cy_USBD_DMADesc_SetXloopDataCount(pDmaCh0InXferDscr, 0x10UL);
        Cy_USBD_DMADesc_SetYloopDataCount(pDmaCh0InXferDscr,
                                          (bufferSize >> 6U));
        Cy_USBD_DMADesc_SetYloopSrcIncrement(pDmaCh0InXferDscr, 16);
        Cy_USBD_DMADesc_SetYloopDstIncrement(pDmaCh0InXferDscr, 0);

        if ((bufferSize & 0x3FUL) == 0) {
            /* Transfer size is a multiple of max. pkt. size. So, we can use only the 2-D DMA descriptor. */
            Cy_USBD_DMADesc_SetNextDescriptor(pDmaCh0InXferDscr, NULL);
            Cy_USBD_DMADesc_SetChannelState(pDmaCh0InXferDscr, CY_USBD_DMA_CHN_DISABLED);
        } else {
            /* Point to the next descriptor. */
            Cy_USBD_DMADesc_SetNextDescriptor(pDmaCh0InXferDscr, &(pUsbdCtxt->dmaCh0InXferDscr1));
            Cy_USBD_DMADesc_SetChannelState(pDmaCh0InXferDscr, CY_USBD_DMA_CHN_ENABLED);
        }
    } else {
        /* DMA will start with descriptor 1 in this case. */
        pDmaCh0InXferDscr = &(pUsbdCtxt->dmaCh0InXferDscr1);
    }

    if ((bufferSize & 0x3FUL) >= 0x04UL) {
        /* Descriptor to copy the word portions of the short packet at the end of transfer. */
        dscr_config.retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.dataPrefetch = false;
        dscr_config.dataSize = CY_USBD_DMA_WORD;
        dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
        dscr_config.srcAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFC0UL));
        dscr_config.dstAddress = (uint8_t *)0x30000000UL;
        dscr_config.srcXincrement = 1; /* Source is RAM buffer. */
        dscr_config.dstXincrement = 1; /* Address increment is required when writing to EPM as well. */
        dscr_config.xCount = ((bufferSize & 0x3FUL) >> 2U);
        dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
        dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
        dscr_config.yCount = 0;        /* For 1D transfer we dont need this */

        if ((bufferSize & 0x03UL) == 0) {
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;

            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr1), &dscr_config);
        } else {
            dscr_config.nextDescriptor = &(pUsbdCtxt->dmaCh0InXferDscr2);
            dscr_config.channelState = CY_USBD_DMA_CHN_ENABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr1), &dscr_config);

            /* Don't wait for trigger here. */
            dscr_config.retrigger = CY_USBD_DMA_RETRIG_IM;
            dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.dataPrefetch = false;
            dscr_config.dataSize = CY_USBD_DMA_BYTE;
            dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
            dscr_config.srcAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFFCUL));
            dscr_config.dstAddress = (uint8_t *)(0x30000000UL + (bufferSize & 0x3CUL));
            dscr_config.srcXincrement = 1; /* Source is RAM buffer. */
            dscr_config.dstXincrement = 1; /* Address increment is required when writing to EPM as well. */
            dscr_config.xCount = (bufferSize & 0x03UL);
            dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
            dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
            dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr2), &dscr_config);
        }
    } else {
        if ((bufferSize & 0x3FUL) != 0)
        {
            dscr_config.retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
            dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.dataPrefetch = false;
            dscr_config.dataSize = CY_USBD_DMA_BYTE;
            dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
            dscr_config.srcAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFC0UL));
            dscr_config.dstAddress = (uint8_t *)0x30000000UL;
            dscr_config.srcXincrement = 1;
            dscr_config.dstXincrement = 1;
            dscr_config.xCount = (bufferSize & 0x3FUL);
            dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
            dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
            dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr1), &dscr_config);
        }
    }

    Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0, CY_DMAC_INTR_MASK);
    Cy_USBD_DMAChn_SetDesc(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0,
                           pDmaCh0InXferDscr);
    Cy_USBD_DMAChn_Enable(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0);

    if (pUsbdCtxt->ep0SendDone == false) {
        /* Force the trigger to DMA channel input trigger to 1. */
        Cy_TrigMux_SwTrigger(CY_USBD_EGREP_OUT_TRIG,
                             CY_TRIGGER_TWO_CYCLES);
        pUsbdCtxt->ep0SendDone = true;
    }

    /* Clear busy bit to get the data moving. */
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

    /* Wait until DMA transfer is completed. */
    dmaIntr = Cy_USBD_DMAChn_GetIntrStatus(Cy_USBD_EP0In_DmaBase(pUsbdCtxt),
                                           pUsbdCtxt->channel0);

    /* Make sure USB link is in L0 until the transfer is completed. */
    Cy_SysLib_DelayUs(10);

    /* Call USBD LinkActive function so that state are taken care */
    Cy_USBD_GetUSBLinkActive(pUsbdCtxt);

    while (dmaIntr == 0)
    {
        /* If a new request has been received, abort the request and exit. */
        if (Cy_USBHS_Cal_IsNewCtrlRqtReceived(pUsbdCtxt->pCalCtxt)) {
            break;
        }

        loopCnt++;
        if (loopCnt >= 2500U) {
            /* Timeout. */
            break;
        }

#if FREERTOS_ENABLE
        vTaskDelay(1);
#else
        Cy_SysLib_Delay(1);
#endif /* FREERTOS_ENABLE */

        dmaIntr = Cy_USBD_DMAChn_GetIntrStatus(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0);
    }

    if (dmaIntr != 0) {
        /* Clear the DMA channel interrupt status. */
        Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0, dmaIntr);
        DBG_USBD_TRACE("%x DONE\r\n", dmaIntr);
    } else {
        DBG_USBD_WARN("Ep0Send TIMEOUT\r\n");

        /* Disable the DMA channel. */
        Cy_USBD_DMAChn_Disable(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0);

        /*
         * Flush any data in the EPM buffer. Trigger will need to be set for
         * next DMA transfer.
         */
        Cy_USBD_FlushEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN);
        pUsbdCtxt->ep0SendDone = false;

        if (Cy_USBHS_Cal_IsNewCtrlRqtReceived(pUsbdCtxt->pCalCtxt) == false) {
            retval = CY_USBD_STATUS_TIMEOUT;
        }
    }

    return  retval;
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0Data
****************************************************************************//**
*
*  This function receives  data overthe DMA for endpoint 0. Based on speed of
*  device, SS or HS function will be invoked.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/

/*
 * This call is non blocking and if Caller wish to retire RecvEndp0Data
 * after specific timeout then it should implement timer and start timer.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0Data (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;

    if(pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)  {
        retStatus = Cy_USB_USBD_RecvEndp0DataSs(pUsbdCtxt, pBuffer, bufferSize);
    } else {
        retStatus = Cy_USB_USBD_RecvEndp0DataHs(pUsbdCtxt, pBuffer, bufferSize);
    }
    return retStatus;
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0DataSs
****************************************************************************//**
*
*  This function to  recieve data for endpoint 0 through DMA for SS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
* CY_USBD_STATUS_FAILURE if operation is unsuccessful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0DataSs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;
    cy_en_hbdma_mgr_status_t dmaStatus;
    uint32_t actualSize = 0;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("RecvEnd0DataSs:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
        DBG_USBD_ERR("RecvEnd0DataSs:bufferOrSize Null\r\n");
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    /* Make sure we are in U0 state before proceeding. */
    Cy_USBSS_Cal_GetUsbLinkActive(pUsbdCtxt->pSsCalCtxt);

    if (pUsbdCtxt->ep0RecvCb != NULL) {
        pUsbdCtxt->pEp0ReceiveBuffer = pBuffer;
        pUsbdCtxt->ep0ExpRcvSize     = bufferSize;
    }

    /* Receive the data on EP0 using HB DMA */
    dmaStatus = Cy_HBDma_Channel_ReceiveData(&(pUsbdCtxt->outEp0DmaUsb3Ch),
                                             0,
                                             pBuffer,
                                             bufferSize,
                                             NULL);

    if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
        /* Allow control transfer handshake. */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

        if (pUsbdCtxt->ep0RecvCb == NULL) {
            dmaStatus = Cy_HBDma_Channel_WaitForReceiveCplt(&(pUsbdCtxt->outEp0DmaUsb3Ch), 0, &actualSize);
            if (dmaStatus == CY_HBDMA_MGR_SUCCESS) {
                if (actualSize == bufferSize) {
                    DBG_USBD_TRACE("EP0Recv Done\r\n");
                    retStatus = CY_USBD_STATUS_SUCCESS;
                } else {
                    DBG_USBD_WARN("EP0Recv size error: exp=%d act=%d\r\n", bufferSize, actualSize);
                }
            } else {
                DBG_USBD_ERR("EP0Recv DMA error: %x\r\n", dmaStatus);
            }
        } else {
            /* API is non blocking and transfer will complete at a later stage. */
            retStatus = CY_USBD_STATUS_SUCCESS;
        }
    } else {
        DBG_USBD_ERR("EP0Recv DMAQueue error: %x\r\n", dmaStatus);
        pUsbdCtxt->pEp0ReceiveBuffer = NULL;
        pUsbdCtxt->ep0ExpRcvSize = 0;
    }

    return retStatus;
}

/*******************************************************************************
* Function name: Cy_USB_USBD_RecvEndp0DataHs
****************************************************************************//**
*
*  This function to  recieve data for endpoint 0 through DMA for HS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pBuffer
* pointer to buffer where received data will be stored.
*
* \param bufferSize
* Buffer Size.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_BAD_PARAM if pBuffer NULL or bufferSize 0x00.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_stc_usbd_dma_descr_conf_t dscr_config;
    cy_stc_usbd_dma_descr_t *pDmaCh1OutXferDscr;
    cy_en_usbd_ret_code_t retval = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("pUsbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
        DBG_USBD_ERR("pBuffer NULL or bufferSize:%0xx\r\n", bufferSize);
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    pUsbdCtxt->pEp0ReceiveBuffer = pBuffer;
    pUsbdCtxt->ep0ExpRcvSize = bufferSize;
    Cy_USBHS_Cal_UpdateXferCount(pUsbdCtxt->pCalCtxt, 0x00, CY_USB_ENDP_DIR_OUT, bufferSize);

    /* If the amount of data to receive is more than 64 bytes (max. pkt. size
     * for EP0), we need to use two DMA descriptors. The first one will transfer
     * all the full data packets, waiting for trigger from EPM between packets.
     * The second one will transfer the remaining portion at the end.
     *
     * Note: On EP0, we assume that the Host will send us the pre-agreed
     *       amount of data (wLength).
     */
    if (bufferSize >= 64U) {
        /* Need to set src, dst address and update size of transfer ie xCount. */
        pDmaCh1OutXferDscr = &(pUsbdCtxt->dmaCh1OutXferDscr0);

        Cy_USBD_DMADesc_SetSrcAddress(pDmaCh1OutXferDscr, (uint8_t *)0x30004000UL);
        Cy_USBD_DMADesc_SetDstAddress(pDmaCh1OutXferDscr, pBuffer);
        Cy_USBD_DMADesc_SetXloopDataCount(pDmaCh1OutXferDscr, 0x10UL);
        Cy_USBD_DMADesc_SetYloopDataCount(pDmaCh1OutXferDscr, (bufferSize >> 6U));

        if ((bufferSize & 0x3FUL) == 0) {
            /* Transfer size is a multiple of max. pkt. size. So, we can use only the 2-D DMA descriptor. */
            Cy_USBD_DMADesc_SetNextDescriptor(pDmaCh1OutXferDscr, NULL);
            Cy_USBD_DMADesc_SetChannelState(pDmaCh1OutXferDscr, CY_USBD_DMA_CHN_DISABLED);
        } else {
            /* Point to next descriptor. */
            Cy_USBD_DMADesc_SetNextDescriptor(pDmaCh1OutXferDscr, &(pUsbdCtxt->dmaCh1OutXferDscr1));
            Cy_USBD_DMADesc_SetChannelState(pDmaCh1OutXferDscr, CY_USBD_DMA_CHN_ENABLED);
        }
    } else {
        /* DMA will start with descriptor 1 in this case. */
        pDmaCh1OutXferDscr = &(pUsbdCtxt->dmaCh1OutXferDscr1);
    }

    if ((bufferSize & 0x3FUL) >= 0x04UL) {
        /* Descriptor to read out the word portions of the short packet at the end of transfer. */
        dscr_config.retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.dataPrefetch = false;
        dscr_config.dataSize = CY_USBD_DMA_WORD;
        dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
        dscr_config.srcAddress = (uint8_t *)0x30004000UL;
        dscr_config.dstAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFC0UL));
        dscr_config.srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
        dscr_config.dstXincrement = 1; /* Destination is RAM address */
        dscr_config.xCount = ((bufferSize & 0x3FUL) >> 2U);
        dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
        dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
        dscr_config.yCount = 0;        /* For 1D transfer we dont need this */

        if ((bufferSize & 0x03UL) == 0) {
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;

            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr1), &dscr_config);
        } else {
            dscr_config.nextDescriptor = &(pUsbdCtxt->dmaCh1OutXferDscr2);
            dscr_config.channelState = CY_USBD_DMA_CHN_ENABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr1), &dscr_config);

            dscr_config.retrigger = CY_USBD_DMA_RETRIG_IM;
            dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.dataPrefetch = false;
            dscr_config.dataSize = CY_USBD_DMA_BYTE;
            dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
            dscr_config.srcAddress = (uint8_t *)(0x30004000UL + (bufferSize & 0x3CUL));
            dscr_config.dstAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFFCUL));
            dscr_config.srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
            dscr_config.dstXincrement = 1; /* Destination is RAM address */
            dscr_config.xCount = (bufferSize & 0x03UL);
            dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
            dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
            dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr2), &dscr_config);
        }
    } else {
        if ((bufferSize & 0x3FUL) != 0) {
            dscr_config.retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
            dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
            dscr_config.dataPrefetch = false;
            dscr_config.dataSize = CY_USBD_DMA_BYTE;
            dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
            dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
            dscr_config.srcAddress = (uint8_t *)0x30004000UL;
            dscr_config.dstAddress = (uint8_t *)(pBuffer + (bufferSize & 0xFFC0UL));
            dscr_config.srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
            dscr_config.dstXincrement = 1; /* Destination is RAM address */
            dscr_config.xCount = (bufferSize & 0x3FUL);
            dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
            dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
            dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
            dscr_config.nextDescriptor = NULL;
            dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
            Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr1), &dscr_config);
        }
    }

    Cy_USBD_DMAChn_SetDesc(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, pDmaCh1OutXferDscr);
    if (pUsbdCtxt->ep0RecvCb != NULL) {
        Cy_USBD_DMAChn_EnableIntr(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1);
    } else {
        Cy_USBD_DMAChn_DisableIntr(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1);
    }
    Cy_USBD_DMAChn_Enable(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1);

    /*  DMA is enabled so inform USB controller to recieve data and stop NAK. */
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    Cy_SysLib_DelayUs(10);

    /* Make sure USB link is in L0 until the transfer is completed. */
    /* Call USBD LinkActive function so that state are taken care */
    Cy_USBD_GetUSBLinkActive(pUsbdCtxt);

    return retval;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_IsEp0ReceiveDone
****************************************************************************//**
*
* This function checks whether a EP0 data receive operation is pending.
* Please note that the function will return true to indicate transfer done
* if there was no pending transfer in the first place.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* true if there is no pending EP0 OUT data transfer.
* false if there is a pending EP0 OUT data transfer.
*******************************************************************************/
bool Cy_USBD_IsEp0ReceiveDone (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    bool ep0_idle = true;

    if (pUsbdCtxt != NULL) {
        if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
            if (Cy_USBD_DMAChn_IsEnabled(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1)) {
                ep0_idle = false;
            }

            if (Cy_USBHS_Cal_IsNewCtrlRqtReceived(pUsbdCtxt->pCalCtxt)) {
                DBG_USBD_TRACE("Recv abort\r\n");
                Cy_USB_USBD_RetireRecvEndp0DataHs(pUsbdCtxt);
                ep0_idle = true;
            }
        } else {
            ep0_idle = (pUsbdCtxt->pEp0ReceiveBuffer == NULL);
        }
    }

    return ep0_idle;
}

/*******************************************************************************
* Function name: Cy_USBD_HandleGetDscr
****************************************************************************//**
*
*  This function handles all get descriptor command coming from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_setup_req_t setupReq,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    bool isReqHandledByApp = false;
    uint16_t wValue, wLength, wTotalLength;
    uint8_t *pBuffer = NULL;
    uint32_t bufferSize = 0x00;
    uint8_t dscrType, dscrIndex;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;

    DBG_USBD_INFO("HandleGetDscr Speed:%d\r\n", pUsbdCtxt->devSpeed);

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlGetDsc:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /* For Valid request send descriptor otherwise send STALL. */
    wValue = setupReq.wValue;
    wLength = setupReq.wLength;

    dscrType = ((wValue & CY_USB_CTRL_REQ_DSCR_TYPE_MASK)
                                      >> CY_USB_CTRL_REQ_DSCR_TYPE_POS);
    dscrIndex = ((wValue & CY_USB_CTRL_REQ_DSCR_INDEX_MASK)
                                      >> CY_USB_CTRL_REQ_DSCR_INDEX_POS);
    switch (dscrType) {

        case  CY_USB_DEVICE_DSCR:
            DBG_USBD_INFO("DevDscr\r\n");
            if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
                (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
                pBuffer = pUsbdCtxt->dscrs.pUsbDevDscr;
            } else {
                pBuffer = pUsbdCtxt->dscrs.pUsbSsDevDscr;
            }
            if (pBuffer) {
                bufferSize =  CY_USB_MIN(wLength, CY_USB_DEVICE_DSCR_LEN);
            }
            break;

        case CY_USB_CONFIG_DSCR:
            DBG_USBD_INFO("CfgDscr\r\n");
            if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
                /* device implementing maximum two descriptors. */
                pBuffer = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
            } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
            } else {
                /* super speed case handled here */
                pBuffer = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
            }

            /*
             * Device can support multiple configuration but at a time
             * one configuration will be active. dscrIndex points to
             * configur number.
             * As per USB specification Valid config number starts from 1 but
             * index number mentioned in "USB request" starts from 0x00.
             * To check with actual config number, dscrIndex+1 should
             * be used.
             *
             * If config number is not supported in present descriptor then
             * make buffer NULL so that STALL should be sent.
             */
            if (!(Cy_USBD_isCfgValid((dscrIndex + 1), pBuffer))) {
                DBG_USBD_ERR("dscrIndex CfgNotValid\r\n");
                pBuffer = NULL;
            }

            if (pBuffer) {
                wTotalLength = CY_USB_BUILD_2B_WORD(pBuffer[3],pBuffer[2]);
                bufferSize = CY_USB_MIN(wLength, wTotalLength);
            }
            break;

        case CY_USB_STRING_DSCR:
           /*
            * If string index is greater than CY_USBD_MAX_STR_DSCR_INDX OR if
            * the string descriptor is not registered, then pass the request to
            * application.
            * If string descriptor is not available then send STALL.
            */
            DBG_USBD_INFO("StrDscr\r\n");
            if (pUsbdCtxt->strDscrAvailable == TRUE) {
                if ((dscrIndex > CY_USBD_MAX_STR_DSCR_INDX) ||
                    (pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex] == NULL)) {
                    isReqHandledByApp = true;
                } else {
                    pBuffer = pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex];
                    bufferSize = CY_USB_MIN(wLength, pBuffer[0]);
                }
            } else {
                /* if string dscr not registered (index 0 does not matter) */
                pBuffer = NULL;
            }
            break;

        case CY_USB_DEVICE_QUAL_DSCR:
            DBG_USBD_INFO("QualDscr\r\n");
            pBuffer = pUsbdCtxt->dscrs.pUsbDevQualDscr;
            if (pBuffer) {
                bufferSize = CY_USB_MIN(wLength, CY_USB_DEVICE_QUAL_DSCR_LEN);
            }
            break;

        case CY_USB_OTHERSPEED_DSCR:
           /*
            * if device is operating at full speed then other speed config
            * means high speed config. if device operating at high speed
            * then other speed means full speed config descriptor.
            */
            DBG_USBD_INFO("OtherSpeedDscr\r\n");
            if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
            } else {
                pBuffer = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
            }

            if (pBuffer) {
                wTotalLength =  CY_USB_BUILD_2B_WORD(pBuffer[3],pBuffer[2]);
                bufferSize = CY_USB_MIN(wLength, wTotalLength);
                /*
                 * Copy the descriptor into local buffer and change
                 * descriptor type.
                */
                memcpy((uint8_t *)pUsbdCtxt->otherSpeedCfgDscrBuf, pBuffer, bufferSize);
                pBuffer = (uint8_t *)(pUsbdCtxt->otherSpeedCfgDscrBuf);
                pBuffer[1U] = 0x07U;
            }
            break;

        case CY_USB_BOS_DSCR:
            /*
             * For USBSS, use SsBosDscr.
             * For USB FS/HS it should be 2.1 version number.
             */
            DBG_USBD_INFO("BosDscr\r\n");
            if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbSsBosDscr;
            } else {
                if (pUsbdCtxt->dscrs.pUsbDevDscr[2] == 0x10) {
                    pBuffer = pUsbdCtxt->dscrs.pUsbHsBosDscr;
                }
            }

            if (pBuffer) {
                wTotalLength =  CY_USB_BUILD_2B_WORD(pBuffer[3],pBuffer[2]);
                bufferSize = CY_USB_MIN(wLength, wTotalLength);
            }
            break;

        default:
            /* Control should not reach here. */
            DBG_USBD_INFO("UnknownDscr dscrType:%d\r\n", dscrType);
            isReqHandledByApp = true;
            break;
    }   /* end of switch */

    /*
     * First Check dscr should be handled by application, if true then
     * inform application and come out from function.
     * If pBuffer is valid then send data/ack based on request.
     * If pBuffer is Null then send STALL. This request not handled by device.
     */
    if (isReqHandledByApp) {
        if (pUsbdCtxt->setupCb) {
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                          CY_USB_ENDP_DIR_IN, CY_USB_SET);
        }
        return CY_USBD_STATUS_SUCCESS;
    }

    if (pBuffer) {
        if (bufferSize == 0) {
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        } else {
            Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, pBuffer, bufferSize);
            if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
                    (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

                /*
                 * If (wLength > bufferSize) and bufferSize is a multiple of max.
                 * pkt. size; we need to send a ZLP to complete the transfer.
                 */
                if ((wLength > bufferSize) && ((bufferSize & 0x3FUL) == 0)) {
                    Cy_USBHS_Cal_SendEgressZLP(pUsbdCtxt->pCalCtxt, 0x00);
                }
            }
        }
    } else {
        DBG_USBD_INFO("SndSTALL\r\n");
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_IN, CY_USB_SET);
    }
    return(retStatus);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleGetStatus
****************************************************************************//**
*
*  This function handles get status request coming from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetStatus (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_setup_req_t setupReq)
{
    uint8_t recipient;
    uint8_t *pBuffer = NULL;
    /* Get status always returns two byte of data except for the PTM status request */
    uint16_t bufferSize = 0x02;
    uint16_t endpNum = 0x00;
    uint32_t getStatusData = 0x00;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlGetStatus:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    DBG_USBD_TRACE("HandleGetStatus\r\n");
    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);

    switch (recipient) {

        case CY_USB_CTRL_REQ_RECIPENT_DEVICE:
            DBG_USBD_INFO("GetStatusDevice\r\n");
            if(setupReq.wValue == 0x01) {
                /* PTM STATUS Request */
                getStatusData = Cy_USBSS_Cal_Get_PtmStatus(pUsbdCtxt->pSsCalCtxt);
                bufferSize = 0x04;
            }
            else {
                getStatusData = pUsbdCtxt->usbDeviceStat;
            }
            pBuffer = (uint8_t *)&getStatusData;
            break;

        case CY_USB_CTRL_REQ_RECIPENT_INTF:
            /* If device is remote wake capable, report capability on all interfaces as well. */
            getStatusData = 0x00;
            if ((pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) && (pUsbdCtxt->remoteWakeupAbility)) {
                getStatusData = 0x01UL;
                if ((setupReq.wIndex < CY_USB_MAX_INTF) &&
                        ((pUsbdCtxt->intfRemoteWakeEnabled & (0x01UL << setupReq.wIndex)) != 0)) {
                    getStatusData = 0x03UL;
                }
            }
            pBuffer = (uint8_t *)&getStatusData;
            break;


        case CY_USB_CTRL_REQ_RECIPENT_ENDP:
            endpNum = setupReq.wIndex & 0x7FU;
            if (((setupReq.wIndex) & 0x80U) != 0) {
                if (pUsbdCtxt->endpInfoIn[endpNum].halt) {
                    getStatusData = (uint16_t)CY_USB_GET_STATUS_ENDP_HALT;
                } else {
                    getStatusData = 0x00;
                }
            } else {
                if (pUsbdCtxt->endpInfoOut[endpNum].halt) {
                    getStatusData = (uint16_t)CY_USB_GET_STATUS_ENDP_HALT;
                } else {
                    getStatusData = 0x00;
                }
            }
            pBuffer = (uint8_t *)&getStatusData;
            break;

        default:
            pBuffer = NULL;
            break;
    }

    if (pBuffer) {
        Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, pBuffer, bufferSize);
    } else {
        DBG_USBD_INFO("GetStatusStall\r\n");
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_IN, CY_USB_SET);
    }   /* end of else pBuffer */
    return(retStatus);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleSetFeature
****************************************************************************//**
*
*  This function handles set-feature request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetFeature (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_setup_req_t setupReq,
                          cy_stc_usb_cal_msg_t *pMsg)
{

    uint8_t testMode;
    uint8_t recipient;
    uint16_t endpNum;
    cy_en_usb_endp_dir_t dir;
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    bool handled = false;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlSetFea:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /*
     * In case of set feature, direction will be available
     * with endpoint number.
     */
    if ((setupReq.wIndex) & (uint16_t)0x80U) {
        dir = CY_USB_ENDP_DIR_IN;
    } else {
        dir = CY_USB_ENDP_DIR_OUT;
    }

    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);
    endpNum = setupReq.wIndex & 0x7FU;

    if ((CY_USB_CTRL_REQ_RECIPENT_ENDP == recipient) &&
        (CY_USB_FEATURE_ENDP_HALT == setupReq.wValue) &&
        (endpNum < CY_USB_MAX_ENDP_NUMBER)) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
            DBG_USBD_TRACE("setFeatureSetupClbEndp\r\n");
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                                                      CY_USB_SET);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_INTF == recipient) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
            DBG_USBD_TRACE("setFeatureSetupClbIntf\r\n");
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);

            /* Update function remote wake setting based on Set Feature command. */
            if ((pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) &&
                    (setupReq.wValue == CY_USB_FEATURE_FUNC_SUSPEND) &&
                    (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) &&
                    (pUsbdCtxt->remoteWakeupAbility) &&
                    (CY_USB_MAX_INTF > (uint8_t)(setupReq.wIndex))) {
                if ((setupReq.wIndex & 0x0200UL) != 0) {
                    pUsbdCtxt->intfRemoteWakeEnabled |= (0x01UL << (uint8_t)(setupReq.wIndex));
                } else {
                    pUsbdCtxt->intfRemoteWakeEnabled &= ~(0x01UL << (uint8_t)(setupReq.wIndex));
                }
            }
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                                                      CY_USB_SET);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_DEVICE == recipient) {
        switch (setupReq.wValue) {
            case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                /*
                 * If device has ability then update data structure and send ACK.
                 * If device does not have ability then send STALL through endp0.
                 */
                if (pUsbdCtxt->remoteWakeupAbility) {
                    pUsbdCtxt->remoteWakeupEnable = 1;
                    DBG_USBD_TRACE("RemoteWakeup Enabled\r\n");
                    pUsbdCtxt->usbDeviceStat |= CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP;
                    /* Trigger callback */
                    if (pUsbdCtxt->setupCb) {
                        DBG_USBD_TRACE("setFeatureSetupClbDevice\r\n");
                        pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                    } else {
                        /*
                         * if application does not have callback then USBD can
                         * respond and send ACK during status stage.
                         */
                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    }
                } else {
                    retStatus =
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_IN,
                                                  CY_USB_SET);
                }
                handled = true;
                break;

            case CY_USB_FEATURE_DEVICE_TEST_MODE:
                testMode = (uint8_t)((setupReq.wIndex & 0xFF00) >> 8);
                calRetStatus = Cy_USBHS_Cal_SetTestMode(pUsbdCtxt->pCalCtxt,
                        (cy_en_usbhs_cal_test_mode_t)testMode);

                /* TEST MODE set proerly then HW can send ACK for Status Stage */
                if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
                    /* Trigger callback */
                    if (pUsbdCtxt->setupCb) {
                        DBG_USBD_INFO("setFeatureSetupClbTest\r\n");
                        pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                    } else {
                        /*
                         * if application does not have callback then USBD can
                         * respond and send ACK during status stage.
                         */
                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    }
                    handled = true;
                }
                break;

            case CY_USB_FEATURE_U1_ENABLE:
                if(pUsbdCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
                    pUsbdCtxt->usbDeviceStat |= CY_USB_GET_STATUS_DEV_U1_ENABLE;

                    /* Trigger callback so that application can decide on LPM policy. */
                    if (pUsbdCtxt->setupCb != NULL) {
                        pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                    } else {
                        /* Enable LPM once U1/U2 is enabled by host. */
                        DBG_USBD_INFO("LPMEnable: U1\r\n");
                        Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, false);

                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    }

                    handled = true;
                }

                break;

            case CY_USB_FEATURE_U2_ENABLE:
                if(pUsbdCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
                    pUsbdCtxt->usbDeviceStat |= CY_USB_GET_STATUS_DEV_U2_ENABLE;

                    /* Trigger callback so that application can decide on LPM policy. */
                    if (pUsbdCtxt->setupCb != NULL) {
                        pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                    } else {
                        /* Enable LPM once U1/U2 is enabled by host. */
                        DBG_USBD_INFO("LPMEnable: U2\r\n");
                        Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, false);
                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    }

                    handled = true;
                }
                break;

            case CY_USB_FEATURE_LTM_ENABLE:
                if(pUsbdCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
                    /* TODO: Add proper handling for these requests. For now, just acknowledge. */
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    handled = true;
                }
                break;

                case CY_USB_FEATURE_LDM_ENABLE:
                    DBG_USBD_INFO("LDM setFeature\r\n");
                    Cy_USBSS_Cal_PTMConfig(pUsbdCtxt->pSsCalCtxt, true);
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    handled = true;
                break;

            default:
                /* Unknown feature selector: Request will be stalled below. */
                break;
        }
    }

    if (!handled) {
        retStatus =
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                      CY_USB_ENDP_DIR_IN, CY_USB_SET);
    }

    if (CY_USB_CAL_STATUS_FAILURE == calRetStatus) {
       retStatus = CY_USBD_STATUS_FAILURE;
    }

    return (retStatus);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleClearFeature
****************************************************************************//**
*
*  This function handles clear-feature request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
/* Clear feature does not have data stage. */
cy_en_usbd_ret_code_t
Cy_USBD_HandleClearFeature (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    uint8_t recipient;
    uint16_t endpNum;
    cy_en_usb_endp_dir_t dir;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    bool handled = false;

    if ((setupReq.wIndex) & (uint16_t)0x80U) {
        dir = CY_USB_ENDP_DIR_IN;
    } else {
        dir = CY_USB_ENDP_DIR_OUT;
    }

    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);
    endpNum = setupReq.wIndex & 0x7FU;

    if ((CY_USB_CTRL_REQ_RECIPENT_ENDP == recipient) &&
        (CY_USB_FEATURE_ENDP_HALT == setupReq.wValue) &&
        (endpNum < CY_USB_MAX_ENDP_NUMBER)) {

        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
            DBG_USBD_TRACE("clearFeature:SetupClbEndp\r\n");
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /*
             * Call CAL function to clear stall for given endpoint
             * and send ACK during status stage.
             */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum,
                                                      dir, CY_USB_CLEAR);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_INTF == recipient) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
            DBG_USBD_TRACE("ClearFeature:SetupClbIntf\r\n");
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                                                      CY_USB_SET);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_DEVICE == recipient) {
        switch(setupReq.wValue) {
            case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
            /*
             * If device has ability then update data structure and send ACK.
             * If device does not have ability then send STALL through endp0.
             */
                if (pUsbdCtxt->remoteWakeupAbility) {
                    pUsbdCtxt->remoteWakeupEnable = 0;
                    pUsbdCtxt->usbDeviceStat &= ~CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP;
                    /* Trigger callback */
                    if (pUsbdCtxt->setupCb) {
                        DBG_USBD_TRACE("clearFeature:SetupClbDevice\r\n");
                        pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                    } else {
                        /*
                         * if application does not have callback then USBD can
                         * respond and send ACK during status stage.
                         */
                        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    }
                } else {
                    retStatus=
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_IN,
                                                  CY_USB_SET);
                }
                handled = true;
                break;

            case CY_USB_FEATURE_U1_ENABLE:
                DBG_USBD_TRACE("clearFeature:U1 Enable\r\n");
                pUsbdCtxt->usbDeviceStat &= ~CY_USB_GET_STATUS_DEV_U1_ENABLE;

                /* Trigger callback so that application can decide on LPM policy. */
                if (pUsbdCtxt->setupCb != NULL) {
                    pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                } else {
                    /* Disable LPM transitions. */
                    DBG_USBD_INFO("LPMDisable: U1\r\n");
                    Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                }

                handled = true;
                break;

            case CY_USB_FEATURE_U2_ENABLE:
                DBG_USBD_TRACE("clearFeature:U2 Enable\r\n");
                pUsbdCtxt->usbDeviceStat &= ~CY_USB_GET_STATUS_DEV_U2_ENABLE;

                /* Trigger callback so that application can decide on LPM policy. */
                if (pUsbdCtxt->setupCb != NULL) {
                    pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                } else {
                    DBG_USBD_INFO("LPMDisable: U2\r\n");
                    Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                }
                handled = true;
                break;

            case CY_USB_FEATURE_LTM_ENABLE:
                /* TODO: Add proper handling for these requests. For now, just acknowledge. */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                handled = true;
                break;

             case CY_USB_FEATURE_LDM_ENABLE:
                DBG_USBD_INFO("LDMDisable\r\n");
                Cy_USBSS_Cal_PTMConfig(pUsbdCtxt->pSsCalCtxt, false);
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                handled = true;
                break;

            default:
                /* STALL is sent from below. */
                break;

        }
    }

    if (!handled) {
        /* Stall EP0 if the request has not been handled. */
        retStatus =
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                      CY_USB_ENDP_DIR_IN, CY_USB_SET);
    }
    return(retStatus);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleSetConfiguration
****************************************************************************//**
*
*  This function handles set configuration request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetConfiguration (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_setup_req_t setupReq,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    uint8_t *pCfgDscr;
    uint8_t configNum;
    uint8_t totalIntf;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    uint32_t index;
    /*
     * Check config value proper or not.
     * Make required configuration as active configuration.
     * Configure endpoints.
     * Call application call back related to set_config.
     */

    DBG_USBD_INFO("HandleSetCfg\r\n");
    configNum = (uint8_t)((setupReq.wValue) & (0x00FF));

    if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
        pCfgDscr = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
    } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
        pCfgDscr = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
    } else {
        pCfgDscr = pUsbdCtxt->dscrs.pUsbSsCfgDscr;
    }

    if (0x00 == configNum) {
        /*
         * If host sends setCfg with config 0 then device should move to
         * Address state and send ACK to host.
         */
        pUsbdCtxt->pActiveCfgDscr = NULL;
        pUsbdCtxt->activeCfgNum = 0x00;
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ADDRESS;
        pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
        pUsbdCtxt->pActiveCfgDscr = NULL;
        pUsbdCtxt->activeCfgNum = 0x00;
        pUsbdCtxt->selfPowered = 0x00;
        pUsbdCtxt->remoteWakeupAbility = 0x00;
        pUsbdCtxt->remoteWakeupEnable = 0x00;
        pUsbdCtxt->EnumerationDone = false;
        pUsbdCtxt->intfRemoteWakeEnabled = 0x00UL;

        /* Trigger callback */
        if (pUsbdCtxt->setConfigCb) {
           DBG_USBD_INFO("SetCfg0Clb\r\n");
           pUsbdCtxt->setConfigCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        return(retStatus);
    }

    if (Cy_USBD_isCfgValid(configNum,pCfgDscr)) {
        pUsbdCtxt->pActiveCfgDscr = pCfgDscr;
        pUsbdCtxt->activeCfgNum = configNum;
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = CY_USB_DEVICE_STATE_CONFIGURED;
        totalIntf = pUsbdCtxt->numOfIntfInActiveCfg =
                            Cy_USBD_FindNumOfIntf (pCfgDscr);
        DBG_USBD_INFO("Number of interface in cfg:%d\r\n",totalIntf);
        for (index = 0x00; index < totalIntf; index++) {
            pUsbdCtxt->numOfAltSettings[index] =
                        Cy_USBD_UpdateNumOfAltSetting(pUsbdCtxt, index);
            DBG_USBD_INFO("TotalAltSetting:%d\r\n",pUsbdCtxt->numOfAltSettings[index]);
        }
        if (Cy_USBD_FindRemoteWakeupAbility(pCfgDscr)) {
            pUsbdCtxt->remoteWakeupAbility = 0x01;
        } else {
            pUsbdCtxt->remoteWakeupAbility = 0x00;
        }

        if (Cy_USBD_FindSelfPower(pCfgDscr)) {
            pUsbdCtxt->selfPowered = 0x01;
            pUsbdCtxt->usbDeviceStat |= CY_USB_GET_STATUS_DEV_SELF_POWER;
        } else {
            pUsbdCtxt->selfPowered = 0x00;
            pUsbdCtxt->usbDeviceStat &= ~CY_USB_GET_STATUS_DEV_SELF_POWER;
        }
        pUsbdCtxt->EnumerationDone = true;

        /* Trigger callback */
        if (pUsbdCtxt->setConfigCb) {
           DBG_USBD_INFO("SetCfgClb\r\n");

           if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_SS_GEN1) || (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_SS_GEN1X2)) {
               /*
                * In Gen1x1 and Gen1x2 operation, retain a break of 3 AXI cycles between each burst fetched by USB
                * egress DMA adapter.
                */
               Cy_HBDma_Mgr_SetUsbEgressAdapterDelay(pUsbdCtxt->pHBDmaMgr, 3);
           } else {
               /*
                * No delays to be applied between bursts in Gen2x1 and Gen2x2 operation.
                * The setting is don't care for USBHS or USBFS operation.
                */
               Cy_HBDma_Mgr_SetUsbEgressAdapterDelay(pUsbdCtxt->pHBDmaMgr, 0);
           }

           pUsbdCtxt->setConfigCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }

        /* No data stage so enable HW to send ACK in status stage. */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    } else {
        /* Need to send stall */
        DBG_USBD_INFO("SetCfgStall\r\n");
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_IN,
                                                  CY_USB_SET);
        retStatus = CY_USBD_STATUS_FAILURE;
    }
    return(retStatus);
}   /* End of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleGetConfiguration
****************************************************************************//**
*
*  This function handles get configuration request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetConfiguration (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_setup_req_t setupReq)
{
    uint32_t curConfig;
    uint16_t bufferSize = 0x00;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    curConfig = (uint32_t)pUsbdCtxt->activeCfgNum;
    bufferSize =  0x01;
    Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)&curConfig, bufferSize);
    return(retStatus);
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USBD_HandleSetInterface
****************************************************************************//**
*
*  This function handles set interface request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetInterface (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;
    uint8_t *pCfgDscr = pUsbdCtxt->pActiveCfgDscr;
    uint32_t intfNum, altSetting;

    intfNum = setupReq.wIndex;
    altSetting = setupReq.wValue;

    if ((pCfgDscr == NULL) ||
         (intfNum >= (*(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF)))) {
        /* Corruption in memory so just send STALL. */
        retCode =
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                      CY_USB_ENDP_DIR_IN, CY_USB_SET);
    } else {
        /* things are fine so update info, trigger callback, initiate ACK. */
        pUsbdCtxt->altSettings[intfNum] = altSetting;
        if (pUsbdCtxt->setIntfCb) {
            pUsbdCtxt->setIntfCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
        /* No data stage so enable HW to send ACK. */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    }
    return(retCode);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleGetInterface
****************************************************************************//**
*
*  This function handles get interface request came from host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setupReq
* setup request which came from host.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetInterface (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq)
{
    uint32_t bufferSize = 0x00;
    uint32_t altSetting;
    uint16_t intf;
    uint8_t *pBuffer;
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    /*
     * altSetting is always updated in deviceInfo so it can be used as it is.
     * Only Interface number should be checked.
     * Code can be written without pBuffer also.
     */
    intf = setupReq.wIndex;
    if (Cy_USBD_isIntfValid(intf, pUsbdCtxt->pActiveCfgDscr)) {
        altSetting = (uint32_t)pUsbdCtxt->altSettings[intf];
        pBuffer = (uint8_t *)&altSetting;
        bufferSize =  0x01;
        calRetStatus = Cy_USBHS_Cal_UpdateXferCount(pUsbdCtxt->pCalCtxt, 0x00,
                                                CY_USB_ENDP_DIR_IN, bufferSize);
        Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, pBuffer, bufferSize);
    } else {
        /* If interface is not valid interface then send STALL. */
        retStatus =
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                      CY_USB_ENDP_DIR_IN, CY_USB_SET);
    }

    if (CY_USB_CAL_STATUS_FAILURE == calRetStatus) {
        retStatus = CY_USBD_STATUS_FAILURE;
    }
    return(retStatus);
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleReset
****************************************************************************//**
*
*  This function handles bus reset.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    cy_stc_usb_cal_msg_t *pMsg)
{
    uint32_t endp0MaxPktSize;
    cy_stc_usb_endp_config_t endpConfig;
    uint32_t intfNum;
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    /*
     * This section assumes Fast enumeation enabled.
     * Some of the data structure needs to be reinitialized.
     * Configure like device working in FS mode.
     * Configure H/W register related to endpoint and take endp size from
     * available descriptor.
     */
    DBG_USBD_INFO("HandleReset\r\n");

    /* If the USBHS link was previously suspended, we have to go through the resume flow first. */
    if ((pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) && (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND)) {
        Cy_USBHS_Cal_HsHandleL2Resume(pUsbdCtxt->pCalCtxt);
    }

    /* Initialize usb device info structure. */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;

    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;
    pUsbdCtxt->intfRemoteWakeEnabled = 0x00UL;
    pUsbdCtxt->numOfIntfInActiveCfg = 0x00;

    /* During reset make sure that all altesetting should be 0 */
    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
        pUsbdCtxt->numOfAltSettings[intfNum] = 0x00;
    }
    /*
     * Configure endpoint 0x00 and make it valid.
     * As off now ignoring retCode because endp0MaxPktSize will be 0
     * in case of error condition.
     */
    retCode = Cy_USBD_FindEndp0MaxPktSize(pUsbdCtxt->dscrs.pUsbDevDscr,
                                          pUsbdCtxt->devSpeed,
                                          &endp0MaxPktSize);
    endpConfig.valid = 0x01;
    endpConfig.endpNumber = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_CTRL;
    endpConfig.maxPktSize = endp0MaxPktSize;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;
    endpConfig.allowNakTillDmaRdy = false;      /* Allow NAK function should not be enabled for EP0. */

    pUsbdCtxt->endpInfoIn[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoIn[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoIn[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;


    pUsbdCtxt->endpInfoOut[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoOut[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoOut[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;

    calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /* Connect trigger from EP0 egress EPM to DMA channel. */
    Cy_TrigMux_Connect(CY_USBD_EGREP_OUT_TRIG,
                        CY_USBD_EGREP_DMA_TRIG_BASE + pUsbdCtxt->channel0,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from DMA channel to EP0 egress EPM. */
    Cy_TrigMux_Connect(CY_USBD_EGREP_DMAOUT_TRIG_BASE + pUsbdCtxt->channel0,
                        CY_USBD_EGREP_IN_TRIG,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from EP0 ingress EPM to DMA channel. */
    Cy_TrigMux_Connect(CY_USBD_INGEP_OUT_TRIG,
                        CY_USBD_INGEP_DMA_TRIG_BASE + pUsbdCtxt->channel1,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from DMA channel to EP0 ingress EPM. */
    Cy_TrigMux_Connect(CY_USBD_INGEP_DMAOUT_TRIG_BASE + pUsbdCtxt->channel1,
                       CY_USBD_INGEP_IN_TRIG,
                       false, TRIGGER_TYPE_EDGE);

    /* Clear the flag indicating with any EP0 data send has been done. */
    pUsbdCtxt->ep0SendDone = false;

    Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_HandleReset(pUsbdCtxt->pCalCtxt);

    /* Keep L1 and Ux entry disabled at startup. */
    Cy_USB_LpmSetClearNYET(pUsbdCtxt, true);

    if (pUsbdCtxt->busResetCb) {
        /* Inform about Speed */
        pUsbdCtxt->busResetCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (retCode);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleSsReset
****************************************************************************//**
*
*  This function handles reset in SS mode.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSsReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    uint32_t endp0MaxPktSize;
    cy_stc_usb_endp_config_t endpConfig;
    uint32_t intfNum;
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    /*
     * This section assumes Fast enumeation enabled.
     * Some of the data structure needs to be reinitialized.
     * Configure like device working in SS_GEN1 more.
     * Configure H/W register related to endpoint and take endp size from
     * available descriptor.
     */

    /* Initialize usb device info structure. */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbdCtxt->usbDeviceStat = 0x00;

    /* Update operating device speed based on CAL context. */
    if (pUsbdCtxt->pSsCalCtxt->dualLaneEnabled) {
        if (pUsbdCtxt->pSsCalCtxt->gen2Enabled) {
            pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_SS_GEN2X2;
        } else {
            pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_SS_GEN1X2;
        }
    } else {
        if (pUsbdCtxt->pSsCalCtxt->gen2Enabled) {
            pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_SS_GEN2;
        } else {
            pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_SS_GEN1;
        }
    }

    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;

    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;
    pUsbdCtxt->intfRemoteWakeEnabled = 0x00;
    pUsbdCtxt->numOfIntfInActiveCfg = 0x00;

    /* During reset make sure that all altesetting should be 0 */
    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
        pUsbdCtxt->numOfAltSettings[intfNum] = 0x00;
    }
    /*
     * Configure endpoint 0x00 and make it valid.
     * As off now ignoring retCode because endp0MaxPktSize will be 0
     * in case of error condition.
     */
    retCode = Cy_USBD_FindEndp0MaxPktSize(pUsbdCtxt->dscrs.pUsbSsDevDscr,
                                          pUsbdCtxt->devSpeed,
                                          &endp0MaxPktSize);
    endpConfig.valid = 0x01;
    endpConfig.endpNumber = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_CTRL;
    endpConfig.maxPktSize = endp0MaxPktSize;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;

    pUsbdCtxt->endpInfoIn[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoIn[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoIn[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;

    /*
     * If there was a retry buffer region previously allocated,
     * free it.
     */
    if (pUsbdCtxt->endpInfoIn[0].retryBufOffset < 16384U) {
        FreeRetryBufferRegion(pUsbdCtxt,
                              pUsbdCtxt->endpInfoIn[0].retryBufOffset, 1U);
        pUsbdCtxt->endpInfoIn[0].retryBufOffset = 0xFFFFU;
    }

    pUsbdCtxt->endpInfoOut[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoOut[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoOut[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].retryBufOffset = 0xFFFFU;

    calRetCode = Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;

        /* Allocate 1KB of retry buffer space for EP0-IN. */
        pUsbdCtxt->endpInfoIn[0].retryBufOffset = GetRetryBufferRegion(
                pUsbdCtxt, 1U);
        if (pUsbdCtxt->endpInfoIn[0].retryBufOffset >= 16384U) {
            endpConfig.valid = false;
            Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
            pUsbdCtxt->endpInfoIn[0].valid = false;
            retCode = CY_USBD_STATUS_FAILURE;
        } else {
            Cy_USBSS_Cal_SetEndpRetryOffset(pUsbdCtxt->pSsCalCtxt, 0,
                    pUsbdCtxt->endpInfoIn[0].retryBufOffset);
        }
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /* Clear the flag indicating with any EP0 data send has been done. */
    pUsbdCtxt->ep0SendDone = false;

    /* Flush and reset EP0 in both directions. */
    Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, 0x00, CY_USB_ENDP_DIR_IN);

    /* Keep LPM disabled at beginning of connection. */
    DBG_USBD_TRACE("LPMDisable\r\n");
    Cy_USB_LpmSetClearNYET(pUsbdCtxt, true);

    /* Flush the EPM to start with. */
    Cy_USBSS_Cal_FlushEPM(pUsbdCtxt->pSsCalCtxt, false);

    /*
     * Notify application if this is reset event.
     * Notification is not required for connect event at present.
     */
    if (pMsg->type == CY_USBSS_CAL_MSG_LNK_RESET) {
        if (pUsbdCtxt->busResetCb) {
            /* Inform about Speed */
            pUsbdCtxt->busResetCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
    }
    return (retCode);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleHsGrant
****************************************************************************//**
*
*  High speed related switching is handled here.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleHsGrant (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HsGrant: pUsbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    /*
     * high speed config descriptor will be more relavent.
     * FS descriptor will become other descriptor.
     * speed become HighSpeed.
     */
    pUsbdCtxt->dscrs.pUsbCfgDscr = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
    pUsbdCtxt->dscrs.pUsbOtherSpeedCfgDscr = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_HS;

    if (pUsbdCtxt->busSpeedCb) {
        /* Inform about Speed */
        pUsbdCtxt->busSpeedCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* End of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleRxFailure
****************************************************************************//**
*
* Enables USB stack level work-around to attempt Gen2 connection start-up multiple
* times in case of training failure in Polling.Active or Polling.Config states.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param enable
* Whether work-around is to be enabled.
*
* \param retry_cnt
* Number of connection attempts to be made in case of Gen2 training failure.
*******************************************************************************/
void
Cy_USBD_HandleRxFailure (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool enable, uint8_t retry_cnt)
{
    pUsbdCtxt->gen2_lnk_retry_cnt = 0;

    if (enable) {
        pUsbdCtxt->handleRxFailure = true ;
        pUsbdCtxt->gen2_lnk_retry_limit = retry_cnt;
    } else {
        pUsbdCtxt->handleRxFailure = false ;
        pUsbdCtxt->gen2_lnk_retry_limit = 0;
    }
}

/*******************************************************************************
* Function name: Cy_USBD_HandleResetDone
****************************************************************************//**
*
*  This function handles reset-done message.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleResetDone (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("ResetDone: pUsbdCtxt NULL\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DEFAULT;

    if (pUsbdCtxt->busResetDoneCb) {
        /* Inform about Speed */
        pUsbdCtxt->busResetDoneCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleCtrlXfrSetupStage
****************************************************************************//**
*
*  This function handles setup stage of control transfer. It also prepare for
*  data/ack stage based on request.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleCtrlXfrSetupStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                 cy_stc_usb_cal_msg_t *pMsg)
{
    uint32_t  setupData0;
    uint32_t  setupData1;
    uint8_t   bmRequest, bRequest;
    uint8_t   reqType;

    cy_stc_usb_setup_req_t setupReq;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    pUsbdCtxt->setupReqActive = true;

    setupData0 = pMsg->data[0];
    setupData1 = pMsg->data[1];

    /* Make sure EP0 stall condition is cleared before handling the request. */
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                  CY_USB_ENDP_DIR_IN, CY_USB_CLEAR);

    /* Decode the fields from the setup request. */
    pUsbdCtxt->setupReq.bmRequest = bmRequest = setupReq.bmRequest =
            (uint8_t)((setupData0 & CY_USB_BMREQUEST_SETUP0_MASK) >>
                       CY_USB_BMREQUEST_SETUP0_POS);
    pUsbdCtxt->setupReq.bRequest = bRequest = setupReq.bRequest =
            (uint8_t)((setupData0 & CY_USB_BREQUEST_SETUP0_MASK) >>
                        CY_USB_BREQUEST_SETUP0_POS);

    pUsbdCtxt->setupReq.wValue = setupReq.wValue =
            (uint16_t)((setupData0 & CY_USB_WVALUE_SETUP0_MASK) >>
                        CY_USB_WVALUE_SETUP0_POS);

    pUsbdCtxt->setupReq.wIndex = setupReq.wIndex =
            (uint16_t)((setupData1 & CY_USB_WINDEX_SETUP1_MASK) >>
                        CY_USB_WINDEX_SETUP1_POS);
    pUsbdCtxt->setupReq.wLength = setupReq.wLength =
            (uint16_t)((setupData1 & CY_USB_WLENGTH_SETUP1_MASK) >>
                        CY_USB_WLENGTH_SETUP1_POS);

    /* reqType can be STD, CLASS, VENDOR or OTHERS */
    reqType = ((bmRequest & CY_USB_CTRL_REQ_TYPE_MASK) >>
                                                CY_USB_CTRL_REQ_TYPE_POS);

    if (CY_USB_CTRL_REQ_STD == reqType) {
        switch (bRequest) {

            case CY_USB_SC_GET_STATUS:
                retStatus = Cy_USBD_HandleGetStatus(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_CLEAR_FEATURE:
                retStatus = Cy_USBD_HandleClearFeature(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SET_FEATURE:
                retStatus = Cy_USBD_HandleSetFeature(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SET_ADDRESS:
                /*
                 * SET ADDRESS command handle by hw so control will not
                 * come here.
                 */
                break;

            case CY_USB_SC_GET_DESCRIPTOR:
                /* Due to bmRequest and bRequest control reached here */
                retStatus = Cy_USBD_HandleGetDscr(pUsbdCtxt, setupReq,pMsg);
                break;

            case CY_USB_SC_SET_DESCRIPTOR:
                break;

            case CY_USB_SC_GET_CONFIGURATION:
                retStatus = Cy_USBD_HandleGetConfiguration(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_SET_CONFIGURATION:
                /*
                 * TBD: Following things will be done here.
                 * state change to Configured, All endpoints will be
                 * initialized and enabdled. callback will be called
                 * and information will be passed to application.
                 */
                retStatus = Cy_USBD_HandleSetConfiguration(pUsbdCtxt,setupReq,
                                                           pMsg);
                break;

            case CY_USB_SC_GET_INTERFACE:
                retStatus = Cy_USBD_HandleGetInterface(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_SET_INTERFACE:
                retStatus = Cy_USBD_HandleSetInterface(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SYNC_FRAME:
                /* Just send acknowledge */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                break;

            case CY_USB_SC_SET_SEL:
                /* If callback is registered, pass the request to it.
                 * Otherwise, stall the request.
                 */
                if (pUsbdCtxt->setupCb) {
                    pUsbdCtxt->setupReq = setupReq;
                    pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
                } else {
                    retStatus =
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                                  CY_USB_ENDP_DIR_OUT, CY_USB_SET);
                }
                break;

            case CY_USB_SC_SET_ISOC_DELAY:
                /* Pass isocDelay to SSCAL context */
                pUsbdCtxt->pSsCalCtxt->isochDelay = setupReq.wValue;
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                break;

        }   /* endof switch (bRequest) */
    } else {
        /*
         * For non standard request, if setup callback is not registered
         * then send STALL.
         */
        if (pUsbdCtxt->setupCb) {
            pUsbdCtxt->setupReq = setupReq;
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                          CY_USB_ENDP_DIR_IN, CY_USB_SET);
        }
    }   /* end of ELSE for standatd request */
    return(retStatus);
}   /* end of function  */

/*******************************************************************************
* Function name: Cy_USBD_HandleStatusStage
****************************************************************************//**
*
*  This function handles status stage of control transfer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in all other failure case.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleStatusStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlStaStage:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
    pUsbdCtxt->setupReqActive = false;

    /* On USB 2.x connection, enable L1 entry if device has been configured and
     * application has not disabled LPM handling.
     *
     * On USB 3.x connection, LPM has to be explicitly enabled by application.
     */
    if ((pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) &&
        (pUsbdCtxt->lpmDisabled == false) &&
        (pUsbdCtxt->lpmEnablePending == false) &&
        (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED)) {
        Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, CY_USB_CLEAR);
    }

    if (pUsbdCtxt->statusStageComplCb) {
        pUsbdCtxt->statusStageComplCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_HandleSuspend
****************************************************************************//**
*
*  This function handles suspend functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSuspend (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    /* Keeping PLL enabled for HS & SS */
    bool keep_pll_enabled = true;

    DBG_USBD_TRACE("HandleSuspend >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlSuspend:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devState <= CY_USB_DEVICE_STATE_RESET) {
        /* Before reset comes, dont do anything. */
        DBG_USBD_INFO("Suspend came when DevState < RESET\r\n");
        return(CY_USBD_STATUS_SUCCESS);
    }

    /* Ignore back to back suspend interrupts. */
    if (pUsbdCtxt->devState != CY_USB_DEVICE_STATE_SUSPEND) {
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = CY_USB_DEVICE_STATE_SUSPEND;

        /* Now handle Suspend */
        DBG_USBD_INFO("L2-SUSPEND Entry\r\n");

        /* In case of Full-Speed connection, a delay is required. */
        if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
#if FREERTOS_ENABLE
            vTaskDelay(5U);
#else
            Cy_SysLib_Delay(5U);
#endif /* FREERTOS_ENABLE */
        }

        DBG_USBD_INFO("HSPhySuspend: PLL_EN=%d\r\n", keep_pll_enabled);
        Cy_USBHS_Cal_HsHandleL2SuspendEntry(pUsbdCtxt->pCalCtxt, keep_pll_enabled);

        if (pUsbdCtxt->suspendCb) {
            pUsbdCtxt->suspendCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }

        DBG_USBD_INFO("HandleSuspend <<\r\n");
    }

    return (CY_USBD_STATUS_SUCCESS);
}


/*******************************************************************************
* Function name: Cy_USBD_HandleResume
****************************************************************************//**
*
*  This function handles resume functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleResume (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                      cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_USBD_TRACE("Cy_USBD_HandleResume >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlResume:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) {
        pUsbdCtxt->devState = pUsbdCtxt->prevDevState;
        pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_SUSPEND;
        DBG_USBD_TRACE("prevState:0x%x devState:0x%x\r\n",
                pUsbdCtxt->prevDevState, pUsbdCtxt->devState);
    }

    if (pUsbdCtxt->resumeCb) {
        pUsbdCtxt->resumeCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }

    DBG_USBD_TRACE("Cy_USBD_HandleResume <<\r\n");
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleL1Sleep
****************************************************************************//**
*
*  This function handles L1-Sleep functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleL1Sleep (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlL1Sleep:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    /* Ignore back-to-back L1 interrupts. */
    if (pUsbdCtxt->devState != CY_USB_DEVICE_STATE_HS_L1) {
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = CY_USB_DEVICE_STATE_HS_L1;

        if (!pUsbdCtxt->lpmDisabled) {
            /* Block further entry into L1 state for now. */
            Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, true);
        }

        if (pUsbdCtxt->l1SleepCb) {
            pUsbdCtxt->l1SleepCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }

        if (pUsbdCtxt->setupReqActive) {
            /* Make sure that we initiate L1 exit if there is a pending control request. */
            Cy_USBD_GetUSBLinkActive(pUsbdCtxt);
        }
    }

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleL1Resume
****************************************************************************//**
*
*  This function handles L1-Resume functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleL1Resume (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_en_usb_device_state_t tempState;
    DBG_USBD_TRACE("HndlL1Resume >>\r\n");

    /* This function will be called for HS so no need to check speed. */
    if (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_HS_L1) {
        DBG_USBD_TRACE("IN HS_L1 STATE\r\n");
        tempState = pUsbdCtxt->prevDevState;
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = tempState;

        if (!pUsbdCtxt->lpmDisabled) {
#if FREERTOS_ENABLE
            /* Start a timer to re-enable L1 state entry after a delay. */
            if (xTimerStart(pUsbdCtxt->usbdTimerHandle, 1) == pdPASS) {
                pUsbdCtxt->lpmEnablePending = true;
            } else {
                Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, false);
            }
#else
            if (gCurrentTimerTick != 0) {
                /* Set the timer tick at which next LPM task should be run. */
                pUsbdCtxt->nextTaskTick = gCurrentTimerTick + 50UL;
                pUsbdCtxt->lpmEnablePending = true;
            } else {
                /* Timer does not seem to be running. Re-enable LPM immediately. */
                Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt, false);
            }
#endif /* FREERTOS_ENABLE */
        }

        if (pUsbdCtxt->l1ResumeCb) {
            pUsbdCtxt->l1ResumeCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
    } else {
        /* Not in HS_L1 state so dont act on resume */
        DBG_USBD_TRACE("NOT-IN HS_L1 STATE\r\n");
    }

    DBG_USBD_TRACE("HndlL1Resume <<\r\n");
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleZlp
****************************************************************************//**
*
*  This function handles Zero-Length-Packet. This functionality applicable to HS
*  controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
/*
 * Function: Cy_USBD_HandleZlp()
 * Description: Handle ZLP for an endpoint.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleZlp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlZlp:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }
    if (pUsbdCtxt->zlpCb) {
        pUsbdCtxt->zlpCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleDone
****************************************************************************//**
*
*  This function handles done interrupt for an endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleDone (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlDone:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->doneCb) {
        pUsbdCtxt->doneCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);

        /* Re-enable the DONE interrupt for this endpoint. */
        Cy_USBHS_Cal_UpdateEpIntrMask(pUsbdCtxt->pCalCtxt,
                                      pMsg->data[0], CY_USB_ENDP_DIR_OUT,
                                      USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk, true);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleRateChange
****************************************************************************//**
*
*  This function handles rate change. This functionality is applicable to USB3
*  controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleRateChange (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlRateChg:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    /* Update USB3 data rate based on data from the CAL. */
    pUsbdCtxt->devSpeed = (cy_en_usb_speed_t)pMsg->data[0];

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleSlp
****************************************************************************//**
*
*  This function handles short length packet functionality. This functionality is
*  applicable to HS controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSlp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HandleSlp:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    /* Handle SLP interrupt on EP0-OUT here itself. For other endpoints, pass on to the callback. */
    if ((pMsg->type == CY_USB_CAL_MSG_OUT_SLP) && (pMsg->data[0] == 0x00)) {
        /*
         * RecvEndp0Data function would have unmasked SLP interrupt on EP0 only
         * after configuring the DMA channel correctly. Just assert the trigger
         * output from the USB block to ensure that the data gets read out by
         * the DMA channel.
         */
        Cy_TrigMux_SwTrigger(CY_USBD_INGEP_OUT_TRIG, CY_TRIGGER_TWO_CYCLES);
    } else {
        if ((pUsbdCtxt->debugSlpCb) && ((pMsg->type == CY_USB_CAL_MSG_OUT_SLP)) && ((pMsg->data[0] == pUsbdCtxt->debugOutEp))) {
            pUsbdCtxt->debugSlpCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            if (pUsbdCtxt->slpCb) {
                pUsbdCtxt->slpCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
            }
        }
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_HandleSsDisconnect
****************************************************************************//**
*
*  This function handles disconnect event/message came from bottom layer.
*  Applicable to SS controller only.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleSsDisconnect (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    /* Complete SS device disconnection if required. */
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        Cy_USBD_DisconnectSsDevice(pUsbdCtxt);
    }

    if (pUsbdCtxt->DisconnectCb) {
        pUsbdCtxt->DisconnectCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USBD_HandleMsg
****************************************************************************//**
*
* This function handles all messages comes from bottom layer.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pMsg
* pointer to message sent by bottom layer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_HandleMsg (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    DBG_USBD_TRACE("Cy_USBD_HandleMsg >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("HndlMsg:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    DBG_USBD_TRACE("MsgType:%d\r\n", pMsg->type);

    switch (pMsg->type) {

        case CY_USB_CAL_MSG_SOF:
            /* Nothing for SoF message. Just print if required. */
            DBG_USBD_TRACE("CY_USB_CAL_MSG_SOF\r\n");
            break;

        case CY_USB_CAL_MSG_RESET:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_RESET\r\n");
            retStatus = Cy_USBD_HandleReset(pUsbdCtxt, pMsg);
            if (pUsbdCtxt->termDetectCount <= CY_USB_MAX_TERMINATION_DETECTION_COUNT) {
                /*
                 * During USB 2.0 reset, need to enable 3.x controller so
                 * termination can be detected. This activity needs to be
                 * repeated three times.
                 */
                if (Cy_USBSS_Cal_IsEnabled(pUsbdCtxt->pSsCalCtxt)) {
                    Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_HS_RESET_SKIP);
                } else {
#if FREERTOS_ENABLE
                    /* Start a timer to enable the USB3 link after a delay. */
                    if (!pUsbdCtxt->usb3ConnectTimerStarted) {
                        BaseType_t taskWoken;

                        pUsbdCtxt->usb3ConnectTimerStarted = true;
                        xTimerStartFromISR(pUsbdCtxt->usb3ConnectTimer, &taskWoken);
                    }
#else
                    /* Enable USB 3.x link. */
                    Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_SS_REEN);
                    Cy_USBD_ConnectSsDevice(pUsbdCtxt);
#endif /* FREERTOS_ENABLE */
                }
            } else {
                Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_SS_DISB);
            }
            break;

        case CY_USBSS_CAL_MSG_LNK_RESET:
            /* During Super speed connection only this message valid. */
            DBG_USBD_INFO("CY_USBSS_CAL_MSG_LNK_RESET\r\n");
            retStatus = Cy_USBD_HandleSsReset(pUsbdCtxt, pMsg);
            break;

        case CY_USBSS_CAL_MSG_LNK_RX_DETECT_ACTIVE:
            /* During SS connection only this message valid. */
            DBG_USBD_TRACE("RxDetectActive\r\n");
            break;

        case CY_USBSS_CAL_MSG_LNK_COMPLIANCE:
            DBG_USBD_TRACE("LinkCompliance\r\n");
            if (!(pUsbdCtxt->disableHsOnComplianceEntry)) {
                DBG_USBD_INFO("Moving from SS to HS\r\n");
                Cy_USBD_DisconnectSsDevice(pUsbdCtxt);
                Cy_USBD_ConnectHsDevice(pUsbdCtxt);
                pUsbdCtxt->termDetectCount++;
            } else {
                DBG_USBD_INFO("Handling LinkCompliance mode\r\n");
                Cy_USBSS_Cal_EnterLinkCompliance(pUsbdCtxt->pSsCalCtxt);
            }
            break;

        case CY_USBSS_CAL_MSG_LNK_SS_DISABLE:
            /* During SS connection only this message valid. */
            DBG_USBD_TRACE("SsLinkDisable\r\n");
            break;

        case CY_USBSS_CAL_MSG_LNK_CONNECT:
            /* During SS connection only this message valid. */
            DBG_USBD_TRACE("LinkConnect\r\n");

            /*
             * LNK-CONNECT message comes then termination is detected so
             * go ahead with SS device and disable/disconnect HS device.
             */
            Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_HS_DISB);
            Cy_USBD_DisconnectHsDevice(pUsbdCtxt);
            pUsbdCtxt->isHsEnabled = false;

            retStatus = Cy_USBD_HandleSsReset(pUsbdCtxt, pMsg);
            break;

        case CY_USBSS_CAL_MSG_LNK_DISCONNECT:
            /* During SS connection only this message valid. */
            DBG_USBD_TRACE("LinkDisconnect\r\n");

            /*
             * As per speed negotiation guidence, first SS need to be tried.
             * if ss device reach to SSDISABLE/SSDISCONNECT state then start
             * trying for HS negotiation by
             * 1. Disconnect SS device enable HS device.
             * 2. Increament termination detection count.
             * These two will leads to HS detect and eventually HS/FS reset.
             */
            Cy_USBD_AddEvtToLog(pUsbdCtxt, CY_USBD_EVT_SS_TO_HS);
            Cy_USBD_DisconnectSsDevice(pUsbdCtxt);
            if (!pUsbdCtxt->isHsEnabled) {
                Cy_USBD_ConnectHsDevice(pUsbdCtxt);
                pUsbdCtxt->isHsEnabled = true;
            }
            pUsbdCtxt->termDetectCount++;
            break;

        case CY_USB_CAL_MSG_HSGRANT:
            /* During HS connection only this message valid. */
            Cy_USBD_HandleHsGrant(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_RESET_DONE:
            /* During HS connection only this message valid. */
            DBG_USBD_INFO("CY_USB_CAL_MSG_RESET_DONE\r\n");
            Cy_USBD_HandleResetDone(pUsbdCtxt, pMsg);
            break;

        case CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE:
            DBG_USBD_TRACE("CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE\r\n");
            /* Trigger device re-connection where required. */
            if (
                    (pMsg->data[1] != 0) ||
                    (
                     (pUsbdCtxt->handleRxFailure == true) &&
                     (pUsbdCtxt->gen2_lnk_retry_cnt < pUsbdCtxt->gen2_lnk_retry_limit)
                    )
               ) {
                Cy_USBD_DisconnectSsDevice(pUsbdCtxt);
                Cy_USBD_EnableConnection(pUsbdCtxt, (cy_en_usb_speed_t)pMsg->data[0], false);

                if (pMsg->data[1] == 0) {
                    pUsbdCtxt->gen2_lnk_retry_cnt++;
                }
            }
            break;

            /* CASE fall through here is intentional. */
        case CY_USB_CAL_MSG_SUDAV:
        case CY_USB_CAL_MSG_PROT_SUTOK:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_SUDAV\r\n");
            /*
             * Control request came. Handle as per requirement.
             * 1. For fast enumeration standrard request will be handled here.
             * 2. For application-enumeration, trigger callback register by
             *    application.
             * 3. All Non standard request, trigger callback register by
             *    application.
             */
            retStatus = Cy_USBD_HandleCtrlXfrSetupStage(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_STATUS_STAGE:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_STATUS_STAGE\r\n");
            Cy_USBD_HandleStatusStage(pUsbdCtxt,pMsg);
            break;

        case CY_USB_CAL_MSG_SETADDR:
            DBG_USBD_INFO("CY_USB_CAL_MSG_SETADDR\r\n");
            pUsbdCtxt->gen2_lnk_retry_cnt = 0;
            pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
            pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ADDRESS;
            pUsbdCtxt->usbDeviceStat = 0x00;

            if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                Cy_USBSS_Cal_GetDevAddress(pUsbdCtxt->pSsCalCtxt,
                        &(pUsbdCtxt->devAddr));

#if FREERTOS_ENABLE
                /* Change the LPM re-enable timer duration to 50 ticks in USB 3.x operation. */
                xTimerChangePeriod(pUsbdCtxt->usbdTimerHandle, 50, 0);
#endif /* FREERTOS_ENABLE */
            } else {
                Cy_USBHS_Cal_GetDevAddress(pUsbdCtxt->pCalCtxt,
                        &(pUsbdCtxt->devAddr));

#if FREERTOS_ENABLE
                /* Change the LPM re-enable timer duration to 10 ticks in USB 2.x operation. */
                xTimerChangePeriod(pUsbdCtxt->usbdTimerHandle, 10, 0);
#endif /* FREERTOS_ENABLE */
            }

            /* Send callback to application layer so that the current USB connection type can be identified. */
            if (pUsbdCtxt->setAddrCb) {
                pUsbdCtxt->setAddrCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
            }
            DBG_USBD_INFO("DevAddr:%d\r\n",pUsbdCtxt->devAddr);
            break;

        case CY_USB_CAL_MSG_PROT_SETADDR_0:
            /* device should come to default state */
            DBG_USBD_TRACE("Msg:SetAddr0\r\n");
            break;

        case CY_USB_CAL_MSG_SUSP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_SUSP\r\n");
            /*
             * ISR takes care of PHY and controller related settings. use
             * this message to inform upper layer about SUSPEND.
             */
            Cy_USBD_HandleSuspend(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_L1_SLEEP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_L1_SLEEP\r\n");
            /*
             * ISR takes care of PHY and controller related settings. use
             * this message to inform upper layer about L1_SLEEP.
             */
            Cy_USBD_HandleL1Sleep(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_RESUME_START:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_RESUME_START\r\n");

            if (pUsbdCtxt->devState <= CY_USB_DEVICE_STATE_RESET) {
                /* Before reset comes, don't do anything. */
                DBG_USBD_INFO("RESUME CAME when DevState < RESET\r\n");
                break;
            }

            if (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) {
                Cy_USBHS_Cal_HsHandleL2Resume(pUsbdCtxt->pCalCtxt);
            } else {
                if (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_HS_L1) {
                    DBG_USBD_TRACE("HS_L1 so comeout from L1\r\n");
                    Cy_USBHS_Cal_HsHandleL1WakeupCommon(pUsbdCtxt->pCalCtxt);
                }
            }
            break;

        case CY_USB_CAL_MSG_RESUME_END:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_RESUME_END\r\n");
            Cy_USBD_HandleResume(pUsbdCtxt,pMsg);
            break;

        case CY_USB_CAL_MSG_L1_URESUME:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_L1_URESUME\r\n");
            Cy_USBD_HandleL1Resume(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_IN_ZLP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_IN_ZLP\r\n");
            Cy_USBD_HandleZlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_IN_SLP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_IN_SLP\r\n");
            Cy_USBD_HandleSlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_OUT_ZLP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_OUT_ZLP\r\n");
            Cy_USBD_HandleZlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_OUT_SLP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_OUT_SLP\r\n");
            Cy_USBD_HandleSlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_ERRLIMIT:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_ERRLIMIT\r\n");
            break;

        case CY_USB_CAL_MSG_PROT_HOST_ERR:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_HOST_ERR\r\n");
            break;

        case CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP\r\n");

            /* USB 3.x connection to be disabled here. */
            Cy_USBD_DisableConnection(pUsbdCtxt, false);

            /*
             * Ideally, USB 2.0 connection should have been tried at this point. Instead
             * use an application callback to cause USB connection to be retried at a later point.
             */
            if (pUsbdCtxt->DisconnectCb) {
                pUsbdCtxt->DisconnectCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
            }
            break;

        case CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG\r\n");

            /* USB 3.x connection to be disabled here. */
            Cy_USBD_DisableConnection(pUsbdCtxt, false);

            /*
             * Ideally, USB 2.0 connection should have been tried at this point. Instead
             * use an application callback to cause USB connection to be retried at a later point.
             */
            if (pUsbdCtxt->DisconnectCb) {
                pUsbdCtxt->DisconnectCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
            }
            break;

        case CY_USB_CAL_MSG_PROT_TIMEOUT_PING:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_TIMEOUT_PING\r\n");
            break;

        case CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CAP:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CAP\r\n");
            break;

        case CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CFG:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CFG\r\n");
            break;

        case CY_USBSS_CAL_MSG_EPM_UNDERRUN:
            if (pUsbdCtxt->underrunWarningDone == false) {
                DBG_USBD_WARN("USBSS: Egress EP_UNDERRUN interrupt received\r\n");
                pUsbdCtxt->underrunWarningDone = true;
            }
            break;

        case CY_USB_CAL_MSG_OUT_DONE:
            DBG_USBD_TRACE("CY_USB_CAL_MSG_OUT_DONE\r\n");
            Cy_USBD_HandleDone(pUsbdCtxt, pMsg);
            break;

        case CY_USBSS_CAL_MSG_USB3_RATE_CHG:
            DBG_USBD_TRACE("CY_USBSS_CAL_MSG_USB3_RATE_CHG\r\n");
            Cy_USBD_HandleRateChange(pUsbdCtxt, pMsg);
            break;

        case CY_USBSS_CAL_MSG_UX_REENABLE:
            /* Don't do anything if LPM disable has been requested by user already. */
            if (pUsbdCtxt->lpmDisabled == false) {
#if FREERTOS_ENABLE
                /* Set a flag to indicate Ux re-enable is pending and start a timer. */
                if (xTimerStart(pUsbdCtxt->usbdTimerHandle, 1) == pdPASS) {
                    pUsbdCtxt->lpmEnablePending = true;
                } else {
                    Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, true);
                }
#else
                if (gCurrentTimerTick != 0) {
                    /* Set the timer tick at which next LPM task should be run. */
                    pUsbdCtxt->nextTaskTick = gCurrentTimerTick + 50UL;
                    pUsbdCtxt->lpmEnablePending = true;
                } else {
                    Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, true);
                }
#endif /* FREERTOS_ENABLE */
            }
            break;

        case CY_USBSS_CAL_MSG_ABORT_UX_ENABLE:
            if (pUsbdCtxt->lpmEnablePending) {
                pUsbdCtxt->lpmEnablePending = false;
#if FREERTOS_ENABLE
                xTimerStop(pUsbdCtxt->usbdTimerHandle, 0);
#else
                pUsbdCtxt->nextTaskTick = 0;
#endif /* FREERTOS_ENABLE */
            }
            break;

        case CY_USBSS_CAL_MSG_PORT_CONFIGURED:
            /* Clear tDisabledCount once Port Config handshake is complete. */
            pUsbdCtxt->termDetectCount = 0;
            break;

        case CY_USBSS_CAL_MSG_LPBK_FORCED:
#if FREERTOS_ENABLE
            if (!pUsbdCtxt->lpbkStateTimerStarted) {
                BaseType_t taskWoken;

                /* Start a timer to release the LTSSM force after 50 ms. */
                if (xTimerStartFromISR(pUsbdCtxt->usbdTimerHandle, &taskWoken) == pdPASS) {
                    pUsbdCtxt->lpbkStateTimerStarted = true;
                } else {
                    /* Release the LTSSM immediately. */
                    Cy_USBSS_Cal_ReleaseLTSSM(pUsbdCtxt->pSsCalCtxt);
                }
            }
#else
            /* Release the LTSSM immediately. */
            Cy_USBSS_Cal_ReleaseLTSSM(pUsbdCtxt->pSsCalCtxt);
#endif /* FREERTOS_ENABLE */
            break;

        default:
            DBG_USBD_ERR("HndlMsg:default(%x)\r\n", pMsg->type);
            break;
    }
    DBG_USBD_TRACE("Cy_USBD_HandleMsg <<\r\n");
    return(retStatus);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_InitUsbDscrPtrs
****************************************************************************//**
*
* This function initializes all descriptor pointers to NULL.
*
* \param pDscr
* pointer to data structure where all descriptors are stored.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_InitUsbDscrPtrs (cy_stc_usb_set_dscr_ptrs_t *pDscr)
{
    uint32_t dscrIndex = 0x00;
    pDscr->pUsbDevDscr = NULL;
    pDscr->pUsbSsDevDscr = NULL;
    pDscr->pUsbDevQualDscr = NULL;
    pDscr->pUsbCfgDscr = NULL;
    pDscr->pUsbOtherSpeedCfgDscr = NULL;
    pDscr->pUsbFsCfgDscr = NULL;
    pDscr->pUsbHsCfgDscr = NULL;
    pDscr->pUsbSsCfgDscr = NULL;
    for (dscrIndex = 0x00; dscrIndex <CY_USBD_MAX_STR_DSCR_INDX; dscrIndex++) {
        pDscr->pUsbStringDescr[dscrIndex] = NULL;
    }
    pDscr->pUsbHsBosDscr = NULL;
    pDscr->pUsbSsBosDscr = NULL;
    return;
}   /* End of function() */


/*******************************************************************************
* Function name: Cy_USBD_ConnectHsDevice
****************************************************************************//**
*
* This functions connects HS device to BUS and make it visible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_ConnectHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    DBG_USBD_INFO("ConnectHsDevice >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("ConnectHsDevice:usbdCtxt Null\r\n");
        return;
    }

    /*
     * Device always starts with Full speed and it will move to HS after
     * then HSGRANT interrupt.
     */
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->startHsDevSpeed = CY_USBD_USB_DEV_FS;

    /* Keep L1 and Ux entry disabled at startup. */
    Cy_USB_LpmSetClearNYET(pUsbdCtxt, true);

    /* clear all dev control interrupt and enable required interrupt */
    Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_EnableReqDevCtrlIntr(pUsbdCtxt->pCalCtxt);

    /* Enable SLP interrupt for endpoint 0x00 */
    Cy_USBHS_Cal_EnableCtrlSlpIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_ConnUsbPins(pUsbdCtxt->pCalCtxt);

    DBG_USBD_INFO("ConnectHsDevice <<\r\n");
    return;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_ConnectSsDevice
****************************************************************************//**
*
* This functions connects SS device to BUS and make it visible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_ConnectSsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usb_speed_t usbSpeed;

    DBG_USBD_TRACE("Cy_USBD_ConnectSsDevice >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("ConnectSsDevice::usbdCtxt Null\r\n");
        return;
    }

    usbSpeed = pUsbdCtxt->startSsDevSpeed;
    pUsbdCtxt->underrunWarningDone = false;

    /* Enable EP0 endpoints. */
    Cy_USBD_EnableEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_OUT, true);
    Cy_USBD_EnableEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN, true);

    /* Make sure the interrupts for link state changes are enabled when event logging is required. */
    Cy_USBSS_Cal_InitEventLog(pUsbdCtxt->pSsCalCtxt, pUsbdCtxt->pUsbEvtBuf, pUsbdCtxt->usbEvtLogSize);

    /* During disconnect, main interrupt was disabled so enable now. */
    Cy_USBSS_Cal_EnableLinkIntr(pUsbdCtxt->pSsCalCtxt, true);
    Cy_USBSS_Cal_EnableProtIntr(pUsbdCtxt->pSsCalCtxt, true);
    Cy_USBSS_Cal_EnableMainIntr(pUsbdCtxt->pSsCalCtxt, true);
    Cy_USBSS_Cal_Connect(pUsbdCtxt->pSsCalCtxt, usbSpeed);

    DBG_USBD_TRACE("Cy_USBD_ConnectSsDevice <<\r\n");
    return;
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USBD_EnableConnection
****************************************************************************//**
*
* This functions connects USB device with given speed and make it visible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param usbSpeed
* usb device speed which need to be configured in controller HW.
*
* \param isApiCall
* Whether this is due to a direct public API call.
*******************************************************************************/
static void
Cy_USBD_EnableConnection (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_en_usb_speed_t usbSpeed,
                          bool isApiCall)
{
    /* Make sure NAKALL bit is cleared at the beginning of a new connection. */
    Cy_USB_USBD_EndpSetClearNakNrdyAll(pUsbdCtxt, false);

    /*
     * This API will be called by main/application to connect
     * device with certain speed. Final speed will be decided
     * after negotiation. It is important to store information
     * about initial speed with which system want to start
     * negotiation.
     */
    pUsbdCtxt->devSpeed = usbSpeed;
#if FREERTOS_ENABLE
    pUsbdCtxt->usb3ConnectTimerStarted = false;
#endif /* FREERTOS_ENABLE */

    DBG_USBD_INFO("USBD_CONN:dev speed:%d\r\n", usbSpeed);
    if (usbSpeed > CY_USBD_USB_DEV_HS) {
        pUsbdCtxt->startSsDevSpeed = usbSpeed;
        if (isApiCall) {
            pUsbdCtxt->termDetectCount = 0x00;
        }

        Cy_USBD_ConnectSsDevice(pUsbdCtxt);
    } else {
        /* Make sure SS connection is not attempted on bus reset. */
        pUsbdCtxt->termDetectCount = CY_USB_MAX_TERMINATION_DETECTION_COUNT + 1;

        /* Apply the device connection speed chosen by the user. */
        Cy_USBHS_Cal_SetControllerSpeed(pUsbdCtxt->pCalCtxt, usbSpeed);
        pUsbdCtxt->startHsDevSpeed = usbSpeed;

        /* Enable connection. */
        Cy_USBD_ConnectHsDevice(pUsbdCtxt);
    }
}

/*******************************************************************************
* Function name: Cy_USBD_ConnectDevice
****************************************************************************//**
*
* This functions connects USB device with given speed and make it visible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param usbSpeed
* usb device speed which need to be configured in controller HW.
*
*******************************************************************************/
void
Cy_USBD_ConnectDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_en_usb_speed_t usbSpeed)
{
    DBG_USBD_TRACE("Cy_USBD_ConnectDevice >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("ConnectDevice:usbdCtxt Null\r\n");
        return;
    }

    Cy_USBD_EnableConnection(pUsbdCtxt, usbSpeed, true);
    DBG_USBD_TRACE("Cy_USBD_ConnectDevice <<\r\n");
    return;
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_ResetUsbdCommonDs
****************************************************************************//**
*
* This functions reset all USBD common data structure.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/

/* Any speed specific cleanup should have beed done before calling this function.*/
void
Cy_USBD_ResetUsbdCommonDs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    uint32_t intfNum;
    /*
     * This is not API. It is internal function where caller should have
     * already checked NULL ptr.
     * Common cleanup required at USBD layer during disconnect.
     */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_NOT_CONNECTED;

    /*
     * The default speed controller should come-up in.
     * startSsDevSpeed shall be set by user when the Cy_USBD_ConnectDevice API is called.
     */
    pUsbdCtxt->startHsDevSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->selfPowered = 0x00;
    pUsbdCtxt->remoteWakeupAbility = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;
    pUsbdCtxt->EnumerationDone = false;
    pUsbdCtxt->ep0SendDone = false;
    pUsbdCtxt->usbDeviceStat = 0x00;
    pUsbdCtxt->intfRemoteWakeEnabled = 0x00;
    pUsbdCtxt->numOfIntfInActiveCfg = 0x00;
    pUsbdCtxt->EnumerationDone = false;
    pUsbdCtxt->isHsEnabled = false;

    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
        pUsbdCtxt->numOfAltSettings[intfNum] = 0x00;
    }
    return;
}   /* End of function Cy_USBD_ResetUsbdCommonDs() */


/*******************************************************************************
* Function name: Cy_USBD_DisconnectHsDevice
****************************************************************************//**
*
* This functions dis-connects HS device from BUS and make it invisible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_DisconnectHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("DisConDev:usbdCtxt Null\r\n");
        return;
    }

    /* disable interrupts  and make device invisible on BUS */
    Cy_USBHS_Cal_DisableAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    /* disable Endp0 SLP interrupt. */
    Cy_USBHS_Cal_DisableCtrlSlpIntr(pUsbdCtxt->pCalCtxt);
    /* Remove pull-up on D+ to signal disconnect. */
    Cy_USBHS_Cal_DisconUsbPins(pUsbdCtxt->pCalCtxt);
    /* Since device is disconnected so Flush all endpoint */
    Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt);
    /* cleanup other USBD common data structures. */
    Cy_USBD_ResetUsbdCommonDs(pUsbdCtxt);
    return;
}   /* End of function. */

/*******************************************************************************
* Function name: Cy_USBD_DisconnectSsDevice
****************************************************************************//**
*
* This functions dis-connects SS device from BUS and make it invisible to host.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_DisconnectSsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    DBG_USBD_TRACE("DisconnectSsDevice  >>\r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("DisConDev:usbdCtxt Null\r\n");
        return;
    }

    /* Disable link interrupts first. */
    Cy_USBSS_Cal_EnableLinkIntr(pUsbdCtxt->pSsCalCtxt, false);

    /* Now disable the USB connection and finally disable other interrupts. */
    Cy_USBSS_Cal_DisConnect(pUsbdCtxt->pSsCalCtxt);
    Cy_USBSS_Cal_EnableProtIntr(pUsbdCtxt->pSsCalCtxt, false);
    Cy_USBSS_Cal_EnableMainIntr(pUsbdCtxt->pSsCalCtxt, false);

    /* Reset the DMA channels used for EP0 IN and OUT transfers. */
    Cy_HBDma_Channel_Reset(&(pUsbdCtxt->inEp0DmaUsb3Ch));
    Cy_HBDma_Channel_Reset(&(pUsbdCtxt->outEp0DmaUsb3Ch));

    Cy_USBSS_Cal_FlushAllEndpSocket(pUsbdCtxt->pSsCalCtxt);

    /* Flush and reset EP0. */
    Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, 0x00, CY_USB_ENDP_DIR_OUT);

    if (!pUsbdCtxt->isHsEnabled) {
        /* Disable EP0 endpoints. */
        Cy_USBD_EnableEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_OUT, false);
        Cy_USBD_EnableEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN, false);

        /* cleanup other USBD common data structures. */
        Cy_USBD_ResetUsbdCommonDs(pUsbdCtxt);
        Cy_USB_USBD_EndpInit(pUsbdCtxt);
    }

    DBG_USBD_TRACE("DisconnectSsDevice  <<\r\n");
    return;
}   /* End of function. */

/*******************************************************************************
* Function name: Cy_USBD_DisableConnection
****************************************************************************//**
*
* Disconnect device from BUS. This function should be called by module/layer
* who detects vbus insertion/removal.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param isApiCall
* Whether this is an external API call.
*
*******************************************************************************/
static void
Cy_USBD_DisableConnection (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           bool isApiCall)
{
    if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USBD_DisconnectHsDevice(pUsbdCtxt);
    } else {
        Cy_USBD_DisconnectSsDevice(pUsbdCtxt);
    }

#if FREERTOS_ENABLE
    /* Make sure that the USB3 connect timer is stopped. */
    xTimerStop(pUsbdCtxt->usb3ConnectTimer, 0);
    pUsbdCtxt->usb3ConnectTimerStarted = false;
#endif /* FREERTOS_ENABLE */

    if (isApiCall) {
        pUsbdCtxt->termDetectCount = 0x00;
    }
}

/*******************************************************************************
* Function name: Cy_USBD_DisconnectDevice
****************************************************************************//**
*
* Disconnect device from BUS. This function should be called by module/layer
* who detects vbus insertion/removal.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_DisconnectDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    DBG_USBD_TRACE("Cy_USBD_DisconnectDevice >>\r\n");
    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("DisConDev:usbdCtxt Null\r\n");
        return;
    }

    Cy_USBD_DisableConnection(pUsbdCtxt, true);
    return;
}   /* End of function. */


/*******************************************************************************
* Function name: Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig
****************************************************************************//**
*
*It initializes DMA descriptor for endpoint 0 IN transfer.
*
* \param pEndp0InCpuDmaDscrConfig
* dma descriptor configuration.
*
* \param first
* first or other descriptor.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig (cy_stc_usbd_dma_descr_conf_t
                                         *pEndp0InCpuDmaDscrConfig,
                                         bool first)
{
    if (first)
    {
        pEndp0InCpuDmaDscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        pEndp0InCpuDmaDscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0InCpuDmaDscrConfig->triggerOutType = CY_USBD_DMA_X_LOOP;
        pEndp0InCpuDmaDscrConfig->triggerInType = CY_USBD_DMA_X_LOOP;
        pEndp0InCpuDmaDscrConfig->channelState = CY_USBD_DMA_CHN_ENABLED;
        pEndp0InCpuDmaDscrConfig->dataPrefetch = false;
        pEndp0InCpuDmaDscrConfig->dataSize = CY_USBD_DMA_WORD;
        pEndp0InCpuDmaDscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0InCpuDmaDscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0InCpuDmaDscrConfig->descriptorType = CY_USBD_DMA_2D_XFER;
        pEndp0InCpuDmaDscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
        pEndp0InCpuDmaDscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
        pEndp0InCpuDmaDscrConfig->srcXincrement = 1; /* Source will be dscr pointer */
        pEndp0InCpuDmaDscrConfig->dstXincrement = 1; /* Address increment is needed for Egress SRAM as well. */
        pEndp0InCpuDmaDscrConfig->xCount = 1;     /* TBD: Need to update as size of descriptor */
        pEndp0InCpuDmaDscrConfig->srcYincrement = 0x10; /* Increment by 16 for 64 bytes. */
        pEndp0InCpuDmaDscrConfig->dstYincrement = 0x00; /* Do not increment for the next packet. */
        pEndp0InCpuDmaDscrConfig->yCount = 1;        /* TBD: To be updated based on transfer size. */
        pEndp0InCpuDmaDscrConfig->nextDescriptor = NULL;
    }
    else
    {
        pEndp0InCpuDmaDscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        pEndp0InCpuDmaDscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0InCpuDmaDscrConfig->triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0InCpuDmaDscrConfig->triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0InCpuDmaDscrConfig->channelState = CY_USBD_DMA_CHN_DISABLED;
        pEndp0InCpuDmaDscrConfig->dataPrefetch = false;
        pEndp0InCpuDmaDscrConfig->dataSize = CY_USBD_DMA_BYTE;
        pEndp0InCpuDmaDscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0InCpuDmaDscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0InCpuDmaDscrConfig->descriptorType = CY_USBD_DMA_1D_XFER;
        pEndp0InCpuDmaDscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
        pEndp0InCpuDmaDscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
        pEndp0InCpuDmaDscrConfig->srcXincrement = 1; /* Soruce will be dscr pointer */
        pEndp0InCpuDmaDscrConfig->dstXincrement = 1; /* Destination is FIFO so increamenet auto */
        pEndp0InCpuDmaDscrConfig->xCount = 1;     /* TBD: Need to update as size of descriptor */
        pEndp0InCpuDmaDscrConfig->srcYincrement = 0; /* Not required for 1D transfer. */
        pEndp0InCpuDmaDscrConfig->dstYincrement = 0; /* Not required for 1D tranfer. */
        pEndp0InCpuDmaDscrConfig->yCount = 0;        /* For 1D trasnfer we dont need this. */
        pEndp0InCpuDmaDscrConfig->nextDescriptor = NULL;
    }
}   /* End of function  */


/*******************************************************************************
* Function name: Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig
****************************************************************************//**
*
* It initializes DMA descriptor for EP0-OUT transfers.
*
* \param pEndp0InCpuDmaDscrConfig
* dma descriptor configuration.
*
* \param first
* first or other descriptor.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig (cy_stc_usbd_dma_descr_conf_t *pEndp0OutdscrConfig,
                                          bool first)
{
    if (first) {
        pEndp0OutdscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;  /* Wait for input trigger after each packet. */
        pEndp0OutdscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN; /* Assert interrupt at end of descriptor chain. */
        pEndp0OutdscrConfig->triggerOutType = CY_USBD_DMA_X_LOOP;     /* Trigger to be generated after each packet. */
        pEndp0OutdscrConfig->triggerInType = CY_USBD_DMA_X_LOOP;      /* Input trigger starts transfer of one packet. */
        pEndp0OutdscrConfig->channelState = CY_USBD_DMA_CHN_ENABLED;
        pEndp0OutdscrConfig->dataPrefetch = false;
        pEndp0OutdscrConfig->dataSize = CY_USBD_DMA_WORD;             /* Transfer 4 bytes on each AHB cycle. */
        pEndp0OutdscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0OutdscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0OutdscrConfig->descriptorType = CY_USBD_DMA_2D_XFER;
        pEndp0OutdscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
        pEndp0OutdscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
        pEndp0OutdscrConfig->srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
        pEndp0OutdscrConfig->dstXincrement = 1; /* Destination is RAM address */
        pEndp0OutdscrConfig->xCount = 0x10;     /* In each loop, transfer 0x10 * 4 = 0x40 bytes. */
        pEndp0OutdscrConfig->srcYincrement = 0;    /* Go back to EP base address for each packet. */
        pEndp0OutdscrConfig->dstYincrement = 0x10; /* Increment by 0x10 * 4 = 64 bytes after each packet. */
        pEndp0OutdscrConfig->yCount = 1;        /* TBD: Update based on number of packets to be transferred. */
        pEndp0OutdscrConfig->nextDescriptor = NULL;
    } else {
        pEndp0OutdscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        pEndp0OutdscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0OutdscrConfig->triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0OutdscrConfig->triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        pEndp0OutdscrConfig->channelState = CY_USBD_DMA_CHN_DISABLED;
        pEndp0OutdscrConfig->dataPrefetch = false;
        pEndp0OutdscrConfig->dataSize = CY_USBD_DMA_BYTE;
        pEndp0OutdscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0OutdscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        pEndp0OutdscrConfig->descriptorType = CY_USBD_DMA_1D_XFER;
        pEndp0OutdscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
        pEndp0OutdscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
        pEndp0OutdscrConfig->srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
        pEndp0OutdscrConfig->dstXincrement = 1; /* Destination is RAM address */
        pEndp0OutdscrConfig->xCount = 1;        /* TBD: Update based on actual packet size. */
        pEndp0OutdscrConfig->srcYincrement = 0; /* Not required for 1D transfer */
        pEndp0OutdscrConfig->dstYincrement = 0; /* Not required for 1D tranfer */
        pEndp0OutdscrConfig->yCount = 0;        /* For 1D transfer we dont need this */
        pEndp0OutdscrConfig->nextDescriptor = NULL;
    }
}   /* end of function. */


/*******************************************************************************
* Function name: Cy_USB_USBD_InitCpuDmaChannelCfg
****************************************************************************//**
*
* It initializes DMA channel config.
*
* \param pDmaChCfg
* dma channel configuration.
*
* \param pDmaDscr
* pointer to DMA descriptor.
*
* \return
* None.
*
*******************************************************************************/
/* Other three config are same. if require then it can be passed as parameter.*/
void
Cy_USB_USBD_InitCpuDmaChannelCfg (cy_stc_usbd_dma_chn_conf_t *pDmaChCfg,
                                  cy_stc_usbd_dma_descr_t *pDmaDscr)
{
    pDmaChCfg->descriptor = pDmaDscr;
    pDmaChCfg->priority = 1;
    pDmaChCfg->enable = false;
    pDmaChCfg->bufferable = false;
    return;
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USB_USBD_cpuDmaInit
****************************************************************************//**
*
* It initializes Cpu DMA functionality.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_cpuDmaInit (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_usbd_dma_descr_conf_t endp0InCpuDmaDscrConfig;
    cy_stc_usbd_dma_descr_conf_t endp0OutCpuDmaDscrConfig;

    cy_stc_usbd_dma_chn_conf_t cpuDmaCh0InConfig;
    cy_stc_usbd_dma_chn_conf_t cpuDmaCh1OutConfig;

    Cy_DMAC_Enable(pUsbdCtxt->pCpuDmacBase);

    /*
     * Prepare DMA channel for IN tranfer.  This includes prepare DMA dscr
     * config, then initialize dma descriptor, prepare dma channel config
     * and then initialize DMA channel.
     */
    Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig(&endp0InCpuDmaDscrConfig, true);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr0), &endp0InCpuDmaDscrConfig);
    Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig(&endp0InCpuDmaDscrConfig, false);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr1), &endp0InCpuDmaDscrConfig);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr2), &endp0InCpuDmaDscrConfig);

    Cy_USB_USBD_InitCpuDmaChannelCfg(&cpuDmaCh0InConfig,
                                        &(pUsbdCtxt->dmaCh0InXferDscr0));
    Cy_USBD_DMAChn_Init(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0, &cpuDmaCh0InConfig);

    /* Similar way prepare DMA for Out channel. */
    Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig(&endp0OutCpuDmaDscrConfig, true);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr0), &endp0OutCpuDmaDscrConfig);
    Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig(&endp0OutCpuDmaDscrConfig, false);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr1), &endp0OutCpuDmaDscrConfig);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr2), &endp0OutCpuDmaDscrConfig);

    Cy_USB_USBD_InitCpuDmaChannelCfg(&cpuDmaCh1OutConfig,
                                            &(pUsbdCtxt->dmaCh1OutXferDscr0));
    Cy_USBD_DMAChn_Init(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, &cpuDmaCh1OutConfig);

   /*
    * TBD: Later DMA interrupt can be registered from here but it is not
    * required as of now.
    */
    return CY_USBD_STATUS_SUCCESS;
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USB_USBD_EndpInit
****************************************************************************//**
*
* This function initializes endpoint 0 and other endpoints for HS and SS controller.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE in all other case.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpInit (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    uint32_t index=0x00;
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t  retCode = CY_USBD_STATUS_SUCCESS;

    DBG_USBD_TRACE("Cy_USB_USBD_EndpInit >> \r\n");

    /* Clear retry buffer used status. */
    for (index = 0; index < 8U; index++) {
        pUsbdCtxt->ssRetryBufStatus[index] = 0;
    }

    /*
     * Make sure endp_info and endp_config in sync.
     * Configure endpoint 0x00 first.
     * Disable all endpoints by writting valid = 0x00.
     * enable endpoint 0 during reset and others during set Config.
     */
    endpConfig.valid = 0x00;
    endpConfig.endpNumber = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_CTRL;
    endpConfig.maxPktSize = 64;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;
    pUsbdCtxt->endpInfoIn[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoIn[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoIn[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].retryBufOffset = 0xFFFFU;

    pUsbdCtxt->endpInfoOut[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoOut[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoOut[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].retryBufOffset = 0xFFFFU;

    /* for endpoint 0  IN/OUT taken care in single call. */
    calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /* Configure USB3 EP0 IN/OUT endpoint */
    endpConfig.maxPktSize = 512;
    calRetCode = Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /*
     * Configure all endpoint as bulk endpoint but make it invalid
     * by setting valid = 0x00. After setConfig these endpoints will
     * be configured properly.
     */
    endpConfig.valid = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_BULK;
    endpConfig.maxPktSize = 64;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;

    /* Configuring all IN endpoint and updating USBD data structure also. */
    for (index = 0x01; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        endpConfig.endpNumber = index;
        pUsbdCtxt->endpInfoIn[index].maxPktSize = endpConfig.maxPktSize;
        pUsbdCtxt->endpInfoIn[index].endpType = endpConfig.endpType;
        pUsbdCtxt->endpInfoIn[index].valid = endpConfig.valid;
        pUsbdCtxt->endpInfoIn[index].burstSize = endpConfig.burstSize;
        pUsbdCtxt->endpInfoIn[index].streamID = endpConfig.streamID;
        pUsbdCtxt->endpInfoIn[index].valid = endpConfig.valid;
        pUsbdCtxt->endpInfoIn[index].retryBufOffset = 0xFFFFU;

        calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
        if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
            retCode = CY_USBD_STATUS_SUCCESS;
        } else {
            DBG_USBD_ERR("Cy_USBHS_Cal_EndpConfig IN endpNum:%d Failed\r\n",index);
            retCode = CY_USBD_STATUS_FAILURE;
        }

        calRetCode = Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
        if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
            retCode = CY_USBD_STATUS_SUCCESS;
        } else {
            DBG_USBD_ERR("Cy_USBSS_Cal_EndpConfig IN endpNum:%d Failed\r\n",index);
            retCode = CY_USBD_STATUS_FAILURE;
        }
    }

    endpConfig.endpDirection = CY_USB_ENDP_DIR_OUT;
    for (index = 0x01; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        endpConfig.endpNumber = index;
        pUsbdCtxt->endpInfoOut[index].maxPktSize = endpConfig.maxPktSize;
        pUsbdCtxt->endpInfoOut[index].endpType = endpConfig.endpType;
        pUsbdCtxt->endpInfoOut[index].valid = endpConfig.valid;
        pUsbdCtxt->endpInfoOut[index].burstSize = endpConfig.burstSize;
        pUsbdCtxt->endpInfoOut[index].streamID = endpConfig.streamID;
        pUsbdCtxt->endpInfoOut[index].valid = endpConfig.valid;
        pUsbdCtxt->endpInfoOut[index].retryBufOffset = 0xFFFFU;

        calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
        if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
            retCode = CY_USBD_STATUS_SUCCESS;
        } else {
            DBG_USBD_ERR("Cy_USBHS_Cal_EndpConfig OUT endpNum:%d Failed\r\n",index);
            retCode = CY_USBD_STATUS_FAILURE;
        }

        calRetCode = Cy_USBSS_Cal_EndpConfig(pUsbdCtxt->pSsCalCtxt, endpConfig);
        if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
            retCode = CY_USBD_STATUS_SUCCESS;
        } else {
            DBG_USBD_ERR("Cy_USBSS_Cal_EndpConfig OUT endpNum:%d Failed\r\n",index);
            retCode = CY_USBD_STATUS_FAILURE;
        }
    }
    DBG_USBD_TRACE("Cy_USB_USBD_EndpInit << \r\n");
    return retCode;
}   /* end of function  */


/*******************************************************************************
* Function name: Cy_USB_USBD_DisableHsDevice
****************************************************************************//**
*
* This function disable all HS device interrupt and make device invisible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_DisableHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{

    DBG_USBD_TRACE("Cy_USB_USBD_DisableHsDevice >> \r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("DisableHsDevice:usbdCtxt Null\r\n");
        return;
    }
    /* Disconnect USB device from BUS, Clear and disable HS interrupt. */
    Cy_USBHS_Cal_DisconUsbPins(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_DisableAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    /* disable Endp0 SLP interrupt. */
    Cy_USBHS_Cal_DisableCtrlSlpIntr(pUsbdCtxt->pCalCtxt);

    DBG_USBD_TRACE("Cy_USB_USBD_DisableHsDevice << \r\n");
    return;
}    /*  End of function Cy_USB_USBD_DisableHsDevice() */


/*******************************************************************************
* Function name: Cy_USB_USBD_EnableHsDevice
****************************************************************************//**
*
* This function enable HS device and make it visible on BUS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_EnableHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    DBG_USBD_INFO("Cy_USB_USBD_EnableHsDevice >> \r\n");

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EnableHsDevice:usbdCtxt Null\r\n");
        return;
    }
    /*
     * Clear all interrupt, Enable required interrupt and
     * then make device visible on BUS.
     */
    Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_EnableReqDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    /* Enable SLP interrupt for endpoint 0x00 */
    Cy_USBHS_Cal_EnableCtrlSlpIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_ConnUsbPins(pUsbdCtxt->pCalCtxt);
    DBG_USBD_INFO("Cy_USB_USBD_EnableHsDevice << \r\n");
    return;
}    /*  End of function Cy_USB_USBD_EnableHsDevice() */


/*******************************************************************************
* Function name: Cy_USBD_CtrlHSEnableOnCompliance
****************************************************************************//**
*
* Function used to enable/disable switch to USB-HS (2.0) mode on entry into
* the USB 3.x Compliance LTSSM state. This transition is left enabled on
* start-up and can be enabled through this API when device configurations
* that do not support USB-HS are being used.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param hsEnable
*  Whether transition into USB-HS is enabled.
*
* \return
* None.
*
*******************************************************************************/
void Cy_USBD_CtrlHSEnableOnCompliance (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       bool hsEnable)
{
}

#if FREERTOS_ENABLE
/*******************************************************************************
* Function name: Cy_USBD_TaskHandler
****************************************************************************//**
*
* This function handles msg coming from CAL layer.
*
* \param pTaskParam
* void pointer given to task handler.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_TaskHandler (void *pTaskParam)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_usb_cal_msg_t queueMsg;
    BaseType_t  xStatus;
    BaseType_t  xHigherPriorityTaskWoken;
    UBaseType_t nMsgs;
    TickType_t  waitPeriod = 10;
    uint32_t lock;

    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pTaskParam;
    xStatus = xQueueReceive(pUsbdCtxt->usbdMsgQueue, &queueMsg, waitPeriod);

    do {
        lock = Cy_SysLib_EnterCriticalSection();
        if (xStatus == pdPASS) {
            Cy_SysLib_ExitCriticalSection(lock);
            Cy_USBD_HandleMsg(pUsbdCtxt, &queueMsg);

            /*
             * If another message is present in the queue when we return
             * from the handler, fetch it immediately and process it.
             */
            lock = Cy_SysLib_EnterCriticalSection();
            nMsgs = uxQueueMessagesWaiting(pUsbdCtxt->usbdMsgQueue);
            if (nMsgs != 0) {
                xStatus = xQueueReceiveFromISR(pUsbdCtxt->usbdMsgQueue,
                                               &queueMsg,
                                               &xHigherPriorityTaskWoken);
                Cy_SysLib_ExitCriticalSection(lock);
            } else {
                Cy_SysLib_ExitCriticalSection(lock);
                xStatus = xQueueReceive(pUsbdCtxt->usbdMsgQueue, &queueMsg,
                                        waitPeriod);
            }
        } else {
            Cy_SysLib_ExitCriticalSection(lock);
            xStatus = xQueueReceive(pUsbdCtxt->usbdMsgQueue, &queueMsg,
                                    waitPeriod);
        }
    } while (1);
}   /* end of function */

/*
 * Specifies whether each CAL message type is potentially of high frequency.
 * If yes, such messages can be silently dropped without a warning message.
 */
static const bool Cy_USBD_CalMsg_HighFreq[] = {
    true,               /* CY_USB_CAL_MSG_INVALID */
    false,              /* CY_USB_CAL_MSG_RESET */
    false,              /* CY_USB_CAL_MSG_HSGRANT */
    false,              /* CY_USB_CAL_MSG_RESET_DONE */
    false,              /* CY_USB_CAL_MSG_SUDAV */
    false,              /* CY_USB_CAL_MSG_STATUS_STAGE */
    false,              /* CY_USB_CAL_MSG_SETADDR */
    false,              /* CY_USB_CAL_MSG_SUSP */
    false,              /* CY_USB_CAL_MSG_RESUME_START */
    false,              /* CY_USB_CAL_MSG_RESUME_END */
    false,              /* CY_USB_CAL_MSG_DEEP_SLEEP_EXIT */
    false,              /* CY_USB_CAL_MSG_L1_SLEEP */
    false,              /* CY_USB_CAL_MSG_L1_URESUME */
    false,              /* CY_USB_CAL_MSG_IN_ZLP */
    false,              /* CY_USB_CAL_MSG_IN_SLP */
    false,              /* CY_USB_CAL_MSG_OUT_ZLP */
    false,              /* CY_USB_CAL_MSG_OUT_SLP */
    false,              /* CY_USB_CAL_MSG_OUT_DONE */
    true,               /* CY_USB_CAL_MSG_SOF */
    true,               /* CY_USB_CAL_MSG_ERRLIMIT */
    false,              /* CY_USBSS_CAL_MSG_LNK_RESET */
    false,              /* CY_USBSS_CAL_MSG_LNK_CONNECT */
    false,              /* CY_USBSS_CAL_MSG_LNK_DISCONNECT */
    false,              /* CY_USBSS_CAL_MSG_LNK_INT */
    true,               /* CY_USBSS_CAL_MSG_LNK_ERR_LIMIT */
    false,              /* CY_USBSS_CAL_MSG_LNK_RX_DETECT_ACTIVE */
    false,              /* CY_USBSS_CAL_MSG_LNK_SS_DISABLE */
    false,              /* CY_USBSS_CAL_MSG_LNK_COMPLIANCE */
    false,              /* CY_USBSS_CAL_MSG_USB3_LNKFAIL */
    false,              /* CY_USBSS_CAL_MSG_EP_INT */
    false,              /* CY_USBSS_CAL_MSG_PROT_INT */
    false,              /* CY_USB_CAL_MSG_PROT_SETADDR_0 */
    false,              /* CY_USB_CAL_MSG_PROT_SUTOK */
    false,              /* CY_USB_CAL_MSG_PROT_HOST_ERR */
    false,              /* CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP */
    false,              /* CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG */
    false,              /* CY_USB_CAL_MSG_PROT_TIMEOUT_PING */
    false,              /* CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CAP */
    false,              /* CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CFG */
    false,              /* CY_USBSS_CAL_MSG_PROT_EP_INT */
    false,              /* CY_USBSS_CAL_MSG_VBUS_CHANGE */
    false,              /* CY_USBSS_CAL_MSG_USB3_WARM_RESET */
    false,              /* CY_USBSS_CAL_MSG_USB3_U3_SUSPEND */
    false,              /* CY_USBSS_CAL_MSG_USB3_U0_RESUME */
    false,              /* CY_USBSS_CAL_MSG_UX_REENABLE */
    true,               /* CY_USBSS_CAL_MSG_EPM_UNDERRUN */
    false,              /* CY_USBSS_CAL_MSG_USB3_LMP_FAIL */
    false,              /* CY_USBSS_CAL_MSG_USB3_RATE_CHG */
    false,              /* CY_USBSS_CAL_MSG_ABORT_UX_ENABLE */
    false,              /* CY_USBSS_CAL_MSG_HANDLE_RXLOCK_FAILURE */
    false,              /* CY_USB_CAL_MSG_EP0_RCV_DONE */
    false,              /* CY_USBSS_CAL_MSG_PORT_CONFIGURED */
    false,              /* CY_USBSS_CAL_MSG_LPBK_FORCED */
    true                /* CY_USB_CAL_MSG_MAX */
};
#endif /* FREERTOS_ENABLE */

/*******************************************************************************
* Function name: Cy_USBD_ProcessMsg
****************************************************************************//**
*
* This function sends msg to USBD thread.
*
* \param pUsbd
* void pointer carry USBD context.
*
* \param pCalMgs
* message prepared by CAL layer.
*
* \return
* Whether context switch is required at the end of the ISR which generated the
* message.
*
*******************************************************************************/
bool
Cy_USBD_ProcessMsg (void *pUsbd, void *pCalMgs)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_usb_cal_msg_t *pMsg;
    bool retval = false;

#if FREERTOS_ENABLE
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif /* FREERTOS_ENABLE */

    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pUsbd;
    pMsg = (cy_stc_usb_cal_msg_t *)pCalMgs;

#if FREERTOS_ENABLE
    /*
     * In case of USBHS, DMA completion message directly goes to application
     * and ZLP message goes through CAL->USBD->App. This might leads to race
     * condition and DMA READ message reach early to application instead
     * ZLP even though ZLP event came first.
     * To avoid these race condition, need to send ZLP directly to application.
     * Same can be used for USBSS without side affect so no check of HS/SS.
     */
    if (pMsg->type == CY_USB_CAL_MSG_OUT_ZLP) {
        Cy_USBD_HandleZlp(pUsbdCtxt,pMsg);
        return retval;
    }

    /* For all other message follow ususal path ie msg to USBD layer. */
    status = xQueueSendFromISR(pUsbdCtxt->usbdMsgQueue,  pMsg,
                                            &(xHigherPriorityTaskWoken));
    if (status != pdPASS) {
        if (Cy_USBD_CalMsg_HighFreq[pMsg->type] == false) {
            DBG_USBD_ERR("UsbdMsgSndFail (%x)\r\n", pMsg->type);
        }
    } else {

        /*
         * If this is a control request, ensure that STALL on EP0 is cleared
         * immediately and that L1 entry is disallowed till request has been
         * handled.
         */
        if (pMsg->type == CY_USB_CAL_MSG_SUDAV) {
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, CY_USB_ENDP_0,
                                          CY_USB_ENDP_DIR_IN, CY_USB_CLEAR);
        }

        /* When a new control request is received, make sure that USB ingress/egress sockets
         * for EP0 are suspended if they are still active.
         */
        if (pMsg->type == CY_USB_CAL_MSG_PROT_SUTOK) {
            if (Cy_HBDma_Channel_GetChannelState(&(pUsbdCtxt->inEp0DmaUsb3Ch)) == CY_HBDMA_CHN_OVERRIDE) {
                Cy_HBDma_Channel_Reset(&(pUsbdCtxt->inEp0DmaUsb3Ch));
                Cy_USBSS_Cal_FlushEndpSocket(pUsbdCtxt->pSsCalCtxt, 0, CY_USB_ENDP_DIR_IN);
                Cy_USBSS_Cal_FlushEPM(pUsbdCtxt->pSsCalCtxt, true);
            }
            if (Cy_HBDma_Channel_GetChannelState(&(pUsbdCtxt->outEp0DmaUsb3Ch)) == CY_HBDMA_CHN_OVERRIDE) {
                Cy_USB_USBD_RetireRecvEndp0DataSs(pUsbdCtxt);
            }
        }

        retval = xHigherPriorityTaskWoken;
    }
#else
    if (pMsg->type == CY_USB_CAL_MSG_PROT_SUTOK) {
        if (Cy_HBDma_Channel_GetChannelState(&(pUsbdCtxt->inEp0DmaUsb3Ch)) == CY_HBDMA_CHN_OVERRIDE) {
            Cy_HBDma_Channel_Reset(&(pUsbdCtxt->inEp0DmaUsb3Ch));
            Cy_USBSS_Cal_EndpReset(pUsbdCtxt->pSsCalCtxt, 0, CY_USB_ENDP_DIR_IN);
            Cy_USBSS_Cal_FlushEPM(pUsbdCtxt->pSsCalCtxt, true);
        }
        if (Cy_HBDma_Channel_GetChannelState(&(pUsbdCtxt->outEp0DmaUsb3Ch)) == CY_HBDMA_CHN_OVERRIDE) {
            Cy_USB_USBD_RetireRecvEndp0DataSs(pUsbdCtxt);
        }
    }

    /* Directly call the handle message function. */
    Cy_USBD_HandleMsg(pUsbdCtxt, pMsg);
#endif /* FREERTOS_ENABLE */

    return retval;
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USBD_SendEgressZLP
****************************************************************************//**
*
* This function sends ZLP for given endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_SendEgressZLP (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum)
{
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SntEgrZlp:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }


    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetCode =
            Cy_USBHS_Cal_SendEgressZLP(pUsbdCtxt->pCalCtxt, endpNum);
    } else {
        /* TBD: Yet to implement. */
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}

/*******************************************************************************
* Function name: Cy_USBD_ClearZlpSlpIntrEnableMask
****************************************************************************//**
*
* This function clears endpoint interrupt for ingrress and egresss and enables
* respective mask register.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param zlpSlp
* true for zlp and false for slp.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_ClearZlpSlpIntrEnableMask (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                   uint32_t endpNum,
                                   cy_en_usb_endp_dir_t endpDir, bool zlpSlp)
{
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("ClrZlpSlpEnaMas:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetCode =
            Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask(pUsbdCtxt->pCalCtxt,
                                                   endpNum, endpDir, zlpSlp);
    } else {
        /* TBD: Yet to implement. */
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function */


/*******************************************************************************
* Function name: Cy_USB_LpmSetClearNYET
****************************************************************************//**
*
* This is a stack internal function used to enable/disable LPM support.
* Applications should always use Cy_USBD_LpmEnable and Cy_USBD_LpmDisable
* for this purpose.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param setClear
* true for set and false for clear.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if status update failed.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_LpmSetClearNYET (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    /* Call CAL function to disable/enable LPM without affecting status variable. */
    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {
        calRetStatus = Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt,
                                                    setClear);
    } else {
        if (setClear) {
            calRetStatus = Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
        } else {
            calRetStatus = Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, false);
        }
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBD_GetUSBLinkActive
****************************************************************************//**
*
* This function ensures that the USB link is brought into the L0 (USB2) or
* U0 (USB3) state when the respective low power states.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_GetUSBLinkActive (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_SUCCESS;

    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        calRetStatus = Cy_USBSS_Cal_SetLinkPowerState(pUsbdCtxt->pSsCalCtxt,
                                                      CY_USBSS_LPM_U0);
    } else {
        /*
         * Send remote wake signal to get the link back into L0.
         */
        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_FS) {
            /* If Link in L2 or L1, then only activate link. */
            if (
                    (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_HS_L1) ||
                    (pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) ||
                    (!Cy_USBHS_Cal_IsLinkActive(pUsbdCtxt->pCalCtxt))
               ) {
               calRetStatus = Cy_USBHS_Cal_GetLinkActive(pUsbdCtxt->pCalCtxt);
            }
        }
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function   */


/*******************************************************************************
* Function name: Cy_USBD_LpmDisable
****************************************************************************//**
*
*  This function disables all further transitions into a USB low power
*  (U1, U2, or L1)  state.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_LpmDisable (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_FAILURE;

    if (pUsbdCtxt == NULL) {
        DBG_USBD_ERR("LpmDisable:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        calRetStatus = Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);
    } else {
        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_FS) {
            calRetStatus = Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt,
                                                        CY_USB_SET);
        }
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        pUsbdCtxt->lpmDisabled = true;
        pUsbdCtxt->lpmEnablePending = false;
#if FREERTOS_ENABLE
        xTimerStop(pUsbdCtxt->usbdTimerHandle, 0);
#else
        pUsbdCtxt->nextTaskTick = 0;
#endif /* FREERTOS_ENABLE */
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* end of function */

/*******************************************************************************
* Function name: Cy_USBD_LpmEnable
****************************************************************************//**
*
* This function re-enables state based transitions into USB low power
* (U1, U2 and L1) modes.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_LpmEnable (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_FAILURE;

    if (pUsbdCtxt == NULL) {
        DBG_USBD_ERR("LpmEnable:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        calRetStatus = Cy_USBSS_Cal_LPMEnable(pUsbdCtxt->pSsCalCtxt, false);
    } else {
        if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_FS) {
            calRetStatus = Cy_USBHS_Cal_LpmSetClearNYET(pUsbdCtxt->pCalCtxt,
                                                        CY_USB_CLEAR);
        }
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        pUsbdCtxt->lpmDisabled = false;
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}


/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0DataSs
****************************************************************************//**
*
*  This function will disable dma channel for SS which was submitted to recieve
*  data.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_RetireRecvEndp0DataSs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    /* Reset the DMA channel and flush the EPM. */
    Cy_HBDma_Channel_Reset(&(pUsbdCtxt->outEp0DmaUsb3Ch));
    Cy_USBD_FlushEndp(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_OUT);
    pUsbdCtxt->pEp0ReceiveBuffer = NULL;
    pUsbdCtxt->ep0ExpRcvSize = 0;
    return;
}


/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0DataHs
****************************************************************************//**
*
*  This function will disable dma channel for HS which was submitted to recieve
*  data.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USB_USBD_RetireRecvEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    /* Disable the DMA channel. */
    Cy_USBD_DMAChn_Disable(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt),
                           pUsbdCtxt->channel1);

    /* Flush any data in the EPM buffer. */
    Cy_USBD_FlushEndp(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_OUT);

    /* Clear pending DMA interrupts and status variables. */
    Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, CY_DMAC_INTR_MASK);
    pUsbdCtxt->pEp0ReceiveBuffer = NULL;
    pUsbdCtxt->ep0ExpRcvSize = 0;
    return;
}   /* End of function */

/*******************************************************************************
* Function name: Cy_USB_USBD_RetireRecvEndp0Data
****************************************************************************//**
*
*  This function will disable dma channel which was submitted to recieve data.
*  Common function for HS and SS.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_RetireRecvEndp0Data (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("RetireRecvEndp0DataHs:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USB_USBD_RetireRecvEndp0DataHs(pUsbdCtxt);
    } else {
        Cy_USB_USBD_RetireRecvEndp0DataSs(pUsbdCtxt);
    }
    return(CY_USBD_STATUS_SUCCESS);
}   /* End of function */


/*******************************************************************************
* Function name: Cy_USBD_SendSSDeviceNotification
****************************************************************************//**
*
*  This function sends a Device Notification Transaction Packet to the host
*  controller on a USB 3.2 link.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param type
* Type of notification to be sent.
*
* \param param0
* 24-bits of data to be added in DWORD1 of the TP.
*
* \param param1
* Data to be added in DWORD2 of the TP.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE if function is called for FS/HS device.
* CY_USBD_STATUS_BAD_PARAM if type is not valid.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USBD_SendSSDeviceNotification (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_en_usbd_notification_type_t type,
                                  uint32_t param0, uint32_t param1)
{
    uint32_t tpData[3] = {0, 0, 0};

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("SendSSDeviceNotification:usbdCtxt Null\r\n");
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devSpeed < CY_USBD_USB_DEV_SS_GEN1) {
        DBG_USBD_ERR("SendSSDeviceNotification:invalidSpeed\r\n");
        return(CY_USBD_STATUS_FAILURE);
    }

    if ((type < CY_USBD_NOTIF_FUNC_WAKE) || (type > CY_USBD_NOTIF_BIAM)) {
        DBG_USBD_ERR("SendSSDeviceNotification:invalidType\r\n");
        return(CY_USBD_STATUS_BAD_PARAM);
    }

    /* Prepare and send the notification packet. */
    tpData[0] = CY_USBSS_PKT_TYPE_TP;
    tpData[1] = CY_USBSS_TP_SUBTYPE_NOTICE | ((uint32_t)type << 4U) | ((param0 & 0xFFFFFFUL) << 8U);
    tpData[2] = param1;
    Cy_USBSS_Cal_ProtSendTp(pUsbdCtxt->pSsCalCtxt, tpData);

    return(CY_USBD_STATUS_SUCCESS);
}   /* End of function */

/*******************************************************************************
 * Function Name: Cy_USBD_TickIncrement
 ****************************************************************************//**
 *
 * Increment the timer tick variable used to timestamp the USB event logs.
 * Should be called from SysTick interrupt handler.
 *
 * \param pUsbdCtxt
 * Pointer to USBD stack context structure.
 *
 * \return
 * None
 *******************************************************************************/
void Cy_USBD_TickIncrement (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    gCurrentTimerTick++;

#if (!FREERTOS_ENABLE)
    if ((pUsbdCtxt != NULL) && (gCurrentTimerTick >= pUsbdCtxt->nextTaskTick)) {
        pUsbdCtxt->nextTaskTick = 0;
        Cy_USBD_LpmEnableTask(pUsbdCtxt);
    }
#endif /* (!FREERTOS_ENABLE) */
}

/*******************************************************************************
 * Function Name: Cy_USBD_GetTimerTick
 ****************************************************************************//**
 *
 * Retrieve the current value of the timer tick.
 *
 * \return
 * Current value of timestamp variable.
 *******************************************************************************/
uint32_t Cy_USBD_GetTimerTick (void)
{
    return (gCurrentTimerTick);
}

/*******************************************************************************
 * Function Name: Cy_USBD_ResetTimerTick
 ****************************************************************************//**
 *
 * Clear the timer tick value maintained by the USB stack.
 *
 *******************************************************************************/
void Cy_USBD_ResetTimerTick (void)
{
    gCurrentTimerTick = 0;
}

/*******************************************************************************
 * Function Name: Cy_USBD_InitEventLog
 ****************************************************************************//**
 *
 * Enables capture of USB HS/SS driver events and state changes into RAM buffer.
 *
 * \param pUsbdCtxt
 * The pointer to the USBD context structure \ref cy_stc_usb_usbd_ctxt_t
 * allocated by the user.
 *
 * \param pEvtLogBuf
 * RAM buffer pointer to log the data into. Can be NULL if logging is to
 * be disabled.
 *
 * \param evtLogSize
 * Size of RAM buffer in 32-bit words.
 *
 * \return
 * The status code of the function execution \ref cy_en_usbd_ret_code_t.
 *******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_InitEventLog (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            uint32_t *pEvtLogBuf,
                                            uint16_t evtLogSize)
{
    if (pUsbdCtxt == NULL) {
        return CY_USBD_STATUS_CTXT_NULL;
    }

    if (((pEvtLogBuf != NULL) && (evtLogSize == 0)) || ((pEvtLogBuf == NULL) && (evtLogSize != 0))) {
        return CY_USBD_STATUS_BAD_PARAM;
    }

    pUsbdCtxt->pUsbEvtBuf    = pEvtLogBuf;
    pUsbdCtxt->usbEvtLogSize = evtLogSize;
    pUsbdCtxt->usbEvtLogIdx  = 0;

    /* Make sure the required LINK level interrupts are enabled. */
    if (pUsbdCtxt->pSsCalCtxt != NULL) {
        Cy_USBSS_Cal_InitEventLog(pUsbdCtxt->pSsCalCtxt, pEvtLogBuf, evtLogSize);
    }

    if (pEvtLogBuf != NULL) {
        /* Set the current event as none. */
        pUsbdCtxt->pUsbEvtBuf[0] = CY_SSCAL_EVT_NONE;
    }

    return CY_USBD_STATUS_SUCCESS;
}

/*******************************************************************************
 * Function Name: Cy_USBD_AddEvtToLog
 ****************************************************************************//**
 *
 * Add USB event/state to the RAM log buffer.
 *
 * \param pUsbdCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usb_usbd_ctxt_t.
 *
 * \param evtId
 * Event ID to be logged.
 *
 * \return
 * None
 *******************************************************************************/
void Cy_USBD_AddEvtToLog (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          uint32_t evtId)
{
    uint32_t intMask;

    if ((pUsbdCtxt != NULL) && (pUsbdCtxt->pUsbEvtBuf != NULL)) {
        intMask = Cy_SysLib_EnterCriticalSection();
        if (evtId & CY_EVT_LOG_DBG_MSG) {
            pUsbdCtxt->pUsbEvtBuf[pUsbdCtxt->usbEvtLogIdx] = evtId;
        } else {
            pUsbdCtxt->pUsbEvtBuf[pUsbdCtxt->usbEvtLogIdx] = ((gCurrentTimerTick << 8) | evtId);
        }

        pUsbdCtxt->usbEvtLogIdx++;
        if (pUsbdCtxt->usbEvtLogIdx >= pUsbdCtxt->usbEvtLogSize) {
            pUsbdCtxt->usbEvtLogIdx = 0;
        }

        pUsbdCtxt->pUsbEvtBuf[pUsbdCtxt->usbEvtLogIdx] = CY_SSCAL_EVT_NONE;
        Cy_SysLib_ExitCriticalSection(intMask);
    }
}

/*******************************************************************************
 * Function Name: Cy_USBD_GetEvtLogIndex
 ****************************************************************************//**
 *
 * Returns the current USB driver event log index.
 *
 * \param pUsbdCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usb_usbd_ctxt_t.
 *
 * \return
 * Current event log index.
 *******************************************************************************/
uint16_t Cy_USBD_GetEvtLogIndex (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if ((pUsbdCtxt != NULL) && (pUsbdCtxt->pUsbEvtBuf != NULL)) {
        return pUsbdCtxt->usbEvtLogIdx;
    }

    return 0;
}

/*******************************************************************************
* Function name: Cy_USBD_EP0OutDma_IntrHandler
****************************************************************************//**
*
* Handler for interrupt associated with EP0-OUT DMA transfer in USB 2.x modes.
* Should be called from the ISR registered for DMAC channel 1 which is used for
* EP0-OUT transfers.
*
* \param pUsbdCtxt
* USB stack context pointer.
*
* \return
* None.
*******************************************************************************/
void
Cy_USBD_EP0OutDma_IntrHandler (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_usb_cal_msg_t xMsg;
    uint32_t pendingCnt;
    uint32_t dmaIntr;

    if (pUsbdCtxt != NULL) {
        /* Clear pending interrupts on the DMA channel. */
        dmaIntr = Cy_USBD_DMAChn_GetIntrStatus(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1);
        Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, dmaIntr);

        /* Notify the application about the DMA transfer completion. */
        if ((pUsbdCtxt->ep0RecvCb != NULL) && (pUsbdCtxt->pEp0ReceiveBuffer != NULL)) {
            xMsg.type    = CY_USB_CAL_MSG_EP0_RCV_DONE;
            xMsg.data[0] = (uint32_t)pUsbdCtxt->pEp0ReceiveBuffer;
            xMsg.data[1] = pUsbdCtxt->ep0ExpRcvSize -
                Cy_USBHS_Cal_GetXferCount(pUsbdCtxt->pCalCtxt, 0, CY_USB_ENDP_DIR_OUT, &pendingCnt);
            pUsbdCtxt->ep0RecvCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, &xMsg);
        }

        /* Clear the EP0-OUT transfer status. */
        pUsbdCtxt->pEp0ReceiveBuffer = NULL;
        pUsbdCtxt->ep0ExpRcvSize = 0;
    }
}

/*******************************************************************************
 * Function Name: Cy_USBD_SetDmaClkFreq
 ****************************************************************************//**
 *
 * Function which sets the desired DMA clock frequency during USB 3.x operation
 * on the FX3G2 device. This function should be called before USB connection is
 * enabled. The default clock selected is 240 MHz derived from USB2 PLL.
 *
 * \param pUsbdCtxt
 * USB stack context pointer.
 *
 * \param dmaFreq
 * Desired DMA clock frequency.
 *
 * \return
 * Status code
 *******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SetDmaClkFreq (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_en_hbdma_clk_freq_t dmaFreq)
{
    cy_en_usb_cal_ret_code_t calStatus;
    cy_en_usbd_ret_code_t status = CY_USBD_STATUS_SUCCESS;

    if ((pUsbdCtxt == NULL) || (pUsbdCtxt->pSsCalCtxt == NULL)) {
        return CY_USBD_STATUS_CTXT_NULL;
    }

    if (dmaFreq > CY_HBDMA_CLK_SSPHY_CLK) {
        return CY_USBD_STATUS_BAD_PARAM;
    }

    calStatus = Cy_USBSS_Cal_SetDmaClkFreq(pUsbdCtxt->pSsCalCtxt, dmaFreq);
    if (calStatus != CY_USB_CAL_STATUS_SUCCESS) {
        status = CY_USBD_STATUS_FAILURE;
    }

    return status;
}

/*******************************************************************************
* Function Name: Cy_USBD_DisableLPMDeviceExit
****************************************************************************//**
*
* Function to disable support for device initiated exit from USB 3.x low power modes
* completely.
*
* \param pUsbdCtxt
* USB Stack context structure.
*
* \param devExitDisable
* true to disable device initiated exit from LPM states.
*******************************************************************************/
void
Cy_USBD_DisableLPMDeviceExit (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, bool devExitDisable)
{
    if ((pUsbdCtxt != NULL) && (pUsbdCtxt->pSsCalCtxt != NULL)) {
        Cy_USBSS_Cal_DisableLPMDeviceExit(pUsbdCtxt->pSsCalCtxt, devExitDisable);
    }
}

/*******************************************************************************
* Function Name: Cy_USBD_DisableSSOnBusReset
****************************************************************************//**
*
* Function which disables the USB 3.x reconnection when USB 2.x Bus Reset
* is received. This function is only provided for testing of link robustness
* and not expected to be used in an application.
*
* \param pUsbdCtxt
* USB Stack context structure.
*******************************************************************************/
void Cy_USBD_DisableSSOnBusReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (pUsbdCtxt != NULL) {
        DBG_USBD_INFO("SS connect on USB 2.x Bus Reset is disabled\r\n");
        pUsbdCtxt->ssOnBusResetDisable = true;
    }
}


/*******************************************************************************
* Function Name: Cy_USBD_SelectConfigLane
****************************************************************************//**
*
* Function to select the USB 3.x Configuration Lane. This API can be used in cases
* where the USB connection orientation is being detected externally instead of
* through CC1/CC2 voltage measurement by the controller itself.
*
* \param pUsbdCtxt
* USB Stack context structure.
* \param laneSel
* Desired configuration lane selection.
*
* \return
* Whether the orientation selection has been registered successfully.
*******************************************************************************/
cy_en_usbd_ret_code_t Cy_USBD_SelectConfigLane (
                                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                cy_en_usb_config_lane_t laneSel)
{
    if ((pUsbdCtxt == NULL) || (pUsbdCtxt->pSsCalCtxt == NULL)) {
        return CY_USBD_STATUS_CTXT_NULL;
    }

    if (laneSel > CY_USB_CFG_LANE_1) {
        return CY_USBD_STATUS_BAD_PARAM;
    }

    Cy_USBSS_Cal_SelectConfigLane(pUsbdCtxt->pSsCalCtxt, laneSel);
    return CY_USBD_STATUS_SUCCESS;
}

#if defined(__cplusplus)
}
#endif

/* End of file */

