/***************************************************************************//**
* \file cy_usbss_cal_drv.c
* \version 1.0
*
* Driver implementation for the USBSS (3.x) block in the FX10 device family.
* The driver functions are meant to be called by the USB device stack (USBD)
* and not expected to be called directly by user code.
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

#ifdef FX2G3_EN
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_debug.h"
#include "cy_gpio.h"

#if defined(__cplusplus)
extern "C" {
#endif


/* Compile out all TRACE statements. */
#undef DBG_SSCAL_TRACE
#define DBG_SSCAL_TRACE(str, ...)


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_MeasureCCVoltage
****************************************************************************//**
*
* Measure the voltage on CC line and return the corresponding ADC reading.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param cc2select
* true to measure CC2 voltage, false for CC1.
*
* \return
* ADC reading equivalent to the CC voltage.
*******************************************************************************/
uint8_t
Cy_USBSS_Cal_MeasureCCVoltage (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool cc2select)
{
    return (0);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetBusOrientation
****************************************************************************//**
*
* Identifies the USB-C connection orientation for the present
* connection.
*
* \param pCalCtxt
* USBSS Controller context structure.
*
* \return
* Index of SSPHY to be used for connection.
*******************************************************************************/
uint8_t
Cy_USBSS_Cal_GetBusOrientation (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return (0);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_InitSSPhy
****************************************************************************//**
*
* Initializes the specified USB40 PHY instance.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param phyIdx
* Index of the PHY instance to be initialized.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_InitSSPhy (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint8_t phyIdx)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Init
****************************************************************************//**
*
* Initializes the USBSS IP and relevant variables.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pUsbdCtxt
* The pointer to the USBD context structure.
*
* \param cb
* Callback function registered by the user \ref cy_usb_cal_msg_callback_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_Init (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                   void *pUsbdCtxt,
                   cy_usb_cal_msg_callback_t cb)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetGen2EBDepth
****************************************************************************//**
*
* Function to update the Elastic Buffer Half Depth setting to be used in Gen2
* USB connection.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param gen2_ebdepth
* Elastic Buffer Half Depth setting: Valid range from 4 to 16.
*
* \return
* void
*******************************************************************************/
void
Cy_USBSS_Cal_SetGen2EBDepth (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint8_t   gen2_ebdepth)
{

}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_InitEventLog
****************************************************************************//**
*
* Enables capture of USBSS CAL event and state changes into RAM buffer.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pEvtLogBuf
* RAM buffer pointer to log the data into. Can be NULL if logging is to
* be disabled.
*
* \param evtLogSize
* Size of RAM buffer in bytes.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_InitEventLog (
        cy_stc_usbss_cal_ctxt_t *pCalCtxt,
        uint32_t *pEvtLogBuf,
        uint16_t  evtLogSize)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_IsEnabled
****************************************************************************//**
*
* Indicates whether USB 3.x connection is enabled on the device.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* true if connection is enabled, false otherwise.
*
*******************************************************************************/
bool Cy_USBSS_Cal_IsEnabled (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return (false);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Connect
****************************************************************************//**
*
* Enables the USB3 RX terminations for connecting to the host and enables
* relevant USB LINK and PROTOCOL interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param usbSpeed
* The desired speed at which the device needs to be connected to the host
* \ref cy_en_usb_speed_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_Connect (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      cy_en_usb_speed_t usbSpeed)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SendMsg
****************************************************************************//**
*
* Sends message to the USBD layer using a callback registered by the user. This
* function should call only ISR safe routine.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pMsg
* Message structure pointer.
*
* \return
* Whether context switch is required at the end of the ISR.
*
*******************************************************************************/
bool
Cy_USBSS_Cal_SendMsg (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                      void *pMsg)
{
    /*
     * Here call back function provided by USBD layer will be called.
     * OS dependent message function should be implemented as part of
     * callback function.
     */
    return (pCalCtxt->msgCb(pCalCtxt->pUsbdCtxt, (void *)pMsg));
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_IntrHandler
****************************************************************************//**
*
* Interrupt Handler function for USB3 IP.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pMsg
* Message structure pointer.
*
* \return None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_IntrHandler (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_WakeupIntrHandler
****************************************************************************//**
*
* Wakeup Interrupt Handler function for USB3 IP.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*******************************************************************************/
void
Cy_USBSS_Cal_WakeupIntrHandler (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_Get_PtmStatus()
****************************************************************************//**
*
* Sends the PTM STATUS to the USBD layer to repsond to GET_STATUS request.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.

* \return uint32_t
* The PTM status .
*
*******************************************************************************/
uint32_t
Cy_USBSS_Cal_Get_PtmStatus (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return pCalCtxt->ptmStatus;
}


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_LPMEnable
****************************************************************************//**
*
* Enables the USB3 Link layer Low Power Mode Handling. The LINK layer can accept
* going into U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param isResume
* Whether delayed LPM entry is being resumed.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_LPMEnable (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool isResume)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ForceLPMAccept
****************************************************************************//**
*
* Enables the USB3 Link layer to always accept LPM requests going into
* U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* If true, the device accept LPM requests always.
* If false, the device may accept or reject LPM requests.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ForceLPMAccept (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool enable)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_LPMDisable
****************************************************************************//**
*
* Enables the USB3 Link layer to disable accepting LPM requests going into
* U1/U2 upon receiving LGO_U1/LGO_U2.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_LPMDisable (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetLinkPowerState
****************************************************************************//**
*
* Forces the USB3 Link layer to a particular Link Power State by sending
* LGO_Ux requests.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param lnkMode
* Link Mode that needs to be set from \ref cy_en_usbss_lnk_power_mode_t
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetLinkPowerState (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                cy_en_usbss_lnk_power_mode_t lnkMode)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetUsbLinkActive
****************************************************************************//**
*
* Get the USB3 Link layer to U0 State if it is in U1/U2 state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
void Cy_USBSS_Cal_GetUsbLinkActive (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetLinkPowerState
****************************************************************************//**
*
* Retrieves the Link Power State of the device.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param *pMode
* Pointer to a memory where the link power state will be stored
*  from \ref cy_en_usbss_lnk_power_mode_t
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetLinkPowerState (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                cy_en_usbss_lnk_power_mode_t *pMode)
{
    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendTp
****************************************************************************//**
*
* This function will send transaction packet. It is responsibility of caller to
* prepare transaction packet and call this function.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param *pTpData
* Pointer to a memory where transaction packet stored.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_ProtSendTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t *pTpData)
{

    return;
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendAckTp
****************************************************************************//**
*
* This function prepares and send ACK transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param numP
* Number of Data packet buffer receiever can accept.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendAckTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir, uint8_t numP,
                            uint16_t bulkStream)
{
   
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendErdyTp
****************************************************************************//**
*
* This function prepares and send ERDY transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param numP
* Number of Data packet buffer receiever can accept.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendErdyTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                             cy_en_usb_endp_dir_t endpDir, uint8_t numP,
                             uint16_t bulkStream)
{

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendZlpTp
****************************************************************************//**
*
* This function prepares and send ZLP transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
/* TBD: Need to add endp number as paramter so zlp can be sent for any endp.*/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendZlpTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ProtSendNrdyTp
****************************************************************************//**
*
* This function prepares and send NRDY transaction packet.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param bulkStream
* stream ID incase of bulk transfer
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ProtSendNrdyTp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                             cy_en_usb_endp_dir_t endpDir, uint16_t bulkStream)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetClearStall
****************************************************************************//**
*
* This function enable or disable STALL condition in hw.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetClearStall (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                uint32_t endpNum,
                                cy_en_usb_endp_dir_t endpDir,
                                bool setClear)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpReset
****************************************************************************//**
*
* Function to reset an endpoint. This clears sticky bits e.g retry bit,
* flowcontrol bit.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpReset (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of Function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetSeqNum
****************************************************************************//**
*
* This function gets the current value of sequence number for an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param pSeqNum
* Current value of sequence number will be store here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetSeqNum (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir,  uint8_t *pSeqNum)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetSeqNum
****************************************************************************//**
*
*  This function sets value of sequence number in hardware for an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param pSeqNum
* sequence number need to be written in HW register.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetSeqNum (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                        cy_en_usb_endp_dir_t endpDir,  uint8_t seqNum)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
} /* End of function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_GetDevAddress
****************************************************************************//**
*
*  This function will get device address assigned by Host.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param pDevAddr
* device address will be stored here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_GetDevAddress (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint8_t *pDevAddr)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function Cy_USBSS_Cal_GetDevAddress() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetClearNrdy
****************************************************************************//**
*
* This function enable or disable NRDY condition in hw. By setting NRDY bit,
* endpoint will keep sending NRDY till NRDY bit is cleared.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetClearNrdy (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNum,
                               cy_en_usb_endp_dir_t endpDir,
                               bool setClear)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function  */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetClearNRDYALL
****************************************************************************//**
*
* This function enable or disable NRDY condition for all endpopint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param setClear
* true for set and false for clear.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetClearNrdyAll (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool setClear)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpConfig
****************************************************************************//**
*
* This function handles configuration of endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param configParam
* Configuration Parameters stored here \ref cy_stc_usb_endp_config_t.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpConfig (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                            cy_stc_usb_endp_config_t configParam)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpSetPktsPerBuffer
****************************************************************************//**
*
* This function sets the number of maximum sized packets that can fit into a
* single DMA buffer for an Ingress endpoint. This needs to be called in cases
* where the maximum packet size is not a power of 2, so that the EPM can operate
* correctly.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param pktsPerBuffer
* packats per buffer.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpSetPktsPerBuffer (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                   uint32_t endpNum,
                                   uint8_t pktsPerBuffer)
{

    return (CY_USB_CAL_STATUS_SUCCESS);
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpMapStream
****************************************************************************//**
*
* This function maps an unused USB Ingress/Egress socket to the specified
* stream associated with a bulk endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param streamId
* stream ID.
*
* \param socketNum
* Socket Number.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpMapStream (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir, uint16_t streamId,
                            uint32_t socketNum)
{
    return (CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpUnmapStream
****************************************************************************//**
*
* This function unmaps the DMA socket to stream mapping for a bulk endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param socketNum
* Socket Number.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EndpUnmapStream (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                            cy_en_usb_endp_dir_t endpDir,
                            uint32_t socketNum)
{
    return (CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableEndp
****************************************************************************//**
*
* This function enables/disable endpoint and set/reset respective interrupt.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableEndp (cy_stc_usbss_cal_ctxt_t *pCalCtxt, uint32_t endpNum,
                         cy_en_usb_endp_dir_t endpDir,  bool enable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBSS_Cal_EnableEndp () */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetEpBurstMode
****************************************************************************//**
*
* This function enables/disables the MULT (allow burst across multiple DMA buffers)
* feature for the specified USB endpoints. Enabling the feature can improve the data
* transfer throughput on the respective endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param enable
* true for enable burst mode and false for disable burst mode.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetEpBurstMode (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                             bool enable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBSS_Cal_SetEpBurstMode () */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SetEndpRetryOffset
****************************************************************************//**
*
* This function sets the egress endpoint retry buffer offset for Type-2
* (CTRL and BULK) endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param retryBufOffset
* Retry buffer offset.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SetEndpRetryOffset (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                 uint32_t endpNum,
                                 uint16_t retryBufOffset)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
} /* End of function Cy_USBSS_Cal_SetEndpRetryOffset */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_UpdateXferCount
****************************************************************************//**
*
* This function updates xfer count in an ndpoint register.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \param xferCount
* transfer count value stored here.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_UpdateXferCount (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum,
                              cy_en_usb_endp_dir_t endpDir,
                              uint32_t xferCount)
{
    /* TBD: Update required registers. */
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableStatusCtrl
****************************************************************************//**
*
* This function enable/disable Status Control feature mentioned in PROT_CS
* register.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableStatusCtrl (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool enable)
{

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ClearStatusClrBusy
****************************************************************************//**
*
* This function write "1" to STATUS_CLR_BUSY which clears  the bit to initiate
* STATUS response.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_ClearStatusClrBusy (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SendAckSetupDataStatusStage
****************************************************************************//**
*
* This function enable HW to send ACK in status stage.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_SendAckSetupDataStatusStage (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushEPM
****************************************************************************//**
*
* This function will flush Ingress and egress both the EPM.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
* \param force
* Whether flush should be forcibly done even if pointers are zero.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushEPM (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool force)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushEndpSocket
****************************************************************************//**
*
* This function will flush socket associated with an endpoint.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushEndpSocket (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_FlushAllEndpSocket
****************************************************************************//**
*
* This function will flush socket associated with all endpoints.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_FlushAllEndpSocket (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableMainIntr
****************************************************************************//**
*
* This function will enable/disable main interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableMainIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableLinkIntr
****************************************************************************//**
*
* This function will enable/disable link interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableLinkIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableProtIntr
****************************************************************************//**
*
* This function will enable/disable protocol interrupts.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param EnableDisable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableProtIntr (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             bool EnableDisable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpIsNakNrdySet
****************************************************************************//**
*
* This function checks whether the specified endpoint is currently in the NRDYed
* state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* Whether endpoint is currently NRDYed. True means NRDYed state.
*
*******************************************************************************/
bool
Cy_USBSS_Cal_EndpIsNakNrdySet (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
    return true;
}   /* End of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EndpIsStallSet
****************************************************************************//**
*
* This function checks whether the specified endpoint is currently in the STALLed
* state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param endpNum
* Endpoint number.
*
* \param endpDir
* Endpoint direction.
*
* \return
* Whether endpoint is currently STALLed. True means STALLed state.
*
*******************************************************************************/
bool
Cy_USBSS_Cal_EndpIsStallSet (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                             uint32_t endpNum, cy_en_usb_endp_dir_t endpDir)
{
   
    return false;
}   /* End of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnableSsDevice
****************************************************************************//**
*
* This function enable/disable USB device in SS mode.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param enable
* true for enable and false for disable.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_EnableSsDevice (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool enable)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DisConnect
****************************************************************************//**
*
*  This function handles disconnect device at CAL layer.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_DisConnect (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function () */


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_PTMConfig
****************************************************************************//**
*
* This function handles PTM related configuration.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \param ptmControl
* Set to 1 for enabling LDM & PTM. Set to 0 for disabling LDM and reset PTM logic.
*
* \return
* The status code of the function execution \ref cy_en_usb_cal_ret_code_t.
*
*******************************************************************************/
/*
 * ptmControl must be set to 1 for receiving LDM RX interrupt and further
 * proceeding with PTM engine logic.
 */
cy_en_usb_cal_ret_code_t
Cy_USBSS_Cal_PTMConfig (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool ptmControl)
{

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_EnterLinkCompliance
****************************************************************************//**
*
* This function performs the actions required when the USB LTSSM enters the
* Compliance state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_EnterLinkCompliance (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_NextCompliancePattern
****************************************************************************//**
*
* This function causes the LTSSM to move to the next compliance pattern
* once a Ping.LFPS has been detected.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBSS_Cal_NextCompliancePattern (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ExitLinkCompliance
****************************************************************************//**
*
* This function performs the required actions when the LTSSM exits the Compliance
* state due to a Warm Reset.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*******************************************************************************/
void
Cy_USBSS_Cal_ExitLinkCompliance (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}   /* end of function */

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DeepSleepPrep
****************************************************************************//**
*
* This function prepares the USB32DEV controller IP block for entry into deep
* sleep state.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*
* \return
* true if application can proceed with deep sleep entry, false otherwise.
*******************************************************************************/
bool
Cy_USBSS_Cal_DeepSleepPrep (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
    return true;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DeepSleepExit
****************************************************************************//**
*
* This function restores the USB32DEV controller IP block state after USB
* link has resumed from U3 (suspend) state. This function is called from the
* driver internally and not intended to be called directly.
*
* \param pCalCtxt
* The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
* allocated by the user.
*******************************************************************************/
void
Cy_USBSS_Cal_DeepSleepExit (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
   
}

/*******************************************************************************
 * Function Name: Cy_USBSS_Cal_SetDmaClkFreq
 ****************************************************************************//**
 *
 * Function which sets the desired DMA clock frequency on the FX3G2 device.
 * This function should be called before USB connection is enabled. The default
 * clock selected is 240 MHz derived from USB2 PLL.
 *
 * \param pCalCtxt
 * The pointer to the USBSS context structure \ref cy_stc_usbss_cal_ctxt_t
 * allocated by the user.
 *
 * \param dmaFreq
 * Desired DMA clock frequency.
 *
 * \return
 * Status code
 *******************************************************************************/
cy_en_usb_cal_ret_code_t Cy_USBSS_Cal_SetDmaClkFreq (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                                     cy_en_hbdma_clk_freq_t dmaFreq)
{

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_DisableLPMDeviceExit
****************************************************************************//**
*
* Function to disable support for device initiated exit from USB low power modes
* completely.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param devExitDisable
* true to disable device initiated exit from LPM states.
*******************************************************************************/
void
Cy_USBSS_Cal_DisableLPMDeviceExit (cy_stc_usbss_cal_ctxt_t *pCalCtxt, bool devExitDisable)
{

}

/*******************************************************************************
* Function Name: Cy_USBSS_Cal_ReleaseLTSSM
****************************************************************************//**
*
* Function to release the LTSSM from a forced state.
*
* \param pCalCtxt
* USBSS Controller context structure.
*******************************************************************************/
void
Cy_USBSS_Cal_ReleaseLTSSM (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

}


/*******************************************************************************
* Function Name: Cy_USBSS_Cal_SelectConfigLane
****************************************************************************//**
*
* Function to select the USB 3.x Configuration Lane. This API can be used in cases
* where the USB connection orientation is being detected externally instead of
* through CC1/CC2 voltage measurement by the controller itself.
*
* \param pCalCtxt
* USBSS Controller context structure.
* \param laneSel
* Desired configuration lane selection.
*******************************************************************************/
void Cy_USBSS_Cal_SelectConfigLane (cy_stc_usbss_cal_ctxt_t *pCalCtxt,
                                    cy_en_usb_config_lane_t laneSel)
{

}

#if defined(__cplusplus)
}
#endif

#endif /* FX2G3_EN */

/* End of file */
