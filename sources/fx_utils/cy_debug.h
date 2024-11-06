/***************************************************************************//**
* \file cy_debug.h
* \version 1.0
*
* Provides macros and definitions associated with the FX debug logger module.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
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
 * FX Utils, part of the usbfxstack library is used for the full-speed and high-speed operation modes of the FX devices.
 * \defgroup group_usbfxstack_fx_utils_macros Macros
 * \defgroup group_usbfxstack_fx_utils_enums Enumerated Types
 * \defgroup group_usbfxstack_fx_utils_structs Data Structures
 * \defgroup group_usbfxstack_fx_utils_typedefs Type Definitions
 * \defgroup group_usbfxstack_fx_utils_functions Functions
 */

/**
 * \addtogroup group_usbfxstack_fx_utils_macros
 * \{
 */

#ifndef _CY_DEBUG_H_

/** Indicates the use of cy_debug */
#define _CY_DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef DOXYGEN

#if DEBUG_INFRA_EN
#define DBG_BLOAD_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:BLOAD:"str, ##__VA_ARGS__)
#define DBG_BLOAD_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:BLOAD:"str, ##__VA_ARGS__)
#define DBG_BLOAD_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:BLOAD:"str, ##__VA_ARGS__)
#define DBG_BLOAD_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:BLOAD:"str, ##__VA_ARGS__)
#else
#define DBG_BLOAD_ERR(str, ...)
#define DBG_BLOAD_WARN(str, ...)
#define DBG_BLOAD_INFO(str, ...)
#define DBG_BLOAD_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_HSCAL_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:HSCAL:"str, ##__VA_ARGS__)
#define DBG_HSCAL_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:HSCAL:"str, ##__VA_ARGS__)
#define DBG_HSCAL_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:HSCAL:"str, ##__VA_ARGS__)
#define DBG_HSCAL_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:HSCAL:"str, ##__VA_ARGS__)
#else
#define DBG_HSCAL_ERR(str, ...)
#define DBG_HSCAL_WARN(str, ...)
#define DBG_HSCAL_INFO(str, ...)
#define DBG_HSCAL_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_SSCAL_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:SSCAL:"str, ##__VA_ARGS__)
#define DBG_SSCAL_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:SSCAL:"str, ##__VA_ARGS__)
#define DBG_SSCAL_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:SSCAL:"str, ##__VA_ARGS__)
#define DBG_SSCAL_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:SSCAL:"str, ##__VA_ARGS__)
#else
#define DBG_SSCAL_ERR(str, ...)
#define DBG_SSCAL_WARN(str, ...)
#define DBG_SSCAL_INFO(str, ...)
#define DBG_SSCAL_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_USBD_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:USBD:"str, ##__VA_ARGS__)
#define DBG_USBD_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:USBD:"str, ##__VA_ARGS__)
#define DBG_USBD_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:USBD:"str, ##__VA_ARGS__)
#define DBG_USBD_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:USBD:"str, ##__VA_ARGS__)
#else
#define DBG_USBD_ERR(str, ...)
#define DBG_USBD_WARN(str, ...)
#define DBG_USBD_INFO(str, ...)
#define DBG_USBD_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_APP_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:APP:"str, ##__VA_ARGS__)
#define DBG_APP_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:APP:"str, ##__VA_ARGS__)
#define DBG_APP_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:APP:"str, ##__VA_ARGS__)
#define DBG_APP_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:APP:"str, ##__VA_ARGS__)
#else
#define DBG_APP_ERR(str, ...)
#define DBG_APP_WARN(str, ...)
#define DBG_APP_INFO(str, ...)
#define DBG_APP_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_vHOST_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:vHost:"str, ##__VA_ARGS__)
#define DBG_vHOST_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:vHost:"str, ##__VA_ARGS__)
#define DBG_vHOST_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:vHost:"str, ##__VA_ARGS__)
#define DBG_vHOST_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:vHost:"str, ##__VA_ARGS__)
#else
#define DBG_vHOST_ERR(str, ...)
#define DBG_vHOST_WARN(str, ...)
#define DBG_vHOST_INFO(str, ...)
#define DBG_vHOST_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_HBDMA_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:HbDma:"str, ##__VA_ARGS__)
#define DBG_HBDMA_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:HbDma:"str, ##__VA_ARGS__)
#define DBG_HBDMA_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:HbDma:"str, ##__VA_ARGS__)
#define DBG_HBDMA_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:HbDma:"str, ##__VA_ARGS__)
#else
#define DBG_HBDMA_ERR(str, ...)
#define DBG_HBDMA_WARN(str, ...)
#define DBG_HBDMA_INFO(str, ...)
#define DBG_HBDMA_TRACE(str, ...)
#endif

#if DEBUG_INFRA_EN
#define DBG_LVDS_ERR(str, ...)   Cy_Debug_AddToLog(1, "Err:LVDS:"str, ##__VA_ARGS__)
#define DBG_LVDS_WARN(str, ...)  Cy_Debug_AddToLog(2, "W:LVDS:"str, ##__VA_ARGS__)
#define DBG_LVDS_INFO(str, ...)  Cy_Debug_AddToLog(3, "Info:LVDS:"str, ##__VA_ARGS__)
#define DBG_LVDS_TRACE(str, ...) Cy_Debug_AddToLog(4, "T:LVDS:"str, ##__VA_ARGS__)
#else
#define DBG_LVDS_ERR(str, ...)
#define DBG_LVDS_WARN(str, ...)
#define DBG_LVDS_INFO(str, ...)
#define DBG_LVDS_TRACE(str, ...)
#endif

/*
    Data Watchpoint trace based code profiling usage guide:
    For the section of code that you wish to profile, follow these steps:

    1. Initialize the DWT_CONTROL and DEMCR_TRACENA by calling initDWT() 
        in main() function.

    2. For section that you wish to calculate the CPU cycles spent add this code:

    =============================================================================
    volatile uint32_t start=0, stop=0, CPUcycles=0; 
    start = ReadCycleCounter();
    .
    .
    // Do stuff here
    .
    .
    stop = ReadCycleCounter();

    CPUcycles = (uint32_t)(stop - start);
    print(CPUcycles);
    StopTimer();
    =============================================================================

    3. Now we have acquired the CPU cycles spent while executing that particular
        section of code. Based on the CPU clock frequency, you can get the time
        spent in microseconds.
    
    4. For example:
        CPU Clock = 50Mhz
        CPUcycles = 11693

        Time spent = (1/50,000,000)*11,693. This is equal to 233 microseconds.
*/

/* Data watchpoint Trace control  register */
#define DWT_CONTROL_CM4             (*((volatile uint32_t*)0xE0001000))

/* Data watchpoint Trace cycle count enable register */
#define DWT_CYCCNTENA_CM4           (1UL<<0)

/* Data watchpoint Trace cycle count value register */
#define DWT_CYCCNT_CM4              (*((volatile uint32_t*)0xE0001004))
#define DEMCR_CM4                   (*((volatile uint32_t*)0xE000EDFC))
#define DEMCR_TRACENA_CM4           (1UL << 24)

#define Cy_Debug_initDWT()                          \
        do{                                         \
        (DEMCR_CM4 |= DEMCR_TRACENA_CM4);           \
        (DWT_CONTROL_CM4 |= (DWT_CYCCNTENA_CM4));   \
        }while(0)

#define Cy_Debug_ResetCycleCounter()                \
        DWT_CYCCNT_CM4 = 0

#define Cy_Debug_ReadCycleCounter()                 \
        DWT_CYCCNT_CM4

#define Cy_Debug_StopTimer()                        \
        (DWT_CONTROL_CM4 &= ~(DWT_CYCCNTENA_CM4))  

#endif /* DOXYGEN */

/** \} group_usbfxstack_fx_utils_macros */

/** 
 * \addtogroup group_usbfxstack_fx_utils_enums
 * \{
 */

/**
 * @typedef cy_en_debug_interface_t
 * @brief List of output interfaces through which the log messages from
 * firmware can be sent out.
 */
typedef enum
{
    CY_DEBUG_INTFCE_UART_SCB0,                  /**< Prints through SCB0-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB1,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB2,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB3,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB4,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB5,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_UART_SCB6,                  /**< Prints through SCB1-UART interface. */
    CY_DEBUG_INTFCE_USBFS_CDC,                  /**< Prints through USBFS interface using CDC class. */
    CY_DEBUG_INTFCE_USB_CDC,                    /**< Prints through USBHS/USBSS interface using CDC class. */
    CY_DEBUG_INTFCE_RAM,                        /**< Keeps stored in a circular buffer in RAM */
} cy_en_debug_interface_t;

/** \} group_usbfxstack_fx_utils_enums */

/** 
 * \addtogroup group_usbfxstack_fx_utils_macros
 * \{
 */

/** By default, use SCB0 when logging through UART. */
#define CY_DEBUG_INTFCE_UART    (CY_DEBUG_INTFCE_UART_SCB0)

/** Number of DMA descriptors used to transfer log data to the destination. */
#define CY_DEBUG_DMA_DSCR_CNT   (4)

/** \} group_usbfxstack_fx_utils_macros */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

/**
 * @brief Structure used to configure the debug logger module.
 */
typedef struct
{
    uint8_t *pBuffer;                           /**< Pointer to the RAM buffer where log data will be stored. */
    uint8_t traceLvl;                           /**< Verbosity level below which messages should be logged. */
    uint16_t bufSize;                           /**< Size of the RAM buffer in bytes. */
    cy_en_debug_interface_t dbgIntfce;          /**< Identifies the interface through which data is logged. */
    bool printNow;                              /**< Whether messages should be logged as soon as possible. */
} cy_stc_debug_config_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_typedefs
 * \{
 */

/**
 * Callback function used to notify application about data received on the debug interface.
 */
typedef void (*cy_cb_debug_data_recv_cb_t) (
        uint8_t  *pDataBuf,                     /**< Pointer to read data buffer. */
        uint16_t  dataLength,                   /**< Length of data received. */
        void     *pUserCtxt                     /**< Caller context data. */
        );


/** \} group_usbfxstack_fx_utils_typedefs */

/**
 * \addtogroup group_usbfxstack_fx_utils_structs
 * \{
 */

/**
 * @brief Structure containing the information relating to read operation queued
 * on USBFS CDC device.
 */
typedef struct cy_stc_debug_recv_context_
{
    uint8_t                    *pReadBuffer;    /**< Pointer to read data buffer. */
    cy_cb_debug_data_recv_cb_t  readDoneCb;     /**< Callback function pointer. */
    void                       *pUserCtxt;      /**< Caller context for the read callback. */
    uint16_t                    requestSize;    /**< Size of data requested. */
    uint16_t                    readCount;      /**< Actual size of data read. */
} cy_stc_debug_recv_context_t;

/**
 * @brief Structure containing the debug logging module's internal state
 * information.
 */
typedef struct cy_stc_debug_context_
{
    uint8_t                 *pMsg;              /**< Pointer to the RAM based temporary log buffer. */
    uint16_t                 bufSize;           /**< Size of the RAM based log buffer. */
    uint16_t                 rdPtr;             /**< Index from which the read data should next be taken out. */
    uint16_t                 wrPtr;             /**< Index at which new log data should be added. */
    cy_en_debug_interface_t  intfc;             /**< Identifies the interface through which data is logged. */
    uint8_t                  dbgLevel;          /**< Verbosity level below which messages should be logged. */
    bool                     printNow;          /**< Whether immediate output of data is required. */

    uint8_t                  maxDmaSize;        /**< Maximum size of DMA data transfer. */
    bool                     inProgress;        /**< Whether printing of messages is already in progress. */
    void                    *pDbgScb;           /**< Identifies SCB block used for data logging. */
    uint32_t                 dbgDmaDscr[CY_DEBUG_DMA_DSCR_CNT][8];
                                                /**< DMAC descriptors used to send data to SCB. */

    cy_stc_debug_recv_context_t dbgRcvInfo;     /**< Current debug read status. */
} cy_stc_debug_context_t;

/** \} group_usbfxstack_fx_utils_structs */

/**
 * \addtogroup group_usbfxstack_fx_utils_enums
 * \{
 */

/**
 * @typedef cy_en_debug_status_t
 * @brief Possible return status values from the low level logging
 * function.
 */
typedef enum
{
    CY_DEBUG_STATUS_SUCCESS,                    /**< Logging successful. */
    CY_DEBUG_STATUS_FAILURE,                    /**< Logging failed. */
} cy_en_debug_status_t;

/** \} group_usbfxstack_fx_utils_enums */

/**
 * \addtogroup group_usbfxstack_fx_utils_functions
 * \{
 */

/*****************************************************************************
 * Function Name: InitUart
 *************************************************************************//**
 *
 *  Initialize SCB as UART for printing logs.
 *
 * \param scbIndex
 * Index of SCB to be used for UART output. Only 0 and 1 supported at present.
 *
 ****************************************************************************/
void InitUart(uint8_t scbIndex);

/*******************************************************************************
 * Function name: Cy_Debug_LogInit
 ****************************************************************************//**
 *
 * The API initializes the debug logger module with the desired configuration.
 *
 * \param pDbgCfg
 * Debug module config parameters.
 *
 *******************************************************************************/
void Cy_Debug_LogInit(cy_stc_debug_config_t *pDbgCfg);

/*******************************************************************************
 * Function name: Cy_Debug_AddToLog
 ****************************************************************************//**
 *
 * This is the main logging function supported by this debug module. It takes
 * in a variable list of arguments like the printf function, but only the "%c",
 * "%d", "%x" format specifiers are supported. A verbosity level is associated
 * with each message to be printed.
 *
 * \param dbgLevel
 * Verbosity level associated with the message.
 *
 * \param message
 * Format string.
 *
 * \return
 * Success/Failure return code.
 *******************************************************************************/
cy_en_debug_status_t Cy_Debug_AddToLog (uint8_t dbgLevel,
                                          char *message, ...);

/*******************************************************************************
 * Function name: Cy_Debug_PrintLog
 ****************************************************************************//**
 *
 * This API is used to get the logger module to incrementally output a part of the
 * saved log messages to the selected interfaces such as UART or Virtual COM port.
 *
 *******************************************************************************/
void Cy_Debug_PrintLog(void);

/*******************************************************************************
 * Function name: Cy_Debug_ChangeLogLevel
 ****************************************************************************//**
 *
 * This API can be used to change the verbosity level of messages enabled by the
 * logger module at runtime.
 *
 * \param level
 * New verbosity level to be set.
 *
 *******************************************************************************/
void Cy_Debug_ChangeLogLevel(uint8_t level);

/*******************************************************************************
 * Function name: Cy_Debug_SetPrintNow
 ****************************************************************************//**
 *
 * This API allows the immediate print feature of the debug module to be updated
 * at runtime.
 *
 * \param immed_print_en
 * Whether immediate (blocking) print function is to be enabled or disabled.
 *
 *******************************************************************************/
void Cy_Debug_SetPrintNow(bool immed_print_en);

/*******************************************************************************
 * Function name: Cy_Debug_QueueDataRead
 ****************************************************************************//**
 *
 * If a CDC interface is being used for data logging, this function queues a read
 * on the corresponding OUT endpoint. The doneCbk will be called once the requested
 * amount of data is received or a SLP/ZLP is received on the OUT endpoint.
 *
 * \param pReadBuffer
 * Pointer to buffer into which data is to be read.
 *
 * \param dataLength
 * Maximum size of data to be read. Expected to be a multiple of 64.
 *
 * \param doneCbk
 * Callback function to be called on read completion.
 *
 * \param pUserCtxt
 * User context structure to be passed to the callback.
 *
 * \return
 * true if read operation is queued, false otherwise.
 *
 *******************************************************************************/
bool Cy_Debug_QueueDataRead(uint8_t *pReadBuffer, uint16_t dataLength,
                            cy_cb_debug_data_recv_cb_t doneCbk, void *pUserCtxt);

/*******************************************************************************
 * Function name: Cy_Debug_HandleReadIntr
 ****************************************************************************//**
 *
 * Function called from USBFS ISR corresponding to CDC OUT endpoint to read
 * the data and pass on to user callback.
 *
 *******************************************************************************/
void Cy_Debug_HandleReadIntr(void);

/** \} group_usbfxstack_fx_utils_functions */


#if defined(__cplusplus)
}
#endif

#endif /* _CY_DEBUG_H_ */

/** \} group_usbfxstack_fx_utils */

/*[]*/

