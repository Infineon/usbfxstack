/***************************************************************************//**
* \file cy_hbdma_mgr.h
* \version 1.0
*
* Provides API declarations of the High BandWidth DMA manager middleware.
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
 * \addtogroup group_usbfxstack_hb_dma
 * \{
 */

/**
 * \addtogroup group_usbfxstack_hb_dma_macros
 * \{
 */

#if !defined(CY_HBDMA_MGR_H)

/** Indicates the use of HB-DMA Manager */
#define CY_HBDMA_MGR_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "cy_device.h"
#include "cy_pdl.h"
#include "cy_hbdma.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#endif /* FREERTOS_ENABLE */

#if defined(CY_IP_MXS40LVDS2USB32SS)

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
 *                          Macro Definitions
 *******************************************************************************/

/** HBDMA Manager Middleware identifier */
#define CY_HBDMA_MGR_ID                 CY_PDL_DRV_ID(0x81U)

/** Maximum number of adapters supported by the HBWSS DMA manager. */
#define CY_HBDMA_MAX_ADAP_CNT           (6U)

/** Number of DataWire instances being used in the DMA driver. */
#define CY_HBDMA_DW_ADAP_CNT            (2U)

/** Number of DataWire descriptors required per channel. */
#define CY_HBDMA_DW_DSCR_PER_CHN        (3U)

/** Number of entries supported in the HBDma Interrupt Message Queue. */
#define CY_HBDMA_INTR_QUEUE_ENTRIES     (128U)

/** Queue almost full threshold after which DMA interrupts are blocked. */
#define CY_HBDMA_INTRQ_FULL_THRESHOLD   (CY_HBDMA_INTR_QUEUE_ENTRIES - 16U)

/** Queue free threshold after which DMA interrupts are re-enabled. */
#define CY_HBDMA_INTRQ_FREE_THRESHOLD   (CY_HBDMA_INTR_QUEUE_ENTRIES - 64U)

/** Maximum DMA buffer size supported. */
#define CY_HBDMA_MAX_BUFFER_SIZE(en_64k)        ((en_64k != 0) ? 0x10000U : 0xFFF0U)

/** Size of stack reserved for HBDma manager task in 32-bit words. */
#define CY_HBDMA_TASK_STACK_DEPTH       (512)

/** Priority of HBDma manager task. */
#define CY_HBDMA_TASK_PRIORITY          (10)

/** \} group_usbfxstack_hb_dma_macros */

/**
 * \addtogroup group_usbfxstack_hb_dma_enums
 * \{
 */

/*******************************************************************************
 *                              Enumerated Types
 *******************************************************************************/

/**
 * @typedef cy_en_hbdma_mgr_status_t
 * @brief List of High BandWidth DMA manager status return codes.
 */
typedef enum
{
    CY_HBDMA_MGR_SUCCESS = 0U,                                                          /**< Operation completed successfully */
    CY_HBDMA_MGR_BAD_PARAM = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 1U),              /**< One or more input parameters are invalid */
    CY_HBDMA_MGR_TIMEOUT = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 2U),                /**< Timeout occurred for the operation  */
    CY_HBDMA_MGR_DRV_HW_ERROR = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 3U),           /**< A hardware error occurred during operation */
    CY_HBDMA_MGR_MEMORY_ERROR = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 4U),           /**< Out of memory */
    CY_HBDMA_MGR_SEQUENCE_ERROR = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 5U),         /**< Call sequence error. */
    CY_HBDMA_MGR_MEM_CORRUPTION = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 6U),         /**< Memory corruption error. */
    CY_HBDMA_MGR_SOCK_BUSY = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 7U),              /**< Socket busy error. */
    CY_HBDMA_MGR_NOT_SUPPORTED = (CY_HBDMA_MGR_ID | CY_PDL_STATUS_ERROR | 8U),          /**< Not supported error. */
} cy_en_hbdma_mgr_status_t;

/**
 * @typedef cy_en_hbdma_chn_type_t
 * @brief Types of HBW DMA channels.
 */
typedef enum
{
    CY_HBDMA_TYPE_IP_TO_IP = 0,         /**< Peripheral to peripheral DMA channel. */
    CY_HBDMA_TYPE_IP_TO_MEM,            /**< Peripheral to memory DMA channel. */
    CY_HBDMA_TYPE_MEM_TO_IP             /**< Memory to peripheral DMA channel. */
} cy_en_hbdma_chn_type_t;

/**
 * @typedef cy_en_hbdma_cb_type_t
 * @brief List of event for which DMA channel callbacks are provided.
 */
typedef enum
{
    CY_HBDMA_CB_NONE = 0,               /**< Invalid callback type. */
    CY_HBDMA_CB_XFER_CPLT,              /**< Transfer completed. */
    CY_HBDMA_CB_PROD_EVENT,             /**< Buffer received from producer. */
    CY_HBDMA_CB_CONS_EVENT,             /**< Buffer drained by the consumer. */
    CY_HBDMA_CB_ERROR,                  /**< DMA transfer error raised by the adapter. */
    CY_HBDMA_CB_SUSPENDED               /**< DMA transfer has been suspended. */
} cy_en_hbdma_cb_type_t;

/**
 * @typedef cy_en_hbdma_chn_state_t
 * @brief List of HBW DMA channel states.
 */
typedef enum
{
    CY_HBDMA_CHN_NOT_CONFIGURED = 0,    /**< DMA channel has not been initialized. */
    CY_HBDMA_CHN_CONFIGURED,            /**< DMA channel has been created and configured */
    CY_HBDMA_CHN_ACTIVE,                /**< DMA Channel has active transaction going on */
    CY_HBDMA_CHN_OVERRIDE,              /**< DMA channel has over-ride data transfer going on */
    CY_HBDMA_CHN_DATADROP,              /**< Auto DMA channel configured to drop all incoming data. */
    CY_HBDMA_CHN_ERROR                  /**< DMA Channel has encountered an error. */
} cy_en_hbdma_chn_state_t;

/** \} group_usbfxstack_hb_dma_enums */

/**
 * \addtogroup group_usbfxstack_hb_dma_typedefs
 * \{
 */

/*******************************************************************************
 *                              Type Definitions
 *******************************************************************************/

/** \cond INTERNAL */
struct cy_stc_hbdma_channel;
struct cy_stc_hbdma_mgr_context;
struct cy_stc_hbdma_buff_status;
/** \endcond */

/**
 * @brief Type of callback function to be provided by the application to
 * enable dynamic enable/disable control of the HBDma interrupts. This function
 * will be used by the HBDma manager as required to prevent overflows in
 * the message queue used to receive interrupt notifications.
 *
 * \param enableIntr
 * Whether the DMA adapter interrupts are to be enabled or disabled.
 *
 * \return
 * void
 */
typedef void (*cy_cb_hbdma_mgr_intr_ctrl_cb_t) (
        bool enableIntr
        );

/**
 * @brief Type of callback function to be registered to receive DMA channel
 * event notifications.
 *
 * \param handle
 * Handle to DMA channel on which event of interest has been detected.
 *
 * \param type
 * Type of the DMA event being notified.
 *
 * \param pbufStat
 * Provides active DMA buffer status where applicable.
 *
 * \param userCtx
 * User provided opaque data which is passed back to the event callback.
 *
 * \return
 * void
 */
typedef void (*cy_cb_hbdma_event_callback_t) (
        struct cy_stc_hbdma_channel *handle,
        cy_en_hbdma_cb_type_t type,
        struct cy_stc_hbdma_buff_status *pbufStat,
        void *userCtx
        );

/** \} group_usbfxstack_hb_dma_typedefs */

/**
 * \addtogroup group_usbfxstack_hb_dma_structs
 * \{
 */

/**
 * @brief Structure encapsulating the configuration parameters for a High BandWidth
 * DMA channel to be created.
 */
typedef struct cy_stc_hbdma_chn_config
{
    uint32_t size;                              /**< Size of DMA buffers to be used. */
    uint16_t count;                             /**< Number of DMA buffers to be used. */
    uint16_t prodHdrSize;                       /**< Size of header associated with the channel. */
    uint32_t prodBufSize;                       /**< Buffer size to be configured on producer side. */
    cy_en_hbdma_chn_type_t chType;              /**< Type of DMA channel to be created. */
    bool bufferMode;                            /**< Whether unit of data transfer is a buffer. */
    bool eventEnable;                           /**< Enable for DMA event triggers. */
    uint8_t prodSckCount;                       /**< Number of producer sockets: 0, 1 or 2 */
    uint8_t consSckCount;                       /**< Number of consumer sockets: 0, 1 or 2 */
    uint8_t endpAddr;                           /**< USB endpoint associated with the channel (optional). */
    cy_hbdma_socket_id_t prodSck[2];            /**< ID of the producer socket(s). */
    cy_hbdma_socket_id_t consSck[2];            /**< ID of the consumer socket(s). */
    uint16_t usbMaxPktSize;                     /**< Maximum packet size of the associated USB endpoint. */
    uint32_t intrEnable;                        /**< Enable for DMA interrupts. */
    cy_cb_hbdma_event_callback_t cb;            /**< Event callback function pointer. */
    void *userCtx;                              /**< User context for the callback. */
} cy_stc_hbdma_chn_config_t;

/**
 * @brief Structure encapsulation all configuration and state information associated with
 * a High BandWidth DMA channel. A pointer to this structure should be passed as the channel
 * handle to all DMA channel API functions.
 */
typedef struct cy_stc_hbdma_channel
{
    struct cy_stc_hbdma_mgr_context *pContext;  /**< HBWSS DMA manager context. */
    cy_en_hbdma_chn_type_t type;                /**< The type of the DMA channel */
    bool bufferMode;                            /**< Whether unit of data transfer is a buffer. */
    bool eventEnable;                           /**< Event enable for sockets associated with this channel. */
    uint8_t prodSckCount;                       /**< Number of producer sockets: 0, 1 or 2 */
    uint8_t consSckCount;                       /**< Number of consumer sockets: 0, 1 or 2 */
    uint16_t count;                             /**< Number of buffers for the channel */
    uint32_t size;                              /**< The buffer size for the channel */
    cy_hbdma_socket_id_t prodSckId[2];          /**< The producer (ingress) socket IDs */
    cy_hbdma_socket_id_t consSckId[2];          /**< The consumer (egress) socket IDs */
    uint16_t prodHdrSize;                       /**< Size of header associated with the channel. */
    uint32_t prodBufSize;                       /**< Buffer size to be configured on producer side. */
    uint16_t firstProdDscrIndex[2];             /**< Head of producer descriptor chain for each socket. */
    uint16_t firstConsDscrIndex[2];             /**< Head of consumer descriptor chain for each socket. */
    uint16_t curProdDscrIndex[2];               /**< Current descriptor for producer(s). */
    uint16_t curConsDscrIndex[2];               /**< Current descriptor for consumer(s). */
    uint16_t nextProdDscr;                      /**< Next produce descriptor to be updated. */
    uint16_t nextConsDscr;                      /**< Next consume descriptor to be updated. */
    uint16_t lastProdDscr;                      /**< The last active produce descriptor. */
    uint16_t overrideDscrIndex;                 /**< Descriptor for send/receive transfers */
    uint32_t xferSize;                          /**< Current transfer size */
    uint32_t notification;                      /**< Notifications enabled for this channel */
    cy_cb_hbdma_event_callback_t cbFunc;        /**< Function to be called on events */
    void *cbCtx;                                /**< User context to pass to the callback. */
    cy_en_hbdma_chn_state_t state;              /**< Current state of the DMA channel. */
    uint8_t activeSckIndex;                     /**< Socket Index which needs to be processed */
    uint8_t pendingEvtCnt;                      /**< Number of pending consume events to be processed. */
    uint8_t nextProdSck;                        /**< Socket index on which next producer event for a 2:1 channel
                                                     is expected. */
    uint16_t overrideCnt;                       /**< Count of override operations performed on this channel. */
    uint8_t commitCnt[2];                       /**< Number of commit operations performed to each consumer socket. */
    uint8_t discardCnt[2];                      /**< Number of pending discard operations on each consumer socket. */
    uint8_t endpAddr;                           /**< USB endpoint associated with the channel. */

    bool egressDWTrigDone[2];                   /**< Whether DataWire trigger for each egress channel has been done. */
    bool ingressDWRqtQueued[2];                 /**< Whether ingress DataWire request has been queued. */
    bool egressDWRqtQueued[2];                  /**< Whether egress DataWire request has been queued. */
    uint16_t epMaxPktSize;                      /**< Maximum packet size for the USBHS endpoint. */
    cy_stc_dma_descriptor_t *pProdDwDscr[2];    /**< Current DataWire descriptor pointer for producer channels. */
    cy_stc_dma_descriptor_t *pConsDwDscr[2];    /**< Current DataWire descriptor pointer for consumer channels. */
    uint16_t curIngressXferSize[2];             /**< Current ingress DataWire transfer size in bytes. */
    uint8_t *pCurEgressDataBuf[2];              /**< Current buffer from which egress DataWire channel is reading. */
    uint16_t curEgressXferSize[2];              /**< Size of data in buffer from which egress channel is reading. */
} cy_stc_hbdma_channel_t;

/**
 * @brief Structure encapsulating the status of the current DMA buffer associated with
 * a HBW DMA channel.
 */
typedef struct cy_stc_hbdma_buff_status
{
    uint8_t *pBuffer;                           /**< Pointer to the data buffer. */
    uint32_t size;                              /**< Actual size of the buffer in bytes. */
    uint32_t count;                             /**< Byte count of valid data in buffer. */
    uint16_t status;                            /**< Status fields associated with the descriptor. */
} cy_stc_hbdma_buff_status_t;

/**
 * @brief Context structure used by the HBW DMA descriptor list allocator.
 */
typedef struct cy_stc_hbdma_dscr_list
{
    uint16_t availCount;                                /**< Count of available descriptors. */
    uint16_t totalCount;                                /**< Total number of descriptors. */
    uint32_t dscrStatus[CY_HBDMA_MAX_DSCR_CNT / 32U];   /**< Bit map representing state of each descriptor. */
} cy_stc_hbdma_dscr_list_t;

/**
 * @brief Context structure used by the HBW DMA buffer allocator.
 */
typedef struct cy_stc_hbdma_buf_mgr
{
    uint8_t    *startAddr;                      /**< Start address of memory region available for allocation. */
    uint32_t    regionSize;                     /**< Size of memory region available for allocation. */
    uint32_t   *usedStatus;                     /**< Bit-map that stores the status of memory blocks. */
    uint32_t    statusSize;                     /**< Size of the status array in 32 bit words. */
    uint32_t    searchPos;                      /**< Word address from which to start searching for memory. */
} cy_stc_hbdma_buf_mgr_t;

/**
 * @brief Context structure for the HBWSS DMA manager middleware library.
 * The DMA manager can function in a bare-metal configuration as well as in a
 * RTOS based configuration. Additional fields are added to the structure in
 * case the RTOS configuration is used.
 */
typedef struct cy_stc_hbdma_mgr_context
{
    cy_stc_hbdma_context_t *pDrvContext;        /**< HBWSS DMA driver context structure. */
    cy_stc_hbdma_dscr_list_t *pDscrPool;        /**< HBWSS DMA descriptor pool. */
    cy_stc_hbdma_buf_mgr_t *pBufMgr;            /**< HBWSS DMA buffer manager. */
    uint32_t socketUsed[CY_HBDMA_MAX_ADAP_CNT]; /**< Bit-map showing the sockets in use. */
    cy_stc_hbdma_channel_t *sckChannelMap[CY_HBDMA_MAX_ADAP_CNT][CY_HBDMA_SOCK_PER_ADAPTER];
                                                /**< Table used to map sockets to DMA channels. */
    cy_stc_dma_descriptor_t *dwDscrList;        /**< List of DataWire descriptors used across all channels. */
    uint16_t usbIngMultEnable;                  /**< Bit-mask providing mult enable setting for USB ingress sockets. */
    uint16_t usbEgrMultEnable;                  /**< Bit-mask providing mult enable setting for USB egress sockets. */

#if FREERTOS_ENABLE
    TaskHandle_t  dmaMgrTask;                   /**< Handle to HbDMA RTOS task. */
    QueueHandle_t dmaIntrQueue;                 /**< Handle to HbDMA interrupt message queue. */
    uint32_t queueOverflowCount;                /**< Count of interrupt message queue overflow events. */
    cy_cb_hbdma_mgr_intr_ctrl_cb_t isrCtrlCb;   /**< Callback to enable/disable ISR execution. */
    bool intrDisabled;                          /**< Whether HbDMA interrupts are disabled. */
    bool cbFromISREnable;                       /**< Whether handling of DMA callbacks from ISR is enabled. */
#endif /* FREERTOS_ENABLE */
    bool en_64k;                                /**< Whether 64KB DMA buffer support is enabled. */

    void *pUsbStackCtx;                         /**< USB stack context pointer. Needed for USB-HS operation. */
} cy_stc_hbdma_mgr_context_t;

/**
 * @brief Structure holding HbDma interrupt information.
 */
typedef struct cy_stc_hbdma_intr_msg
{
    uint32_t socketId;                          /**< Socket on which interrupt was received. */
    uint32_t intrType;                          /**< Type of interrupt received. */
    uint32_t curDscr;                           /**< Active descriptor at the time of interrupt. */
} cy_stc_hbdma_intr_msg_t;

/** \} group_usbfxstack_hb_dma_structs */

/**
 * \addtogroup group_usbfxstack_hb_dma_functions
 * \{
 */

/*******************************************************************************
 *                            Function Prototypes
 *******************************************************************************/

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_GetVersion
 ****************************************************************************//**
 *
 * Returns the High BandWidth Manager Middleware version information in the
 * form of a 32-bit word including the major, minor, patch and build numbers.
 *
 * \return
 * Version information for this library in the format:
 *         b31:28 -> Major Version
 *         b27:24 -> Minor Version
 *         b23:16 -> Patch Version
 *         b15:00 -> Build Number
 *******************************************************************************/
uint32_t
Cy_HBDma_Mgr_GetVersion(
        void
        );

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
        uint16_t maxDscrCount
        );

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
        cy_stc_hbdma_dscr_list_t *list_p
        );

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
        uint16_t *dscrIndex_p
        );

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
        uint16_t dscrIndex
        );

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
        uint32_t regionSize
        );

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
        bool freeAll
        );

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
        uint32_t bufferSize
        );

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
        void *buffer_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_Init
 ****************************************************************************//**
 *
 * Initialize the High BandWidth DMA manager library.
 *
 * \param context_p
 * Pointer to the DMA manager context structure.
 *
 * \param drvContext_p
 * Pointer to the HBDMA driver context structure.
 *
 * \param dscrPool_p
 * Pointer to the HBDMA descriptor list.
 *
 * \param bufMgr_p
 * Pointer to the HBDMA buffer manager context.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the DMA manager init was successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_Init (
        cy_stc_hbdma_mgr_context_t *context_p,
        cy_stc_hbdma_context_t *drvContext_p,
        cy_stc_hbdma_dscr_list_t *dscrPool_p,
        cy_stc_hbdma_buf_mgr_t *bufMgr_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_DeInit
 ****************************************************************************//**
 *
 * De-initialize the High BandWidth DMA manager library.
 *
 * \param context_p
 * Pointer to the DMA manager context structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the manager de-init is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the context structure passed is invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_DeInit (
        cy_stc_hbdma_mgr_context_t *context_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_RegisterUsbContext
 ****************************************************************************//**
 *
 * Register the USB stack context pointer with the High BandWidth manager. A valid
 * stack context is required to make use of USB-HS endpoints and DataWire channels
 * with the High BandWidth channel API.
 *
 * \param context_p
 * Pointer to the DMA manager context structure.
 * \param pUsbStackCtx
 * Pointer to USB stack context structure passed as an opaque pointer.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_RegisterUsbContext (
        cy_stc_hbdma_mgr_context_t *context_p,
        void *pUsbStackCtx
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_UpdateMultEn
 ****************************************************************************//**
 *
 * Update the MULT setting for sockets associated with USB endpoints. When the
 * MULT feature is enabled at the socket level, the USB32DEV endpoint memory
 * is allowed to combine the data from multiple DMA buffers into one transfer
 * burst, thereby getting better data throughput in typical use cases.
 *
 * @warning This function is expected to be called from the USB32 device stack
 * and not directly by the user.
 *
 * \param pContext
 * Pointer to the DMA manager context structure.
 *
 * \param endpNumber
 * USB endpoint index.
 *
 * \param isEgressEp
 * Whether this is an Egress endpoint.
 *
 * \param multEnable
 * Whether to enable the MULT feature.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the MULT setting is updated correctly.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Mgr_UpdateMultEn (
        cy_stc_hbdma_mgr_context_t *pContext,
        uint32_t endpNumber,
        bool isEgressEp,
        bool multEnable
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Create
 ****************************************************************************//**
 *
 * Create a High BandWidth DMA channel based on the parameters specified in the
 * config structure.
 *
 * \param pDmaMgr
 * Pointer to the DMA manager context structure.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param config
 * Desired DMA channel configuration.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel creation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Create (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_stc_hbdma_channel_t *handle,
        cy_stc_hbdma_chn_config_t *config
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Destroy
 ****************************************************************************//**
 *
 * Destroy a High BandWidth DMA channel. The implementation makes sure that the
 * DMA sockets associated with the channel are disabled in addition to cleaning
 * up the data structures and freeing memory elements.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel destroy operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Destroy (
        cy_stc_hbdma_channel_t *handle
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Enable
 ****************************************************************************//**
 *
 * Enable a High BandWidth DMA channel for data transfer. Any sockets associated
 * with the DMA channel will be enabled. If a non-zero xferSize is specified,
 * the channel gets disabled automatically after transferring the specified
 * amount of data.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param xferSize
 * Size of data to be transferred through the channel.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel enable operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Enable (
        cy_stc_hbdma_channel_t *handle,
        uint32_t xferSize
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Disable
 ****************************************************************************//**
 *
 * Disable a High BandWidth DMA channel.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel disable operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Disable (
        cy_stc_hbdma_channel_t *handle
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Reset
 ****************************************************************************//**
 *
 * Reset a High BandWidth DMA channel. This leaves all the DMA buffers associated
 * with the channel in the empty state and the channel itself in the disabled state.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel reset operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_Reset (
        cy_stc_hbdma_channel_t *handle
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_SetWrapUp
 ****************************************************************************//**
 *
 *  This API is used to forcibly commit a DMA buffer to the consumer, and is
 *  useful in the case where data transfer has abruptly stopped without the
 *  producer being able to commit the data buffer.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param sckOffset
 *  Socket id to wrapup.
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel reset operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the DMA channel is not in the required state.
 *******************************************************************************/

cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_SetWrapUp(cy_stc_hbdma_channel_t *pHandle, uint8_t sckOffset);

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetBuffer
 ****************************************************************************//**
 *
 * Get the status of the active DMA buffer associated with the DMA channel.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Return parameter to pass the buffer status through.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the API is called when the descriptor is not
 * in the expected state.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_GetBuffer (
        cy_stc_hbdma_channel_t *handle,
        cy_stc_hbdma_buff_status_t *bufStat_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_CommitBuffer
 ****************************************************************************//**
 *
 * Mark a DMA buffer occupied on the consumer side of the DMA channel.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Information about the buffer to be committed.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the commit operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_CommitBuffer (
        cy_stc_hbdma_channel_t *handle,
        cy_stc_hbdma_buff_status_t *bufStat_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_DiscardBuffer
 ****************************************************************************//**
 *
 * Mark a DMA buffer empty on the producer side of the DMA channel.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param bufStat_p
 * Information about the buffer to be dropped.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the discard operation is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_DiscardBuffer (
        cy_stc_hbdma_channel_t *handle,
        cy_stc_hbdma_buff_status_t *bufStat_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_SendData
 ****************************************************************************//**
 *
 * Prepares to send data through a DMA channel. This API should be called when
 * the channel is in the disabled state, and will return after the DMA operation
 * has been queued. The Cy_HBDma_Channel_WaitForSendCplt function can be called
 * to wait until the transfer is completed.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of consumer socket through which data is to be sent.
 *
 * \param dataBuf_p
 * Pointer to the buffer containing data to be sent.
 *
 * \param dataSize
 * Size of data to be sent.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the data to be sent is queued successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not idle.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_SendData (
        cy_stc_hbdma_channel_t *handle,
        uint16_t sckIdx,
        uint8_t *dataBuf_p,
        uint32_t dataSize
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_WaitForSendCplt
 ****************************************************************************//**
 *
 * Waits until the DMA transfer requested using Cy_HBDma_Channel_SendData API
 * has been completed.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of consumer socket through which data is to be sent.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the send operation was completed.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if SendData has not been called previously.
 * CY_HBDMA_MGR_TIMEOUT if the operation times out on the consumer side.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_WaitForSendCplt (
        cy_stc_hbdma_channel_t *handle,
        uint16_t  sckIdx
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_ReceiveData
 ****************************************************************************//**
 *
 * Prepare to receive a specific amount of data using a DMA channel. This API
 * should be called while the channel is in the disabled state and will return
 * as soon as channel is enabled for data transfer.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of the producer socket within the channel through which data will be received.
 *
 * \param dataBuf_p
 * Pointer of buffer where the received data should be placed.
 *
 * \param bufferSize
 * Maximum amount of data that may be received.
 *
 * \param actualSize_p
 * Return parameter to get the actual received data size through (can be NULL).
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the receive operation is queued successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not idle.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_ReceiveData (
        cy_stc_hbdma_channel_t *handle,
        uint16_t sckIdx,
        uint8_t  *dataBuf_p,
        uint32_t  bufferSize,
        uint32_t *actualSize_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_WaitForReceiveCplt
 ****************************************************************************//**
 *
 * Wait until a HBW DMA operation initiated using Cy_HBDma_Channel_ReceiveData
 * has been completed. The actual amount of data read is returned through the
 * actualSize_p parameter.
 *
 * \param handle
 * Handle to the DMA channel structure.
 *
 * \param sckIdx
 * Index of the producer socket in the channel to be used for data read.
 *
 * \param actualSize_p
 * Optional return parameter to get the actual size of data received.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the data is received successfully.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if a previous ReceiveData call is not pending.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_WaitForReceiveCplt (
        cy_stc_hbdma_channel_t *handle,
        uint16_t  sckIdx,
        uint32_t *actualSize_p
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetBufferInfo
 ****************************************************************************//**
 *
 * Retrieve the set of DMA buffers associated with a HBDma channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \param pBufPtrs
 * Return array to be filled with the DMA buffer pointers.
 *
 * \param bufferCnt
 * Number of buffer pointers to be fetched.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the buffer query is successful.
 * CY_HBDMA_MGR_BAD_PARAM if the parameters are invalid.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_GetBufferInfo (
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t **pBufPtrs,
        uint8_t bufferCnt
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_AutoDropData
 ****************************************************************************//**
 *
 * This function allows an active Auto DMA channel to be configured to drop
 * all data that is being received on the producer side. The channel has to be
 * reset and then enabled to restore normal operation.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if the channel has been set up to drop the incoming data.
 * CY_HBDMA_MGR_BAD_PARAM if the channel is not valid or not an AUTO channel.
 * CY_HBDMA_MGR_SEQUENCE_ERROR if the channel is not active.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_Channel_AutoDropData (
        cy_stc_hbdma_channel_t *pHandle
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_Cb
 ****************************************************************************//**
 *
 * High BandWidth DMA channel callback function implementation.
 *
 * \param socketId
 * Socket on which interrupt was received.
 *
 * \param intrType
 * Type of interrupt which was received.
 *
 * \param curDscr
 * Active descriptor for the socket.
 *
 * \param userCtx
 * User context for the callback.
 *
 * \return
 * true if further interrupt processing is to be disabled.
 *******************************************************************************/
bool Cy_HBDma_Channel_Cb (
        cy_hbdma_socket_id_t socketId,
        cy_en_hbdma_sock_evt_t intrType,
        uint32_t curDscr,
        void *userCtx
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_TaskHandler
 ****************************************************************************//**
 *
 * High BandWidth DMA manager task.
 *
 * \param pTaskParam
 * Pointer to the HBDma manager context structure.
 *******************************************************************************/
void Cy_HBDma_Mgr_TaskHandler (
        void *pTaskParam
        );

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_RegisterISRCtrlCallback
 ****************************************************************************//**
 *
 * Register a function callback that the DMA manager can call to dynamically
 * enable or disable the HBDMA interrupts based on the state of the interrupt
 * notification message queue.
 *
 * \param pDmaMgr
 * Pointer to the DMA manager context structure.
 *
 * \param cb
 * Pointer to function callback.
 *******************************************************************************/
void Cy_HBDma_Mgr_RegisterISRCtrlCallback (
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        cy_cb_hbdma_mgr_intr_ctrl_cb_t cb
        );


/*******************************************************************************
 * Function Name: Cy_HBDma_Channel_GetChannelState
 ****************************************************************************//**
 *
 * Retrieve the state of DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel structure.
 *
 * \return
 * channel state.
 *******************************************************************************/
cy_en_hbdma_chn_state_t Cy_HBDma_Channel_GetChannelState(cy_stc_hbdma_channel_t *pHandle);

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_DmaCallbackConfigure
 ****************************************************************************//**
 *
 * Function to configure context from which DMA callback functions are generated.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param callbackFromISREnable
 * Whether sending of DMA callbacks directly from ISR is enabled.
 *******************************************************************************/
void Cy_HBDma_Mgr_DmaCallbackConfigure(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        bool callbackFromISREnable);

/*******************************************************************************
 * Function Name: Cy_HBDma_Mgr_GetIntrDropCount
 ****************************************************************************//**
 *
 * Function to get count of DMA interrupt messages dropped due to message queue
 * overflow.
 *
 * If the count is non-zero at any stage, it indicates that DMA interrupts are
 * being generated too fast for the CPU to handle. Either larger DMA buffers
 * should be used to reduce interrupt frequency or the
 * Cy_HBDma_Mgr_RegisterISRCtrlCallback() API should be used to dynamically
 * throttle DMA interrupts.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \return
 * Count of DMA interrupt messages which have been dropped so far.
 *******************************************************************************/
uint32_t Cy_HBDma_Mgr_GetIntrDropCount(
        cy_stc_hbdma_mgr_context_t *pDmaMgr);

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_SetUsbEgressAdapterDelay
 ****************************************************************************//**
 *
 * Update the number of cycles of delay to be applied between consecutive AXI
 * data fetches made by the USB egress DMA adapter. The function is meant to
 * be used by the USB stack based on current USB speed.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param gblDelayCycles
 * Number of delay cycles to be applied in the range of 0 to 15.
 *
 *******************************************************************************/
void Cy_HBDma_Mgr_SetUsbEgressAdapterDelay(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint8_t gblDelayCycles);

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_SetLvdsAdapterIngressMode
 ****************************************************************************//**
 *
 * This function enables or disables the support for egress data transfers from
 * RAM buffers which are not 16-byte aligned based on whether the specified
 * LVDS DMA adapters are working in Ingress only mode or not. For any ingress-only
 * adapter, this support can be disabled to gain better DMA performance.
 *
 * \param pDmaMgr
 * Pointer to DMA manager context structure.
 *
 * \param isAdap0Ingress
 * Whether adapter 0 (sockets 0 to 15) is being used only in ingress direction.
 *
 * \param isAdap1Ingress
 * Whether adapter 1 (sockets 16 to 31) is being used only in ingress direction.
 *
 *******************************************************************************/
void Cy_HBDma_Mgr_SetLvdsAdapterIngressMode(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        bool isAdap0Ingress,
        bool isAdap1Ingress);

/*******************************************************************************
 * Function name: Cy_HBDma_DW_Configure
 ****************************************************************************//**
 *
 * Configure the trigger connections for the DataWire used for transfers
 * through a USB High-Speed Endpoint.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param enable
 * Whether trigger connections are to be enabled or disabled.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_Configure(
        cy_stc_hbdma_channel_t *pHandle,
        bool enable);

/*******************************************************************************
 * Function name: Cy_HBDma_DW_QueueRead
 ****************************************************************************//**
 *
 * Function which queues read operation using DataWire DMA on USBHS OUT endpoint
 * corresponding to a DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param prodIndex
 * Index of the producer from which to read data.
 * \param pBuffer
 * Pointer to the data buffer to read data into.
 * \param dataSize
 * Size of data expected. This should be a multiple of the max packet size.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_QueueRead(
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 prodIndex,
        uint8_t                *pBuffer,
        uint16_t                dataSize);

/*******************************************************************************
 * Function name: Cy_HBDma_DW_CompleteShortRead
 ****************************************************************************//**
 *
 * Function which terminates ongoing USBHS ingress transfer when a short packet
 * has been received on the endpoint.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param prodIndex
 * Index of the producer from which to read data.
 * \param shortPktSize
 * Size of the short packet received in bytes.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_CompleteShortRead(
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 prodIndex,
        uint16_t                shortPktSize);

/*******************************************************************************
 * Function name: Cy_HBDma_DW_QueueWrite
 ****************************************************************************//**
 *
 * Function which queues write operation using DataWire DMA on USBHS IN endpoint
 * corresponding to a DMA channel.
 *
 * \param pHandle
 * Handle to the DMA channel.
 * \param consIndex
 * Index of the consumer to write data into.
 * \param pBuffer
 * Pointer to the data buffer containing the data.
 * \param dataSize
 * Size of data to be transferred.
 *
 * \return
 * CY_HBDMA_MGR_SUCCESS if function is successful, error code otherwise.
 *******************************************************************************/
cy_en_hbdma_mgr_status_t
Cy_HBDma_DW_QueueWrite(
        cy_stc_hbdma_channel_t *pHandle,
        uint8_t                 consIndex,
        uint8_t                *pBuffer,
        uint16_t                dataSize);

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleDW0Interrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles transfer completion interrupt from any of the
 * DataWire channels associated with non EP0 USB-HS OUT endpoints (channels 1 to 15).
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleDW0Interrupt(
        cy_stc_hbdma_mgr_context_t *pDmaMgr);

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleDW1Interrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles transfer completion interrupt from any of the
 * DataWire channels associated with non EP0 USB-HS IN endpoints (channels 1 to 15).
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleDW1Interrupt(
        cy_stc_hbdma_mgr_context_t *pDmaMgr);

/*******************************************************************************
 * Function name: Cy_HBDma_Mgr_HandleUsbShortInterrupt
 ****************************************************************************//**
 *
 * DMA manager function that handles SLP or ZLP interrupts from USB-HS OUT
 * endpoints. This is expected to be triggered from interrupt callback provided
 * by the USB stack.
 *
 * \param pDmaMgr
 * Handle to the DMA manager context.
 * \param epNum
 * Endpoint number on which SLP/ZLP was received.
 * \param pktSize
 * Actual size (in bytes) of the packet received.
 *
 *******************************************************************************/
void
Cy_HBDma_Mgr_HandleUsbShortInterrupt(
        cy_stc_hbdma_mgr_context_t *pDmaMgr,
        uint8_t                     epNum,
        uint16_t                    pktSize);

/** \} group_usbfxstack_hb_dma_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXS40LVDS2USB32SS) */

#endif /* (CY_HBDMA_MGR_H) */

/** \} group_usbfxstack_hb_dma */

/* [] END OF FILE */
