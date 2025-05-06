/***************************************************************************//**
* \file cy_usbfs_cdc.c
* \version 1.0
*
* Implements a USB CDC VCom device using the MXUSBFS IP to enable the debug
* logging function.
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
#include "cy_debug.h"
#include "cy_usbfs_cdc.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Device descriptor. */
static const uint8_t fsCdcDeviceDscr[] = {
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor */
    0x00,0x02,                      /* USB 2.00 */
    0x02,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x08,                           /* Maxpacket size for EP0  */
    0xB4,0x04,                      /* Vendor ID */
    0x08,0x00,                      /* Product ID  - Using the Cypress USB-UART PID for driver binding */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacturer string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Configuration descriptor. */
static const uint8_t fsCdcConfigDscr[] = {
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x43,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x03,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100 mA */

    /* Communication Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of endpoints */
    0x02,                           /* Interface class: Communication interface */
    0x02,                           /* Interface sub class */
    0x01,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* CDC Class-specific Descriptors */
    /* Header functional Descriptor */
    0x05,                           /* Descriptors length(5) */
    0x24,                           /* Descriptor type : CS_Interface */
    0x00,                           /* DescriptorSubType : Header Functional Descriptor */
    0x10,0x01,                      /* bcdCDC : CDC Release Number */

    /* Abstract Control Management Functional Descriptor */
    0x04,                           /* Descriptors Length (4) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x02,                           /* bDescriptorSubType: Abstract Control Model Functional Descriptor */
    0x02,                           /* bmCapabilities: Supports the request combination of Set_Line_Coding,
                                       Set_Control_Line_State, Get_Line_Coding and the notification Serial_State */

    /* Union Functional Descriptor */
    0x05,                           /* Descriptors Length (5) */
    0x24,                           /* bDescriptorType: CS_INTERFACE */
    0x06,                           /* bDescriptorSubType: Union Functional Descriptor */
    0x00,                           /* bMasterInterface */
    0x01,                           /* bSlaveInterface */

    /* Call Management Functional Descriptor */
    0x05,                           /*  Descriptors Length (5) */
    0x24,                           /*  bDescriptorType: CS_INTERFACE */
    0x01,                           /*  bDescriptorSubType: Call Management Functional Descriptor */
    0x00,                           /*  bmCapabilities: Device sends/receives call management information only over
                                        the Communication Class Interface. */
    0x01,                           /*  Interface Number of Data Class interface */

    /* Endpoint Descriptor(Interrupt) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x83,                           /* Endpoint address and description */
    0x03,                           /* Interrupt endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x02,                           /* Servicing interval for data transfers */

    /* Data Interface Descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of endpoints */
    0x0A,                           /* Interface class: Data interface */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Endpoint Descriptor (OUT) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x02,                           /* Endpoint address and description */
    0x02,                           /* BULK endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x00,                           /* Servicing interval for data transfers */

    /* Endpoint Descriptor (IN) */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x81,                           /* Endpoint address and description */
    0x02,                           /* Bulk endpoint type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x00                            /* Servicing interval for data transfers */
};

/* Language ID string. */
static const uint8_t fsCdcLangString[] = {
    0x04,
    0x03,
    0x09, 0x04
};

/* Manufacturer string. */
static const uint8_t fsCdcMfgString[] = {
    0x12,
    0x03,
    'I', 0x00,
    'n', 0x00,
    'f', 0x00,
    'i', 0x00,
    'n', 0x00,
    'e', 0x00,
    'o', 0x00,
    'n', 0x00
};

/* Product string. */
static const uint8_t fsCdcProdString[] = {
    0x18,
    0x03,
    'F', 0x00,
    'X', 0x00,
    '1', 0x00,
    '0', 0x00,
    ' ', 0x00,
    'L', 0x00,
    'o', 0x00,
    'g', 0x00,
    'g', 0x00,
    'e', 0x00,
    'r', 0x00
};

/*
 * The number of clock cycles required to identify a bus reset.
 * This value is hardware dependant and should not be changed.
 */
#define USB_BUS_RST_CNT_VALUE     (0x0000000F)
#define USB_REG_LOCK_TIMEOUT      (0xFFFFFFFF)

/*
 * Since the register addresses are scattered, the access to the register
 * is made easy by the following structure pointer arrays.
 */
const PUSBSIE_REGS_T USBSIE[] =
{
    NULL,
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP1_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP2_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP3_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP4_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP5_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP6_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP7_CNT0),
    (PUSBSIE_REGS_T)&(USBFS0_USBDEV->SIE_EP8_CNT0)
};

const PUSBARB_REGS_T USBARB[] =
{
    NULL,
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP1_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP2_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP3_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP4_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP5_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP6_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP7_CFG),
    (PUSBARB_REGS_T)&(USBFS0_USBDEV->ARB_EP8_CFG)
};

const PUSBARB16_REGS_T USBARB16[] =
{
    NULL,
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW1_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW2_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW3_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW4_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW5_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW6_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW7_WA16),
    (PUSBARB16_REGS_T)&(USBFS0_USBDEV->ARB_RW8_WA16)
};

/* USB module internal handle structure */
static volatile cy_usbfs_devhandle_t gl_usb = {0};

/* Forward declaration of static functions. */
static void usb_intr_high_handler(void);

/*******************************************************************************
 * Function name: CyUsbFsCdc_ControlDataReceive
 ****************************************************************************//**
 *
 * Control the handling of data received through the USBFS CDC interface.
 * When data receive is not enabled, the driver will keep discarding any data
 * received on the OUT endpoint. When data receive is enabled, it is expected
 * that the user will queue read operations to fetch the OUT data as required.
 *
 * \param recvEnable
 * Whether data receive handling is enabled.
 *******************************************************************************/
void CyUsbFsCdc_ControlDataReceive (bool recvEnable)
{
    gl_usb.cdcRecvEnabled = recvEnable;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_Init
 ****************************************************************************//**
 *
 * The API initializes the USB interface. This is mainly a software state
 * machine initialization. The PHY is not enabled at this point. The API
 * helps to cleanup previous state information.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Init (void)
{
    cy_stc_sysint_t usb_intr_cfg;

    /* Verify that the USBFS block has not already been initialized. */
    if (gl_usb.state != USBFS_STATE_INACTIVE)
    {
        return false;
    }

    /* Initialize the USB software state machine. */
    gl_usb.state = USBFS_STATE_DISABLED;

    /* Setup the default CDC configuration. */
    gl_usb.cdcConfig[0] = 0x00;
    gl_usb.cdcConfig[1] = 0x10;
    gl_usb.cdcConfig[2] = 0x0E;
    gl_usb.cdcConfig[3] = 0x00;         /* Lower 4 bytes indicate baud rate of 0x000E1000 = 921600 */
    gl_usb.cdcConfig[4] = 0x00;         /* 0=1 stop bit, 1=1.5 stop bits, 2=2 stop bits. */
    gl_usb.cdcConfig[5] = 0x00;         /* 0=No parity, 1=Odd, 2=Even, 3=Mark, 4=Space */
    gl_usb.cdcConfig[6] = 0x08;         /* Number of data bits. */
    gl_usb.cdcConfig[7] = 0x00;

    /*
     * Setup the interrupt vectors. EP0 and USB interrupts are
     * to be configured for high priority and EPx interrupts
     * are to be configured as low priority. The interrupts
     * are not enabled at this time.
     */
    USBFS0_USBLPM->INTR_LVL_SEL = (
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_SOF_LVL_SEL_Pos) |
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_BUS_RESET_LVL_SEL_Pos) |
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_EP0_LVL_SEL_Pos) |
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_LPM_LVL_SEL_Pos) |
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_RESUME_LVL_SEL_Pos) |
            (USB_INTR_PRIORITY_HIGH << USBFS_USBLPM_INTR_LVL_SEL_EP2_LVL_SEL_Pos));

#if CY_CPU_CORTEX_M4
    usb_intr_cfg.intrSrc      = usb_interrupt_hi_IRQn;
    usb_intr_cfg.intrPriority = 4U;
    Cy_SysInt_Init(&usb_intr_cfg, usb_intr_high_handler);
#else
    usb_intr_cfg.cm0pSrc      = usb_interrupt_hi_IRQn;
    usb_intr_cfg.intrSrc      = NvicMux7_IRQn;
    usb_intr_cfg.intrPriority = 2U;
    Cy_SysInt_Init(&usb_intr_cfg, usb_intr_high_handler);
#endif /* CY_CPU_CORTEX_M4 */

    return true;
}

/* Internal function which initalizes the EP0. */
static void usb_ep0_init (void)
{
    /* Update state-machine */
    gl_usb.ep0_state = USB_EP0_STATE_SETUP;

    /* Configure the endpoint. */
    USBFS0_USBDEV->EP0_CNT = 0;
    USBFS0_USBDEV->EP0_CR  = USBDEV_EP_MODE_NAK_INOUT;
}

/* Internal function which disables EP0. */
static void usb_ep0_disable (void)
{
    /* Disable the endpoint. */
    USBFS0_USBDEV->EP0_CNT = 0;
    USBFS0_USBDEV->EP0_CR  = USBDEV_EP_MODE_DISABLE;

    /* Update the state machine. */
    gl_usb.ep0_state   = USB_EP0_STATE_DISABLED;
    gl_usb.ep0_buffer  = NULL;
    gl_usb.ep0_zlp_rqd = false;
}

/* Internal function which disables and resets all endpoints except EP0 */
static void usb_epx_disable (void)
{
    uint8_t index;

    for (index = 1; index <= USBFS_NUM_EP; index++)
    {
        USBARB[index]->arb_epx_cfg    = USBFS_USBDEV_ARB_EP1_CFG_CRC_BYPASS_Msk |
            USBFS_USBDEV_ARB_EP1_CFG_RESET_PTR_Msk;
        USBARB[index]->arb_epx_sr     = 0xFFFFFFFFUL;
        USBARB[index]->arb_epx_int_en = 0;
        USBSIE[index]->sie_epx_cr0    = USBDEV_EP_MODE_DISABLE;

        gl_usb.ep_handle[index].toggle  = false;
        gl_usb.ep_handle[index].enabled = false;
    }
}

/* Internal function which used to enable the USBFS data connection. */
static void usb_connect(void)
{
    uint32_t value;

    USBFS0_USBDEV->CR0 = USBFS_USBDEV_CR0_USB_ENABLE_Msk;

    value = (
            USBFS_USBLPM_POWER_CTL_ENABLE_DMO_Msk |
            USBFS_USBLPM_POWER_CTL_ENABLE_DPO_Msk |
            USBFS_USBLPM_POWER_CTL_SUSPEND_Msk
            );
    USBFS0_USBLPM->POWER_CTL = value;
    Cy_SysLib_DelayUs(2);

    value &= ~(USBFS_USBLPM_POWER_CTL_SUSPEND_Msk);
    USBFS0_USBLPM->POWER_CTL = value;

    /* Set the dedicated endpoint buffer sizes as 64 bytes. */
    USBFS0_USBDEV->BUF_SIZE = USBFS_BUF_SIZE_64BYTE;

    /* Disable all endpoints and configure endpoint 0. */
    usb_epx_disable();
    usb_ep0_init();

    USBFS0_USBDEV->USBIO_CR1  = 0;
    USBFS0_USBLPM->POWER_CTL |= USBFS_USBLPM_POWER_CTL_DP_UP_EN_Msk;

    /* Clear and enable the relevant USB interrupts. */
    USBFS0_USBLPM->INTR_SIE      = 0xFFFFFFFFUL;
    USBFS0_USBLPM->INTR_SIE_MASK = USBFS_USBLPM_INTR_SIE_MASK_EP0_INTR_MASK_Msk |
        USBFS_USBLPM_INTR_SIE_MASK_BUS_RESET_INTR_MASK_Msk;

    gl_usb.state = USBFS_STATE_CONNECTED;
}

/*
 * Internal function to disable the USB connection.
 */
static void usb_disconnect (void)
{
    /* Disable the USB interrupts. */
    USBFS0_USBLPM->INTR_SIE_MASK = 0;
    USBFS0_USBLPM->INTR_SIE      = 0xFFFFFFFFUL;

    usb_ep0_disable();

    USBFS0_USBLPM->INTR_SIE_MASK = 0;
    USBFS0_USBDEV->USBIO_CR1     = USBFS_USBDEV_USBIO_CR1_IOMODE_Msk;
    USBFS0_USBLPM->POWER_CTL     = 0;
    gl_usb.state = USBFS_STATE_DISABLED;

    usb_epx_disable();

    USBFS0_USBDEV->CR1 = 0;
    USBFS0_USBDEV->CR0 = 0;
    USBFS0_USBDEV->USB_CLK_EN &= ~USBFS_USBDEV_USB_CLK_EN_CSR_CLK_EN_Msk;
    Cy_SysLib_DelayUs(1);
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_Enable
 ****************************************************************************//**
 *
 * The function initializes the USB hardware and enables the D+/D- lines and does
 * a pull up on the D+ line for the external host to detect the presence of
 * the device. The API expects that the USB block is already initialized.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Enable (void)
{
    if (gl_usb.state != USBFS_STATE_DISABLED)
    {
        return false;
    }

    /* Update relevant software state machine. */
    gl_usb.dev_stat  = 0;

    /* Enable USB clock. */
    USBFS0_USBDEV->USB_CLK_EN |= USBFS_USBDEV_USB_CLK_EN_CSR_CLK_EN_Msk;
    Cy_SysLib_DelayUs(1);

    /* Enable locking of clock as well as enable the regulator. */
    USBFS0_USBDEV->CR1 = USBFS_USBDEV_CR1_REG_ENABLE_Msk | USBFS_USBDEV_CR1_ENABLE_LOCK_Msk;

    /* Disable the PS2 mode. */
    USBFS0_USBDEV->USBIO_CR1 = 0;

    /* Update the reset counter value. */
    USBFS0_USBDEV->BUS_RST_CNT = USB_BUS_RST_CNT_VALUE;

    /*
     * Enable the interrupt vectors. The actual interrupts shall get
     * enabled as per the state machine.
     */
#if CY_CPU_CORTEX_M4
    NVIC_EnableIRQ(usb_interrupt_hi_IRQn);
    NVIC_EnableIRQ(usb_interrupt_lo_IRQn);
#else
    NVIC_EnableIRQ(NvicMux7_IRQn);
#endif /* CY_CPU_CORTEX_M4 */

    /* Enable the USB connection. */
    usb_connect();
    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_Disable
 ****************************************************************************//**
 *
 * The function disables the USB hardware and disconnects the D+/D- lines.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Disable (void)
{
    if (gl_usb.state <= USBFS_STATE_DISABLED)
    {
        return false;
    }

    /* Disable the interrupt vectors. */
#if CY_CPU_CORTEX_M4
    NVIC_DisableIRQ(usb_interrupt_hi_IRQn);
    NVIC_DisableIRQ(usb_interrupt_lo_IRQn);
#else
    NVIC_DisableIRQ(NvicMux7_IRQn);
#endif /* CY_CPU_CORTEX_M4 */

    /* Ensure that the USB device module is disconnected. */
    usb_disconnect();

    /* Disconnect physically from the bus. */
    USBFS0_USBDEV->USBIO_CR1 = USBFS_USBDEV_USBIO_CR1_IOMODE_Msk;

    gl_usb.state = USBFS_STATE_DISABLED;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_GetState
 ****************************************************************************//**
 *
 * The API returns the current state of the USB device module.
 *
 * \return
 * The current USB device/stack state.
 *******************************************************************************/
cy_en_usbfs_devstate_t CyUsbFsCdc_GetState (void)
{
    return gl_usb.state;
}

/* The function queues a single packet read on EP0 */
static bool usb_ep0_queue_read (void)
{
    if (
            (gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT) ||
            ((USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk) != 0)
       )
    {
        return false;
    }

    USBFS0_USBDEV->EP0_CNT = 0;
    USBFS0_USBDEV->EP0_CR  = USBDEV_EP_MODE_ACK_OUT_STATUS_IN;
    return true;
}

/* The function reads one packet of data from EP0. It is expected that
 * the caller ensures the parameter validity and calls only when there
 * is valid data available in the endpoint buffer. */
static bool usb_ep0_read_packet (uint8_t *data, uint8_t *count)
{
    uint32_t value = USBFS0_USBDEV->EP0_CNT;
    uint8_t index;
    uint8_t fifo_cnt = (value & USBFS_USBDEV_EP0_CNT_BYTE_COUNT_Msk);
    bool toggle = ((value & USBFS_USBDEV_EP0_CNT_DATA_TOGGLE_Msk) != 0) ? true : false;
    bool status = true;

    /* Decrement by 2 to account for the CRC bytes. */
    fifo_cnt -= 2;

    if (
            (gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT) ||
            (!(value & USBFS_USBDEV_EP0_CNT_DATA_VALID_Msk)) ||
            (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk)
       )
    {
        return false;
    }

    /* Check for the correct data toggle. */
    if (!(gl_usb.ep0_toggle ^ toggle))
    {
        /* Read data from the FIFO. */
        for (index = 0; index < fifo_cnt; index++)
        {
            data[index] = (uint8_t)USBFS0_USBDEV->EP0_DR[index];
        }

        gl_usb.ep0_toggle = (gl_usb.ep0_toggle) ? false : true;
    }
    else
    {
        status = false;
    }

    /* Make the data invalid. */
    USBFS0_USBDEV->EP0_CNT = 0;

    *count = fifo_cnt;

    return status;
}

/* The function queues a single packet of data on EP0. The caller is
 * expected to ensure parameter validity and ensure that EP0 is ready
 * for transmission. */
static bool usb_ep0_send_packet (uint8_t *data, uint8_t count)
{
    uint8_t index;

    if (
            (gl_usb.ep0_state != USB_EP0_STATE_DATA_IN) ||
            ((USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk) != 0)
       )
    {
        return false;
    }

    /* Copy data into the FIFO. */
    for (index = 0; index < count; index++)
    {
        USBFS0_USBDEV->EP0_DR[index] = data[index];
    }

    /* Commit the data. */
    if (gl_usb.ep0_toggle)
    {
        USBFS0_USBDEV->EP0_CNT = (count | USBFS_USBDEV_EP0_CNT_DATA_TOGGLE_Msk);
        gl_usb.ep0_toggle = false;
    }
    else
    {
        USBFS0_USBDEV->EP0_CNT = count;
        gl_usb.ep0_toggle = true;
    }

    USBFS0_USBDEV->EP0_CR = USBDEV_EP_MODE_ACK_IN_STATUS_OUT;
    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_CompleteEp0Status
 ****************************************************************************//**
 *
 * The API completes the status phase of the current EP0 request. The caller
 * is expected to call the function in sequence. The status phase is handled
 * inplicitly when the setup_read and setup_write functions are invoked correctly
 * with the last flag set to true. The function does not wait for the transfer to
 * complete. In case of an error, the EP0 shall be stalled by the USB module.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_CompleteEp0Status (void)
{
    if ((USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk) != 0)
    {
        return false;
    }

    if ((gl_usb.setup_pkt.attrib & 0x80) != 0)
    {
        gl_usb.ep0_state = USB_EP0_STATE_STATUS_OUT;
        USBFS0_USBDEV->EP0_CR = USBDEV_EP_MODE_STATUS_OUT_ONLY;
    }
    else
    {
        gl_usb.ep0_state = USB_EP0_STATE_STATUS_IN;
        USBFS0_USBDEV->EP0_CR = USBDEV_EP_MODE_STATUS_IN_ONLY;
    }

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_StallEp0
 ****************************************************************************//**
 *
 * The function stalls endpoint zero to indicate error to current request. The
 * stall is automatically cleared on receiving a new setup request.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_StallEp0 (void)
{
    if (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk)
    {
        return false;
    }

    gl_usb.ep0_state = USB_EP0_STATE_STALL;
    USBFS0_USBDEV->EP0_CR = USBDEV_EP_MODE_STALL_INOUT;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_SetupEp0Read
 ****************************************************************************//**
 *
 * The API does not wait for the read to complete. The function just updates
 * the state machine and queues the first packet read. The read has to be
 * completed by repeatedly queueing packet read requests. The last parameter
 * can be used to do  multiple partial transfers. For default single tranfers,
 * the last parameter should always be true.
 *
 * \param data
 * Buffer to write the received USB EP0 data into. The caller should ensure
 * that the buffer is capable of receiving upto a size of length bytes.
 *
 * \param length
 * Length of data to be transferred. This has to be a multiple of eight bytes.
 *
 * \param last 
 * Whether the request is a partial transfer or not. Set to true if the stack
 * needs to implicily handle the status phase after completing the transfer.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SetupEp0Read (uint8_t *data, uint16_t length, bool last)
{
    bool status = true;

    /* All read should be multiple of EP0 size. */
    if ((data == NULL) || ((length & (USBFS_EP0_SIZE - 1)) != 0))
    {
        return false;
    }

    if ((gl_usb.ep0_state != USB_EP0_STATE_SETUP) && (gl_usb.ep0_state != USB_EP0_STATE_DATA_OUT))
    {
        return false;
    }

    /* Update state-machine. */
    gl_usb.ep0_last    = last;
    gl_usb.ep0_state   = USB_EP0_STATE_DATA_OUT;
    gl_usb.ep0_buffer  = data;
    gl_usb.ep0_length  = length;

    /* Queue a read on the endpoint */
    status = usb_ep0_queue_read();
    return status;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_SetupEp0Write
 ****************************************************************************//**
 *
 * The API does not wait for the write to complete. The function just updates
 * the state machine and queues the first packet. The write has to be completed
 * by repeatedly queueing packet requests. The last parameter can be used to do
 * multiple partial transfers. For default single tranfers, the last parameter
 * should always be true.
 *
 * \param data
 * Buffer containing the USB EP0 data to be transferred.
 *
 * \param length
 * Length of data to be transferred.
 *
 * \param last 
 * Whether the request is a partial transfer or not. Set to true if the stack
 * needs to implicily handle the status phase after completing the transfer.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SetupEp0Write (uint8_t *data, uint16_t length, bool last)
{
    uint8_t count;
    bool status = true;

    if (data == NULL)
    {
        return false;
    }

    if ((gl_usb.ep0_state != USB_EP0_STATE_SETUP) && (gl_usb.ep0_state != USB_EP0_STATE_DATA_IN))
    {
        return false;
    }

    count = (length < USBFS_EP0_SIZE) ? length : USBFS_EP0_SIZE;

    /* Update state-machine. */
    gl_usb.ep0_last   = last;
    gl_usb.ep0_state  = USB_EP0_STATE_DATA_IN;
    gl_usb.ep0_buffer = data + count;
    gl_usb.ep0_length = length - count;

    /* Check if the transfer requires a ZLP for
     * termination. This is required only when
     * the transfer length is less than the requested
     * length and the transfer length is a multiple of
     * EP0 max packet size. */
    if ((last) && (((length & (USBFS_EP0_SIZE - 1)) == 0) && (length < gl_usb.setup_pkt.length)))
    {
        gl_usb.ep0_zlp_rqd = true;
    }
    else
    {
        gl_usb.ep0_zlp_rqd = false;
    }

    /* Send out the first packet of data. */
    status = usb_ep0_send_packet(data, count);
    return status;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_Ep0AckWait
 ****************************************************************************//**
 *
 * The function does a blocking wait until the status phase is completed.
 * This function should be invoked only if the task loop can be safely blocked.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_Ep0AckWait (void)
{
    uint32_t timeout = 100000;

    /* Wait for the status to be sent. */
    while (--timeout)
    {
        if ((USBFS0_USBDEV->EP0_CR &
                    (USBFS_USBDEV_EP0_CR_IN_RCVD_Msk | USBFS_USBDEV_EP0_CR_ACKED_TXN_Msk)) ==
                (USBFS_USBDEV_EP0_CR_IN_RCVD_Msk | USBFS_USBDEV_EP0_CR_ACKED_TXN_Msk)
           )
        {
            do
            {
                USBFS0_USBDEV->EP0_CR |= USBFS_USBDEV_EP0_CR_IN_RCVD_Msk;
                Cy_SysLib_DelayUs(1);
                timeout--;

                if (timeout == 0)
                {
                    return false;
                }

            } while ((USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_IN_RCVD_Msk) != 0);

            break;
        }

        /* We need to break out of the loop if there was
         * a USB reset. In all other cases, we can afford
         * to wait for the 100ms timeout. */
        if ((USBFS0_USBLPM->INTR_SIE & USBFS_USBLPM_INTR_SIE_BUS_RESET_INTR_Msk) != 0)
        {
            return false;
        }

        Cy_SysLib_DelayUs(1);
    }

    if (timeout == 0)
    {
        return false;
    }

    return true;
}

/* Copy data from EP0 buffer to internal buffer. The caller is expected to
 * ensure parameter validity and also ensure that there is a valid setup
 * packet available in the endpoint buffer. */
static void usb_copy_setup_pkt (uint8_t *ptr)
{
    uint8_t index;

    for (index = 0; index < 8; index++)
    {
        ptr[index] = USBFS0_USBDEV->EP0_DR[index] & USBFS_USBDEV_EP0_DR_DATA_BYTE_Msk;
    }
}

/* Module defined USB EP0 standard request handler. The caller is
 * expected to ensure parameter validity. */
static bool usb_ep0_std_rqt_handler (cy_stc_usb_setup_pkt_t *pkt)
{
    uint8_t target;
    uint8_t rsplen = (uint8_t)pkt->length;
    bool status = false;

    target = (pkt->attrib & 0x03u);

    switch (pkt->cmd)
    {
        /* GET_STATUS */
        case 0x00:
            /* Until device is configured, only device specific status is to be returned. */
            if ((gl_usb.state != USBFS_STATE_CONFIGURED) && (target != 0))
            {
                break;
            }

            if (target == 0x02) /* Endpoint */
            {
                uint8_t ep = (pkt->index & 0x7F);
                uint16_t data = 0;
                if (ep <= USBFS_NUM_EP)
                {
                    if (ep != 0)
                    {
                        data = ((USBSIE[ep]->sie_epx_cr0 & USBFS_USBDEV_SIE_EP1_CR0_STALL_Msk) != 0);
                    }

                    status = CyUsbFsCdc_SetupEp0Write((uint8_t *)&data, pkt->length, true);
                }
            }
            else /* Device/Interface */
            {
                status = CyUsbFsCdc_SetupEp0Write((uint8_t *)&gl_usb.dev_stat, pkt->length, true);
            }
            break;

        /* CLEAR_FEATURE */
        case 0x01:
            /* Until device is configured, only device specific requests are supported. */
            if ((gl_usb.state != USBFS_STATE_CONFIGURED) && (target != 0))
            {
                break;
            }

            if (target == 0) /* Device specific request. */
            {
                uint8_t feature = (pkt->value & 0xFF);
                if ((feature == 1) || (feature == 2)) /* Only remote wake and test mode features are supported. */
                {
                    if (feature == 1) /* Remote wake */
                    {
                        gl_usb.dev_stat &= 0xFD;
                    }

                    status = true;
                }
            }

            if ((target == 0x02) && (pkt->value == 0)) /* EP_HALT */
            {
                status = CyUsbFsCdc_EpClearStall(pkt->index & 0x7F);
            }

            if (status == true)
            {
                status = CyUsbFsCdc_CompleteEp0Status();
            }
            break;

        /* SET_FEATURE */
        case 0x03:
            /* Until device is configured, only device specific requests are supported. */
            if ((gl_usb.state != USBFS_STATE_CONFIGURED) && (target != 0))
            {
                break;
            }

            if (target == 0)
            {
                uint8_t feature = (pkt->value & 0xFF);
                if ((feature == 1) || (feature == 2)) /* Only remote wake and test mode features are supported. */
                {
                    if (feature == 1) /* Remote wake */
                    {
                        gl_usb.dev_stat |= 0x02;
                    }

                    status = true;
                }
            }
            if ((target == 0x02) && (pkt->value == 0)) /* EP_HALT */
            {
                status = CyUsbFsCdc_EpSetStall(pkt->index & 0x7F);
            }

            if (status == true)
            {
                status = CyUsbFsCdc_CompleteEp0Status();
            }
            break;

        /* SET_ADDRESS */
        case 0x05:
            if ((pkt->value != 0) && (pkt->value <= 127U))
            {
                uint32_t value;

                /* Set the address only after responding to the setup request. */
                status = CyUsbFsCdc_CompleteEp0Status();

                if (status)
                {
                    status = CyUsbFsCdc_Ep0AckWait();

                    if (status)
                    {
                        value = USBFS0_USBDEV->CR0 & ~USBFS_USBDEV_CR0_DEVICE_ADDRESS_Msk;
                        USBFS0_USBDEV->CR0 = value | pkt->value;
                        gl_usb.state = USBFS_STATE_ADDRESSED;
                    }
                }
            }

            if (!status)
            {
                CyUsbFsCdc_StallEp0();
                status = true;
            }
            break;

        /* GET_DESCRIPTOR */
        case 0x06:
            {
                switch (pkt->value >> 8u)
                {
                    case 0x01: /* DEVICE_DESC */
                        if (rsplen > 18)
                            rsplen = 18;
                        status = CyUsbFsCdc_SetupEp0Write((uint8_t *)fsCdcDeviceDscr, rsplen, true);
                        break;

                    case 0x02: /* CONFIG_DESC */
                        if (rsplen > fsCdcConfigDscr[2])
                            rsplen = fsCdcConfigDscr[2];
                        status = CyUsbFsCdc_SetupEp0Write((uint8_t *)fsCdcConfigDscr, rsplen, true);
                        break;

                    case 0x03: /* STRING_DESC */
                        {
                            uint8_t *strPtr = NULL;
                            uint8_t  strIdx = (pkt->value & 0xFF);
                            switch (strIdx)
                            {
                                case 0:
                                    strPtr = (uint8_t *)fsCdcLangString;
                                    break;
                                case 1:
                                    strPtr = (uint8_t *)fsCdcMfgString;
                                    break;
                                case 2:
                                    strPtr = (uint8_t *)fsCdcProdString;
                                    break;
                                default:
                                    break;
                            }
                            if (strPtr != NULL)
                            {
                                if (rsplen > strPtr[0])
                                    rsplen = strPtr[0];
                                status = CyUsbFsCdc_SetupEp0Write(strPtr, rsplen, true);
                            }
                        }
                        break;
                }
            }
            break;

        /* SET_CONFIGURATION */
        case 0x09:
            /* Only one configuration supported. */
            if (pkt->value <= 1)
            {
                gl_usb.active_cfg = pkt->value;
                if (gl_usb.active_cfg != 0)
                {
                    gl_usb.state = USBFS_STATE_CONFIGURED;

                    /* Enable the CDC endpoints so that device sends NAK response on IN EPs. */
                    CyUsbFsCdc_EpEnable(1, false);
                    CyUsbFsCdc_EpEnable(2, true);
                    CyUsbFsCdc_EpEnable(3, false);

                    /* Enable data received interrupt for EP 2. */
                    USBFS0_USBDEV->SIE_EP_INT_EN |= 0x02UL;
                }
                else
                {
                    gl_usb.state = USBFS_STATE_ADDRESSED;

                    /* Disable the CDC endpoints. */
                    CyUsbFsCdc_EpDisable(1);
                    CyUsbFsCdc_EpDisable(2);
                    CyUsbFsCdc_EpDisable(3);
                }

                status = CyUsbFsCdc_CompleteEp0Status();
            }
            break;

        /* GET_CONFIGURATION */
        case 0x08:
            if (pkt->length == 1)
            {
                status = CyUsbFsCdc_SetupEp0Write((uint8_t *)&gl_usb.active_cfg, 1, true);
            }
            break;

        /* SET_INTERFACE */
        case 0x0B:
            /* No alternate setting support. Stall the request. */
            CyUsbFsCdc_StallEp0();
            status = true;
            break;

        /* GET_INTERFACE */
        case 0x0A:
            if (pkt->length == 1)
            {
                uint16_t data = 0;
                status = CyUsbFsCdc_SetupEp0Write((uint8_t *)&data, 1, true);
            }
            break;

        default:
            /* Do nothing. */
            break;
    }

    return status;
}

/* Handled for the USB-CDC class specific control requests. */
static bool usb_ep0_cdc_rqt_handler (cy_stc_usb_setup_pkt_t *pkt)
{
    bool status = false;
    uint8_t rsplen = (uint8_t)pkt->length;

    switch (pkt->cmd)
    {
        /* SET_LINE_CODING */
        case 0x20:
            if (rsplen > USBFS_EP0_SIZE)
            {
                status = CyUsbFsCdc_StallEp0();
            }
            else
            {
                rsplen = USBFS_EP0_SIZE;
                status = CyUsbFsCdc_SetupEp0Read((uint8_t *)(gl_usb.cdcConfig), rsplen, true);
            }
            break;

        /* GET_LINE_CODING */
        case 0x21:
            if (rsplen > 7)
                rsplen = 7;
            status = CyUsbFsCdc_SetupEp0Write((uint8_t *)(gl_usb.cdcConfig), rsplen, true);
            break;

        /* SET_CONTROL_LINE_STATE */
        case 0x22:
            if (gl_usb.state == USBFS_STATE_CONFIGURED)
            {
                status = CyUsbFsCdc_CompleteEp0Status();

                /* If user data receive is not enabled, enable the OUT EP so that we can fetch
                 * and discard the OUT data.
                 */
                if (!gl_usb.cdcRecvEnabled)
                {
                    CyUsbFsCdc_QueueEpRead(2);
                }
            }
            break;

        default:
            break;
    }

    return status;
}

/* Internal setup packet handler. The caller is expected to ensure
 * parameter validity. The function is expected to be invoked from
 * the EP0 interrupt handler. */
static bool usb_ep0_setup_pkt_handler (cy_stc_usb_setup_pkt_t *pkt)
{
    uint8_t type;
    bool status = false;

    /* Setup packet is already copied into the
     * SRAM buffer. */
    type = pkt->attrib & 0x60;

    if (type == 0)
    {
        /* Standard request */
        status = usb_ep0_std_rqt_handler(pkt);
    }

    if (type == 0x20)
    {
        status = usb_ep0_cdc_rqt_handler(pkt);
    }

    return status;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpEnable
 ****************************************************************************//**
 *
 * Enable an endpoint with the selected configuration.
 * The endpoint shall be initialized and configured with the provided parameters.
 * The API expects the endpoint to be in disabled state.
 *
 * \param ep_index
 * Index of endpoint to be enabled. Valid values are from 1 to 8 (inclusive).
 *
 * \param is_out
 * true if the endpoint is an OUT endpoint, false for IN endpoint.
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpEnable (uint8_t ep_index, bool is_out)
{
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;
    PUSBSIE_REGS_T sie_p;

    if (gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    arb_p = USBARB[ep_index];
    sie_p = USBSIE[ep_index];
    buf_offset = (USBFS_EP_MAX_PKT_SIZE * ep_index);

    gl_usb.ep_handle[ep_index].toggle = false;

    /* Configure the endpoint. */
    arb_p->arb_epx_cfg    = USBFS_USBDEV_ARB_EP1_CFG_RESET_PTR_Msk | USBFS_USBDEV_ARB_EP1_CFG_CRC_BYPASS_Msk;
    arb_p->arb_epx_sr     = 0xFFFFFFFFUL;
    arb_p->arb_epx_int_en = 0;

    /* 8 Bit access */
    arb_p->arb_rwx_wa     = arb_p->arb_rwx_ra     = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    /*
     * Bad hardware. Set the toggle so that the following
     * read / write call shall toggle it again.
     */
    sie_p->sie_epx_cnt0 = USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk;
    sie_p->sie_epx_cnt1 = USBFS_EP_MAX_PKT_SIZE;
    if (is_out)
    {
        USBFS0_USBDEV->EP_TYPE |= (uint8_t)(1 << (ep_index - 1));
        sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_OUT);
    }
    else /* IN EP */
    {
        USBFS0_USBDEV->EP_TYPE &= ~(uint8_t)(1 << (ep_index - 1));
        sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_IN);
    }

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].is_out = is_out;

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].enabled = true;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDisable
 ****************************************************************************//**
 *
 * Disables a previously enabled endpoint.
 * The endpoint shall be disabled and all data in the FIFO cleared. The endpoint
 * shall stop responding to requests from USB host.
 *
 * \param ep_index
 * Index of endpoint to be disabled. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDisable (uint8_t ep_index)
{
    PUSBARB_REGS_T arb_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    arb_p = USBARB[ep_index];

    /* Disable the endpoint. */
    arb_p->arb_epx_cfg    = USBFS_USBDEV_ARB_EP1_CFG_RESET_PTR_Msk | USBFS_USBDEV_ARB_EP1_CFG_CRC_BYPASS_Msk;
    arb_p->arb_epx_sr     = 0xFFFFFFFFUL;
    arb_p->arb_epx_int_en = 0;
    USBSIE[ep_index]->sie_epx_cr0 = USBDEV_EP_MODE_DISABLE;

    /* Store the configuration information. */
    gl_usb.ep_handle[ep_index].enabled = false;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_IsEpReady
 ****************************************************************************//**
 *
 * Checks whether the endpoint is ready for data transfer. The API expects that
 * the endpoint is enabled and active.
 *
 * \param ep_index
 * Endpoint to be checked
 *
 * \return
 * Endpoint status:
 *      true  - If this is an IN endpoint, then the EP is ready to send data.
 *              If there was a previous transfer then it has completed successfully.
 *              If this is an OUT endpoint, then the EP has received a packet
 *              of data from the USB host.
 *      false - If this is an IN endpoint, data is being sent out and not completed.
 *              If this is an OUT endpoint, then the data is not yet received.
 *              If the endpoint is not active or USB connection is not active,
 *              this API returns CyFalse.
 *******************************************************************************/
bool CyUsbFsCdc_IsEpReady (uint8_t ep_index)
{
    uint32_t value;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    value = USBSIE[ep_index]->sie_epx_cr0;
    if ((value & USBFS_USBDEV_SIE_EP1_CR0_STALL_Msk) != 0)
    {
        return false;
    }

    if (gl_usb.ep_handle[ep_index].is_out)
    {
        if (
                ((value & USBFS_USBDEV_SIE_EP1_CR0_MODE_Msk) == USBDEV_EP_MODE_NAK_OUT) &&
                ((value & USBFS_USBDEV_SIE_EP1_CR0_ACKED_TXN_Msk) != 0)
           )
        {
            return true;
        }
    }
    else
    {
        if ((value & USBFS_USBDEV_SIE_EP1_CR0_MODE_Msk) == USBDEV_EP_MODE_NAK_IN)
        {
            return true;
        }
    }

    return false;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpFlush
 ****************************************************************************//**
 *
 * The function resets the endpoint and re-arms an OUT endpoint to receive
 * data if a receive was already queued.
 *
 * \param ep_index
 * Index of endpoint which is to be flushed.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpFlush (uint8_t ep_index)
{
    bool is_ready;
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    buf_offset = (USBFS_EP_MAX_PKT_SIZE * ep_index);
    arb_p = USBARB[ep_index];

    is_ready = CyUsbFsCdc_IsEpReady(ep_index);
    if ((!is_ready) && (!gl_usb.ep_handle[ep_index].is_out))
    {
        if (gl_usb.ep_handle[ep_index].toggle)
        {
            gl_usb.ep_handle[ep_index].toggle = false;
        }
        else
        {
            gl_usb.ep_handle[ep_index].toggle = true;
        }

        USBSIE[ep_index]->sie_epx_cr0 = (USBDEV_EP_MODE_NAK_IN);
    }

    arb_p->arb_rwx_wa     = arb_p->arb_rwx_ra     = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    if ((gl_usb.ep_handle[ep_index].is_out) && (is_ready))
    {
        /* Update the next data toggle. */
        bool toggle = ((USBSIE[ep_index]->sie_epx_cnt0 & USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk) != 0);

        /* Check if there is a PID mismatch. */
        if (toggle == gl_usb.ep_handle[ep_index].toggle)
        {
            gl_usb.ep_handle[ep_index].toggle = (toggle) ? false : true;
        }

        CyUsbFsCdc_QueueEpRead(ep_index);
    }

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpSetStall
 ****************************************************************************//**
 *
 * The endpoint shall stall all IN / OUT tokens after the function has been
 * executed.
 *
 * \param ep_index
 * Index of endpoint to be modified. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpSetStall (uint8_t ep_index)
{
    uint32_t value;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    /* Stall the endpoint. */
    value = (gl_usb.ep_handle[ep_index].is_out) ? USBDEV_EP_MODE_ACK_OUT : USBDEV_EP_MODE_ACK_IN;
    value |= USBFS_USBDEV_SIE_EP1_CR0_STALL_Msk;
    USBSIE[ep_index]->sie_epx_cr0 = value;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpClearStall
 ****************************************************************************//**
 *
 * The function shall clear a previously stalled endpoint. The function shall
 * reset the data toggle even if the endpoint was not previously stalled.
 * This call shall  also reset the endpoint to the default state. So read
 * should be explicitly invoked for an OUT endpoint. The endpoint shall start
 * NAKing all the IN / OUT tokens once the function gets executed.
 *
 * \param ep_index
 * Index of endpoint to be modified. Valid values are from 1 to 8 (inclusive).
 *
 * \return
 * true if the action is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpClearStall (uint8_t ep_index)
{
    uint16_t buf_offset;
    PUSBARB_REGS_T arb_p;
    PUSBSIE_REGS_T sie_p;

    if (!gl_usb.ep_handle[ep_index].enabled)
    {
        return false;
    }

    buf_offset = (USBFS_EP_MAX_PKT_SIZE * ep_index);
    arb_p = USBARB[ep_index];
    sie_p = USBSIE[ep_index];
    gl_usb.ep_handle[ep_index].toggle = false;

    /* Reset the endpoint. */
    arb_p->arb_epx_cfg = USBFS_USBDEV_ARB_EP1_CFG_RESET_PTR_Msk | USBFS_USBDEV_ARB_EP1_CFG_CRC_BYPASS_Msk;
    arb_p->arb_rwx_wa     = arb_p->arb_rwx_ra     = (buf_offset & 0xFF);
    arb_p->arb_rwx_wa_msb = arb_p->arb_rwx_ra_msb = (buf_offset >> 8);

    /* Bad hardware. Set the toggle so that the following
     * read / write call shall toggle it again. */
    sie_p->sie_epx_cnt0 = USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk;
    sie_p->sie_epx_cr0  = (gl_usb.ep_handle[ep_index].is_out) ? USBDEV_EP_MODE_NAK_OUT : USBDEV_EP_MODE_NAK_IN;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_QueueEpRead
 ****************************************************************************//**
 *
 * The function enables the selected endpoint to receive one packet of data.
 * It does not wait for the data to be received.
 *
 * \param ep_index
 * Index of endpoint on which read is to be queued.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_QueueEpRead (uint8_t ep_index)
{
    PUSBSIE_REGS_T sie_p;

    sie_p = USBSIE[ep_index];
    if ((sie_p->sie_epx_cr0 & USBFS_USBDEV_SIE_EP1_CR0_MODE_Msk) != USBDEV_EP_MODE_NAK_OUT)
    {
        return false;
    }

    /* Queue a read to the endpoint. */
    sie_p->sie_epx_cnt1 = USBFS_EP_MAX_PKT_SIZE;
    sie_p->sie_epx_cnt0 = 0;
    sie_p->sie_epx_cr0  = USBDEV_EP_MODE_ACK_OUT;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDataRead
 ****************************************************************************//**
 *
 * Retrieves the data packet available on the endpoint.
 * The function expects that a data is already available and retrieves the
 * packet from the endpoint buffer.
 *
 * \param ep_index
 * Index of endpoint on which read is to be performed.
 *
 * \param data
 * Pointer to buffer into which the data should be read.
 *
 * \param count
 * Size of the data to be read (in bytes).
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDataRead (uint8_t ep_index, uint8_t *data, uint8_t *count)
{
    uint8_t index, ep_cnt;
    bool toggle;
    PUSBSIE_REGS_T sie_p;

    if ((data == NULL) || (count == NULL))
    {
        return false;
    }

    sie_p  = USBSIE[ep_index];
    toggle = ((sie_p->sie_epx_cnt0 & USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk) != 0);

    /* Check if there is a PID mismatch. */
    if (toggle ^ gl_usb.ep_handle[ep_index].toggle)
    {
        return false;
    }

    /* Read the data bytes from the USB memory. */
    ep_cnt = (sie_p->sie_epx_cnt1 - 2);
    for (index = 0; index < ep_cnt; index++)
    {
        data[index] = USBARB[ep_index]->arb_rwx_dr;
    }

    *count = ep_cnt;

    /* Update the next data toggle. */
    gl_usb.ep_handle[ep_index].toggle = (toggle) ? false : true;

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_SendZlp
 ****************************************************************************//**
 *
 * The function sends a zero-length packet on the selected IN endpoint.
 *
 * \param ep_index
 * Index of endpoint on which ZLP is to be sent.
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_SendZlp (uint8_t ep_index)
{
    PUSBSIE_REGS_T sie_p;

    sie_p = USBSIE[ep_index];

    /* Commit the data to USB. */
    sie_p->sie_epx_cnt1 = 0;
    if (gl_usb.ep_handle[ep_index].toggle)
    {
        sie_p->sie_epx_cnt0 = USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk;
        gl_usb.ep_handle[ep_index].toggle = false;

    }
    else
    {
        sie_p->sie_epx_cnt0 = 0;
        gl_usb.ep_handle[ep_index].toggle = true;
    }

    USBARB[ep_index]->arb_epx_cfg |= USBFS_USBDEV_ARB_EP1_CFG_IN_DATA_RDY_Msk;
    sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_ACK_IN);

    return true;
}

/*******************************************************************************
 * Function name: CyUsbFsCdc_EpDataWrite
 ****************************************************************************//**
 *
 * The function copies the data available in the buffer to the endpoint buffer
 * and arms the endpoint for transfer.
 *
 * \param ep_index
 * Index of endpoint on which data is to be transferred.
 *
 * \param data
 * Pointer to buffer containing the data to be written.
 *
 * \param count
 * Size of the data to be transferred (in bytes).
 *
 * \return
 * true if the operation is successful, false otherwise.
 *******************************************************************************/
bool CyUsbFsCdc_EpDataWrite (uint8_t ep_index, uint8_t *data, uint8_t count)
{
    uint8_t index;
    PUSBSIE_REGS_T sie_p;

    /* Parameter validity checks. */
    if ((data == NULL) || (gl_usb.ep_handle[ep_index].enabled == false) || (count > USBFS_EP_MAX_PKT_SIZE))
    {
        return false;
    }

    sie_p = USBSIE[ep_index];
    if ((sie_p->sie_epx_cr0 & USBFS_USBDEV_SIE_EP1_CR0_MODE_Msk) != USBDEV_EP_MODE_NAK_IN)
    {
        return false;
    }

    /* Write the data to the USB memory. */
    for (index = 0; index < count; index++)
    {
        USBARB[ep_index]->arb_rwx_dr = data[index];
    }

    /* Commit the data to USB. */
    sie_p->sie_epx_cnt1 = count;
    if (gl_usb.ep_handle[ep_index].toggle)
    {
        sie_p->sie_epx_cnt0 = USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk;
        gl_usb.ep_handle[ep_index].toggle = false;
    }
    else
    {
        sie_p->sie_epx_cnt0 = 0;
        gl_usb.ep_handle[ep_index].toggle = true;
    }

    USBARB[ep_index]->arb_epx_cfg |= USBFS_USBDEV_ARB_EP1_CFG_IN_DATA_RDY_Msk;
    sie_p->sie_epx_cr0 = (USBDEV_EP_MODE_ACK_IN);

    return true;
}

/* Internal EP0 interrupt handler */
static void usb_ep0_int_handler (void)
{
    uint32_t value;
    uint32_t timeout;
    uint8_t count = 0;
    bool status = true;

    /* Clear the interrupt. */
    USBFS0_USBLPM->INTR_SIE = USBFS_USBLPM_INTR_SIE_EP0_INTR_Msk;

    /* Check if the interrupt was triggered due to a new setup packet. */
    value = USBFS0_USBDEV->EP0_CR;

    if (value & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk)
    {
        /* Wait until the setup packet is fully received. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Any writes to the register will clear the bit. */
            USBFS0_USBDEV->EP0_CR = (USBDEV_EP_MODE_NAK_INOUT | USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk);
            timeout--;
            if (timeout == 0)
            {
                /*
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk);

        /* Copy the setup packet. */
        usb_copy_setup_pkt((uint8_t *)&gl_usb.setup_pkt);

        /* Now update the state machine. */
        gl_usb.ep0_state   = USB_EP0_STATE_SETUP;
        gl_usb.ep0_buffer  = NULL;
        gl_usb.ep0_zlp_rqd = false;
        gl_usb.ep0_toggle  = true;

        /* Stall EP0 in case of error. */
        status = usb_ep0_setup_pkt_handler((cy_stc_usb_setup_pkt_t *)&gl_usb.setup_pkt);
        if (!status)
        {
            CyUsbFsCdc_StallEp0();
        }
    }
    else if (value & USBFS_USBDEV_EP0_CR_OUT_RCVD_Msk)
    {
        /* Make sure that the interrupt is cleared. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Abort if a new SETUP token has been received. */
            if (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk)
            {
                return;
            }

            USBFS0_USBDEV->EP0_CR = (USBDEV_EP_MODE_NAK_INOUT | USBFS_USBDEV_EP0_CR_OUT_RCVD_Msk);
            timeout--;
            if (timeout == 0)
            {
                /*
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_OUT_RCVD_Msk);

        if ((gl_usb.ep0_state == USB_EP0_STATE_DATA_OUT) && (gl_usb.ep0_buffer != NULL))
        {
            /* A data packet has been received. Check
             * if additional data packets need to be
             * received. */
            status = usb_ep0_read_packet(gl_usb.ep0_buffer, &count);
            if (status)
            {
                gl_usb.ep0_buffer += count;
                gl_usb.ep0_length -= count;

                /* If this was a SLP / ZLP, this is the last transfer.
                 * Otherwise queue next read. */
                if ((count == USBFS_EP0_SIZE) && (gl_usb.ep0_length != 0))
                {
                    status = usb_ep0_queue_read();
                }
                else /* Last transfer. */
                {
                    /* Send the status phase if this is the last packet.
                     * Do not send the status phase if this was a partial
                     * request. */
                    if ((gl_usb.ep0_last) && (status))
                    {
                        status = CyUsbFsCdc_CompleteEp0Status();
                    }
                }
            }

            if (!status)
            {
                CyUsbFsCdc_StallEp0();
            }
        }
    }
    else if (value & USBFS_USBDEV_EP0_CR_IN_RCVD_Msk)
    {
        /* Make sure that the interrupt is cleared. */
        timeout = USB_REG_LOCK_TIMEOUT;
        do
        {
            /* Abort if a new SETUP token is received. */
            if (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_SETUP_RCVD_Msk)
            {
                return;
            }

            USBFS0_USBDEV->EP0_CR = (USBDEV_EP_MODE_NAK_INOUT | USBFS_USBDEV_EP0_CR_IN_RCVD_Msk);
            timeout--;
            if (timeout == 0)
            {
                /*
                 * This is never expected. If this happens, hardware is broken.
                 * Continue handling the packet so that we exit the state machine
                 */
                break;
            }

        } while (USBFS0_USBDEV->EP0_CR & USBFS_USBDEV_EP0_CR_IN_RCVD_Msk);

        if ((gl_usb.ep0_state == USB_EP0_STATE_DATA_IN) && (gl_usb.ep0_buffer != NULL))
        {
            /* A data packet has been sent out.
             * If the last packet has been sent out,
             * change to status mode. Otherwise queue
             * the next packet. */
            if (gl_usb.ep0_length != 0)
            {
                count  = (gl_usb.ep0_length < USBFS_EP0_SIZE) ? gl_usb.ep0_length : USBFS_EP0_SIZE;
                status = usb_ep0_send_packet(gl_usb.ep0_buffer, count);

                gl_usb.ep0_buffer += count;
                gl_usb.ep0_length -= count;
            }
            else if (gl_usb.ep0_zlp_rqd) /* Terminate with ZLP. */
            {
                /* Clear the flag. */
                gl_usb.ep0_zlp_rqd = false;

                status = usb_ep0_send_packet(gl_usb.ep0_buffer, 0);
            }
            else /* Last transfer. */
            {
                /* Send the status phase if this is the last packet. */
                if (gl_usb.ep0_last)
                {
                    status = CyUsbFsCdc_CompleteEp0Status();
                }
            }

            if (!status)
            {
                CyUsbFsCdc_StallEp0();
            }
        }
    }
}

/* Internal USB reset interrupt handler */
static void usb_reset_int_handler (void)
{
    /* USB_ENABLE bit is cleared on RESET and needs to be set. */
    USBFS0_USBDEV->CR0    = USBFS_USBDEV_CR0_USB_ENABLE_Msk;
    USBFS0_USBDEV->EP0_CR = USBDEV_EP_MODE_NAK_INOUT;

    gl_usb.state     = USBFS_STATE_RESET;

    /* Clear the remote wake up bit in device status. */
    gl_usb.dev_stat &= ~(0x02);

    /* Clear the interrupt. */
    USBFS0_USBLPM->INTR_SIE = USBFS_USBLPM_INTR_SIE_BUS_RESET_INTR_Msk;
}

/* Internal function to read and discard the data received on the CDC OUT endpoint. */
static void
CyUsbFsCdc_DiscardOutData (void)
{
    uint8_t dummy;
    uint8_t index, ep_cnt;
    bool toggle;
    PUSBSIE_REGS_T sie_p;

    (void)dummy;

    sie_p  = USBSIE[2];
    toggle = ((sie_p->sie_epx_cnt0 & USBFS_USBDEV_SIE_EP1_CNT0_DATA_TOGGLE_Msk) != 0);

    /* Check if there is a PID mismatch. */
    if (toggle ^ gl_usb.ep_handle[2].toggle)
    {
        return;
    }

    /* Read the data bytes from the USB memory. */
    ep_cnt = (sie_p->sie_epx_cnt1 - 2);
    for (index = 0; index < ep_cnt; index++)
    {
        dummy = USBARB[2]->arb_rwx_dr;
    }

    /* Update the next data toggle. */
    gl_usb.ep_handle[2].toggle = (toggle) ? false : true;

    /* Queue the next read operation. */
    CyUsbFsCdc_QueueEpRead(2);
}

/* Internal USB_INTR_HIGH interrupt handler */
static void usb_intr_high_handler (void)
{
    uint32_t sie_intr_status = USBFS0_USBLPM->INTR_SIE_MASKED;

    /*
     * Handle the reset interrupt in the interrupt context to
     * avoid race with GPIO and timer interrupts.
     */
    if (sie_intr_status & USBFS_USBLPM_INTR_SIE_BUS_RESET_INTR_Msk)
    {
        usb_reset_int_handler();
    }

    if (sie_intr_status & USBFS_USBLPM_INTR_SIE_EP0_INTR_Msk)
    {
        usb_ep0_int_handler();
    }

    if (USBFS0_USBLPM->INTR_CAUSE_HI & USBFS_USBLPM_INTR_CAUSE_HI_EP2_INTR_Msk)
    {
        USBFS0_USBDEV->SIE_EP_INT_SR = 0x02UL;
        if (gl_usb.cdcRecvEnabled)
        {
            Cy_Debug_HandleReadIntr();
        }
        else
        {
            CyUsbFsCdc_DiscardOutData();
        }
    }
}

#if defined(__cplusplus)
}
#endif

/* [] */
