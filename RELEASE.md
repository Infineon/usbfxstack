# USBFXStack Middleware Library v1.3.0

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](./docs/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

1. Added the `Cy_LVDS_SetPhyTrainingLoopCount()` API to perform PHY training multiple times to improve the training results and reduce the probability of subsequent link training failures or data corruption. The LVDS data source (FPGA) should send the PHY training patterns for a duration of 50 µs times the number of PHY training loops selected

2. Added the `Cy_LVDS_GpifSMStop()` API to allow the GPIF state machine engine to be stopped before it restarts using the `Cy_LVDS_GpifSMStart()` function

3. Updated the High BandWidth DMA channel API to support the use of USB 2.x endpoints as data producers or consumers

    1. Added virtual sockets corresponding to the USB 2.x endpoints to the `cy_hbdma_socket_id_t` enumeration

    2. If the USB 2.x endpoints are being used with the High BandWidth DMA channel API, change the following in the application:

        1. Call the `Cy_HBDma_Mgr_RegisterUsbContext()` function to register the USBD stack instance with the DMA manager
        2. Specify the maximum packet size of the endpoint through the `usbMaxPktSize` field in the `cy_stc_hbdma_chn_config_t` structure when creating the DMA channel
        3. Set `eventEnable` to '0' and enable producer/consumer interrupts as event signaling is not supported for by USB 2.x endpoints
        4. Call the `Cy_HBDma_Mgr_HandleDW0Interrupt()` and `Cy_HBDma_Mgr_HandleDW1Interrupt()` functions from the ISRs for the DataWire0 and DataWire1 channels, which will be used for USB 2.x data transfers
        5. Call the `Cy_HBDma_Mgr_HandleUsbShortInterrupt()` function when the `CY_USB_USBD_CB_ZLP` or `CY_USB_USBD_CB_SLP` callbacks are received from the USBD stack

4. Added the `Cy_UsbFx_OnResetInit()` function, which ensures the initialization when the firmware application starts running

5. Added a callback notification when the device receives USB 2.x SOF or USB 3.x ITP messages. Register a new callback function for the `CY_USB_USBD_CB_SOF_ITP` type to receive these notifications

6. Updated USB 2.x driver to enable the use of either XTALIN or XTALOUT pin for providing the external 24 MHz clock reference for the PLL

7. Added an option to use a TCPWM timer to implement delays required in the USB 3.x driver instead of blocking the CPU execution inside the task handler


## Defect fixes

- Fixed potential race condition in handling of back-to-back EP0 control requests, which could result in a transfer timeout
- Updated USB 2.x driver to resolve electrical compliance failures observed on a few parts
- Updated USB 3.x driver to resolve link layer compliance failures in Gen2x2 and Gen1x2 modes


## Known issues and limitations

None.


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.5.0
CAT1A Peripheral Driver Library (PDL)                   | 3.16.0
FreeRTOS for Infineon MCUs                              | 10.5.002
GCC Compiler                                            | 14.2.1
Arm&reg; Compiler                                       | 6.22

<br>


## More information

For more information, see the following documents:

- [USBFXStack Middleware README.md](./README.md)

- [USBFXStack API Reference Manual](./docs/api_reference_manual.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox)

- [Infineon Technologies AG](https://www.infineon.com)

---
© 2025, Cypress Semiconductor Corporation (an Infineon Technologies company) or an affiliate of Cypress Semiconductor Corporation.
