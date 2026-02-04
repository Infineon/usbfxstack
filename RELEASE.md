# USBFXStack Middleware Library v1.3.2

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

- USB API changes
  - Added a `Cy_USBD_ControlUsb2Support()` API, which allows the fallback from USB 3.x connection to USB 2.x to be disabled. A callback function can be registered with the type of `CY_USB_USBD_CB_USB3_LINKFAIL` to get notified when USB 3.x link initialization has failed and fallback to USB 2.x is disabled
  - Added a `Cy_USBD_GetDeviceStats()` API to get a count of USB link recovery cycles that have taken place
  - Updated the USBD stack so that USB 3.x re-connection attempt after a USB 2.x bus reset is done from the stack task instead of directly from the timer callback. This resolves a potential race condition between operations performed in USB application callbacks and the link re-initialization done by the stack.

- LVDS API changes
  - Implemented a firmware-based PHY training sequence, which is a more robust alternative to the default training performed by the LVDS block. The firmware-based training scheme can be enabled by calling the `Cy_LVDS_CustomTraining_Select()` function with the `customTrainEn` parameter set to true, before the LVDS block is initialized by calling `Cy_LVDS_Init()` or `Cy_LVDS_InitExt().`<br>
     If the firmware-based training is enabled, the state machine will take over and complete the training at the end of the `Cy_LVDS_PhyTrainingStart()` call. The state of the training task can be monitored using the `Cy_LVDS_CustomTraining_Task()` function. The FPGA LVDS transmitter needs to keep sending the PHY training pattern on all data and control lanes until this training process is completed.<br>
     Use of the firmware-based training process is recommended to ensure that the LVDS receiver works correctly and receives all data properly after the training process.
  - Added an `Cy_LVDS_OverridePhyTraining()` API, which allows the results of the LVDS PHY training to be overridden with a static sampling delay value to be used


## Defect fixes

- Updated the `Cy_Debug_LogDeInit()` function to properly shut down the USBFS port if it is used for logging
- Updated DataWire DMA configuration to resolve intermittent throughput drop seen in a few  USB 2.x use cases
- Updated endpoint flush and reset functions to resolve corner case failures
- Updated the `Cy_USBSS_Cal_GetBusOrientation()` function to avoid errors due to ongoing USB-PD communication


## Known issues and limitations

None.


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.5.0
CAT1A Peripheral Driver Library (PDL)                   | 3.18.0
FreeRTOS for Infineon MCUs                              | 10.5.004
GCC Compiler                                            | 14.2.1
Arm&reg; Compiler                                       | 6.22

<br>


## More information

For more information, see the following documents:

- [USBFXStack Middleware README.md](./README.md)

- [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox)

- [Infineon Technologies AG](https://www.infineon.com)

---
© 2026, Cypress Semiconductor Corporation (an Infineon Technologies company) or an affiliate of Cypress Semiconductor Corporation.
