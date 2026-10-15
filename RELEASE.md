# USBFXStack Middleware Library v1.3.4

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

- USB Stack and API changes
  - Added additional candidate coefficients for use in the USB 3.2 Gen1 (5 Gbps) receiver equalizer adaptation process
  - Updated stack to deliver the `suspendCb` callback when USB 3.x link enters the `U3` state in addition to the USB 2.x suspend state
  - Updated the output format used by the `Cy_USBSS_Cal_PrintCtleResults()` debug function to print all values in a single line
  - Added a callback interface that automatically enables or disables the debug logging interface when the main USB link is used for logging

- LVDS API changes
  - Added `Cy_LVDS_WriteEgressRegister()` and `Cy_LVDS_ReadIngressRegister()` functions to enable single-word data transfers across the LVCMOS interface using data registers instead of DMA sockets
  - Reduced timeouts used during LVDS/LVCMOS initialization to approximately 10 ms
  - Updated L3 entry and exit functions to support the LVDS link loopback use case

- Utility function changes
  - Updated the infrastructure to simplify the use of CDC interface added to the device USB function for logging. Functions are provided to update the configuration descriptors with the CDC interface descriptors, handle the CDC class-specific requests, and automatically enable or disable the logging interface as the state of the USB link changes

## Defect fixes

- Corrected the `Cy_LVDS_GpifSMSwitch` API to support repeated state machine switch operations
- Corrected LVDS driver to allow custom link training on LVDS Port-1 when Port-0 is not in use
- Corrected the `Cy_Debug_QueueDataRead()` API to work properly when USBFS interface is being used for logging


## Known issues and limitations

None.


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.8.0
CAT1A Peripheral Driver Library (PDL)                   | 3.23.0
FreeRTOS for Infineon MCUs                              | 10.6.202
GCC Compiler                                            | 14.2.1
Arm&reg; Compiler                                       | 6.22

<br>


## More information

For more information, see the following documents:

- [USBFXStack Middleware README.md](./README.md)

- [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880004&aid=ai010888)

- [USB 3.2 peripheral controller webinar](https://www.infineon.com/event/webinar/2025/usb1020gbps?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=usbfxstack&uid=ci0108880002&aid=ai010888)

- [USB 2.0 peripheral controller webinar](https://www.infineon.com/event/webinar/2026/usb-2-0-platforms-for-imaging-scanning-biometrics?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=usbfxstack&uid=ci0108880001&aid=ai010888)

- [Infineon Technologies AG](https://www.infineon.com)

---
© 2026, Cypress Semiconductor Corporation (an Infineon Technologies company) or an affiliate of Cypress Semiconductor Corporation.
