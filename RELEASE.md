# USBFXStack Middleware Library v1.3.1

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

- Provided a new API called `Cy_USBHS_App_IsChannelActive` to check whether any transfers are pending on USB High-Speed DataWire channel.
- Updated interrupt handling in the USB stack to allow Endpoint-0 OUT transfers to be handled completely from the callback function itself.


## Defect fixes

- Resolved potential throughput drop on the LVCMOS to USB data path seen on some FX2G3 devices.
- Corrected cases where debug output stops abruptly when the USB CDC interface is used for logging.
- Corrected sequence used to disable the USB PLL when in the USB 2.x suspended state.


## Known issues and limitations

None.


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.5.0
CAT1A Peripheral Driver Library (PDL)                   | 3.16.0
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
© 2025, Cypress Semiconductor Corporation (an Infineon Technologies company) or an affiliate of Cypress Semiconductor Corporation.
