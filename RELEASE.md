# USBFXStack Middleware Library v1.2.0

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](./docs/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

- Updated the USB driver stack to support all USB specification compliance requirements including correct handling of USB link low power states
- Updated USB driver stack to support USB-C orientation detection on an external USB Power Delivery (PD) controller
- New LVDS/LVCMOS init function supporting all interface frequencies within the supported range
- Added capability to route debug log data through the USB Communication Device Class (CDC) interface added to the main USB device configuration
- Updated Debug Design For Test (DDFT) select function to support observation of additional LVDS/LVCMOS related signals


## Defect fixes

- Fixed occasional USB enumeration failures in different interoperability scenarios


## Known issues and limitations

- The USB stack is not fully compliant with USB specification requirements for 20 Gbps (Gen2x2) devices and rare failures may be seen in link layer test cases. Such failures will be resolved in a subsequent release of the stack. Note that all compliance requirements for USB 2.x, 10 Gbps (Gen2x1), and 5 Gbps (Gen1x1) devices are already satisfied by this version of the stack


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.4.0
CAT1A Peripheral Driver Library (PDL)                   | 3.16.0
USBFXStack Middleware Library                           | 1.2.0
FreeRTOS for Infineon MCUs                              | 10.5.002
GCC Compiler                                            | 11.3.1
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
