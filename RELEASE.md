# USBFXStack Middleware Library v1.2.1

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](./docs/api_reference_manual.html) for a complete description of the USBFXStack.


## Defect fixes

- Fixed implementation of the `Cy_LVDS_GpifSMControl()` function so that the GPIF waveform is not invalidated when being resumed
- Updated fault handler module to handle cases where FreeRTOS is disabled
- Fixed build warnings when compiling with GCC 14.2.1 toolchain

## Known issues and limitations

- The USB stack is not fully compliant with USB specification requirements for 20 Gbps (Gen2x2) devices and rare failures may be seen in link layer test cases. Such failures will be resolved in a subsequent release of the stack. Note that all compliance requirements for USB 2.x, 10 Gbps (Gen2x1), and 5 Gbps (Gen1x1) devices are already satisfied by this version of the stack


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.5.0
CAT1A Peripheral Driver Library (PDL)                   | 3.16.0
USBFXStack Middleware Library                           | 1.2.1
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
