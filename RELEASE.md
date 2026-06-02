# USBFXStack Middleware Library v1.3.3

See the [README.md](./README.md) and the [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html) for a complete description of the USBFXStack.


## Feature updates

- USB API changes
  - Added a new API `Cy_USBD_GetDeviceSpeedStr()` to return the device's current operating speed as a human-readable string
  - Added a new API `Cy_USBD_GetUSBPowerState()` to return the current USB link power state (U0/U1/U2 for USB 3.x; L0/SUSPEND for USB 2.x)
  - Added a new API `Cy_USBD_EnterDeepsleep()` to prepare the USB stack and associated DMA channels for entry into device deep sleep, and to restore state after wake-up
  - Added a new API `Cy_USBD_SelectHSCtleConfig()` to select the USB 2.x High-Speed Continuous Time Linear Equalizer (CTLE) level before enabling the USB connection; also added the underlying `Cy_USBHS_Cal_SetCtleSelValue()` API and the `cy_en_hs_ctle_sel_t` enumeration (levels 0–7, stack default is `CY_USBHS_CTLE_SEL_5`)

- LVDS API changes
  - Added a new API `Cy_LVDS_IsWidelinkModeActive()` to check whether the LVDS link is operating in WideLink mode
  - Added a new API `Cy_LVDS_GetLinkState()` to retrieve the current link state (L0/L1/L3) of a specified Sensor Interface Port, along with the new `cy_en_lvds_link_state_t` enumeration
  - Added a new API `Cy_LVDS_CustomTraining_QueryResults()` to retrieve the per-lane optimal phase and passing-window-size data from a completed custom training run
  - Added new inline APIs `Cy_LVDS_SetWakeupInterruptMask()` and `Cy_LVDS_GetWakeupInterruptMask()` to manage the LVDS global wakeup interrupt mask

## Defect fixes

- Updated LVDS custom training sequence to use socket 14 instead of socket 0 (Port 0) and socket 30 instead of socket 16 (Port 1), to avoid conflicts with other DMA usage
- Fixed LVDS custom training to disable thread error interrupts before the training sequence begins, preventing spurious error callbacks during the process
- Fixed EP0 HBDma channel creation to properly release the OUT channel when IN channel allocation fails, preventing a resource leak on initialization error
- Fixed USB 2.x EP0 error recovery logic to avoid incorrectly aborting an in-flight transfer when a new control request arrives concurrently
- Fixed DMA interrupt mask usage in EP0 handlers to use `CY_USBD_DMA_INTR_MASK` consistently
- Added missing `Cy_DMA_Enable()` calls for the EP0 IN and OUT DMA controllers during USB 2.x initialization

## Known issues and limitations

None.


## Supported software and tools

This version of the USBFXStack Middleware Library is validated for the compatibility with the following software and tools:

Software and tools                                      | Version
:---                                                    | :----
ModusToolbox&trade; software environment                | 3.5.0
CAT1A Peripheral Driver Library (PDL)                   | 3.19.0
FreeRTOS for Infineon MCUs                              | 10.5.004
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
