# USBFXStack Middleware Library 1.3.4

The USBFXStack Middleware Library provides a powerful and versatile solution designed to support the EZ-USB&trade; FX devices with a variety of integrated modules. This library includes essential drivers and utilities to help developers efficiently manage and optimize their applications.

As best practices and development flows are continuously evolving, it is recommended to periodically sync the latest updates from this library to ensure you are using the most recent improvements and features.

## Supported EZ-USB&trade; FX devices

**USB 3.2 devices:**

- [EZ-USB&trade; FX20](https://www.infineon.com/products/universal-serial-bus/usb-3-2-peripheral-controllers/ez-usb-fx20-usb-20-gbps-peripheral-controller?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880008&aid=ai010888): Up to 20 Gbps, USB 3.2 Gen2x2
- [EZ-USB&trade; FX10](https://www.infineon.com/products/universal-serial-bus/usb-3-2-peripheral-controllers/ez-usb-fx10-fx5n-usb-10-gbps-peripheral-controller?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880007&aid=ai010888): Up to 10 Gbps, USB 3.2 Gen2
- [EZ-USB&trade; FX5N](https://www.infineon.com/products/universal-serial-bus/usb-3-2-peripheral-controllers/ez-usb-fx10-fx5n-usb-10-gbps-peripheral-controller?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880007&aid=ai010888): Up to 10 Gbps, USB 3.2 Gen1x2
- [EZ-USB&trade; FX5](https://www.infineon.com/products/universal-serial-bus/usb-3-2-peripheral-controllers/ez-usb-fx5-usb-5-gbps-peripheral-controller?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880006&aid=ai010888): Up to 5 Gbps, USB 3.2 Gen1

**USB 2.0 devices:**

- [EZ-USB&trade; FX2G3](https://www.infineon.com/products/universal-serial-bus/usb-2-0-peripheral-controllers/ez-usb-fx2g3-usb-2-0-peripheral-controller?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880005&aid=ai010888): Up to 480 Mbps, USB 2.0

## Features

- **LVDS/LVCMOS IP driver:** Provides a driver for the Sensor Interface Port (SIP) on the EZ-USB&trade; FX device that can be configured to operate using the LVDS/LVCMOS interface for efficient data transfer

- **USB stack:** An integrated USB stack that supports up to USB 20 Gbps functionality on the EZ-USB&trade; FX device, enabling robust USB communication capabilities

- **DMA manager:** Offers generic DMA channel abstraction for managing DMA resources, facilitating data movement across USB and LVDS/LVCMOS interfaces on the EZ-USB&trade; FX device

- **Utility functions:** A set of utility functions providing core functionalities such as logging support, USB data transfer abstraction, and fault handling mechanisms


## More information

For further details and documentation, see the following resources:

- [USBFXStack API Reference Manual](https://infineon.github.io/usbfxstack/api_reference_manual.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.infineon.com/modustoolbox?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=usbfxstack&uid=ci0108880004&aid=ai010888)

- [USB 3.2 peripheral controller webinar](https://www.infineon.com/event/webinar/2025/usb1020gbps?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=usbfxstack&uid=ci0108880002&aid=ai010888)

- [USB 2.0 peripheral controller webinar](https://www.infineon.com/event/webinar/2026/usb-2-0-platforms-for-imaging-scanning-biometrics?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=usbfxstack&uid=ci0108880001&aid=ai010888)

- [Infineon Technologies AG](https://www.infineon.com)


---------------------------------------------------------

(c) 2026, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
This software, associated documentation and materials ("Software") is owned by Infineon Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection, worldwide copyright laws, and international treaty provisions. Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software. If no license agreement applies, then any use, reproduction, modification, translation, or compilation of this Software is prohibited without the express written permission of Infineon.
<br>
Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. Infineon reserves the right to make changes to the Software without notice. You are responsible for properly designing, programming, and testing the functionality and safety of your intended application of the Software, as well as complying with any legal requirements related to its use. Infineon does not guarantee that the Software will be free from intrusion, data theft or loss, or other breaches (“Security Breaches”), and Infineon shall have no liability arising out of any Security Breaches. Unless otherwise explicitly approved by Infineon, the Software may not be used in any application where a failure of the Product or any consequences of the use thereof can reasonably be expected to result in personal injury.
