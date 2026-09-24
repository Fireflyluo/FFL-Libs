/**
 * @file usb_config.h
 * @brief CherryUSB device 配置：STM32F103C8 fsdev + CDC ACM。
 *
 * F103 medium-density：PMA 512B，PMA_ACCESS=2（与 HAL PCD 一致）。
 */
#ifndef USB_CONFIG_H
#define USB_CONFIG_H

#include <stdio.h>

#define CONFIG_USB_PRINTF(...) printf(__VA_ARGS__)
#define CONFIG_USB_DBG_LEVEL USB_DBG_ERROR
#define CONFIG_USB_ALIGN_SIZE 4
#define USB_NOCACHE_RAM_SECTION

#define CONFIG_USBDEV_MAX_BUS 1
#define CONFIG_USBDEV_MAX_EP_NUM 8

#ifndef CONFIG_USBDEV_EP_NUM
#define CONFIG_USBDEV_EP_NUM 8
#endif

#ifndef CONFIG_USBDEV_REQUEST_BUFFER_LEN
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 256
#endif

#define CONFIG_USBDEV_FSDEV_PMA_ACCESS 2
#ifndef CONFIG_USB_FSDEV_RAM_SIZE
#define CONFIG_USB_FSDEV_RAM_SIZE 512
#endif

#define USB_DEVICE_VERSION 0x0100

#endif /* USB_CONFIG_H */
