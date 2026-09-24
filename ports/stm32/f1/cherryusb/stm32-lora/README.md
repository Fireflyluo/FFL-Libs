# CherryUSB port：STM32F103 + STM32-LORA 板

将上游 `third_party/usb/cherryusb/upstream` 的 **fsdev** device port 接到本板原生 USB。

| 项 | 值 |
|----|-----|
| MCU | STM32F103C8T6（medium-density） |
| USB IP | 片上 FS device（fsdev，非 OTG） |
| 引脚 | PA11=DM / PA12=DP（USB2 座） |
| 时钟 | HSE 16MHz → PLL 72MHz → USB 48MHz（PLL/1.5） |
| 中断 | `USB_LP_CAN1_RX0_IRQn` |
| PMA | 512B，`PMA_ACCESS=2` |

## 文件

- `usb_config.h`：CherryUSB device 宏（EP 数、请求缓冲、PMA）
- `usb_port_stm32f1.c`：`HAL_PCD_MspInit/DeInit`（USB 时钟、NVIC）

上游 glue（`port/fsdev/usb_glue_st.c`）会调用 `HAL_PCD_MspInit`，并提供
`USB_LP_IRQHandler` → `USBD_IRQHandler(0)`。

## 使用

应用工程 `includes` 本目录，并把 `usb_port_stm32f1.c` 与上游
`usb_dc_fsdev.c`、`usb_glue_st.c`、`usbd_core.c` 及 class 源文件编入 target。
可运行示例见 `examples/cherryusb/stm32-lora-cdc/`。
