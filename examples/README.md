# 使用示例

`examples/` 放置可以复制、检查和改造的最小工程。示例不会代替正式组件文档，但能展示 Xmake target、组件依赖和 port 的组合方式。

## 当前示例

### stm32-base-driver（STM32-LORA 主示例，已合并）

目录：[stm32-base-driver](stm32-base-driver/readme.md)

STM32F103C8T6 交叉编译固件，用 **OSAL 多任务** 演示仓库组件与板级外设：

| 任务文件 | 演示内容 |
|----------|----------|
| `app/heartbeat_task.c` | LED + 蜂鸣（60s 两声） |
| `app/sc7a20_task.c` | `ffl.sc7a20` + `ports/stm32/f1` I2C + `ffl.ringbuffer` |
| `app/flash_task.c` | 板载 W25Q 初始化 / 就绪探测 |
| `app/usb_img.c` | **USB CDC** 收图写入外部 Flash（日志走 COM4） |
| `app/lcd_task.c` | ST7735 从 W25Q 刷屏，**5s 换图** |
| `app/features_demo.c` | `ffl.atomic` / ringbuffer / sw_timer 功能点 |

南向适配：[`ports/stm32/f1`](../ports/stm32/f1/)（osal 临界区、I2C transport、CherryUSB）。  
任务详解：`stm32-base-driver/docs/tasks/`。

### cherryusb/stm32-lora-cdc

目录：[cherryusb/stm32-lora-cdc](cherryusb/stm32-lora-cdc/readme.md)

CherryUSB CDC 最小验证与吞吐测速（已并入 base-driver 的 USB 收图路径）。

### driver-dual-entry

目录：[driver-dual-entry](driver-dual-entry/)

展示 SC7A20 的两种接入方式（component / quick-package / source-trimmed）。

## 自己创建示例

建议每个示例只回答一个问题，并在 README 中写明：

- 使用的组件 target；
- 使用的 MCU、板卡和工具链；
- 应用必须提供的 bus、GPIO、IRQ、时间或临界区回调；
- host/mock 与真实硬件验证的区别。

第三方库可运行接入放在这里，上游源码仍按 [third_party/README.md](../third_party/README.md)。
