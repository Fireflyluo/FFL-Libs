# stm32-lora-cdc：CherryUSB CDC 板端验证

在 STM32-LORA 板（STM32F103C8T6）上用 CherryUSB `v1.6.1` fsdev port
枚举 **CDC ACM 虚拟串口**，验证第三方协议栈接入。

> 板级总览：[`../../stm32-lora/README.md`](../../stm32-lora/README.md)

## 仓库边界

| 层级 | 路径 | 说明 |
|------|------|------|
| 上游源码 | `third_party/usb/cherryusb/upstream` | **不修改** |
| MCU port | `ports/stm32/f1/cherryusb/stm32-lora` | USB 48MHz、NVIC、`usb_config.h` |
| 可运行示例 | 本目录 | CDC 描述符 + 测速触发 |

F103 要点：**USBCLK=48MHz**（PLL/1.5）；**D+ 必须 1.5k 上拉到 3.3V**（无片内上拉）。

## 实测吞吐

| 方向 | 结果 |
|------|------|
| IN（设备→PC） | **≈622 KB/s（5.1 Mbps）** |
| OUT（PC→设备） | **≈474 KB/s（3.9 Mbps）** |

主机：枚举 `FF55:5710`，打开 COM + DTR；发 `S` 满速 IN，发 `X` 停止。

## 依赖

| 项 | 路径 |
|----|------|
| CherryUSB 上游 | `third_party/usb/cherryusb/upstream` |
| 板级 port | `ports/stm32/f1/cherryusb/stm32-lora` |
| STM32 SDK | `examples/stm32-base-driver/sdk`（复用） |

## 接线

- USB2 座：PA11=DM / PA12=DP，接到 PC USB 口
- 调试：USART1 PA9 → PowerWriter COM4 @115200

## 构建

```powershell
xmake f -P examples/cherryusb/stm32-lora-cdc -p cross --toolchain=arm-none-eabi -a arm -m release
xmake    -P examples/cherryusb/stm32-lora-cdc
```

产物：`build/cross/arm/release/stm32-lora-cdc.bin`

## 烧录

```powershell
openocd -s <scripts> -f interface/cmsis-dap.cfg -f target/stm32f1x.cfg -c "..."
# 见仓库维护脚本或 examples/stm32-base-driver README
```

## 验证现象

1. PC 设备管理器出现 `STM32-LORA CherryUSB CDC`（VID `FF55` PID `5710`）
2. 打开新串口并置 DTR 后，每 1s 收到 `[cdc] t=.. out=.. in=..`
3. 向该串口写入任意字节，固件原样回显
4. COM4 打印 `[boot]` / `[usb] usbd_initialize done`

## 边界

- 仅 device CDC，未测 host / MSC / 其它 class
- 未改 `upstream/`；时钟与 NVIC 在 `ports/stm32/f1/cherryusb/stm32-lora`
