# USB Third-Party Libraries

USB 协议栈和 USB 相关上游库统一放在这里。协议栈源码、版本和许可证属于上游项目，本仓库不对其做内部组件化改名。

当前计划纳入：

- `cherryusb/`：CherryUSB USB Host/Device 协议栈

CherryUSB 的具体 MCU/IP port 不放在本目录。CH32、PY32 或具体开发板的时钟、IRQ、DMA、GPIO 和 USB IP 适配放到 `ports/cherryusb/`；可运行验证放到 `examples/cherryusb/`。
