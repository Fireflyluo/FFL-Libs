# XN297L 2.4G射频芯片驱动说明文档

## 1. 概述

XN297L是一款低功耗2.4GHz无线收发芯片，兼容NRF24L01协议。该芯片专为电池供电的物联网设备设计，在保持良好通信性能的同时实现了超低功耗特性，适用于无线传感器网络、智能家居、可穿戴设备等应用。

### 1.1 主要特性

- **协议兼容**：兼容NRF24L01寄存器和指令集
- **工作频率**：2.400GHz - 2.483GHz（126个频道）
- **数据速率**：250kbps, 1Mbps, 2Mbps
- **超低功耗**：接收电流典型值13.5mA，发送电流典型值11.3mA（0dBm）
- **输出功率**：-20dBm 至 +4dBm（多级可调）
- **接收灵敏度**：-89dBm @ 250kbps, -82dBm @ 1Mbps, -79dBm @ 2Mbps
- **通信距离**：空旷环境下可达80米（低速率模式）
- **接口类型**：SPI接口（最高10MHz）
- **睡眠电流**：<1μA（深度睡眠模式）
- **自动应答**：硬件支持自动ACK和重传
- **FIFO缓冲**：32字节TX/RX FIFO
- **快速启动**：从睡眠到接收模式仅需130μs

### 1.2 驱动架构

```
XN297L Driver
├── Core Layer (xn297L.c/h)    # 核心驱动实现
└── Examples                    # 使用示例（参考demo目录）
```

## 2. 配置选项

### 2.1 设备配置结构

```c
typedef struct {
    uint8_t channel;           // RF通道 (0-125)
    uint8_t data_rate;         // 数据速率 (250K, 1M, 2M)
    uint8_t tx_power;          // 发射功率等级
    uint8_t payload_size;      // 有效载荷大小 (1-32字节)
    uint8_t address_width;     // 地址宽度 (3-5字节)
    uint8_t retry_delay;       // 自动重传延迟
    uint8_t retry_count;       // 自动重传次数 (0-15)
    bool auto_ack;             // 自动应答使能
    bool dynamic_payload;      // 动态载荷使能
    bool low_power_mode;       // 超低功耗模式使能
} xn297l_config_t;
```

### 2.2 基本使用流程

```c
#include "xn297L.h"

// 1. 初始化配置
xn297l_config_t config = {
    .channel = 50,
    .data_rate = XN297L_DR_250KBPS,
    .tx_power = XN297L_TX_POWER_0DBM,
    .payload_size = 8,
    .address_width = 5,
    .retry_delay = XN297L_ARD_1000US,
    .retry_count = 5,
    .auto_ack = true,
    .dynamic_payload = false,
    .low_power_mode = true
};

// 2. 初始化设备
if (xn297l_init(&config) != XN297L_OK) {
    // 初始化失败处理
}

// 3. 配置地址
uint8_t rx_addr[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
uint8_t tx_addr[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

xn297l_set_rx_address(0, rx_addr);  // 管道0接收地址
xn297l_set_tx_address(tx_addr);     // 发送地址

// 4. 进入接收模式
xn297l_set_mode(XN297L_MODE_RX);

// 5. 发送数据
uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
if (xn297l_transmit(tx_data, 8) == XN297L_OK) {
    // 发送成功
}

// 6. 接收数据
uint8_t rx_data[32];
uint8_t rx_len;
if (xn297l_receive(rx_data, &rx_len) == XN297L_OK) {
    // 接收成功，处理数据
}

// 7. （可选）进入深度睡眠模式以节省功耗
xn297l_set_mode(XN297L_MODE_DEEP_SLEEP);
```

## 3. 平台适配

### 3.1 SPI接口要求

需要实现以下SPI操作函数：

```c
// SPI读写操作
uint8_t xn297l_spi_rw(uint8_t data);

// CS引脚控制
void xn297l_cs_set(bool state);

// CE引脚控制
void xn297l_ce_set(bool state);

// IRQ中断处理（推荐使用）
void xn297l_irq_handler(void);
```

### 3.2 延时函数

需要提供微秒和毫秒级延时函数：

```c
void xn297l_delay_us(uint16_t us);
void xn297l_delay_ms(uint16_t ms);
```

## 4. API参考

### 4.1 初始化函数

- `xn297l_init()` - 初始化XN297L芯片
- `xn297l_soft_reset()` - 软件复位芯片
- `xn297l_set_mode()` - 设置工作模式（待机、接收、发送、深度睡眠）

### 4.2 配置函数

- `xn297l_set_channel()` - 设置RF通道
- `xn297l_set_data_rate()` - 设置数据速率
- `xn297l_set_tx_power()` - 设置发射功率
- `xn297l_set_address_width()` - 设置地址宽度
- `xn297l_enable_low_power()` - 启用超低功耗模式

### 4.3 地址管理函数

- `xn297l_set_tx_address()` - 设置发送地址
- `xn297l_set_rx_address()` - 设置接收管道地址
- `xn297l_enable_rx_pipe()` - 使能/禁用接收管道

### 4.4 数据传输函数

- `xn297l_transmit()` - 发送数据包
- `xn297l_receive()` - 接收数据包
- `xn297l_flush_tx()` - 清空发送FIFO
- `xn297l_flush_rx()` - 清空接收FIFO

### 4.5 电源管理函数

- `xn297l_enter_deep_sleep()` - 进入深度睡眠模式
- `xn297l_wake_up()` - 从深度睡眠唤醒
- `xn297l_get_power_status()` - 获取电源状态

### 4.6 状态查询函数

- `xn297l_get_status()` - 获取芯片状态寄存器
- `xn297l_clear_irq_flags()` - 清除中断标志
- `xn297l_get_fifo_status()` - 获取FIFO状态
- `xn297l_is_tx_done()` - 检查发送是否完成
- `xn297l_is_rx_ready()` - 检查是否有数据可接收

## 5. 示例工程

完整的使用示例请参考 `demo/xmake` 目录中的工程文件。

## 6. 注意事项

1. **电源要求**：XN297L工作电压为1.9V-3.6V
2. **低功耗优化**：在电池供电应用中，合理使用深度睡眠模式可显著延长电池寿命
3. **唤醒时间**：从深度睡眠模式唤醒需要约130μs，应用层需考虑此延迟
4. **天线设计**：PCB天线设计对通信距离影响很大，建议参考官方参考设计
5. **数据速率选择**：低数据速率（250kbps）提供更好的接收灵敏度和通信距离
6. **功率与功耗平衡**：根据实际通信距离需求选择合适的发射功率，避免不必要的功耗
7. **FIFO溢出**：在高数据率应用中，注意及时处理接收FIFO以避免数据丢失
8. **中断优先级**：IRQ中断应设置为较高优先级，确保及时处理数据收发事件
9. **ESD防护**：RF引脚对静电敏感，建议添加TVS二极管等ESD保护元件
10. **温度补偿**：在宽温度范围应用中，可能需要进行频率漂移补偿