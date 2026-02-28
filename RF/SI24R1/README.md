# SI24R1 2.4G射频芯片驱动说明文档

## 1. 概述

SI24R1是一款高性能2.4GHz无线收发芯片，兼容NRF24L01+协议。该芯片集成了完整的RF收发器、基带处理器和SPI接口，适用于无线遥控、智能家居、工业控制等2.4GHz ISM频段应用。

### 1.1 主要特性

- **工作频率**：2.400GHz - 2.483GHz（126个频道）
- **数据速率**：250kbps, 1Mbps, 2Mbps
- **输出功率**：-10dBm 至 +7dBm（8级可调）
- **接收灵敏度**：-80dBm @ 2Mbps
- **通信距离**：空旷环境下可达100米（视具体配置而定）
- **接口类型**：SPI接口（最高10MHz）
- **低功耗模式**：支持多种省电模式
- **自动应答**：硬件支持自动ACK和重传
- **FIFO缓冲**：32字节TX/RX FIFO

### 1.2 驱动架构

```
SI24R1 Driver
├── Core Layer (SI24R1.c/h)    # 核心驱动实现
└── Examples                    # 使用示例（参考demo目录）
```

## 2. 配置选项

### 2.1 设备配置结构

```c
typedef struct {
    uint8_t channel;           // RF通道 (0-125)
    uint8_t data_rate;         // 数据速率 (250K, 1M, 2M)
    uint8_t tx_power;          // 发射功率 (0-7)
    uint8_t payload_size;      // 有效载荷大小 (1-32字节)
    uint8_t address_width;     // 地址宽度 (3-5字节)
    uint8_t retry_delay;       // 重传延迟 (250μs-4ms)
    uint8_t retry_count;       // 重传次数 (0-15)
    bool auto_ack;             // 自动应答使能
    bool dynamic_payload;      // 动态载荷使能
} si24r1_config_t;
```

### 2.2 基本使用流程

```c
#include "SI24R1.h"

// 1. 初始化配置
si24r1_config_t config = {
    .channel = 40,
    .data_rate = SI24R1_DR_2MBPS,
    .tx_power = SI24R1_TX_POWER_7DBM,
    .payload_size = 32,
    .address_width = 5,
    .retry_delay = SI24R1_ARD_1500US,
    .retry_count = 15,
    .auto_ack = true,
    .dynamic_payload = false
};

// 2. 初始化设备
if (si24r1_init(&config) != SI24R1_OK) {
    // 初始化失败处理
}

// 3. 设置地址
uint8_t tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t rx_addr[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
si24r1_set_tx_address(tx_addr);
si24r1_set_rx_address(0, rx_addr); // 管道0

// 4. 发送数据
uint8_t tx_data[32] = "Hello SI24R1!";
if (si24r1_transmit(tx_data, 13) == SI24R1_OK) {
    // 发送成功
}

// 5. 接收数据
uint8_t rx_data[32];
uint8_t rx_len;
if (si24r1_receive(rx_data, &rx_len) == SI24R1_OK) {
    // 接收成功，处理数据
}
```

## 3. 平台适配

### 3.1 SPI接口要求

需要实现以下SPI操作函数：

```c
// SPI读写操作
uint8_t si24r1_spi_rw(uint8_t data);

// CS引脚控制
void si24r1_cs_set(bool state);

// CE引脚控制
void si24r1_ce_set(bool state);
```

### 3.2 延时函数

需要提供微秒和毫秒级延时函数：

```c
void si24r1_delay_us(uint16_t us);
void si24r1_delay_ms(uint16_t ms);
```

## 4. API参考

### 4.1 初始化函数

- `si24r1_init()` - 初始化SI24R1芯片
- `si24r1_soft_reset()` - 软件复位芯片
- `si24r1_set_mode()` - 设置工作模式（待机、接收、发送）

### 4.2 地址配置函数

- `si24r1_set_tx_address()` - 设置发送地址
- `si24r1_set_rx_address()` - 设置接收地址
- `si24r1_set_address_width()` - 设置地址宽度

### 4.3 数据传输函数

- `si24r1_transmit()` - 发送数据
- `si24r1_receive()` - 接收数据
- `si24r1_is_tx_done()` - 检查发送是否完成
- `si24r1_is_rx_ready()` - 检查是否有数据可接收

### 4.4 状态查询函数

- `si24r1_get_status()` - 获取芯片状态
- `si24r1_clear_irq_flags()` - 清除中断标志
- `si24r1_get_fifo_status()` - 获取FIFO状态

## 5. 示例工程

完整的使用示例请参考 `demo/xmake` 目录中的工程文件。

## 6. 注意事项

1. **电源要求**：SI24R1工作电压为1.9V-3.6V
2. **SPI时序**：SPI时钟频率不应超过10MHz
3. **天线设计**：PCB天线设计对性能影响很大，建议参考官方参考设计
4. **通道选择**：避免使用Wi-Fi常用的通道（1, 6, 11）以减少干扰
5. **功率设置**：高功率模式会增加功耗，在电池供电应用中需权衡
6. **自动重传**：启用自动重传会增加通信延迟，但提高可靠性
7. **FIFO管理**：注意FIFO溢出问题，及时读取接收数据
8. **中断处理**：建议使用中断方式处理数据收发完成事件