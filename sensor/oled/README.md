# OLED 显示驱动说明文档

## 1. 概述

本OLED驱动为江协科技基础上调整，支持常见的SSD1306/SH1106等单色OLED显示屏，提供完整的图形显示功能。

### 1.1 主要特性

- **显示支持**：128x64, 128x32, 96x16等多种分辨率
- **接口类型**：I2C和SPI接口（可配置）
- **图形功能**：点、线、矩形、圆形、字符串显示
- **字体支持**：内置多种字体大小（6x8, 8x16, 12x24, 16x32）
- **中文支持**：支持GB2312编码的中文字符显示
- **图像显示**：支持BMP格式图像显示
- **低功耗**：支持屏幕开关、对比度调节、全屏反转等功能
- **滚动功能**：硬件支持水平和垂直滚动

### 1.2 驱动架构

```
OLED Driver
├── Core Layer (OLED.c/h)      # 核心驱动实现
├── Data Layer (OLED_Data.c/h) # 字体和图像数据
└── Examples                    # 使用示例（参考demo目录）
```

## 2. 配置选项

### 2.1 编译时配置

在 `OLED.h` 文件中可以配置以下参数：

```c
// 显示屏类型选择
#define OLED_TYPE_SSD1306       // SSD1306控制器
#define OLED_TYPE_SH1106        // SH1106控制器

// 接口类型选择
#define OLED_USE_I2C            // 使用I2C接口
#define OLED_USE_SPI            // 使用SPI接口

// 显示屏尺寸配置
#define OLED_WIDTH    128       // 显示宽度
#define OLED_HEIGHT   64        // 显示高度

// I2C地址配置（仅I2C模式）
#define OLED_I2C_ADDR 0x78      // I2C地址（通常为0x78或0x7A）

// SPI引脚配置（仅SPI模式）
#define OLED_CS_PIN   GPIO_PIN_4
#define OLED_DC_PIN   GPIO_PIN_5  
#define OLED_RST_PIN  GPIO_PIN_6
```

### 2.2 基本使用流程

```c
#include "OLED.h"

// 1. 初始化OLED显示屏
OLED_Init();

// 2. 清屏
OLED_Clear();

// 3. 显示文本
OLED_ShowString(0, 0, "Hello OLED!", 16);  // 16号字体

// 4. 显示数字
OLED_ShowNum(0, 20, 12345, 5, 12);        // 12号字体

// 5. 绘制图形
OLED_DrawLine(0, 40, 127, 40);             // 画线
OLED_DrawRectangle(10, 50, 50, 60);        // 画矩形

// 6. 刷新显示
OLED_Refresh_Gram();
```

## 3. 平台适配

### 3.1 I2C接口要求（如果使用I2C）

需要实现以下I2C操作函数：

```c
// I2C写操作
void OLED_I2C_WriteByte(uint8_t data);

// I2C写命令
void OLED_I2C_WriteCmd(uint8_t cmd);

// I2C写数据
void OLED_I2C_WriteData(uint8_t data);
```

### 3.2 SPI接口要求（如果使用SPI）

需要实现以下SPI操作函数：

```c
// SPI写字节
void OLED_SPI_WriteByte(uint8_t data);

// 控制DC引脚（命令/数据选择）
void OLED_DC_Set(bool state);

// 控制CS引脚
void OLED_CS_Set(bool state);

// 控制RST引脚（复位）
void OLED_RST_Set(bool state);
```

### 3.3 延时函数

需要提供毫秒级延时函数：

```c
void OLED_Delay_ms(uint16_t ms);
```

## 4. API参考

### 4.1 初始化函数

- `OLED_Init()` - 初始化OLED显示屏
- `OLED_Clear()` - 清空显示缓冲区
- `OLED_Refresh_Gram()` - 刷新显示内容到屏幕

### 4.2 文本显示函数

- `OLED_ShowString()` - 显示字符串
- `OLED_ShowChar()` - 显示单个字符
- `OLED_ShowNum()` - 显示数字
- `OLED_ShowChinese()` - 显示中文字符（需要GB2312字库）

### 4.3 图形绘制函数

- `OLED_DrawPoint()` - 绘制点
- `OLED_DrawLine()` - 绘制直线
- `OLED_DrawRectangle()` - 绘制矩形
- `OLED_DrawCircle()` - 绘制圆形
- `OLED_Fill()` - 填充区域

### 4.4 显示控制函数

- `OLED_Set_Pos()` - 设置显示位置
- `OLED_ON()` - 开启显示
- `OLED_OFF()` - 关闭显示
- `OLED_Set_Contrast()` - 设置对比度
- `OLED_Set_Inverse()` - 设置显示反转

### 4.5 滚动控制函数

- `OLED_HorizontalScroll()` - 水平滚动
- `OLED_VerticalScroll()` - 垂直滚动
- `OLED_StopScroll()` - 停止滚动

## 5. 示例工程

完整的使用示例请参考 `demo/xmake` 目录中的工程文件。

## 6. 注意事项

1. **电源要求**：OLED显示屏通常需要3.3V或5V电源
2. **接口选择**：根据硬件连接选择合适的接口类型（I2C或SPI）
3. **I2C地址**：不同厂商的OLED模块可能有不同的I2C地址，常见为0x78或0x7A
4. **内存占用**：128x64分辨率需要1KB显存，确保MCU有足够的RAM
5. **刷新频率**：频繁刷新会增加功耗，在静态显示应用中可降低刷新频率
6. **烧屏防护**：长时间显示静态内容可能导致烧屏，建议定期清屏或改变显示内容
7. **温度范围**：OLED在低温环境下响应速度会变慢，高温环境下寿命会缩短
8. **静电防护**：OLED对静电敏感，操作时需注意防静电措施
## Quick Include

```c
#include "OLED.h"
```
