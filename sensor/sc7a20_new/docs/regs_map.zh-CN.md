# 寄存器映射（中文）

> English version: [regs_map.md](./regs_map.md)

## 设备识别

- `SC7A20_WHO_AM_I (0x0F)`，期望值 `SC7A20_CHIP_ID (0x11)`
- `SC7A20_VERSION (0x70)`，期望值 `SC7A20_VERSION_VAL (0x28)`

## 控制寄存器

- `SC7A20_CTRL0 (0x1F)`: `HR`, `OSR`
- `SC7A20_CTRL1 (0x20)`: 三轴使能、`LPen`、`ODR`
- `SC7A20_CTRL2 (0x21)`: 滤波相关
- `SC7A20_CTRL3 (0x22)`: 中断路由
- `SC7A20_CTRL4 (0x23)`: 量程 `FS`、字节序 `BLE`、`BDU`
- `SC7A20_CTRL5 (0x24)`、`SC7A20_CTRL6 (0x25)`

## 数据输出寄存器

- `SC7A20_OUTX_L..SC7A20_OUTZ_H (0x28..0x2D)`
- `SC7A20_OUTX_New_L..SC7A20_OUTZ_New_H (0x61..0x66)`

## 状态/FIFO

- `SC7A20_DRDY_STATUS (0x27)`
- `SC7A20_FIFO_CTRL (0x2E)`
- `SC7A20_FIFO_SRC (0x2F)`
- `SC7A20_FIFO_DATA (0x69)`

## 其他

- `SC7A20_SOFT_RESET (0x68)` 写入 `0xA5`
- `SC7A20_I2C_CTRL (0x6F)`
- `SC7A20_SPI_CTRL (0x0E)`
