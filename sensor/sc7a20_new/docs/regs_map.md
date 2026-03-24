# regs_map

## Identity

- `SC7A20_WHO_AM_I (0x0F)`, expected `SC7A20_CHIP_ID (0x11)`
- `SC7A20_VERSION (0x70)`, expected `SC7A20_VERSION_VAL (0x28)`

## Control

- `SC7A20_CTRL0 (0x1F)`: HR, OSR
- `SC7A20_CTRL1 (0x20)`: axis enable, LPen, ODR
- `SC7A20_CTRL2 (0x21)`: filter
- `SC7A20_CTRL3 (0x22)`: interrupt routing
- `SC7A20_CTRL4 (0x23)`: FS, BLE, BDU
- `SC7A20_CTRL5 (0x24)`, `SC7A20_CTRL6 (0x25)`

## Data output

- `SC7A20_OUTX_L..SC7A20_OUTZ_H (0x28..0x2D)`
- `SC7A20_OUTX_New_L..SC7A20_OUTZ_New_H (0x61..0x66)`

## Status/FIFO

- `SC7A20_DRDY_STATUS (0x27)`
- `SC7A20_FIFO_CTRL (0x2E)`
- `SC7A20_FIFO_SRC (0x2F)`
- `SC7A20_FIFO_DATA (0x69)`

## Other

- `SC7A20_SOFT_RESET (0x68)` write `0xA5`
- `SC7A20_I2C_CTRL (0x6F)`
- `SC7A20_SPI_CTRL (0x0E)`
