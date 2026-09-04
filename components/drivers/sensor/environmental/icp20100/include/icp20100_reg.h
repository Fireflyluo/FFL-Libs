#ifndef ICP20100_REG_H
#define ICP20100_REG_H

#include <stdint.h>

#define ICP20100_REG_DEVICE_ID      0x0Cu
#define ICP20100_REG_MODE_SELECT    0xC0u
#define ICP20100_REG_FIFO_FILL      0xC4u
#define ICP20100_REG_VERSION        0xD3u
#define ICP20100_REG_FIFO_BASE      0xFAu

#define ICP20100_REG_DUMMY_INIT     0xEEu
#define ICP20100_REG_DUMMY_VALUE    0xF0u

#define ICP20100_DEVICE_ID_DEFAULT  0x63u
#define ICP20100_I2C_ADDR_AD0_LOW   0x63u
#define ICP20100_I2C_ADDR_AD0_HIGH  0x64u

typedef enum {
    ICP20100_OP_MODE0 = 0u,
    ICP20100_OP_MODE1 = 1u,
    ICP20100_OP_MODE2 = 2u,
    ICP20100_OP_MODE3 = 3u,
    ICP20100_OP_MODE4 = 4u
} icp20100_op_mode_t;

typedef enum {
    ICP20100_MEAS_MODE_FORCED = 0u,
    ICP20100_MEAS_MODE_CONTINUOUS = 1u
} icp20100_meas_mode_t;

typedef enum {
    ICP20100_POWER_MODE_NORMAL = 0u,
    ICP20100_POWER_MODE_ACTIVE = 1u
} icp20100_power_mode_t;

typedef enum {
    ICP20100_FIFO_PRES_TEMP = 0u,
    ICP20100_FIFO_TEMP_ONLY = 1u,
    ICP20100_FIFO_TEMP_PRES = 2u,
    ICP20100_FIFO_PRES_ONLY = 3u
} icp20100_fifo_mode_t;

typedef union {
    uint8_t reg;
    struct {
        uint8_t FIFO_READOUT_MODE : 2;
        uint8_t POWER_MODE : 1;
        uint8_t MEAS_MODE : 1;
        uint8_t FORCED_MEAS_TRIGGER : 1;
        uint8_t MEAS_CONFIG : 3;
    } bit;
} icp20100_mode_select_t;

typedef union {
    uint8_t reg;
    struct {
        uint8_t FIFO_LEVEL : 5;
        uint8_t FIFO_FULL : 1;
        uint8_t FIFO_EMPTY : 1;
        uint8_t FIFO_FLUSH : 1;
    } bit;
} icp20100_fifo_fill_t;

#endif
