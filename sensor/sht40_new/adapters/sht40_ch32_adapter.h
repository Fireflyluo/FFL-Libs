#ifndef SHT40_NEW_CH32_ADAPTER_H
#define SHT40_NEW_CH32_ADAPTER_H

#include "../inc/sht40_core.h"
#include "drv_i2c.h"

typedef struct {
    i2c_num_t i2c_num;
    uint8_t dev_addr;
} sht40_ch32_bus_ctx_t;

extern const sht40_bus_ops_t g_sht40_ch32_i2c_ops;

#endif
