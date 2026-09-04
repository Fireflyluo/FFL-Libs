#ifndef SC7A20_NEW_CH32_ADAPTER_H
#define SC7A20_NEW_CH32_ADAPTER_H

#include "../inc/sc7a20_core.h"
#include "drv_i2c.h"

typedef struct {
    i2c_num_t i2c_num;
    uint8_t dev_addr;
} sc7a20_ch32_bus_ctx_t;

extern const sc7a20_bus_ops_t g_sc7a20_ch32_i2c_ops;

#endif
