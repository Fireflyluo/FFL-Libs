#ifndef FFL_SHT40_CH32_ADAPTER_H
#define FFL_SHT40_CH32_ADAPTER_H

#include "ffl/driver_port.h"
#include "drv_i2c.h"

typedef struct {
    i2c_num_t i2c_num;
    uint32_t timeout_ms;
} ffl_sht40_ch32_i2c_t;

int ffl_sht40_ch32_transport_init(ffl_transport_t *transport,
                                   ffl_sht40_ch32_i2c_t *bus,
                                   i2c_num_t i2c_num,
                                   uint8_t addr7);
void ffl_sht40_ch32_time_init(ffl_time_ops_t *time_ops);

#endif /* FFL_SHT40_CH32_ADAPTER_H */
