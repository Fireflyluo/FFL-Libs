/**
 * @file sc7a20_bind.c
 * @brief SC7A20 bind 薄封装：不碰 I2C 外设，只接契约。
 */
#include "ffl_port_stm32f1_sc7a20.h"

#include "ffl_port_stm32f1_driver_port.h"

int ffl_stm32f1_sc7a20_bind(ffl_sc7a20_device_t *device,
                            const ffl_transport_t *transport,
                            uint8_t addr7) {
  int rc;

  if (device == NULL || transport == NULL) {
    return -1;
  }
  ffl_stm32f1_time_init();
  rc = ffl_sc7a20_bind(device, transport, ffl_stm32f1_time_ops(), NULL);
  if (rc != 0) {
    return rc;
  }
  return ffl_sc7a20_set_i2c_addr(device, addr7);
}
