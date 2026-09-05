#ifndef FFL_ICP20100_H
#define FFL_ICP20100_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "../icp20100.h"

#define FFL_ICP20100_DEFAULT_ADDR7 ICP20100_I2C_ADDR_AD0_LOW

typedef icp20100_dev_t ffl_icp20100_device_t;
typedef icp20100_cfg_t ffl_icp20100_config_t;
typedef icp20100_sample_t ffl_icp20100_sample_t;
typedef icp20100_raw_sample_t ffl_icp20100_raw_sample_t;
typedef icp20100_op_mode_t ffl_icp20100_op_mode_t;
typedef icp20100_meas_mode_t ffl_icp20100_meas_mode_t;
typedef icp20100_power_mode_t ffl_icp20100_power_mode_t;
typedef icp20100_fifo_mode_t ffl_icp20100_fifo_mode_t;

#define FFL_ICP20100_OP_MODE0 ICP20100_OP_MODE0
#define FFL_ICP20100_OP_MODE1 ICP20100_OP_MODE1
#define FFL_ICP20100_OP_MODE2 ICP20100_OP_MODE2
#define FFL_ICP20100_OP_MODE3 ICP20100_OP_MODE3
#define FFL_ICP20100_OP_MODE4 ICP20100_OP_MODE4

#define FFL_ICP20100_MEAS_MODE_FORCED     ICP20100_MEAS_MODE_FORCED
#define FFL_ICP20100_MEAS_MODE_CONTINUOUS ICP20100_MEAS_MODE_CONTINUOUS

#define FFL_ICP20100_POWER_MODE_NORMAL ICP20100_POWER_MODE_NORMAL
#define FFL_ICP20100_POWER_MODE_ACTIVE ICP20100_POWER_MODE_ACTIVE

#define FFL_ICP20100_FIFO_PRES_TEMP ICP20100_FIFO_PRES_TEMP
#define FFL_ICP20100_FIFO_TEMP_ONLY ICP20100_FIFO_TEMP_ONLY
#define FFL_ICP20100_FIFO_TEMP_PRES ICP20100_FIFO_TEMP_PRES
#define FFL_ICP20100_FIFO_PRES_ONLY ICP20100_FIFO_PRES_ONLY

void ffl_icp20100_config_init(ffl_icp20100_config_t *config);
int ffl_icp20100_bind(ffl_icp20100_device_t *device,
                      const ffl_transport_t *transport,
                      const ffl_time_ops_t *time_ops,
                      void *time_ctx);
int ffl_icp20100_set_i2c_addr(ffl_icp20100_device_t *device, uint8_t addr7);
int ffl_icp20100_init(ffl_icp20100_device_t *device,
                      const ffl_icp20100_config_t *config);
int ffl_icp20100_probe(ffl_icp20100_device_t *device,
                       uint8_t *chip_id,
                       uint8_t *version);
int ffl_icp20100_stop_measurement(ffl_icp20100_device_t *device);
int ffl_icp20100_configure(ffl_icp20100_device_t *device,
                            const ffl_icp20100_config_t *config);
int ffl_icp20100_read_raw(ffl_icp20100_device_t *device,
                           ffl_icp20100_raw_sample_t *raw_sample);
int ffl_icp20100_read_sample(ffl_icp20100_device_t *device,
                              ffl_icp20100_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
