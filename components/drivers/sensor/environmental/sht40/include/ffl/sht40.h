#ifndef FFL_SHT40_H
#define FFL_SHT40_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "sht40_core.h"

#define FFL_SHT40_DEFAULT_ADDR7 0x46u

typedef sht40_dev_t ffl_sht40_device_t;
typedef sht40_precision_t ffl_sht40_precision_t;
typedef sht40_heater_cmd_t ffl_sht40_heater_cmd_t;
typedef sht40_sample_t ffl_sht40_sample_t;
typedef sht40_done_cb_t ffl_sht40_done_fn;
typedef sht40_sample_cb_t ffl_sht40_sample_fn;

#define FFL_SHT40_PRECISION_HIGH   SHT40_PRECISION_HIGH
#define FFL_SHT40_PRECISION_MEDIUM SHT40_PRECISION_MEDIUM
#define FFL_SHT40_PRECISION_LOW    SHT40_PRECISION_LOW

#define FFL_SHT40_HEATER_200MW_1S    SHT40_HEATER_200MW_1S
#define FFL_SHT40_HEATER_200MW_100MS SHT40_HEATER_200MW_100MS
#define FFL_SHT40_HEATER_110MW_1S    SHT40_HEATER_110MW_1S
#define FFL_SHT40_HEATER_110MW_100MS SHT40_HEATER_110MW_100MS
#define FFL_SHT40_HEATER_20MW_1S     SHT40_HEATER_20MW_1S
#define FFL_SHT40_HEATER_20MW_100MS  SHT40_HEATER_20MW_100MS

typedef struct {
    uint8_t i2c_addr7;
} ffl_sht40_config_t;

void ffl_sht40_config_init(ffl_sht40_config_t *config);
int ffl_sht40_bind(ffl_sht40_device_t *device,
                   const ffl_transport_t *transport,
                   const ffl_time_ops_t *time_ops,
                   void *time_ctx);
int ffl_sht40_set_i2c_addr(ffl_sht40_device_t *device, uint8_t addr7);
int ffl_sht40_init(ffl_sht40_device_t *device, const ffl_sht40_config_t *config);
int ffl_sht40_soft_reset(ffl_sht40_device_t *device);
int ffl_sht40_read_serial(ffl_sht40_device_t *device, uint32_t *serial);
int ffl_sht40_read_sample(ffl_sht40_device_t *device,
                          ffl_sht40_precision_t precision,
                          ffl_sht40_sample_t *out_sample);
int ffl_sht40_heater(ffl_sht40_device_t *device, ffl_sht40_heater_cmd_t command);
int ffl_sht40_soft_reset_async(ffl_sht40_device_t *device,
                               ffl_sht40_done_fn callback,
                               void *user);
int ffl_sht40_read_sample_async(ffl_sht40_device_t *device,
                                ffl_sht40_precision_t precision,
                                ffl_sht40_sample_fn callback,
                                void *user);
int ffl_sht40_cancel_async(ffl_sht40_device_t *device);

#ifdef __cplusplus
}
#endif

#endif
