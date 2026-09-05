#ifndef FFL_SHT30_H
#define FFL_SHT30_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "sht30_core.h"

#define FFL_SHT30_DEFAULT_ADDR7 0x44u

typedef sht30_dev_t ffl_sht30_device_t;
typedef sht30_repeatability_t ffl_sht30_repeatability_t;
typedef sht30_heater_cmd_t ffl_sht30_heater_cmd_t;
typedef sht30_sample_t ffl_sht30_sample_t;
typedef sht30_done_cb_t ffl_sht30_done_fn;
typedef sht30_sample_cb_t ffl_sht30_sample_fn;

#define FFL_SHT30_REPEATABILITY_HIGH   SHT30_PRECISION_HIGH
#define FFL_SHT30_REPEATABILITY_MEDIUM SHT30_PRECISION_MEDIUM
#define FFL_SHT30_REPEATABILITY_LOW    SHT30_PRECISION_LOW

#define FFL_SHT30_HEATER_ENABLE  SHT30_HEATER_ENABLE
#define FFL_SHT30_HEATER_DISABLE SHT30_HEATER_DISABLE

typedef struct {
    uint8_t i2c_addr7;
} ffl_sht30_config_t;

void ffl_sht30_config_init(ffl_sht30_config_t *config);
int ffl_sht30_bind(ffl_sht30_device_t *device,
                   const ffl_transport_t *transport,
                   const ffl_time_ops_t *time_ops,
                   void *time_ctx);
int ffl_sht30_set_i2c_addr(ffl_sht30_device_t *device, uint8_t addr7);
int ffl_sht30_init(ffl_sht30_device_t *device, const ffl_sht30_config_t *config);
int ffl_sht30_soft_reset(ffl_sht30_device_t *device);
int ffl_sht30_read_status(ffl_sht30_device_t *device, uint16_t *status);
int ffl_sht30_read_sample(ffl_sht30_device_t *device,
                          ffl_sht30_repeatability_t repeatability,
                          ffl_sht30_sample_t *out_sample);
int ffl_sht30_heater(ffl_sht30_device_t *device, ffl_sht30_heater_cmd_t command);
int ffl_sht30_soft_reset_async(ffl_sht30_device_t *device,
                               ffl_sht30_done_fn callback,
                               void *user);
int ffl_sht30_read_sample_async(ffl_sht30_device_t *device,
                                ffl_sht30_repeatability_t repeatability,
                                ffl_sht30_sample_fn callback,
                                void *user);
int ffl_sht30_cancel_async(ffl_sht30_device_t *device);

#ifdef __cplusplus
}
#endif

#endif
