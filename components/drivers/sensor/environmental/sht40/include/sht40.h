#ifndef SHT40_NEW_H
#define SHT40_NEW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sht40_core.h"

int sht40_init(sht40_dev_t *dev);
int sht40_soft_reset(sht40_dev_t *dev);
int sht40_read_serial(sht40_dev_t *dev, uint32_t *serial);
int sht40_read_sample(sht40_dev_t *dev, sht40_precision_t precision, sht40_sample_t *out);
int sht40_heater(sht40_dev_t *dev, sht40_heater_cmd_t cmd);

int sht40_soft_reset_async(sht40_dev_t *dev, sht40_done_cb_t cb, void *user);
int sht40_read_sample_async(sht40_dev_t *dev,
                            sht40_precision_t precision,
                            sht40_sample_cb_t cb,
                            void *user);
int sht40_cancel_async(sht40_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif
