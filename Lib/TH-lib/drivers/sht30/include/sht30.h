#ifndef SHT30_H
#define SHT30_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sht30_core.h"

int sht30_init(sht30_dev_t *dev);
int sht30_soft_reset(sht30_dev_t *dev);
int sht30_read_status(sht30_dev_t *dev, uint16_t *status);
int sht30_read_sample(sht30_dev_t *dev, sht30_repeatability_t repeatability, sht30_sample_t *out);
int sht30_heater(sht30_dev_t *dev, sht30_heater_cmd_t cmd);

int sht30_soft_reset_async(sht30_dev_t *dev, sht30_done_cb_t cb, void *user);
int sht30_read_sample_async(sht30_dev_t *dev,
                            sht30_repeatability_t repeatability,
                            sht30_sample_cb_t cb,
                            void *user);
int sht30_cancel_async(sht30_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif