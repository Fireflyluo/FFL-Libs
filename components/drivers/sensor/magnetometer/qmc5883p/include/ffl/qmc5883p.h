#ifndef FFL_QMC5883P_H
#define FFL_QMC5883P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "../qmc5883p.h"

#define FFL_QMC5883P_DEFAULT_ADDR7 0x2Cu

typedef qmc5883p_dev_t ffl_qmc5883p_device_t;
typedef qmc5883p_cfg_t ffl_qmc5883p_config_t;
typedef qmc5883p_vec3f_t ffl_qmc5883p_vec3f_t;
typedef qmc5883p_vec3i16_t ffl_qmc5883p_vec3i16_t;
typedef qmc5883p_mode_t ffl_qmc5883p_mode_t;
typedef qmc5883p_odr_t ffl_qmc5883p_odr_t;
typedef qmc5883p_osr1_t ffl_qmc5883p_osr1_t;
typedef qmc5883p_osr2_t ffl_qmc5883p_osr2_t;
typedef qmc5883p_range_t ffl_qmc5883p_range_t;
typedef qmc5883p_set_reset_mode_t ffl_qmc5883p_set_reset_mode_t;

#define FFL_QMC5883P_MODE_SUSPEND    QMC5883P_MODE_SUSPEND
#define FFL_QMC5883P_MODE_NORMAL     QMC5883P_MODE_NORMAL
#define FFL_QMC5883P_MODE_SINGLE     QMC5883P_MODE_SINGLE
#define FFL_QMC5883P_MODE_CONTINUOUS QMC5883P_MODE_CONTINUOUS

#define FFL_QMC5883P_ODR_10HZ  QMC5883P_ODR_10HZ
#define FFL_QMC5883P_ODR_50HZ  QMC5883P_ODR_50HZ
#define FFL_QMC5883P_ODR_100HZ QMC5883P_ODR_100HZ
#define FFL_QMC5883P_ODR_200HZ QMC5883P_ODR_200HZ

#define FFL_QMC5883P_OSR1_8 QMC5883P_OSR1_8
#define FFL_QMC5883P_OSR1_4 QMC5883P_OSR1_4
#define FFL_QMC5883P_OSR1_2 QMC5883P_OSR1_2
#define FFL_QMC5883P_OSR1_1 QMC5883P_OSR1_1

#define FFL_QMC5883P_OSR2_1 QMC5883P_OSR2_1
#define FFL_QMC5883P_OSR2_2 QMC5883P_OSR2_2
#define FFL_QMC5883P_OSR2_4 QMC5883P_OSR2_4
#define FFL_QMC5883P_OSR2_8 QMC5883P_OSR2_8

#define FFL_QMC5883P_RANGE_30G QMC5883P_RANGE_30G
#define FFL_QMC5883P_RANGE_12G QMC5883P_RANGE_12G
#define FFL_QMC5883P_RANGE_8G  QMC5883P_RANGE_8G
#define FFL_QMC5883P_RANGE_2G  QMC5883P_RANGE_2G

#define FFL_QMC5883P_SET_RESET_ON  QMC5883P_SET_RESET_ON
#define FFL_QMC5883P_SET_ONLY_ON   QMC5883P_SET_ONLY_ON
#define FFL_QMC5883P_SET_RESET_OFF QMC5883P_SET_RESET_OFF

void ffl_qmc5883p_config_init(ffl_qmc5883p_config_t *config);
int ffl_qmc5883p_bind(ffl_qmc5883p_device_t *device,
                      const ffl_transport_t *transport);
int ffl_qmc5883p_set_i2c_addr(ffl_qmc5883p_device_t *device, uint8_t addr7);
int ffl_qmc5883p_init(ffl_qmc5883p_device_t *device,
                      const ffl_qmc5883p_config_t *config);
int ffl_qmc5883p_probe(ffl_qmc5883p_device_t *device, uint8_t *chip_id);
int ffl_qmc5883p_soft_reset(ffl_qmc5883p_device_t *device);
int ffl_qmc5883p_configure(ffl_qmc5883p_device_t *device,
                            const ffl_qmc5883p_config_t *config);
int ffl_qmc5883p_read_raw(ffl_qmc5883p_device_t *device,
                           ffl_qmc5883p_vec3i16_t *sample);
int ffl_qmc5883p_read_ut(ffl_qmc5883p_device_t *device,
                          ffl_qmc5883p_vec3f_t *sample);

#ifdef __cplusplus
}
#endif

#endif
