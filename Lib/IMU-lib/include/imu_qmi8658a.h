#ifndef IMU_QMI8658A_H
#define IMU_QMI8658A_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "imu_bus.h"
#include "imu_types.h"
#include "qmi8658a_reg.h"

typedef struct {
    uint8_t addr;
    qmi8658a_accel_fs_t accel_fs;
    qmi8658a_accel_odr_t accel_odr;
    qmi8658a_gyro_fs_t gyro_fs;
    qmi8658a_gyro_odr_t gyro_odr;
    bool enable_accel;
    bool enable_gyro;
    bool enable_auto_increment;
    bool enable_sync_sample;
    bool accel_lpf_enable;
    uint8_t accel_lpf_mode;
    bool gyro_lpf_enable;
    uint8_t gyro_lpf_mode;
} imu_qmi8658a_cfg_t;

typedef struct {
    const imu_bus_ops_t *bus_ops;
    void *bus_ctx;
    imu_delay_ms_fn delay_ms;
    void *delay_ctx;

    uint8_t addr;
    uint8_t chip_id;
    bool initialized;

    imu_qmi8658a_cfg_t cfg;
} imu_qmi8658a_t;

extern const imu_qmi8658a_cfg_t g_imu_qmi8658a_default_cfg;

int imu_qmi8658a_init(imu_qmi8658a_t *dev, const imu_qmi8658a_cfg_t *cfg);
int imu_qmi8658a_probe(imu_qmi8658a_t *dev, uint8_t *who_am_i);
int imu_qmi8658a_soft_reset(imu_qmi8658a_t *dev);
int imu_qmi8658a_read_reg(imu_qmi8658a_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
int imu_qmi8658a_write_reg(imu_qmi8658a_t *dev, uint8_t reg, const uint8_t *data, uint16_t len);
int imu_qmi8658a_read_raw(imu_qmi8658a_t *dev, imu_raw_sample_t *raw);
int imu_qmi8658a_read_sample(imu_qmi8658a_t *dev, imu_sample_t *sample);
int imu_qmi8658a_set_accel_config(imu_qmi8658a_t *dev,
                                  qmi8658a_accel_fs_t fs,
                                  qmi8658a_accel_odr_t odr);
int imu_qmi8658a_set_gyro_config(imu_qmi8658a_t *dev,
                                 qmi8658a_gyro_fs_t fs,
                                 qmi8658a_gyro_odr_t odr);

#ifdef __cplusplus
}
#endif

#endif
