#ifndef FFL_QMI8658A_H
#define FFL_QMI8658A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "../imu_qmi8658a.h"

#define FFL_QMI8658A_DEFAULT_ADDR7 0x6Au

typedef imu_qmi8658a_t ffl_qmi8658a_device_t;
typedef imu_qmi8658a_cfg_t ffl_qmi8658a_config_t;
typedef imu_raw_sample_t ffl_qmi8658a_raw_sample_t;
typedef imu_sample_t ffl_qmi8658a_sample_t;
typedef qmi8658a_accel_fs_t ffl_qmi8658a_accel_fs_t;
typedef qmi8658a_accel_odr_t ffl_qmi8658a_accel_odr_t;
typedef qmi8658a_gyro_fs_t ffl_qmi8658a_gyro_fs_t;
typedef qmi8658a_gyro_odr_t ffl_qmi8658a_gyro_odr_t;

#define FFL_QMI8658A_ACCEL_FS_2G  QMI8658A_ACCEL_FS_2G
#define FFL_QMI8658A_ACCEL_FS_4G  QMI8658A_ACCEL_FS_4G
#define FFL_QMI8658A_ACCEL_FS_8G  QMI8658A_ACCEL_FS_8G
#define FFL_QMI8658A_ACCEL_FS_16G QMI8658A_ACCEL_FS_16G

#define FFL_QMI8658A_ACCEL_ODR_7174_4HZ QMI8658A_ACCEL_ODR_7174_4HZ
#define FFL_QMI8658A_ACCEL_ODR_3587_2HZ QMI8658A_ACCEL_ODR_3587_2HZ
#define FFL_QMI8658A_ACCEL_ODR_1793_6HZ QMI8658A_ACCEL_ODR_1793_6HZ
#define FFL_QMI8658A_ACCEL_ODR_896_8HZ  QMI8658A_ACCEL_ODR_896_8HZ
#define FFL_QMI8658A_ACCEL_ODR_448_4HZ  QMI8658A_ACCEL_ODR_448_4HZ
#define FFL_QMI8658A_ACCEL_ODR_224_2HZ  QMI8658A_ACCEL_ODR_224_2HZ
#define FFL_QMI8658A_ACCEL_ODR_112_1HZ  QMI8658A_ACCEL_ODR_112_1HZ
#define FFL_QMI8658A_ACCEL_ODR_56_05HZ  QMI8658A_ACCEL_ODR_56_05HZ
#define FFL_QMI8658A_ACCEL_ODR_28_025HZ QMI8658A_ACCEL_ODR_28_025HZ

#define FFL_QMI8658A_GYRO_FS_16DPS   QMI8658A_GYRO_FS_16DPS
#define FFL_QMI8658A_GYRO_FS_32DPS   QMI8658A_GYRO_FS_32DPS
#define FFL_QMI8658A_GYRO_FS_64DPS   QMI8658A_GYRO_FS_64DPS
#define FFL_QMI8658A_GYRO_FS_128DPS  QMI8658A_GYRO_FS_128DPS
#define FFL_QMI8658A_GYRO_FS_256DPS  QMI8658A_GYRO_FS_256DPS
#define FFL_QMI8658A_GYRO_FS_512DPS  QMI8658A_GYRO_FS_512DPS
#define FFL_QMI8658A_GYRO_FS_1024DPS QMI8658A_GYRO_FS_1024DPS
#define FFL_QMI8658A_GYRO_FS_2048DPS QMI8658A_GYRO_FS_2048DPS

#define FFL_QMI8658A_GYRO_ODR_7174_4HZ QMI8658A_GYRO_ODR_7174_4HZ
#define FFL_QMI8658A_GYRO_ODR_3587_2HZ QMI8658A_GYRO_ODR_3587_2HZ
#define FFL_QMI8658A_GYRO_ODR_1793_6HZ QMI8658A_GYRO_ODR_1793_6HZ
#define FFL_QMI8658A_GYRO_ODR_896_8HZ  QMI8658A_GYRO_ODR_896_8HZ
#define FFL_QMI8658A_GYRO_ODR_448_4HZ  QMI8658A_GYRO_ODR_448_4HZ
#define FFL_QMI8658A_GYRO_ODR_224_2HZ  QMI8658A_GYRO_ODR_224_2HZ
#define FFL_QMI8658A_GYRO_ODR_112_1HZ  QMI8658A_GYRO_ODR_112_1HZ
#define FFL_QMI8658A_GYRO_ODR_56_05HZ  QMI8658A_GYRO_ODR_56_05HZ

void ffl_qmi8658a_config_init(ffl_qmi8658a_config_t *config);
int ffl_qmi8658a_bind(ffl_qmi8658a_device_t *device,
                      const ffl_transport_t *transport,
                      const ffl_time_ops_t *time_ops,
                      void *time_ctx);
int ffl_qmi8658a_set_i2c_addr(ffl_qmi8658a_device_t *device, uint8_t addr7);
int ffl_qmi8658a_init(ffl_qmi8658a_device_t *device,
                      const ffl_qmi8658a_config_t *config);
int ffl_qmi8658a_probe(ffl_qmi8658a_device_t *device, uint8_t *who_am_i);
int ffl_qmi8658a_soft_reset(ffl_qmi8658a_device_t *device);
int ffl_qmi8658a_configure(ffl_qmi8658a_device_t *device,
                           const ffl_qmi8658a_config_t *config);
int ffl_qmi8658a_read_raw(ffl_qmi8658a_device_t *device,
                          ffl_qmi8658a_raw_sample_t *raw);
int ffl_qmi8658a_read_sample(ffl_qmi8658a_device_t *device,
                             ffl_qmi8658a_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
