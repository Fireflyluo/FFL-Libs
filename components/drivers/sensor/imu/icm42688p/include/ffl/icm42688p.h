#ifndef FFL_ICM42688P_H
#define FFL_ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ffl/driver_port.h"
#include "../imu_icm42688p.h"

#define FFL_ICM42688P_DEFAULT_ADDR7 ICM42688P_I2C_ADDR_1

typedef imu_icm42688p_t ffl_icm42688p_device_t;
typedef imu_icm42688p_cfg_t ffl_icm42688p_config_t;
typedef imu_raw_sample_t ffl_icm42688p_raw_sample_t;
typedef imu_sample_t ffl_icm42688p_sample_t;
typedef icm42688_sensor_mode_t ffl_icm42688p_sensor_mode_t;
typedef icm42688_accel_fs_t ffl_icm42688p_accel_fs_t;
typedef icm42688_gyro_fs_t ffl_icm42688p_gyro_fs_t;
typedef icm42688_odr_t ffl_icm42688p_odr_t;

#define FFL_ICM42688P_MODE_OFF       ICM42688_MODE_OFF
#define FFL_ICM42688P_MODE_STANDBY   ICM42688_MODE_STANDBY
#define FFL_ICM42688P_MODE_LOW_POWER ICM42688_MODE_LOW_POWER
#define FFL_ICM42688P_MODE_LOW_NOISE ICM42688_MODE_LOW_NOISE

#define FFL_ICM42688P_ACCEL_FS_16G ICM42688_ACCEL_FS_16G
#define FFL_ICM42688P_ACCEL_FS_8G  ICM42688_ACCEL_FS_8G
#define FFL_ICM42688P_ACCEL_FS_4G  ICM42688_ACCEL_FS_4G
#define FFL_ICM42688P_ACCEL_FS_2G  ICM42688_ACCEL_FS_2G

#define FFL_ICM42688P_GYRO_FS_2000DPS   ICM42688_GYRO_FS_2000DPS
#define FFL_ICM42688P_GYRO_FS_1000DPS   ICM42688_GYRO_FS_1000DPS
#define FFL_ICM42688P_GYRO_FS_500DPS    ICM42688_GYRO_FS_500DPS
#define FFL_ICM42688P_GYRO_FS_250DPS    ICM42688_GYRO_FS_250DPS
#define FFL_ICM42688P_GYRO_FS_125DPS    ICM42688_GYRO_FS_125DPS
#define FFL_ICM42688P_GYRO_FS_62_5DPS   ICM42688_GYRO_FS_62_5DPS
#define FFL_ICM42688P_GYRO_FS_31_25DPS  ICM42688_GYRO_FS_31_25DPS
#define FFL_ICM42688P_GYRO_FS_15_625DPS ICM42688_GYRO_FS_15_625DPS

#define FFL_ICM42688P_ODR_32000HZ  ICM42688_ODR_32000HZ
#define FFL_ICM42688P_ODR_16000HZ  ICM42688_ODR_16000HZ
#define FFL_ICM42688P_ODR_8000HZ   ICM42688_ODR_8000HZ
#define FFL_ICM42688P_ODR_4000HZ   ICM42688_ODR_4000HZ
#define FFL_ICM42688P_ODR_2000HZ   ICM42688_ODR_2000HZ
#define FFL_ICM42688P_ODR_1000HZ   ICM42688_ODR_1000HZ
#define FFL_ICM42688P_ODR_500HZ    ICM42688_ODR_500HZ
#define FFL_ICM42688P_ODR_200HZ    ICM42688_ODR_200HZ
#define FFL_ICM42688P_ODR_100HZ    ICM42688_ODR_100HZ
#define FFL_ICM42688P_ODR_50HZ     ICM42688_ODR_50HZ
#define FFL_ICM42688P_ODR_25HZ     ICM42688_ODR_25HZ
#define FFL_ICM42688P_ODR_12_5HZ   ICM42688_ODR_12_5HZ
#define FFL_ICM42688P_ODR_6_25HZ   ICM42688_ODR_6_25HZ
#define FFL_ICM42688P_ODR_3_125HZ  ICM42688_ODR_3_125HZ
#define FFL_ICM42688P_ODR_1_5625HZ ICM42688_ODR_1_5625HZ

void ffl_icm42688p_config_init(ffl_icm42688p_config_t *config);
int ffl_icm42688p_bind(ffl_icm42688p_device_t *device,
                       const ffl_transport_t *transport,
                       const ffl_time_ops_t *time_ops,
                       void *time_ctx);
int ffl_icm42688p_set_i2c_addr(ffl_icm42688p_device_t *device, uint8_t addr7);
int ffl_icm42688p_init(ffl_icm42688p_device_t *device,
                       const ffl_icm42688p_config_t *config);
int ffl_icm42688p_probe(ffl_icm42688p_device_t *device, uint8_t *who_am_i);
int ffl_icm42688p_soft_reset(ffl_icm42688p_device_t *device);
int ffl_icm42688p_configure(ffl_icm42688p_device_t *device,
                            const ffl_icm42688p_config_t *config);
int ffl_icm42688p_read_raw(ffl_icm42688p_device_t *device,
                           ffl_icm42688p_raw_sample_t *raw);
int ffl_icm42688p_read_sample(ffl_icm42688p_device_t *device,
                              ffl_icm42688p_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
