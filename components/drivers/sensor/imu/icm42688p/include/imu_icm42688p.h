#ifndef IMU_ICM42688P_H
#define IMU_ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "ffl/driver_port.h"
#include "icm42688_reg.h"
#include "imu_bus.h"
#include "imu_types.h"

typedef struct {
    icm42688_sensor_mode_t accel_mode;
    icm42688_sensor_mode_t gyro_mode;
    icm42688_accel_fs_t accel_fs;
    icm42688_odr_t accel_odr;
    icm42688_gyro_fs_t gyro_fs;
    icm42688_odr_t gyro_odr;
} imu_icm42688p_cfg_t;

typedef struct {
    const imu_bus_ops_t *bus_ops;
    void *bus_ctx;
    const ffl_transport_t *transport;
    const ffl_time_ops_t *time_ops;
    void *time_ctx;
    imu_delay_ms_fn delay_ms;
    void *delay_ctx;

    uint8_t addr;
    uint8_t chip_id;
    uint8_t current_bank;
    bool initialized;
    volatile uint8_t in_use;

    imu_icm42688p_cfg_t cfg;
} imu_icm42688p_t;

extern const imu_icm42688p_cfg_t g_imu_icm42688p_default_cfg;

int imu_icm42688p_init(imu_icm42688p_t *dev, const imu_icm42688p_cfg_t *cfg);
int imu_icm42688p_probe(imu_icm42688p_t *dev, uint8_t *who_am_i);
int imu_icm42688p_soft_reset(imu_icm42688p_t *dev);
int imu_icm42688p_read_reg(imu_icm42688p_t *dev,
                           icm42688_bank_t bank,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len);
int imu_icm42688p_write_reg(imu_icm42688p_t *dev,
                            icm42688_bank_t bank,
                            uint8_t reg,
                            const uint8_t *data,
                            uint16_t len);
int imu_icm42688p_read_raw(imu_icm42688p_t *dev, imu_raw_sample_t *raw);
int imu_icm42688p_read_sample(imu_icm42688p_t *dev, imu_sample_t *sample);
int imu_icm42688p_set_accel_config(imu_icm42688p_t *dev,
                                   icm42688_accel_fs_t fs,
                                   icm42688_odr_t odr);
int imu_icm42688p_set_gyro_config(imu_icm42688p_t *dev,
                                  icm42688_gyro_fs_t fs,
                                  icm42688_odr_t odr);
int imu_icm42688p_configure(imu_icm42688p_t *dev,
                            const imu_icm42688p_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif
