#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature;
} imu_raw_sample_t;

typedef struct {
    float accel_mps2[3];
    float gyro_rads[3];
    float temperature_c;
    uint32_t timestamp_ms;
} imu_sample_t;

#ifdef __cplusplus
}
#endif

#endif
