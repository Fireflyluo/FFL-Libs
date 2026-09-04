#ifndef IMU_BUS_H
#define IMU_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IMU_BUS_MSG_WRITE (1u << 0)
#define IMU_BUS_MSG_READ  (1u << 1)
#define IMU_BUS_MSG_STOP  (1u << 2)

typedef struct {
    uint8_t *buf;
    uint16_t len;
    uint8_t flags;
} imu_bus_msg_t;

typedef void (*imu_bus_done_cb_t)(void *user, int status);

typedef struct {
    int (*xfer)(void *ctx,
                const imu_bus_msg_t *msgs,
                uint8_t cnt,
                imu_bus_done_cb_t cb,
                void *user);
    int (*cancel)(void *ctx);
} imu_bus_ops_t;

typedef void (*imu_delay_ms_fn)(void *ctx, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
