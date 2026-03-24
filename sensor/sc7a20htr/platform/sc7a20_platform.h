/**
 ******************************************************************************
 * @file    sc7a20_platform.h
 * @brief   SC7A20 platform adaptation interface
 ******************************************************************************
 */
#ifndef SC7A20_PLATFORM_H
#define SC7A20_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sc7a20_core.h"

int accel_init(void);
int accel_read_data(sc7a20_accel_data_t *accel_data);

#if SC7A20_ASYNC_SUPPORT
int accel_init_async(void);
#endif

void read_acceleration_data(void);

#ifdef __cplusplus
}
#endif

#endif /* SC7A20_PLATFORM_H */
