#ifndef QMC5883P_REG_H
#define QMC5883P_REG_H

#include <stdint.h>

#define QMC5883P_REG_CHIP_ID      0x00u
#define QMC5883P_REG_XOUT_L       0x01u
#define QMC5883P_REG_STATUS       0x09u
#define QMC5883P_REG_CONTROL_1    0x0Au
#define QMC5883P_REG_CONTROL_2    0x0Bu

#define QMC5883P_CHIP_ID_DEFAULT  0x80u

typedef enum {
    QMC5883P_MODE_SUSPEND = 0u,
    QMC5883P_MODE_NORMAL = 1u,
    QMC5883P_MODE_SINGLE = 2u,
    QMC5883P_MODE_CONTINUOUS = 3u
} qmc5883p_mode_t;

typedef enum {
    QMC5883P_ODR_10HZ = 0u,
    QMC5883P_ODR_50HZ = 1u,
    QMC5883P_ODR_100HZ = 2u,
    QMC5883P_ODR_200HZ = 3u
} qmc5883p_odr_t;

typedef enum {
    QMC5883P_OSR1_8 = 0u,
    QMC5883P_OSR1_4 = 1u,
    QMC5883P_OSR1_2 = 2u,
    QMC5883P_OSR1_1 = 3u
} qmc5883p_osr1_t;

typedef enum {
    QMC5883P_OSR2_1 = 0u,
    QMC5883P_OSR2_2 = 1u,
    QMC5883P_OSR2_4 = 2u,
    QMC5883P_OSR2_8 = 3u
} qmc5883p_osr2_t;

typedef enum {
    QMC5883P_RANGE_2G = 0u,
    QMC5883P_RANGE_8G = 1u,
    QMC5883P_RANGE_12G = 2u,
    QMC5883P_RANGE_30G = 3u
} qmc5883p_range_t;

typedef enum {
    QMC5883P_SET_RESET_ON = 0u,
    QMC5883P_SET_ONLY_ON = 1u,
    QMC5883P_SET_RESET_OFF = 2u
} qmc5883p_set_reset_mode_t;

typedef union {
    uint8_t reg;
    struct {
        uint8_t MODE : 2;
        uint8_t ODR : 2;
        uint8_t OSR1 : 2;
        uint8_t OSR2 : 2;
    } bit;
} qmc5883p_ctrl1_t;

typedef union {
    uint8_t reg;
    struct {
        uint8_t SET_RESET_MODE : 2;
        uint8_t RNG : 2;
        uint8_t RFU : 2;
        uint8_t SELF_TEST : 1;
        uint8_t SOFT_RST : 1;
    } bit;
} qmc5883p_ctrl2_t;

typedef union {
    uint8_t reg;
    struct {
        uint8_t DRDY : 1;
        uint8_t OVFL : 1;
        uint8_t RFU : 6;
    } bit;
} qmc5883p_status_t;

#endif
