#include "ffl/icm42688p.h"

int main()
{
    ffl_icm42688p_config_t config{};

    ffl_icm42688p_config_init(&config);
    return config.accel_fs == FFL_ICM42688P_ACCEL_FS_4G &&
                   config.gyro_fs == FFL_ICM42688P_GYRO_FS_500DPS
               ? 0
               : 1;
}
