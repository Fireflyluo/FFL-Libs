#include "ffl/qmi8658a.h"

int main()
{
    ffl_qmi8658a_config_t config{};
    ffl_qmi8658a_config_init(&config);
    return config.enable_accel && config.enable_gyro ? 0 : 1;
}
