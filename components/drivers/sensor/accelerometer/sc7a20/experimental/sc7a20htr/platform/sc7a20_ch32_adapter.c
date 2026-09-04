#include "sc7a20_platform.h"

#include "board.h"

#include "drv_i2c.h" //具体的i2c驱动自行实现

static sc7a20_dev_t g_accel_dev;
static uint8_t g_accel_i2c_addr = SC7A20_I2C_ADDR_H;

static int accel_i2c_wait_idle(i2c_num_t i2c_num, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t err;
    uint32_t guard = 0;

    while (1)
    {
        if (bsp_i2c_get_state(i2c_num) == I2C_STATE_IDLE)
        {
            err = bsp_i2c_get_error(i2c_num);
            return (err == I2C_OK) ? I2C_OK : (int)err;
        }

        err = bsp_i2c_get_error(i2c_num);
        if (err != I2C_OK)
        {
            return (int)err;
        }

        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            bsp_i2c_recover(i2c_num);
            return I2C_ERR_TIMEOUT;
        }

        guard++;
        if (guard > 2000000U)
        {
            bsp_i2c_recover(i2c_num);
            return I2C_ERR_TIMEOUT;
        }
    }
}

static sc7a20_status_t accel_i2c_write_reg(uint8_t reg, const uint8_t *data, uint16_t len)
{
    int ret;

    ret = bsp_i2c_write_register_it(I2C_NUM_1, g_accel_i2c_addr, reg, data, len);
    if (ret == I2C_OK)
    {
        ret = accel_i2c_wait_idle(I2C_NUM_1, 50U);
    }

    return (ret == I2C_OK) ? SC7A20_OK : SC7A20_COMM_ERROR;
}

static sc7a20_status_t accel_i2c_read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;

    ret = bsp_i2c_read_register_it(I2C_NUM_1, g_accel_i2c_addr, reg, data, len);
    if (ret == I2C_OK)
    {
        ret = accel_i2c_wait_idle(I2C_NUM_1, 50U);
    }

    return (ret == I2C_OK) ? SC7A20_OK : SC7A20_COMM_ERROR;
}

static sc7a20_ops_t g_accel_ops = {
    .write = accel_i2c_write_reg,
    .read = accel_i2c_read_reg,
    .delay_ms = Delay_Ms,
    .user_data = NULL,
};

int accel_init(void)
{
    sc7a20_config_t config = {
        .i2c_addr = SC7A20_I2C_ADDR_H,
        .range = SC7A20_ACCEL_FS_2G,
        .odr = SC7A20_ACCEL_ODR_50HZ,
        .enable_axis = {1, 1, 1},
        .block_data_update = true,
        .high_resolution_mode = true,
        .low_power_mode = false,
    };
    sc7a20_status_t status;

    g_accel_i2c_addr = config.i2c_addr;
    status = sc7a20_init(&g_accel_dev, &g_accel_ops, &config);
    if (status != SC7A20_OK)
    {
        config.i2c_addr = SC7A20_I2C_ADDR_L;
        g_accel_i2c_addr = config.i2c_addr;
        status = sc7a20_init(&g_accel_dev, &g_accel_ops, &config);
    }

    if (status != SC7A20_OK)
    {
        printf("SC7A20: ERR!! %d\n", status);
        return -1;
    }

    printf("SC7A20: OK! addr=0x%02X\n", g_accel_i2c_addr);
    return 0;
}

int accel_read_data(sc7a20_accel_data_t *accel_data)
{
    sc7a20_status_t status;

    if (accel_data == NULL)
    {
        return -1;
    }

    status = sc7a20_read_acceleration(&g_accel_dev, accel_data);
    if (status != SC7A20_OK)
    {
        return -2;
    }

    return 0;
}

void read_acceleration_data(void)
{
    sc7a20_accel_data_t accel_data;
    sc7a20_status_t status;
    bool data_ready = true;

    status = sc7a20_is_data_ready(&g_accel_dev, &data_ready);
    if (status == SC7A20_OK && data_ready)
    {
        status = sc7a20_read_acceleration(&g_accel_dev, &accel_data);
        if (status == SC7A20_OK)
        {
            printf("Accel[g]: X=%f, Y=%f, Z=%f\n", accel_data.x_g, accel_data.y_g, accel_data.z_g);
            printf("Accel raw: X=%d, Y=%d, Z=%d\n", accel_data.x, accel_data.y, accel_data.z);
        }
    }
}
