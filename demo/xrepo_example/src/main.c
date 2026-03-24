#include "sht40_hal.h"

// Desktop/demo stubs: replace with real I2C + delay on your MCU.
uint8_t SHT40_I2C_Write(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len) {
    (void)i2c_num; (void)addr; (void)data; (void)len;
    return 0;
}

uint8_t SHT40_I2C_Read(uint8_t i2c_num, uint8_t addr, uint8_t *data, uint8_t len) {
    (void)i2c_num; (void)addr; (void)data; (void)len;
    return 0;
}

void SHT40_Delay(uint32_t ms) {
    (void)ms;
}

int main(void) {
    float temperature = 0.0f;
    float humidity = 0.0f;

    SHT40_Init();
    (void)SHT40_Read_Temperature_Humidity(&temperature, &humidity);

    return 0;
}
