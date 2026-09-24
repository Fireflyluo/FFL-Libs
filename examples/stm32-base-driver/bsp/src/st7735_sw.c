/**
 * @file st7735_sw.c
 * @brief ST7735 软 SPI 驱动（PA5=SCK, PA7=MOSI, PA4=CS；PB0=RES, PB1=DC, PB13=BL）。
 *
 * 与 w25q_hw.c（硬 SPI1 remap 读 W25Q）分工：
 *   - Flash：硬 SPI1 + DMA1_CH2（高速，18MHz）；
 *   - LCD：本文件 BSRR 软驱（约 2–3 MHz，32 字节/行开销小）。
 * 不再触碰 AFIO_MAPR：避免与 w25q_hw 的一次性 remap 互相打断。
 */
#include "st7735.h"

#include "stm32f1xx_hal.h"

#define LCD_CS_LOW()  (GPIOA->BRR = GPIO_PIN_4)
#define LCD_CS_HIGH() (GPIOA->BSRR = GPIO_PIN_4)
#define LCD_DC_CMD()  (GPIOB->BRR = GPIO_PIN_1)
#define LCD_DC_DATA() (GPIOB->BSRR = GPIO_PIN_1)
#define LCD_RES_LOW() (GPIOB->BRR = GPIO_PIN_0)
#define LCD_RES_HIGH() (GPIOB->BSRR = GPIO_PIN_0)
#define LCD_BL_ON()   (GPIOB->BSRR = GPIO_PIN_13)
#define LCD_SCK_H()   (GPIOA->BSRR = GPIO_PIN_5)
#define LCD_SCK_L()   (GPIOA->BRR = GPIO_PIN_5)
#define LCD_MOSI_H()  (GPIOA->BSRR = GPIO_PIN_7)
#define LCD_MOSI_L()  (GPIOA->BRR = GPIO_PIN_7)

static void dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t dwt_us(void) {
  return (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock);
}

/** Mode0 软发 1 字节（只发不收，LCD 无 MISO）。 */
static void sw_xfer8(uint8_t out) {
  int i;
  for (i = 7; i >= 0; i--) {
    if ((out >> i) & 1) {
      LCD_MOSI_H();
    } else {
      LCD_MOSI_L();
    }
    LCD_SCK_H();
    __asm volatile("nop\n nop");
    LCD_SCK_L();
  }
}

static void spi_tx(const uint8_t *data, uint32_t len) {
  uint32_t i;
  for (i = 0; i < len; i++) {
    sw_xfer8(data[i]);
  }
}

static void cmd(uint8_t c) {
  LCD_DC_CMD();
  LCD_CS_LOW();
  spi_tx(&c, 1);
  LCD_CS_HIGH();
}

static void data8(uint8_t v) {
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(&v, 1);
  LCD_CS_HIGH();
}

static void bus_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_4; /* SCK, MOSI, CS */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  LCD_CS_HIGH();
  LCD_SCK_L();
  LCD_MOSI_L();

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13; /* RES, DC, BL */
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  LCD_RES_HIGH();
  LCD_DC_CMD();
  LCD_BL_ON();
}

void st7735_bus_acquire(void) {
  static int sw_ready;
  if (!sw_ready) {
    bus_gpio_init();
    dwt_init();
    sw_ready = 1;
  }
  LCD_BL_ON();
}

#define LCD_COL_OFF 2 /* 1.8" 模块常见列偏移，0 可改 */
#define LCD_ROW_OFF 1

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t b[4];
  x0 = (uint16_t)(x0 + LCD_COL_OFF);
  x1 = (uint16_t)(x1 + LCD_COL_OFF);
  y0 = (uint16_t)(y0 + LCD_ROW_OFF);
  y1 = (uint16_t)(y1 + LCD_ROW_OFF);
  cmd(0x2A);
  b[0] = (uint8_t)(x0 >> 8);
  b[1] = (uint8_t)x0;
  b[2] = (uint8_t)(x1 >> 8);
  b[3] = (uint8_t)x1;
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(b, 4);
  LCD_CS_HIGH();

  cmd(0x2B);
  b[0] = (uint8_t)(y0 >> 8);
  b[1] = (uint8_t)y0;
  b[2] = (uint8_t)(y1 >> 8);
  b[3] = (uint8_t)y1;
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(b, 4);
  LCD_CS_HIGH();

  cmd(0x2C);
}

void st7735_init(void) {
  st7735_bus_acquire();
  LCD_RES_LOW();
  HAL_Delay(20);
  LCD_RES_HIGH();
  HAL_Delay(120);

  cmd(0x01); /* SWRESET */
  HAL_Delay(150);
  cmd(0x11); /* SLPOUT */
  HAL_Delay(120);

  cmd(0xB1);
  data8(0x01);
  data8(0x2C);
  data8(0x2D);
  cmd(0xB2);
  data8(0x01);
  data8(0x2C);
  data8(0x2D);
  cmd(0xB3);
  data8(0x01);
  data8(0x2C);
  data8(0x2D);
  data8(0x01);
  data8(0x2C);
  data8(0x2D);
  cmd(0xB4);
  data8(0x07);
  cmd(0xC0);
  data8(0xA2);
  data8(0x02);
  data8(0x84);
  cmd(0xC1);
  data8(0xC5);
  cmd(0xC2);
  data8(0x0A);
  data8(0x00);
  cmd(0xC3);
  data8(0x8A);
  data8(0x2A);
  cmd(0xC4);
  data8(0x8A);
  data8(0xEE);
  cmd(0xC5);
  data8(0x0E);

  cmd(0x3A);
  data8(0x05); /* RGB565 */

  /* 横屏 160x128：MV 横屏 + BGR（bit3=1）。若偏红改回 0xA8=RGB */
  cmd(0x36);
  data8(0xB0);

  cmd(0x29); /* DISPON */
  HAL_Delay(50);
  LCD_BL_ON();
}

void st7735_draw_rgb565(const uint8_t *pixels, uint32_t len) {
  (void)st7735_draw_rgb565_timed(pixels, len);
}

uint32_t st7735_draw_rgb565_timed(const uint8_t *pixels, uint32_t len) {
  uint32_t t0;
  uint32_t us;

  if (pixels == NULL || len != (uint32_t)LCD_W * LCD_H * 2u) {
    return 0;
  }
  st7735_bus_acquire();
  t0 = dwt_us();
  set_window(0, 0, (uint16_t)(LCD_W - 1), (uint16_t)(LCD_H - 1));
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(pixels, len);
  LCD_CS_HIGH();
  us = dwt_us() - t0;
  return us;
}

void st7735_draw_row(uint16_t y, const uint8_t *row) {
  if (row == NULL || y >= LCD_H) {
    return;
  }
  st7735_bus_acquire();
  set_window(0, y, (uint16_t)(LCD_W - 1), y);
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(row, (uint32_t)LCD_W * 2u);
  LCD_CS_HIGH();
}
