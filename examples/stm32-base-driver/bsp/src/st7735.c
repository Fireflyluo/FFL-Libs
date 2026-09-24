/**
 * @file st7735.c
 * @brief ST7789 320x240：硬 SPI1 + **DMA1_CH3 TX**。
 *
 * DMA：SPI1_TX=CH3，USART1_TX=CH4（board.c），分通道不冲突。
 * Flash 仍为软 SPI（PB3/4/5），SPI1 不 remap。
 * 颜色：panel_px() = ~rgb565 抵消面板 INVON。
 */
#include "st7735.h"

#include "font6x8.h"

#include "stm32f1xx_hal.h"

#define LCD_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define LCD_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define LCD_DC_CMD()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define LCD_DC_DATA() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define LCD_RES_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define LCD_RES_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define LCD_BL_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define LCD_BL_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

#define LCD_COL_OFF 0
#define LCD_ROW_OFF 0
#define LCD_MADCTL 0x60

static SPI_HandleTypeDef s_hspi1;
static DMA_HandleTypeDef s_hdma_spi1_tx;
static volatile uint8_t s_spi_dma_busy;
static uint8_t s_line[LCD_W * 2];

static uint16_t panel_px(uint16_t rgb565) { return (uint16_t)(~rgb565); }

static void dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t dwt_us(void) {
  return (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock);
}

void DMA1_Channel3_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_spi1_tx); }

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    s_spi_dma_busy = 0;
  }
}

/** >=32B 走 DMA1_CH3，否则轮询；DMA 超时回退轮询 */
static void spi_tx(const uint8_t *data, uint32_t len) {
  if (len < 32u) {
    (void)HAL_SPI_Transmit(&s_hspi1, (uint8_t *)data, (uint16_t)len,
                           HAL_MAX_DELAY);
    return;
  }
  while (len > 0u) {
    uint16_t chunk = (len > 60000u) ? 60000u : (uint16_t)len;
    uint32_t t0;
    s_spi_dma_busy = 1;
    if (HAL_SPI_Transmit_DMA(&s_hspi1, (uint8_t *)data, chunk) != HAL_OK) {
      s_spi_dma_busy = 0;
      (void)HAL_SPI_Transmit(&s_hspi1, (uint8_t *)data, chunk, HAL_MAX_DELAY);
    } else {
      t0 = HAL_GetTick();
      while (s_spi_dma_busy) {
        if ((HAL_GetTick() - t0) > 200u) {
          s_spi_dma_busy = 0;
          break;
        }
      }
    }
    data += chunk;
    len -= chunk;
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

static void spi_dma_init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  s_hdma_spi1_tx.Instance = DMA1_Channel3;
  s_hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  s_hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&s_hdma_spi1_tx) != HAL_OK) {
    while (1) {
    }
  }
  __HAL_LINKDMA(&s_hspi1, hdmatx, s_hdma_spi1_tx);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

static void bus_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SPI1_DISABLE();

  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void st7735_bus_acquire(void) {
  static int spi_ready;
  if (!spi_ready) {
    bus_gpio_init();
    s_hspi1.Instance = SPI1;
    s_hspi1.Init.Mode = SPI_MODE_MASTER;
    s_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    s_hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    s_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_hspi1.Init.NSS = SPI_NSS_SOFT;
    s_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; /* 36MHz */
    s_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    s_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&s_hspi1) != HAL_OK) {
      while (1) {
      }
    }
    spi_dma_init();
    dwt_init();
    spi_ready = 1;
  }
  LCD_BL_ON();
}

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
  LCD_BL_OFF();
  LCD_RES_LOW();
  HAL_Delay(20);
  LCD_RES_HIGH();
  HAL_Delay(120);

  cmd(0x01); /* SWRESET */
  HAL_Delay(150);
  cmd(0x11); /* SLPOUT */
  HAL_Delay(200);

  cmd(0x3A);
  data8(0x55); /* RGB565 */

  cmd(0x36);
  data8(LCD_MADCTL); /* 横屏 MV|MX */

  cmd(0x2A);
  data8(0x00);
  data8(0x00);
  data8(0x01);
  data8(0x3F); /* x: 0..319 */
  cmd(0x2B);
  data8(0x00);
  data8(0x00);
  data8(0x00);
  data8(0xEF); /* y: 0..239 */

  cmd(0x13); /* NORON */
  cmd(0x29); /* DISPON */
  HAL_Delay(50);
  /* 不发 INVOFF（0x20 对本模组无效）；颜色靠 panel_px() 取反 */
  LCD_BL_ON();
  HAL_Delay(10);
}

void st7735_fill_color(uint16_t rgb565) {
  uint16_t c = panel_px(rgb565);
  uint16_t x;
  uint16_t y;
  for (x = 0; x < LCD_W; x++) {
    s_line[x * 2u] = (uint8_t)(c >> 8);
    s_line[x * 2u + 1u] = (uint8_t)c;
  }
  st7735_bus_acquire();
  set_window(0, 0, (uint16_t)(LCD_W - 1), (uint16_t)(LCD_H - 1));
  LCD_DC_DATA();
  LCD_CS_LOW();
  for (y = 0; y < LCD_H; y++) {
    spi_tx(s_line, LCD_W * 2u);
  }
  LCD_CS_HIGH();
}

uint8_t *st7735_line_buf(void) { return s_line; }

#define LOG_BAR_Y (LCD_H - 10u)

void st7735_log_line(const char *text) {
  uint16_t y;
  uint16_t x;
  uint16_t n;
  uint16_t i;
  uint16_t bg = panel_px(0x2104u);
  uint16_t fg = panel_px(0xFFFFu);

  if (text == NULL) {
    text = "";
  }
  n = 0;
  while (text[n] != '\0' && n < (LCD_W / 6u)) {
    n++;
  }
  st7735_bus_acquire();
  set_window(0, LOG_BAR_Y, (uint16_t)(LCD_W - 1), (uint16_t)(LCD_H - 1));
  for (y = 0; y < 10u; y++) {
    /* 整行底色 */
    for (x = 0; x < LCD_W; x++) {
      s_line[x * 2u] = (uint8_t)(bg >> 8);
      s_line[x * 2u + 1u] = (uint8_t)bg;
    }
    /* 字：从 LOG_BAR_Y+1 起 8px */
    if (y >= 1u && y <= 8u) {
      uint8_t rbits = (uint8_t)(y - 1u);
      for (i = 0; i < n; i++) {
        const uint8_t *g = font6x8_glyph(text[i]);
        uint8_t bits = g[rbits];
        uint8_t col;
        for (col = 0; col < 5u; col++) {
          /* 位7=最左 */
          if ((bits & (uint8_t)(0x80u >> col)) != 0u) {
            uint16_t px = (uint16_t)(i * 6u + col);
            if (px < LCD_W) {
              s_line[px * 2u] = (uint8_t)(fg >> 8);
              s_line[px * 2u + 1u] = (uint8_t)fg;
            }
          }
        }
      }
    }
    LCD_DC_DATA();
    LCD_CS_LOW();
    spi_tx(s_line, LCD_W * 2u);
    LCD_CS_HIGH();
  }
}



uint32_t st7735_gpio_snapshot(void) {
  uint32_t v = 0;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
    v |= 1u;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
    v |= 2u;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
    v |= 4u;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
    v |= 8u;
  }
  return v;
}

void st7735_draw_rgb565(const uint8_t *pixels, uint32_t len) {
  (void)st7735_draw_rgb565_timed(pixels, len);
}

uint32_t st7735_draw_rgb565_timed(const uint8_t *pixels, uint32_t len) {
  uint32_t t0;
  uint32_t us;
  uint32_t i;
  if (pixels == NULL || len != (uint32_t)LCD_W * LCD_H * 2u) {
    return 0;
  }
  st7735_bus_acquire();
  t0 = dwt_us();
  set_window(0, 0, (uint16_t)(LCD_W - 1), (uint16_t)(LCD_H - 1));
  LCD_DC_DATA();
  LCD_CS_LOW();
  for (i = 0; i < len; i += 2u) {
    uint16_t c = (uint16_t)((pixels[i] << 8) | pixels[i + 1u]);
    uint8_t out[2];
    c = panel_px(c);
    out[0] = (uint8_t)(c >> 8);
    out[1] = (uint8_t)c;
    spi_tx(out, 2);
  }
  LCD_CS_HIGH();
  us = dwt_us() - t0;
  return us;
}

void st7735_draw_row(uint16_t y, const uint8_t *row) {
  uint16_t x;
  if (row == NULL || y >= LCD_H) {
    return;
  }
  for (x = 0; x < LCD_W; x++) {
    uint16_t c = (uint16_t)((row[x * 2u] << 8) | row[x * 2u + 1u]);
    c = panel_px(c);
    s_line[x * 2u] = (uint8_t)(c >> 8);
    s_line[x * 2u + 1u] = (uint8_t)c;
  }
  st7735_bus_acquire();
  set_window(0, y, (uint16_t)(LCD_W - 1), y);
  LCD_DC_DATA();
  LCD_CS_LOW();
  spi_tx(s_line, (uint32_t)LCD_W * 2u);
  LCD_CS_HIGH();
}

/**
 * 横屏自检：整行同色，y=0 起 红-绿-蓝-白-黄-青-品红。
 * 即：程序“上”= 屏幕上方；若整组上下颠倒，改 MADCTL 或反序。
 */
uint32_t st7735_selftest_bars(void) {
  static const uint16_t col[7] = {0xF800, 0x07E0, 0x001F,
                                  0xFFFF, 0xFFE0, 0x07FF, 0xF81F};
  uint16_t y, x, band;
  uint32_t t0;
  uint16_t band_h;

  band_h = (uint16_t)(LCD_H / 7u);
  if (band_h == 0u) {
    band_h = 1u;
  }

  st7735_bus_acquire();
  t0 = dwt_us();
  for (y = 0; y < LCD_H; y++) {
    band = (uint16_t)(y / band_h);
    if (band > 6u) {
      band = 6u;
    }
    {
      uint16_t c = panel_px(col[band]);
      uint8_t hi = (uint8_t)(c >> 8);
      uint8_t lo = (uint8_t)c;
      for (x = 0; x < LCD_W; x++) {
        s_line[x * 2u] = hi;
        s_line[x * 2u + 1u] = lo;
      }
    }
    set_window(0, y, (uint16_t)(LCD_W - 1), y);
    LCD_DC_DATA();
    LCD_CS_LOW();
    spi_tx(s_line, LCD_W * 2u);
    LCD_CS_HIGH();
  }
  return dwt_us() - t0;
}
