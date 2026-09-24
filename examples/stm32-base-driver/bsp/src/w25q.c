/**
 * @file w25q.c
 * @brief W25Q 软件 SPI：GPIO 位操作，时序 Mode0。
 *
 * 与 ST7735 硬件 SPI1 并存（不碰 AFIO SPI1 remap）。
 * 本文件优化：位循环完全展开 + BSRR 直写，目标把软读提到 ~1.5–2 MHz 等效。
 */
#include "w25q.h"

#include <errno.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#define CS_LOW()  (GPIOA->BRR = GPIO_PIN_15)
#define CS_HIGH() (GPIOA->BSRR = GPIO_PIN_15)
#define SCK_H()   (GPIOB->BSRR = GPIO_PIN_3)
#define SCK_L()   (GPIOB->BRR = GPIO_PIN_3)
#define MOSI_H()  (GPIOB->BSRR = GPIO_PIN_5)
#define MOSI_L()  (GPIOB->BRR = GPIO_PIN_5)
#define MISO_BIT() ((GPIOB->IDR & GPIO_PIN_4) != 0u)

/* 72MHz 下 1–2 个 nop 足够满足 W25Q tCL/tCH。 */
#define SPI_NOP() __asm volatile("nop")

#define W25Q_SR1_WIP 0x01u
#define W25Q_SR1_WEL 0x02u

static w25q_write_diag_t s_last_write_diag;

/**
 * 发 1 字节 / 收 1 字节，Mode0，完全展开。
 * MOSI 先变，SCK 上升沿采 MISO，SCK 下降沿后准备下一位。
 */
static uint8_t spi_xfer8(uint8_t out) {
  uint8_t in = 0;

#define XFER_BIT(mask)                                                        \
  do {                                                                        \
    if ((out & (mask)) != 0u) {                                               \
      MOSI_H();                                                               \
    } else {                                                                  \
      MOSI_L();                                                               \
    }                                                                         \
    SPI_NOP();                                                                \
    SCK_H();                                                                  \
    SPI_NOP();                                                                \
    if (MISO_BIT()) {                                                         \
      in |= (mask);                                                           \
    }                                                                         \
    SCK_L();                                                                  \
  } while (0)

  XFER_BIT(0x80);
  XFER_BIT(0x40);
  XFER_BIT(0x20);
  XFER_BIT(0x10);
  XFER_BIT(0x08);
  XFER_BIT(0x04);
  XFER_BIT(0x02);
  XFER_BIT(0x01);
#undef XFER_BIT

  return in;
}

/** 分段关中断（每 16 字节开一次），避免长时间 ORE 丢串口字节 */
static void spi_read(uint8_t *data, uint32_t len) {
  uint32_t i = 0;
  while (i < len) {
    uint32_t n = 0;
    __disable_irq();
    while (i < len && n < 16u) {
      data[i++] = spi_xfer8(0xFF);
      n++;
    }
    __enable_irq();
  }
}

static void spi_write(const uint8_t *data, uint32_t len) {
  uint32_t i = 0;
  while (i < len) {
    uint32_t n = 0;
    __disable_irq();
    while (i < len && n < 16u) {
      (void)spi_xfer8(data[i++]);
      n++;
    }
    __enable_irq();
  }
}

static int wait_ready(uint32_t timeout_ms, uint8_t *saw_busy) {
  uint32_t t0 = HAL_GetTick();
  uint8_t sr;

  for (;;) {
    sr = w25q_read_status();
    if ((sr & W25Q_SR1_WIP) == 0u) {
      return 0;
    }
    if (saw_busy != NULL) {
      *saw_busy = 1u;
    }
    if ((HAL_GetTick() - t0) > timeout_ms) {
      return -ETIMEDOUT;
    }
  }
}

static int write_enable(uint8_t *status_after_wren) {
  uint8_t sr;

  CS_LOW();
  __disable_irq();
  (void)spi_xfer8(W25Q_CMD_WREN);
  __enable_irq();
  CS_HIGH();

  sr = w25q_read_status();
  if (status_after_wren != NULL) {
    *status_after_wren = sr;
  }
  return ((sr & W25Q_SR1_WEL) != 0u) ? 0 : -EACCES;
}

int w25q_init(void) {
  GPIO_InitTypeDef gpio = {0};
  uint8_t id[3];

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* 释放 JTAG 脚给软 SPI，保留 SWD */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  CS_HIGH();

  gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  SCK_L();
  MOSI_L();

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_Delay(5);
  /* Enable Reset + Reset，回到已知状态（部分 clone 必须） */
  CS_LOW();
  (void)spi_xfer8(0x66);
  CS_HIGH();
  CS_LOW();
  (void)spi_xfer8(0x99);
  CS_HIGH();
  HAL_Delay(1);
  (void)w25q_read_jedec(id);
  return 0;
}

int w25q_read_jedec(uint8_t id[3]) {
  uint8_t raw[4];
  CS_LOW();
  __disable_irq();
  raw[0] = spi_xfer8(W25Q_CMD_JEDEC);
  raw[1] = spi_xfer8(0xFF);
  raw[2] = spi_xfer8(0xFF);
  raw[3] = spi_xfer8(0xFF);
  __enable_irq();
  CS_HIGH();
  id[0] = raw[1];
  id[1] = raw[2];
  id[2] = raw[3];
  return 0;
}

int w25q_erase_sector(uint32_t addr) {
  uint8_t cmd[4];

  if (addr > 0xFFFFFFu) {
    return -EINVAL;
  }
  if (write_enable(NULL) != 0) {
    return -EACCES;
  }

  CS_LOW();
  cmd[0] = W25Q_CMD_SE;
  cmd[1] = (uint8_t)(addr >> 16);
  cmd[2] = (uint8_t)(addr >> 8);
  cmd[3] = (uint8_t)addr;
  spi_write(cmd, 4);
  CS_HIGH();
  return wait_ready(500u, NULL);
}

int w25q_write_page(uint32_t addr, const uint8_t *data, uint16_t len) {
  static uint8_t frame[4 + 256];
  int rc;
  uint16_t i;

  memset(&s_last_write_diag, 0, sizeof(s_last_write_diag));
  if (data == NULL || len == 0u || len > 256u || addr > 0xFFFFFFu ||
      (((addr & 0xFFu) + len) > 256u)) {
    return -EINVAL;
  }

  rc = write_enable(&s_last_write_diag.status_after_wren);
  if (rc != 0) {
    return rc;
  }

  frame[0] = W25Q_CMD_PP;
  frame[1] = (uint8_t)(addr >> 16);
  frame[2] = (uint8_t)(addr >> 8);
  frame[3] = (uint8_t)addr;
  for (i = 0; i < len; i++) {
    frame[4u + i] = data[i];
  }
  CS_LOW();
  __disable_irq();
  for (i = 0; i < (uint16_t)(4u + len); i++) {
    (void)spi_xfer8(frame[i]);
  }
  __enable_irq();
  CS_HIGH();
  s_last_write_diag.status_after_command = w25q_read_status();
  rc = wait_ready(50u, &s_last_write_diag.saw_busy);
  s_last_write_diag.status_after_wait = w25q_read_status();
  return rc;
}

int w25q_write_page_verified(uint32_t addr, const uint8_t *data, uint16_t len) {
  static uint8_t verify[256];
  int rc;

  rc = w25q_write_page(addr, data, len);
  if (rc != 0) {
    return rc;
  }
  rc = w25q_read(addr, verify, len);
  if (rc != 0) {
    return rc;
  }
  return (memcmp(data, verify, len) == 0) ? 0 : -EIO;
}

void w25q_get_last_write_diag(w25q_write_diag_t *out) {
  if (out != NULL) {
    *out = s_last_write_diag;
  }
}

int w25q_read(uint32_t addr, uint8_t *data, uint16_t len) {
  uint8_t cmd[4];

  if ((data == NULL && len != 0u) || addr > 0xFFFFFFu ||
      len > (0x1000000u - addr)) {
    return -EINVAL;
  }
  if (len == 0u) {
    return 0;
  }
  /* 默认：展开软 SPI（实测整帧 ~400ms 量级，稳定） */
  CS_LOW();
  cmd[0] = W25Q_CMD_READ;
  cmd[1] = (uint8_t)(addr >> 16);
  cmd[2] = (uint8_t)(addr >> 8);
  cmd[3] = (uint8_t)addr;
  (void)spi_xfer8(cmd[0]);
  (void)spi_xfer8(cmd[1]);
  (void)spi_xfer8(cmd[2]);
  (void)spi_xfer8(cmd[3]);
  spi_read(data, len);
  CS_HIGH();
  return 0;
}

int w25q_write_image(uint32_t addr, const uint8_t *data, uint32_t len) {
  uint32_t off = 0;
  uint32_t sector = 0xFFFFFFFFu;

  if (data == NULL || addr > 0xFFFFFFu || len > (0x1000000u - addr)) {
    return -EINVAL;
  }
  while (off < len) {
    uint32_t abs = addr + off;
    uint32_t sec = abs & ~0xFFFu;
    uint16_t chunk;
    int rc;

    if (sec != sector) {
      rc = w25q_erase_sector(sec);
      if (rc != 0) {
        return rc;
      }
      sector = sec;
    }
    chunk = (uint16_t)(256u - (abs & 0xFFu));
    if ((len - off) < chunk) {
      chunk = (uint16_t)(len - off);
    }
    rc = w25q_write_page_verified(abs, data + off, chunk);
    if (rc != 0) {
      return rc;
    }
    off += chunk;
  }
  return 0;
}

uint32_t w25q_read_timed(uint32_t addr, uint8_t *data, uint16_t len) {
  uint32_t t0;
  uint32_t us;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  t0 = (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock);
  (void)w25q_read(addr, data, len);
  us = (uint32_t)(((uint64_t)DWT->CYCCNT * 1000000u) / SystemCoreClock) - t0;
  return us;
}

uint8_t w25q_read_status(void) {
  uint8_t sr;
  CS_LOW();
  __disable_irq();
  (void)spi_xfer8(W25Q_CMD_RDSR1);
  sr = spi_xfer8(0xFF);
  __enable_irq();
  CS_HIGH();
  return sr;
}

int w25q_write_status(uint8_t sr1) {
  int rc;

  rc = write_enable(NULL);
  if (rc != 0) {
    return rc;
  }
  CS_LOW();
  __disable_irq();
  (void)spi_xfer8(0x01); /* WRSR */
  (void)spi_xfer8(sr1);
  __enable_irq();
  CS_HIGH();
  return wait_ready(20u, NULL);
}
