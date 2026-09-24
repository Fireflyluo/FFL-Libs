/**
 * @file w25q_hw.c
 * @brief W25Q 硬件 SPI1 重映射 + DMA/中断事务实现。
 */
#include "w25q_hw.h"

#include <errno.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#define CS_LOW()  (GPIOA->BRR = GPIO_PIN_15)
#define CS_HIGH() (GPIOA->BSRR = GPIO_PIN_15)

#define W25Q_HW_MAX_PAGE 256u
#define W25Q_SR1_WIP 0x01u
#define W25Q_SR1_WEL 0x02u

static SPI_HandleTypeDef s_hspi;
static DMA_HandleTypeDef s_hdma_tx;
static DMA_HandleTypeDef s_hdma_rx;
static uint8_t s_hw_inited;
static volatile uint8_t s_xfer_pending;
static volatile int s_xfer_rc;
static volatile uint32_t s_dma_completed;
static volatile uint32_t s_dma_errors;
static w25q_write_diag_t s_last_write_diag;

static void configure_cs_gpio(void) {
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  CS_HIGH();
}

void DMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_rx); }

void DMA1_Channel3_IRQHandler(void) { HAL_DMA_IRQHandler(&s_hdma_tx); }

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    s_dma_completed++;
    s_xfer_rc = 0;
    s_xfer_pending = 0u;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    s_dma_errors++;
    s_xfer_rc = -EIO;
    s_xfer_pending = 0u;
  }
}

static int spi_txrx_dma(const uint8_t *tx, uint8_t *rx, uint16_t len,
                        uint32_t timeout_ms) {
  uint32_t started;

  if (tx == NULL || rx == NULL || len == 0u) {
    return -EINVAL;
  }
  s_xfer_rc = -EINPROGRESS;
  s_xfer_pending = 1u;
  if (HAL_SPI_TransmitReceive_DMA(&s_hspi, (uint8_t *)tx, rx, len) != HAL_OK) {
    s_xfer_pending = 0u;
    s_xfer_rc = -EIO;
    return -EIO;
  }
  started = HAL_GetTick();
  while (s_xfer_pending != 0u) {
    if ((HAL_GetTick() - started) > timeout_ms) {
      (void)HAL_SPI_Abort(&s_hspi);
      s_xfer_pending = 0u;
      s_xfer_rc = -ETIMEDOUT;
      return -ETIMEDOUT;
    }
  }
  return s_xfer_rc;
}

static int spi_txrx_polling(const uint8_t *tx, uint8_t *rx, uint16_t len,
                            uint32_t timeout_ms) {
  if (tx == NULL || rx == NULL || len == 0u) {
    return -EINVAL;
  }
  return (HAL_SPI_TransmitReceive(&s_hspi, (uint8_t *)tx, rx, len,
                                  timeout_ms) == HAL_OK)
             ? 0
             : -EIO;
}

static int spi_xfer_register(uint8_t tx, uint8_t *rx, uint32_t timeout_ms) {
  uint32_t started = HAL_GetTick();

  if (rx == NULL) {
    return -EINVAL;
  }
  while ((SPI1->SR & SPI_SR_TXE) == 0u) {
    if ((HAL_GetTick() - started) > timeout_ms) {
      return -ETIMEDOUT;
    }
  }
  *(__IO uint8_t *)&SPI1->DR = tx;
  while ((SPI1->SR & SPI_SR_RXNE) == 0u) {
    if ((HAL_GetTick() - started) > timeout_ms) {
      return -ETIMEDOUT;
    }
  }
  *rx = *(__IO uint8_t *)&SPI1->DR;
  return 0;
}

static int ensure_init(void) {
  GPIO_InitTypeDef gpio = {0};

  if (s_hw_inited != 0u) {
    return 0;
  }
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_AFIO_REMAP_SPI1_ENABLE();

  gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  configure_cs_gpio();

  s_hdma_tx.Instance = DMA1_Channel3;
  s_hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_tx.Init.Mode = DMA_NORMAL;
  s_hdma_tx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&s_hdma_tx) != HAL_OK) {
    return -EIO;
  }

  s_hdma_rx.Instance = DMA1_Channel2;
  s_hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_rx.Init.Mode = DMA_NORMAL;
  s_hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&s_hdma_rx) != HAL_OK) {
    return -EIO;
  }

  s_hspi.Instance = SPI1;
  s_hspi.Init.Mode = SPI_MODE_MASTER;
  s_hspi.Init.Direction = SPI_DIRECTION_2LINES;
  s_hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  s_hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  s_hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  s_hspi.Init.NSS = SPI_NSS_SOFT;
  s_hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; /* 562.5 kHz */
  s_hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  s_hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  s_hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  s_hspi.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&s_hspi) != HAL_OK) {
    return -EIO;
  }
  __HAL_LINKDMA(&s_hspi, hdmatx, s_hdma_tx);
  __HAL_LINKDMA(&s_hspi, hdmarx, s_hdma_rx);

  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1u, 0u);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1u, 0u);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  s_hw_inited = 1u;
  return 0;
}

static int read_status_hw(uint8_t *status) {
  uint8_t tx[2] = {W25Q_CMD_RDSR1, 0xFFu};
  uint8_t rx[2];
  int rc;

  if (status == NULL) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }
  CS_LOW();
  rc = spi_txrx_dma(tx, rx, sizeof(tx), 20u);
  CS_HIGH();
  if (rc == 0) {
    *status = rx[1];
  }
  return rc;
}

int w25q_read_status_hw(uint8_t *status) { return read_status_hw(status); }

static int wait_ready_hw(uint32_t timeout_ms, uint8_t *saw_busy) {
  uint32_t started = HAL_GetTick();
  uint8_t status;
  int rc;

  for (;;) {
    rc = read_status_hw(&status);
    if (rc != 0) {
      return rc;
    }
    if ((status & W25Q_SR1_WIP) == 0u) {
      return 0;
    }
    if (saw_busy != NULL) {
      *saw_busy = 1u;
    }
    if ((HAL_GetTick() - started) > timeout_ms) {
      return -ETIMEDOUT;
    }
  }
}

static int write_enable_hw(uint8_t *status_after_wren) {
  uint8_t tx = W25Q_CMD_WREN;
  uint8_t rx;
  uint8_t status;
  int rc;

  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }
  CS_LOW();
  rc = spi_txrx_dma(&tx, &rx, 1u, 20u);
  CS_HIGH();
  if (rc != 0) {
    return rc;
  }
  rc = read_status_hw(&status);
  if (rc != 0) {
    return rc;
  }
  if (status_after_wren != NULL) {
    *status_after_wren = status;
  }
  return ((status & W25Q_SR1_WEL) != 0u) ? 0 : -EACCES;
}

int w25q_hw_init(void) { return ensure_init(); }

int w25q_jedec_hw(uint8_t id[3]) {
  uint8_t tx[4] = {W25Q_CMD_JEDEC, 0xFFu, 0xFFu, 0xFFu};
  uint8_t rx[4];
  int rc;

  if (id == NULL) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }
  CS_LOW();
  rc = spi_txrx_dma(tx, rx, sizeof(tx), 20u);
  CS_HIGH();
  if (rc != 0) {
    id[0] = id[1] = id[2] = 0u;
    return rc;
  }
  id[0] = rx[1];
  id[1] = rx[2];
  id[2] = rx[3];
  return 0;
}

int w25q_jedec_hw_polling(uint8_t id[3]) {
  uint8_t tx[4] = {W25Q_CMD_JEDEC, 0xFFu, 0xFFu, 0xFFu};
  uint8_t rx[4];
  int rc;

  if (id == NULL) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }
  CS_LOW();
  rc = spi_txrx_polling(tx, rx, sizeof(tx), 20u);
  CS_HIGH();
  if (rc != 0) {
    id[0] = id[1] = id[2] = 0u;
    return rc;
  }
  id[0] = rx[1];
  id[1] = rx[2];
  id[2] = rx[3];
  return 0;
}

int w25q_jedec_hw_register(uint8_t id[3]) {
  uint32_t started;
  uint8_t rx;
  int rc;

  if (id == NULL) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }
  CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
  started = HAL_GetTick();
  CS_LOW();
  rc = spi_xfer_register(W25Q_CMD_JEDEC, &rx, 20u);
  if (rc == 0) {
    rc = spi_xfer_register(0xFFu, &id[0], 20u);
  }
  if (rc == 0) {
    rc = spi_xfer_register(0xFFu, &id[1], 20u);
  }
  if (rc == 0) {
    rc = spi_xfer_register(0xFFu, &id[2], 20u);
  }
  while ((SPI1->SR & SPI_SR_BSY) != 0u) {
    if ((HAL_GetTick() - started) > 20u) {
      rc = -ETIMEDOUT;
      break;
    }
  }
  CS_HIGH();
  if (rc != 0) {
    id[0] = id[1] = id[2] = 0u;
  }
  return rc;
}

int w25q_jedec_hw_hardnss(uint8_t id[3]) {
  GPIO_InitTypeDef gpio = {0};
  uint8_t tx[4] = {W25Q_CMD_JEDEC, 0xFFu, 0xFFu, 0xFFu};
  uint8_t rx[4] = {0};
  int rc;

  if (id == NULL) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0) {
    return rc;
  }

  /*
   * This is a separate CubeMX-equivalent hardware-NSS experiment.  PA15 is
   * the remapped SPI1_NSS output; unlike the normal path, CS is not driven by
   * GPIO here.  One complete JEDEC command is kept inside one SPI enable
   * interval because STM32F1 hardware NSS stays active while SPE is set.
  */
  CLEAR_BIT(SPI1->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);

  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* HAL_SPI_Init writes CR1/CR2 from Init.NSS, including SSOE. */
  s_hspi.Init.NSS = SPI_NSS_HARD_OUTPUT;
  rc = (HAL_SPI_Init(&s_hspi) == HAL_OK) ? 0 : -EIO;
  if (rc == 0) {
    rc = (HAL_SPI_TransmitReceive(&s_hspi, tx, rx, sizeof(tx), 20u) ==
          HAL_OK)
             ? 0
             : -EIO;
  }

  /* Disable hardware NSS before returning PA15 to the normal GPIO CS path. */
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
  s_hspi.Init.NSS = SPI_NSS_SOFT;
  if (HAL_SPI_Init(&s_hspi) != HAL_OK) {
    rc = -EIO;
  }
  configure_cs_gpio();
  if (rc != 0) {
    id[0] = id[1] = id[2] = 0u;
    return rc;
  }
  id[0] = rx[1];
  id[1] = rx[2];
  id[2] = rx[3];
  return rc;
}

int w25q_read_hw(uint32_t addr, uint8_t *data, uint16_t len) {
  static uint8_t tx[4u + W25Q_HW_MAX_PAGE];
  static uint8_t rx[4u + W25Q_HW_MAX_PAGE];
  uint16_t done = 0u;
  int rc;

  if ((data == NULL && len != 0u) || addr > 0xFFFFFFu ||
      len > (0x1000000u - addr)) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0 || len == 0u) {
    return rc;
  }
  while (done < len) {
    uint16_t chunk = (uint16_t)(len - done);
    uint16_t i;
    uint32_t current = addr + done;

    if (chunk > W25Q_HW_MAX_PAGE) {
      chunk = W25Q_HW_MAX_PAGE;
    }
    tx[0] = W25Q_CMD_READ;
    tx[1] = (uint8_t)(current >> 16);
    tx[2] = (uint8_t)(current >> 8);
    tx[3] = (uint8_t)current;
    for (i = 0u; i < chunk; i++) {
      tx[4u + i] = 0xFFu;
    }
    CS_LOW();
    rc = spi_txrx_dma(tx, rx, (uint16_t)(4u + chunk), 50u);
    CS_HIGH();
    if (rc != 0) {
      return rc;
    }
    memcpy(data + done, rx + 4u, chunk);
    done = (uint16_t)(done + chunk);
  }
  return 0;
}

int w25q_read_hw_polling(uint32_t addr, uint8_t *data, uint16_t len) {
  static uint8_t tx[4u + W25Q_HW_MAX_PAGE];
  static uint8_t rx[4u + W25Q_HW_MAX_PAGE];
  uint16_t done = 0u;
  int rc;

  if ((data == NULL && len != 0u) || addr > 0xFFFFFFu ||
      len > (0x1000000u - addr)) {
    return -EINVAL;
  }
  rc = ensure_init();
  if (rc != 0 || len == 0u) {
    return rc;
  }
  while (done < len) {
    uint16_t chunk = (uint16_t)(len - done);
    uint16_t i;
    uint32_t current = addr + done;

    if (chunk > W25Q_HW_MAX_PAGE) {
      chunk = W25Q_HW_MAX_PAGE;
    }
    tx[0] = W25Q_CMD_READ;
    tx[1] = (uint8_t)(current >> 16);
    tx[2] = (uint8_t)(current >> 8);
    tx[3] = (uint8_t)current;
    for (i = 0u; i < chunk; i++) {
      tx[4u + i] = 0xFFu;
    }
    CS_LOW();
    rc = spi_txrx_polling(tx, rx, (uint16_t)(4u + chunk), 50u);
    CS_HIGH();
    if (rc != 0) {
      return rc;
    }
    memcpy(data + done, rx + 4u, chunk);
    done = (uint16_t)(done + chunk);
  }
  return 0;
}

int w25q_erase_sector_hw(uint32_t addr) {
  uint8_t tx[4];
  uint8_t rx[4];
  int rc;

  if (addr > 0xFFFFFFu) {
    return -EINVAL;
  }
  rc = write_enable_hw(NULL);
  if (rc != 0) {
    return rc;
  }
  tx[0] = W25Q_CMD_SE;
  tx[1] = (uint8_t)(addr >> 16);
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)addr;
  CS_LOW();
  rc = spi_txrx_dma(tx, rx, sizeof(tx), 20u);
  CS_HIGH();
  if (rc != 0) {
    return rc;
  }
  return wait_ready_hw(500u, NULL);
}

int w25q_write_page_hw(uint32_t addr, const uint8_t *data, uint16_t len) {
  static uint8_t tx[4u + W25Q_HW_MAX_PAGE];
  static uint8_t rx[4u + W25Q_HW_MAX_PAGE];
  int rc;
  uint16_t i;

  memset(&s_last_write_diag, 0, sizeof(s_last_write_diag));
  if (data == NULL || len == 0u || len > W25Q_HW_MAX_PAGE ||
      addr > 0xFFFFFFu || (((addr & 0xFFu) + len) > 256u)) {
    return -EINVAL;
  }
  rc = write_enable_hw(&s_last_write_diag.status_after_wren);
  if (rc != 0) {
    return rc;
  }
  tx[0] = W25Q_CMD_PP;
  tx[1] = (uint8_t)(addr >> 16);
  tx[2] = (uint8_t)(addr >> 8);
  tx[3] = (uint8_t)addr;
  for (i = 0u; i < len; i++) {
    tx[4u + i] = data[i];
  }
  CS_LOW();
  rc = spi_txrx_dma(tx, rx, (uint16_t)(4u + len), 50u);
  CS_HIGH();
  if (rc != 0) {
    return rc;
  }
  rc = read_status_hw(&s_last_write_diag.status_after_command);
  if (rc != 0) {
    return rc;
  }
  rc = wait_ready_hw(20u, &s_last_write_diag.saw_busy);
  if (read_status_hw(&s_last_write_diag.status_after_wait) != 0 && rc == 0) {
    return -EIO;
  }
  return rc;
}

int w25q_write_page_verified_hw(uint32_t addr, const uint8_t *data,
                                 uint16_t len) {
  static uint8_t verify[W25Q_HW_MAX_PAGE];
  int rc;

  rc = w25q_write_page_hw(addr, data, len);
  if (rc != 0) {
    return rc;
  }
  rc = w25q_read_hw(addr, verify, len);
  if (rc != 0) {
    return rc;
  }
  return (memcmp(data, verify, len) == 0) ? 0 : -EIO;
}

void w25q_hw_get_last_write_diag(w25q_write_diag_t *out) {
  if (out != NULL) {
    *out = s_last_write_diag;
  }
}

void w25q_hw_get_dma_stats(uint32_t *completed, uint32_t *errors) {
  if (completed != NULL) {
    *completed = s_dma_completed;
  }
  if (errors != NULL) {
    *errors = s_dma_errors;
  }
}

void w25q_hw_dump_afio(uint32_t *apb2enr, uint32_t *mapr) {
  (void)ensure_init();
  if (apb2enr != NULL) {
    *apb2enr = RCC->APB2ENR;
  }
  if (mapr != NULL) {
    *mapr = AFIO->MAPR;
  }
}

void w25q_hw_dump_spi(uint32_t *cr1, uint32_t *sr) {
  (void)ensure_init();
  if (cr1 != NULL) {
    *cr1 = SPI1->CR1;
  }
  if (sr != NULL) {
    *sr = SPI1->SR;
  }
}

void w25q_hw_dump_gpio(uint32_t *gpioa_crh, uint32_t *gpiob_crl) {
  (void)ensure_init();
  if (gpioa_crh != NULL) {
    *gpioa_crh = GPIOA->CRH;
  }
  if (gpiob_crl != NULL) {
    *gpiob_crl = GPIOB->CRL;
  }
}

uint32_t w25q_read_hw_timed(uint32_t addr, uint8_t *data, uint16_t len) {
  uint32_t started;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  started = DWT->CYCCNT;
  (void)w25q_read_hw(addr, data, len);
  return (uint32_t)(((uint64_t)(DWT->CYCCNT - started) * 1000000u) /
                    SystemCoreClock);
}
