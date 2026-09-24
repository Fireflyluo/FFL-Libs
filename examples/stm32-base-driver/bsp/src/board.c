/**
 * @file board.c
 * @brief 板级：时钟、GPIO、USART1（TX 走 DMA1_CH4 + TXE 兜底）。
 *
 * DMA 映射（F103C8）：
 *   SPI1_TX  = DMA1_CH3（LCD，见 st7735.c）
 *   USART1_TX= DMA1_CH4
 *   USART1_RX= 中断 + 环（命令解析；不用 DMA RX 以免与逐字节协议冲突）
 */
#include "board.h"

#include "board_i2c.h"

#include <string.h>

static UART_HandleTypeDef s_huart1;
static DMA_HandleTypeDef s_hdma_uart_tx;

#define UART_TXQ_SZ 384u
static uint8_t s_txq[UART_TXQ_SZ];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint8_t s_txe_armed;
static volatile uint8_t s_uart_dma_busy;
static volatile uint16_t s_uart_dma_len;

#define UART_RXQ_SZ 768u
static uint8_t s_rxq[UART_RXQ_SZ];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static void board_clock_init(void) {
  RCC_OscInitTypeDef rcc_osc = {0};
  RCC_ClkInitTypeDef rcc_clk = {0};

  rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rcc_osc.HSEState = RCC_HSE_ON;
  rcc_osc.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  rcc_osc.HSIState = RCC_HSI_ON;
  rcc_osc.PLL.PLLState = RCC_PLL_ON;
  rcc_osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rcc_osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK) {
    Error_Handler();
  }
  rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk.APB2CLKDivider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void board_led_init(void) {
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
}

static void board_uart_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);

  s_huart1.Instance = USART1;
  s_huart1.Init.BaudRate = 115200u;
  s_huart1.Init.WordLength = UART_WORDLENGTH_8B;
  s_huart1.Init.StopBits = UART_STOPBITS_1;
  s_huart1.Init.Parity = UART_PARITY_NONE;
  s_huart1.Init.Mode = UART_MODE_TX_RX;
  s_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  s_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&s_huart1) != HAL_OK) {
    Error_Handler();
  }

  s_hdma_uart_tx.Instance = DMA1_Channel4;
  s_hdma_uart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  s_hdma_uart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  s_hdma_uart_tx.Init.MemInc = DMA_MINC_ENABLE;
  s_hdma_uart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  s_hdma_uart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  s_hdma_uart_tx.Init.Mode = DMA_NORMAL;
  s_hdma_uart_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&s_hdma_uart_tx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(&s_huart1, hdmatx, s_hdma_uart_tx);

  __HAL_UART_ENABLE_IT(&s_huart1, UART_IT_RXNE);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
  uint32_t sr = s_huart1.Instance->SR;
  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE |
             USART_SR_PE)) != 0u) {
    uint8_t b = (uint8_t)(s_huart1.Instance->DR & 0xFFu);
    if ((sr & USART_SR_RXNE) != 0u) {
      uint16_t next = (uint16_t)((s_rx_head + 1u) % UART_RXQ_SZ);
      if (next != s_rx_tail) {
        s_rxq[s_rx_head] = b;
        s_rx_head = next;
      }
    }
  }
  /* TXE 兜底（DMA 失败时） */
  if ((__HAL_UART_GET_IT_SOURCE(&s_huart1, UART_IT_TXE) != RESET) &&
      (__HAL_UART_GET_FLAG(&s_huart1, UART_FLAG_TXE) != RESET)) {
    if (s_tx_tail != s_tx_head) {
      s_huart1.Instance->DR = (uint16_t)s_txq[s_tx_tail];
      s_tx_tail = (uint16_t)((s_tx_tail + 1u) % UART_TXQ_SZ);
    } else {
      __HAL_UART_DISABLE_IT(&s_huart1, UART_IT_TXE);
      s_txe_armed = 0u;
    }
  }
}

static int txq_push(const uint8_t *data, int len, int expand_nl) {
  int n = 0;
  while (n < len) {
    uint8_t b = data[n];
    uint16_t next;
    if (expand_nl && b == '\n') {
      /* printf 路径：\n → \r\n */
      next = (uint16_t)((s_tx_head + 1u) % UART_TXQ_SZ);
      if (next == s_tx_tail) {
        break;
      }
      s_txq[s_tx_head] = '\r';
      s_tx_head = next;
    }
    next = (uint16_t)((s_tx_head + 1u) % UART_TXQ_SZ);
    if (next == s_tx_tail) {
      break;
    }
    s_txq[s_tx_head] = b;
    s_tx_head = next;
    n++;
  }
  return n;
}

void board_uart_putc(int ch) {
  uint8_t b = (uint8_t)ch;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  (void)txq_push(&b, 1, 1);
  board_uart_kick_tx();
  if (!primask) {
    __enable_irq();
  }
}

void board_uart_write(const char *s) {
  size_t n;
  if (s == NULL) {
    return;
  }
  n = strlen(s);
  (void)board_uart_write_try(s, (int)n);
}

int board_uart_write_try(const char *data, int len) {
  int n;
  uint32_t primask;
  if (data == NULL || len <= 0) {
    return 0;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  /* ulog 已带 \r\n，不再展开 */
  n = txq_push((const uint8_t *)data, len, 0);
  board_uart_kick_tx();
  if (!primask) {
    __enable_irq();
  }
  return n;
}

int board_uart_getc(void) {
  int ch;
  if (s_rx_head == s_rx_tail) {
    return -1;
  }
  ch = (int)s_rxq[s_rx_tail];
  s_rx_tail = (uint16_t)((s_rx_tail + 1u) % UART_RXQ_SZ);
  return ch;
}

UART_HandleTypeDef *board_uart_handle(void) { return &s_huart1; }

int __io_putchar(int ch) {
  board_uart_putc(ch);
  return ch;
}

static void board_buzzer_pwm_init(void);
void board_buzzer_tone(uint16_t hz);
void board_buzzer_off(void);

void board_init(void) {
  HAL_Init();
  board_clock_init();
  board_led_init();
  board_buzzer_pwm_init();
  board_buzzer_off();
  board_uart_init();
  board_i2c1_init_pb89();
  MODIFY_REG(RCC->CFGR, RCC_CFGR_USBPRE, 0U);
  __HAL_RCC_USB_CLK_ENABLE();
  board_led_off();
}

void board_led_on(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); }
void board_led_off(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}
void board_led_toggle(void) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0); }

/* ---- 蜂鸣器 TIM4_CH1 = PB6 ---- */
static TIM_HandleTypeDef s_htim4;
static uint8_t s_buzzer_pwm_on;

static void board_buzzer_pwm_init(void) {
  TIM_OC_InitTypeDef oc = {0};
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  s_htim4.Instance = TIM4;
  s_htim4.Init.Prescaler = 71; /* 72MHz/72 = 1MHz */
  s_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim4.Init.Period = 999;
  s_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&s_htim4) != HAL_OK) {
    return;
  }
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 500;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  (void)HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, TIM_CHANNEL_1);
}

void board_buzzer_tone(uint16_t hz) {
  GPIO_InitTypeDef gpio = {0};
  uint32_t arr;

  if (hz < 80u) {
    board_buzzer_off();
    return;
  }
  if (hz > 8000u) {
    hz = 8000u;
  }
  arr = (1000000u / (uint32_t)hz) - 1u;
  if (arr < 20u) {
    arr = 20u;
  }
  if (!s_buzzer_pwm_on) {
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    (void)HAL_TIM_PWM_Start(&s_htim4, TIM_CHANNEL_1);
    s_buzzer_pwm_on = 1u;
  }
  __HAL_TIM_SET_AUTORELOAD(&s_htim4, arr);
  __HAL_TIM_SET_COMPARE(&s_htim4, TIM_CHANNEL_1, arr / 2u);
}

void board_buzzer_on(void) { board_buzzer_tone(1000u); }

void board_buzzer_off(void) {
  GPIO_InitTypeDef gpio = {0};
  if (s_buzzer_pwm_on) {
    (void)HAL_TIM_PWM_Stop(&s_htim4, TIM_CHANNEL_1);
    s_buzzer_pwm_on = 0u;
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void board_uart_kick_tx(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_tx_head != s_tx_tail && s_txe_armed == 0u) {
    s_txe_armed = 1u;
    __HAL_UART_ENABLE_IT(&s_huart1, UART_IT_TXE);
  }
  if (!primask) {
    __enable_irq();
  }
}

void board_idle(void) { __WFI(); }

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
