/**
 * @file board.c
 * @brief STM32F103C8T6 最小系统板板级初始化。
 *
 * 时钟拓扑与实板跑通工程一致：
 *   HSE 8MHz -> PLL x9 -> SYSCLK 72MHz -> HCLK 72MHz
 *   -> PCLK1 36MHz(APB1, I2C1 挂这里), PCLK2 72MHz(APB2)
 */
#include "board.h"

#include "sc7a20_i2c_transport.h"
#include "stm32_time_ops.h"

static void board_clock_init(void) {
  RCC_OscInitTypeDef rcc_osc = {0};
  RCC_ClkInitTypeDef rcc_clk = {0};

  rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rcc_osc.HSEState = RCC_HSE_ON;
  rcc_osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  rcc_clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void board_led_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* 熄灭（低电平点亮） */

  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

void board_init(void) {
  HAL_Init(); /* 配置 SysTick 1ms 时基，见 stm32f1xx_it.c 的 SysTick_Handler */
  board_clock_init();    /* 72MHz */
  board_led_init();      /* PC13 */
  bsp_time_init();       /* DWT 微秒计数 */
  bsp_sc7a20_i2c_init(); /* I2C1: PB6/PB7, 100kHz */
  board_led_off();
}

/* PC13 板载 LED 为低电平点亮（有源低）。若你的板子为高电平点亮，
 * 交换 board_led_on / board_led_off 中的电平即可。 */
void board_led_on(void) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void board_led_off(void) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void board_led_toggle(void) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); }

void board_idle(void) { __WFI(); }

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
