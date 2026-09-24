/**
 * @file main.c
 * @brief STM32-LORA + CherryUSB CDC ACM 最小验证固件。
 *
 * - 复用 examples/stm32-base-driver 的 SDK / 时钟（16MHz HSE → 72MHz）
 * - USB 时钟 48MHz，PA11/PA12 片上 FS device
 * - 枚举为 CDC 虚拟串口；DTR 打开后每 1s 发送心跳，收到数据原样回显
 * - USART1 115200 保留板级日志（COM4 / PowerWriter）
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"

#define CDC_IN_EP 0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0xFF55
#define USBD_PID 0x5710
#define USBD_MAX_POWER 100
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)
#define CDC_MAX_MPS 64

static UART_HandleTypeDef s_huart1;
static volatile uint8_t s_dtr;
static volatile uint8_t s_ep_tx_busy;
static volatile uint8_t s_speed_run; /* 收到 'S' 后开始满速 IN */
static volatile uint32_t s_usb_configured;
static volatile uint32_t s_usb_out_count;
static volatile uint32_t s_usb_in_count;
static volatile uint32_t s_usb_in_bytes;
static volatile uint32_t s_usb_out_bytes;
static volatile uint32_t s_usb_tx_submit_fail;

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0100, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS,
                            0x02)
};

static const uint8_t device_quality_descriptor[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "FFL",
    "STM32-LORA CherryUSB CDC",
    "0001",
};

static const uint8_t *device_descriptor_callback(uint8_t speed) {
  (void)speed;
  return device_descriptor;
}
static const uint8_t *config_descriptor_callback(uint8_t speed) {
  (void)speed;
  return config_descriptor;
}
static const uint8_t *device_quality_descriptor_callback(uint8_t speed) {
  (void)speed;
  return device_quality_descriptor;
}
static const char *string_descriptor_callback(uint8_t speed, uint8_t index) {
  (void)speed;
  if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
    return NULL;
  }
  return string_descriptors[index];
}

static const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

static USB_MEM_ALIGNX uint8_t read_buffer[CDC_MAX_MPS];
static USB_MEM_ALIGNX uint8_t write_buffer[128];

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
    while (1) {
    }
  }
  rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_2) != HAL_OK) {
    while (1) {
    }
  }
}

static void board_uart_init(void) {
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
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
  (void)HAL_UART_Init(&s_huart1);
}

int __io_putchar(int ch) {
  uint8_t b = (uint8_t)ch;
  if (ch == '\n') {
    uint8_t cr = (uint8_t)'\r';
    (void)HAL_UART_Transmit(&s_huart1, &cr, 1u, 100u);
  }
  (void)HAL_UART_Transmit(&s_huart1, &b, 1u, 100u);
  return ch;
}

void SysTick_Handler(void) { HAL_IncTick(); }

void HardFault_Handler(void) {
  /* 留现场：主循环会打印 g_hf_count 证明是否进过 HardFault */
  extern volatile uint32_t g_hf_count;
  g_hf_count++;
  while (1) {
  }
}

volatile uint32_t g_hf_count;
volatile uint32_t g_usb_evt_reset;
volatile uint32_t g_usb_evt_cfg;

static void usbd_event_handler(uint8_t busid, uint8_t event) {
  switch (event) {
  case USBD_EVENT_RESET:
    g_usb_evt_reset++;
    s_usb_configured = 0u;
    break;
  case USBD_EVENT_CONFIGURED:
    g_usb_evt_cfg++;
    s_usb_configured = 1u;
    s_ep_tx_busy = 0u;
    usbd_ep_start_read(busid, CDC_OUT_EP, read_buffer, CDC_MAX_MPS);
    break;
  case USBD_EVENT_DISCONNECTED:
    s_usb_configured = 0u;
    break;
  default:
    break;
  }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes) {
  (void)ep;
  s_usb_out_count++;
  s_usb_out_bytes += nbytes;
  if (nbytes > 0u) {
    if (read_buffer[0] == 'S') {
      s_speed_run = 1u;
    } else if (read_buffer[0] == 'X') {
      s_speed_run = 0u;
    }
  }
  usbd_ep_start_read(busid, CDC_OUT_EP, read_buffer, CDC_MAX_MPS);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes) {
  (void)busid;
  (void)ep;
  s_usb_in_count++;
  s_usb_in_bytes += nbytes;
  s_ep_tx_busy = 0u;
}

struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out,
};
struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in,
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr) {
  (void)busid;
  (void)intf;
  s_dtr = dtr ? 1u : 0u;
}

static void cdc_acm_init(uint8_t busid, uintptr_t reg_base) {
  usbd_desc_register(busid, &cdc_descriptor);
  usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
  usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
  usbd_add_endpoint(busid, &cdc_out_ep);
  usbd_add_endpoint(busid, &cdc_in_ep);
  usbd_initialize(busid, reg_base, usbd_event_handler);
}

int main(void) {
  uint32_t last_sec = 0;
  uint32_t last_bytes = 0;
  uint32_t i;

  /* USB FS bulk 最大包 64B；满载发包测吞吐 */
  for (i = 0; i < CDC_MAX_MPS; i++) {
    write_buffer[i] = (uint8_t)(0xA0u + (i & 0x0Fu));
  }

  HAL_Init();
  board_clock_init();
  board_uart_init();

  printf("[boot] cherryusb cdc speed test\n");
  cdc_acm_init(0, USB_BASE);
  printf("[usb] open COM5, DTR on, send 'S' to start IN flood, 'X' to stop\n");

  for (;;) {
    uint32_t tick = HAL_GetTick();

    if ((tick / 1000u) != last_sec) {
      last_sec = tick / 1000u;
      uint32_t delta = s_usb_in_bytes - last_bytes;
      last_bytes = s_usb_in_bytes;
      printf("[spd] t=%lus run=%u in=%luB/s in_pkts=%lu out=%luB fail=%lu\n",
             (unsigned long)last_sec, (unsigned)s_speed_run,
             (unsigned long)delta, (unsigned long)s_usb_in_count,
             (unsigned long)s_usb_out_bytes, (unsigned long)s_usb_tx_submit_fail);
    }

    if (s_usb_configured && s_dtr && s_speed_run && !s_ep_tx_busy) {
      s_ep_tx_busy = 1u;
      if (usbd_ep_start_write(0, CDC_IN_EP, write_buffer, CDC_MAX_MPS) != 0) {
        s_ep_tx_busy = 0u;
        s_usb_tx_submit_fail++;
      }
    }
  }
}
