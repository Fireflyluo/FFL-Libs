/**
 * @file flash_task.c
 * @brief W25Q GPIO 软件 SPI：初始化、JEDEC 与图像槽状态恢复。
 *
 * W25Q 与 ST7789 的 SPI1 + DMA 刷屏路径完全分离。图像仅通过 USB CDC
 * 单包写后 ACK 协议写入；本任务不在启动时擦写 Flash 或执行读加速实验。
 */
#include <stdio.h>

#include "osal_event.h"
#include "osal_tasks.h"
#include "usb_img.h"
#include "w25q.h"

#define EVT_POLL 0x0001u
/* 与 usb_img.c 一致：320×240 RGB565 = 153600B，槽位 0x28000。 */
#define IMG_STRIDE 0x028000u

static uint8_t s_probe_mask;

uint8_t flash_task_ready(void) {
  return (uint8_t)(s_probe_mask | usb_img_ready_mask());
}

uint32_t flash_task_img_addr(uint8_t idx) {
  if (idx > 2u) {
    idx = 0u;
  }
  return (uint32_t)idx * IMG_STRIDE;
}

/** 上电恢复：优先读 0x0F0000 魔数；再按图像首字节粗检。 */
static void probe_w25q(void) {
  uint8_t magic_mask;
  uint8_t first_bytes[3];
  uint8_t image_index;

  s_probe_mask = 0u;
  magic_mask = usb_img_load_ready_from_flash();
  {
    uint8_t mag[2] = {0, 0};
    uint8_t p0[4] = {0, 0, 0, 0};
    (void)w25q_read(0x0F0000u, mag, 2);
    (void)w25q_read(0u, p0, 4);
    printf("[flash] magic@0x0F0000 %02X %02X img0px=%02X%02X%02X%02X\n", mag[0],
           mag[1], p0[0], p0[1], p0[2], p0[3]);
  }
  if (magic_mask != 0u) {
    s_probe_mask = magic_mask;
    printf("[flash] ready magic mask=0x%02X\n", magic_mask);
  } else {
    for (image_index = 0u; image_index < 3u; image_index++) {
      (void)w25q_read(flash_task_img_addr(image_index),
                      &first_bytes[image_index], 1u);
      if (first_bytes[image_index] != 0x00u &&
          first_bytes[image_index] != 0xFFu) {
        s_probe_mask |= (uint8_t)(1u << image_index);
      }
    }
    printf("[flash] probe bytes mask=0x%02X\n", s_probe_mask);
  }
  usb_img_set_ready_mask(s_probe_mask);
}

static void flash_init(uint8_t task_id) {
  uint8_t id[3];

  (void)w25q_init();
  (void)w25q_read_jedec(id);
  printf("[flash] softSPI JEDEC %02X %02X %02X\n", id[0], id[1], id[2]);
  probe_w25q();
  (void)osal_start_reload_timer(task_id, EVT_POLL, 1u);
}

static uint16_t flash_event(uint8_t task_id, uint16_t events) {
  (void)task_id;
  if (events & EVT_POLL) {
    usb_img_poll();
    events &= (uint16_t)~EVT_POLL;
  }
  return events;
}

void flash_task_register(void) { osal_add_Task(flash_init, flash_event, 1u); }
