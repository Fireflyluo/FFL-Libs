/**
 * @file usb_img.c
 * @brief USB CDC 分包收图；每包写入 W25Q 后 ACK。
 *
 * OUT 回调只接收一个 64B 包，任务上下文负责 CRC、擦写和 ACK。
 * ACK 完成前不重臂 OUT 端点，主机必须按包等待，因而不需要大环形缓存。
 */
#include "usb_img.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "usbd_cdc_acm.h"
#include "usbd_core.h"
#include "w25q.h"

#define CDC_IN_EP 0x81
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83
#define USBD_VID 0xFF55
#define USBD_PID 0x5711
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)
#define CDC_MPS 64u
#define IMG_STRIDE 0x028000u
#define READY_ADDR 0x0F0000u

#define REQ_MAGIC0 'I'
#define REQ_MAGIC1 'P'
#define ACK_MAGIC0 'I'
#define ACK_MAGIC1 'A'
#define PROTOCOL_VERSION 1u
#define HEADER_LEN 12u
#define CRC_LEN 2u
#define DATA_MAX 48u

enum {
  PKT_BEGIN = 1u,
  PKT_DATA = 2u,
  PKT_END = 3u,
  PKT_ABORT = 4u,
  PKT_ACK = 0x80u,
  PKT_NACK = 0x81u
};

enum {
  STATUS_OK = 0u,
  STATUS_FORMAT = 1u,
  STATUS_CRC = 2u,
  STATUS_STATE = 3u,
  STATUS_SEQUENCE = 4u,
  STATUS_RANGE = 5u,
  STATUS_FLASH = 6u
};

static volatile uint8_t s_cfg;
static volatile uint8_t s_ep_busy;
static volatile uint8_t s_out_armed;
static volatile uint8_t s_rx_ready;
static volatile uint8_t s_rearm_pending;
static volatile uint8_t s_ready_mask;
static volatile uint8_t s_busy;
static volatile uint8_t s_rx_len;

static USB_MEM_ALIGNX uint8_t s_rx[CDC_MPS];
static USB_MEM_ALIGNX uint8_t s_tx[12];

static uint32_t s_total;
static uint32_t s_got;
static uint16_t s_expected_seq;
static uint16_t s_last_seq;
static uint8_t s_idx;
static uint8_t s_active;
static uint8_t s_last_valid;
static uint8_t s_last_status;
static uint8_t s_last_idx;
static uint32_t s_last_offset;

static const uint8_t dev_desc[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0100, 0x01)};
static const uint8_t cfg_desc[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, 100),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MPS,
                            0x02)};
static const uint8_t qual_desc[] = {0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
                                    0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00};
static const char *str_desc[] = {(const char[]){0x09, 0x04}, "FFL", "IMG-W25Q",
                                 "0001"};

static const uint8_t *cb_dev(uint8_t speed) { (void)speed; return dev_desc; }
static const uint8_t *cb_cfg(uint8_t speed) { (void)speed; return cfg_desc; }
static const uint8_t *cb_qual(uint8_t speed) { (void)speed; return qual_desc; }
static const char *cb_str(uint8_t speed, uint8_t index) {
  (void)speed;
  return index < 4u ? str_desc[index] : NULL;
}
static const struct usb_descriptor g_desc = {
    .device_descriptor_callback = cb_dev,
    .config_descriptor_callback = cb_cfg,
    .device_quality_descriptor_callback = cb_qual,
    .string_descriptor_callback = cb_str,
};

static uint16_t get_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u16(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static uint16_t crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFFu;
  uint8_t bit;
  while (length-- != 0u) {
    crc ^= (uint16_t)*data++ << 8;
    for (bit = 0u; bit < 8u; bit++) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                             : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void arm_out(void) {
  if (s_cfg && !s_out_armed && !s_rx_ready) {
    s_out_armed = 1u;
    (void)usbd_ep_start_read(0, CDC_OUT_EP, s_rx, CDC_MPS);
  }
}

static void reply(uint8_t ok, uint16_t sequence, uint8_t status,
                  uint8_t image_index, uint32_t offset) {
  if (!s_cfg || s_ep_busy) {
    return;
  }
  s_tx[0] = ACK_MAGIC0;
  s_tx[1] = ACK_MAGIC1;
  s_tx[2] = PROTOCOL_VERSION;
  s_tx[3] = ok ? PKT_ACK : PKT_NACK;
  put_u16(&s_tx[4], sequence);
  s_tx[6] = status;
  s_tx[7] = image_index;
  put_u32(&s_tx[8], offset);
  s_ep_busy = 1u;
  (void)usbd_ep_start_write(0, CDC_IN_EP, s_tx, sizeof(s_tx));
}

static void remember(uint16_t sequence, uint8_t status, uint8_t image_index,
                     uint32_t offset) {
  s_last_valid = 1u;
  s_last_seq = sequence;
  s_last_status = status;
  s_last_idx = image_index;
  s_last_offset = offset;
}

static void abort_session(void) {
  s_active = 0u;
  s_total = 0u;
  s_got = 0u;
}

static int erase_image(uint8_t image_index, uint32_t total) {
  uint32_t offset;
  int rc = 0;
  s_busy = 1u;
  for (offset = 0u; offset < total; offset += 4096u) {
    rc = w25q_erase_sector((uint32_t)image_index * IMG_STRIDE + offset);
    if (rc != 0) break;
  }
  s_busy = 0u;
  return rc;
}

static int write_data(uint32_t offset, const uint8_t *data, uint8_t length) {
  uint32_t address = (uint32_t)s_idx * IMG_STRIDE + offset;
  uint8_t written = 0u;
  int rc = 0;
  s_busy = 1u;
  while (written < length) {
    uint16_t room = (uint16_t)(256u - (address & 0xFFu));
    uint8_t chunk = (uint8_t)(length - written);
    if (chunk > room) chunk = (uint8_t)room;
    rc = w25q_write_page_verified(address, &data[written], chunk);
    if (rc != 0) break;
    written = (uint8_t)(written + chunk);
    address += chunk;
  }
  s_busy = 0u;
  return rc;
}

static int persist_ready(void) {
  uint8_t ready[2] = {0xA5u, s_ready_mask};
  int rc;
  s_busy = 1u;
  rc = w25q_erase_sector(READY_ADDR);
  if (rc == 0) rc = w25q_write_page(READY_ADDR, ready, sizeof(ready));
  s_busy = 0u;
  return rc;
}

uint8_t usb_img_load_ready_from_flash(void) {
  uint8_t ready[2];
  s_busy = 1u;
  (void)w25q_read(READY_ADDR, ready, sizeof(ready));
  s_busy = 0u;
  if (ready[0] == 0xA5u) {
    s_ready_mask = (uint8_t)(s_ready_mask | ready[1]);
    return ready[1];
  }
  return 0u;
}

static void handle_begin(uint16_t sequence, uint8_t image_index,
                         const uint8_t *payload, uint8_t length) {
  uint32_t total;
  int rc;
  if (length != 4u || image_index > 2u) {
    reply(0u, sequence, STATUS_FORMAT, image_index, 0u);
    return;
  }
  total = get_u32(payload);
  if (total == 0u || total > IMG_STRIDE) {
    reply(0u, sequence, STATUS_RANGE, image_index, 0u);
    return;
  }
  abort_session();
  s_ready_mask &= (uint8_t)~(1u << image_index);
  rc = erase_image(image_index, total);
  if (rc != 0) {
    reply(0u, sequence, STATUS_FLASH, image_index, 0u);
    return;
  }
  s_idx = image_index;
  s_total = total;
  s_got = 0u;
  s_expected_seq = (uint16_t)(sequence + 1u);
  s_active = 1u;
  remember(sequence, STATUS_OK, image_index, 0u);
  printf("[usb] BEGIN img%u len=%lu base=0x%06lX\n", image_index,
         (unsigned long)total,
         (unsigned long)((uint32_t)image_index * IMG_STRIDE));
  reply(1u, sequence, STATUS_OK, image_index, 0u);
}

static void handle_data(uint16_t sequence, uint8_t image_index, uint32_t offset,
                        const uint8_t *payload, uint8_t length) {
  int rc;
  if (!s_active || image_index != s_idx) {
    reply(0u, sequence, STATUS_STATE, image_index, s_got);
    return;
  }
  if (sequence == s_last_seq && s_last_valid) {
    reply(1u, sequence, s_last_status, s_last_idx, s_last_offset);
    return;
  }
  if (sequence != s_expected_seq) {
    reply(0u, sequence, STATUS_SEQUENCE, s_idx, s_got);
    return;
  }
  if (length == 0u || length > DATA_MAX || offset != s_got ||
      length > s_total - s_got) {
    reply(0u, sequence, STATUS_RANGE, s_idx, s_got);
    return;
  }
  rc = write_data(offset, payload, length);
  if (rc != 0) {
    abort_session();
    reply(0u, sequence, STATUS_FLASH, image_index, offset);
    return;
  }
  s_got += length;
  s_expected_seq = (uint16_t)(sequence + 1u);
  remember(sequence, STATUS_OK, s_idx, s_got);
  reply(1u, sequence, STATUS_OK, s_idx, s_got);
}

static void handle_end(uint16_t sequence, uint8_t image_index, uint32_t offset,
                       uint8_t length) {
  int rc;
  if (!s_active || image_index != s_idx) {
    reply(0u, sequence, STATUS_STATE, image_index, s_got);
    return;
  }
  if (sequence == s_last_seq && s_last_valid) {
    reply(1u, sequence, s_last_status, s_last_idx, s_last_offset);
    return;
  }
  if (sequence != s_expected_seq) {
    reply(0u, sequence, STATUS_SEQUENCE, s_idx, s_got);
    return;
  }
  if (length != 0u || offset != s_total || s_got != s_total) {
    reply(0u, sequence, STATUS_RANGE, s_idx, s_got);
    return;
  }
  s_ready_mask |= (uint8_t)(1u << s_idx);
  rc = persist_ready();
  {
    uint8_t peek[8];
    (void)w25q_read((uint32_t)s_idx * IMG_STRIDE, peek, 8);
    printf("[usb] END img%u verify %02X%02X%02X%02X %02X%02X%02X%02X mask=0x%02X\n",
           image_index, peek[0], peek[1], peek[2], peek[3], peek[4], peek[5],
           peek[6], peek[7], s_ready_mask);
    if (peek[0] == 0xFFu && peek[1] == 0xFFu && peek[2] == 0xFFu &&
        peek[3] == 0xFFu) {
      printf("[usb] WARN flash readback blank — 写入可能未生效（查 WP#/HOLD#）\n");
    }
  }
  if (rc != 0) {
    s_ready_mask &= (uint8_t)~(1u << s_idx);
    abort_session();
    reply(0u, sequence, STATUS_FLASH, image_index, s_got);
    return;
  }
  s_active = 0u;
  remember(sequence, STATUS_OK, image_index, s_got);
  printf("[usb] END img%u written %lu B mask=0x%02X\n", image_index,
         (unsigned long)s_got, s_ready_mask);
  reply(1u, sequence, STATUS_OK, image_index, s_got);
}

static void process_packet(const uint8_t *packet, uint8_t count) {
  uint8_t type;
  uint8_t image_index;
  uint8_t length;
  uint16_t sequence;
  uint32_t offset;
  if (count < HEADER_LEN + CRC_LEN || packet[0] != REQ_MAGIC0 ||
      packet[1] != REQ_MAGIC1 || packet[2] != PROTOCOL_VERSION) {
    reply(0u, 0u, STATUS_FORMAT, 0u, s_got);
    return;
  }
  type = packet[3];
  sequence = get_u16(&packet[4]);
  image_index = packet[6];
  length = packet[7];
  offset = get_u32(&packet[8]);
  if (length > DATA_MAX || count != HEADER_LEN + CRC_LEN + length) {
    reply(0u, sequence, STATUS_FORMAT, image_index, s_got);
    return;
  }
  if (get_u16(&packet[HEADER_LEN + length]) !=
      crc16(packet, (uint8_t)(HEADER_LEN + length))) {
    reply(0u, sequence, STATUS_CRC, image_index, s_got);
    return;
  }
  if (type == PKT_ABORT) {
    abort_session();
    s_last_valid = 0u;
    remember(sequence, STATUS_OK, image_index, 0u);
    reply(1u, sequence, STATUS_OK, image_index, 0u);
    return;
  }
  /* BEGIN 禁止 ACK 重放，必须真正开会话 */
  if (type != PKT_BEGIN && sequence == s_last_seq && s_last_valid) {
    reply(1u, sequence, s_last_status, s_last_idx, s_last_offset);
    return;
  }
  if (type == PKT_BEGIN) {
    s_last_valid = 0u;
    handle_begin(sequence, image_index, &packet[HEADER_LEN], length);
  } else if (type == PKT_DATA) {
    handle_data(sequence, image_index, offset, &packet[HEADER_LEN], length);
  } else if (type == PKT_END) {
    handle_end(sequence, image_index, offset, length);
  } else {
    reply(0u, sequence, STATUS_FORMAT, image_index, s_got);
  }
}

static void usbd_evt(uint8_t busid, uint8_t event) {
  (void)busid;
  if (event == USBD_EVENT_CONFIGURED) {
    s_cfg = 1u;
    s_ep_busy = 0u;
    s_out_armed = 0u;
    s_rx_ready = 0u;
    s_rearm_pending = 0u;
    arm_out();
  } else if (event == USBD_EVENT_RESET || event == USBD_EVENT_DISCONNECTED) {
    s_cfg = 0u;
    s_ep_busy = 0u;
    s_out_armed = 0u;
    s_rx_ready = 0u;
    s_rearm_pending = 0u;
    abort_session();
  }
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t count) {
  (void)busid;
  (void)ep;
  s_out_armed = 0u;
  if (count <= CDC_MPS && !s_rx_ready) {
    s_rx_len = (uint8_t)count;
    s_rx_ready = 1u;
  }
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t count) {
  (void)busid;
  (void)ep;
  (void)count;
  s_ep_busy = 0u;
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr) {
  (void)busid;
  (void)intf;
  (void)dtr;
}

struct usbd_endpoint ep_out = {.ep_addr = CDC_OUT_EP,
                               .ep_cb = usbd_cdc_acm_bulk_out};
struct usbd_endpoint ep_in = {.ep_addr = CDC_IN_EP,
                              .ep_cb = usbd_cdc_acm_bulk_in};
static struct usbd_interface intf0;
static struct usbd_interface intf1;

void usb_img_init(void) {
  usbd_desc_register(0, &g_desc);
  usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf0));
  usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &intf1));
  usbd_add_endpoint(0, &ep_out);
  usbd_add_endpoint(0, &ep_in);
  usbd_initialize(0, USB_BASE, usbd_evt);
  printf("[usb] CDC %04X:%04X packet-ack\n", (unsigned)USBD_VID,
         (unsigned)USBD_PID);
}

void usb_img_poll(void) {
  if (s_rx_ready) {
    uint8_t count = s_rx_len;
    s_rx_ready = 0u;
    process_packet(s_rx, count);
    s_rearm_pending = 1u;
  }
  if (s_rearm_pending && !s_ep_busy) {
    s_rearm_pending = 0u;
    arm_out();
  }
}

uint8_t usb_img_ready_mask(void) { return s_ready_mask; }
uint8_t usb_img_busy(void) { return s_busy; }
void usb_img_set_ready_mask(uint8_t mask) { s_ready_mask |= mask; }
