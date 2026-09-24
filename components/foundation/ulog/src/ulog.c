/**
 * @file ulog.c
 * @brief 轻量异步日志：环形缓冲 + tx_try 泵出。
 *
 * 自 CH32/CH58x 工程的 lib/ulog 移植：
 *   - 去掉 CH58x_common.h / tmos_*
 *   - 临界区默认用 Cortex-M PRIMASK，可覆盖 ulog_port_lock/unlock
 *   - memcpy/memset 使用标准 C 库
 */
#include "ulog.h"

#include <stdio.h>
#include <string.h>

#if (ULOG_BUFFER_SIZE < 64u)
#error "ULOG_BUFFER_SIZE too small"
#endif
#if (ULOG_LINE_MAX < 32u)
#error "ULOG_LINE_MAX too small"
#endif

#ifndef ULOG_PORT_CUSTOM
#if defined(__arm__) || defined(__ARM_ARCH)
static uint32_t ulog_lock(void) {
  uint32_t primask;
  __asm volatile("mrs %0, primask" : "=r"(primask));
  __asm volatile("cpsid i" ::: "memory");
  return primask;
}
static void ulog_unlock(uint32_t primask) {
  __asm volatile("msr primask, %0" ::"r"(primask) : "memory");
}
#else
static uint32_t ulog_lock(void) { return 0u; }
static void ulog_unlock(uint32_t s) { (void)s; }
#endif
#else
uint32_t ulog_port_lock(void);
void ulog_port_unlock(uint32_t state);
#define ulog_lock ulog_port_lock
#define ulog_unlock ulog_port_unlock
#endif

typedef struct {
  uint16_t payload_len;
  uint8_t level;
  uint8_t reserved;
} ulog_record_hdr_t;

typedef struct {
  uint8_t initialized;
  ulog_init_t cfg;
  uint16_t ring_head;
  uint16_t ring_tail;
  uint16_t ring_used;
  uint16_t ring_high_watermark;
  uint16_t staging_len;
  uint16_t staging_off;
  uint32_t enqueued_records;
  uint32_t dropped_records;
  uint32_t overwritten_records;
  uint32_t tx_bytes;
  uint8_t ring[ULOG_BUFFER_SIZE];
  uint8_t staging[ULOG_LINE_MAX];
} ulog_ctx_t;

static ulog_ctx_t s_ulog;

static uint16_t ulog_ring_free(const ulog_ctx_t *ctx) {
  return (uint16_t)(ULOG_BUFFER_SIZE - ctx->ring_used);
}

static uint16_t ulog_ring_advance(uint16_t index, uint16_t delta) {
  index = (uint16_t)(index + delta);
  if (index >= ULOG_BUFFER_SIZE) {
    index = (uint16_t)(index - ULOG_BUFFER_SIZE);
  }
  return index;
}

static void ulog_ring_write_bytes(ulog_ctx_t *ctx, uint16_t index,
                                  const void *data, uint16_t len) {
  uint16_t first;
  if ((ctx == NULL) || (data == NULL) || (len == 0u)) {
    return;
  }
  first = (uint16_t)(ULOG_BUFFER_SIZE - index);
  if (first > len) {
    first = len;
  }
  memcpy(&ctx->ring[index], data, first);
  if (len > first) {
    memcpy(&ctx->ring[0], ((const uint8_t *)data) + first,
           (size_t)(len - first));
  }
}

static void ulog_ring_read_bytes(const ulog_ctx_t *ctx, uint16_t index,
                                 void *data, uint16_t len) {
  uint16_t first;
  if ((ctx == NULL) || (data == NULL) || (len == 0u)) {
    return;
  }
  first = (uint16_t)(ULOG_BUFFER_SIZE - index);
  if (first > len) {
    first = len;
  }
  memcpy(data, &ctx->ring[index], first);
  if (len > first) {
    memcpy(((uint8_t *)data) + first, &ctx->ring[0], (size_t)(len - first));
  }
}

static int ulog_ring_drop_oldest_locked(ulog_ctx_t *ctx) {
  ulog_record_hdr_t hdr;
  uint16_t record_len;
  if ((ctx == NULL) || (ctx->ring_used < sizeof(hdr))) {
    return -1;
  }
  ulog_ring_read_bytes(ctx, ctx->ring_tail, &hdr, (uint16_t)sizeof(hdr));
  record_len = (uint16_t)(sizeof(hdr) + hdr.payload_len);
  if ((hdr.payload_len == 0u) || (record_len > ctx->ring_used) ||
      (record_len > ULOG_BUFFER_SIZE)) {
    ctx->ring_head = 0u;
    ctx->ring_tail = 0u;
    ctx->ring_used = 0u;
    ctx->staging_len = 0u;
    ctx->staging_off = 0u;
    ctx->dropped_records++;
    return -1;
  }
  ctx->ring_tail = ulog_ring_advance(ctx->ring_tail, record_len);
  ctx->ring_used = (uint16_t)(ctx->ring_used - record_len);
  ctx->overwritten_records++;
  ctx->dropped_records++;
  return 0;
}

static int ulog_ring_push_record(ulog_level_t level, const uint8_t *payload,
                                 uint16_t payload_len) {
  ulog_record_hdr_t hdr;
  uint16_t record_len;
  uint32_t irq_state;
  if ((payload == NULL) || (payload_len == 0u)) {
    return -1;
  }
  record_len = (uint16_t)(sizeof(hdr) + payload_len);
  if ((record_len > ULOG_BUFFER_SIZE) || (payload_len > ULOG_LINE_MAX)) {
    s_ulog.dropped_records++;
    return -1;
  }
  hdr.payload_len = payload_len;
  hdr.level = (uint8_t)level;
  hdr.reserved = 0u;
  irq_state = ulog_lock();
  while (ulog_ring_free(&s_ulog) < record_len) {
    if (ulog_ring_drop_oldest_locked(&s_ulog) != 0) {
      break;
    }
  }
  if (ulog_ring_free(&s_ulog) >= record_len) {
    ulog_ring_write_bytes(&s_ulog, s_ulog.ring_head, &hdr,
                          (uint16_t)sizeof(hdr));
    s_ulog.ring_head =
        ulog_ring_advance(s_ulog.ring_head, (uint16_t)sizeof(hdr));
    ulog_ring_write_bytes(&s_ulog, s_ulog.ring_head, payload, payload_len);
    s_ulog.ring_head = ulog_ring_advance(s_ulog.ring_head, payload_len);
    s_ulog.ring_used = (uint16_t)(s_ulog.ring_used + record_len);
    if (s_ulog.ring_used > s_ulog.ring_high_watermark) {
      s_ulog.ring_high_watermark = s_ulog.ring_used;
    }
    s_ulog.enqueued_records++;
    ulog_unlock(irq_state);
    return (int)payload_len;
  }
  s_ulog.dropped_records++;
  ulog_unlock(irq_state);
  return -1;
}

static int ulog_stage_next_record(void) {
  ulog_record_hdr_t hdr;
  uint16_t record_len;
  uint32_t irq_state = ulog_lock();
  if (s_ulog.ring_used < sizeof(hdr)) {
    ulog_unlock(irq_state);
    return 0;
  }
  ulog_ring_read_bytes(&s_ulog, s_ulog.ring_tail, &hdr,
                       (uint16_t)sizeof(hdr));
  record_len = (uint16_t)(sizeof(hdr) + hdr.payload_len);
  if ((hdr.payload_len == 0u) || (hdr.payload_len > ULOG_LINE_MAX) ||
      (record_len > s_ulog.ring_used)) {
    s_ulog.ring_head = 0u;
    s_ulog.ring_tail = 0u;
    s_ulog.ring_used = 0u;
    s_ulog.dropped_records++;
    ulog_unlock(irq_state);
    return -1;
  }
  ulog_ring_read_bytes(&s_ulog,
                       ulog_ring_advance(s_ulog.ring_tail,
                                         (uint16_t)sizeof(hdr)),
                       s_ulog.staging, hdr.payload_len);
  s_ulog.staging_len = hdr.payload_len;
  s_ulog.staging_off = 0u;
  s_ulog.ring_tail = ulog_ring_advance(s_ulog.ring_tail, record_len);
  s_ulog.ring_used = (uint16_t)(s_ulog.ring_used - record_len);
  ulog_unlock(irq_state);
  return 1;
}

static uint16_t ulog_build_prefix(ulog_level_t level, uint8_t *line,
                                  uint16_t capacity) {
  const char *tag;
  if ((line == NULL) || (capacity < 4u)) {
    return 0u;
  }
  tag = ulog_level_name(level);
  line[0] = '[';
  line[1] = (uint8_t)tag[0];
  line[2] = ']';
  line[3] = ' ';
  return 4u;
}

static uint16_t ulog_finalize_line(uint8_t *line, uint16_t len,
                                   uint16_t capacity) {
  if ((line == NULL) || (capacity == 0u)) {
    return 0u;
  }
  if (len >= capacity) {
    len = (uint16_t)(capacity - 1u);
  }
  if ((len == 0u) || (line[len - 1u] != '\n')) {
    if ((len + 2u) <= capacity) {
      line[len++] = '\r';
      line[len++] = '\n';
    } else if ((len + 1u) <= capacity) {
      line[len++] = '\n';
    }
  }
  return len;
}

static int ulog_compose_format(ulog_level_t level, const char *fmt, va_list ap,
                               uint8_t *line, uint16_t capacity) {
  uint16_t prefix_len;
  int body_len;
  if ((fmt == NULL) || (line == NULL) || (capacity == 0u)) {
    return -1;
  }
  prefix_len = ulog_build_prefix(level, line, capacity);
  if (prefix_len >= capacity) {
    return -1;
  }
  body_len = vsnprintf((char *)&line[prefix_len],
                       (size_t)(capacity - prefix_len), fmt, ap);
  if (body_len < 0) {
    return -1;
  }
  if ((uint16_t)body_len >= (uint16_t)(capacity - prefix_len)) {
    prefix_len = (uint16_t)(capacity - 1u);
  } else {
    prefix_len = (uint16_t)(prefix_len + (uint16_t)body_len);
  }
  return (int)ulog_finalize_line(line, prefix_len, capacity);
}

static int ulog_compose_raw(ulog_level_t level, const void *data, uint16_t len,
                            uint8_t *line, uint16_t capacity) {
  uint16_t prefix_len;
  uint16_t copy_len;
  if ((data == NULL) || (len == 0u) || (line == NULL) || (capacity == 0u)) {
    return -1;
  }
  prefix_len = ulog_build_prefix(level, line, capacity);
  if (prefix_len >= capacity) {
    return -1;
  }
  copy_len = len;
  if (copy_len > (uint16_t)(capacity - prefix_len)) {
    copy_len = (uint16_t)(capacity - prefix_len);
  }
  memcpy(&line[prefix_len], data, copy_len);
  return (int)ulog_finalize_line(line, (uint16_t)(prefix_len + copy_len),
                                 capacity);
}

static int ulog_try_direct_send(ulog_level_t level, const uint8_t *data,
                                uint16_t len) {
  int rc;
  if ((level != ULOG_LEVEL_ERROR) || (ULOG_ERROR_DIRECT_ENABLE == 0) ||
      (s_ulog.cfg.tx_direct == NULL)) {
    return 0;
  }
  rc = s_ulog.cfg.tx_direct(s_ulog.cfg.tx_ctx, data, len);
  if (rc == (int)len) {
    s_ulog.tx_bytes += len;
    return (int)len;
  }
  return 0;
}

static void ulog_pump(uint16_t budget) {
  uint16_t chunk_len;
  int rc;
  if ((s_ulog.initialized == 0u) || (s_ulog.cfg.tx_try == NULL)) {
    return;
  }
  while (budget > 0u) {
    if (s_ulog.staging_off >= s_ulog.staging_len) {
      s_ulog.staging_off = 0u;
      s_ulog.staging_len = 0u;
      if (ulog_stage_next_record() <= 0) {
        break;
      }
    }
    chunk_len = (uint16_t)(s_ulog.staging_len - s_ulog.staging_off);
    if (chunk_len > budget) {
      chunk_len = budget;
    }
    rc = s_ulog.cfg.tx_try(s_ulog.cfg.tx_ctx, &s_ulog.staging[s_ulog.staging_off],
                           chunk_len);
    if (rc <= 0) {
      break;
    }
    if ((uint16_t)rc > chunk_len) {
      rc = (int)chunk_len;
    }
    s_ulog.staging_off =
        (uint16_t)(s_ulog.staging_off + (uint16_t)rc);
    s_ulog.tx_bytes += (uint32_t)rc;
    budget = (uint16_t)(budget - (uint16_t)rc);
  }
}

int ulog_init(const ulog_init_t *cfg) {
  if (cfg == NULL) {
    return -1;
  }
  memset(&s_ulog, 0, sizeof(s_ulog));
  s_ulog.cfg = *cfg;
  if (s_ulog.cfg.poll_budget == 0u) {
    s_ulog.cfg.poll_budget = ULOG_POLL_BUDGET;
  }
  s_ulog.initialized = (cfg->tx_try != NULL) ? 1u : 0u;
  return s_ulog.initialized ? 0 : -1;
}

void ulog_reset(void) {
  uint32_t irq_state = ulog_lock();
  memset(&s_ulog, 0, sizeof(s_ulog));
  ulog_unlock(irq_state);
}

bool ulog_is_ready(void) { return (s_ulog.initialized != 0u); }

int ulog_vwritef(ulog_level_t level, const char *fmt, va_list ap) {
  uint8_t line[ULOG_LINE_MAX];
  int line_len;
  if ((s_ulog.initialized == 0u) || (fmt == NULL)) {
    return -1;
  }
  line_len = ulog_compose_format(level, fmt, ap, line, (uint16_t)sizeof(line));
  if (line_len <= 0) {
    s_ulog.dropped_records++;
    return -1;
  }
  if (ulog_try_direct_send(level, line, (uint16_t)line_len) == line_len) {
    return line_len;
  }
  return ulog_ring_push_record(level, line, (uint16_t)line_len);
}

int ulog_writef(ulog_level_t level, const char *fmt, ...) {
  int rc;
  va_list ap;
  va_start(ap, fmt);
  rc = ulog_vwritef(level, fmt, ap);
  va_end(ap);
  return rc;
}

int ulog_write_isr(ulog_level_t level, const void *data, uint16_t len) {
  uint8_t line[ULOG_ISR_LINE_MAX];
  int line_len;
  if ((s_ulog.initialized == 0u) || (data == NULL) || (len == 0u)) {
    return -1;
  }
  line_len = ulog_compose_raw(level, data, len, line, (uint16_t)sizeof(line));
  if (line_len <= 0) {
    s_ulog.dropped_records++;
    return -1;
  }
  if (ulog_try_direct_send(level, line, (uint16_t)line_len) == line_len) {
    return line_len;
  }
  return ulog_ring_push_record(level, line, (uint16_t)line_len);
}

void ulog_poll(void) {
  if ((s_ulog.initialized == 0u) || (s_ulog.cfg.tx_try == NULL)) {
    return;
  }
  ulog_pump(s_ulog.cfg.poll_budget);
}

void ulog_flush_emergency(void) {
  if ((s_ulog.initialized == 0u) || (s_ulog.cfg.tx_try == NULL)) {
    return;
  }
  ulog_pump(0xFFFFu);
}

void ulog_get_stats(ulog_stats_t *stats) {
  uint32_t irq_state;
  if (stats == NULL) {
    return;
  }
  irq_state = ulog_lock();
  memset(stats, 0, sizeof(*stats));
  stats->enqueued_records = s_ulog.enqueued_records;
  stats->dropped_records = s_ulog.dropped_records;
  stats->overwritten_records = s_ulog.overwritten_records;
  stats->tx_bytes = s_ulog.tx_bytes;
  stats->ring_used = s_ulog.ring_used;
  stats->ring_high_watermark = s_ulog.ring_high_watermark;
  stats->staging_pending =
      (uint16_t)(s_ulog.staging_len - s_ulog.staging_off);
  ulog_unlock(irq_state);
}

const char *ulog_level_name(ulog_level_t level) {
  switch (level) {
  case ULOG_LEVEL_TRACE:
    return "T";
  case ULOG_LEVEL_DEBUG:
    return "D";
  case ULOG_LEVEL_INFO:
    return "I";
  case ULOG_LEVEL_WARN:
    return "W";
  case ULOG_LEVEL_ERROR:
    return "E";
  default:
    return "?";
  }
}
