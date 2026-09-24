#ifndef ULOG_H
#define ULOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ULOG_ENABLE
#define ULOG_ENABLE 0
#endif

typedef enum {
  ULOG_LEVEL_TRACE = 0,
  ULOG_LEVEL_DEBUG = 1,
  ULOG_LEVEL_INFO = 2,
  ULOG_LEVEL_WARN = 3,
  ULOG_LEVEL_ERROR = 4
} ulog_level_t;

#ifndef ULOG_LEVEL_MIN
#define ULOG_LEVEL_MIN ULOG_LEVEL_INFO
#endif

#ifndef ULOG_BUFFER_SIZE
#define ULOG_BUFFER_SIZE 1024u
#endif

#ifndef ULOG_LINE_MAX
#define ULOG_LINE_MAX 160u
#endif

#ifndef ULOG_POLL_BUDGET
#define ULOG_POLL_BUDGET 128u
#endif

#ifndef ULOG_ERROR_DIRECT_ENABLE
#define ULOG_ERROR_DIRECT_ENABLE 1
#endif

#ifndef ULOG_ISR_LINE_MAX
#define ULOG_ISR_LINE_MAX 96u
#endif

/** 尽力发送；返回实际接受的字节数（0=忙/满） */
typedef int (*ulog_tx_try_fn_t)(void *ctx, const uint8_t *data, uint16_t len);

typedef struct {
  ulog_tx_try_fn_t tx_try;
  ulog_tx_try_fn_t tx_direct;
  void *tx_ctx;
  uint16_t poll_budget;
} ulog_init_t;

typedef struct {
  uint32_t enqueued_records;
  uint32_t dropped_records;
  uint32_t overwritten_records;
  uint32_t tx_bytes;
  uint16_t ring_used;
  uint16_t ring_high_watermark;
  uint16_t staging_pending;
} ulog_stats_t;

int ulog_init(const ulog_init_t *cfg);
void ulog_reset(void);
bool ulog_is_ready(void);

int ulog_writef(ulog_level_t level, const char *fmt, ...);
int ulog_vwritef(ulog_level_t level, const char *fmt, va_list ap);
int ulog_write_isr(ulog_level_t level, const void *data, uint16_t len);

void ulog_poll(void);
void ulog_flush_emergency(void);
void ulog_get_stats(ulog_stats_t *stats);

const char *ulog_level_name(ulog_level_t level);

#if ULOG_ENABLE
#if ULOG_LEVEL_MIN <= ULOG_LEVEL_TRACE
#define ULOGT(...) ((void)ulog_writef(ULOG_LEVEL_TRACE, __VA_ARGS__))
#else
#define ULOGT(...) ((void)0)
#endif
#if ULOG_LEVEL_MIN <= ULOG_LEVEL_DEBUG
#define ULOGD(...) ((void)ulog_writef(ULOG_LEVEL_DEBUG, __VA_ARGS__))
#else
#define ULOGD(...) ((void)0)
#endif
#if ULOG_LEVEL_MIN <= ULOG_LEVEL_INFO
#define ULOGI(...) ((void)ulog_writef(ULOG_LEVEL_INFO, __VA_ARGS__))
#else
#define ULOGI(...) ((void)0)
#endif
#if ULOG_LEVEL_MIN <= ULOG_LEVEL_WARN
#define ULOGW(...) ((void)ulog_writef(ULOG_LEVEL_WARN, __VA_ARGS__))
#else
#define ULOGW(...) ((void)0)
#endif
#if ULOG_LEVEL_MIN <= ULOG_LEVEL_ERROR
#define ULOGE(...) ((void)ulog_writef(ULOG_LEVEL_ERROR, __VA_ARGS__))
#else
#define ULOGE(...) ((void)0)
#endif
#else
#define ULOGT(...) ((void)0)
#define ULOGD(...) ((void)0)
#define ULOGI(...) ((void)0)
#define ULOGW(...) ((void)0)
#define ULOGE(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif
