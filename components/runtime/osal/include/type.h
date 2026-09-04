#ifndef FFL_OSAL_TYPE_H
#define FFL_OSAL_TYPE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZSUCCESS 1U
#define INVALID_TASK 2U
#define INVALID_MSG_POINTER 3U
#define INVALID_EVENT_ID 4U
#define NO_TIMER_AVAIL 5U
#define TASK_NO_TASK 6U
#define MSG_BUFFER_NOT_AVAIL 7U

typedef unsigned char BOOL;
typedef unsigned int halDataAlign_t;
typedef uint8_t uint8;
typedef uint8_t byte;
typedef uint16_t uint16;
typedef uint16_t int16U;
typedef uint32_t uint32;
typedef uint32_t int32U;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;

#ifndef FALSE
#define FALSE 0U
#endif
#ifndef TRUE
#define TRUE 1U
#endif
#ifndef ARRAY_NULL
#define ARRAY_NULL '\0'
#endif
#ifndef OPEN
#define OPEN 1U
#endif
#ifndef CLOSE
#define CLOSE 0U
#endif
#ifndef HIGH
#define HIGH 1U
#endif
#ifndef LOW
#define LOW 0U
#endif
#ifndef SUCCESS
#define SUCCESS 1U
#endif
#ifndef ERROR
#define ERROR 0U
#endif

typedef void (*osal_critical_hook_t)(void);

void osal_port_set_critical_hooks(osal_critical_hook_t enter_hook, osal_critical_hook_t exit_hook);
void osal_port_enter_critical(void);
void osal_port_exit_critical(void);

#define HAL_ENTER_CRITICAL_SECTION() osal_port_enter_critical()
#define HAL_EXIT_CRITICAL_SECTION() osal_port_exit_critical()

#ifdef __cplusplus
}
#endif

#endif /* FFL_OSAL_TYPE_H */
