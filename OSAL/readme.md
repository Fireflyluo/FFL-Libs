# PY32F403 OSAL事件驱动框架

本文件夹内是基于ti的OSAL事件驱动框架。该框架提供了一个事件驱动模式的裸机框架。

## 一、项目概述

## 二、、目录结构

```
OSAL
├── hal         # 硬件相关（移植相关）
├── MenMang     # 内存管理 (freeRTOS)
├── osal_event.c   # 事件相关
├── osal_memory.c  # 内存管理
├── osal_msg.c     # 消息相关
├── osal_pt.c      # 协程相关
├── osal_timer.h   # OSAL 定时器
├── osal.c           
├── type.h         # 类型定义
└── osal_include.h        

```

## 三、主要特性


# PY32F403 OSAL事件驱动框架

基于TI的OSAL（Operating System Abstraction Layer）事件驱动框架，为PY32F403微控制器定制移植。该框架提供轻量级的事件驱动型任务调度，适用于资源受限的嵌入式环境。

## 项目概述

OSAL是一个轻量级的事件驱动操作系统抽象层，支持多任务管理、事件处理、消息传递、定时器和内存管理。它允许开发者以类似多任务的方式编写代码，而无需完整的实时操作系统，特别适合RAM和Flash资源有限的微控制器。


## 主要特性

- **事件驱动架构**：任务基于事件触发，CPU在无事件时进入低功耗状态
- **优先级任务调度**：支持多个任务按优先级调度
- **消息队列**：任务间通过消息缓冲区通信
- **软件定时器**：支持单次和周期定时器
- **动态内存管理**：提供内存分配/释放，支持内存统计和防护
- **协程支持**（可选）：基于Protothreads的轻量级协程
- **临界区保护**：中断安全的临界区保护机制
- **轻量级**：极小内存占用，适合资源受限环境

## 目录结构

```
OSAL/
├── osal.c               # OSAL系统核心
├── osal.h               # OSAL头文件
├── osal_event.c         # 事件管理实现
├── osal_event.h         # 事件管理头文件
├── osal_memory.c        # 内存管理实现
├── osal_memory.h        # 内存管理头文件
├── osal_msg.c           # 消息队列实现
├── osal_msg.h           # 消息队列头文件
├── osal_pt.c            # 协程支持实现
├── osal_pt.h            # 协程支持头文件
├── osal_timer.c         # 软件定时器管理
├── osal_timer.h         # 定时器头文件
├── type.h               # 类型定义和宏
├── timer.c              # 硬件定时器适配（需完善）
├── timer.h              # 硬件定时器头文件
├── protothreads.h       # Protothreads轻量级协程库
└── README.md            # 说明文档
```

## 快速开始

### 1. 硬件定时器适配

首先需要完成硬件定时器的适配，这是系统的心跳来源：

```c
// timer.c 中需要实现以下函数：
#include "timer.h"
#include "osal_timer.h"

// 硬件定时器初始化，设置系统时钟滴答（建议1ms）
void OSAL_TIMER_TICKINIT(void)
{
    // 初始化PY32F403的定时器（如TIM2、TIM3等）
    // 配置为1ms周期中断
}

// 开启硬件定时器
void OSAL_TIMER_TICKSTART(void)
{
    // 启动定时器
    // HAL_TIM_Base_Start_IT(&htim);
}

// 关闭硬件定时器  
void OSAL_TIMER_TICKSTOP(void)
{
    // 停止定时器
    // HAL_TIM_Base_Stop_IT(&htim);
}

// 在定时器中断服务程序中调用：
void TIMx_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim, TIM_FLAG_UPDATE);
        osal_update_timers(); // 更新系统定时器
    }
}
```
或者采用系统定时器
```c
/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler(); // HAL库的处理

  osal_update_timers();

}
```
### 2. 系统初始化

```c
#include "osal.h"

int main(void)
{
    // 硬件初始化
    HAL_Init();
    SystemClock_Config();
    
    // OSAL系统初始化
    osal_init_system();
    
    // 添加用户任务（优先级数字越大，优先级越高）
    osal_add_Task(app_task_init, app_task_event_processor, 1);
    osal_add_Task(led_task_init, led_task_event_processor, 2);
    
    // 初始化所有任务
    osal_Task_init();
    
    // 启动OSAL调度器（不会返回）
    osal_start_system();
    
    while(1);
}
```

### 3. 任务定义示例

```c
// 任务初始化函数
void app_task_init(uint8 task_id)
{
    // 任务初始化代码
    osal_start_reload_timer(task_id, APP_TIMER_EVENT, 1000); // 启动1s周期定时器
}

// 任务事件处理函数
uint16 app_task_event_processor(uint8 task_id, uint16 events)
{
    if (events & APP_TIMER_EVENT) {
        // 处理定时器事件
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        return events & ~APP_TIMER_EVENT; // 清除已处理事件
    }
    
    if (events & APP_MESSAGE_EVENT) {
        // 处理消息事件
        uint8 *msg = osal_msg_receive(task_id);
        if (msg) {
            // 处理消息
            osal_msg_deallocate(msg);
        }
        return events & ~APP_MESSAGE_EVENT;
    }
    
    return 0; // 返回未处理的事件
}
```

### 4. 消息发送示例

```c
// 在中断或任务中发送消息
void send_message_to_task(uint8 dest_task_id, uint8 *data, uint16 len)
{
    uint8 *msg = osal_msg_allocate(len);
    if (msg) {
        memcpy(msg, data, len);
        osal_msg_send(dest_task_id, msg);
    }
}
```
### 5. 协程使用（可选）
#### 启用协程支持

在`osal.h`中启用协程功能：
```c
#define OSAL_PT_ENABLE
```
协程（Protothreads）是一种轻量级的"线程"，可以在单个任务中实现多个并发的执行流程：
- 极低的内存开销（每个协程仅需几个字节）
- 支持阻塞式等待（事件、延时）而不占用CPU
- 基于宏实现的协作式多任务

### 协程创建和使用

#### 1. 定义协程函数

```c
#include "osal_pt.h"

// 协程函数必须返回char类型，使用PT_THREAD宏定义
PT_THREAD(my_coroutine(osal_pt_t *pt, void *arg))
{
    // 协程开始必须使用PT_BEGIN_WRAPPER
    PT_BEGIN_WRAPPER(pt);
    
    // 协程代码
    while(1) {
        // 等待1秒
        PT_DELAY(pt, 1000);
        
        // 执行一些操作
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        
        // 等待特定事件
        PT_WAIT_EVENT(pt, COROUTINE_EVENT);
        
        // 事件触发后的处理
        // ...
    }
    
    // 协程结束必须使用PT_END_WRAPPER
    PT_END_WRAPPER(pt);
}
```

#### 2. 创建和调度协程

```c
// 在任务初始化中创建协程
void app_task_init(uint8 task_id)
{
    osal_pt_scheduler_t sched;
    
    // 初始化协程调度器
    osal_pt_scheduler_init(&sched);
    
    // 创建协程
    osal_pt_create(&sched, my_coroutine, NULL, "my_coro");
    
    // 启动其他协程...
}

// 在任务事件处理中调度协程
uint16 app_task_event_processor(uint8 task_id, uint16 events)
{
    if (events & APP_COROUTINE_EVENT) {
        // 触发协程事件
        osal_pt_set_event(specific_coroutine, COROUTINE_EVENT);
        return events & ~APP_COROUTINE_EVENT;
    }
    
    // 协程调度（需要在主循环中调用）
    osal_pt_schedule(&sched);
    
    return 0;
}
```



### 协程使用示例

#### 示例1：LED闪烁协程
```c
PT_THREAD(led_blink_coroutine(osal_pt_t *pt, void *arg))
{
    static uint8_t blink_count = 0;
    
    PT_BEGIN_WRAPPER(pt);
    
    while(blink_count < 5) {
        // LED亮
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        PT_DELAY(pt, 200);  // 亮200ms
        
        // LED灭  
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        PT_DELAY(pt, 300);  // 灭300ms
        
        blink_count++;
    }
    
    PT_END_WRAPPER(pt);
}
```

#### 示例2：按键检测协程
```c
PT_THREAD(button_check_coroutine(osal_pt_t *pt, void *arg))
{
    PT_BEGIN_WRAPPER(pt);
    
    while(1) {
        // 等待按键按下事件
        PT_WAIT_EVENT(pt, BUTTON_PRESS_EVENT);
        
        // 防抖延时
        PT_DELAY(pt, 50);
        
        // 确认按键仍然按下
        if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) {
            // 执行按键处理
            handle_button_press();
        }
    }
    
    PT_END_WRAPPER(pt);
}
```

#### 示例3：串口通信协程
```c
PT_THREAD(uart_communication_coroutine(osal_pt_t *pt, void *arg))
{
    PT_BEGIN_WRAPPER(pt);
    
    // 发送AT指令
    uart_send_string("AT\r\n");
    PT_DELAY(pt, 100);  // 等待响应
    
    // 等待响应数据
    PT_WAIT_EVENT(pt, UART_RX_EVENT);
    
    // 处理响应
    process_uart_response();
    
    PT_END_WRAPPER(pt);
}
```

### 协程与任务事件集成

协程可以与OSAL任务事件协同工作：

```c
uint16 uart_task_event_processor(uint8 task_id, uint16 events)
{
    if (events & UART_RX_EVENT) {
        // 触发所有等待UART事件的协程
        osal_pt_t *pt = sched.pt_list;
        while(pt) {
            if(pt->wait_events & UART_RX_EVENT) {
                osal_pt_set_event(pt, UART_RX_EVENT);
            }
            pt = pt->next;
        }
        return events & ~UART_RX_EVENT;
    }
    
    // 调度协程
    osal_pt_schedule(&sched);
    
    return 0;
}
```

### 协程最佳实践

1. **避免长时间运行**：协程应该频繁让出CPU，避免阻塞其他协程
2. **合理使用延时**：使用`PT_DELAY`代替忙等待
3. **事件驱动设计**：基于事件而非轮询
4. **资源管理**：协程退出时确保释放分配的资源
5. **错误处理**：在协程中添加适当的超时和错误检查

### 协程状态管理

协程有以下几种状态：
- `PT_STATE_READY` - 就绪，可执行
- `PT_STATE_RUNNING` - 正在运行
- `PT_STATE_WAITING` - 等待事件或延时
- `PT_STATE_EXITED` - 已退出

## 配置选项

### 内存管理配置（osal_memory.h）

```c
#define MAXMEMHEAP 1024*8        // 内存池大小（8KB）
#define OSALMEM_METRICS 1        // 开启内存统计
#define OSALMEM_GUARD TRUE       // 内存防护检测
```



## API参考

### 任务管理
- `osal_add_Task()` - 添加任务
- `osal_set_event()` - 设置任务事件
- `osal_clear_event()` - 清除任务事件

### 定时器管理  
- `osal_start_timerEx()` - 启动单次定时器
- `osal_start_reload_timer()` - 启动周期定时器
- `osal_stop_timerEx()` - 停止定时器

### 消息队列
- `osal_msg_allocate()` - 分配消息缓冲区
- `osal_msg_send()` - 发送消息
- `osal_msg_receive()` - 接收消息
- `osal_msg_deallocate()` - 释放消息缓冲区

### 内存管理
- `osal_mem_alloc()` - 内存分配
- `osal_mem_free()` - 内存释放
- `osal_heap_mem_used()` - 获取内存使用情况


### 协程API参考

#### 基本控制
- `osal_pt_create()` - 创建协程
- `osal_pt_schedule()` - 调度所有协程
- `osal_pt_set_event()` - 设置协程事件

#### 等待宏
- `PT_DELAY(pt, ms)` - 延时等待
- `PT_WAIT_EVENT(pt, event_mask)` - 事件等待
- `PT_WAIT_UNTIL_WRAPPER(pt, condition)` - 条件等待
- `PT_YIELD_WRAPPER(pt)` - 主动让出

## 移植注意事项

1. **定时器适配**：必须根据PY32F403的硬件完成timer.c中的实现
2. **中断优先级**：确保系统定时器中断优先级设置合理
3. **内存大小**：根据实际需求调整`MAXMEMHEAP`大小
4. **任务优先级**：优先级数字越大优先级越高（0-255）
5. **事件标志**：每个任务有16个事件标志（bit0-15）




## 作者信息

- 作者：fireflyluo
- 硬件平台：PY32F403系列微控制器
- 开发环境：ARM GCC/Keil/IAR





