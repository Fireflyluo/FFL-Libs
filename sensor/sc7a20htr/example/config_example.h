/**
 ******************************************************************************
 * @file    config_example.h
 * @brief   SC7A20HTR 配置示例
 ******************************************************************************
 * @note    本文件展示如何配置SC7A20HTR驱动的异步功能
 *
 ******************************************************************************
 */

#ifndef CONFIG_EXAMPLE_H
#define CONFIG_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

// 启用SC7A20HTR异步功能
#define SC7A20_ASYNC_SUPPORT 1

// 其他可选配置
// #define SC7A20_INTERNAL_BUFFER_SIZE 6    // 内部缓冲区大小（默认6字节）
// #define SC7A20_ASYNC_DEFAULT_TIMEOUT_MS 100  // 默认超时时间（毫秒）

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_EXAMPLE_H */