/**
 * @file ffl/impact_displacement.h
 * @brief 冲击位移估算算法头文件
 *
 * 定义了冲击位移估算算法相关的常量、枚举、结构体和函数接口
 */
#ifndef FFL_IMPACT_DISPLACEMENT_H
#define FFL_IMPACT_DISPLACEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @enum ffl_impact_displacement_status_t
 * @brief 冲击位移算法返回状态枚举
 */
typedef enum {
  FFL_IMPACT_DISPLACEMENT_OK = 0,           /**< 操作成功 */
  FFL_IMPACT_DISPLACEMENT_ERR_ARG = -1,     /**< 参数错误 */
  FFL_IMPACT_DISPLACEMENT_ERR_STATE = -2,   /**< 状态错误 */
  FFL_IMPACT_DISPLACEMENT_ERR_NOT_READY = -3/**< 操作未就绪 */
} ffl_impact_displacement_status_t;

/**
 * @enum ffl_impact_displacement_state_t
 * @brief 冲击位移算法状态枚举
 */
typedef enum {
  FFL_IMPACT_DISPLACEMENT_STATE_UNINIT = 0,     /**< 未初始化 */
  FFL_IMPACT_DISPLACEMENT_STATE_IDLE = 1,       /**< 空闲 */
  FFL_IMPACT_DISPLACEMENT_STATE_COLLECTING = 2, /**< 数据收集中 */
  FFL_IMPACT_DISPLACEMENT_STATE_FINISHED = 3,   /**< 完成 */
  FFL_IMPACT_DISPLACEMENT_STATE_ERROR = 4       /**< 错误 */
} ffl_impact_displacement_state_t;

/**
 * @enum Quality Flags
 * @brief 质量标志位枚举
 */
enum {
  FFL_IMPACT_DISPLACEMENT_QF_NONE = 0u,                             /**< 无质量问题 */
  FFL_IMPACT_DISPLACEMENT_QF_TS_NON_MONOTONIC = (1u << 0),          /**< 时间戳非单调递增 */
  FFL_IMPACT_DISPLACEMENT_QF_DT_GAP = (1u << 1),                    /**< 时间间隔过大 */
  FFL_IMPACT_DISPLACEMENT_QF_CLIPPED = (1u << 2),                   /**< 数据被截断 */
  FFL_IMPACT_DISPLACEMENT_QF_NO_RELEASE = (1u << 3),                /**< 未检测到释放 */
  FFL_IMPACT_DISPLACEMENT_QF_ROTATION_HIGH = (1u << 4),             /**< 旋转角度过大 */
  FFL_IMPACT_DISPLACEMENT_QF_LOW_SAMPLE_COUNT = (1u << 5),          /**< 采样点数太少 */
  FFL_IMPACT_DISPLACEMENT_QF_DURATION_TOO_SHORT = (1u << 6)         /**< 事件持续时间太短 */
};

/**
 * @enum Sample Flags
 * @brief 采样数据标志位枚举
 */
enum {
  FFL_IMPACT_DISPLACEMENT_SAMPLE_CLIPPED = (1u << 0),               /**< 采样数据被截断 */
  /* The caller has independently confirmed that this sample is quiet and can
   * be used as release evidence. This is an in-memory algorithm handoff flag;
   * it is not part of a serialized transport payload. It lets the generic
   * measurement tail use the same evidence after the device settles in a new
   * static attitude. */
  FFL_IMPACT_DISPLACEMENT_SAMPLE_RELEASE_QUALIFIED = (1u << 1)
};

/**
 * @struct ffl_impact_displacement_cfg_t
 * @brief 冲击位移算法配置结构体
 */
typedef struct {
  uint16_t sample_rate_hz;            /**< 采样率(Hz) */
  uint16_t min_event_ms;              /**< 最小事件持续时间(毫秒) */
  uint16_t max_event_ms;              /**< 最大事件持续时间(毫秒) */
  uint16_t release_threshold_mg;      /**< 释放阈值(毫单位重力) */
  uint16_t release_count_min;         /**< 最小连续释放次数 */
  uint16_t max_dt_ms;                 /**< 最大时间间隔(毫秒) */
  float rotation_limit_deg;           /**< 旋转角度限制(度) */
  /* 0表示禁用。启用时，使用EMA(低通滤波)估计重力矢量，
   * 并减去它以获得动态加速度。在PC验证期间，有助于减少
   * 由于较长/慢速运动中的轻微倾斜而引起的重力泄漏。 */
  uint16_t gravity_ema_tau_ms;        /**< 重力估计的EMA时间常数(毫秒) */
  uint8_t enable_zero_velocity_correction; /**< 是否启用零速度修正 */
  uint8_t reserved[1];                /**< 保留字段 */
} ffl_impact_displacement_cfg_t;

/**
 * @struct ffl_impact_displacement_sample_t
 * @brief 冲击位移算法输入采样数据结构体
 */
typedef struct {
  uint32_t timestamp_us;              /**< 时间戳(微秒) */
  int16_t ax_mg;                      /**< X轴加速度(毫单位重力) */
  int16_t ay_mg;                      /**< Y轴加速度(毫单位重力) */
  int16_t az_mg;                      /**< Z轴加速度(毫单位重力) */
  uint8_t flags;                      /**< 标志位 */
  uint8_t reserved[3];                /**< 保留字段 */
} ffl_impact_displacement_sample_t;

/**
 * @struct ffl_impact_displacement_result_t
 * @brief 冲击位移算法输出结果结构体
 */
typedef struct {
  uint32_t event_id;                  /**< 事件ID */
  uint32_t duration_ms;               /**< 事件持续时间(毫秒) */
  uint32_t sample_count;              /**< 采样点数量 */
  float dx_mm;                        /**< X轴位移(毫米) */
  float dy_mm;                        /**< Y轴位移(毫米) */
  float dz_mm;                        /**< Z轴位移(毫米) */
  float disp_mm;                      /**< 总位移(毫米) */
  float peak_acc_mg;                  /**< 峰值加速度(毫单位重力) */
  float terminal_speed_mm_s;          /**< 终端速度(毫米/秒) */
  float rotation_error_mg;            /**< 旋转误差(毫单位重力) */
  uint8_t confidence;                 /**< 置信度(百分比) */
  uint8_t reserved[3];                /**< 保留字段 */
  uint32_t quality_flags;             /**< 质量标志 */
} ffl_impact_displacement_result_t;

/**
 * @struct ffl_impact_displacement_ctx_t
 * @brief 冲击位移算法上下文结构体
 */
typedef struct {
  ffl_impact_displacement_cfg_t cfg;              /**< 配置参数 */
  ffl_impact_displacement_state_t state;          /**< 当前状态 */
  uint32_t event_id;                  /**< 事件ID */
  uint32_t quality_flags;             /**< 质量标志 */
  uint32_t first_ts_us;               /**< 首个时间戳(微秒) */
  uint32_t last_ts_us;                /**< 最后时间戳(微秒) */
  uint32_t sample_count;              /**< 采样计数 */
  uint16_t release_count;             /**< 释放计数 */
  uint16_t reserved0;                 /**< 保留字段 */

  float baseline_ax_mg;               /**< X轴基准加速度(毫单位重力) */
  float baseline_ay_mg;               /**< Y轴基准加速度(毫单位重力) */
  float baseline_az_mg;               /**< Z轴基准加速度(毫单位重力) */
  float g_hat_ax_mg;                  /**< X轴重力估计(毫单位重力) */
  float g_hat_ay_mg;                  /**< Y轴重力估计(毫单位重力) */
  float g_hat_az_mg;                  /**< Z轴重力估计(毫单位重力) */
  uint8_t baseline_set;               /**< 基准是否已设置 */
  uint8_t g_hat_set;                  /**< 重力估计是否已设置 */
  uint8_t has_prev;                   /**< 是否有前一个样本 */
  uint8_t reserved1[1];               /**< 保留字段 */

  float prev_ax_mps2;                 /**< 前一个X轴加速度(m/s²) */
  float prev_ay_mps2;                 /**< 前一个Y轴加速度(m/s²) */
  float prev_az_mps2;                 /**< 前一个Z轴加速度(m/s²) */

  float vx_mps;                       /**< X轴速度(m/s) */
  float vy_mps;                       /**< Y轴速度(m/s) */
  float vz_mps;                       /**< Z轴速度(m/s) */
  float x_m;                          /**< X轴位移(m) */
  float y_m;                          /**< Y轴位移(m) */
  float z_m;                          /**< Z轴位移(m) */

  float peak_acc_mg;                  /**< 峰值加速度(毫单位重力) */
  float tail_sum_ax_mg;               /**< X轴尾部累加和(毫单位重力) */
  float tail_sum_ay_mg;               /**< Y轴尾部累加和(毫单位重力) */
  float tail_sum_az_mg;               /**< Z轴尾部累加和(毫单位重力) */
  uint32_t tail_count;                /**< 尾部计数 */
} ffl_impact_displacement_ctx_t;

/**
 * @brief 获取冲击位移算法版本号
 *
 * @return 版本号字符串
 */
const char *ffl_impact_displacement_get_version(void);

/**
 * @brief 获取默认配置
 *
 * @param cfg 输出配置结构体指针
 */
void ffl_impact_displacement_get_default_cfg(ffl_impact_displacement_cfg_t *cfg);

/**
 * @brief 初始化冲击位移算法
 *
 * @param ctx 上下文结构体指针
 * @param cfg 配置结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_init(ffl_impact_displacement_ctx_t *ctx,
                                      const ffl_impact_displacement_cfg_t *cfg);

/**
 * @brief 重置冲击位移算法
 *
 * @param ctx 上下文结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_reset(ffl_impact_displacement_ctx_t *ctx);

/**
 * @brief 开始一个新事件
 *
 * @param ctx 上下文结构体指针
 * @param event_id 事件ID
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_begin_event(ffl_impact_displacement_ctx_t *ctx,
                                             uint32_t event_id);

/**
 * @brief 设置基准值(毫单位重力)
 *
 * @param ctx 上下文结构体指针
 * @param bx_mg X轴基准值
 * @param by_mg Y轴基准值
 * @param bz_mg Z轴基准值
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_set_baseline_mg(ffl_impact_displacement_ctx_t *ctx,
                                                 float bx_mg, float by_mg,
                                                 float bz_mg);

/**
 * @brief 馈送一个采样数据
 *
 * @param ctx 上下文结构体指针
 * @param sample 采样数据结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_feed_sample(ffl_impact_displacement_ctx_t *ctx,
                                             const ffl_impact_displacement_sample_t *sample);

/**
 * @brief 结束事件并获取结果
 *
 * @param ctx 上下文结构体指针
 * @param out 输出结果结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_end_event(ffl_impact_displacement_ctx_t *ctx,
                                           ffl_impact_displacement_result_t *out);

/**
 * @brief 处理整个事件
 *
 * @param ctx 上下文结构体指针
 * @param samples 采样数据数组指针
 * @param sample_count 采样数据数量
 * @param pre_samples 预采样数量
 * @param event_id 事件ID
 * @param out 输出结果结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_process_event(ffl_impact_displacement_ctx_t *ctx,
                                               const ffl_impact_displacement_sample_t *samples,
                                               uint32_t sample_count,
                                               uint16_t pre_samples,
                                               uint32_t event_id,
                                               ffl_impact_displacement_result_t *out);

/**
 * @brief 获取当前状态
 *
 * @param ctx 上下文结构体指针
 * @return 当前状态
 */
ffl_impact_displacement_state_t ffl_impact_displacement_get_state(const ffl_impact_displacement_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* FFL_IMPACT_DISPLACEMENT_H */
