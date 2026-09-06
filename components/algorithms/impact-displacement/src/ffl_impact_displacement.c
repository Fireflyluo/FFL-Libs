/**
 * @file ffl_impact_displacement.c
 * @brief 冲击位移估算算法实现
 *
 * 实现了通过加速度数据估算冲击事件中物体位移的算法
 */
#include "ffl/impact_displacement.h"

#include <stddef.h>
#include <string.h>

#define FFL_IMPACT_DISPLACEMENT_VERSION "0.1.2"     ///< 版本号

#define GRAVITY_MPS2 9.80665f           ///< 重力加速度(m/s²)
#define MG_TO_MPS2 (GRAVITY_MPS2 / 1000.0f)  ///< 毫重力单位转换为m/s²
#define MPS_TO_MMPS 1000.0f             ///< m/s转换为mm/s
#define M_TO_MM 1000.0f                 ///< 米转换为毫米
#define DEG_TO_RAD 0.01745329251994329577f  ///< 角度转换为弧度

static float ffl_impact_displacement_wrap_pi(float x) {
  const float pi = 3.14159265358979323846f;
  const float two_pi = 6.28318530717958647692f;
  while (x > pi) {
    x -= two_pi;
  }
  while (x < -pi) {
    x += two_pi;
  }
  return x;
}

static float ffl_impact_displacement_sinf_soft(float x) {
  float x2;
  x = ffl_impact_displacement_wrap_pi(x);
  if (x > 1.57079632679489661923f) {
    x = 3.14159265358979323846f - x;
  } else if (x < -1.57079632679489661923f) {
    x = -3.14159265358979323846f - x;
  }
  x2 = x * x;
  return x * (1.0f - (x2 / 6.0f) + ((x2 * x2) / 120.0f) - ((x2 * x2 * x2) / 5040.0f));
}

/*
 * Newton iteration converges quadratically only after the initial estimate is
 * near sqrt(x).  The former implementation started at `guess=x` and stopped
 * after eight iterations; for vector squares in mg^2/mm^2 this could still be
 * tens of times too large, corrupting peak acceleration and displacement.
 * Normalize x to [1,4) with powers of four, solve the bounded problem, then
 * restore the scale. This keeps the helper libm-free and deterministic on a
 * resource-constrained MCU while bounding its error over the complete int16
 * sensor domain.
 */
static float ffl_impact_displacement_sqrtf_soft(float x) {
  float scaled;
  float scale;
  float guess;
  int i;

  if (!(x > 0.0f)) {
    return 0.0f;
  }

  scaled = x;
  scale = 1.0f;
  while (scaled >= 4.0f) {
    scaled *= 0.25f;
    scale *= 2.0f;
  }
  while (scaled < 1.0f) {
    scaled *= 4.0f;
    scale *= 0.5f;
  }

  guess = 1.5f;
  for (i = 0; i < 5; ++i) {
    guess = 0.5f * (guess + (scaled / guess));
  }
  return guess * scale;
}

static float ffl_impact_displacement_norm3(float x, float y, float z) {
  return ffl_impact_displacement_sqrtf_soft((x * x) + (y * y) + (z * z));
}

/**
 * @brief 将整数值限制在0-100范围内并转换为uint8_t
 *
 * @param v 输入整数值
 * @return 限制后的uint8_t值
 */
static uint8_t ffl_impact_displacement_clamp_u8(int v) {
  if (v < 0) {
    return 0u;      // 如果小于0，返回0
  }
  if (v > 100) {
    return 100u;    // 如果大于100，返回100
  }
  return (uint8_t)v; // 否则返回原值
}

/**
 * @brief 计算名义时间间隔(微秒)
 *
 * @param cfg 配置结构体指针
 * @return 名义时间间隔(微秒)
 */
static uint32_t ffl_impact_displacement_nominal_dt_us(const ffl_impact_displacement_cfg_t *cfg) {
  if (cfg == NULL || cfg->sample_rate_hz == 0u) {
    return 0u;      // 如果配置为空或采样率为0，返回0
  }
  return (uint32_t)(1000000u / cfg->sample_rate_hz);  // 计算时间间隔
}

/**
 * @brief 根据旋转角度限制计算重力变化阈值
 *
 * @param limit_deg 旋转角度限制(度)
 * @return 重力变化阈值
 */
static float ffl_impact_displacement_rotation_limit_mg(float limit_deg) {
  float half_rad = 0.5f * limit_deg * DEG_TO_RAD;  // 转换为半角弧度
  return 2000.0f * ffl_impact_displacement_sinf_soft(half_rad);  // 计算阈值
}

/**
 * @brief 根据质量标志计算置信度分数
 *
 * @param flags 质量标志
 * @param samples 采样数量
 * @param release_count 释放计数
 * @param release_count_min 最小释放计数
 * @return 置信度分数
 */
static int ffl_impact_displacement_confidence_from_flags(uint32_t flags, uint32_t samples,
                                             uint16_t release_count,
                                             uint16_t release_count_min) {
  int score = 100;  // 初始分数为100

  if (samples < 3u) {  // 如果采样数少于3
    score -= 45;
  }
  if (release_count < release_count_min) {  // 如果释放计数不足
    score -= 25;
  }
  if ((flags & FFL_IMPACT_DISPLACEMENT_QF_ROTATION_HIGH) != 0u) {  // 如果旋转角度过大
    score -= 35;
  }
  if ((flags & FFL_IMPACT_DISPLACEMENT_QF_DT_GAP) != 0u) {  // 如果时间间隔过大
    score -= 10;
  }
  if ((flags & FFL_IMPACT_DISPLACEMENT_QF_CLIPPED) != 0u) {  // 如果数据被截断
    score -= 10;
  }
  if ((flags & FFL_IMPACT_DISPLACEMENT_QF_TS_NON_MONOTONIC) != 0u) {  // 如果时间戳非单调
    score -= 20;
  }
  if ((flags & FFL_IMPACT_DISPLACEMENT_QF_DURATION_TOO_SHORT) != 0u) {  // 如果持续时间太短
    score -= 20;
  }

  return score;  // 返回最终分数
}

/**
 * @brief 获取冲击位移算法版本号
 *
 * @return 版本号字符串
 */
const char *ffl_impact_displacement_get_version(void) { return FFL_IMPACT_DISPLACEMENT_VERSION; }

/**
 * @brief 获取默认配置
 *
 * @param cfg 输出配置结构体指针
 */
void ffl_impact_displacement_get_default_cfg(ffl_impact_displacement_cfg_t *cfg) {
  if (cfg == NULL) {
    return;  // 如果配置指针为空，直接返回
  }

  memset(cfg, 0, sizeof(*cfg));  // 清零配置结构体
  cfg->sample_rate_hz = 400u;    // 采样率400Hz
  cfg->min_event_ms = 40u;       // 最小事件持续时间40ms
  cfg->max_event_ms = 1000u;     // 最大事件持续时间1000ms
  cfg->release_threshold_mg = 80u;  // 释放阈值80mg
  cfg->release_count_min = 8u;      // 最小释放计数8
  cfg->max_dt_ms = 20u;             // 最大时间间隔20ms
  cfg->rotation_limit_deg = 15.0f;  // 旋转角度限制15度
  cfg->gravity_ema_tau_ms = 0u;     // 重力EMA时间常数关闭
  cfg->enable_zero_velocity_correction = 1u;  // 启用零速度修正
}

/**
 * @brief 重置冲击位移算法
 *
 * @param ctx 上下文结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_reset(ffl_impact_displacement_ctx_t *ctx) {
  ffl_impact_displacement_cfg_t backup_cfg;  // 备份配置
  uint8_t had_cfg;               // 是否有配置标志

  if (ctx == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文为空，返回参数错误
  }

  had_cfg = (ctx->cfg.sample_rate_hz != 0u) ? 1u : 0u;  // 检查是否有配置
  backup_cfg = ctx->cfg;  // 备份当前配置
  memset(ctx, 0, sizeof(*ctx));  // 清零上下文

  if (had_cfg != 0u) {  // 如果之前有配置，恢复配置
    ctx->cfg = backup_cfg;
  } else {  // 否则使用默认配置
    ffl_impact_displacement_get_default_cfg(&ctx->cfg);
  }
  ctx->state = FFL_IMPACT_DISPLACEMENT_STATE_IDLE;  // 设置状态为空闲
  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

/**
 * @brief 初始化冲击位移算法
 *
 * @param ctx 上下文结构体指针
 * @param cfg 配置结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_init(ffl_impact_displacement_ctx_t *ctx,
                                      const ffl_impact_displacement_cfg_t *cfg) {
  if (ctx == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文为空，返回参数错误
  }

  memset(ctx, 0, sizeof(*ctx));  // 清零上下文
  if (cfg != NULL) {  // 如果提供了配置，复制配置
    ctx->cfg = *cfg;
  } else {  // 否则使用默认配置
    ffl_impact_displacement_get_default_cfg(&ctx->cfg);
  }

  if (ctx->cfg.sample_rate_hz == 0u) {  // 检查采样率是否有效
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;
  }

  if (ctx->cfg.max_event_ms < ctx->cfg.min_event_ms) {  // 检查事件时间范围是否有效
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;
  }

  if (ctx->cfg.release_count_min == 0u) {  // 检查最小释放计数是否有效
    ctx->cfg.release_count_min = 1u;  // 至少为1
  }

  ctx->state = FFL_IMPACT_DISPLACEMENT_STATE_IDLE;  // 设置状态为空闲
  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

/**
 * @brief 开始一个新事件
 *
 * @param ctx 上下文结构体指针
 * @param event_id 事件ID
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_begin_event(ffl_impact_displacement_ctx_t *ctx,
                                             uint32_t event_id) {
  ffl_impact_displacement_cfg_t backup_cfg;  // 备份配置
  uint8_t baseline_set;          // 基准设置标志
  float bx;                      // X轴基准
  float by;                      // Y轴基准
  float bz;                      // Z轴基准

  if (ctx == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文为空，返回参数错误
  }

  if (ctx->state == FFL_IMPACT_DISPLACEMENT_STATE_UNINIT) {  // 检查状态是否已初始化
    return FFL_IMPACT_DISPLACEMENT_ERR_STATE;
  }

  backup_cfg = ctx->cfg;  // 备份当前配置
  baseline_set = ctx->baseline_set;  // 保存基准设置状态
  bx = ctx->baseline_ax_mg;  // 保存X轴基准
  by = ctx->baseline_ay_mg;  // 保存Y轴基准
  bz = ctx->baseline_az_mg;  // 保存Z轴基准

  memset(ctx, 0, sizeof(*ctx));  // 清零上下文
  ctx->cfg = backup_cfg;  // 恢复配置
  ctx->baseline_set = baseline_set;  // 恢复基准设置状态
  ctx->baseline_ax_mg = bx;  // 恢复X轴基准
  ctx->baseline_ay_mg = by;  // 恢复Y轴基准
  ctx->baseline_az_mg = bz;  // 恢复Z轴基准

  /* 如果启用了EMA重力估计，重新从基准初始化估计器
   * (流式调用者可能在begin_event之前设置基准) */
  if (ctx->cfg.gravity_ema_tau_ms != 0u && ctx->baseline_set != 0u) {
    ctx->g_hat_ax_mg = ctx->baseline_ax_mg;  // 设置X轴重力估计
    ctx->g_hat_ay_mg = ctx->baseline_ay_mg;  // 设置Y轴重力估计
    ctx->g_hat_az_mg = ctx->baseline_az_mg;  // 设置Z轴重力估计
    ctx->g_hat_set = 1u;  // 标记重力估计已设置
  }

  ctx->event_id = event_id;  // 设置事件ID
  ctx->state = FFL_IMPACT_DISPLACEMENT_STATE_COLLECTING;  // 设置状态为数据收集
  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

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
                                                 float bz_mg) {
  if (ctx == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文为空，返回参数错误
  }
  if (ctx->state == FFL_IMPACT_DISPLACEMENT_STATE_UNINIT) {  // 检查是否已初始化
    return FFL_IMPACT_DISPLACEMENT_ERR_STATE;
  }

  ctx->baseline_ax_mg = bx_mg;  // 设置X轴基准
  ctx->baseline_ay_mg = by_mg;  // 设置Y轴基准
  ctx->baseline_az_mg = bz_mg;  // 设置Z轴基准
  ctx->baseline_set = 1u;       // 标记基准已设置

  if (ctx->cfg.gravity_ema_tau_ms != 0u) {  // 如果启用了重力EMA
    ctx->g_hat_ax_mg = bx_mg;   // 设置X轴重力估计
    ctx->g_hat_ay_mg = by_mg;   // 设置Y轴重力估计
    ctx->g_hat_az_mg = bz_mg;   // 设置Z轴重力估计
    ctx->g_hat_set = 1u;        // 标记重力估计已设置
  }
  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

/**
 * @brief 馈送一个采样数据
 *
 * @param ctx 上下文结构体指针
 * @param sample 采样数据结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_feed_sample(ffl_impact_displacement_ctx_t *ctx,
                                             const ffl_impact_displacement_sample_t *sample) {
  float ax_dyn_mg;        // X轴动态加速度(mg)
  float ay_dyn_mg;        // Y轴动态加速度(mg)
  float az_dyn_mg;        // Z轴动态加速度(mg)
  float a_mag_dyn_mg;     // 动态加速度幅值(mg)
  float a_curr_x;         // 当前X轴加速度(m/s²)
  float a_curr_y;         // 当前Y轴加速度(m/s²)
  float a_curr_z;         // 当前Z轴加速度(m/s²)
  float ref_ax_mg;        // X轴参考值(mg)
  float ref_ay_mg;        // Y轴参考值(mg)
  float ref_az_mg;        // Z轴参考值(mg)
  uint16_t grav_tau_ms;   // 重力EMA时间常数(ms)

  if (ctx == NULL || sample == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文或采样为空，返回参数错误
  }
  if (ctx->state != FFL_IMPACT_DISPLACEMENT_STATE_COLLECTING) {  // 检查是否处于数据收集状态
    return FFL_IMPACT_DISPLACEMENT_ERR_STATE;
  }

  grav_tau_ms = ctx->cfg.gravity_ema_tau_ms;  // 获取重力EMA时间常数

  if (grav_tau_ms != 0u) {  // 如果启用了重力EMA
    if (ctx->g_hat_set == 0u) {  // 如果重力估计尚未设置
      /* 如果调用者没有设置基准，从第一个采样初始化 */
      if (ctx->baseline_set != 0u) {  // 如果有基准值
        ctx->g_hat_ax_mg = ctx->baseline_ax_mg;  // 使用基准值
        ctx->g_hat_ay_mg = ctx->baseline_ay_mg;
        ctx->g_hat_az_mg = ctx->baseline_az_mg;
      } else {  // 否则使用第一个采样的值
        ctx->g_hat_ax_mg = (float)sample->ax_mg;
        ctx->g_hat_ay_mg = (float)sample->ay_mg;
        ctx->g_hat_az_mg = (float)sample->az_mg;
      }
      ctx->g_hat_set = 1u;  // 标记重力估计已设置
    }

    /* 使用前一个估计作为参考(高通行为) */
    ref_ax_mg = ctx->g_hat_ax_mg;
    ref_ay_mg = ctx->g_hat_ay_mg;
    ref_az_mg = ctx->g_hat_az_mg;
  } else {  // 如果没有启用重力EMA，使用基准值作为参考
    ref_ax_mg = ctx->baseline_ax_mg;
    ref_ay_mg = ctx->baseline_ay_mg;
    ref_az_mg = ctx->baseline_az_mg;
  }

  // 计算动态加速度(减去参考/基准值)
  ax_dyn_mg = (float)sample->ax_mg - ref_ax_mg;
  ay_dyn_mg = (float)sample->ay_mg - ref_ay_mg;
  az_dyn_mg = (float)sample->az_mg - ref_az_mg;
  a_mag_dyn_mg = ffl_impact_displacement_norm3(ax_dyn_mg, ay_dyn_mg, az_dyn_mg);  // 计算幅值

  if (a_mag_dyn_mg > ctx->peak_acc_mg) {  // 更新峰值加速度
    ctx->peak_acc_mg = a_mag_dyn_mg;
  }

  if ((sample->flags & FFL_IMPACT_DISPLACEMENT_SAMPLE_CLIPPED) != 0u) {  // 检查采样是否被截断
    ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_CLIPPED;  // 设置质量标志
  }

  /* Count a release sample when either the generic dynamic threshold is met
   * or the caller has already confirmed a physically quiet sample.
   *
   * The second path is needed for an installation that settles in a new static
   * pose after an impact.  The generic algorithm may still see gravity leakage
   * against its original/EMA baseline, while the caller has verified both
   * near-1g magnitude and a small inter-sample delta. Without this explicit
   * handoff, an application can close its measurement window while the result
   * misleadingly carries QF_NO_RELEASE.
   *
   * This flag is local to the sampling API.  It does not weaken clipping,
   * timestamp, duration, rotation or other measurement-quality checks, and it
   * is not encoded into any transport frame. */
  if (((sample->flags & FFL_IMPACT_DISPLACEMENT_SAMPLE_RELEASE_QUALIFIED) != 0u) ||
      (a_mag_dyn_mg <= (float)ctx->cfg.release_threshold_mg)) {
    if (ctx->release_count < 0xFFFFu) {
      ctx->release_count++;  // 增加释放计数
    }
    // 累加尾部数据用于后续旋转误差计算
    ctx->tail_sum_ax_mg += (float)sample->ax_mg;
    ctx->tail_sum_ay_mg += (float)sample->ay_mg;
    ctx->tail_sum_az_mg += (float)sample->az_mg;
    ctx->tail_count++;
  } else {
    ctx->release_count = 0u;  // 重置释放计数
  }

  // 转换为m/s²单位
  a_curr_x = ax_dyn_mg * MG_TO_MPS2;
  a_curr_y = ay_dyn_mg * MG_TO_MPS2;
  a_curr_z = az_dyn_mg * MG_TO_MPS2;

  if (ctx->has_prev == 0u) {  // 如果这是第一个采样
    ctx->first_ts_us = sample->timestamp_us;  // 记录第一个时间戳
    ctx->last_ts_us = sample->timestamp_us;   // 记录最后一个时间戳
    ctx->prev_ax_mps2 = a_curr_x;  // 保存当前加速度作为前一个值
    ctx->prev_ay_mps2 = a_curr_y;
    ctx->prev_az_mps2 = a_curr_z;
    ctx->has_prev = 1u;           // 标记已经有前一个值
    ctx->sample_count = 1u;       // 采样计数为1
    return FFL_IMPACT_DISPLACEMENT_OK;        // 返回成功
  }

  {
    uint32_t delta_us;            // 时间差(微秒)
    float dt_s;                   // 时间间隔(秒)
    float vx_new;                 // 新的X轴速度(m/s)
    float vy_new;                 // 新的Y轴速度(m/s)
    float vz_new;                 // 新的Z轴速度(m/s)

    if (sample->timestamp_us <= ctx->last_ts_us) {  // 检查时间戳是否单调递增
      ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_TS_NON_MONOTONIC;  // 设置质量标志
      delta_us = ffl_impact_displacement_nominal_dt_us(&ctx->cfg);  // 使用名义时间间隔
      if (delta_us == 0u) {
        delta_us = 1u;  // 避免除零错误
      }
    } else {
      delta_us = sample->timestamp_us - ctx->last_ts_us;  // 计算实际时间差
    }

    if (delta_us > ((uint32_t)ctx->cfg.max_dt_ms * 1000u)) {  // 检查时间差是否过大
      ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_DT_GAP;  // 设置质量标志
    }

    dt_s = (float)delta_us / 1000000.0f;  // 转换为秒

    if (grav_tau_ms != 0u && ctx->g_hat_set != 0u) {  // 如果启用了重力EMA
      float tau_s = (float)grav_tau_ms / 1000.0f;     // 时间常数转为秒
      float alpha = (tau_s > 0.0f) ? (dt_s / (tau_s + dt_s)) : 1.0f;  // 计算EMA系数
      if (alpha < 0.0f) {
        alpha = 0.0f;  // 限制alpha范围
      } else if (alpha > 1.0f) {
        alpha = 1.0f;
      }

      // 更新重力估计值
      ctx->g_hat_ax_mg += alpha * ((float)sample->ax_mg - ctx->g_hat_ax_mg);
      ctx->g_hat_ay_mg += alpha * ((float)sample->ay_mg - ctx->g_hat_ay_mg);
      ctx->g_hat_az_mg += alpha * ((float)sample->az_mg - ctx->g_hat_az_mg);
    }

    // 使用梯形积分计算速度和位移
    vx_new = ctx->vx_mps + 0.5f * (ctx->prev_ax_mps2 + a_curr_x) * dt_s;
    vy_new = ctx->vy_mps + 0.5f * (ctx->prev_ay_mps2 + a_curr_y) * dt_s;
    vz_new = ctx->vz_mps + 0.5f * (ctx->prev_az_mps2 + a_curr_z) * dt_s;

    ctx->x_m += 0.5f * (ctx->vx_mps + vx_new) * dt_s;
    ctx->y_m += 0.5f * (ctx->vy_mps + vy_new) * dt_s;
    ctx->z_m += 0.5f * (ctx->vz_mps + vz_new) * dt_s;

    ctx->vx_mps = vx_new;  // 更新速度
    ctx->vy_mps = vy_new;
    ctx->vz_mps = vz_new;
  }

  ctx->prev_ax_mps2 = a_curr_x;  // 更新前一个加速度值
  ctx->prev_ay_mps2 = a_curr_y;
  ctx->prev_az_mps2 = a_curr_z;
  ctx->last_ts_us = sample->timestamp_us;  // 更新最后时间戳
  ctx->sample_count++;                    // 增加采样计数

  if ((ctx->last_ts_us - ctx->first_ts_us) >  // 检查是否超过最大事件持续时间
      ((uint32_t)ctx->cfg.max_event_ms * 1000u)) {
    ctx->state = FFL_IMPACT_DISPLACEMENT_STATE_FINISHED;  // 设置状态为完成
  }

  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

/**
 * @brief 结束事件并获取结果
 *
 * @param ctx 上下文结构体指针
 * @param out 输出结果结构体指针
 * @return 操作状态
 */
ffl_impact_displacement_status_t ffl_impact_displacement_end_event(ffl_impact_displacement_ctx_t *ctx,
                                           ffl_impact_displacement_result_t *out) {
  float duration_s;          // 事件持续时间(秒)
  float rot_err_mg = 0.0f;   // 旋转误差(mg)
  float dx = 0.0f;           // X轴位移(m)
  float dy = 0.0f;           // Y轴位移(m)
  float dz = 0.0f;           // Z轴位移(m)
  float vterm_x = 0.0f;      // X轴终端速度(m/s)
  float vterm_y = 0.0f;      // Y轴终端速度(m/s)
  float vterm_z = 0.0f;      // Z轴终端速度(m/s)

  if (ctx == NULL || out == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果上下文或输出为空，返回参数错误
  }
  if (ctx->state != FFL_IMPACT_DISPLACEMENT_STATE_COLLECTING &&  // 检查状态是否正确
      ctx->state != FFL_IMPACT_DISPLACEMENT_STATE_FINISHED) {
    return FFL_IMPACT_DISPLACEMENT_ERR_STATE;
  }
  if (ctx->sample_count < 2u) {  // 检查采样数量是否足够
    ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_LOW_SAMPLE_COUNT;  // 设置质量标志
    return FFL_IMPACT_DISPLACEMENT_ERR_NOT_READY;
  }

  memset(out, 0, sizeof(*out));  // 清零输出结构体

  duration_s = (float)(ctx->last_ts_us - ctx->first_ts_us) / 1000000.0f;  // 计算持续时间(秒)
  if (duration_s <= 0.0f) {  // 检查持续时间是否有效
    return FFL_IMPACT_DISPLACEMENT_ERR_NOT_READY;
  }

  if ((ctx->last_ts_us - ctx->first_ts_us) <  // 检查事件是否持续时间过短
      ((uint32_t)ctx->cfg.min_event_ms * 1000u)) {
    ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_DURATION_TOO_SHORT;  // 设置质量标志
  }

  if (ctx->release_count < ctx->cfg.release_count_min) {  // 检查是否达到最小释放次数
    ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_NO_RELEASE;      // 设置质量标志
  }

  if (ctx->baseline_set != 0u && ctx->tail_count > 0u) {  // 如果设置了基准且有尾部数据
    // 计算结束时的平均值
    float g_end_x = ctx->tail_sum_ax_mg / (float)ctx->tail_count;
    float g_end_y = ctx->tail_sum_ay_mg / (float)ctx->tail_count;
    float g_end_z = ctx->tail_sum_az_mg / (float)ctx->tail_count;

    // 计算相对于基准的变化
    float dgx = g_end_x - ctx->baseline_ax_mg;
    float dgy = g_end_y - ctx->baseline_ay_mg;
    float dgz = g_end_z - ctx->baseline_az_mg;

    rot_err_mg = ffl_impact_displacement_norm3(dgx, dgy, dgz);  // 计算旋转误差

    if (rot_err_mg > ffl_impact_displacement_rotation_limit_mg(ctx->cfg.rotation_limit_deg)) {
      ctx->quality_flags |= FFL_IMPACT_DISPLACEMENT_QF_ROTATION_HIGH;  // 如果旋转误差过大，设置质量标志
    }
  }

  if (ctx->cfg.enable_zero_velocity_correction != 0u) {  // 如果启用零速度修正
    // 计算速度漂移修正量
    float b_res_x = ctx->vx_mps / duration_s;
    float b_res_y = ctx->vy_mps / duration_s;
    float b_res_z = ctx->vz_mps / duration_s;
    float t2 = duration_s * duration_s;  // 时间平方

    // 应用零速度修正
    dx = ctx->x_m - 0.5f * b_res_x * t2;
    dy = ctx->y_m - 0.5f * b_res_y * t2;
    dz = ctx->z_m - 0.5f * b_res_z * t2;

    // 计算修正后的终端速度
    vterm_x = ctx->vx_mps - b_res_x * duration_s;
    vterm_y = ctx->vy_mps - b_res_y * duration_s;
    vterm_z = ctx->vz_mps - b_res_z * duration_s;
  } else {  // 如果未启用零速度修正
    dx = ctx->x_m;        // 直接使用计算出的位移
    dy = ctx->y_m;
    dz = ctx->z_m;
    vterm_x = ctx->vx_mps;  // 直接使用计算出的速度
    vterm_y = ctx->vy_mps;
    vterm_z = ctx->vz_mps;
  }

  // 填充输出结果结构体
  out->event_id = ctx->event_id;
  out->duration_ms = (uint32_t)(duration_s * 1000.0f);  // 持续时间(毫秒)
  out->sample_count = ctx->sample_count;                // 采样数量
  out->dx_mm = dx * M_TO_MM;                            // X轴位移(毫米)
  out->dy_mm = dy * M_TO_MM;                            // Y轴位移(毫米)
  out->dz_mm = dz * M_TO_MM;                            // Z轴位移(毫米)
  out->disp_mm = ffl_impact_displacement_norm3(out->dx_mm, out->dy_mm, out->dz_mm);  // 总位移(毫米)
  out->peak_acc_mg = ctx->peak_acc_mg;                  // 峰值加速度
  out->terminal_speed_mm_s =                            // 终端速度(mm/s)
      ffl_impact_displacement_norm3(vterm_x, vterm_y, vterm_z) * MPS_TO_MMPS;
  out->rotation_error_mg = rot_err_mg;                  // 旋转误差
  out->quality_flags = ctx->quality_flags;              // 质量标志
  out->confidence = ffl_impact_displacement_clamp_u8(ffl_impact_displacement_confidence_from_flags(  // 置信度
      out->quality_flags, out->sample_count, ctx->release_count,
      ctx->cfg.release_count_min));

  ctx->state = FFL_IMPACT_DISPLACEMENT_STATE_FINISHED;  // 设置状态为完成
  return FFL_IMPACT_DISPLACEMENT_OK;  // 返回成功
}

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
                                               ffl_impact_displacement_result_t *out) {
  uint32_t i;              // 循环计数器
  float sum_ax = 0.0f;     // X轴加速度总和
  float sum_ay = 0.0f;     // Y轴加速度总和
  float sum_az = 0.0f;     // Z轴加速度总和
  ffl_impact_displacement_status_t st; // 状态返回值

  if (ctx == NULL || samples == NULL || out == NULL) {
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;  // 如果参数为空，返回参数错误
  }
  if (sample_count < 2u) {  // 检查采样数量是否足够
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;
  }
  if (pre_samples == 0u || pre_samples > sample_count) {  // 检查预采样数量是否有效
    return FFL_IMPACT_DISPLACEMENT_ERR_ARG;
  }

  // 计算预采样部分的平均值作为基准
  for (i = 0u; i < pre_samples; ++i) {
    sum_ax += (float)samples[i].ax_mg;
    sum_ay += (float)samples[i].ay_mg;
    sum_az += (float)samples[i].az_mg;
  }

  // 开始新事件
  st = ffl_impact_displacement_begin_event(ctx, event_id);
  if (st != FFL_IMPACT_DISPLACEMENT_OK) {
    return st;
  }

  // 设置基准值
  st = ffl_impact_displacement_set_baseline_mg(ctx, sum_ax / (float)pre_samples,
                                   sum_ay / (float)pre_samples,
                                   sum_az / (float)pre_samples);
  if (st != FFL_IMPACT_DISPLACEMENT_OK) {
    return st;
  }

  // 馈送所有采样数据
  for (i = 0u; i < sample_count; ++i) {
    st = ffl_impact_displacement_feed_sample(ctx, &samples[i]);
    if (st != FFL_IMPACT_DISPLACEMENT_OK) {
      return st;
    }
  }

  // 结束事件并返回结果
  return ffl_impact_displacement_end_event(ctx, out);
}

/**
 * @brief 获取当前状态
 *
 * @param ctx 上下文结构体指针
 * @return 当前状态
 */
ffl_impact_displacement_state_t ffl_impact_displacement_get_state(const ffl_impact_displacement_ctx_t *ctx) {
  if (ctx == NULL) {
    return FFL_IMPACT_DISPLACEMENT_STATE_ERROR;  // 如果上下文为空，返回错误状态
  }
  return ctx->state;  // 返回当前状态
}
