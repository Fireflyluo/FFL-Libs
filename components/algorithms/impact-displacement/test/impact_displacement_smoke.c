#include <assert.h>
#include <string.h>

#include "ffl/impact_displacement.h"

static ffl_impact_displacement_sample_t make_sample(uint32_t timestamp_us,
                                                     int16_t ax_mg,
                                                     uint8_t flags) {
  ffl_impact_displacement_sample_t value;

  memset(&value, 0, sizeof(value));
  value.timestamp_us = timestamp_us;
  value.ax_mg = ax_mg;
  value.az_mg = 1000;
  value.flags = flags;
  return value;
}

static void test_empty_arguments(void) {
  ffl_impact_displacement_ctx_t ctx;
  ffl_impact_displacement_result_t result;
  ffl_impact_displacement_sample_t sample;

  memset(&ctx, 0, sizeof(ctx));
  memset(&result, 0, sizeof(result));
  memset(&sample, 0, sizeof(sample));

  assert(ffl_impact_displacement_get_state(NULL) ==
         FFL_IMPACT_DISPLACEMENT_STATE_ERROR);
  assert(ffl_impact_displacement_init(NULL, NULL) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_reset(NULL) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_begin_event(NULL, 1u) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_set_baseline_mg(NULL, 0.0f, 0.0f, 0.0f) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_feed_sample(NULL, &sample) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_feed_sample(&ctx, NULL) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_end_event(NULL, &result) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_end_event(&ctx, NULL) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
  assert(ffl_impact_displacement_process_event(&ctx, NULL, 0u, 0u, 1u,
                                               &result) ==
         FFL_IMPACT_DISPLACEMENT_ERR_ARG);
}

static void test_state_flow(void) {
  ffl_impact_displacement_cfg_t cfg;
  ffl_impact_displacement_ctx_t ctx;
  ffl_impact_displacement_result_t result;
  ffl_impact_displacement_sample_t value;

  ffl_impact_displacement_get_default_cfg(&cfg);
  memset(&ctx, 0, sizeof(ctx));
  memset(&result, 0, sizeof(result));

  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_UNINIT);
  assert(ffl_impact_displacement_begin_event(&ctx, 1u) ==
         FFL_IMPACT_DISPLACEMENT_ERR_STATE);
  assert(ffl_impact_displacement_init(&ctx, &cfg) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_IDLE);

  value = make_sample(0u, 0, 0u);
  assert(ffl_impact_displacement_feed_sample(&ctx, &value) ==
         FFL_IMPACT_DISPLACEMENT_ERR_STATE);
  assert(ffl_impact_displacement_end_event(&ctx, &result) ==
         FFL_IMPACT_DISPLACEMENT_ERR_STATE);
  assert(ffl_impact_displacement_begin_event(&ctx, 9u) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_COLLECTING);
  assert(ffl_impact_displacement_set_baseline_mg(&ctx, 0.0f, 0.0f, 1000.0f) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_end_event(&ctx, &result) ==
         FFL_IMPACT_DISPLACEMENT_ERR_NOT_READY);
  assert((ctx.quality_flags & FFL_IMPACT_DISPLACEMENT_QF_LOW_SAMPLE_COUNT) !=
         0u);
  assert(ffl_impact_displacement_reset(&ctx) == FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_IDLE);
}

static void test_synthetic_short_event_quality(void) {
  ffl_impact_displacement_cfg_t cfg;
  ffl_impact_displacement_ctx_t ctx;
  ffl_impact_displacement_result_t result;
  ffl_impact_displacement_sample_t samples[4];

  ffl_impact_displacement_get_default_cfg(&cfg);
  cfg.release_count_min = 3u;
  cfg.enable_zero_velocity_correction = 0u;
  samples[0] = make_sample(0u, 0, 0u);
  samples[1] = make_sample(1000u, 1000, FFL_IMPACT_DISPLACEMENT_SAMPLE_CLIPPED);
  samples[2] = make_sample(1000u, 0, 0u);
  samples[3] = make_sample(30000u, 0, 0u);

  assert(ffl_impact_displacement_init(&ctx, &cfg) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_process_event(&ctx, samples, 4u, 1u, 77u,
                                               &result) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(result.event_id == 77u);
  assert(result.sample_count == 4u);
  assert(result.duration_ms == 30u);
  assert(result.peak_acc_mg > 999.0f);
  assert((result.quality_flags &
          FFL_IMPACT_DISPLACEMENT_QF_DURATION_TOO_SHORT) != 0u);
  assert((result.quality_flags & FFL_IMPACT_DISPLACEMENT_QF_CLIPPED) != 0u);
  assert((result.quality_flags &
          FFL_IMPACT_DISPLACEMENT_QF_TS_NON_MONOTONIC) != 0u);
  assert((result.quality_flags & FFL_IMPACT_DISPLACEMENT_QF_DT_GAP) != 0u);
  assert((result.quality_flags & FFL_IMPACT_DISPLACEMENT_QF_NO_RELEASE) != 0u);
  assert(result.confidence < 100u);
  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_FINISHED);
}

static void test_synthetic_complete_event(void) {
  ffl_impact_displacement_cfg_t cfg;
  ffl_impact_displacement_ctx_t ctx;
  ffl_impact_displacement_result_t result;
  ffl_impact_displacement_sample_t value;
  const uint32_t timestamps[] = {0u, 10000u, 20000u, 30000u, 40000u};
  const int16_t accelerations[] = {0, 1000, 0, 0, 0};
  uint32_t i;

  ffl_impact_displacement_get_default_cfg(&cfg);
  cfg.sample_rate_hz = 1000u;
  cfg.min_event_ms = 5u;
  cfg.max_event_ms = 100u;
  cfg.release_count_min = 2u;
  cfg.enable_zero_velocity_correction = 0u;

  assert(ffl_impact_displacement_init(&ctx, &cfg) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_begin_event(&ctx, 42u) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_set_baseline_mg(&ctx, 0.0f, 0.0f, 1000.0f) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  for (i = 0u; i < 5u; ++i) {
    value = make_sample(timestamps[i], accelerations[i], 0u);
    assert(ffl_impact_displacement_feed_sample(&ctx, &value) ==
           FFL_IMPACT_DISPLACEMENT_OK);
  }
  assert(ffl_impact_displacement_end_event(&ctx, &result) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(result.event_id == 42u);
  assert(result.duration_ms == 40u);
  assert(result.sample_count == 5u);
  assert(result.peak_acc_mg > 999.0f);
  assert(result.disp_mm > 0.0f);
  assert(result.quality_flags == FFL_IMPACT_DISPLACEMENT_QF_NONE);
  assert(result.confidence == 100u);
}

int main(void) {
  assert(ffl_impact_displacement_get_version() != NULL);
  test_empty_arguments();
  test_state_flow();
  test_synthetic_short_event_quality();
  test_synthetic_complete_event();
  return 0;
}
