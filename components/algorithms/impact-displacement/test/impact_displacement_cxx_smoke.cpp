#include <cassert>

#include "ffl/impact_displacement.h"

int main() {
  ffl_impact_displacement_cfg_t cfg{};
  ffl_impact_displacement_ctx_t ctx{};
  ffl_impact_displacement_result_t result{};
  ffl_impact_displacement_sample_t first{};
  ffl_impact_displacement_sample_t second{};

  ffl_impact_displacement_get_default_cfg(&cfg);
  cfg.min_event_ms = 0u;
  cfg.release_count_min = 1u;
  cfg.enable_zero_velocity_correction = 0u;
  assert(ffl_impact_displacement_init(&ctx, &cfg) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_get_state(&ctx) ==
         FFL_IMPACT_DISPLACEMENT_STATE_IDLE);
  assert(ffl_impact_displacement_begin_event(&ctx, 123u) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_set_baseline_mg(&ctx, 0.0f, 0.0f, 1000.0f) ==
         FFL_IMPACT_DISPLACEMENT_OK);

  first.timestamp_us = 0u;
  first.az_mg = 1000;
  second.timestamp_us = 1000u;
  second.az_mg = 1000;
  assert(ffl_impact_displacement_feed_sample(&ctx, &first) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_feed_sample(&ctx, &second) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(ffl_impact_displacement_end_event(&ctx, &result) ==
         FFL_IMPACT_DISPLACEMENT_OK);
  assert(result.event_id == 123u);
  assert(result.sample_count == 2u);
  assert(result.quality_flags == FFL_IMPACT_DISPLACEMENT_QF_NONE);
  assert(ffl_impact_displacement_reset(&ctx) == FFL_IMPACT_DISPLACEMENT_OK);
  return 0;
}
