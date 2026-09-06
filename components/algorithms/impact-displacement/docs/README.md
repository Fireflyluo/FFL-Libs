# `ffl.impact_displacement`

This component contains the standalone impact-displacement algorithm core
migrated from the source project's `impact_displacement.h` and
`impact_displacement.c` only.

The source project's `impact_episode_v02*` files are deliberately not part of
this component. Application-level episode lifecycle, event policy, RF framing,
and protocol fields remain in the source project and must be integrated by the
application layer.

## API and layout

- Public header: `include/ffl/impact_displacement.h`
- Core source: `src/ffl_impact_displacement.c`
- Primary Xmake target: `ffl.impact_displacement` (`object`, default off)
- Host checks: `ffl.impact_displacement.test` and
  `ffl.impact_displacement.cxx-test`

All public names use the `FFL_IMPACT_DISPLACEMENT_*` and
`ffl_impact_displacement_*` prefixes. No legacy-name compatibility layer is
provided. The public header is C11/C++17 compatible and exposes C linkage to
C++ consumers.

## Usage

The caller owns one `ffl_impact_displacement_ctx_t` and supplies acceleration
samples in mg with microsecond timestamps. A streaming flow is:

1. call `ffl_impact_displacement_init()`;
2. call `ffl_impact_displacement_begin_event()`;
3. set the static reference with
   `ffl_impact_displacement_set_baseline_mg()`;
4. feed samples with `ffl_impact_displacement_feed_sample()`;
5. call `ffl_impact_displacement_end_event()` for the result.

`ffl_impact_displacement_process_event()` is the convenience path for a
complete in-memory sample sequence and a caller-selected pre-sample baseline.

## MCU constraints

The core has no HAL, RTOS, bus, IRQ, DMA, or board dependency. It stores the
algorithm state in caller-owned floating-point fields and performs floating-
point integration plus small software `sin`/`sqrt` approximations. On an MCU
without an FPU, measure the Flash, RAM, stack, and execution-time cost before
enabling it in a production sampling loop. Resource-constrained builds can
keep the object target and link only this leaf component; there is no hidden
family-wide package.

Host GCC tests validate API behavior with synthetic samples only. They do not
replace physical sensor characterization, timing validation, or hardware
verification on the target MCU.

## Host validation

Build each test target before running its registered test so that a missing
target cannot look like a successful `nothing to test` result:

```text
xmake f -P . -c
xmake build -P . -r ffl.impact_displacement.test
xmake test -P . ffl.impact_displacement.test/default
xmake build -P . -r ffl.impact_displacement.cxx-test
xmake test -P . ffl.impact_displacement.cxx-test/default
```
