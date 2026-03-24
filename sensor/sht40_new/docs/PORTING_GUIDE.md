# PORTING GUIDE

1. Implement `xfer(ctx,msgs,cnt,cb,user)` and optional `cancel`.
2. Provide `delay_ms` callback in `sht40_dev_t`.
3. Fill device struct and call `sht40_init()`.
4. Use `sht40_read_sample()` for sync reads.

CH32 adapter is provided in `adapters/sht40_ch32_adapter.*`.
