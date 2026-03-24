# SC7A20 New Driver

- Chinese docs: [README.zh-CN.md](./README.zh-CN.md)
- Chinese porting guide: [docs/PORTING_GUIDE.zh-CN.md](./docs/PORTING_GUIDE.zh-CN.md)
- Chinese API reference: [docs/API_REFERENCE.zh-CN.md](./docs/API_REFERENCE.zh-CN.md)
- Chinese register map: [docs/regs_map.zh-CN.md](./docs/regs_map.zh-CN.md)

This directory contains a portable SC7A20 driver rebuilt around a single transaction bus contract.

## Key points

- Core code uses only `sc7a20_reg.h` plus the `xfer` contract.
- No platform-specific code in `src/`.
- Sync and async APIs share the same transaction model.
- All register accesses are described by `sc7a20_comm_msg_t[]` atomically.

## Directory layout

- `sc7a20_reg.h`: user-supplied register map (copied from current project for convenience).
- `inc/sc7a20.h`: public API.
- `inc/sc7a20_core.h`: bus contract + core types.
- `src/sc7a20_core.c`: register transaction helpers + decode + config apply.
- `src/sc7a20_sync.c`: synchronous API.
- `src/sc7a20_async.c`: asynchronous API.
- `adapters/mock_adapter.*`: software adapter for host tests.
- `docs/*`: migration and API notes.
- `test/*`: minimal host-side tests.
