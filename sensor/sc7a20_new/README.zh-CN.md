# SC7A20 新驱动

> 中文文档。English version: [README.md](./README.md)

本目录提供基于统一事务总线契约重构的 SC7A20 可移植驱动。

## 设计目标

- 只依赖 `sc7a20_reg.h` 和总线 `xfer` 契约
- 驱动核心 (`src/`) 不含任何平台相关代码
- 同步/异步 API 共用同一套事务模型
- 所有寄存器访问都通过 `sc7a20_comm_msg_t[]` 原子描述

## 目录说明

- `sc7a20_reg.h`: 用户提供的寄存器定义（当前仓库中放了一份拷贝）
- `inc/sc7a20.h`: 对外公共 API
- `inc/sc7a20_core.h`: 内核类型与总线契约
- `src/sc7a20_core.c`: 寄存器事务辅助、数据解析、配置下发
- `src/sc7a20_sync.c`: 同步 API
- `src/sc7a20_async.c`: 异步 API
- `adapters/`: 适配层示例（如 CH32、mock）
- `docs/`: 移植说明、API 参考、寄存器映射
- `test/`: 主机侧最小测试

## 快速接入

1. 实现 `sc7a20_bus_ops_t`（`xfer` 必选，`cancel` 可选）
2. 准备 `sc7a20_dev_t`：填入 `ops`、`bus_ctx`、`addr`
3. 调用 `sc7a20_init()` 或 `sc7a20_init_with_config()`
4. 使用 `sc7a20_read_xyz_raw()` / `sc7a20_read_xyz_g()` 获取数据

## 相关文档

- [移植指南（中文）](./docs/PORTING_GUIDE.zh-CN.md)
- [API 参考（中文）](./docs/API_REFERENCE.zh-CN.md)
- [寄存器映射（中文）](./docs/regs_map.zh-CN.md)
