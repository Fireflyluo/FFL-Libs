# Ad-Hoc-lib 配置说明

本文档说明 `Ad-Hoc-lib` 的统一配置入口与推荐使用方式，目标是让协议库在不同工程中以同一套方式配置。

## 1. 配置分层

`Ad-Hoc-lib` 的配置分为两层：

1. 运行时配置（实例级）  
   通过 `adhoc_cfg_t` 传入，适合每个节点动态不同的参数：
   - `domain_id/node_id/gateway_no`
   - `t1_us/t2_us/t3_us/t4_us`
   - `retry_max/network_window_us/regroup_interval_us`

2. 编译期配置（库级）  
   通过 `include/adhoc_config.h` 统一管理，适合队列容量、默认阈值、调试开关等库行为参数。

## 2. 统一配置入口

- 头文件：`include/adhoc_config.h`
- 覆盖方式：
  - 直接用编译宏覆盖（`-DADHOC_CONFIG_...=...`）
  - 指定用户配置头：`-DADHOC_CONFIG_USER_HEADER="xxx.h"`

示例（xmake）：

```lua
add_cflags(
    "-DADHOC_CONFIG_SM_DEFAULT_RETRY_MAX=20",
    "-DADHOC_CONFIG_DATA_TX_QUEUE_CAPACITY=24",
    "-DADHOC_CONFIG_USER_HEADER=\\\"adhoc_user_config.h\\\""
)
```

## 3. 可配置项总览

### 3.1 时序模块

- `ADHOC_CONFIG_TMOS_TICK_US`（默认 `625`）
- `ADHOC_CONFIG_TIMING_M_RECOMMENDED_MIN`（默认 `10`）
- `ADHOC_CONFIG_TIMING_M_RECOMMENDED_MAX`（默认 `1000`）

### 3.2 状态机（SM）

- `ADHOC_CONFIG_SM_GATEWAY_ID_MAX`（默认 `1000000`）
- `ADHOC_CONFIG_SM_BEACON_ID_MIN`（默认 `1000001`）
- `ADHOC_CONFIG_SM_DEFAULT_RETRY_MAX`（默认 `30`）
- `ADHOC_CONFIG_SM_DEFAULT_NETWORK_WINDOW_US`（默认 `30000000`）
- `ADHOC_CONFIG_SM_RSSI_STRONG_MIN_DBM`（默认 `-65`）
- `ADHOC_CONFIG_SM_RSSI_MEDIUM_MIN_DBM`（默认 `-80`）
- `ADHOC_CONFIG_SM_JOIN_LEVEL_MAX`（默认 `254`）
- `ADHOC_CONFIG_SM_UPSTREAM_BINDING_CAPACITY`（默认 `6`）
- `ADHOC_CONFIG_SM_NEIGHBOR_CACHE_CAPACITY`（默认 `6`）
- `ADHOC_CONFIG_SM_UPSTREAM_LOSS_CYCLES`（默认 `3`）
- `ADHOC_CONFIG_TEST_DROP_JOIN_CONFIRM`（默认 `0`，测试开关）

### 3.3 组网确认队列

- `ADHOC_CONFIG_REPLY_LIST_CAPACITY`（默认 `24`）
- `ADHOC_CONFIG_REPLY_MAX_PER_FRAME`（默认 `5`）

### 3.4 数据面

- `ADHOC_CONFIG_DATA_DEDUP_CAPACITY`（默认 `64`）
- `ADHOC_CONFIG_DATA_ACK_QUEUE_CAPACITY`（默认 `32`）
- `ADHOC_CONFIG_DATA_TX_QUEUE_CAPACITY`（默认 `16`）
- `ADHOC_CONFIG_DATA_TX_REPORT_QUEUE_CAPACITY`（默认 `16`）
- `ADHOC_CONFIG_DATA_TX_RETRY_MAX`（默认 `30`）
- `ADHOC_CONFIG_DATA_DEFAULT_DEDUP_WINDOW_MS`（默认 `6h`）
- `ADHOC_CONFIG_DATA_GATEWAY_ID_MAX`（默认 `1000000`）

### 3.5 CRC8

- `ADHOC_CONFIG_CRC8_TABLE_STORAGE`
  - 默认：`RISC-V + GCC` 放入 `.flash1_rodata`
  - 其他平台：默认无段属性

## 4. 约束与建议

1. `adhoc_config.h` 内含基础边界检查（如容量必须 > 0）。
2. 与帧格式直接相关的参数（如 `ADHOC_FRAME_SIZE`、`ADHOC_DATA_MSG_USER_LEN`）不建议改为可配置项，避免协议兼容性漂移。
3. 工程内的业务角色选择（网关/信标）、节点 ID、时序值优先走运行时 `adhoc_cfg_t`，不要再用构建宏硬编码。

## 5. 最小迁移建议

从旧工程迁移时，建议顺序：

1. 保持 `adhoc_cfg_t` 运行时配置路径不变；
2. 将原来散落在构建脚本里的协议库宏迁移为 `ADHOC_CONFIG_*`；
3. 在单板构建通过后，再做多板联调验证。

