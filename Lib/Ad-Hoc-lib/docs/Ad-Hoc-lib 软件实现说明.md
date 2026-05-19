# Ad-Hoc-lib 软件实现说明

本文描述库内代码结构、关键数据流和实现约束，便于二次开发与问题定位。

## 1. 分层结构

调用关系（逻辑）：

`应用任务 -> adhoc_node(API) -> adhoc_sm + adhoc_data_plane -> adhoc_link_ops_t(抽象) -> 平台链路适配`

职责边界：

- `adhoc_node`：对外唯一入口，调度状态机与数据面
- `adhoc_sm`：A 帧与组网状态机
- `adhoc_data_plane`：D 帧、去重、ACK、转发队列
- `adhoc_link_ops_t`：协议层与链路层边界

## 2. 主要模块

### 2.1 `adhoc_api.h` + `adhoc_node.c`

- 初始化：`adhoc_node_init`
- 角色切换：`adhoc_node_set_role`
- 输入路径：`adhoc_node_on_rx`
- 周期驱动：`adhoc_node_poll`
- 输出路径：`adhoc_node_fetch_tx`
- 数据提交：`adhoc_node_submit_data`
- 结果回传：`adhoc_node_fetch_data_tx_report`
- 运行态快照：`adhoc_node_get_runtime_status`

关键点：

- 内部通过 `adhoc_node_sync_route_context` 将状态机路由信息同步到数据面。
- 采用“单 pending_tx”模式，同一周期只保留一帧待发，避免跨层并发写冲突。

### 2.2 `adhoc_sm.*`（组网状态机）

- 负责 `ST1/U1/UN/C1/CN` 状态迁移
- 负责网关 `T2` 收集窗口和 `network_window_us` 停发
- 维护 `neighbor_cache(K=6)` 与 `upstream_bindings(0..5)`
- 维护下行确认队列 `downlink_confirm_list`

### 2.3 `adhoc_data_plane.*`（数据面）

- 负责编解码 `D` 帧和 ACK 载荷
- 维护去重表、待发队列、ACK 队列、TX 报告队列
- 实现源数据重试、监听确认、网关 ACK 汇聚

### 2.4 `adhoc_frame.*` + `adhoc_crc8.*` + `adhoc_timing.*`

- `adhoc_frame`：帧字段编解码 + 时间戳字段辅助函数
- `adhoc_crc8`：固定参数 CRC8
- `adhoc_timing`：`T5/T6/n/m` 计算与时隙奇偶检查

### 2.5 `adhoc_reply_list.*`

- 组网确认队列（环形，容量 24）
- 入队去重（按 `node_id`）
- A 帧确认槽位打包（支持偏移与最大项控制）

## 3. 关键运行流程

### 3.1 接收路径

1. 链路层给出完整 32B 帧与 RSSI。
2. `adhoc_node_on_rx` 校验：长度、CRC、域号、时隙奇偶。
3. `A` 帧进入 `adhoc_sm_on_rx`；`D` 帧进入 `adhoc_data_plane_on_rx`。
4. 若状态变化，路由信息会同步到数据面。

### 3.2 发送调度（`adhoc_node_poll`）

发送优先级：

1. 网关 ACK（仅网关角色）
2. 组网帧（A）
3. 信标数据转发（D）

说明：每次生成待发帧后由外部调用 `adhoc_node_fetch_tx` 拉取，再交给链路层 `tx`。

### 3.3 数据提交与结果

- 信标调用 `adhoc_node_submit_data` 后进入待发队列。
- 结果来自两类成功：
  - 收到网关 ACK 命中
  - 监听到上级转发命中（提前收敛）
- 超过重试上限返回 `RETRY_EXHAUSTED`。

## 4. 静态内存模型

库不进行动态分配，调用方需提供静态内存：

- 通过 `adhoc_node_required_size()` 获取 `node_mem` 最小容量
- 典型内部固定容量：
  - 去重表 `64`
  - ACK 队列 `32`
  - 数据待发队列 `16`
  - TX 报告队列 `16`
  - 组网确认队列 `24`
  - 邻居缓存 `6`
  - 上级绑定表 `6`

## 5. 错误码与行为约定

`adhoc_rc_t`：

- `ADHOC_OK`
- `ADHOC_EINVAL`：参数或输入帧非法
- `ADHOC_ESTATE`：状态非法/未初始化/角色不匹配
- `ADHOC_EBUSY`：当前无法接收新待发（如 pending 已占用）
- `ADHOC_ENOFRAME`：当前无待发/无报告可取

建议：应用侧区分“软失败”（`EBUSY/ENOFRAME`）与“硬失败”（持续 `ESTATE/EINVAL`）。

## 6. 观测与调试接口

- 协议运行态：`adhoc_node_get_runtime_status`
- 数据发送结果：`adhoc_node_fetch_data_tx_report`
- 链路适配可自行扩展状态结构（如本工程 `adhoc_link_aros_status_t`）

建议关注指标：

- 组网：`state/joined_level/upstream_id/upstream_no`
- 时间窗：`gateway_network_locked/network_lock_closed`
- 数据面：`ACKED` 与 `RETRY_EXHAUSTED` 比例

## 7. 设计约束总结

- 协议层不感知射频信道策略。
- 所有时序判断基于 `uint32_t` 微秒时钟，允许回绕（通过无符号差值比较）。
- 组网与数据面通过显式路由同步解耦，不在数据面硬编码上级信息。
