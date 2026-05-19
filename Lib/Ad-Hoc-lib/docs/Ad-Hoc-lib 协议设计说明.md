# Ad-Hoc-lib 协议设计说明

本文定义 `Ad-Hoc-lib` 当前实现的协议基线，重点是“可移植实现必须遵守的行为”。

## 1. 设计目标与边界

- 协议层只处理：帧格式、组网状态机、数据面转发/确认。
- 链路层只处理：单帧收发、时间源、随机源。
- 频点、信道切换、跳频、功率策略等射频细节不进入协议层。
- 采用静态内存，不使用动态分配。

## 2. 节点与角色

- 网关节点：`node_id <= 1000000`
- 信标节点：`node_id >= 1000001`
- 角色：
  - `ADHOC_ROLE_GATEWAY`
  - `ADHOC_ROLE_BEACON`

## 3. 帧模型（固定 32B）

### 3.1 总体结构

- `Byte0`：`Head`（`msg_class[1] + gateway_no[3] + slot_high4[4]`）
- `Byte1`：`Level`
- `Byte2..6`：`Sender`（`domain_id[11] + node_id[29]`）
- `Byte7..30`：`Content`（24B）
- `Byte31`：`CRC8`

`CRC8` 参数：`poly=0x07, init=0x00, xorout=0x00`。

### 3.2 消息类别

- `A` 帧：组网控制帧（`msg_class=0`）
- `D` 帧：业务数据帧（`msg_class=1`）

## 4. A 帧内容语义（组网）

### 4.1 Content 通用头

- `Content[0..3]`：`LMT_A`（32bit 秒级时间戳）

### 4.2 ID 槽位

`Content[4..23]` 被切成 `5` 个 `4B` ID 槽位，每项编码为 `payload_id = id_flag[3] + node_id[29]`。

`id_flag` 语义：

- `0..5`：下级“入网请求绑定号”（绑定到某个上级编号）
- `6`：已确认节点声明“我的已确认上级”
- `7`：上级对下级的入网确认

### 4.3 三类 A 帧布局

1) 网关 `A V=0` 广播：

- `LMT_A(4B) + confirm(flag=7,child_id) x 0..5`

2) 未确认信标 `A V=n` 入网请求：

- `LMT_A(4B) + ID(flag=upstream_no(0..5), upstream_id) + confirm(flag=7,child_id) x 0..4`

3) 已确认信标 `A V=n` 广播：

- `LMT_A(4B) + ID(flag=6, upstream_id) + confirm(flag=7,child_id) x 0..4`

说明：信标帧会预留一个槽位给上级声明，因此每帧最多携带 `4` 个下级确认；网关帧最多 `5` 个。

### 4.4 入网确认判定

- 未确认节点仅在收到“来自当前上级方向”的 `flag=7 + self_node_id` 时转入确认态。
- 收到 `flag=0..5` 或 `flag=6` 不触发 `U1/UN -> C1/CN`。
- 网关/已确认节点收到下级请求（`flag=0..5` 且 `node_id` 命中自身）后，入确认队列时统一转换为 `flag=7`。

## 5. D 帧内容语义（数据面）

### 5.1 数据帧（上行）

`Content` 布局：

- `0..2`：`LMT_D`（24bit 秒级时间戳）
- `3..6`：`source_id`（`id_flag + node_id`）
- `7..23`：`user[17]`

### 5.2 ACK 帧（下行）

网关 ACK 帧（`level=0`）在 `Content` 中顺序打包最多 `3` 条 ACK 项，每项 `7B`：

- `payload_id[4] + lmt_d[3]`

### 5.3 去重与确认键

- 去重键：`(source_id, lmt_d)`
- ACK 命中键：`(source_id, lmt_d)`
- 监听确认（beacon）：监听到上级转发了同键数据后，本地待发队列会提前移除，视作成功。

### 5.4 转发约束

- 首跳源数据：转发时 `source.id_flag` 改写为本节点 `upstream_no`。
- 非首跳转发：仅当收到的数据 `source.id_flag == local_upstream_no` 才允许继续转发。

## 6. 时序与状态机

### 6.1 基础时序

- `T5 = T1 + T2`
- `T6 = T3 + T4`
- `slot` 奇偶约束：`(slot_no XOR level) & 1 == 0`

### 6.2 网关侧行为

- 周期发送 `A V=0`。
- 每次发射后打开 `T2` 收集窗口，仅在窗口内接收入网请求。
- 组网时间窗 `network_window_us` 到期后：
  - `gateway_network_locked = 1`
  - 停止发射 `A V=0`
  - 关闭收集窗口

### 6.3 信标侧状态

- `ST1`：监听候选上级
- `U1/UN`：已选方向但未确认，周期发送入网请求
- `C1/CN`：已确认，周期发送本级 `A`，并可转发数据

回退条件：

- 未确认态重试达 `retry_max`
- 连续 `3*T5` 未观测到当前上级
- 本地组网窗口结束（`network_lock_end_us` 到期）
- 可选重组网计时到期（`regroup_interval_us`）

## 7. 多网关选择（Top-K）

- 邻居缓存容量：`K=6`
- 上级编号绑定容量：`0..5`（共 6 条）
- 候选优先级：`signal_rank` -> `RSSI` -> `upstream_level`（越小越优）-> `gateway_no` -> `upstream_id`
- `signal_rank` 规则：
  - `rssi >= -65`：强（单周期可选）
  - `-80 <= rssi < -65`：中（2 周期窗口）
  - `< -80`：弱（4 周期窗口，要求 2 次命中）

## 8. 已实现与保留项

已实现：

- `flag=7` 入网确认闭环
- 网关 `T2/T5` 收发节拍与 `30s` 时间窗停发
- 信标方向锁定、`3*T5` 上级失效回退
- Top-K 邻居缓存与上级编号绑定

保留（后续可演进）：

- 全网统一绝对结束时刻的跨节点严格对齐
- 多网关场景下更复杂的策略（如权重融合、策略热切换）
