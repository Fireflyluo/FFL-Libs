# Ad-Hoc-lib 使用说明

本文提供可直接迁移到其他工程的最小接入流程。协议语义请同时参考：

- `protocol-design.md`
- `software-architecture.md`
- `porting-guide.md`

## 1. 接入前提

- 已实现 `adhoc_link_ops_t`（见 `porting-guide.md`）
- 已准备静态内存用于 `adhoc_node`
- 应用任务具备周期调度能力（建议固定轮询节拍）

## 2. 初始化

### 2.1 组装配置 `adhoc_cfg_t`

- `domain_id`：组网域（0..2047）
- `node_id`：节点 ID（29bit，且非 0）
- `gateway_no`：网关编号（0..7）
- `t1_us ~ t4_us`：基础时序参数
- `retry_max`：重试上限（建议 `30` 起步）
- `network_window_us`：组网时间窗（建议 `30000000`）
- `regroup_interval_us`：重组网周期（`0` 表示关闭）

### 2.2 初始化步骤

```c
uint8_t node_mem[1024]; // 示例容量，实际以 adhoc_node_required_size() 为准
uint32_t need = adhoc_node_required_size();

if (sizeof(node_mem) < need) {
    // 提前报错，避免初始化失败
}
adhoc_node_init(node_mem, sizeof(node_mem), &cfg, &link_ops, link_ctx);
adhoc_node_set_role(node_mem, ADHOC_ROLE_GATEWAY /* or ADHOC_ROLE_BEACON */);
```

建议：启动时校验 `sizeof(node_mem) >= need`，避免后续硬故障。

## 3. 主循环推荐顺序

下面顺序是可工作的最小闭环：

1. `adhoc_node_poll(node, now_us)`：推进状态机与调度发送
2. `adhoc_node_fetch_tx(node, &tx)`：拉取待发帧
3. `link_ops.tx(...)`：下发到射频链路
4. `link_ops.poll_rx(...)`：轮询接收完整帧
5. `adhoc_node_on_rx(node, &rx)`：喂给协议栈
6. `adhoc_node_fetch_data_tx_report(...)`：拉取上报结果

注意：

- `adhoc_node_poll` 不会主动读取链路接收缓冲，`on_rx` 必须由应用层显式调用。
- 建议应用层维护 TX/RX 预算，避免单次循环阻塞。

## 4. 信标数据上报

仅信标角色可提交数据：

```c
adhoc_node_submit_data(node, source_id_flag, lmt_d, user17, now_us);
```

- `source_id_flag`：0..7（通常业务先从 `0` 起步）
- `lmt_d=0`：库内自动按 `now_us` 生成
- `user` 长度固定 `17B`

## 5. 发送结果回传

通过 `adhoc_node_fetch_data_tx_report` 获取结果：

- `ADHOC_NODE_DATA_TX_REPORT_ACKED`
- `ADHOC_NODE_DATA_TX_REPORT_RETRY_EXHAUSTED`
- `ADHOC_NODE_DATA_TX_REPORT_NONE`

建议把 `source_node_id + lmt_d + retry_count` 记录到业务日志，便于板测复盘。

## 6. 运行态观测

通过 `adhoc_node_get_runtime_status` 可读取：

- 组网状态：`state/joined_level/retry_count`
- 上级信息：`upstream_id/upstream_no/upstream_gateway_no/upstream_last_seen_us`
- 网关窗口：`gateway_network_started/gateway_network_locked/gateway_network_end_us`
- 信标锁定：`network_lock_active/network_lock_closed/network_lock_end_us`

## 7. 常见问题

1) `adhoc_node_on_rx` 经常返回非法：

- 检查是否严格为 `32B` 帧
- 检查 CRC 与域号是否一致
- 检查时隙奇偶是否匹配

2) `submit_data` 失败：

- 当前是否为信标角色
- 是否已完成入网（`joined_level != 0`）
- TX 队列是否已满（会返回忙）

3) 网关一直发包不停止：

- 检查 `network_window_us` 是否为 `0` 或配置异常
- 通过 `gateway_network_locked` 判断是否已到窗尾

## 8. 在本仓库中的参考实现

- 协议任务示例：`app/tasks/ad_hoc_task.c`
- 链路适配示例：`app/adapters/adhoc_link_aros.c`
- CH32 端口示例：`lib/Ad-Hoc-lib/port/ch32v208/`
