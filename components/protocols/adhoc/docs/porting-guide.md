# Ad-Hoc-lib 移植指南

本文面向把 `Ad-Hoc-lib` 移植到新 MCU/新射频驱动的工程。

## 1. 移植目标

- 协议代码保持不改或最小改动。
- 所有板级/射频差异收敛到 `adhoc_link_ops_t` 与可选平台辅助层。
- 保持“协议层与链路层解耦”，避免把频点/信道逻辑写入协议库。

## 2. 必要输入

- 可用的单调时间源（微秒）
- 随机数源（16 位）
- 单帧发送接口
- 单帧接收轮询接口（可返回 RSSI）

## 3. 链路接口契约（必须实现）

接口：`lib/Ad-Hoc-lib/include/adhoc_link.h`

### 3.1 `init(ctx)`

- 初始化一次链路资源。
- 要求可重复调用（重复调用不应破坏状态）。

### 3.2 `start_rx(ctx)`

- 允许在每个协议轮询周期调用。
- 若当前发射繁忙，建议返回 `ADHOC_LINK_OK` 并跳过启动，避免硬错误。

### 3.3 `tx(ctx, buf, len)`

- 成功返回 `ADHOC_LINK_OK`。
- 链路忙返回 `ADHOC_LINK_EBUSY`。
- 其他错误返回 `ADHOC_LINK_EIO`。

### 3.4 `poll_rx(ctx, buf, len, rssi)`

- 无帧返回 `ADHOC_LINK_RX_EMPTY`。
- 有帧时必须返回完整协议帧长度（当前为 `32` 字节）。
- 如果硬件提供 RSSI，应填充；否则可写 `0`。

### 3.5 `now_us(ctx)`

- 返回单调递增微秒时间。
- 可使用 `uint32_t` 回绕计时（协议已按无符号差值处理）。

### 3.6 `rand_u16(ctx)`

- 返回 `uint16_t` 随机数。
- 主要用于时隙扰动/调度随机化，质量不要求密码学等级。

## 4. 推荐移植步骤

1. 实现一层平台适配（例如 `port/<platform>/`）：
   - `now_us`
   - `rand_u16`
   - 可选临界区包装
2. 实现 `adhoc_link_ops_t`：
   - 对接现有 RF 驱动收发 API
3. 在应用任务中集成 `adhoc_node` 生命周期：
   - `init -> set_role -> poll/on_rx/fetch_tx`
4. 开启串口日志，先做单板网关验证，再做双板验证。

## 5. 应用层最小接入骨架

```c
// 1) init
adhoc_node_init(node_mem, node_mem_size, &cfg, &link_ops, link_ctx);
adhoc_node_set_role(node_mem, role);

// 2) loop
for (;;) {
    // poll rx -> on_rx
    // node poll
    // fetch tx -> link tx
    // fetch tx report
}
```

完整调用顺序示例见 `USAGE.md`。

## 6. 参数建议

- `network_window_us`：建议从 `30000000`（30s）起步
- `retry_max`：建议从 `30` 起步
- `t1/t2/t3/t4`：保证 `t5=t1+t2`、`t6=t3+t4` 且 `t3>t5`

## 7. 常见移植错误

1. `poll_rx` 返回半包：会导致 `adhoc_node_on_rx` 直接拒收。
2. `now_us` 不单调：状态机会异常回退或超时误判。
3. 在协议层引入信道切换：破坏解耦，后续难复用。
4. 未处理 `tx busy`：会造成数据重试异常飙升。
5. 未持续调用 `adhoc_node_poll`：A/D 帧发射节拍中断。

## 8. 联调最小验收清单

### 8.1 单板（网关）

- `gateway_network_locked` 在时间窗后置位
- 到期后 A 帧停止递增

### 8.2 双板（网关+信标）

- 信标可见 `ST1 -> U1/UN -> C1/CN`
- 信标 `submit/ack` 统计可持续增长
- 上级失效（`3*T5`）后可回退到 `ST1`

### 8.3 多板（多网关）

- 节点方向选择稳定（Top-K 生效）
- 错误方向确认不触发确认态跃迁
- 锁定与回退行为符合预期

## 9. CH32V208 现成参考

- 端口示例：`lib/Ad-Hoc-lib/port/ch32v208/`
- 链路适配示例：`app/adapters/adhoc_link_aros.c`
- Windows 仿真参考：`lib/Ad-Hoc-lib/test/`

建议直接按上述示例结构复制到新工程，再替换底层驱动调用。
