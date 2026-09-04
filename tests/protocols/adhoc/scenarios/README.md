# scenarios — 测试场景配置

本目录存放仿真测试的场景拓扑定义文件。

## 配置文件格式 (`.conf`)

`basic_topology.conf` 使用自定义的简洁格式:

```ini
; 注释行以 ; 开头

[nodes]           ; 节点总数 N
3
[topo]            ; 拓扑矩阵: from to rssi
0 1 -40           ; 节点0→节点1, RSSI=-40dBm
0 2 -40           ; 节点0→节点2, RSSI=-40dBm
1 0 -40
1 2 -50
2 0 -40
2 1 -50
[node]            ; 节点定义: index role gateway_no node_id
0 G 0 1           ; 索引0, 网关, gateway_no=0, node_id=1
[node]
1 B 0 1000001     ; 索引1, 信标, gateway_no=0, node_id=1000001
[node]
2 B 0 1000002
```

### 字段说明

| 字段 | 格式 | 说明 |
|------|------|------|
| `[nodes]` | 数字 | 节点总数 |
| `[topo]` | `from to rssi` | `from`/`to` 为节点索引(0~N-1), `rssi` 为负 dBm 值或 `x`(不可达) |
| `[node]` | `index role gateway_no node_id` | role: `G`=网关, `B`=信标 |

### RSSI 与信号等级

| RSSI | 等级 | 判定条件 |
|------|------|----------|
| ≥ -65 | 强 | 1 周期内 1 次命中即入网 |
| -80 ~ -65 | 中 | 2 周期内 1 次命中即入网 |
| < -80 | 弱 | 4 周期内 2 次命中即入网 |

## 当前场景

| 场景 | 节点 | 拓扑 | 状态 |
|------|------|------|------|
| 场景1 | 3 (1GW+2BCN) | 全连通 | 默认启用 |
| 场景2 | 3 (1GW+2BCN) | 链式 | 注释 |
| 场景3 | 4 (2GW+2BCN) | 多网关 | 注释 |

## 与 main.c 的关系

> **注意**: 当前 `main.c` 的拓扑由 `SCENARIO_MODE` 宏控制硬编码, 不从 `.conf` 文件读取。
> `.conf` 文件用于文档记录和手动拓扑参考。若需实际切换场景, 请修改 `main.c` 中的 `SCENARIO_MODE` 宏值:

```c
#define SCENARIO_MODE  0u  // 0=全连通, 1=树形网状混合
```

如需新增场景, 在 `sim_init_scenario()` 中添加 `#elif SCENARIO_MODE == N` 分支。
