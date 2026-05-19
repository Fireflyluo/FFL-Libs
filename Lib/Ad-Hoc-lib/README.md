# Ad-Hoc-lib

`Ad-Hoc-lib` 是本工程的自组网协议库，路径固定在 `lib/Ad-Hoc-lib/`。  
库与底层 RF 驱动解耦，协议逻辑不放在 `AROS-RF-LIB` 内。

## 1. 库定位

- 提供协议层能力：帧模型、时序、组网状态机、数据转发与确认。
- 通过 `adhoc_link_ops_t` 对接任意链路实现。
- 静态内存模型，无动态分配。
- 适合作为“可复用协议内核”被多个工程接入。

## 2. 快速入口

- 对外 API：`include/adhoc_api.h`
- 最小接入：`docs/USAGE.md`
- 协议语义：`docs/protocol-design.md`
- 软件实现：`docs/software-architecture.md`
- 平台移植：`docs/porting-guide.md`
- 文档导航：`docs/README.md`

## 3. 目录结构

- `include/`：公开头文件
- `src/`：协议实现
- `docs/`：协议、实现、移植、使用文档
- `port/ch32v208/`：CH32V208 平台参考实现

## 4. 当前工程参考

- 协议任务接入示例：`app/tasks/ad_hoc_task.c`
- 链路适配示例：`app/adapters/adhoc_link_aros.c`
- CH32 平台端口：`port/ch32v208/`

## 5. 构建（在本仓库）

```bash
xmake
```
