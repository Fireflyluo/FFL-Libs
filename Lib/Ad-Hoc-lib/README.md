# Ad-Hoc-lib

`Ad-Hoc-lib` 是本工程的自组网协议库。  
默认保留仓库内副本 `lib/Ad-Hoc-lib/` 作为参考/回退路径；当 `xmake` 配置了外部 `adhoc_repo_dir` 时，工程会优先通过本地 `xrepo` 包仓引入协议核心。  
库与底层 RF 驱动解耦，协议逻辑不放在 `AROS-RF-LIB` 内。

## 1. 库定位

- 提供协议层能力：帧模型、时序、组网状态机、数据转发与确认。
- 通过 `adhoc_link_ops_t` 对接任意链路实现。
- 静态内存模型，无动态分配。
- 适合作为"可复用协议内核"被多个工程接入。

## 2. 快速入口

| 文档 | 说明 |
|------|------|
| [USAGE.md](docs/USAGE.md) | 最小接入说明 |
| [protocol-design.md](docs/protocol-design.md) | 协议语义（帧格式、Flag、状态机、时间窗） |
| [software-architecture.md](docs/software-architecture.md) | 软件实现拆解（模块职责、收发路径、静态内存模型） |
| [configuration.md](docs/configuration.md) | 统一配置说明（`adhoc_config.h` 可配置项） |
| [软件设计文档.md](docs/软件设计文档.md) | 软件设计基线 |
| [porting-guide.md](docs/porting-guide.md) | 平台移植指南 |
| [docs/README.md](docs/README.md) | 文档导航总览 |
| [test/README.md](test/README.md) | Windows 多线程仿真入口 |

## 3. 目录结构

- `include/`：公开头文件
- `src/`：协议实现
- `docs/`：协议、实现、移植、使用文档
- `port/ch32v208/`：CH32V208 平台参考实现
- `test/`：Windows 多线程仿真与回归场景

## 4. 当前工程参考

- 协议任务接入示例：`app/tasks/ad_hoc_task.c`
- 链路适配示例：`app/adapters/adhoc_link_aros.c`
- CH32 平台端口：`port/ch32v208/`

## 5. 构建（在本仓库）

```bash
xmake
```
协议本身（Ad-Hoc）：约 ROM 13.3KB、RAM 14B。

## 6. 与统一规范的关系

Ad-Hoc-lib 作为**协议库**，不遵循传感器库的器件开关模式（无 `add_requires` configs 选项）。其接入方式为直接链接静态库 `adhoc-lib-core`，通过 `adhoc_link_ops_t` 注入链路实现。
