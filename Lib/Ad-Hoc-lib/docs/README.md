# Ad-Hoc-lib 文档总览

本文档集面向“在其他工程复用 `Ad-Hoc-lib` 协议库”的开发者，重点覆盖：

- 协议模型与帧语义
- 软件实现结构与运行机制
- 平台移植边界与接口契约
- 最小接入与联调方法

## 文档导航

- `protocol-design.md`：协议设计基线（帧格式、Flag 语义、状态机、时间窗）
- `software-architecture.md`：软件实现拆解（模块职责、收发路径、静态内存模型）
- `porting-guide.md`：移植指南（链路适配接口、平台依赖、验证清单）
- `USAGE.md`：使用说明（初始化、主循环、上报与观测）

## 读者建议顺序

1. 首次接入：先看 `protocol-design.md` -> `porting-guide.md` -> `USAGE.md`
2. 问题定位：先看 `software-architecture.md` -> `USAGE.md` 的“常见问题”
3. 对齐协议细节：以 `protocol-design.md` 为库内基线，同时结合工程级协议文档核对

## 版本边界说明

- 本目录描述的是当前 `lib/Ad-Hoc-lib/` 实际实现状态。
- 如果工程级协议文档后续升级，本目录需要同步更新，避免“文档口径领先/落后于代码”。
