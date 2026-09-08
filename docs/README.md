# 文档导航

这个目录按使用目的划分：

- 面向使用者的说明：仓库根目录 README、各目录 README、组件目录下的 `docs/`。
- 面向维护者的资料：[`maintainer/`](maintainer/README.md)。

## 从哪里开始

| 目标                          | 入口                                                                  |
| ----------------------------- | --------------------------------------------------------------------- |
| 选择组件并接入 Xmake 工程     | [根目录 README](../README.md)                                         |
| 查看所有可用组件              | [组件目录说明](../components/README.md)                               |
| 学习某个驱动的 API 和移植方式 | 对应组件的 `docs/README.md`、`API_REFERENCE.md` 或 `PORTING_GUIDE.md` |
| 使用板级适配                  | [ports 说明](../ports/README.md)                                      |
| 运行已有示例                  | [examples 说明](../examples/README.md)                                |
| 配置 ARM 或 WCH 工具链        | [工具链说明](../toolchains/README.md)                                 |
| 使用 CherryUSB 等上游库       | [第三方库说明](../third_party/README.md)                              |
| 查看测试与验证范围            | [测试说明](../tests/README.md)                                        |

## 文档中的状态含义

- **已验证**：有明确的自动测试或目标板证据。
- **已构建**：已完成编译，但不代表外设在真实硬件上工作。
- **待验证**：已有代码或适配，但缺少目标板证据。
- **实验性**：不进入默认构建，不承诺稳定 API。
