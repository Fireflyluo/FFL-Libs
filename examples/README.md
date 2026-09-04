# Examples

`examples/` 只存放最小、可复现的组件使用工程。每个示例必须明确列出依赖的组件和 port，且不得通过默认聚合 target 引入无关驱动。

现有 `driver-dual-entry/` 演示同一驱动 core 的完整本地 package 入口与源码裁剪入口。后续每迁入一个正式组件，至少增加一个独立 example 或组件内 host test。
