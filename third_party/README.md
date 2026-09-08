# Third-Party Libraries

`third_party/` 用于保存本仓库依赖或参考的上游开源库。第三方源码保持上游目录和命名，不改造成 `components/` 中的 `ffl.*` 组件。

```text
third_party/
└── usb/
    └── cherryusb/       # CherryUSB 上游源码与本地接入说明
```

## 目录约定

- 按技术领域分类，例如 `usb/`、`crypto/`、`filesystem/`。
- 每个库单独一个目录，优先使用 Git submodule 或固定版本源码归档。
- 保留上游 `LICENSE`、版本信息和来源地址。
- 本仓库的 MCU、USB IP、开发板适配代码放到 `ports/`。
- 可运行的接入验证放到 `examples/`。
- 不在这里放本仓库自己的通用 API，也不让正式 `components/` 反向依赖未验证的第三方库。

第三方库默认不进入根 `xmake.lua` 的构建入口。只有在对应适配或示例中显式 `includes()` 后才参与构建。
