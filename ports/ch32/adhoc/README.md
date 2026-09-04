# CH32V208 Port

该目录是 `Ad-Hoc-lib` 在 CH32V208 平台的最小端口参考实现。

## 内容

- `adhoc_port_ch32_now_tc/now_us`
  - 基于 `arf_get_tc()` 与 `ARF_TC_us` 提供协议层微秒时钟。
- `adhoc_port_ch32_rand_u16`
  - 使用 TMOS 标准库 `rand/srand` 生成 16 位随机数。
- `adhoc_port_ch32_critical_enter/exit`
  - 对应 `__disable_irq()` / `__enable_irq()`。

## 用法建议

- 本目录只提供“平台能力封装”，不直接做链路收发。
- 链路收发适配示例见：`app/adapters/adhoc_link_aros.c`。
- 迁移到其他平台时，优先复刻本目录接口形态，再替换底层实现。

完整移植流程见 `lib/Ad-Hoc-lib/docs/porting-guide.md`。
