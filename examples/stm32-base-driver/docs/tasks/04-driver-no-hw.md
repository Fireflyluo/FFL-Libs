# Task 4：无硬件 / 无 port 时如何降级

**场景：** 没有焊 SC7A20、I2C 不通、或只想先验证调度链路。

## 仓库允许的降级

组件可以**不 init 成功**；应用用标志位降级，**不要**在 `components/` 里写死“没有传感器也假装读成功”。

本示例做法：

```c
s_sensor_ok = (rc == 0) ? 1u : 0u;

static void read_one_sample(void) {
  if (s_sensor_ok != 0u) {
    rc = ffl_sc7a20_read_raw(&s_accel, &raw);
  } else {
    rc = -1; /* 不碰 I2C */
  }

  if (rc == 0) {
    encode_sample(rec, &raw);
    g_sample_ok_count++;
  } else {
    memset(rec, 0xFF, sizeof(rec));  /* 占位记录 */
    g_sample_err_count++;
  }
  ffl_ringbuffer_put(&s_rb, rec, 6); /* 链路仍走 put */
}
```

## 你仍然能验证什么

| 组件 | 无传感器时 |
|------|------------|
| `ffl.osal` | LED/STATS 事件照常 |
| `ffl.sw_timer` | 500ms 照常触发 |
| `ffl.ringbuffer` | 仍写入 6 字节 0xFF，`drained` 照常增 |
| `ffl.sc7a20` | 不调用 read，或调用后 rc≠0 |

COM4 预期：

```text
[stats] sensor_ok=0 ok=0 err=N drained=...
```

`ok` 停住、`err` 增、`drained` 增 → **调度与缓冲 OK**，只是驱动/硬件未就绪。

## 和“假成功”的区别

| 做法 | 是否推荐 |
|------|----------|
| 应用层 `sensor_ok` 降级 + 占位数据 | 推荐（本示例） |
| 在 sc7a20 core 里编译假数据 | 不推荐（污染组件） |
| 不写 ringbuffer、只 printf | 可，但少演示一条链路 |

## 和 port 的关系

没有板级 I2C 时：

- 仍可**不链接** transport，或  
- 提供返回 `-ENODEV` 的空 port  

驱动 core 不会包含 HAL；缺的是 **port 实现**，不是改 core。

下一篇：[05-features.md](05-features.md)
