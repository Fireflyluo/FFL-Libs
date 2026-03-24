# xrepo example

This example shows how to consume the drivers via a local xmake repository.

## Build

```bash
cd demo/xrepo_example
xmake
```

## Configure (optional)

```bash
xmake f --sensor_oled=n
xmake
```

## Notes

- This example uses SHT40 and provides minimal desktop stubs for I2C/Delay.
- Replace the stub functions with real HAL implementations on your MCU.
