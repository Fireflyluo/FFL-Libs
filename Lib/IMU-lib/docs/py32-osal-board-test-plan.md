# PY32 OSAL IMU Board Test Plan

## Goal

Validate the current `imu-lib` package integration on the PY32F403 OSAL board with two IMUs on the same `I2C2` bus:

- `QMI8658A` at `0x6A`
- `ICM42688P` at `0x69`

The board-side log UART for this round is `COM19` at `115200`, and the debugger is on `COM13`.

## Preflight

1. Confirm the debugger is enumerated and `pyocd` can see the target.
2. Confirm the log UART is `COM19`.
3. Ensure no stale process is holding `COM19` or the CMSIS-DAP probe.
4. Confirm both IMUs are powered and soldered to the `I2C2` bus.

## Build

Project: `D:\Desktop\Ad Hoc Network\PY32f403_dome\demo\osal project`

Commands:

```powershell
cd "D:\Desktop\Ad Hoc Network\PY32f403_dome\demo\osal project"
xmake f -c
xmake
```

Expected artifact:

- `dist/osal.elf`
- `dist/osal.bin`
- `dist/osal.hex`

If build fails, rerun with:

```powershell
xmake -vD
```

## Flash

```powershell
cd "D:\Desktop\Ad Hoc Network\PY32f403_dome\demo\osal project"
pyocd flash -t py32f403xd -f 10M dist/osal.elf
```

## Serial Observe

Minimal PowerShell monitor:

```powershell
$sp = New-Object System.IO.Ports.SerialPort "COM19",115200,"None",8,"one"
$sp.NewLine = "`r`n"
$sp.ReadTimeout = 200
$sp.Open()
try {
    while ($true) {
        try {
            $text = $sp.ReadExisting()
            if ($text) { Write-Host -NoNewline $text }
        } catch {}
        Start-Sleep -Milliseconds 100
    }
} finally {
    $sp.Close()
}
```

## Expected Runtime Sequence

1. `board_init()` initializes UART2 DMA and `I2C2`.
2. `I2C2_ScanDevices()` prints scanned addresses before the OSAL scheduler starts.
3. `sensor_task_init()` attempts both IMU initializations.
4. Every 200 ms, the task prints one line per ready IMU.

## Acceptance Criteria

### L0: Bus Discovery

Serial output includes at least:

- `I2C2 : 0x69`
- `I2C2 : 0x6A`

or the final summary line contains both addresses.

### L1: Probe and Init

Serial output includes:

- `QMI8658A init rc=0 addr=0x6A id=0x05`
- `ICM42688P init rc=0 addr=0x69 id=0x47`

Note: current `imu-lib` code expects `ICM42688P` WHO_AM_I to be `0x47`. The copied `icm42688_reg.h` also contains `ICM42688P_DEVICE_ID 0x68U`, so any `ICM42688P` probe failure with `0x69` still visible on the bus should first be triaged as an ID-definition mismatch.

### L2: Stable Sampling

Within 3 seconds after boot, repeated lines appear for both devices:

- `QMI8658A seq=...`
- `ICM42688P seq=...`

and no repeated `read rc=` failure loop.

### L3: Motion Response

Rotate or tap the board. At least one accelerometer axis and one gyro axis should change noticeably on both sensors.

### L4: Reboot Consistency

Power-cycle or reset the board 3 times. Each boot should still pass L0-L2.

## Failure Triage

### No Serial Output

- Recheck `COM19`
- Recheck UART2 `115200`
- Recheck whether another process is holding the port

### No `0x69` or `0x6A`

- Check `PB10/PB11`
- Check pull-ups and power
- Check whether `MX_I2C2_Init()` ran

### `0x69/0x6A` Present but Init Fails

- Check WHO_AM_I expectation
- Check register read path in `imu_port_adapter.c`
- Check whether the device requires post-reset delay longer than current value

### Init Passes but Read Fails Later

- Watch for bus timeout or repeated NACK
- Check whether `HAL_I2C_Mem_Read` / `HAL_I2C_Mem_Write` matches each chip's transaction pattern

## Current Test Scope

This plan validates:

- xmake local package integration
- I2C bus reachability
- WHO_AM_I path
- basic init path
- periodic sample read path

This plan does not yet validate:

- interrupt mode
- FIFO mode
- calibration
- power mode switching
