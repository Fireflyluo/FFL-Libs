-- ============================================================================
-- examples/stm32-base-driver: STM32F103C8T6 最小系统板固件示例
--
-- 演示组件（全部来自仓库 components/，不复制到本目录）：
--   ffl.osal / ffl.ringbuffer / ffl.sw_timer / ffl.sc7a20
--
-- 按仓库端口惯例：MCU 固件 target 直接把组件源码 add_files 进来，使整个
-- target 统一使用 Cortex-M3 编译选项（组件自身 xmake 保留 host object 用法，
-- 供 host 测试与需要裁剪的工程使用）。
--
-- 构建：
--   xmake f -P examples/stm32-base-driver
--   xmake    -P examples/stm32-base-driver
-- 产物：stm32-base-driver(.elf/.bin)，烧录到 F103C8T6。
-- ============================================================================

-- 注册仓库的 arm-none-eabi 工具链定义（bin 需在 PATH，或配置 sdkdir）
includes("../../toolchains/arm-none-eabi.lua")

-- 相对本文件（示例工程根）定位仓库 components
local ROOT = "../.."
local OSAL_SRC = path.join(ROOT, "components/runtime/osal/src")
local PT_INC = path.join(ROOT, "components/runtime/protothreads/include")
local ATOMIC_INC = path.join(ROOT, "components/foundation/atomic/include")
local DRIVER_PORT_INC = path.join(ROOT, "components/foundation/driver-port/include")
local RB_SRC = path.join(ROOT, "components/foundation/ringbuffer/src")
local RB_INC = path.join(ROOT, "components/foundation/ringbuffer/include")
local ST_SRC = path.join(ROOT, "components/foundation/sw-timer/src")
local ST_INC = path.join(ROOT, "components/foundation/sw-timer/include")
local S7A20_SRC = path.join(ROOT, "components/drivers/sensor/accelerometer/sc7a20/src")
local S7A20_INC = path.join(ROOT, "components/drivers/sensor/accelerometer/sc7a20/include")
local OSAL_INC = path.join(ROOT, "components/runtime/osal/include")

local SDK = "sdk"
local HAL_SRC = path.join(SDK, "STM32F1xx_HAL_Driver/Src")
local HAL_INC = path.join(SDK, "STM32F1xx_HAL_Driver/Inc")
local CMSIS_DEV_INC = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Include")
local CMSIS_INC = path.join(SDK, "CMSIS/Include")
local STARTUP = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s")
local SYSTEM_C = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c")
local LINKER = path.join(os.projectdir(), "project/STM32F103C8Tx_FLASH.ld")

set_languages("c11")

target("stm32-base-driver")
    set_kind("binary")
    set_basename("stm32-base-driver")
    set_plat("cross")
    set_arch("arm")
    set_toolchains("arm-none-eabi")

    -- 让 -mcpu=cortex-m3/-mthumb 等原样传给 arm-none-eabi-gcc，
    -- 不被 xmake 的自动忽略标志逻辑吞掉
    set_policy("check.auto_ignore_flags", false)

    -- ---- 应用与板级 port（本示例自有，仅 port 层，不含组件实现） ----
    add_files(
        "app/main.c",
        "app/app_task.c",
        "bsp/src/board.c",
        "bsp/src/osal_port_stm32.c",
        "bsp/src/stm32_time_ops.c",
        "bsp/src/sc7a20_i2c_transport.c",
        "bsp/src/stm32f1xx_it.c",
        "bsp/src/syscalls.c",
        "bsp/src/sysmem.c"
    )

    -- ---- 仓库组件源码（ffl.osal 含 osal_pt，依赖 protothreads 头） ----
    add_files(
        path.join(OSAL_SRC, "osal.c"),
        path.join(OSAL_SRC, "osal_event.c"),
        path.join(OSAL_SRC, "osal_memory.c"),
        path.join(OSAL_SRC, "osal_msg.c"),
        path.join(OSAL_SRC, "osal_timer.c"),
        path.join(OSAL_SRC, "osal_pt.c"),
        path.join(OSAL_SRC, "osal_port.c"),
        path.join(RB_SRC, "ringbuffer.c"),
        path.join(ST_SRC, "sw_timer.c"),
        path.join(S7A20_SRC, "sc7a20_core.c"),
        path.join(S7A20_SRC, "sc7a20_sync.c"),
        path.join(S7A20_SRC, "ffl_sc7a20.c")
    )

    -- ---- STM32F1 SDK：HAL 子集 + 系统时钟 + 启动文件 ----
    add_files(
        path.join(HAL_SRC, "stm32f1xx_hal.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_cortex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_exti.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_i2c.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pwr.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc_ex.c"),
        SYSTEM_C,
        STARTUP
    )

    -- ---- include ----
    add_includedirs(
        "app",
        "bsp/include",
        HAL_INC,
        path.join(HAL_INC, "Legacy"),
        CMSIS_DEV_INC,
        CMSIS_INC,
        OSAL_INC,
        PT_INC,
        ATOMIC_INC,
        DRIVER_PORT_INC,
        RB_INC,
        ST_INC,
        S7A20_INC
    )

    -- ---- 宏与编译选项 ----
    add_defines("STM32F103xB", "USE_HAL_DRIVER")
    add_defines("FFL_SC7A20_ASYNC_ENABLED=0") -- 示例只演示同步读

    add_cflags("-mcpu=cortex-m3", "-mthumb", "-ffunction-sections",
               "-fdata-sections", "-Wall")
    add_asflags("-mcpu=cortex-m3", "-mthumb")

    add_ldflags("-mcpu=cortex-m3", "-mthumb", "--specs=nano.specs",
                "-Wl,--gc-sections", "-T" .. LINKER)

    -- 生成 .bin 便于直接烧录
    after_build(function (target)
        local elf = target:targetfile()
        if elf then
            os.execv("arm-none-eabi-objcopy", {"-O", "binary", elf, elf .. ".bin"})
        end
    end)
