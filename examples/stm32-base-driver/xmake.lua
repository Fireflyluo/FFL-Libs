-- ============================================================================
-- examples/stm32-base-driver：FFL 仓库用法示例固件（STM32F103 交叉编译）
--
-- 构建来源分层（路径均相对仓库根）：
--   1) components/     ffl 组件源码（不复制到本目录）
--   2) ports/stm32/f1  MCU 南向：osal 临界区、I2C xfer、sc7a20 bind
--   3) app/            业务任务与功能演示
--   4) bsp/            本板时钟 / LED / UART / I2C 引脚
--   5) sdk/            裁剪版 STM32F1 HAL + CMSIS
--
-- 阅读顺序建议：readme.md → docs/tasks/ → app/main.c → app/app_task.c
--
-- 构建：
--   xmake f -P examples/stm32-base-driver -p cross --toolchain=arm-none-eabi -a arm -m release
--   xmake    -P examples/stm32-base-driver
-- ============================================================================

-- 注册仓库的 arm-none-eabi 工具链定义（bin 需在 PATH，或配置 sdkdir）
includes("../../toolchains/arm-none-eabi.lua")

-- 相对本文件（示例工程根）定位仓库 components
local ROOT = "../.."
local OSAL_SRC = path.join(ROOT, "components/runtime/osal/src")
local PT_INC = path.join(ROOT, "components/runtime/protothreads/include")
local ATOMIC_INC = path.join(ROOT, "components/foundation/atomic/include")
local DRIVER_PORT_INC = path.join(ROOT, "components/foundation/driver-port/include")
local ULOG_SRC = path.join(ROOT, "components/foundation/ulog/src")
local ULOG_INC = path.join(ROOT, "components/foundation/ulog/include")
local RB_SRC = path.join(ROOT, "components/foundation/ringbuffer/src")
local RB_INC = path.join(ROOT, "components/foundation/ringbuffer/include")
local ST_SRC = path.join(ROOT, "components/foundation/sw-timer/src")
local ST_INC = path.join(ROOT, "components/foundation/sw-timer/include")
local S7A20_SRC = path.join(ROOT, "components/drivers/sensor/accelerometer/sc7a20/src")
local S7A20_INC = path.join(ROOT, "components/drivers/sensor/accelerometer/sc7a20/include")
local OSAL_INC = path.join(ROOT, "components/runtime/osal/include")
-- 芯片南向适配（正式 ports）
local PORT_F1 = path.join(ROOT, "ports/stm32/f1")
local CUSB = path.join(ROOT, "third_party/usb/cherryusb/upstream")
local PORT_USB = path.join(PORT_F1, "cherryusb/stm32-lora")

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
    set_optimize("smallest")

    -- 让 -mcpu=cortex-m3/-mthumb 等原样传给 arm-none-eabi-gcc，
    -- 不被 xmake 的自动忽略标志逻辑吞掉
    set_policy("check.auto_ignore_flags", false)

    -- ---- 应用（OSAL 多任务） + 板级 + ports/stm32/f1 南向 ----
    -- 原板：W25Q 软 SPI + ST7735 硬 SPI1+DMA + USB CDC 收图
    add_files(
        "app/main.c",
        "app/heartbeat_task.c",
        "app/sc7a20_task.c",
        "app/flash_task.c",
        "app/lcd_task.c",
        "app/alarm_task.c",
        "app/features_demo.c",
        "app/usb_img.c",
        "bsp/src/board.c",
        "bsp/src/board_i2c.c",
        "bsp/src/stm32f1xx_it.c",
        "bsp/src/st7735.c",
        "bsp/src/font6x8.c",
        "bsp/src/w25q.c",
        "bsp/src/syscalls.c",
        "bsp/src/sysmem.c",
        path.join(PORT_F1, "osal/osal_port.c"),
        path.join(PORT_F1, "driver_port/driver_port.c"),
        path.join(PORT_F1, "sc7a20/sc7a20_bind.c"),
        path.join(PORT_USB, "usb_port_stm32f1.c"),
        path.join(CUSB, "core/usbd_core.c"),
        path.join(CUSB, "class/cdc/usbd_cdc_acm.c"),
        path.join(CUSB, "port/fsdev/usb_dc_fsdev.c"),
        path.join(CUSB, "port/fsdev/usb_glue_st.c")
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
        path.join(S7A20_SRC, "ffl_sc7a20.c"),
        path.join(ULOG_SRC, "ulog.c")
    )

    -- ---- STM32F1 SDK：HAL 子集 + 系统时钟 + 启动文件 ----
    add_files(
        path.join(HAL_SRC, "stm32f1xx_hal.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_cortex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_dma.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_exti.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_i2c.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_spi.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_uart.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_tim.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pwr.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pcd.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pcd_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_ll_usb.c"),
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
        S7A20_INC,
        ULOG_INC,
        path.join(PORT_F1, "osal"),
        path.join(PORT_F1, "driver_port"),
        path.join(PORT_F1, "sc7a20"),
        PORT_USB,
        path.join(CUSB, "core"),
        path.join(CUSB, "common"),
        path.join(CUSB, "class/cdc"),
        path.join(CUSB, "port/fsdev")
    )

    -- ---- 宏与编译选项 ----
    add_defines("STM32F103xB", "USE_HAL_DRIVER")
    add_defines("FFL_SC7A20_ASYNC_ENABLED=0")
    add_defines("ULOG_ENABLE=1", "ULOG_LEVEL_MIN=ULOG_LEVEL_INFO",
                "ULOG_BUFFER_SIZE=512", "ULOG_LINE_MAX=128",
                "ULOG_ISR_LINE_MAX=64")

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
