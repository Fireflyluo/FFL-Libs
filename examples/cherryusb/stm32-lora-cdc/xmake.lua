-- examples/cherryusb/stm32-lora-cdc
-- STM32F103C8 + CherryUSB fsdev CDC ACM
-- SDK 复用 examples/stm32-base-driver/sdk

includes("../../../toolchains/arm-none-eabi.lua")

local ROOT = "../../.."
local CUSB = path.join(ROOT, "third_party/usb/cherryusb/upstream")
local PORT_BOARD = path.join(ROOT, "ports/stm32/f1/cherryusb/stm32-lora")
local SDK = path.join(ROOT, "examples/stm32-base-driver/sdk")
local HAL_SRC = path.join(SDK, "STM32F1xx_HAL_Driver/Src")
local HAL_INC = path.join(SDK, "STM32F1xx_HAL_Driver/Inc")
local CMSIS_DEV_INC = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Include")
local CMSIS_INC = path.join(SDK, "CMSIS/Include")
local STARTUP = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s")
local SYSTEM_C = path.join(SDK, "CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c")
local LINKER = path.join(ROOT, "examples/stm32-base-driver/project/STM32F103C8Tx_FLASH.ld")

set_languages("c11")

target("stm32-lora-cdc")
    set_kind("binary")
    set_basename("stm32-lora-cdc")
    set_plat("cross")
    set_arch("arm")
    set_toolchains("arm-none-eabi")
    set_policy("check.auto_ignore_flags", false)

    add_files("main.c")
    add_files(path.join(PORT_BOARD, "usb_port_stm32f1.c"))
    add_files(
        path.join(ROOT, "examples/stm32-base-driver/bsp/src/syscalls.c"),
        path.join(ROOT, "examples/stm32-base-driver/bsp/src/sysmem.c")
    )

    -- CherryUSB device + fsdev + CDC
    add_files(
        path.join(CUSB, "core/usbd_core.c"),
        path.join(CUSB, "class/cdc/usbd_cdc_acm.c"),
        path.join(CUSB, "port/fsdev/usb_dc_fsdev.c"),
        path.join(CUSB, "port/fsdev/usb_glue_st.c")
    )

    -- STM32 HAL 子集（含 PCD / LL USB）
    add_files(
        path.join(HAL_SRC, "stm32f1xx_hal.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_cortex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_flash_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_gpio_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pcd.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pcd_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_pwr.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_rcc_ex.c"),
        path.join(HAL_SRC, "stm32f1xx_hal_uart.c"),
        path.join(HAL_SRC, "stm32f1xx_ll_usb.c"),
        SYSTEM_C,
        STARTUP
    )

    add_includedirs(
        PORT_BOARD,
        path.join(CUSB, "core"),
        path.join(CUSB, "common"),
        path.join(CUSB, "class/cdc"),
        path.join(CUSB, "port/fsdev"),
        path.join(ROOT, "examples/stm32-base-driver/bsp/include"),
        HAL_INC,
        path.join(HAL_INC, "Legacy"),
        CMSIS_DEV_INC,
        CMSIS_INC
    )

    add_defines("STM32F103xB", "USE_HAL_DRIVER")
    add_cflags("-mcpu=cortex-m3", "-mthumb", "-ffunction-sections",
               "-fdata-sections", "-Wall")
    add_asflags("-mcpu=cortex-m3", "-mthumb")
    add_ldflags("-mcpu=cortex-m3", "-mthumb", "--specs=nano.specs",
                "-Wl,--gc-sections", "-T" .. LINKER)

    after_build(function (target)
        local elf = target:targetfile()
        if elf then
            os.execv("arm-none-eabi-objcopy", {"-O", "binary", elf, elf .. ".bin"})
        end
    end)
