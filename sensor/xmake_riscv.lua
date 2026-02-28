-- 嵌入式传感器驱动库 - RISC-V (WCH) 构建配置
-- 使用沁恒 RISC-V 工具链

-- 工具链路径
local TOOLCHAIN_FOLDER = "E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12"
local TOOLCHAIN_PREFIX = "riscv-wch-elf-"
local TOOLCHAIN_BIN = path.join(TOOLCHAIN_FOLDER, "bin")

-- 主配置
set_xmakever("2.6.4")
set_project("embedded-sensors-riscv")
set_version("1.0.0")

-- 定义工具链
toolchain("riscv-wch-gcc")
    set_kind("standalone")
    set_bindir(TOOLCHAIN_BIN)
    set_toolset("cc", TOOLCHAIN_PREFIX .. "gcc")
    set_toolset("cxx", TOOLCHAIN_PREFIX .. "g++")
    set_toolset("ld", TOOLCHAIN_PREFIX .. "ld")
    set_toolset("ar", TOOLCHAIN_PREFIX .. "ar")
    set_toolset("as", TOOLCHAIN_PREFIX .. "as")
    set_toolset("objdump", TOOLCHAIN_PREFIX .. "objdump")
    set_toolset("objcopy", TOOLCHAIN_PREFIX .. "objcopy")
    set_toolset("size", TOOLCHAIN_PREFIX .. "size")
    
    -- 添加 RISC-V 编译选项
    on_load(function (toolchain)
        local cflags = {
            "-march=rv32imac",
            "-mabi=ilp32",
            "-mcmodel=medany",
            "-msmall-data-limit=8",
            "-mno-save-restore",
            "-fno-common",
            "-ffunction-sections",
            "-fdata-sections",
            "-fno-builtin",
            "-fno-hosted",
            "-Wall",
            "-Werror=return-type"
        }
        toolchain:add("cflags", cflags)
        toolchain:add("cxflags", cflags)
    end)
toolchain_end()

-- 设置默认工具链
set_toolchains("riscv-wch-gcc")

-- 添加目标
target("icm42688p")
    set_kind("static")
    add_files("ICM42688P/icm42688p.c")
    add_headerfiles("ICM42688P/icm42688p.h", "ICM42688P/icm42688_reg.h")
    add_includedirs("ICM42688P", {public = true})
    set_languages("c11")

target("qmi8658a")
    set_kind("static")
    add_files("QMI8658A/qmi8658a_driver.c", "QMI8658A/qmi8658a_reg.c")
    add_headerfiles("QMI8658A/qmi8658a_driver.h", "QMI8658A/qmi8658a_reg.h", "QMI8658A/gyro_device.h")
    add_includedirs("QMI8658A", {public = true})
    set_languages("c11")

target("sc7a20htr")
    set_kind("static")
    add_files("sc7a20htr/sc7a20_core.c", "sc7a20htr/sc7a20_core_async.c")
    add_headerfiles("sc7a20htr/inc/sc7a20_core.h", "sc7a20htr/inc/sc7a20_async.h", 
                    "sc7a20htr/inc/sc7a20_def.h", "sc7a20htr/inc/sc7a20_hal.h", 
                    "sc7a20htr/inc/sc7a20_reg.h")
    add_includedirs("sc7a20htr", "sc7a20htr/inc", {public = true})
    set_languages("c11")

target("sht40")
    set_kind("static")
    add_files("sht40/sht40_hal.c")
    add_headerfiles("sht40/sht40_hal.h")
    add_includedirs("sht40", {public = true})
    set_languages("c11")

target("oled")
    set_kind("static")
    add_files("oled/OLED.c", "oled/OLED_Data.c")
    add_headerfiles("oled/OLED.h", "oled/OLED_Data.h")
    add_includedirs("oled", {public = true})
    set_languages("c11")

-- 聚合所有驱动的目标
target("all-sensors")
    set_kind("phony")
    add_deps("icm42688p", "qmi8658a", "sc7a20htr", "sht40", "oled")