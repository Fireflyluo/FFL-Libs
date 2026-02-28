-- 传感器驱动模块 ARM 交叉编译配置
-- 使用 ARM GCC 工具链构建

-- 设置项目名称
set_project("embedded-sensor-drivers-arm")

-- 设置交叉编译工具链和目标平台
set_plat("cross")
set_arch("arm")

-- 设置 ARM GCC 工具链
set_toolchains("gnu-rm", {
    sdkdir = "D:/APP/path/arm-gcc-15.2",
    bin_prefix = "arm-none-eabi-",
    cross = "arm-none-eabi-"
})

-- 设置语言标准
set_languages("c11")

-- 添加编译选项
add_cflags(
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    "-Wall",
    "-Wextra",
    "-ffunction-sections",
    "-fdata-sections",
    {force = true}
)

-- C++ 选项（如果需要）
add_cxxflags(
    "-fno-exceptions",
    "-fno-rtti",
    {force = true}
)

-- 链接选项
add_ldflags(
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    "-Wl,--gc-sections",
    {force = true}
)

-- 包含各个传感器驱动
includes("ICM42688P/xmake.lua")
includes("QMI8658A/xmake.lua") 
includes("sc7a20htr/xmake.lua")
includes("sht40/xmake.lua")
includes("oled/xmake.lua")

-- 聚合目标
target("all_sensors")
    set_kind("phony")
    add_deps("icm42688p", "qmi8658a", "sc7a20htr", "sht40", "oled")
    on_load(function (target)
        print("嵌入式传感器驱动库 (ARM) - 所有驱动")
    end)