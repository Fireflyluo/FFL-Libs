-- 嵌入式传感器驱动库项目配置
-- 支持C11及以上标准

-- 项目基本信息
set_project("embedded-sensor-drivers")
set_version("1.0.0")
set_xmakever("2.5.1")

-- 设置默认语言标准
set_languages("c11")

-- 全局配置选项
option("sensor_icm42688p", {default = true, showmenu = true, description = "Enable ICM42688P driver"})
option("sensor_qmi8658a", {default = true, showmenu = true, description = "Enable QMI8658A driver"})
option("sensor_sc7a20htr", {default = true, showmenu = true, description = "Enable SC7A20HTR driver"})
option("sensor_sht40", {default = true, showmenu = true, description = "Enable SHT40 driver"})
option("sensor_oled", {default = true, showmenu = true, description = "Enable OLED driver"})

-- 定义传感器驱动模块
includes("sensor/xmake.lua")

-- 示例程序（如果需要）
target("example")
    set_kind("phony")
    add_deps("icm42688p", "qmi8658a", "sc7a20htr", "sht40", "oled")
    on_run(function()
        print("嵌入式传感器驱动库 - 示例工程")
        print("使用 'xmake create -P .' 创建新的示例工程")
    end)
