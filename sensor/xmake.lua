-- 传感器驱动模块总入口

-- 按配置开关包含各个传感器驱动
if has_config("sensor_icm42688p") then
    includes("ICM42688P/xmake.lua")
end
if has_config("sensor_qmi8658a") then
    includes("QMI8658A/xmake.lua")
end
if has_config("sensor_sc7a20htr") then
    includes("sc7a20htr/xmake.lua")
end
if has_config("sensor_sht40") then
    includes("sht40/xmake.lua")
end
if has_config("sensor_oled") then
    includes("oled/xmake.lua")
end
