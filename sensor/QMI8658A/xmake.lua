-- QMI8658A 6轴IMU驱动构建配置

-- 定义QMI8658A驱动库
target("qmi8658a")
    set_kind("static") -- 创建静态库
    set_basename("qmi8658a")
    add_files("qmi8658a_driver.c", "qmi8658a_reg.c")
    add_headerfiles("qmi8658a_driver.h", "qmi8658a_reg.h", "gyro_device.h")
    add_includedirs(".", {public = true}) -- 公开包含目录，供其他模块使用
    set_languages("c11")
    add_defines("QMI8658A_DRIVER_AVAILABLE", {public = true})
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
