-- 示例工程：使用嵌入式传感器驱动库

-- 设置项目信息
set_project("sensor-example")
set_version("1.0.0")
set_languages("c11")

-- 添加对传感器驱动库的依赖
add_requires("embedded-sensor-drivers")

-- 定义示例应用程序
target("sensor-example-app")
    set_kind("binary") -- 生成可执行文件
    add_files("main.c") -- 主程序文件
    add_packages("embedded-sensor-drivers") -- 链接传感器驱动库
    
    -- 根据需要启用特定的传感器驱动
    add_defines("USE_ICM42688P")
    add_defines("USE_QMI8658A") 
    add_defines("USE_SC7A20HTR")
    add_defines("USE_SHT40")
    add_defines("USE_OLED")
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra")
    elseif is_plat("windows") then
        add_cxflags("/W4")
    end

-- 设置默认构建目标
set_default("sensor-example-app")