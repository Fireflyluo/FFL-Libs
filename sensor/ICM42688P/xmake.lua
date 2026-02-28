-- ICM42688P 6轴IMU驱动构建配置

-- 定义ICM42688P驱动库
target("icm42688p")
    set_kind("static") -- 创建静态库
    set_basename("icm42688p")
    add_files("icm42688p.c")
    add_headerfiles("icm42688p.h", "icm42688_reg.h")
    add_includedirs(".", {public = true}) -- 公开包含目录，供其他模块使用
    set_languages("c11")
    add_defines("ICM42688P_DRIVER_AVAILABLE", {public = true})
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
