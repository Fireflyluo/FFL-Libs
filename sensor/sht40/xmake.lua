-- SHT40温湿度传感器驱动构建配置

-- 定义SHT40驱动库
target("sht40")
    set_kind("static") -- 创建静态库
    set_basename("sht40")
    add_files("sht40_hal.c")
    add_headerfiles("sht40_hal.h")
    add_includedirs(".", {public = true}) -- 公开包含目录，供其他模块使用
    set_languages("c11")
    add_defines("SHT40_DRIVER_AVAILABLE", {public = true})
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
