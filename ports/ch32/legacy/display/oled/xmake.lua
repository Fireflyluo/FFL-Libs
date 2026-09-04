-- OLED显示屏驱动构建配置

-- 定义OLED驱动库
target("oled")
    set_kind("static") -- 创建静态库
    set_basename("oled")
    add_files("OLED.c", "OLED_Data.c")
    add_headerfiles("OLED.h", "OLED_Data.h")
    add_includedirs(".", {public = true}) -- 公开包含目录，供其他模块使用
    set_languages("c11")
    add_defines("OLED_DRIVER_AVAILABLE", {public = true})
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
