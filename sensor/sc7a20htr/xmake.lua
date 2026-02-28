-- SC7A20HTR三轴加速度计驱动构建配置

-- 定义SC7A20HTR驱动库
target("sc7a20htr")
    set_kind("static") -- 创建静态库
    set_basename("sc7a20htr")
    
    -- 添加源文件
    add_files("sc7a20_core.c", "sc7a20_core_async.c")
    
    -- 添加平台相关文件（如果存在）
    if os.exists("sensor/sc7a20htr/platform/platform.c") then
        add_files("platform/platform.c")
    end
    
    -- 添加头文件
    add_headerfiles("sc7a20_core.h", "sc7a20_async.h", "sc7a20_def.h", "sc7a20_hal.h", "sc7a20_reg.h")
    
    -- 添加包含目录
    add_includedirs(".", {public = true}) -- 当前目录
    add_includedirs("inc", {public = true}) -- inc目录包含重要的头文件
    
    -- 如果平台目录存在，也添加到包含路径
    if os.exists("sensor/sc7a20htr/platform") then
        add_includedirs("platform", {public = true})
    end
    
    set_languages("c11")
    add_defines("SC7A20HTR_DRIVER_AVAILABLE", {public = true})
    
    -- 添加编译选项
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
