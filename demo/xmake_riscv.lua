-- CH32V208 
set_project("CH32V208GBU_Templete")
set_version("1.0.0")

add_rules("mode.debug", "mode.release")
-- 自动生成 VSCode 的 compile_commands.json 文件
add_rules("plugin.compile_commands.autoupdate", {
    outputdir = ".vscode",}
)
-- 设置交叉编译工具链和目标平台
set_plat("cross")
set_arch("riscv")

-- 工具链路径
local TOOLCHAIN_FOLDER = "E:/APP/MRS2/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC12"
local TOOLCHAIN_PREFIX = "riscv-wch-elf-"

-- 设置工具
set_toolset("cc", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-gcc")
set_toolset("cxx", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-g++")
set_toolset("as", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-gcc")
set_toolset("ld", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-gcc")
set_toolset("ar", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-ar")
set_toolset("objcopy", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-objcopy")
set_toolset("objdump", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-objdump")
set_toolset("size", TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-size")



-- 目标设置
target("CH32V208GBU_Templete")
    set_kind("binary")
    set_extension(".elf")
    
    -- 编译选项
    add_cflags(
        "-march=rv32imacxw",
        "-mabi=ilp32",
        "-msmall-data-limit=8",
        "-msave-restore",
        "-fmax-errors=20",
        "-Os",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-Wunused",
        "-Wuninitialized",
        "-g",
        "-std=gnu17",
        {force = true})
    
    -- C++选项
    add_cxxflags(
        "-march=rv32imacxw",
        "-mabi=ilp32",
        "-msmall-data-limit=8",
        "-msave-restore",
        "-fmax-errors=20",
        "-Os",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-Wunused",
        "-Wuninitialized",
        "-g",
        "-std=gnu++11",
        "-fabi-version=0",
        {force = true})
    
    -- ASM_FLAGS
    add_asflags(
        "-march=rv32imacxw",
        "-mabi=ilp32",
        "-msmall-data-limit=8",
        "-msave-restore",
        "-fmax-errors=20",
        "-Os",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-Wunused",
        "-Wuninitialized",
        "-g",
        {force = true})
    
    -- 为汇编文件添加 -x assembler-with-cpp
    on_load(function (target)
        target:add("asflags", "-x", "assembler-with-cpp", {force = true})
    end)
    

    -- 调试选项
    if is_mode("debug") then
        add_cflags("-g", "-O0", {force = true})
        add_defines("DEBUG=1")
    else
        add_cflags("-Os", {force = true})
        add_defines("NDEBUG=1")
    end

    -- 链接标志
    add_ldflags(
        "-march=rv32imacxw",
        "-mabi=ilp32",
        "-msmall-data-limit=8",
        "-msave-restore",
        "-fmax-errors=20",
        "-Os",
        "-fmessage-length=0",
        "-fsigned-char",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
        "-Wunused",
        "-Wuninitialized",
        "-g",
        "-T", "./sdk/HAL/Link.ld",
        "-L./sdk/LIB",
        "-nostartfiles",
        "-Xlinker", "--gc-sections",
        "-Wl,-Map,CH32V208GBU_Templete.map",
        "--specs=nano.specs",
        "--specs=nosys.specs",
        {force = true})
    
    -- 包含路径
    add_includedirs(
        "sdk/Debug",
        "sdk/Core",      
        "sdk/Startup",
        "sdk/Peripheral/inc",
        "sdk/HAL/include",
        "sdk/LIB",
        "bsp/include"
       
    )

     add_includedirs(
        "APP/include",
        "Profile/include",
        "task/inc"
    
    )
    
    -- 宏定义
    add_defines("CH32V20x_D8W")
    
    -- 添加 -fmacro-prefix-map
    on_load(function (target)
        local source_dir = os.projectdir()
        target:add("cxflags", "-fmacro-prefix-map=" .. source_dir .. "=..", {force = true})
    end)
    
     -- 添加文件
    add_files("sdk/Startup/startup_ch32v20x_D8W.S")
    add_files(
        "sdk/Peripheral/src/*.c",
        "sdk/Debug/debug.c",
        "sdk/Core/core_riscv.c",   
        "sdk/LIB/ble_task_scheduler.S",    
        "sdk/HAL/KEY.c",
        "sdk/HAL/LED.c",
        "sdk/HAL/MCU.c",
        "sdk/HAL/RTC.c",
        "sdk/HAL/SLEEP.c" ,
        "bsp/*.c"
    
    )

    add_files(
        "Profile/devinfoservice.c",
        "Profile/gattprofile.c"
        
    )

    add_files(
        "APP/ch32v20x_it.c",
        "APP/peripheral.c",
        "APP/main.c",
        "APP/system_ch32v20x.c"
       
    )
    
       add_files(
        "task/*.c"
       )

    -- 链接库
    add_links("wchble")

    -- 编译后操作，输出到 build 目录
    after_build(function (target)
        local build_dir = target:targetdir()  -- 获取构建目录
        local elf_file = target:targetfile()  -- 获取 elf 文件路径
        local hex_file = path.join(build_dir, "CH32V208GBU_Templete.hex")
        local lst_file = path.join(build_dir, "CH32V208GBU_Templete.lst")
        
        print("生成文件到: " .. build_dir)


        
        -- 生成 hex 文件
        os.exec("%s -O ihex %s %s", 
            TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-objcopy", 
            elf_file, 
            hex_file
        )
        -- 显示大小信息
        os.exec("%s --format=berkeley %s", 
            TOOLCHAIN_FOLDER .. "/bin/riscv-wch-elf-size", 
            elf_file
        )
    end)