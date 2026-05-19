local function _find_wch_cross_prefix(bindir)
    local prefixes = {
        "riscv32-wch-elf-",
        "riscv-wch-elf-"
    }

    local gcc_names = {
        "gcc.exe",
        "gcc"
    }

    local prefix
    local gcc_name

    if not bindir or bindir == "" then
        return nil
    end

    for _, prefix in ipairs(prefixes) do
        for _, gcc_name in ipairs(gcc_names) do
            if os.isfile(path.join(bindir, prefix .. gcc_name)) then
                return prefix
            end
        end
    end

    return nil
end

local function _resolve_wch_tool(bindir, cross, toolname)
    local suffixes = {
        ".exe",
        ""
    }
    local suffix
    local candidate

    if not bindir or bindir == "" or not cross or cross == "" then
        return nil
    end

    for _, suffix in ipairs(suffixes) do
        candidate = path.join(bindir, cross .. toolname .. suffix)
        if os.isfile(candidate) then
            return candidate
        end
    end

    return path.join(bindir, cross .. toolname)
end

toolchain("wch-riscv")
    set_homepage("https://www.wch.cn/")
    set_description("WCH RISC-V Embedded GCC toolchain")
    set_kind("standalone")

    on_check(function (toolchain)
        local sdkdir = toolchain:sdkdir() or toolchain:config("sdkdir") or get_config("sdk")
        local bindir = toolchain:bindir() or toolchain:config("bindir") or get_config("bin")
        local cross = toolchain:cross() or toolchain:config("cross") or get_config("cross")

        if (not bindir or bindir == "") and sdkdir and sdkdir ~= "" then
            bindir = path.join(sdkdir, "bin")
        end

        if (not cross or cross == "") and bindir and bindir ~= "" then
            cross = _find_wch_cross_prefix(bindir)
        end

        if not bindir or bindir == "" or not cross or cross == "" then
            raise("WCH RISC-V toolchain not found, please set sdkdir/bindir or --sdk/--bin for wch-riscv")
        end

        toolchain:config_set("sdkdir", sdkdir)
        toolchain:config_set("bindir", bindir)
        toolchain:config_set("cross", cross)

        return true
    end)

    on_load(function (toolchain)
        local common_arch_flags = {
            "-march=rv32imacxw",
            "-mabi=ilp32",
            "-msmall-data-limit=8",
            "-msave-restore",
            "-fmessage-length=0",
            "-fsigned-char"
        }
        local bindir = toolchain:bindir()
        local cross = toolchain:cross()

        toolchain:add("toolset", "cc", _resolve_wch_tool(bindir, cross, "gcc"))
        toolchain:add("toolset", "cxx", _resolve_wch_tool(bindir, cross, "g++"))
        toolchain:add("toolset", "as", _resolve_wch_tool(bindir, cross, "gcc"))
        toolchain:add("toolset", "ld", _resolve_wch_tool(bindir, cross, "gcc"))
        toolchain:add("toolset", "sh", _resolve_wch_tool(bindir, cross, "gcc"))
        toolchain:add("toolset", "ar", _resolve_wch_tool(bindir, cross, "ar"))
        toolchain:add("toolset", "ranlib", _resolve_wch_tool(bindir, cross, "ranlib"))
        toolchain:add("toolset", "strip", _resolve_wch_tool(bindir, cross, "strip"))
        toolchain:add("toolset", "objcopy", _resolve_wch_tool(bindir, cross, "objcopy"))
        toolchain:add("toolset", "objdump", _resolve_wch_tool(bindir, cross, "objdump"))
        toolchain:add("toolset", "size", _resolve_wch_tool(bindir, cross, "size"))
        toolchain:add("toolset", "nm", _resolve_wch_tool(bindir, cross, "nm"))

        if bindir and bindir ~= "" and is_host("windows") then
            toolchain:add("runenvs", "PATH", bindir)
        end

        toolchain:add("cxflags", table.unpack(common_arch_flags))
        toolchain:add("asflags", table.unpack(common_arch_flags))
        toolchain:add("ldflags", "-march=rv32imacxw", "-mabi=ilp32", "-msmall-data-limit=8", "-msave-restore")
        toolchain:add("shflags", "-march=rv32imacxw", "-mabi=ilp32", "-msmall-data-limit=8", "-msave-restore")
    end)
toolchain_end()
