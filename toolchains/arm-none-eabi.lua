local function _resolve_arm_tool(bindir, toolname)
    local prefix = "arm-none-eabi-"
    local suffixes = {
        ".exe",
        ""
    }

    if not bindir or bindir == "" then
        return prefix .. toolname
    end

    for _, suffix in ipairs(suffixes) do
        local candidate = path.join(bindir, prefix .. toolname .. suffix)
        if os.isfile(candidate) then
            return candidate
        end
    end

    return path.join(bindir, prefix .. toolname)
end

toolchain("arm-none-eabi")
    set_homepage("https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads")
    set_description("Arm GNU Toolchain for bare-metal Cortex-M projects")
    set_kind("standalone")

    on_check(function (toolchain)
        local sdkdir = toolchain:sdkdir() or toolchain:config("sdkdir") or get_config("sdk")
        local bindir = toolchain:bindir() or toolchain:config("bindir") or get_config("bin")

        if (not bindir or bindir == "") and sdkdir and sdkdir ~= "" then
            bindir = path.join(sdkdir, "bin")
        end

        toolchain:config_set("sdkdir", sdkdir)
        toolchain:config_set("bindir", bindir)
        return true
    end)

    on_load(function (toolchain)
        local bindir = toolchain:bindir() or toolchain:config("bindir")

        toolchain:add("toolset", "cc", _resolve_arm_tool(bindir, "gcc"))
        toolchain:add("toolset", "cxx", _resolve_arm_tool(bindir, "g++"))
        toolchain:add("toolset", "as", _resolve_arm_tool(bindir, "gcc"))
        toolchain:add("toolset", "ld", _resolve_arm_tool(bindir, "gcc"))
        toolchain:add("toolset", "sh", _resolve_arm_tool(bindir, "gcc"))
        toolchain:add("toolset", "ar", _resolve_arm_tool(bindir, "ar"))
        toolchain:add("toolset", "ranlib", _resolve_arm_tool(bindir, "ranlib"))
        toolchain:add("toolset", "strip", _resolve_arm_tool(bindir, "strip"))
        toolchain:add("toolset", "objcopy", _resolve_arm_tool(bindir, "objcopy"))
        toolchain:add("toolset", "objdump", _resolve_arm_tool(bindir, "objdump"))
        toolchain:add("toolset", "size", _resolve_arm_tool(bindir, "size"))
        toolchain:add("toolset", "nm", _resolve_arm_tool(bindir, "nm"))

        if bindir and bindir ~= "" and is_host("windows") then
            toolchain:add("runenvs", "PATH", bindir)
        end
    end)
toolchain_end()
