set_project("fireflyluo-embedded-libs")
set_version("2.0.0")
set_xmakever("2.5.9")
set_languages("c11", "cxx17")

-- Host validation uses GCC via MinGW. MCU consumers explicitly select a
-- GNU cross toolchain with `-p cross --toolchain=<name>`.
if not is_plat("cross") then
    set_plat("mingw")
    set_arch("x86_64")
    set_toolchains("mingw")
end

includes("toolchains/arm-none-eabi.lua")
includes("toolchains/wch-riscv.lua")
includes("components/xmake.lua")

target("components")
    set_kind("phony")
    set_default(true)
    on_run(function ()
        print("fireflyluo embedded components")
        print("Use an explicit ffl.<component> target or add_deps() from a consuming project.")
    end)
