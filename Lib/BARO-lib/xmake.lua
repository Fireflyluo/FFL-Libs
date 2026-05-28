add_rules("mode.debug", "mode.release")
set_languages("c11")

option("baro_icp20100")
    set_default(false)
    set_showmenu(true)
    set_description("Enable ICP20100 barometer driver")
option_end()

target("baro-lib")
    set_kind("static")
    set_basename("baro-lib")

    if has_config("baro_icp20100") then
        add_files("drivers/icp20100/src/icp20100.c")
        add_defines("BARO_DRIVER_ICP20100", {public = true})
    end

    add_headerfiles(
        "include/*.h",
        "drivers/icp20100/include/*.h"
    )

    add_includedirs(
        "include",
        "drivers/icp20100/include",
        {public = true}
    )

    add_defines("BARO_LIB_AVAILABLE", {public = true})

    on_load(function (target)
        if not has_config("baro_icp20100") then
            raise("baro-lib: no driver enabled, set baro_icp20100=y")
        end
    end)

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
