add_rules("mode.debug", "mode.release")
set_languages("c11")

option("th_sht40")
    set_default(false)
    set_showmenu(true)
    set_description("Enable SHT40 temperature/humidity driver")
option_end()

option("th_sht30")
    set_default(false)
    set_showmenu(true)
    set_description("Enable SHT30 temperature/humidity driver")
option_end()

target("th-lib")
    set_kind("static")
    set_basename("th-lib")

    if has_config("th_sht40") then
        add_files(
            "drivers/sht40/src/sht40_core.c",
            "drivers/sht40/src/sht40_sync.c",
            "drivers/sht40/src/sht40_async.c"
        )
        add_defines("TH_DRIVER_SHT40", {public = true})
    end

    if has_config("th_sht30") then
        add_files(
            "drivers/sht30/src/sht30_core.c",
            "drivers/sht30/src/sht30_sync.c",
            "drivers/sht30/src/sht30_async.c"
        )
        add_defines("TH_DRIVER_SHT30", {public = true})
    end

    add_headerfiles(
        "include/*.h",
        "drivers/sht40/include/*.h",
        "drivers/sht30/include/*.h"
    )

    add_includedirs(
        "include",
        "drivers/sht40/include",
        "drivers/sht30/include",
        {public = true}
    )

    add_defines("TH_LIB_AVAILABLE", {public = true})

    on_load(function (target)
        if not has_config("th_sht40") and not has_config("th_sht30") then
            raise("th-lib: no driver enabled, set th_sht40=y or th_sht30=y")
        end
    end)

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
