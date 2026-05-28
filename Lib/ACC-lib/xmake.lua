add_rules("mode.debug", "mode.release")
set_languages("c11")

option("acc_sc7a20")
    set_default(false)
    set_showmenu(true)
    set_description("Enable SC7A20 accelerometer driver")
option_end()

target("acc-lib")
    set_kind("static")
    set_basename("acc-lib")

    if has_config("acc_sc7a20") then
        add_files(
            "drivers/sc7a20/src/sc7a20_core.c",
            "drivers/sc7a20/src/sc7a20_sync.c",
            "drivers/sc7a20/src/sc7a20_async.c"
        )
        add_defines("ACC_DRIVER_SC7A20", {public = true})
    end

    add_headerfiles(
        "include/*.h",
        "drivers/sc7a20/include/*.h"
    )

    add_includedirs(
        "include",
        "drivers/sc7a20/include",
        {public = true}
    )

    add_defines("ACC_LIB_AVAILABLE", {public = true})

    on_load(function (target)
        if not has_config("acc_sc7a20") then
            raise("acc-lib: no driver enabled, set acc_sc7a20=y")
        end
    end)

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
