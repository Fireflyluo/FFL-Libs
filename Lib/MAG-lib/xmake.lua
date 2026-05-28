add_rules("mode.debug", "mode.release")
set_languages("c11")

option("mag_qmc5883p")
    set_default(false)
    set_showmenu(true)
    set_description("Enable QMC5883P magnetometer driver")
option_end()

target("mag-lib")
    set_kind("static")
    set_basename("mag-lib")

    if has_config("mag_qmc5883p") then
        add_files("drivers/qmc5883p/src/qmc5883p.c")
        add_defines("MAG_DRIVER_QMC5883P", {public = true})
    end

    add_headerfiles(
        "include/*.h",
        "drivers/qmc5883p/include/*.h"
    )

    add_includedirs(
        "include",
        "drivers/qmc5883p/include",
        {public = true}
    )

    add_defines("MAG_LIB_AVAILABLE", {public = true})

    on_load(function (target)
        if not has_config("mag_qmc5883p") then
            raise("mag-lib: no driver enabled, set mag_qmc5883p=y")
        end
    end)

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
