-- SC7A20 New driver build config

target("sc7a20_new")
    set_kind("static")
    set_basename("sc7a20_new")

    add_files("src/sc7a20_core.c", "src/sc7a20_sync.c", "src/sc7a20_async.c")
    add_headerfiles("inc/*.h", "sc7a20_reg.h")

    add_includedirs(".", {public = true})
    add_includedirs("inc", {public = true})

    set_languages("c11")
    add_defines("SC7A20_NEW_DRIVER_AVAILABLE", {public = true})

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
