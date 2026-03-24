-- SHT40 New driver build config

target("sht40_new")
    set_kind("static")
    set_basename("sht40_new")

    add_files("src/sht40_core.c", "src/sht40_sync.c", "src/sht40_async.c")
    add_headerfiles("inc/*.h")

    add_includedirs(".", {public = true})
    add_includedirs("inc", {public = true})

    set_languages("c11")
    add_defines("SHT40_NEW_DRIVER_AVAILABLE", {public = true})

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
