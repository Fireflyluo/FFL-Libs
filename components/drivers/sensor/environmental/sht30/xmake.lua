target("ffl.sht30")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sht30_core.c", "src/sht30_sync.c", "src/sht30_async.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic")

    if is_plat("mingw", "linux", "macosx") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cflags("/W4", "/utf-8")
    end
