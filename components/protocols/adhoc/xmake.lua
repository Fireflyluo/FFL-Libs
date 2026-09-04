target("ffl.adhoc")
    set_kind("object")
    set_default(false)
    set_languages("c17", "cxx17")
    add_files("src/*.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})

    if is_plat("mingw", "linux", "macosx") then
        add_cflags("-Wall", "-Wextra")
    elseif is_plat("windows") then
        add_cflags("/W4", "/utf-8")
    end
