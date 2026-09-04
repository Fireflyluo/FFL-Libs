target("ffl.adhoc")
    set_kind("object")
    set_default(false)
    set_languages("c17", "cxx17")
    add_files("src/*.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra")
    end
