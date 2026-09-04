target("ffl.icp20100")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/icp20100.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end
