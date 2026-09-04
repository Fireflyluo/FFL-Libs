target("ffl.sht40")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sht40_core.c", "src/sht40_sync.c", "src/sht40_async.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end

target("ffl.sht40.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/sht40_crc_test.c")
    add_deps("ffl.sht40")
    add_tests("default")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end
