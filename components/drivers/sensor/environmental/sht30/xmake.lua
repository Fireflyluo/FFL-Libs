target("ffl.sht30")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sht30_core.c", "src/sht30_sync.c", "src/sht30_async.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end

target("ffl.sht30.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/sht30_crc_test.c")
    add_deps("ffl.sht30")
    add_tests("default")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end
