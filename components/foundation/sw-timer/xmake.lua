target("ffl.sw_timer")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sw_timer.c")
    add_headerfiles("include/sw_timer.h")
    add_includedirs("include", {public = true})

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end

target("ffl.sw_timer.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/sw_timer_test.cpp")
    add_deps("ffl.sw_timer")
    add_tests("default")

    if not is_plat("windows") then
        add_cxxflags("-Wall", "-Wextra", "-Werror")
    end
