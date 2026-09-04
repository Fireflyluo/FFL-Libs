target("ffl.sw_timer")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sw_timer.c")
    add_headerfiles("include/sw_timer.h")
    add_includedirs("include", {public = true})

    if is_plat("mingw", "linux", "macosx") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cflags("/W4", "/utf-8")
    end

target("ffl.sw_timer.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/sw_timer_test.cpp")
    add_deps("ffl.sw_timer")
    add_tests("default")

    if is_plat("mingw", "linux", "macosx") then
        add_cxxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxxflags("/W4", "/utf-8")
    end
