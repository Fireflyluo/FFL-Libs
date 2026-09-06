target("ffl.sw_timer")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sw_timer.c")
    add_headerfiles("include/(ffl/sw_timer.h)")
    add_includedirs("include", {public = true})

    add_cflags("-Wall", "-Wextra", "-Werror")

target("ffl.sw_timer.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/sw_timer_test.cpp")
    add_deps("ffl.sw_timer")
    add_tests("default")

    add_cxxflags("-Wall", "-Wextra", "-Werror")
