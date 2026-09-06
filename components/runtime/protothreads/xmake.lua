target("ffl.protothreads")
    set_kind("headeronly")
    set_default(false)
    set_languages("c11", "cxx17")
    add_headerfiles("include/protothreads.h")
    add_includedirs("include", {public = true})

target("ffl.protothreads.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/protothreads_test.c")
    add_deps("ffl.protothreads")
    add_tests("default")

    if not is_plat("cross") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end

target("ffl.protothreads.cxx-test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/protothreads_cxx_test.cpp")
    add_deps("ffl.protothreads")
    add_tests("default")

    if not is_plat("cross") then
        add_cxxflags("-Wall", "-Wextra", "-Werror")
    end
