target("ffl.ringbuffer")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/ringbuffer.c")
    add_headerfiles("include/ringbuffer.h")
    add_includedirs("include", {public = true})

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end

target("ffl.ringbuffer.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/ringbuffer_test.cpp")
    add_deps("ffl.ringbuffer")
    add_tests("default")

    if not is_plat("windows") then
        add_cxxflags("-Wall", "-Wextra", "-Werror")
    end
