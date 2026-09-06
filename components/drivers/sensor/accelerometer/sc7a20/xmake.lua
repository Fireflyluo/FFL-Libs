option("sc7a20_async")
    set_default(true)
    set_showmenu(true)
    set_description("Compile asynchronous SC7A20 API support")
option_end()

target("ffl.sc7a20")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sc7a20_core.c", "src/sc7a20_sync.c", "src/ffl_sc7a20.c")
    add_headerfiles("include/*.h", "include/(ffl/*.h)")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic", "ffl.driver_port")

    if has_config("sc7a20_async") then
        add_files("src/sc7a20_async.c")
        add_defines("FFL_SC7A20_ASYNC_ENABLED=1", {public = true})
    else
        add_defines("FFL_SC7A20_ASYNC_ENABLED=0", {public = true})
    end

    add_cflags("-Wall", "-Wextra", "-Werror")

target("ffl.sc7a20.full")
    set_kind("static")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sc7a20_core.c", "src/sc7a20_sync.c", "src/sc7a20_async.c", "src/ffl_sc7a20.c")
    add_headerfiles("include/*.h", "include/(ffl/*.h)")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic", "ffl.driver_port")
    add_defines("FFL_SC7A20_ASYNC_ENABLED=1", {public = true})

    add_cflags("-Wall", "-Wextra", "-Werror")

target("ffl-sc7a20")
    set_kind("static")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/sc7a20_core.c", "src/sc7a20_sync.c", "src/sc7a20_async.c", "src/ffl_sc7a20.c")
    add_headerfiles("include/*.h", "include/(ffl/*.h)")
    add_includedirs("include", {public = true})
    add_deps("ffl.atomic", "ffl.driver_port")
    add_defines("FFL_SC7A20_ASYNC_ENABLED=1", {public = true})

    add_cflags("-Wall", "-Wextra", "-Werror")

target("ffl.sc7a20.unit")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/unit_test.c", "test/mock_adapter.c")
    add_deps("ffl.sc7a20")
    add_tests("default")

if has_config("sc7a20_async") then
    target("ffl.sc7a20.integration")
        set_kind("binary")
        set_default(false)
        set_languages("c11", "cxx17")
        add_files("test/integration_test.c", "test/mock_adapter.c")
        add_deps("ffl.sc7a20")
        add_tests("default")
end

target("ffl.sc7a20.test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/ffl_sc7a20_test.c")
    add_deps("ffl.sc7a20")
    add_tests("default")
    add_cflags("-Wall", "-Wextra", "-Werror")

target("ffl.sc7a20.cxx-test")
    set_kind("binary")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("test/ffl_sc7a20_cxx_compile.cpp")
    add_deps("ffl.sc7a20")
    add_tests("default")
    add_cxxflags("-Wall", "-Wextra", "-Werror")
