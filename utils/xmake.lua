-- Utility modules build config

if has_config("utils_ringbuffer") then
    target("ringbuffer")
        set_kind("static")
        set_basename("ringbuffer")
        add_files("ringbuffer.c")
        add_headerfiles("ringbuffer.h")
        add_includedirs(".", {public = true})
        set_languages("c11")
        add_defines("RINGBUFFER_AVAILABLE", {public = true})

        if is_plat("mingw", "linux", "macosx") then
            add_cxflags("-Wall", "-Wextra", "-Werror")
        elseif is_plat("windows") then
            add_cxflags("/W4", "/utf-8")
        end
end

if has_config("utils_sw_timer") then
    target("sw_timer")
        set_kind("static")
        set_basename("sw_timer")
        add_files("sw_timer.c")
        add_headerfiles("sw_timer.h")
        add_includedirs(".", {public = true})
        set_languages("c11")
        add_defines("SW_TIMER_AVAILABLE", {public = true})

        if is_plat("mingw", "linux", "macosx") then
            add_cxflags("-Wall", "-Wextra", "-Werror")
        elseif is_plat("windows") then
            add_cxflags("/W4", "/utf-8")
        end
end
