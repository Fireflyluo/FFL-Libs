target("ffl.osal")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/osal.c", "src/osal_event.c", "src/osal_memory.c", "src/osal_msg.c", "src/osal_timer.c", "src/osal_pt.c", "src/osal_port.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.protothreads")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end
