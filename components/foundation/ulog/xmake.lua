target("ffl.ulog")
    set_kind("static")
    set_default(false)
    set_languages("c11")
    add_files("src/ulog.c")
    add_headerfiles("include/ulog.h")
    add_includedirs("include", {public = true})
    -- MCU 默认可裁剪；应用可再 add_defines 覆盖
    add_defines("ULOG_ENABLE=1")
