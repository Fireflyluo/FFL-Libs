set_languages("c11", "cxx17")

option("demo_accel_fifo")
    set_default(false)
    set_showmenu(true)
    set_description("Enable FIFO in the cropped source-driver target")
option_end()

target("ffl-demo-accel-full")
    set_kind("static")
    set_basename("ffl-demo-accel-full")

    add_files(
        "src/demo_accel_core.c",
        "src/demo_accel_fifo.c"
    )
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_defines("DEMO_ACCEL_FIFO_ENABLED=1", {public = true})

target("ffl-demo-accel-source")
    set_kind("object")
    set_default(false)

    add_files("src/demo_accel_core.c")
    if has_config("demo_accel_fifo") then
        add_files("src/demo_accel_fifo.c")
        add_defines("DEMO_ACCEL_FIFO_ENABLED=1", {public = true})
    else
        add_defines("DEMO_ACCEL_FIFO_ENABLED=0", {public = true})
    end

    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
