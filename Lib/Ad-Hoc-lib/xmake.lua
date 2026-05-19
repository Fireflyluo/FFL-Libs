add_rules("mode.debug", "mode.release")
set_languages("gnu17")

add_cflags(
    "-ffunction-sections",
    "-fdata-sections",
    "-fno-common",
    "-Wno-unused-parameter",
    {force = true}
)

if is_mode("debug") then
    add_cflags("-g", "-Og", {force = true})
else
    add_cflags("-Os", {force = true})
end

target("adhoc-lib-core")
    set_kind("static")
    set_group("libs")

    add_files(
        "src/adhoc_node.c",
        "src/adhoc_frame.c",
        "src/adhoc_crc8.c",
        "src/adhoc_timing.c",
        "src/adhoc_sm.c",
        "src/adhoc_reply_list.c",
        "src/adhoc_data_plane.c"
    )

    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
