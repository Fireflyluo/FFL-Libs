target("ffl.qmi8658a")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/imu_qmi8658a.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.imu_common")

    if not is_plat("windows") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    end
