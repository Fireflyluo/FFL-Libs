target("ffl.icm42688p")
    set_kind("object")
    set_default(false)
    set_languages("c11", "cxx17")
    add_files("src/imu_icm42688p.c")
    add_headerfiles("include/*.h")
    add_includedirs("include", {public = true})
    add_deps("ffl.imu_common")

    if is_plat("mingw", "linux", "macosx") then
        add_cflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cflags("/W4", "/utf-8")
    end
