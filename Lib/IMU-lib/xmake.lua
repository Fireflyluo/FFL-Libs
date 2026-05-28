add_rules("mode.debug", "mode.release")
set_languages("c11")

option("imu_qmi8658a")
    set_default(false)
    set_showmenu(true)
    set_description("Enable QMI8658A driver")
option_end()

option("imu_icm42688p")
    set_default(false)
    set_showmenu(true)
    set_description("Enable ICM42688P driver")
option_end()

target("imu-lib")
    set_kind("static")
    set_basename("imu-lib")

    if has_config("imu_qmi8658a") then
        add_files("drivers/qmi8658a/src/imu_qmi8658a.c")
        add_defines("IMU_DRIVER_QMI8658A", {public = true})
    end

    if has_config("imu_icm42688p") then
        add_files("drivers/icm42688p/src/imu_icm42688p.c")
        add_defines("IMU_DRIVER_ICM42688P", {public = true})
    end
    add_headerfiles(
        "include/*.h",
        "drivers/qmi8658a/include/*.h",
        "drivers/icm42688p/include/*.h"
    )

    add_includedirs(
        "include",
        "drivers/qmi8658a/include",
        "drivers/icm42688p/include",
        {public = true}
    )

    add_defines("IMU_LIB_AVAILABLE", {public = true})

    on_load(function (target)
        if not has_config("imu_qmi8658a") and not has_config("imu_icm42688p") then
            raise("imu-lib: no driver enabled, set at least one of imu_qmi8658a/imu_icm42688p")
        end
    end)

    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
