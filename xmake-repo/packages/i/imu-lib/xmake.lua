package("imu-lib")
    set_description("Portable IMU library for QMI8658A and ICM42688P")
    set_license("MIT")

    add_configs("qmi8658a", {description = "Enable QMI8658A driver", default = false, type = "boolean"})
    add_configs("icm42688p", {description = "Enable ICM42688P driver", default = false, type = "boolean"})

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "IMU-lib"))

    on_load(function (package)
        package:add("links", "imu-lib")
        package:add("includedirs", "include")
        package:add("includedirs", "drivers/qmi8658a/include")
        package:add("includedirs", "drivers/icm42688p/include")
    end)

    on_install(function (package)
        local configs = {
            "--imu_qmi8658a=" .. (package:config("qmi8658a") and "y" or "n"),
            "--imu_icm42688p=" .. (package:config("icm42688p") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)

    on_test(function (package)
        if package:config("qmi8658a") then
            assert(package:has_cfuncs("imu_qmi8658a_init", {includes = "imu_qmi8658a.h"}))
        end
        if package:config("icm42688p") then
            assert(package:has_cfuncs("imu_icm42688p_init", {includes = "imu_icm42688p.h"}))
        end
    end)
package_end()
