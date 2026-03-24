package("embedded-sensor-drivers")
    set_description("Embedded sensor drivers (ICM42688P, QMI8658A, SC7A20HTR, SHT40, OLED)")
    set_license("MIT")

    add_configs("sensor_icm42688p", {description = "Enable ICM42688P driver", default = true, type = "boolean"})
    add_configs("sensor_qmi8658a", {description = "Enable QMI8658A driver", default = true, type = "boolean"})
    add_configs("sensor_sc7a20htr", {description = "Enable SC7A20HTR driver", default = true, type = "boolean"})
    add_configs("sensor_sht40", {description = "Enable SHT40 driver", default = true, type = "boolean"})
    add_configs("sensor_oled", {description = "Enable OLED driver", default = true, type = "boolean"})

    -- Repository layout: <project>/xmake-repo/packages/e/embedded-sensor-drivers/xmake.lua
    -- Use the project root as the source directory.
    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", ".."))

    on_load(function (package)
        -- Export only enabled libraries and include dirs
        if package:config("sensor_icm42688p") then
            package:add("links", "icm42688p")
            package:add("includedirs", "sensor/ICM42688P")
        end
        if package:config("sensor_qmi8658a") then
            package:add("links", "qmi8658a")
            package:add("includedirs", "sensor/QMI8658A")
        end
        if package:config("sensor_sc7a20htr") then
            package:add("links", "sc7a20htr")
            package:add("includedirs", "sensor/sc7a20htr/inc")
            package:add("includedirs", "sensor/sc7a20htr/platform")
        end
        if package:config("sensor_sht40") then
            package:add("links", "sht40")
            package:add("includedirs", "sensor/sht40")
        end
        if package:config("sensor_oled") then
            package:add("links", "oled")
            package:add("includedirs", "sensor/oled")
        end

        -- Keep the default include root in case some headers use repo-relative paths
        package:add("includedirs", ".")
    end)

    on_install(function (package)
        local configs = {
            "--sensor_icm42688p=" .. (package:config("sensor_icm42688p") and "y" or "n"),
            "--sensor_qmi8658a=" .. (package:config("sensor_qmi8658a") and "y" or "n"),
            "--sensor_sc7a20htr=" .. (package:config("sensor_sc7a20htr") and "y" or "n"),
            "--sensor_sht40=" .. (package:config("sensor_sht40") and "y" or "n"),
            "--sensor_oled=" .. (package:config("sensor_oled") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)
package_end()
