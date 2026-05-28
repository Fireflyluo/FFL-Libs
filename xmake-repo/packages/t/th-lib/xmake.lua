package("th-lib")
    set_description("Temperature/Humidity category library (SHT40/SHT30)")
    set_license("MIT")

    add_configs("sht40", {description = "Enable SHT40 driver", default = false, type = "boolean"})
    add_configs("sht30", {description = "Enable SHT30 driver", default = false, type = "boolean"})

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "TH-lib"))

    on_load(function (package)
        package:add("links", "th-lib")
        package:add("includedirs", "include")
        if package:config("sht40") then
            package:add("includedirs", "drivers/sht40/include")
        end
        if package:config("sht30") then
            package:add("includedirs", "drivers/sht30/include")
        end
    end)

    on_install(function (package)
        local configs = {
            "--th_sht40=" .. (package:config("sht40") and "y" or "n"),
            "--th_sht30=" .. (package:config("sht30") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)

    on_test(function (package)
        if package:config("sht40") then
            assert(package:has_cfuncs("sht40_init", {includes = "sht40.h"}))
        end
        if package:config("sht30") then
            assert(package:has_cfuncs("sht30_init", {includes = "sht30.h"}))
        end
    end)
package_end()
