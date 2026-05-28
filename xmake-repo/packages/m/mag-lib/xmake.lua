package("mag-lib")
    set_description("Magnetometer category library (QMC5883P)")
    set_license("MIT")

    add_configs("qmc5883p", {description = "Enable QMC5883P driver", default = false, type = "boolean"})

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "MAG-lib"))

    on_load(function (package)
        package:add("links", "mag-lib")
        package:add("includedirs", "include")
        package:add("includedirs", "drivers/qmc5883p/include")
    end)

    on_install(function (package)
        local configs = {
            "--mag_qmc5883p=" .. (package:config("qmc5883p") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)

    on_test(function (package)
        if package:config("qmc5883p") then
            assert(package:has_cfuncs("qmc5883p_init", {includes = "qmc5883p.h"}))
        end
    end)
package_end()
