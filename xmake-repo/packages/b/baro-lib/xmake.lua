package("baro-lib")
    set_description("Barometer category library (ICP20100)")
    set_license("MIT")

    add_configs("icp20100", {description = "Enable ICP20100 driver", default = false, type = "boolean"})

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "BARO-lib"))

    on_load(function (package)
        package:add("links", "baro-lib")
        package:add("includedirs", "include")
        package:add("includedirs", "drivers/icp20100/include")
    end)

    on_install(function (package)
        local configs = {
            "--baro_icp20100=" .. (package:config("icp20100") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)

    on_test(function (package)
        if package:config("icp20100") then
            assert(package:has_cfuncs("icp20100_init", {includes = "icp20100.h"}))
        end
    end)
package_end()
