package("acc-lib")
    set_description("Accelerometer category library (SC7A20)")
    set_license("MIT")

    add_configs("sc7a20", {description = "Enable SC7A20 driver", default = false, type = "boolean"})

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "ACC-lib"))

    on_load(function (package)
        package:add("links", "acc-lib")
        package:add("includedirs", "include")
        package:add("includedirs", "drivers/sc7a20/include")
    end)

    on_install(function (package)
        local configs = {
            "--acc_sc7a20=" .. (package:config("sc7a20") and "y" or "n")
        }
        import("package.tools.xmake").install(package, configs)
    end)

    on_test(function (package)
        if package:config("sc7a20") then
            assert(package:has_cfuncs("sc7a20_init", {includes = "sc7a20.h"}))
        end
    end)
package_end()
