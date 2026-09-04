package("ffl-sc7a20")
    set_description("Complete static SC7A20 driver core with no board-specific HAL")
    set_license("MIT")

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", ".."))

    on_load(function (package)
        package:add("links", "ffl-sc7a20")
        package:add("includedirs", "include")
    end)

    on_install(function (package)
        import("package.tools.xmake").install(package, {}, {target = "ffl-sc7a20"})
    end)

    on_test(function (package)
        assert(package:has_cfuncs("sc7a20_init", {includes = "sc7a20.h"}))
    end)
package_end()
