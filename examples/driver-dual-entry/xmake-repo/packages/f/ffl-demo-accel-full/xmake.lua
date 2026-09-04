package("ffl-demo-accel-full")
    set_description("Complete static library for the dual-entry driver example")
    set_license("MIT")

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "component"))

    on_load(function (package)
        package:add("links", "ffl-demo-accel-full")
        package:add("includedirs", "include")
    end)

    on_install(function (package)
        import("package.tools.xmake").install(package)
    end)

    on_test(function (package)
        assert(package:has_cfuncs("demo_accel_read_mg", {includes = "demo_accel.h"}))
    end)
package_end()
