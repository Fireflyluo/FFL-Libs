package("adhoc-lib")
    set_description("Ad-Hoc protocol core library")
    set_license("MIT")

    set_sourcedir(path.join(os.scriptdir(), "..", "..", "..", "..", "Lib", "Ad-Hoc-lib"))

    on_load(function (package)
        package:add("links", "adhoc-lib-core")
        package:add("includedirs", "include")
    end)

    on_install(function (package)
        import("package.tools.xmake").install(package)
    end)

    on_test(function (package)
        assert(package:has_cfuncs("adhoc_node_required_size", {includes = "adhoc_api.h"}))
    end)
package_end()
