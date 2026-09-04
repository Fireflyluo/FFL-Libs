set_project("fireflyluo-embedded-libs")
set_version("2.0.0")
set_xmakever("2.5.9")
set_languages("c11", "cxx17")

includes("components/xmake.lua")

target("components")
    set_kind("phony")
    set_default(true)
    on_run(function ()
        print("fireflyluo embedded components")
        print("Use an explicit ffl.<component> target or add_deps() from a consuming project.")
    end)
