set_project("driver-dual-entry-source-example")
set_languages("c11", "cxx17")

includes("component")

target("source-trimmed-app")
    set_kind("binary")
    add_files("source-trimmed/main.c")
    add_deps("ffl-demo-accel-source")
