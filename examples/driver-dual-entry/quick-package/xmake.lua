set_project("driver-dual-entry-package-example")
set_languages("c11")

add_repositories("driver-dual-entry " .. path.join(os.scriptdir(), "..", "xmake-repo"))
add_requires("ffl-demo-accel-full")

target("quick-package-app")
    set_kind("binary")
    add_files("main.c")
    add_packages("ffl-demo-accel-full")
