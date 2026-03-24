set_project("xrepo-example")
set_version("0.1.0")
set_languages("c11")

-- Local repository inside this repo
add_repositories("embedded-sensors " .. path.join(os.scriptdir(), "..", "..", "xmake-repo"))

-- This example is platform-agnostic; choose your toolchain as needed:
--   xmake f -p cross -a arm --toolchain=gnu-rm
--   xmake f -p riscv --toolchain=riscv-wch-gcc

add_requires("embedded-sensor-drivers", {
    configs = {
        sensor_icm42688p = true,
        sensor_qmi8658a = true,
        sensor_sc7a20htr = true,
        sensor_sht40 = true,
        sensor_oled = false
    }
})

target("app")
    set_kind("binary")
    add_files("src/*.c")
    add_packages("embedded-sensor-drivers")
