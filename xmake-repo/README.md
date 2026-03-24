# Local Xmake Repository

This directory is a local xmake package repository. It exposes the package:

- embedded-sensor-drivers

## Use in another project

1) Add the repository path

```lua
add_repositories("embedded-sensors path/to/fireflyluo-Embedded-Libs/xmake-repo")
```

2) Require the package (customize drivers)

```lua
add_requires("embedded-sensor-drivers", {
    configs = {
        sensor_icm42688p = true,
        sensor_qmi8658a = true,
        sensor_sc7a20htr = true,
        sensor_sc7a20_new = true,
        sensor_sht40 = true,
        sensor_sht40_new = true,
        sensor_oled = false,
        utils_ringbuffer = true,
        utils_sw_timer = true
    }
})
```

3) Link to your target

```lua
target("app")
    set_kind("binary")
    add_files("src/*.c")
    add_packages("embedded-sensor-drivers")
```
