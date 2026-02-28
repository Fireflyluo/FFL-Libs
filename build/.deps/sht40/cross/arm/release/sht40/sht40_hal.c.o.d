{
    depfiles_format = "gcc",
    files = {
        [[sht40\sht40_hal.c]]
    },
    depfiles = "sht40_hal.o: sht40\\sht40_hal.c sht40\\sht40_hal.h\
",
    values = {
        [[D:\APP\path\arm-gcc-15.2\bin\arm-none-eabi-gcc]],
        {
            "-std=c11",
            "-Isht40",
            "-DSHT40_DRIVER_AVAILABLE",
            "-mcpu=cortex-m4",
            "-mthumb",
            "-mfpu=fpv4-sp-d16",
            "-mfloat-abi=hard",
            "-Wall",
            "-Wextra",
            "-ffunction-sections",
            "-fdata-sections"
        }
    }
}