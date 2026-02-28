{
    depfiles_format = "gcc",
    files = {
        [[oled\OLED_Data.c]]
    },
    depfiles = "OLED_Data.o: oled\\OLED_Data.c oled\\OLED_Data.h\
",
    values = {
        [[D:\APP\path\arm-gcc-15.2\bin\arm-none-eabi-gcc]],
        {
            "-std=c11",
            "-Ioled",
            "-DOLED_DRIVER_AVAILABLE",
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