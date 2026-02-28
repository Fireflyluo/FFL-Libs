{
    depfiles_format = "gcc",
    files = {
        [[ICM42688P\icm42688p.c]]
    },
    depfiles = "icm42688p.o: ICM42688P\\icm42688p.c ICM42688P\\icm42688p.h  ICM42688P\\icm42688_reg.h\
",
    values = {
        [[D:\APP\path\arm-gcc-15.2\bin\arm-none-eabi-gcc]],
        {
            "-std=c11",
            "-IICM42688P",
            "-DICM42688P_DRIVER_AVAILABLE",
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