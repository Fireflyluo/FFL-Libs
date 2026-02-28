{
    depfiles_format = "gcc",
    files = {
        [[sc7a20htr\sc7a20_core.c]]
    },
    depfiles = "sc7a20_core.o: sc7a20htr\\sc7a20_core.c sc7a20htr\\inc/sc7a20_core.h  sc7a20htr\\inc/sc7a20_def.h sc7a20htr\\inc/sc7a20_reg.h  sc7a20htr\\inc/sc7a20_hal.h\
",
    values = {
        [[D:\APP\path\arm-gcc-15.2\bin\arm-none-eabi-gcc]],
        {
            "-std=c11",
            "-Isc7a20htr",
            [[-Isc7a20htr\inc]],
            "-DSC7A20HTR_DRIVER_AVAILABLE",
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