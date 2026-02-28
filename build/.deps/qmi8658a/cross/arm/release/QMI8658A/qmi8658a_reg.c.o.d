{
    depfiles_format = "gcc",
    files = {
        [[QMI8658A\qmi8658a_reg.c]]
    },
    depfiles = "qmi8658a_reg.o: QMI8658A\\qmi8658a_reg.c QMI8658A\\qmi8658a_reg.h\
",
    values = {
        [[D:\APP\path\arm-gcc-15.2\bin\arm-none-eabi-gcc]],
        {
            "-std=c11",
            "-IQMI8658A",
            "-DQMI8658A_DRIVER_AVAILABLE",
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