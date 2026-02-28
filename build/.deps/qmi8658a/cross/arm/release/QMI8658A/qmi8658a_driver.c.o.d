{
    depfiles_format = "gcc",
    files = {
        [[QMI8658A\qmi8658a_driver.c]]
    },
    depfiles = "qmi8658a_driver.o: QMI8658A\\qmi8658a_driver.c QMI8658A\\qmi8658a_driver.h  QMI8658A\\qmi8658a_reg.h QMI8658A\\gyro_device.h\
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