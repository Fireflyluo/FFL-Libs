{
    files = {
        [[QMI8658A\qmi8658a_driver.c]]
    },
    depfiles_format = "gcc",
    depfiles = "qmi8658a_driver.o: QMI8658A\\qmi8658a_driver.c QMI8658A\\qmi8658a_driver.h  QMI8658A\\qmi8658a_reg.h QMI8658A\\gyro_device.h\
",
    values = {
        [[E:\APP\MRS2\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC12\bin\riscv-wch-elf-gcc]],
        {
            "-march=rv32imac",
            "-mabi=ilp32",
            "-mcmodel=medany",
            "-msmall-data-limit=8",
            "-mno-save-restore",
            "-fno-common",
            "-ffunction-sections",
            "-fdata-sections",
            "-fno-builtin",
            "-fno-hosted",
            "-Wall",
            "-Werror=return-type",
            "-std=c11",
            "-IQMI8658A"
        }
    }
}