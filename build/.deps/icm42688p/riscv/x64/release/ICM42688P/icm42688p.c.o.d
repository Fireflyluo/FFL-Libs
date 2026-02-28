{
    files = {
        [[ICM42688P\icm42688p.c]]
    },
    depfiles_format = "gcc",
    depfiles = "icm42688p.o: ICM42688P\\icm42688p.c ICM42688P\\icm42688p.h  ICM42688P\\icm42688_reg.h\
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
            "-IICM42688P"
        }
    }
}