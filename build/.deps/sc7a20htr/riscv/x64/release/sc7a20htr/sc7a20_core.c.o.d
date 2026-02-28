{
    files = {
        [[sc7a20htr\sc7a20_core.c]]
    },
    depfiles_format = "gcc",
    depfiles = "sc7a20_core.o: sc7a20htr\\sc7a20_core.c sc7a20htr\\inc/sc7a20_core.h  sc7a20htr\\inc/sc7a20_def.h sc7a20htr\\inc/sc7a20_reg.h  sc7a20htr\\inc/sc7a20_hal.h\
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
            "-Isc7a20htr",
            [[-Isc7a20htr\inc]]
        }
    }
}