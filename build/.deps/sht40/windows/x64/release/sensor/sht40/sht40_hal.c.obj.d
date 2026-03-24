{
    depfiles_format = "cl_json",
    files = {
        [[sensor\sht40\sht40_hal.c]]
    },
    values = {
        [[C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Tools\MSVC\14.50.35717\bin\HostX64\x64\cl.exe]],
        {
            "-nologo",
            "-MD",
            "-std:c11",
            [[-Isensor\sht40]],
            "-DSHT40_DRIVER_AVAILABLE",
            "/W4",
            "/utf-8"
        }
    },
    depfiles = "{\
    \"Version\": \"1.2\",\
    \"Data\": {\
        \"Source\": \"d:\\\\mcu\\\\0.fireflyluo-embedded-libs-main\\\\sensor\\\\sht40\\\\sht40_hal.c\",\
        \"ProvidedModule\": \"\",\
        \"Includes\": [\
            \"d:\\\\mcu\\\\0.fireflyluo-embedded-libs-main\\\\sensor\\\\sht40\\\\sht40_hal.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\stdint.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\vcruntime.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\sal.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\concurrencysal.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\vadefs.h\",\
            \"e:\\\\windows kits\\\\10\\\\include\\\\10.0.26100.0\\\\ucrt\\\\stdio.h\",\
            \"e:\\\\windows kits\\\\10\\\\include\\\\10.0.26100.0\\\\ucrt\\\\corecrt.h\",\
            \"e:\\\\windows kits\\\\10\\\\include\\\\10.0.26100.0\\\\ucrt\\\\corecrt_wstdio.h\",\
            \"e:\\\\windows kits\\\\10\\\\include\\\\10.0.26100.0\\\\ucrt\\\\corecrt_stdio_config.h\"\
        ]\
    }\
}"
}