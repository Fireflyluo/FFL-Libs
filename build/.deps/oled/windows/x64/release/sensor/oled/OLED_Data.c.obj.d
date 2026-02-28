{
    files = {
        [[sensor\oled\OLED_Data.c]]
    },
    depfiles_format = "cl_json",
    depfiles = "{\
    \"Version\": \"1.2\",\
    \"Data\": {\
        \"Source\": \"d:\\\\mcu\\\\0.fireflyluo-embedded-libs-main\\\\sensor\\\\oled\\\\oled_data.c\",\
        \"ProvidedModule\": \"\",\
        \"Includes\": [\
            \"d:\\\\mcu\\\\0.fireflyluo-embedded-libs-main\\\\sensor\\\\oled\\\\oled_data.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\stdint.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\vcruntime.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\sal.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\concurrencysal.h\",\
            \"c:\\\\program files\\\\microsoft visual studio\\\\18\\\\insiders\\\\vc\\\\tools\\\\msvc\\\\14.50.35717\\\\include\\\\vadefs.h\"\
        ]\
    }\
}",
    values = {
        [[C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Tools\MSVC\14.50.35717\bin\HostX64\x64\cl.exe]],
        {
            "-nologo",
            "-MD",
            "-std:c11",
            [[-Isensor\oled]],
            "-DOLED_DRIVER_AVAILABLE",
            "/W4",
            "/utf-8"
        }
    }
}