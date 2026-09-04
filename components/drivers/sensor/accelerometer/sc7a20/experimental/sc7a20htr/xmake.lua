-- SC7A20HTR driver build config

target("sc7a20htr")
    set_kind("static")
    set_basename("sc7a20htr")

    -- Source files
    add_files("sc7a20_core.c", "sc7a20_core_async.c")

    -- Optional platform file
    if os.exists("platform/platform.c") then
        add_files("platform/platform.c")
    end

    -- Headers
    add_headerfiles("inc/*.h")
    if os.exists("platform") then
        add_headerfiles("platform/*.h")
    end

    -- Include dirs
    add_includedirs(".", {public = true})
    add_includedirs("inc", {public = true})
    if os.exists("platform") then
        add_includedirs("platform", {public = true})
    end

    set_languages("c11")
    add_defines("SC7A20HTR_DRIVER_AVAILABLE", {public = true})

    -- Compile options
    if is_plat("mingw", "linux", "macosx") then
        add_cxflags("-Wall", "-Wextra", "-Werror")
    elseif is_plat("windows") then
        add_cxflags("/W4", "/utf-8")
    end
