@echo off
REM ========================================
REM  Ad-Hoc-lib Simulation Build & Run Script
REM  支持 MSVC 和 MinGW-GCC
REM ========================================
setlocal enabledelayedexpansion

set TEST_DIR=%~dp0
set LIB_DIR=%TEST_DIR%..\

set SRC_FILES=%TEST_DIR%main.c ^
    %TEST_DIR%channel_win32\adhoc_channel.c ^
    %TEST_DIR%channel_win32\adhoc_link_win32.c ^
    %TEST_DIR%channel_win32\adhoc_logger.c ^
    %LIB_DIR%src\adhoc_node.c ^
    %LIB_DIR%src\adhoc_sm.c ^
    %LIB_DIR%src\adhoc_data_plane.c ^
    %LIB_DIR%src\adhoc_frame.c ^
    %LIB_DIR%src\adhoc_crc8.c ^
    %LIB_DIR%src\adhoc_timing.c ^
    %LIB_DIR%src\adhoc_reply_list.c

set OUT_EXE=%TEST_DIR%adhoc_sim.exe

echo === Ad-Hoc-lib Simulation Build ===

REM --- try GCC first ---
where gcc >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [Compiler: MinGW-GCC]
    gcc -Wall -O2 -std=c11 -I"%LIB_DIR%include" -I"%TEST_DIR%" %SRC_FILES% -o "%OUT_EXE%" 2>&1
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] GCC build failed!
        pause
        exit /b 1
    )
    echo Build OK.
    goto :run
)

REM --- try MSVC ---
where cl >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [Compiler: MSVC]
    cl /nologo /W3 /O2 /MT /D_CRT_SECURE_NO_WARNINGS /utf-8 ^
       /I"%LIB_DIR%include" /I"%TEST_DIR%" ^
       %SRC_FILES% /Fe:"%OUT_EXE%" 2>&1
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] MSVC build failed! Run from VS Developer Command Prompt.
        pause
        exit /b 1
    )
    echo Build OK.
    goto :run
)

echo [ERROR] No compiler found (gcc or cl)! Install MinGW-w64 or Visual Studio.
pause
exit /b 1

:run
echo.
echo === Running simulation (45s) ===
echo.
cd /d "%TEST_DIR%"
"%OUT_EXE%"

echo.
echo === Done ===
echo Logs: %TEST_DIR%logs\
pause
