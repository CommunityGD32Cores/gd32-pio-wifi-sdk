@echo off

set DEBUGGER=%1

:: Download
set FILE=scripts\images\image-all.bin
set ADDR=0x08000000
if not exist %FILE% echo WARN: %FILE% not exist!

if "%DEBUGGER%" == "" (
    goto printhelp
)

if not "%DEBUGGER%"=="JLINK" if not "%DEBUGGER%"=="GDLINK" if not "%DEBUGGER%"=="DAPLINK" (
    goto printhelp
)

:: Download_by_JLINK
if "%DEBUGGER%" == "JLINK" (
    set JLINK_PATH="C:\Program Files (x86)\SEGGER\JLink\Jlink.exe"
    echo r > JLINK_SCRIPT
    echo exec SetCompareMode = 0 >> JLINK_SCRIPT
    echo loadfile %FILE% %ADDR% >> JLINK_SCRIPT
    echo qc >> JLINK_SCRIPT
    echo r >> JLINK_SCRIPT

    %JLINK_PATH% -device GD32W51x -if SWD -speed 4000 -autoconnect 1 -JTAGConf -1,-1 -CommanderScript JLINK_SCRIPT
    del JLINK_SCRIPT
    goto end
)

:: Download_by_GDLINK
if "%DEBUGGER%"=="GDLINK" set dbg=CMSIS-DAP
if "%DEBUGGER%"=="DAPLINK" set dbg=CMSIS-DAP
if "%dbg%"=="CMSIS-DAP" (
    set OpenOCD_PATH=GD32W51x_Addon\OpenOCD\bin\openocd.exe
    set GD32W51x_CFG_PATH=GD32W51x_Addon\OpenOCD\bin\openocd_gdlink_gd32w51x_ns.cfg
    set OpenOCD_CMD="program  ./scripts/images/image-all.bin %ADDR% verify reset exit;"
    %OpenOCD_PATH% -f "%GD32W51x_CFG_PATH%" -c %OpenOCD_CMD%
    goto end
)
:printhelp
    echo Please input which debugger used.
    echo Supported Debugger: GDLINK or DAPLINK or JLINK
:end