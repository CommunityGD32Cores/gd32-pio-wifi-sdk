@echo off

set OUTPUT_PATH=%1
set OUTPUT_NAME=%2
set TOOLKIT=%3
set TOOLKIT_PATH=%4
set "OUTPUT_PATH=%OUTPUT_PATH:/=\%"

set ROOT=%OUTPUT_PATH%\..\..\..\..\..
set SREC_CAT=%ROOT%\scripts\imgtool\srec_cat.exe
set OUTPUT_IMAGE_PATH=%ROOT%\scripts\images



if "%TOOLKIT%" == "KEIL" (
    :: Generate txt for debug
    %TOOLKIT_PATH%\ARM\ARMCC\bin\fromelf.exe --text -c -d --output=%OUTPUT_PATH%\..\%OUTPUT_NAME%.txt %OUTPUT_PATH%\%OUTPUT_NAME%.axf

    :: Generate binary image
    %TOOLKIT_PATH%\ARM\ARMCC\bin\fromelf.exe --bin --8x1  --bincombined --output=%OUTPUT_PATH%\..\%OUTPUT_NAME%.bin %OUTPUT_PATH%\%OUTPUT_NAME%.axf
)
if "%TOOLKIT%" == "IAR" (
    :: Generate ASM file
    %TOOLKIT_PATH%\bin\ielfdumparm.exe %OUTPUT_PATH%\%OUTPUT_NAME%.axf  --output %OUTPUT_PATH%\..\%OUTPUT_NAME%.asm --code

    :: Generate binary image
    %TOOLKIT_PATH%\bin\ielftool.exe %OUTPUT_PATH%\%OUTPUT_NAME%.axf  --bin %OUTPUT_PATH%\..\%OUTPUT_NAME%.bin
)
if "%TOOLKIT%" == "GCC" (
    arm-none-eabi-objdump -xS  %OUTPUT_PATH%\%OUTPUT_NAME%.axf > %OUTPUT_PATH%\..\%OUTPUT_NAME%.txt
    arm-none-eabi-objcopy -Obinary %OUTPUT_PATH%\%OUTPUT_NAME%.axf %OUTPUT_PATH%\..\%OUTPUT_NAME%.bin
)

IF EXIST %OUTPUT_IMAGE_PATH%\%OUTPUT_NAME%* del %OUTPUT_IMAGE_PATH%\%OUTPUT_NAME%*
:: Copy to output image path
copy %OUTPUT_PATH%\..\%OUTPUT_NAME%.bin %OUTPUT_IMAGE_PATH%\
:: Generate Hex file
%SREC_CAT%  %OUTPUT_IMAGE_PATH%\%OUTPUT_NAME%.bin -Binary -offset 0x0C000000 -o %OUTPUT_IMAGE_PATH%\%OUTPUT_NAME%.hex -Intel
 
::exit
