@echo off
set OUTPUT_PATH=%1
set TOOLKIT=%2
set TOOLKIT_PATH=%3
set "OUTPUT_PATH=%OUTPUT_PATH:/=\%"

echo TOOLKIT=%TOOLKIT%
echo TOOLKIT_PATH=%TOOLKIT_PATH%

set ROOT=%OUTPUT_PATH%\..\..\..\..\..\..
set INDEX=0
set OUTPUT_IMAGE_PATH=%ROOT%\scripts\images
set SREC_CAT=%ROOT%\scripts\imgtool\srec_cat.exe
set OUTPUT_FILE=nspe

IF EXIST %OUTPUT_PATH%\..\nspe*     del %OUTPUT_PATH%\..\nspe*

if "%TOOLKIT%" == "KEIL" (
    :: Generate txt for debug
     %TOOLKIT_PATH%\ARM\ARMCC\bin\fromelf.exe --text -c -d --output=%OUTPUT_PATH%\..\%OUTPUT_FILE%.txt %OUTPUT_PATH%\%OUTPUT_FILE%.axf

    :: Generate binary image
     %TOOLKIT_PATH%\ARM\ARMCC\bin\fromelf.exe --bin --8x1 --bincombined --output=%OUTPUT_PATH%\..\nspe.bin %OUTPUT_PATH%\%OUTPUT_FILE%.axf
)
if "%TOOLKIT%" == "IAR" (
    :: Generate ASM file
    %TOOLKIT_PATH%\bin\ielfdumparm.exe %OUTPUT_PATH%\%OUTPUT_FILE%.axf  --output %OUTPUT_PATH%\..\%OUTPUT_FILE%.asm --code

    :: Generate binary image
    %TOOLKIT_PATH%\bin\ielftool.exe %OUTPUT_PATH%\%OUTPUT_FILE%.axf  --bin %OUTPUT_PATH%\..\%OUTPUT_FILE%.bin
)

if "%TOOLKIT%" == "GCC" (
    arm-none-eabi-objdump -xS  %OUTPUT_PATH%\%OUTPUT_FILE%.axf > %OUTPUT_PATH%\..\%OUTPUT_FILE%.txt
    arm-none-eabi-objcopy -Obinary %OUTPUT_PATH%\%OUTPUT_FILE%.axf %OUTPUT_PATH%\..\%OUTPUT_FILE%.bin
)

:: Copy nspe.bin to the image path as ota image
copy %OUTPUT_PATH%\..\nspe.bin  %OUTPUT_IMAGE_PATH%\image-ota.bin

:: concatenate mbl-ns.bin and nspe.bin
%SREC_CAT% %OUTPUT_IMAGE_PATH%\mbl-ns.bin -Binary -offset 0 %OUTPUT_PATH%\..\nspe.bin -Binary -offset 0xa000 -fill 0xFF 0x7FFC 0xA000 -o %OUTPUT_IMAGE_PATH%\image-all.bin -Binary 
:: Convert to HEX file
if exist %OUTPUT_IMAGE_PATH%\image-all.bin %SREC_CAT% %OUTPUT_IMAGE_PATH%\image-all.bin -Binary -offset 0x0C000000 -o %OUTPUT_IMAGE_PATH%\image-all.hex -Intel


:error
::exit
