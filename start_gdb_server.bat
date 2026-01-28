@echo off
REM ============================================================
REM JLink GDB Server 启动脚本 for STM32F405
REM ============================================================

set JLINK_PATH="C:\Program Files\SEGGER\JLink_V878"
set DEVICE=STM32F405VG
set INTERFACE=SWD
set SPEED=4000
set PORT=2331

echo ============================================================
echo Starting JLink GDB Server
echo Device: %DEVICE%
echo Interface: %INTERFACE%
echo Speed: %SPEED% kHz
echo GDB Port: %PORT%
echo ============================================================

%JLINK_PATH%\JLinkGDBServerCL.exe -device %DEVICE% -if %INTERFACE% -speed %SPEED% -port %PORT% -LocalhostOnly 1 -noir

pause
